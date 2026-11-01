/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j;

import static com.tngtech.archunit.core.domain.JavaClass.Predicates.resideInAnyPackage;
import static com.tngtech.archunit.core.domain.JavaModifier.PUBLIC;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.classes;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.noClasses;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.tngtech.archunit.core.importer.ImportOption;
import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;
import com.tngtech.archunit.lang.ArchRule;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * The layer rule (CONTRIBUTING.md, docs/API.md) as executable checks: a lower layer never imports a higher one,
 * implementations stay hidden, time and threads come from configuration, and no platform vocabulary leaks in.
 */
class ArchitectureTest {

    private static final String ROOT = "io.mavlink4j";

    private static final JavaClasses PRODUCTION = new ClassFileImporter()
            .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
            .importPackages(ROOT);

    /** One dynamic test per rule, so a surefire count shows every rule that ran. */
    @TestFactory
    Stream<DynamicTest> layerRules() {
        return Stream.of(
                DynamicTest.dynamicTest("transportImportsNothingAboveIt", () -> transportImportsNothingAboveIt.check(PRODUCTION)),
                DynamicTest.dynamicTest("codecImportsOnlyTransportAndConfig", () -> codecImportsOnlyTransportAndConfig.check(PRODUCTION)),
                DynamicTest.dynamicTest("sessionNeverImportsServices", () -> sessionNeverImportsServices.check(PRODUCTION)),
                DynamicTest.dynamicTest("configImportsNoLayer", () -> configImportsNoLayer.check(PRODUCTION)),
                DynamicTest.dynamicTest("rootTypesDependOnNoLayer", () -> rootTypesDependOnNoLayer.check(PRODUCTION)),
                DynamicTest.dynamicTest("defaultImplementationsAreNotPublic", () -> defaultImplementationsAreNotPublic.check(PRODUCTION)),
                DynamicTest.dynamicTest("noWallClockOutsideTheClockSeam", () -> noWallClockOutsideTheClockSeam.check(PRODUCTION)),
                DynamicTest.dynamicTest("noThreadCreationOutsideTheThreadSeam", () -> noThreadCreationOutsideTheThreadSeam.check(PRODUCTION)),
                DynamicTest.dynamicTest("noPlatformPackages", () -> noPlatformPackages.check(PRODUCTION)));
    }

    static final ArchRule transportImportsNothingAboveIt = noClasses()
            .that().resideInAPackage(ROOT + ".transport..")
            .should().dependOnClassesThat().resideInAnyPackage(
                    ROOT + ".codec..", ROOT + ".session..", ROOT + ".service..", ROOT + ".config..")
            .because("L1 is the bottom layer: bytes in, bytes out");

    static final ArchRule codecImportsOnlyTransportAndConfig = noClasses()
            .that().resideInAPackage(ROOT + ".codec..")
            .should().dependOnClassesThat().resideInAnyPackage(ROOT + ".session..", ROOT + ".service..")
            .because("L2 frames bytes; routing and request/response are above it");

    static final ArchRule sessionNeverImportsServices = noClasses()
            .that().resideInAPackage(ROOT + ".session..")
            .should().dependOnClassesThat().resideInAPackage(ROOT + ".service..")
            .because("L3 offers primitives that L4 services compose");

    static final ArchRule configImportsNoLayer = noClasses()
            .that().resideInAPackage(ROOT + ".config..")
            .should().dependOnClassesThat().resideInAnyPackage(
                    ROOT + ".transport..", ROOT + ".codec..", ROOT + ".session..", ROOT + ".service..")
            .because("settings are plain values every layer may read");

    static final ArchRule rootTypesDependOnNoLayer = noClasses()
            .that().resideInAPackage(ROOT)
            .should().dependOnClassesThat().resideInAnyPackage(
                    ROOT + ".transport..", ROOT + ".codec..", ROOT + ".session..", ROOT + ".service..", ROOT + ".config..")
            .because("L0 identity types sit under every layer");

    static final ArchRule defaultImplementationsAreNotPublic = classes()
            .that().haveSimpleNameStartingWith("Default")
            .should().notHaveModifier(PUBLIC)
            .because("implementations are reached through their interface, never named by a user");

    static final ArchRule noWallClockOutsideTheClockSeam = noClasses()
            .that().resideOutsideOfPackage(ROOT + ".config..")
            .should().callMethod(java.time.Instant.class, "now")
            .orShould().callMethod(java.time.Clock.class, "systemUTC")
            .orShould().callMethod(java.time.Clock.class, "systemDefaultZone")
            .because("time is injected as an InstantSource so tests can drive it");

    static final ArchRule noThreadCreationOutsideTheThreadSeam = noClasses()
            .that().doNotHaveFullyQualifiedName(ROOT + ".session.LibraryThreads")
            .and().resideOutsideOfPackage(ROOT + ".config..")
            .should().callConstructor(Thread.class, Runnable.class)
            .orShould().callConstructor(Thread.class, Runnable.class, String.class)
            .orShould().callConstructor(Thread.class)
            .orShould().callMethod(java.util.concurrent.Executors.class, "newCachedThreadPool")
            .orShould().callMethod(java.util.concurrent.Executors.class, "newFixedThreadPool", int.class)
            .orShould().callMethod(java.util.concurrent.Executors.class, "newSingleThreadExecutor")
            .orShould().callMethod(java.util.concurrent.Executors.class, "newScheduledThreadPool", int.class)
            .because("threads come from the configured ThreadFactory");

    static final ArchRule noPlatformPackages = noClasses()
            .should().dependOnClassesThat(resideInAnyPackage("com.drones..", "org.springframework.."))
            .because("the library knows no host application and no framework");

    /** The words of the host platform must not appear in any source or test file (string and comment included). */
    @Test
    void noPlatformWordsInSources() throws IOException {
        List<String> forbidden = List.of("com.drones", "com/drones");
        Path self = Path.of("src/test/java/io/mavlink4j/ArchitectureTest.java");
        try (Stream<Path> files = Files.walk(Path.of("src"))) {
            List<Path> javaFiles = files.filter(p -> p.toString().endsWith(".java")).toList();
            assertTrue(javaFiles.size() > 100, "scan must reach the sources, found " + javaFiles.size());
            for (Path file : javaFiles) {
                if (file.endsWith(self.getFileName())) {
                    continue;
                }
                String text = Files.readString(file);
                for (String word : forbidden) {
                    assertTrue(!text.contains(word), file + " contains platform word '" + word + "'");
                }
            }
        }
    }
}
