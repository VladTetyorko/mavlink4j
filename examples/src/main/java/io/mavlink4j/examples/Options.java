/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.examples;

import java.time.Duration;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * Parsed command line shared by the examples: {@code --fake}, {@code --bind host:port},
 * {@code --timeout seconds}, plus per-example {@code --name value} options (a bare {@code --name} is a flag).
 */
final class Options {

    static final String DEFAULT_BIND = "0.0.0.0:14550";
    static final int DEFAULT_TIMEOUT_SECONDS = 10;

    private final Map<String, String> values;

    private Options(Map<String, String> values) {
        this.values = values;
    }

    /**
     * @param args    the raw arguments
     * @param allowed the per-example option names (without dashes); {@code fake}, {@code bind} and
     *                {@code timeout} are always allowed
     * @return the parsed options
     * @throws IllegalArgumentException on an unknown or malformed option
     */
    static Options parse(String[] args, String... allowed) {
        Set<String> known = new java.util.HashSet<>(Set.of("fake", "bind", "timeout"));
        known.addAll(Set.of(allowed));
        Map<String, String> values = new HashMap<>();
        for (int i = 0; i < args.length; i++) {
            String arg = args[i];
            if (!arg.startsWith("--") || !known.contains(arg.substring(2))) {
                throw new IllegalArgumentException("unknown argument: " + arg);
            }
            String name = arg.substring(2);
            boolean hasValue = i + 1 < args.length && !args[i + 1].startsWith("--");
            values.put(name, hasValue ? args[++i] : "true");
        }
        return new Options(values);
    }

    boolean flag(String name) {
        return values.containsKey(name);
    }

    boolean fake() {
        return flag("fake");
    }

    String get(String name, String fallback) {
        return values.getOrDefault(name, fallback);
    }

    int bindPort() {
        String bind = get("bind", DEFAULT_BIND);
        return Integer.parseInt(bind.substring(bind.lastIndexOf(':') + 1));
    }

    String bindHost() {
        String bind = get("bind", DEFAULT_BIND);
        return bind.substring(0, bind.lastIndexOf(':'));
    }

    Duration timeout() {
        return Duration.ofSeconds(Long.parseLong(get("timeout", Integer.toString(DEFAULT_TIMEOUT_SECONDS))));
    }
}
