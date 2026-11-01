/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import java.util.Objects;
import java.util.concurrent.ThreadFactory;

/**
 * The one place the library turns a host-supplied {@link ThreadFactory} into a thread it owns: the
 * thread is named for its job (so a thread dump reads {@code mavlink-rx-<link>}, not
 * {@code Thread-17}) and marked daemon (a leaked library thread must never keep a host JVM alive).
 * The factory still chooses the kind — OS-backed or virtual.
 */
final class LibraryThreads {

    private LibraryThreads() {
    }

    static Thread create(ThreadFactory factory, String name, Runnable task) {
        Objects.requireNonNull(factory, "factory");
        Thread thread = factory.newThread(Objects.requireNonNull(task, "task"));
        thread.setName(Objects.requireNonNull(name, "name"));
        thread.setDaemon(true);
        return thread;
    }
}
