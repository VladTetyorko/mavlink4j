/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.examples;

import java.io.PrintStream;

/** Turns an example body into an exit code: 0 success, 1 failure, 2 bad arguments. */
final class Runner {

    /** An example body; returns the exit code. */
    @FunctionalInterface
    interface Body {
        int run() throws Exception;
    }

    private Runner() {
    }

    static int execute(PrintStream out, Body body) {
        try {
            return body.run();
        } catch (IllegalArgumentException e) {
            out.println("error: " + e.getMessage());
            return 2;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            out.println("error: interrupted");
            return 1;
        } catch (Exception | AssertionError e) {
            out.println("error: " + e);
            return 1;
        }
    }
}
