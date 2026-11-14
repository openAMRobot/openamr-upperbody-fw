/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Minimal test helpers for the host unit tests (no external framework).
 */
#ifndef CTL017_TEST_UTIL_H
#define CTL017_TEST_UTIL_H

#include <stdio.h>
#include <stdlib.h>

static int g_failures;
static int g_checks;

#define CHECK(cond)                                                              \
    do {                                                                         \
        g_checks++;                                                              \
        if (!(cond)) {                                                           \
            g_failures++;                                                        \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                        \
    } while (0)

#define CHECK_EQ(a, b)                                                           \
    do {                                                                         \
        long long va_ = (long long)(a);                                          \
        long long vb_ = (long long)(b);                                          \
        g_checks++;                                                              \
        if (va_ != vb_) {                                                        \
            g_failures++;                                                        \
            fprintf(stderr, "%s:%d: CHECK_EQ failed: %s (%lld) != %s (%lld)\n",  \
                    __FILE__, __LINE__, #a, va_, #b, vb_);                       \
        }                                                                        \
    } while (0)

#define RUN(test)                                                                \
    do {                                                                         \
        int before_ = g_failures;                                                \
        test();                                                                  \
        printf("%s %s\n", (g_failures == before_) ? "PASS" : "FAIL", #test);     \
    } while (0)

#define TEST_MAIN_END()                                                          \
    do {                                                                         \
        printf("%d checks, %d failures\n", g_checks, g_failures);                \
        return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;                         \
    } while (0)

#endif
