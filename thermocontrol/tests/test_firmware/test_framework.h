/**
 * @file test_framework.h
 * @brief Minimal Test Framework for C (No External Dependencies)
 *
 * Simple unit test framework for embedded C.
 * No dependencies - just standard C library.
 *
 * Design philosophy:
 *   - Zero external dependencies (easy to run anywhere)
 *   - Simple assert macros
 *   - Output plain text (easy to parse)
 *   - Fast feedback (fail fast)
 *
 * Usage:
 *   TEST(function_name) {
 *       ASSERT_EQ(actual, expected);
 *       ASSERT_TRUE(condition);
 *       ASSERT_FLOAT_EQ(a, b, tolerance);
 *   }
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* ============================================================================
   GLOBAL STATE
   ============================================================================ */

static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;

/* ============================================================================
   ASSERT MACROS
   ============================================================================ */

/**
 * Assert that a condition is true.
 * If false, print error message and increment fail counter.
 */
#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "  ✗ ASSERT_TRUE failed: %s (line %d)\n", #condition, __LINE__); \
            g_tests_failed++; \
        } \
    } while(0)

/**
 * Assert that a condition is false.
 */
#define ASSERT_FALSE(condition) \
    do { \
        if ((condition)) { \
            fprintf(stderr, "  ✗ ASSERT_FALSE failed: %s (line %d)\n", #condition, __LINE__); \
            g_tests_failed++; \
        } \
    } while(0)

/**
 * Assert that two integers are equal.
 */
#define ASSERT_EQ(actual, expected) \
    do { \
        if ((actual) != (expected)) { \
            fprintf(stderr, "  ✗ ASSERT_EQ failed: %d != %d (line %d)\n", (int)(actual), (int)(expected), __LINE__); \
            g_tests_failed++; \
        } \
    } while(0)

/**
 * Assert that two floats are approximately equal (within tolerance).
 */
#define ASSERT_FLOAT_EQ(actual, expected, tolerance) \
    do { \
        float diff = fabs((actual) - (expected)); \
        if (diff > (tolerance)) { \
            fprintf(stderr, "  ✗ ASSERT_FLOAT_EQ failed: %.6f != %.6f (diff: %.6f, tol: %.6f, line %d)\n", \
                    (float)(actual), (float)(expected), diff, (float)(tolerance), __LINE__); \
            g_tests_failed++; \
        } \
    } while(0)

/**
 * Assert that two strings are equal.
 */
#define ASSERT_STR_EQ(actual, expected) \
    do { \
        if (strcmp((actual), (expected)) != 0) { \
            fprintf(stderr, "  ✗ ASSERT_STR_EQ failed: \"%s\" != \"%s\" (line %d)\n", (actual), (expected), __LINE__); \
            g_tests_failed++; \
        } \
    } while(0)

/* ============================================================================
   TEST MACRO
   ============================================================================ */

/**
 * Define a test function.
 *
 * Usage:
 *   TEST(my_test_name) {
 *       ASSERT_EQ(2 + 2, 4);
 *       ASSERT_TRUE(condition);
 *   }
 *
 * This creates a function test_my_test_name() and calls it from main().
 */
#define TEST(test_name) \
    void test_##test_name(void); \
    void test_##test_name(void)

/**
 * Run a test and print result.
 */
#define RUN_TEST(test_name) \
    do { \
        int failed_before = g_tests_failed; \
        printf("▶ %s\n", #test_name); \
        test_##test_name(); \
        g_tests_run++; \
        if (g_tests_failed == failed_before) { \
            g_tests_passed++; \
            printf("  ✓ PASS\n"); \
        } \
    } while(0)

/* ============================================================================
   RESULT SUMMARY
   ============================================================================ */

/**
 * Print test summary and exit with appropriate code.
 * Exit code: 0 if all pass, 1 if any fail.
 */
#define TEST_SUMMARY() \
    do { \
        printf("\n================================\n"); \
        printf("Test Results:\n"); \
        printf("  Passed: %d / %d\n", g_tests_passed, g_tests_run); \
        printf("  Failed: %d / %d\n", g_tests_failed, g_tests_run); \
        printf("================================\n"); \
        return (g_tests_failed > 0) ? 1 : 0; \
    } while(0)

#endif /* TEST_FRAMEWORK_H */
