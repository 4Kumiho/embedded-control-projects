/**
 * @file test_framework.h
 * @brief Minimal Test Framework for SimpleThermometer
 *
 * Ultra-simple test framework with minimal assertions.
 * No external dependencies - just standard C library.
 *
 * Usage:
 *   TEST(my_test) {
 *       ASSERT_EQ(actual, expected);
 *       ASSERT_TRUE(condition);
 *       ASSERT_FLOAT_EQ(a, b, tolerance);
 *   }
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef TEST_FRAMEWORK_H_INCLUDED
#define TEST_FRAMEWORK_H_INCLUDED

#include <stdio.h>    /* For fprintf() */
#include <stdlib.h>   /* For exit() */
#include <math.h>     /* For fabs() */

/* ============================================================================
   GLOBAL TEST COUNTERS
   ============================================================================ */

/**
 * Global counter: total number of tests run
 */
static int g_tests_run = 0;

/**
 * Global counter: number of passed tests
 */
static int g_tests_passed = 0;

/**
 * Global counter: number of failed tests
 */
static int g_tests_failed = 0;

/* ============================================================================
   ASSERT MACROS
   ============================================================================ */

/**
 * @brief Assert that a condition is true
 *
 * If condition is false, prints error and increments fail counter.
 * Macros use do-while(0) pattern for proper statement semantics.
 */
#define ASSERT_TRUE(condition) \
    do { \
        /* Check if condition is false */ \
        if (!(condition)) { \
            /* Print error message with condition text and line number */ \
            fprintf(stderr, "  ✗ ASSERT_TRUE failed: %s (line %d)\n", \
                    #condition, __LINE__); \
            /* Increment global fail counter */ \
            g_tests_failed++; \
        } \
    } while(0)

/**
 * @brief Assert that two integers are equal
 */
#define ASSERT_EQ(actual, expected) \
    do { \
        /* Check if actual value does not equal expected */ \
        if ((actual) != (expected)) { \
            /* Print error with both values and line number */ \
            fprintf(stderr, "  ✗ ASSERT_EQ failed: %d != %d (line %d)\n", \
                    (int)(actual), (int)(expected), __LINE__); \
            /* Increment global fail counter */ \
            g_tests_failed++; \
        } \
    } while(0)

/**
 * @brief Assert that two floats are approximately equal (within tolerance)
 */
#define ASSERT_FLOAT_EQ(actual, expected, tolerance) \
    do { \
        /* Calculate absolute difference between values */ \
        float diff = fabs((actual) - (expected)); \
        /* Check if difference exceeds tolerance */ \
        if (diff > (tolerance)) { \
            /* Print error with both values, difference, tolerance, and line number */ \
            fprintf(stderr, "  ✗ ASSERT_FLOAT_EQ failed: %.6f != %.6f " \
                    "(diff: %.6f, tol: %.6f, line %d)\n", \
                    (float)(actual), (float)(expected), diff, \
                    (float)(tolerance), __LINE__); \
            /* Increment global fail counter */ \
            g_tests_failed++; \
        } \
    } while(0)

/* ============================================================================
   TEST MACRO
   ============================================================================ */

/**
 * @brief Define a test function
 *
 * Usage:
 *   TEST(my_test) {
 *       ASSERT_EQ(2 + 2, 4);
 *       ASSERT_TRUE(x > 0);
 *   }
 *
 * This creates a function test_my_test() that can be called from main.
 */
#define TEST(test_name) \
    /* Forward declaration of test function */ \
    void test_##test_name(void); \
    /* Function definition with body follows */ \
    void test_##test_name(void)

/**
 * @brief Run a single test and print result
 *
 * Usage in main():
 *   RUN_TEST(my_test);
 */
#define RUN_TEST(test_name) \
    do { \
        /* Record number of failures before test */ \
        int failed_before = g_tests_failed; \
        /* Print test name */ \
        printf("▶ %s\n", #test_name); \
        /* Call the test function */ \
        test_##test_name(); \
        /* Increment total test counter */ \
        g_tests_run++; \
        /* Check if test passed (no new failures) */ \
        if (g_tests_failed == failed_before) { \
            /* Increment passed counter */ \
            g_tests_passed++; \
            /* Print success message */ \
            printf("  ✓ PASS\n"); \
        } \
    } while(0)

/* ============================================================================
   RESULT SUMMARY
   ============================================================================ */

/**
 * @brief Print test summary and return exit code
 *
 * Should be called at end of main() to print results and exit.
 * Exit code is 0 if all tests pass, 1 if any fail.
 *
 * Usage:
 *   TEST_SUMMARY();
 */
#define TEST_SUMMARY() \
    do { \
        /* Print separator line */ \
        printf("\n================================\n"); \
        /* Print test results header */ \
        printf("Test Results:\n"); \
        /* Print number of passed tests */ \
        printf("  Passed: %d / %d\n", g_tests_passed, g_tests_run); \
        /* Print number of failed tests */ \
        printf("  Failed: %d / %d\n", g_tests_failed, g_tests_run); \
        /* Print separator line */ \
        printf("================================\n"); \
        /* Return 0 if no failures, 1 if any failures */ \
        return (g_tests_failed > 0) ? 1 : 0; \
    } while(0)

#endif /* TEST_FRAMEWORK_H_INCLUDED */
