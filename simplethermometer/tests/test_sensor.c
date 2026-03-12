/**
 * @file test_sensor.c
 * @brief Unit Tests for Temperature Sensor
 *
 * Tests the sensor reading, filtering, and value ranges.
 *
 * Every line is commented.
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include <stdio.h>              /* For printf() */
#include "test_framework.h"     /* Include test framework macros */
#include "../firmware/sensor.h" /* Include sensor to test */

/* ============================================================================
   TEST 1: Sensor Initialization
   ============================================================================ */

/**
 * @brief Test that sensor_init() doesn't crash
 */
TEST(sensor_init_succeeds)
{
    /* Call sensor_init() function */
    sensor_init();

    /* If we reach here, initialization succeeded (no crash or error) */
    ASSERT_TRUE(1);
}

/* ============================================================================
   TEST 2: Sensor Returns Valid Temperature Range
   ============================================================================ */

/**
 * @brief Test that sensor readings are within expected range [15.0, 35.0]
 */
TEST(sensor_returns_valid_range)
{
    /* Initialize sensor before reading */
    sensor_init();

    /* Read temperature multiple times and verify each is in valid range */
    for (int i = 0; i < 10; i++) {
        /* Call sensor_read_temperature() to get current reading */
        float temp = sensor_read_temperature();

        /* Assert that temperature is >= 15.0°C (minimum valid temperature) */
        ASSERT_TRUE(temp >= 15.0f);

        /* Assert that temperature is <= 35.0°C (maximum valid temperature) */
        ASSERT_TRUE(temp <= 35.0f);
    }
}

/* ============================================================================
   TEST 3: Sensor Returns Float Value
   ============================================================================ */

/**
 * @brief Test that sensor returns reasonable float value (not NaN or infinity)
 */
TEST(sensor_returns_valid_float)
{
    /* Initialize sensor */
    sensor_init();

    /* Read a temperature value */
    float temp = sensor_read_temperature();

    /* Check that temperature is not infinity by seeing if it's less than 1000 */
    ASSERT_TRUE(temp < 1000.0f);

    /* Check that temperature is not negative infinity by seeing if it's greater than -1000 */
    ASSERT_TRUE(temp > -1000.0f);
}

/* ============================================================================
   TEST 4: Sensor Produces Variation (Not Constant Value)
   ============================================================================ */

/**
 * @brief Test that sensor readings vary over time (not stuck at one value)
 */
TEST(sensor_produces_variation)
{
    /* Initialize sensor */
    sensor_init();

    /* Read first temperature value */
    float temp1 = sensor_read_temperature();

    /* Read second temperature value (after some iterations) */
    float temp2 = temp1;

    /* Keep reading until we get a different value or max iterations */
    int max_iterations = 20;

    /* Loop up to 20 times */
    for (int i = 0; i < max_iterations; i++) {
        /* Read next temperature */
        temp2 = sensor_read_temperature();

        /* If this temperature is different from previous, break loop */
        if (temp2 != temp1) {
            break;
        }
    }

    /* Assert that we got at least some variation in readings */
    /* (due to noise and oscillation, values should differ) */
    ASSERT_TRUE(temp2 != temp1);
}

/* ============================================================================
   TEST 5: Temperature Changes Smoothly (First-Order Filter)
   ============================================================================ */

/**
 * @brief Test that temperature changes gradually, not in large jumps
 *
 * The first-order filter should prevent sudden large changes.
 * Max change per reading should be small (due to filter).
 */
TEST(sensor_changes_smoothly)
{
    /* Initialize sensor */
    sensor_init();

    /* Read first temperature */
    float prev_temp = sensor_read_temperature();

    /* Read next 10 temperatures */
    for (int i = 0; i < 10; i++) {
        /* Read current temperature */
        float curr_temp = sensor_read_temperature();

        /* Calculate absolute change from previous reading */
        float change = prev_temp > curr_temp ? prev_temp - curr_temp : curr_temp - prev_temp;

        /* Assert that change is less than 1.0°C per reading */
        /* (with low-pass filter, changes should be gradual) */
        ASSERT_TRUE(change < 1.0f);

        /* Update previous temperature for next iteration */
        prev_temp = curr_temp;
    }
}

/* ============================================================================
   MAIN TEST RUNNER
   ============================================================================ */

/**
 * @brief Main function that runs all sensor tests
 *
 * @return Exit code (0 if all pass, 1 if any fail)
 */
int main(void)
{
    /* Print test suite header */
    printf("\n");
    printf("==========================================\n");
    printf("SimpleThermometer - Sensor Tests\n");
    printf("==========================================\n");

    /* Run each test and print results */

    /* Test 1: Initialization succeeds */
    RUN_TEST(sensor_init_succeeds);

    /* Test 2: Returns values in valid range */
    RUN_TEST(sensor_returns_valid_range);

    /* Test 3: Returns valid float (not NaN/inf) */
    RUN_TEST(sensor_returns_valid_float);

    /* Test 4: Produces varying values */
    RUN_TEST(sensor_produces_variation);

    /* Test 5: Changes smoothly */
    RUN_TEST(sensor_changes_smoothly);

    /* Print summary and exit with appropriate code */
    TEST_SUMMARY();
}
