/**
 * @file test_serial_parser.c
 * @brief Unit Tests for Serial Parser
 *
 * Tests parsing of temperature messages in format "TEMP:XX.X°C"
 *
 * Every line is commented.
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include <stdio.h>              /* For printf() and sscanf() */
#include <string.h>             /* For strlen(), strcpy() */
#include "test_framework.h"     /* Include test framework macros */

/* ============================================================================
   HELPER FUNCTION: Parse Temperature String
   ============================================================================ */

/**
 * @brief Parse temperature from a string like "TEMP:22.3°C\r\n"
 *
 * This function demonstrates what the Python ground station does.
 *
 * @param str Input string (e.g., "TEMP:22.3°C")
 * @param temp Pointer to float where parsed temperature is stored
 * @return 1 if successful, 0 if parse failed
 */
int parse_temperature(const char *str, float *temp)
{
    /* Use sscanf to parse "TEMP:" prefix and decimal number */
    /* %f reads a floating-point number */
    /* The function returns number of successfully parsed items */
    int parsed = sscanf(str, "TEMP:%f", temp);

    /* Return 1 if we successfully parsed 1 item (the temperature) */
    return (parsed == 1) ? 1 : 0;
}

/* ============================================================================
   TEST 1: Parse Valid Temperature
   ============================================================================ */

/**
 * @brief Test parsing a valid temperature string
 */
TEST(parser_valid_temperature)
{
    /* Define a test string with valid temperature format */
    const char *input = "TEMP:22.3";

    /* Declare variable to hold parsed temperature */
    float temp = 0.0f;

    /* Call parse_temperature to extract temperature from string */
    int success = parse_temperature(input, &temp);

    /* Assert that parsing was successful (returned 1) */
    ASSERT_EQ(success, 1);

    /* Assert that parsed temperature matches expected value (22.3°C) */
    ASSERT_FLOAT_EQ(temp, 22.3f, 0.01f);
}

/* ============================================================================
   TEST 2: Parse Temperature Boundary Values
   ============================================================================ */

/**
 * @brief Test parsing min/max temperature values
 */
TEST(parser_boundary_values)
{
    /* Test minimum valid temperature (15°C) */

    /* Create string with minimum temperature */
    const char *input_min = "TEMP:15.0";

    /* Declare variable for parsed temperature */
    float temp = 0.0f;

    /* Parse minimum temperature string */
    int success = parse_temperature(input_min, &temp);

    /* Assert parsing succeeded */
    ASSERT_EQ(success, 1);

    /* Assert parsed value is 15.0 */
    ASSERT_FLOAT_EQ(temp, 15.0f, 0.01f);

    /* Test maximum valid temperature (35°C) */

    /* Create string with maximum temperature */
    const char *input_max = "TEMP:35.0";

    /* Parse maximum temperature string */
    success = parse_temperature(input_max, &temp);

    /* Assert parsing succeeded */
    ASSERT_EQ(success, 1);

    /* Assert parsed value is 35.0 */
    ASSERT_FLOAT_EQ(temp, 35.0f, 0.01f);
}

/* ============================================================================
   TEST 3: Parse Temperature with Various Decimal Places
   ============================================================================ */

/**
 * @brief Test parsing temperatures with different precision
 */
TEST(parser_different_precision)
{
    /* Test temperature with 1 decimal place */

    /* String with 1 decimal: "TEMP:20.5" */
    const char *input1 = "TEMP:20.5";

    /* Declare variable for temperature */
    float temp = 0.0f;

    /* Parse string */
    int success = parse_temperature(input1, &temp);

    /* Verify parsing succeeded and value is correct */
    ASSERT_EQ(success, 1);
    ASSERT_FLOAT_EQ(temp, 20.5f, 0.01f);

    /* Test temperature with 2 decimal places */

    /* String with 2 decimals: "TEMP:23.45" */
    const char *input2 = "TEMP:23.45";

    /* Parse string */
    success = parse_temperature(input2, &temp);

    /* Verify parsing succeeded and value is correct */
    ASSERT_EQ(success, 1);
    ASSERT_FLOAT_EQ(temp, 23.45f, 0.01f);

    /* Test temperature with no decimal places */

    /* String with no decimals: "TEMP:25" */
    const char *input3 = "TEMP:25";

    /* Parse string */
    success = parse_temperature(input3, &temp);

    /* Verify parsing succeeded and value is correct */
    ASSERT_EQ(success, 1);
    ASSERT_FLOAT_EQ(temp, 25.0f, 0.01f);
}

/* ============================================================================
   TEST 4: Reject Invalid Format
   ============================================================================ */

/**
 * @brief Test that invalid strings are rejected
 */
TEST(parser_rejects_invalid)
{
    /* Declare temperature variable */
    float temp = 0.0f;

    /* Test 1: String without "TEMP:" prefix */

    /* Input string: "22.3" (missing prefix) */
    const char *input1 = "22.3";

    /* Try to parse (should fail) */
    int success = parse_temperature(input1, &temp);

    /* Assert that parsing failed (returned 0) */
    ASSERT_EQ(success, 0);

    /* Test 2: String with wrong prefix */

    /* Input string: "HEAT:22.3" (wrong prefix, not "TEMP:") */
    const char *input2 = "HEAT:22.3";

    /* Try to parse (should fail) */
    success = parse_temperature(input2, &temp);

    /* Assert that parsing failed (returned 0) */
    ASSERT_EQ(success, 0);

    /* Test 3: String with no colon */

    /* Input string: "TEMP 22.3" (space instead of colon) */
    const char *input3 = "TEMP 22.3";

    /* Try to parse (should fail) */
    success = parse_temperature(input3, &temp);

    /* Assert that parsing failed (returned 0) */
    ASSERT_EQ(success, 0);
}

/* ============================================================================
   MAIN TEST RUNNER
   ============================================================================ */

/**
 * @brief Main function that runs all serial parser tests
 *
 * @return Exit code (0 if all pass, 1 if any fail)
 */
int main(void)
{
    /* Print test suite header */
    printf("\n");
    printf("==========================================\n");
    printf("SimpleThermometer - Serial Parser Tests\n");
    printf("==========================================\n");

    /* Run each test and print results */

    /* Test 1: Valid temperature parsing */
    RUN_TEST(parser_valid_temperature);

    /* Test 2: Boundary values (min/max) */
    RUN_TEST(parser_boundary_values);

    /* Test 3: Different decimal precision */
    RUN_TEST(parser_different_precision);

    /* Test 4: Invalid format rejection */
    RUN_TEST(parser_rejects_invalid);

    /* Print summary and exit with appropriate code */
    TEST_SUMMARY();
}
