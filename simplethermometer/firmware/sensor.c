/**
 * @file sensor.c
 * @brief Temperature Sensor Simulation Implementation
 *
 * Simple simulated sensor that generates realistic temperature readings
 * with a basic first-order response and noise.
 *
 * Every line of code has a comment explaining what it does.
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include "sensor.h"           /* Include sensor.h header file */
#include <stdlib.h>           /* Include standard library (for rand()) */
#include <math.h>             /* Include math library (for sin, cos functions) */
#include <time.h>             /* Include time library (for seeding rand) */

/* ============================================================================
   GLOBAL STATE VARIABLES
   ============================================================================ */

/**
 * Static variable to hold the previous temperature value
 * Used to simulate first-order sensor response (slow change)
 * Initialized to 20.0°C (ambient temperature)
 */
static float g_temperature_previous = 20.0f;

/**
 * Static variable to track elapsed time for generating waveform
 * Incremented each time sensor_read_temperature() is called
 * Used to create realistic temperature oscillation
 */
static uint32_t g_time_counter = 0;

/* ============================================================================
   SENSOR INITIALIZATION
   ============================================================================ */

/**
 * @brief Initialize sensor by seeding random number generator
 *
 * This function is called once at firmware startup.
 * It initializes the random number generator so we get different values each run.
 */
void sensor_init(void)
{
    /* Get current time as seed for random number generator */
    unsigned int seed = (unsigned int)time(NULL);

    /* Seed the standard C random number generator with current time */
    srand(seed);
}

/* ============================================================================
   SENSOR READING
   ============================================================================ */

/**
 * @brief Read temperature from simulated sensor
 *
 * This function simulates a real temperature sensor by:
 * 1. Generating a base temperature (23°C + sine wave for variation)
 * 2. Adding random noise (±0.5°C)
 * 3. Applying first-order filter (slow response)
 *
 * @return Current temperature in °C (float, 15.0 to 35.0 range)
 */
float sensor_read_temperature(void)
{
    /* --- Step 1: Generate base temperature with oscillation --- */

    /* Increment time counter (represents each 500ms reading) */
    g_time_counter++;

    /* Convert counter to angle in radians for sine wave (complete cycle ~120 readings) */
    float angle = (2.0f * 3.14159f * (float)g_time_counter) / 120.0f;

    /* Calculate base temperature: 25°C + 5°C * sine wave (osculates ±5°C) */
    float base_temperature = 25.0f + 5.0f * sin(angle);

    /* --- Step 2: Add random noise to simulate sensor imperfection --- */

    /* Generate random number between 0 and 1 */
    float random_fraction = (float)rand() / (float)RAND_MAX;

    /* Scale random number to ±0.5°C noise (range: -0.5 to +0.5) */
    float noise = (random_fraction - 0.5f) * 1.0f;

    /* Add noise to base temperature */
    float temperature_with_noise = base_temperature + noise;

    /* --- Step 3: Apply first-order filter for realistic slow response --- */

    /* Define filter coefficient (0.1 means 10% new value, 90% old value) */
    float alpha = 0.1f;

    /* Low-pass filter: new_value = α*current + (1-α)*previous */
    /* This makes temperature change slowly, like a real sensor */
    float filtered_temperature = alpha * temperature_with_noise +
                                (1.0f - alpha) * g_temperature_previous;

    /* Save this temperature for next iteration (needed for filter) */
    g_temperature_previous = filtered_temperature;

    /* --- Step 4: Clamp temperature to valid range [15, 35]°C --- */

    /* If temperature is below 15°C, set it to 15°C */
    if (filtered_temperature < 15.0f) {
        filtered_temperature = 15.0f;
    }

    /* If temperature is above 35°C, set it to 35°C */
    if (filtered_temperature > 35.0f) {
        filtered_temperature = 35.0f;
    }

    /* Return the final temperature value */
    return filtered_temperature;
}
