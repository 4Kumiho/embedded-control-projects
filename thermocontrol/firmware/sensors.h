/**
 * @file sensors.h
 * @brief Temperature sensor abstraction and simulation
 *
 * This module handles:
 * - Simulated temperature sensor that mimics real hardware
 * - Gaussian noise generation (realistic sensor jitter)
 * - Ambient temperature baseline
 * - Heating/cooling response to external command
 *
 * Why simulation instead of real hardware?
 * - This is a PC-based training project
 * - Easier to test and reproduce results
 * - Can simulate sensor faults and noise
 * - Perfect learning tool for embedded firmware patterns
 *
 * Real embedded sensors would read via:
 * - I2C (BMP280 temperature sensor)
 * - SPI (thermistor with ADC)
 * - Analog input (thermistor circuit)
 * But the firmware interface is identical!
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>  /* uint32_t, float */

/**
 * @defgroup Sensors Sensor Module
 * @{
 */

/* ============================================================================
   CONFIGURATION CONSTANTS
   ============================================================================

   These are the "sensor parameters" that define behavior.
   In a real system, you'd tune these based on your actual hardware.
 */

/**
 * @name Sensor Physical Constants
 * @{
 */

/** Ambient room temperature (°C) - baseline without any heating */
#define SENSOR_AMBIENT_TEMP       20.0f

/** Maximum heater output (+100% = max heating) in °C/second per unit command */
#define SENSOR_HEATER_RATE        0.05f

/** Maximum cooler output (-100% = max cooling) in °C/second per unit command */
#define SENSOR_COOLER_RATE        0.03f

/** Thermal time constant (seconds) - how fast room responds to heating.
    Higher = slower response. τ = 30 sec means:
    - After 30 seconds of constant heating, temperature reaches ~63% of final value
    - After 3τ = 90 seconds, reaches ~95% of final value
    See: https://en.wikipedia.org/wiki/Time_constant_(exponential_response)
 */
#define SENSOR_TIME_CONSTANT      30.0f

/** Standard deviation of Gaussian noise (°C).
    Real sensors have 0.01-0.1°C noise. We use 0.05°C as realistic.
    σ = 0.05 means 68% of readings are within ±0.05°C of true value.
 */
#define SENSOR_NOISE_SIGMA        0.05f

/** @} */

/* ============================================================================
   DATA TYPES
   ============================================================================ */

/**
 * @struct sensor_state_t
 * @brief Encapsulates all sensor simulation state
 *
 * This is kept opaque (hidden) from the user - they shouldn't modify
 * these values directly. Instead, use the provided functions:
 * - sensor_init()
 * - sensor_read()
 * - sensor_set_command()
 *
 * Internal fields (don't touch!):
 * - current_temp: Simulated room temperature (°C)
 * - last_update_time: When we last updated the simulation (ms)
 * - heater_command: Current heating/cooling command (0-100%, can be negative)
 */
typedef struct {
    float current_temp;       ///< Current temperature in simulation (°C)
    uint32_t last_update_time; ///< Last time sensor was read (ms timestamp)
    float heater_command;     ///< Heater command from firmware (-100 to +100)
} sensor_state_t;

/**
 * @struct sensor_reading_t
 * @brief Single temperature sensor reading
 *
 * What you get back from sensor_read().
 * Contains the measured temperature with simulated noise.
 */
typedef struct {
    float temperature_celsius;  ///< Temperature value in °C (with noise)
    uint32_t timestamp_ms;      ///< When this reading was taken (ms)
} sensor_reading_t;

/* ============================================================================
   FUNCTION DECLARATIONS
   ============================================================================ */

/**
 * @brief Initialize the temperature sensor simulation
 *
 * Must be called once at program startup.
 * Sets initial temperature to ambient and initializes random seed.
 *
 * @param sensor Pointer to sensor_state_t structure to initialize
 * @return 0 on success, -1 on failure
 *
 * @example
 *   sensor_state_t my_sensor;
 *   if (sensor_init(&my_sensor) != 0) {
 *       printf("Sensor init failed\n");
 *   }
 */
int sensor_init(sensor_state_t *sensor);

/**
 * @brief Read the current temperature with simulated noise
 *
 * What happens:
 * 1. Updates internal temperature based on time elapsed and heater command
 * 2. Adds realistic Gaussian noise (normal distribution)
 * 3. Returns the noisy reading
 *
 * This mimics what a real sensor would do:
 * - Physically respond to heating (exponential response)
 * - Have measurement noise (every sensor has jitter)
 *
 * Thermal model used:
 *   dT/dt = -1/τ * (T - T_ambient) + (heater_command / 100) * heating_rate
 *
 * Where:
 * - τ = SENSOR_TIME_CONSTANT (30 sec)
 * - Negative term: room cools back to ambient when not heating
 * - Positive term: heating pushes temperature up
 *
 * @param sensor Pointer to sensor_state_t (must have been initialized with sensor_init)
 * @param current_time_ms Current system time in milliseconds (from platform.h)
 * @param reading Output parameter - will be filled with sensor reading
 * @return 0 on success, -1 on failure
 *
 * @example
 *   platform_timer_t timer;
 *   platform_timer_start(&timer);
 *
 *   for (int i = 0; i < 100; i++) {
 *       uint32_t now = platform_timer_elapsed_ms(&timer);
 *       sensor_reading_t reading;
 *       sensor_read(&my_sensor, now, &reading);
 *       printf("T = %.2f°C\n", reading.temperature_celsius);
 *       platform_sleep_ms(100);
 *   }
 */
int sensor_read(sensor_state_t *sensor, uint32_t current_time_ms,
                sensor_reading_t *reading);

/**
 * @brief Set the heater/cooler command for the sensor simulation
 *
 * The firmware gives a control signal (from the PID controller),
 * and this function applies it to the thermal simulation.
 *
 * Range of command:
 * - +100: Full heating (room warms at SENSOR_HEATER_RATE)
 * - 0: No heating/cooling (room drifts to ambient)
 * - -100: Full cooling (room cools at SENSOR_COOLER_RATE)
 *
 * Example behavior:
 *   - At T=20°C with command=+100, temperature rises ~1-2°C per second
 *   - Once T reaches setpoint, command→0, temperature stabilizes
 *   - Due to thermal time constant, approach is exponential (not linear)
 *
 * @param sensor Pointer to sensor_state_t
 * @param command Heater command: -100 (cool) to +100 (heat)
 *
 * @example
 *   // User wants room at 25°C, PID calculates command=+50
 *   sensor_set_command(&my_sensor, 50);
 *   // Next sensor_read() will show temperature increasing
 */
void sensor_set_command(sensor_state_t *sensor, float command);

/**
 * @brief Manually set the current temperature (for testing/reset)
 *
 * Useful for:
 * - Resetting sensor to known state (e.g., ambient temp)
 * - Testing edge cases (e.g., what if sensor reads 50°C?)
 * - Simulating sensor faults
 *
 * Normally you wouldn't call this; sensor temperature changes naturally
 * through heating. But for unit tests it's invaluable.
 *
 * @param sensor Pointer to sensor_state_t
 * @param temp_celsius New temperature value in °C
 *
 * @example
 *   // Test: what happens if sensor suddenly reads freezing temperature?
 *   sensor_set_temp(&my_sensor, 0.0f);
 *   sensor_reading_t reading;
 *   sensor_read(&my_sensor, 0, &reading);
 *   // Firmware should respond with maximum heating
 */
void sensor_set_temp(sensor_state_t *sensor, float temp_celsius);

/**
 * @brief Get the true (non-noisy) temperature
 *
 * Returns the underlying "perfect" temperature without noise.
 * Useful for debugging - compare true value with noisy sensor reading
 * to see how much noise was added.
 *
 * @param sensor Pointer to sensor_state_t
 * @return Current true temperature in °C (float)
 *
 * @example
 *   sensor_reading_t reading;
 *   sensor_read(&my_sensor, now, &reading);
 *   float true_temp = sensor_get_true_temp(&my_sensor);
 *   float noise = reading.temperature_celsius - true_temp;
 *   printf("True: %.2f, Measured: %.2f, Noise: %.3f°C\n",
 *          true_temp, reading.temperature_celsius, noise);
 */
float sensor_get_true_temp(const sensor_state_t *sensor);

/**
 * @} End of Sensors group
 */

#endif /* SENSORS_H */
