/**
 * @file sensors.c
 * @brief Temperature sensor simulation implementation
 *
 * Implements:
 * - Thermal model (exponential response to heating)
 * - Gaussian noise generation (Box-Muller transform)
 * - State management
 *
 * Physics background:
 * ==================
 * Real temperature change in a room follows:
 *   dT/dt = Q/C - h·A·(T - T_ambient)
 *
 * Where:
 * - Q = heat input (heater power)
 * - C = heat capacity of room
 * - h·A = surface area × heat transfer coefficient
 *
 * Simplified form (normalized):
 *   dT/dt = -1/τ · (T - T_ambient) + (U/100) · K
 *
 * Where:
 * - τ = thermal time constant (seconds)
 * - U = heater command (0-100)
 * - K = heating coefficient (°C/sec)
 *
 * Solution (integrating):
 *   T(t) = T_ambient + (T_0 - T_ambient)·exp(-t/τ) + ∫heating effect
 *
 * We use simple Euler integration (good enough for simulation):
 *   T_new = T_old + dT/dt · Δt
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include "sensors.h"
#include "platform.h"
#include <stdlib.h>      /* rand(), srand() */
#include <math.h>        /* exp(), sqrt(), cos(), M_PI */
#include <string.h>      /* memset() */

/* ============================================================================
   RANDOM NUMBER GENERATION
   ============================================================================

   For realistic sensor noise, we need Gaussian (normal) distribution,
   not uniform random numbers.

   Algorithm: Box-Muller transform
   - Takes 2 uniform random numbers
   - Produces 2 Gaussian random numbers
   - Formula: Z = sqrt(-2 * ln(U1)) * cos(2π * U2)
 */

/**
 * Generate a Gaussian random number with mean=0, stddev=1
 *
 * Box-Muller transform:
 * 1. Generate U1, U2 ~ Uniform(0,1)
 * 2. Compute R = sqrt(-2 * ln(U1))
 * 3. Compute θ = 2π * U2
 * 4. Return Z = R * cos(θ)
 *
 * This gives a standard normal distribution N(0, 1).
 * To scale to N(μ, σ), use: value = μ + σ * Z
 *
 * @return Gaussian random number, N(0, 1)
 */
static float gaussian_random(void)
{
    /* Generate two uniform random numbers in range (0, 1) */
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;

    /* Avoid log(0) - if u1 is exactly 0, clamp to small positive value */
    if (u1 < 0.0001f) u1 = 0.0001f;

    /* Box-Muller transform */
    float magnitude = sqrtf(-2.0f * logf(u1));
    float angle = 2.0f * 3.14159265f * u2;
    float gaussian = magnitude * cosf(angle);

    return gaussian;
}

/* ============================================================================
   SENSOR STATE MANAGEMENT
   ============================================================================ */

/**
 * int sensor_init(sensor_state_t *sensor)
 *
 * Initialize sensor to ambient temperature and prepare simulation.
 *
 * Steps:
 * 1. Validate input (null pointer check)
 * 2. Initialize all fields to default values
 * 3. Seed random number generator for reproducible results
 * 4. Set starting time to zero
 *
 * Why initialize to ambient?
 * - Room starts at room temperature
 * - Heating will increase from here
 * - Realistic initial condition
 */
int sensor_init(sensor_state_t *sensor)
{
    /* Defensive programming: check for null pointer */
    if (!sensor) {
        return -1;
    }

    /* Zero-initialize the entire structure (good practice) */
    memset(sensor, 0, sizeof(sensor_state_t));

    /* Set starting temperature to ambient */
    sensor->current_temp = SENSOR_AMBIENT_TEMP;

    /* Initialize timestamp */
    sensor->last_update_time = 0;

    /* No heater command initially */
    sensor->heater_command = 0.0f;

    /* Seed random number generator.
       srand() uses this to initialize the PRNG.
       time(NULL) gives a different seed each run (non-deterministic).
       For debugging, you might use srand(42) for reproducible randomness.
    */
    srand((unsigned int)time(NULL));

    return 0;
}

/**
 * int sensor_read(sensor_state_t *sensor, uint32_t current_time_ms,
 *                 sensor_reading_t *reading)
 *
 * Main sensor reading function. Does three things:
 *
 * 1. UPDATE thermal model:
 *    - Calculate time elapsed since last read: Δt = current_time - last_time
 *    - Apply thermal dynamics: dT = (-1/τ * (T - T_amb) + K * U/100) * Δt
 *    - Update temperature: T_new = T + dT
 *    - Remember current time for next read
 *
 * 2. ADD NOISE:
 *    - Generate Gaussian random value: noise ~ N(0, σ)
 *    - Add to temperature: T_noisy = T + noise
 *
 * 3. RETURN reading:
 *    - Pack temperature and timestamp into output structure
 *
 * Thermal dynamics explained:
 *   dT/dt = -1/τ · (T - T_ambient) + (U/100) · K
 *
 * First term: -1/τ · (T - T_ambient)
 *   - If room is hot (T > T_ambient), this term is negative (cools room)
 *   - If room is cold (T < T_ambient), this term is positive (warms room)
 *   - Strength depends on τ: large τ means slow response
 *
 * Second term: (U/100) · K
 *   - U is heater command (0 to 100)
 *   - K is heating rate (°C/sec)
 *   - Positive U adds heat; negative U removes heat
 *
 * Equilibrium (when dT/dt = 0):
 *   0 = -1/τ · (T_eq - T_amb) + (U/100) · K
 *   T_eq = T_amb + (U/100) · K · τ
 *
 * Example: U=100, τ=30, K=0.05, T_amb=20
 *   T_eq = 20 + (100/100) * 0.05 * 30 = 20 + 1.5 = 21.5°C
 *   So constant +100% heating reaches ~21.5°C
 *
 * @param sensor Pointer to initialized sensor_state_t
 * @param current_time_ms Current system time in milliseconds
 * @param reading Output: will be filled with sensor reading
 * @return 0 on success, -1 on error
 */
int sensor_read(sensor_state_t *sensor, uint32_t current_time_ms,
                sensor_reading_t *reading)
{
    /* Input validation */
    if (!sensor || !reading) {
        return -1;
    }

    /* ========== STEP 1: UPDATE THERMAL MODEL ========== */

    /* Calculate elapsed time since last update.
       First call: elapsed_ms will be 0 (no change from start state)
       Subsequent calls: elapsed_ms is time since last sensor_read()
    */
    uint32_t elapsed_ms = current_time_ms - sensor->last_update_time;
    float elapsed_sec = (float)elapsed_ms / 1000.0f;

    /* Remember current time for next update */
    sensor->last_update_time = current_time_ms;

    /* Calculate temperature change: dT/dt = ... */

    /* Term 1: Exponential cooling/warming to ambient
       -1/τ · (T - T_ambient)
       This brings room back to ambient temperature.
    */
    float delta_t_ambient = -(sensor->current_temp - SENSOR_AMBIENT_TEMP) / SENSOR_TIME_CONSTANT;

    /* Term 2: Heating effect from heater command
       If command > 0: heating (positive contribution)
       If command < 0: cooling (negative contribution)
    */
    float heating_rate = (sensor->heater_command >= 0.0f) ?
                         SENSOR_HEATER_RATE : SENSOR_COOLER_RATE;
    float delta_t_heater = (sensor->heater_command / 100.0f) * heating_rate;

    /* Total rate of change: dT/dt = ... */
    float dT_dt = delta_t_ambient + delta_t_heater;

    /* Apply Euler integration: T_new = T_old + dT/dt * Δt
       Euler is simple and adequate for this low-frequency update (100 ms).
       More precise methods (RK4) would be overkill here.
    */
    sensor->current_temp += dT_dt * elapsed_sec;

    /* Clamp temperature to realistic range (optional safety measure)
       Room can't get below absolute zero or above melting point...
       In practice: -50°C to +80°C is a reasonable range
    */
    if (sensor->current_temp < -50.0f) sensor->current_temp = -50.0f;
    if (sensor->current_temp > 80.0f) sensor->current_temp = 80.0f;

    /* ========== STEP 2: ADD GAUSSIAN NOISE ========== */

    /* Generate noise: noise ~ N(0, σ) = Gaussian with std dev σ
       Box-Muller gives us N(0, 1), multiply by σ to scale
    */
    float noise = gaussian_random() * SENSOR_NOISE_SIGMA;

    /* Add noise to reading (but not to internal state!)
       This way, repeated reads of same time give different values (realistic)
       but true temperature drifts smoothly (as physics demands)
    */
    float noisy_temperature = sensor->current_temp + noise;

    /* ========== STEP 3: FILL OUTPUT STRUCTURE ========== */

    reading->temperature_celsius = noisy_temperature;
    reading->timestamp_ms = current_time_ms;

    return 0;
}

/**
 * void sensor_set_command(sensor_state_t *sensor, float command)
 *
 * Sets the heater/cooler command for the simulation.
 *
 * The firmware's PID controller calculates a control signal,
 * which flows back to the thermal simulator via this function.
 *
 * This closes the feedback loop:
 * Firmware reads temperature → PID calculates error → Output heating command
 *   → Sensor simulation applies heating → Temperature changes →
 *   → Firmware reads new temperature → loop repeats
 *
 * Command interpretation:
 * - command = +100: Full heater (heat as fast as possible)
 * - command = +50: Half heater (medium heating)
 * - command = 0: No heating (room drifts to ambient)
 * - command = -50: Half cooler (medium cooling)
 * - command = -100: Full cooler (cool as fast as possible)
 *
 * No validation: Firmware is responsible for saturation.
 * We just store whatever value is given.
 *
 * @param sensor Pointer to sensor_state_t
 * @param command Heating command (-100 to +100)
 */
void sensor_set_command(sensor_state_t *sensor, float command)
{
    if (!sensor) return;

    /* Simple assignment - no bounds checking.
       If firmware sends invalid values, we let it fail (reveals bugs).
       In production, you might add checks like:
         if (command < -100.0f) command = -100.0f;
         if (command > +100.0f) command = +100.0f;
    */
    sensor->heater_command = command;
}

/**
 * void sensor_set_temp(sensor_state_t *sensor, float temp_celsius)
 *
 * Manually set the sensor temperature.
 *
 * Used for:
 * - Resetting sensor to known state (e.g., ambient temp)
 * - Simulating sensor initialization
 * - Testing firmware response to edge cases
 *
 * Example: "What if sensor suddenly reads freezing?"
 *   sensor_set_temp(&sensor, 0.0f);
 *   // Next sensor_read will return something close to 0°C (with noise)
 *   // PID will respond with max heating
 *   // Firmware behavior can be verified
 *
 * @param sensor Pointer to sensor_state_t
 * @param temp_celsius New temperature value
 */
void sensor_set_temp(sensor_state_t *sensor, float temp_celsius)
{
    if (!sensor) return;
    sensor->current_temp = temp_celsius;
}

/**
 * float sensor_get_true_temp(const sensor_state_t *sensor)
 *
 * Returns the underlying temperature without noise.
 *
 * Useful for:
 * - Debugging: compare measured (noisy) vs true (clean) temperature
 * - Analyzing sensor noise characteristics
 * - Unit testing: verify physics is correct
 *
 * Example:
 *   sensor_reading_t reading;
 *   sensor_read(&sensor, 1000, &reading);
 *   float true_temp = sensor_get_true_temp(&sensor);
 *
 *   // Compare
 *   if (true_temp != reading.temperature_celsius) {
 *       printf("Noise added: %.3f°C\n",
 *              reading.temperature_celsius - true_temp);
 *   }
 *
 * @param sensor Pointer to sensor_state_t
 * @return Current temperature without noise, in °C
 */
float sensor_get_true_temp(const sensor_state_t *sensor)
{
    if (!sensor) return 0.0f;
    return sensor->current_temp;
}
