/**
 * @file pid_controller.h
 * @brief Proportional-Integral-Derivative (PID) feedback control
 *
 * PID controller for temperature control.
 *
 * What is PID?
 * ============
 * PID is a feedback control algorithm that:
 * 1. Measures current value (temperature)
 * 2. Compares with desired value (setpoint)
 * 3. Calculates error
 * 4. Adjusts output (heater) based on three terms:
 *    - P (Proportional): Respond to current error
 *    - I (Integral): Respond to accumulated past error
 *    - D (Derivative): Respond to rate of change (damping)
 *
 * Equation:
 *   u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de/dt
 *
 * Where:
 * - u(t) = output (heater command)
 * - e(t) = error = setpoint - actual
 * - Kp, Ki, Kd = tuning gains (to be determined)
 *
 * Why each term?
 * ===============
 * P (Proportional): "React now"
 *   - Directly proportional to current error
 *   - Faster response, but overshoots
 *   - If error is large, output is large
 *
 * I (Integral): "Fix steady-state error"
 *   - Accumulates error over time
 *   - Slowly builds up to eliminate drift
 *   - If system slightly off, integral keeps pushing
 *   - But if too large, causes oscillation
 *
 * D (Derivative): "Dampen and smooth"
 *   - Responds to rate of change
 *   - Acts as damper (reduces overshoot)
 *   - Adds stability but can amplify noise
 *   - Usually kept small
 *
 * Tuning:
 * ========
 * Default gains are set in model/pid_controller.m (Octave/MATLAB)
 * and imported here. For this project:
 * - Kp = 1.5  (main response)
 * - Ki = 0.1  (slow integral correction)
 * - Kd = 0.5  (damping)
 *
 * These are tuned to give:
 * - Error < 0.2°C at steady state
 * - Overshoot < 2°C
 * - Settling time < 120 seconds
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>  /* uint32_t */

/**
 * @defgroup PIDControl PID Controller
 * @{
 */

/* ============================================================================
   CONFIGURATION
   ============================================================================

   These are the PID gains. They should be set via model/pid_gains.h
   after running the tuning script in Octave, but we provide reasonable
   defaults here.

   Process for tuning (Fase 5):
   1. Run model/pid_controller.m in Octave
   2. Analyze Bode plot and step response
   3. Export optimal gains to pid_gains.h
   4. Recompile firmware with new gains
   5. Test on actual system
 */

/* Try to include gains from Octave tuning (Fase 5)
   If not found, use fallback defaults */
#ifndef PID_GAINS_H_INCLUDED
    #define PID_KP  1.5f   ///< Proportional gain
    #define PID_KI  0.1f   ///< Integral gain
    #define PID_KD  0.5f   ///< Derivative gain
#endif

/** Output saturation limit (heater can't exceed this)
    Hardware limitation: heater command must be -100 to +100 */
#define PID_OUTPUT_MAX     100.0f
#define PID_OUTPUT_MIN    -100.0f

/** Integral anti-windup limit (prevent integral from growing unbounded)
    If integral gets too large, it causes large overshoot.
    This caps the integral term to prevent that. */
#define PID_INTEGRAL_MAX    500.0f
#define PID_INTEGRAL_MIN   -500.0f

/* ============================================================================
   DATA STRUCTURES
   ============================================================================ */

/**
 * @struct pid_state_t
 * @brief Encapsulates PID controller state
 *
 * Opaque structure - don't modify fields directly.
 * Use pid_init(), pid_update(), pid_set_gains() instead.
 *
 * Internal fields:
 * - setpoint: Desired temperature (set by user)
 * - error_prev: Previous error (for derivative term)
 * - integral_sum: Accumulated error (for integral term)
 * - output: Last calculated output
 */
typedef struct {
    float setpoint;         ///< Desired temperature (°C)
    float error_prev;       ///< Previous error (for D term)
    float integral_sum;     ///< Accumulated error (for I term)
    float output;           ///< Last calculated PID output
    float kp, ki, kd;       ///< Tuning gains
    uint32_t last_update;   ///< Timestamp of last update (for dt calculation)
} pid_state_t;

/* ============================================================================
   FUNCTION DECLARATIONS
   ============================================================================ */

/**
 * @brief Initialize PID controller
 *
 * Sets up initial state:
 * - Setpoint = initial target temperature
 * - Errors = 0 (we're at setpoint)
 * - Gains = from PID_KP/KI/KD defines
 *
 * Must call before pid_update().
 *
 * @param pid Pointer to pid_state_t to initialize
 * @param setpoint_celsius Initial setpoint (typically room ambient, 20°C)
 * @param current_time_ms Current system time (for derivative calculation)
 * @return 0 on success, -1 on error
 *
 * @example
 *   pid_state_t pid;
 *   if (pid_init(&pid, 25.0f, 0) != 0) {
 *       printf("PID init failed\n");
 *       return 1;
 *   }
 */
int pid_init(pid_state_t *pid, float setpoint_celsius, uint32_t current_time_ms);

/**
 * @brief Update PID controller with new measurement
 *
 * Does the real PID computation:
 * 1. Calculate error: e = setpoint - measurement
 * 2. Calculate P term: P = Kp * e
 * 3. Calculate I term: I = Ki * ∫e dt (accumulated)
 * 4. Calculate D term: D = Kd * de/dt (change in error)
 * 5. Sum: output = P + I + D
 * 6. Saturate: clamp to [-100, +100]
 * 7. Store for next derivative calculation
 *
 * Time delta (dt):
 * - Calculated from timestamps: dt = (now - last_time) / 1000 seconds
 * - If dt is 0 or very small, skip update (prevent division issues)
 * - Larger dt (slow sampling) = less responsive
 * - Smaller dt (fast sampling) = more responsive
 *
 * Output interpretation:
 * - Positive: Heating (send to heater)
 * - Negative: Cooling (send to cooler)
 * - Magnitude: Intensity (0-100%)
 *
 * @param pid Pointer to initialized pid_state_t
 * @param measurement_celsius Current measured temperature
 * @param current_time_ms Current system time (for dt calculation)
 * @return Output command (-100 to +100) suitable for sending to heater
 *
 * @example
 *   // Main control loop
 *   sensor_reading_t reading;
 *   sensor_read(&sensor, now_ms, &reading);
 *   float heater_cmd = pid_update(&pid, reading.temperature_celsius, now_ms);
 *   sensor_set_command(&sensor, heater_cmd);
 */
float pid_update(pid_state_t *pid, float measurement_celsius,
                 uint32_t current_time_ms);

/**
 * @brief Set new PID setpoint (desired temperature)
 *
 * User changes the target temperature via GUI or command.
 * This function updates the setpoint without resetting error history.
 *
 * When setpoint changes:
 * - New error is calculated on next pid_update()
 * - PID smoothly drives temperature to new setpoint
 * - If new setpoint is higher: heating increases
 * - If new setpoint is lower: cooling increases
 *
 * Validation: Should be 15-40°C (from SRS), but we don't enforce
 * here (firmware is responsible for range checking).
 *
 * @param pid Pointer to pid_state_t
 * @param new_setpoint_celsius New target temperature in °C
 *
 * @example
 *   // User moves slider to 28°C on GUI
 *   pid_set_setpoint(&pid, 28.0f);
 *   // Next pid_update will calculate new error and respond
 */
void pid_set_setpoint(pid_state_t *pid, float new_setpoint_celsius);

/**
 * @brief Set custom PID gains (normally from pid_gains.h)
 *
 * Allows runtime tuning of gains without recompiling.
 * Useful for:
 * - Interactive tuning via serial console
 * - Loading gains from stored configuration
 * - Testing different tuning strategies
 *
 * Typical gain ranges:
 * - Kp: 0.1 to 10 (main response)
 * - Ki: 0.0 to 1.0 (slow integral correction)
 * - Kd: 0.0 to 5.0 (damping)
 *
 * Too high gains → oscillation
 * Too low gains → slow response
 *
 * @param pid Pointer to pid_state_t
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 *
 * @example
 *   // Aggressive tuning (faster response, more overshoot)
 *   pid_set_gains(&pid, 2.0f, 0.2f, 0.8f);
 *
 *   // Conservative tuning (slow response, less overshoot)
 *   pid_set_gains(&pid, 0.8f, 0.05f, 0.2f);
 */
void pid_set_gains(pid_state_t *pid, float kp, float ki, float kd);

/**
 * @brief Get current PID output (last calculated command)
 *
 * Returns the output from the most recent pid_update() call.
 * Useful for:
 * - Logging control signal
 * - Debugging (comparing output vs error)
 * - Telemetry transmission
 *
 * @param pid Pointer to pid_state_t
 * @return Last computed output (-100 to +100)
 *
 * @example
 *   float cmd = pid_get_output(&pid);
 *   printf("Heater command: %.1f%%\n", cmd);
 */
float pid_get_output(const pid_state_t *pid);

/**
 * @brief Get current error (setpoint - measurement)
 *
 * Error is the difference between desired and actual.
 * Useful for:
 * - Performance metrics: "How far off are we?"
 * - Debugging: verify error calculation
 * - Telemetry
 *
 * Sign interpretation:
 * - Positive error: System is too cold (below setpoint)
 * - Negative error: System is too hot (above setpoint)
 *
 * @param pid Pointer to pid_state_t
 * @return Last calculated error in °C
 *
 * @example
 *   float error = pid_get_error(&pid);
 *   if (fabs(error) < 0.2f) {
 *       printf("Temperature within tolerance\n");
 *   }
 */
float pid_get_error(const pid_state_t *pid);

/**
 * @brief Reset PID state (clear accumulated errors)
 *
 * Clears:
 * - Integral sum (erases past error history)
 * - Previous error (resets derivative)
 *
 * Used when:
 * - Setpoint changes dramatically (avoids integral overshoot)
 * - System is reset
 * - For testing/simulation
 *
 * Does NOT change: current setpoint or gains
 *
 * @param pid Pointer to pid_state_t
 * @param current_time_ms Current system time (for next derivative calc)
 *
 * @example
 *   // User manually set room temperature (e.g., window opened)
 *   // Clear old errors to prevent overshoot when re-closing window
 *   pid_reset(&pid, now_ms);
 */
void pid_reset(pid_state_t *pid, uint32_t current_time_ms);

/**
 * @} End of PIDControl group
 */

#endif /* PID_CONTROLLER_H */
