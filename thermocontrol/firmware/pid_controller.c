/**
 * @file pid_controller.c
 * @brief PID Controller implementation
 *
 * Implements the Proportional-Integral-Derivative (PID) feedback control
 * algorithm for temperature control.
 *
 * Key concepts:
 * =============
 * PID Output = Kp·e(t) + Ki·∫e(t)dt + Kd·de/dt
 *
 * Where:
 * - e(t) = error = setpoint - measurement
 * - ∫e(t)dt = integral (accumulated error over time)
 * - de/dt = derivative (rate of error change)
 * - Kp, Ki, Kd = tuning gains (determined in Octave, Fase 5)
 *
 * Implementation notes:
 * ====================
 * 1. Integral is updated incrementally: I_new = I_old + e * dt
 *    (Instead of computing full integral each time)
 *
 * 2. Anti-windup: Clamp integral to prevent unbounded growth
 *    (Integral saturation at PID_INTEGRAL_MAX/MIN)
 *
 * 3. Derivative: Computed from change in error: (e - e_prev) / dt
 *    (Raw derivative is noisy, but acceptable for our slow updates)
 *
 * 4. Output saturation: Clamp output to hardware limits [-100, +100]
 *    (Heater can't exceed max power)
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include "pid_controller.h"
#include <math.h>    /* fabs() for absolute value */
#include <string.h>  /* memset() */

/* ============================================================================
   PID INITIALIZATION
   ============================================================================ */

/**
 * int pid_init(pid_state_t *pid, float setpoint_celsius, uint32_t current_time_ms)
 *
 * Initialize PID controller to known state.
 *
 * Initial state:
 * - error_prev = 0 (no derivative on first call)
 * - integral_sum = 0 (no accumulated error yet)
 * - output = 0 (no command initially)
 * - last_update = current time (for next derivative calc)
 * - setpoint = setpoint_celsius (what user wants)
 * - kp, ki, kd = from #defines (can be changed later with pid_set_gains)
 *
 * Why initialize to zero?
 * - Clean slate prevents spurious behavior on startup
 * - First measurement will calculate error properly
 * - Integral builds up gradually (prevents kick)
 *
 * @param pid Pointer to uninitialized pid_state_t
 * @param setpoint_celsius Target temperature (e.g., 20°C for room ambient)
 * @param current_time_ms System time from platform.h (e.g., 0 if starting fresh)
 * @return 0 on success, -1 if pid is null
 */
int pid_init(pid_state_t *pid, float setpoint_celsius, uint32_t current_time_ms)
{
    /* Input validation */
    if (!pid) {
        return -1;
    }

    /* Zero-initialize entire structure (defensive programming) */
    memset(pid, 0, sizeof(pid_state_t));

    /* Set target temperature */
    pid->setpoint = setpoint_celsius;

    /* Initialize gains from #defines (set in pid_gains.h or defaults above) */
    pid->kp = PID_KP;
    pid->ki = PID_KI;
    pid->kd = PID_KD;

    /* Initialize timestamp for derivative calculation */
    pid->last_update = current_time_ms;

    /* All error terms start at zero (clean slate) */
    pid->error_prev = 0.0f;
    pid->integral_sum = 0.0f;
    pid->output = 0.0f;

    return 0;
}

/* ============================================================================
   PID CONTROL UPDATE
   ============================================================================ */

/**
 * float pid_update(pid_state_t *pid, float measurement_celsius,
 *                  uint32_t current_time_ms)
 *
 * Main PID computation. Called every time a new measurement arrives.
 *
 * Algorithm (step-by-step):
 * =========================
 *
 * Step 1: Calculate time delta
 *   - dt = (current_time_ms - last_update) / 1000.0f  [convert to seconds]
 *   - If dt ≤ 0, skip update (no time passed)
 *   - Remember current time for next call
 *
 * Step 2: Calculate error
 *   - e = setpoint - measurement
 *   - Positive error: too cold, need heating
 *   - Negative error: too hot, need cooling
 *
 * Step 3: Proportional term (P)
 *   - P = Kp * e
 *   - Immediate response to current error
 *   - Larger error → larger response
 *
 * Step 4: Integral term (I) - accumulates past error
 *   - I += Ki * e * dt  (increment integral)
 *   - Integral grows if error persists
 *   - AntWindup: clamp to PID_INTEGRAL_MIN/MAX
 *   - Prevents unbounded integral from causing overshoot
 *
 * Step 5: Derivative term (D) - dampens oscillations
 *   - de/dt = (e - e_prev) / dt
 *   - D = Kd * de/dt
 *   - If error is increasing (de/dt > 0), reduce output
 *   - If error is decreasing (de/dt < 0), increase output
 *   - Acts like damper: smooths response
 *
 * Step 6: Sum all terms
 *   - output = P + I + D
 *   - This is the controller command
 *
 * Step 7: Saturate output
 *   - Clamp to [-100, +100] (hardware limits)
 *   - If output > 100: set to 100 (max heating)
 *   - If output < -100: set to -100 (max cooling)
 *   - Prevents commanding impossible values
 *
 * Step 8: Remember current error for next derivative
 *   - e_prev = e
 *
 * Return the output command
 *
 * Example behavior:
 * =================
 * Setpoint = 25°C, current measurement = 20°C
 *   e = 25 - 20 = +5°C
 *   P = 1.5 * 5 = 7.5
 *   I = 0 (first call, integral not yet accumulated)
 *   D = 0 (first call, no previous error)
 *   output = 7.5 → heater turns on to 7.5%
 *
 * Next call (after 100ms): measurement = 20.1°C
 *   dt = 0.1 seconds
 *   e = 25 - 20.1 = 4.9°C
 *   P = 1.5 * 4.9 = 7.35
 *   I += 0.1 * 4.9 * 0.1 = 0.049
 *   de/dt = (4.9 - 5.0) / 0.1 = -1.0
 *   D = 0.5 * -1.0 = -0.5  (damping: error decreasing, reduce output)
 *   output = 7.35 + 0.049 - 0.5 = 6.9 → heater at 6.9%
 *
 * @param pid Pointer to initialized pid_state_t
 * @param measurement_celsius Current measurement from sensor (with noise)
 * @param current_time_ms Current system time in milliseconds
 * @return PID output command (-100 to +100) for heater/cooler
 */
float pid_update(pid_state_t *pid, float measurement_celsius,
                 uint32_t current_time_ms)
{
    /* Input validation */
    if (!pid) {
        return 0.0f;
    }

    /* ========== STEP 1: CALCULATE TIME DELTA ========== */

    /* How long since last update? (in milliseconds)
       Needed for derivative and integral terms. */
    uint32_t elapsed_ms = current_time_ms - pid->last_update;
    float dt = (float)elapsed_ms / 1000.0f;  /* Convert to seconds */

    /* If no time has passed, don't update (prevents division by zero in derivative) */
    if (dt < 0.001f) {  /* Less than 1 ms */
        return pid->output;  /* Return last output */
    }

    /* Remember current time for next update */
    pid->last_update = current_time_ms;

    /* ========== STEP 2: CALCULATE ERROR ========== */

    /* Error = setpoint - actual
       Positive: too cold (below setpoint) → need heating
       Negative: too hot (above setpoint) → need cooling
    */
    float error = pid->setpoint - measurement_celsius;

    /* ========== STEP 3: PROPORTIONAL TERM ========== */

    /* P = Kp * e
       Immediate response. Larger error → larger output.
       This alone would oscillate (overshoots), but we'll dampen it.
    */
    float p_term = pid->kp * error;

    /* ========== STEP 4: INTEGRAL TERM (with anti-windup) ========== */

    /* I += Ki * e * dt
       Accumulates error over time. If temperature slightly below setpoint,
       integral slowly builds up until steady-state error is eliminated.

       Why dt in the formula?
       - Integral should accumulate: ∫e dt ≈ Σ(e * Δt)
       - If dt is large, integration step is larger
       - If dt is small, integration step is smaller
       - Accounts for variable sampling rate
    */
    pid->integral_sum += pid->ki * error * dt;

    /* Anti-windup: Clamp integral to prevent unbounded growth
       Large integral can cause massive overshoot.
       This cap prevents that. */
    if (pid->integral_sum > PID_INTEGRAL_MAX) {
        pid->integral_sum = PID_INTEGRAL_MAX;
    }
    if (pid->integral_sum < PID_INTEGRAL_MIN) {
        pid->integral_sum = PID_INTEGRAL_MIN;
    }

    float i_term = pid->integral_sum;

    /* ========== STEP 5: DERIVATIVE TERM ========== */

    /* de/dt = (current_error - previous_error) / dt
       Tells us if error is increasing or decreasing.
       - If error decreasing (de/dt < 0): dampen output
       - If error increasing (de/dt > 0): increase output more
       Acts as shock absorber: smooths oscillations.

       NOTE: Real sensors are noisy, so raw derivative is risky.
       In production, you'd filter or only use if measurement passes filter.
       Here, it's acceptable because sensor noise is synthetic and small (σ=0.05).
    */
    float de_dt = (error - pid->error_prev) / dt;
    float d_term = pid->kd * de_dt;

    /* Remember this error for next time (for next derivative) */
    pid->error_prev = error;

    /* ========== STEP 6: SUM ALL TERMS ========== */

    /* output = P + I + D
       This is the raw, unsaturated control signal */
    float output = p_term + i_term + d_term;

    /* ========== STEP 7: SATURATE OUTPUT ========== */

    /* Clamp to hardware limits.
       Real heater can't exceed ±100% power. */
    if (output > PID_OUTPUT_MAX) {
        output = PID_OUTPUT_MAX;
    }
    if (output < PID_OUTPUT_MIN) {
        output = PID_OUTPUT_MIN;
    }

    /* ========== STEP 8: REMEMBER OUTPUT FOR LOGGING/TELEMETRY ========== */

    pid->output = output;

    /* Return the command */
    return output;
}

/* ============================================================================
   PID GAIN ADJUSTMENT
   ============================================================================ */

/**
 * void pid_set_setpoint(pid_state_t *pid, float new_setpoint_celsius)
 *
 * Update the target temperature.
 *
 * Called when user changes setpoint via GUI or command.
 * Does NOT reset error history - allows smooth transition.
 *
 * @param pid Pointer to pid_state_t
 * @param new_setpoint_celsius New target temperature in °C
 */
void pid_set_setpoint(pid_state_t *pid, float new_setpoint_celsius)
{
    if (!pid) return;
    pid->setpoint = new_setpoint_celsius;
    /* Note: We don't reset integral/errors here.
       This allows a smooth transition to the new setpoint.
       If user jumps from 20°C to 40°C, PID will gradually ramp up heating.
    */
}

/**
 * void pid_set_gains(pid_state_t *pid, float kp, float ki, float kd)
 *
 * Runtime gain tuning.
 *
 * Allows changing gains without recompiling.
 * Useful for:
 * - Interactive tuning via serial console
 * - Loading stored configuration
 * - Experimenting with different strategies
 *
 * @param pid Pointer to pid_state_t
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void pid_set_gains(pid_state_t *pid, float kp, float ki, float kd)
{
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/* ============================================================================
   PID OBSERVATION / DEBUGGING
   ============================================================================ */

/**
 * float pid_get_output(const pid_state_t *pid)
 *
 * Get last computed output.
 *
 * @param pid Pointer to pid_state_t
 * @return Last output command (-100 to +100)
 */
float pid_get_output(const pid_state_t *pid)
{
    if (!pid) return 0.0f;
    return pid->output;
}

/**
 * float pid_get_error(const pid_state_t *pid)
 *
 * Get current error (setpoint - measurement).
 *
 * @param pid Pointer to pid_state_t
 * @return Current error in °C
 */
float pid_get_error(const pid_state_t *pid)
{
    if (!pid) return 0.0f;
    return pid->error_prev;
}

/**
 * void pid_reset(pid_state_t *pid, uint32_t current_time_ms)
 *
 * Reset accumulated errors and derivative memory.
 *
 * Clears:
 * - Integral sum (erases past error history)
 * - Previous error (resets derivative)
 *
 * Used when:
 * - Setpoint changes dramatically
 * - System experiences external disturbance
 * - For testing/debugging
 *
 * Does NOT change: setpoint or gains
 *
 * @param pid Pointer to pid_state_t
 * @param current_time_ms Current system time (for next derivative calc)
 */
void pid_reset(pid_state_t *pid, uint32_t current_time_ms)
{
    if (!pid) return;
    pid->error_prev = 0.0f;
    pid->integral_sum = 0.0f;
    pid->output = 0.0f;
    pid->last_update = current_time_ms;
}
