/**
 * @file ThermoModel.cpp
 * @brief Thermal model implementation with RK4 integrator
 *
 * Implements the physics of temperature change in a room
 * using 4th-order Runge-Kutta numerical integration.
 *
 * Core equation:
 *   dT/dt = -1/τ * (T - T_ambient) + (U/100) * K_heater
 *
 * Implemented as RK4 integrator which is accurate for nonlinear ODEs.
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include "ThermoModel.h"
#include <algorithm>  /* std::min, std::max */

/* ============================================================================
   CONSTRUCTOR
   ============================================================================ */

/**
 * ThermoModel::ThermoModel()
 *
 * Initialize with room at ambient temperature and no heating.
 */
ThermoModel::ThermoModel()
    : temperature_(AMBIENT_TEMP), heater_command_(0.0f)
{
    /* Constructor body empty - initialization done in member initializer list
       This is the modern C++ style (RAII - Resource Acquisition Is Initialization)
    */
}

/* ============================================================================
   INTERFACE FUNCTIONS
   ============================================================================ */

/**
 * void ThermoModel::set_heater_command(float command)
 *
 * Store the heater command from firmware's PID output.
 *
 * The command is applied in next update() call.
 * We don't apply it immediately because:
 * 1. Decouples control from simulation frequency
 * 2. Firmware can update command without triggering physics update
 * 3. Cleaner separation of concerns
 *
 * Range checking: We accept any value.
 * Firmware should saturate to [-100, 100], but if it doesn't,
 * the calculate_derivative() function handles it gracefully.
 *
 * @param command Heating command from PID (-100 to +100)
 */
void ThermoModel::set_heater_command(float command)
{
    heater_command_ = command;
}

/**
 * float ThermoModel::get_temperature() const
 *
 * Return the current simulated temperature.
 *
 * @return Room temperature in °C
 */
float ThermoModel::get_temperature() const
{
    return temperature_;
}

/**
 * float ThermoModel::get_heater_command() const
 *
 * Return the last heater command.
 *
 * @return Heater command (-100 to +100)
 */
float ThermoModel::get_heater_command() const
{
    return heater_command_;
}

/**
 * void ThermoModel::reset(float initial_temp)
 *
 * Reset simulation to initial state.
 *
 * @param initial_temp Starting temperature (default: 20°C)
 */
void ThermoModel::reset(float initial_temp)
{
    temperature_ = initial_temp;
    heater_command_ = 0.0f;
}

/* ============================================================================
   DERIVATIVE CALCULATION
   ============================================================================

   This is the heart of the physics simulation.
   Calculates the ODE: dT/dt = f(T, U)
 */

/**
 * float ThermoModel::calculate_derivative(float T) const
 *
 * Calculate temperature derivative dT/dt.
 *
 * Implements the thermal dynamics equation:
 *   dT/dt = -1/τ * (T - T_ambient) + (U/100) * K
 *
 * Decomposed into two physical effects:
 *
 * 1. Natural drift to ambient:
 *    Δt_drift = -1/τ * (T - T_ambient)
 *
 *    If T > T_ambient:
 *      - (T - T_ambient) is positive
 *      - -1/τ makes it negative
 *      - dT/dt < 0, temperature decreases (room cools)
 *
 *    If T < T_ambient:
 *      - (T - T_ambient) is negative
 *      - -1/τ makes it positive
 *      - dT/dt > 0, temperature increases (room warms)
 *
 *    Magnitude depends on τ (time constant):
 *      - Small τ: fast drift
 *      - Large τ: slow drift
 *
 * 2. Heater effect:
 *    Δt_heat = (U/100) * K
 *
 *    U = command (0-100)
 *    U/100 = normalized command (0-1)
 *    K = heater gain (°C/sec)
 *    Product: heating rate proportional to command
 *
 *    If U > 0: heating (positive contribution)
 *    If U < 0: cooling (negative contribution)
 *    If U = 0: no heating/cooling
 *
 * Final equation:
 *    dT/dt = drift + heat
 *
 * Example simulation:
 * - T = 20°C, T_ambient = 20°C, U = 0
 *   => drift = 0, heat = 0, dT/dt = 0 (equilibrium)
 *
 * - T = 20°C, T_ambient = 20°C, U = 100
 *   => drift = 0, heat = 0.05, dT/dt = 0.05°C/sec (heating)
 *
 * - T = 25°C, T_ambient = 20°C, U = 0
 *   => drift = -1/30 * 5 = -0.167°C/sec, dT/dt = -0.167
 *   => Temperature decays back to ambient (after 30s, reaches 63% of starting deviation)
 *
 * @param T Current temperature (°C)
 * @return Rate of change dT/dt (°C/sec)
 */
float ThermoModel::calculate_derivative(float T) const
{
    /* Term 1: Natural drift toward ambient temperature
       -1/τ * (T - T_ambient)

       The negative sign ensures:
       - If room is hot (T > T_ambient), drift pulls temperature DOWN
       - If room is cold (T < T_ambient), drift pulls temperature UP
    */
    float drift = -(T - AMBIENT_TEMP) / TIME_CONSTANT;

    /* Term 2: Heater/cooler effect
       (U/100) * K

       - U/100 converts percentage to fraction [0, 1]
       - K is the heater gain (max heating rate in °C/sec)
       - Multiply to get actual heating rate
    */
    float gain = (heater_command_ >= 0.0f) ? HEATER_GAIN : COOLER_GAIN;
    float heating = (heater_command_ / 100.0f) * gain;

    /* Sum both effects */
    float dT_dt = drift + heating;

    return dT_dt;
}

/* ============================================================================
   RUNGE-KUTTA 4TH ORDER INTEGRATOR
   ============================================================================

   RK4 is a standard ODE solver that provides high accuracy.

   Algorithm:
   ---------
   To solve: dy/dt = f(t, y)
   With initial condition: y(t) ≈ y_current

   Compute four slopes:
     k1 = f(t, y)                          // Slope at current point
     k2 = f(t + dt/2, y + k1*dt/2)         // Slope at midpoint (using k1)
     k3 = f(t + dt/2, y + k2*dt/2)         // Slope at midpoint (using k2)
     k4 = f(t + dt, y + k3*dt)             // Slope at next point (using k3)

   Take weighted average:
     y_new = y + (k1 + 2*k2 + 2*k3 + k4) * dt / 6

   The weights (1, 2, 2, 1) are chosen to give 4th-order accuracy.
   Error scales as O(dt^5), so very accurate for reasonable dt.

   Why RK4 over Euler?
   ------------------
   Euler: y_new = y + f(y) * dt
          - Fast but inaccurate
          - Error scales as O(dt^2) - grows quickly
          - For dt=0.01, error accumulates rapidly

   RK4:   Uses 4 evaluations to build better approximation
          - More function calls (4x slower)
          - Much more accurate (O(dt^5) error)
          - Acceptable since calculate_derivative() is cheap

   Performance trade-off:
   - Euler: 10,000 Hz simulator → 1 function call per update
   - RK4:   10,000 Hz simulator → 4 function calls per update = 4x slower
   - Modern CPUs easily handle this

   In our simulation:
   - We use RK4 for physical correctness
   - Could use Euler for speed if needed
   - Firmware-side update (100 ms) is much slower than physics (1-10 ms)
   - No bottleneck
 */

/**
 * void ThermoModel::update(float dt)
 *
 * Advance physics simulation by dt seconds using RK4 integrator.
 *
 * Implementation of Runge-Kutta 4th order integrator.
 * Solves: dT/dt = f(T, U) where U is heater command
 *
 * @param dt Time step in seconds (typical: 0.001 to 0.01)
 */
void ThermoModel::update(float dt)
{
    /* Safety check: prevent negative or zero timesteps
       (Would cause division issues or infinite loops) */
    if (dt <= 0.0f) {
        return;  /* No update if dt invalid */
    }

    /* ========== RUNGE-KUTTA 4TH ORDER INTEGRATION ========== */

    /* k1: Slope at current state
       Calculate dT/dt using current temperature
    */
    float k1 = calculate_derivative(temperature_);

    /* k2: Slope at midpoint using k1
       Estimate temperature at t + dt/2 using k1 slope
       T_mid1 = T + k1 * (dt/2)
       Then evaluate dT/dt at that point
    */
    float k2 = calculate_derivative(temperature_ + k1 * dt * 0.5f);

    /* k3: Slope at midpoint using k2
       Estimate temperature at t + dt/2 using k2 slope
       T_mid2 = T + k2 * (dt/2)
       Then evaluate dT/dt at that point
    */
    float k3 = calculate_derivative(temperature_ + k2 * dt * 0.5f);

    /* k4: Slope at next point using k3
       Estimate temperature at t + dt using k3 slope
       T_next_approx = T + k3 * dt
       Then evaluate dT/dt at that point
    */
    float k4 = calculate_derivative(temperature_ + k3 * dt);

    /* Weighted average of four slopes
       Formula: T_new = T + (k1 + 2*k2 + 2*k3 + k4) * dt / 6

       Weights explanation:
       - k1 and k4 weighted as 1x (boundary slopes)
       - k2 and k3 weighted as 2x (midpoint slopes are more representative)
       - Denominator 6 normalizes the average
       - Total weight: 1 + 2 + 2 + 1 = 6

       This weighting is optimal for 4th-order accuracy.
    */
    float slope_avg = (k1 + 2.0f * k2 + 2.0f * k3 + k4) / 6.0f;
    temperature_ += slope_avg * dt;

    /* Optional: Clamp temperature to physically reasonable range
       Prevents unrealistic values from accumulating errors.
       (-50°C to +80°C is a reasonable limit for a room)
    */
    temperature_ = std::max(-50.0f, std::min(80.0f, temperature_));
}
