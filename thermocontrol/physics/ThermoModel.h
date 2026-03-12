/**
 * @file ThermoModel.h
 * @brief Thermal room model with physics simulation
 *
 * Simulates temperature dynamics of a room using differential equations
 * and numerical integration (Runge-Kutta 4th order).
 *
 * Physics Model:
 * ==============
 * A room with thermal mass, subject to heating/cooling.
 *
 * Governing equation (1st-order ODE):
 *   dT/dt = -1/τ * (T - T_ambient) + (U/100) * K_heater
 *
 * Where:
 * - T = room temperature (°C)
 * - τ = thermal time constant (seconds) - how "slow" is the room
 * - T_ambient = ambient temperature (20°C) - what room tends toward
 * - U = heater command (0-100%) - from firmware PID
 * - K_heater = heater gain (°C/sec) - how fast heater warms room
 *
 * Physical interpretation:
 * - First term: exponential return to ambient (natural cooling/heating)
 * - Second term: forced heating/cooling from heater
 *
 * Steady-state (dT/dt = 0):
 *   T_ss = T_ambient + (U/100) * K_heater * τ
 *
 * Example:
 *   U = 100% (full heat)
 *   τ = 30 sec
 *   K_heater = 0.05 °C/sec
 *   T_ambient = 20°C
 *   => T_ss = 20 + (100/100) * 0.05 * 30 = 20 + 1.5 = 21.5°C
 *
 * Numerical Integration (RK4):
 * ============================
 * ODE solvers convert differential equations to discrete timesteps.
 *
 * Methods (in order of accuracy):
 * 1. Euler (1st order): T_new = T + dT/dt * dt
 *    - Simple, fast, inaccurate for large dt
 * 2. RK2 (2nd order): Uses two slopes, better accuracy
 * 3. RK4 (4th order): Uses four slopes, very accurate
 * 4. Higher orders: Diminishing returns
 *
 * We use RK4 because:
 * - High accuracy (error ~ O(dt^5))
 * - Standard choice in physics simulations
 * - Still fast enough for real-time (only 4 function evaluations)
 *
 * RK4 Algorithm:
 *   k1 = f(t, y)
 *   k2 = f(t + dt/2, y + k1*dt/2)
 *   k3 = f(t + dt/2, y + k2*dt/2)
 *   k4 = f(t + dt, y + k3*dt)
 *   y_new = y + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
 *
 * The weighted average (1, 2, 2, 1) gives 4th-order accuracy.
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef THERMOMODEL_H
#define THERMOMODEL_H

#include <cstdint>

/**
 * @class ThermoModel
 * @brief Thermal room simulation using ODE solver
 *
 * This class manages the physics of temperature change in a room.
 * The firmware (C) provides heater command,
 * this class (C++) computes resulting temperature.
 *
 * Usage:
 *   ThermoModel room;
 *   room.set_heater_command(50.0f);  // 50% heater
 *   room.update(0.1f);               // Advance 0.1 seconds
 *   float T = room.get_temperature();
 *
 * Opaque state:
 * - Current temperature
 * - Heater command
 * - Simulation parameters (τ, K, etc.)
 */
class ThermoModel {
public:
    /* ========== CONSTANTS ========== */

    /** Ambient room temperature (°C)
        Room naturally drifts toward this if no heating/cooling. */
    static constexpr float AMBIENT_TEMP = 20.0f;

    /** Thermal time constant (seconds)
        How fast room responds to heating/cooling.
        τ=30: After 30 sec of constant heating, reaches ~63% of final temp.
        τ=10: Fast response (small room, good insulation)
        τ=100: Slow response (large room, poor insulation)
    */
    static constexpr float TIME_CONSTANT = 30.0f;

    /** Heater gain (°C/sec at full power)
        How fast temperature increases with heater at 100%.
        Real value depends on heater power and room size.
    */
    static constexpr float HEATER_GAIN = 0.05f;

    /** Cooler gain (°C/sec at full power)
        How fast temperature decreases with cooler at 100%.
        Usually slower than heating (cooling less efficient).
    */
    static constexpr float COOLER_GAIN = 0.03f;

    /* ========== CONSTRUCTOR ========== */

    /**
     * Initialize thermal model
     *
     * Sets initial temperature to ambient and no heater command.
     */
    ThermoModel();

    /* ========== SIMULATION INTERFACE ========== */

    /**
     * Set heater/cooler command from firmware
     *
     * Called by firmware's PID output. Updates internal state
     * that will be used in next update() call.
     *
     * @param command Heater command (-100 to +100)
     *        Positive = heating, negative = cooling
     */
    void set_heater_command(float command);

    /**
     * Advance simulation by dt seconds
     *
     * This is the main physics update. Uses RK4 integrator
     * to solve the ODE with high accuracy.
     *
     * Algorithm (RK4):
     * 1. Evaluate dT/dt at current state (k1)
     * 2. Evaluate dT/dt at midpoint with k1 slope (k2)
     * 3. Evaluate dT/dt at midpoint with k2 slope (k3)
     * 4. Evaluate dT/dt at next step with k3 slope (k4)
     * 5. Weighted average: T_new = T + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
     *
     * Typical dt = 0.001 to 0.01 seconds (1-10 ms)
     * Smaller dt = more accurate but slower
     * Larger dt = faster but less accurate (can become unstable)
     *
     * @param dt Time step in seconds (float > 0)
     *
     * @example
     *   // Simulate at 1 kHz (1 ms timestep)
     *   room.update(0.001f);
     *
     *   // Simulate at 100 Hz (10 ms timestep)
     *   room.update(0.01f);
     */
    void update(float dt);

    /**
     * Get current room temperature
     *
     * Returns the simulated "true" temperature without noise.
     * (Firmware adds noise separately in sensor_read().)
     *
     * @return Room temperature in °C (float)
     */
    float get_temperature() const;

    /**
     * Get heater command (what firmware requested)
     *
     * For logging/debugging. Shows the control signal.
     *
     * @return Last heater command (-100 to +100)
     */
    float get_heater_command() const;

    /**
     * Reset to initial state
     *
     * Sets temperature back to ambient, clears heater command.
     * Used for test resets.
     *
     * @param initial_temp Starting temperature (default: AMBIENT_TEMP)
     */
    void reset(float initial_temp = AMBIENT_TEMP);

    /* ========== PHYSICS CALCULATION (Private internal) ========== */

private:
    /** Current room temperature in °C */
    float temperature_;

    /** Current heater/cooler command (-100 to +100) */
    float heater_command_;

    /**
     * Calculate temperature derivative dT/dt
     *
     * Internal function used by RK4 integrator.
     * Implements the differential equation:
     *   dT/dt = -1/τ * (T - T_ambient) + (U/100) * K
     *
     * @param T Current temperature (°C)
     * @return dT/dt (°C/second)
     */
    float calculate_derivative(float T) const;
};

#endif /* THERMOMODEL_H */
