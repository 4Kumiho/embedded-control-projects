/**
 * @file thermocontrol_c_api.h
 * @brief C interface to C++ physics engine
 *
 * Bridge between C firmware and C++ physics simulation.
 *
 * Why a C API?
 * ============
 * Firmware is written in C (embedded standard).
 * Physics engine is C++ (easier to write, better testing).
 * We need a boundary layer that converts between them.
 *
 * Design pattern:
 * - Firmware never directly calls C++ code
 * - All interactions go through this C API
 * - Hides C++ complexity from C code
 *
 * C vs C++ incompatibilities:
 * - C doesn't have classes, namespaces, templates
 * - C has no "this" pointer
 * - C++ mangling makes symbols incompatible
 * - Solution: extern "C" tells compiler not to mangle
 *
 * Example flow:
 *   firmware.c: sensor_set_command(heater_cmd)
 *   ↓
 *   thermocontrol_c_api.c: thermocontrol_set_command(heater_cmd)
 *   ↓
 *   ThermoModel.cpp: model->set_heater_command(heater_cmd)
 *   ↓
 *   ThermoModel.cpp: model->update(dt)
 *   ↓
 *   thermocontrol_c_api.c: return current temperature
 *   ↓
 *   firmware.c: get temperature value
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef THERMOCONTROL_C_API_H
#define THERMOCONTROL_C_API_H

#include <stdint.h>   /* uint32_t */

/**
 * @defgroup ThermoAPI Physics Engine C API
 * @{
 */

/* ============================================================================
   C-LANGUAGE DECLARATIONS
   ============================================================================

   extern "C" tells C++ compiler:
   "These functions have C linkage, don't mangle names"
   "C code will call them, so make them compatible"

   Only valid when compiling with C++ compiler.
   When included in C files, the extern "C" is not needed (C is default).
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
   INITIALIZATION
   ============================================================================ */

/**
 * @brief Initialize the physics engine
 *
 * Must be called once at program startup before using any other API.
 *
 * Sets up:
 * - ThermoModel instance (heap-allocated by implementation)
 * - Initial temperature to ambient
 * - All parameters to defaults
 *
 * @return 0 on success, -1 on error
 *
 * @example
 *   if (thermocontrol_init() != 0) {
 *       printf("Physics engine init failed\n");
 *       return 1;
 *   }
 */
int thermocontrol_init(void);

/**
 * @brief Shutdown the physics engine
 *
 * Cleans up resources. Call once at program end.
 *
 * @return 0 on success, -1 on error
 */
int thermocontrol_shutdown(void);

/* ============================================================================
   PHYSICS UPDATE
   ============================================================================ */

/**
 * @brief Update physics simulation by dt seconds
 *
 * Advances the thermal model using RK4 integrator.
 * Called frequently (e.g., every 1-10 ms) for accuracy.
 *
 * Algorithm:
 * - Takes current heater command (set via thermocontrol_set_command)
 * - Evaluates temperature derivative
 * - Integrates forward by dt
 * - Updates internal temperature state
 *
 * Time accumulation:
 * - Multiple calls to update accumulate properly
 * - No "lost" time between calls
 * - dt values don't need to be uniform (adaptive stepping possible)
 *
 * @param dt Time step in seconds (float > 0)
 * @return 0 on success, -1 on error (dt invalid)
 *
 * @example
 *   // Update at 1 kHz (1 ms timestep)
 *   thermocontrol_update(0.001f);
 *
 *   // Update at 100 Hz (10 ms timestep)
 *   thermocontrol_update(0.01f);
 */
int thermocontrol_update(float dt);

/* ============================================================================
   COMMAND INPUT (From Firmware)
   ============================================================================ */

/**
 * @brief Set heater/cooler command
 *
 * Called by firmware's PID controller to command the heater.
 * The command is applied in the next thermocontrol_update() call.
 *
 * Semantics:
 * - Positive command: Heating (command towards +100)
 * - Negative command: Cooling (command towards -100)
 * - Zero command: No heating/cooling
 *
 * Range: -100 to +100
 * If firmware sends values outside this range, behavior is undefined
 * (Implementation may clamp or ignore).
 *
 * @param command Heater command percentage (-100 to +100)
 *
 * @example
 *   // Firmware PID output: 50% heater
 *   thermocontrol_set_command(50.0f);
 *
 *   // Then advance physics
 *   thermocontrol_update(0.01f);  // 10 ms timestep
 *
 *   // Query resulting temperature
 *   float T = thermocontrol_get_temperature();
 */
void thermocontrol_set_command(float command);

/* ============================================================================
   STATE QUERIES (To Firmware / GUI)
   ============================================================================ */

/**
 * @brief Get current room temperature
 *
 * Returns the "true" temperature from the simulation
 * (before firmware adds sensor noise).
 *
 * This is what the physics engine has computed.
 * Firmware will add Gaussian noise to simulate sensor jitter.
 *
 * @return Room temperature in °C (float)
 *
 * @example
 *   float true_temp = thermocontrol_get_temperature();
 *   // Add noise for realistic sensor reading
 *   float measured_temp = true_temp + gaussian_noise();
 */
float thermocontrol_get_temperature(void);

/**
 * @brief Get the heater command (for logging)
 *
 * Returns what firmware last requested via thermocontrol_set_command().
 * Useful for diagnostics and telemetry.
 *
 * @return Last heater command (-100 to +100)
 */
float thermocontrol_get_command(void);

/* ============================================================================
   SIMULATION CONTROL
   ============================================================================ */

/**
 * @brief Reset simulation to initial state
 *
 * Sets temperature back to ambient, clears heater command.
 * Useful for test resets or starting fresh.
 *
 * @param initial_temp Starting temperature (°C), or 0 for ambient default
 * @return 0 on success, -1 on error
 *
 * @example
 *   // Reset to ambient (20°C)
 *   thermocontrol_reset(0);
 *
 *   // Reset to specific temperature
 *   thermocontrol_reset(25.0f);
 */
int thermocontrol_reset(float initial_temp);

/* ============================================================================
   PARAMETER QUERIES (For documentation/testing)
   ============================================================================ */

/**
 * @brief Get ambient temperature (constant)
 *
 * The temperature the room naturally drifts toward
 * when no heating/cooling is applied.
 *
 * @return Ambient temperature in °C
 */
float thermocontrol_get_ambient_temp(void);

/**
 * @brief Get thermal time constant
 *
 * Characterizes how fast the room responds to heating.
 * τ = 30 sec means after 30 sec of constant heating,
 * room reaches ~63% of final steady-state temperature.
 *
 * @return Time constant in seconds
 */
float thermocontrol_get_time_constant(void);

/**
 * @brief Get heater gain
 *
 * Maximum heating rate (°C/sec) when heater is at 100%.
 * Depends on heater power and room thermal mass.
 *
 * @return Heater gain in °C/sec
 */
float thermocontrol_get_heater_gain(void);

/**
 * @} End of ThermoAPI group
 */

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* THERMOCONTROL_C_API_H */
