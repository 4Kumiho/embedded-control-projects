/**
 * @file thermocontrol_c_api.cpp
 * @brief C API implementation - bridge between C firmware and C++ physics
 *
 * Implements the C interface by wrapping C++ ThermoModel class.
 *
 * Design:
 * - Single global ThermoModel instance (hidden from C code)
 * - Functions convert C parameters to C++ calls
 * - Error handling at C level (return int codes)
 * - Memory management hidden from C code
 *
 * Why static global?
 * - Embedded systems typically have single physics engine
 * - Avoids malloc/new (no dynamic allocation)
 * - Fast access (no pointer dereferencing)
 * - Simplicity (C code doesn't manage memory)
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include "thermocontrol_c_api.h"
#include "ThermoModel.h"
#include <cstdio>     /* printf for debugging */

/* ============================================================================
   INTERNAL STATE (Hidden from C Code)
   ============================================================================ */

/** Single global instance of the physics engine
    Hidden from C code - accessed only through API functions.
    Static variable: created at program start, destroyed at program end.
    No dynamic allocation (new/delete) needed.
*/
static ThermoModel g_model;

/** Initialization flag - prevents double-initialization */
static bool g_initialized = false;

/* ============================================================================
   INITIALIZATION / SHUTDOWN
   ============================================================================ */

/**
 * int thermocontrol_init(void)
 *
 * Initialize the physics engine.
 *
 * In C++, the ThermoModel constructor is called automatically
 * when the static variable is created. So this function mainly
 * just sets a flag and validates.
 *
 * In a more complex system, you might allocate memory here,
 * open files, start threads, etc.
 *
 * @return 0 on success, -1 on error
 */
int thermocontrol_init(void)
{
    if (g_initialized) {
        fprintf(stderr, "WARNING: thermocontrol_init() called twice\n");
        return 0;  /* Already initialized, not an error */
    }

    /* Constructor of g_model has already run
       (static global initialization happens before main())
       Just mark as ready. */
    g_initialized = true;

    return 0;
}

/**
 * int thermocontrol_shutdown(void)
 *
 * Shutdown the physics engine.
 *
 * Destructor of g_model will run automatically at program end.
 * This function is mostly for symmetry and future extensibility.
 *
 * @return 0 on success
 */
int thermocontrol_shutdown(void)
{
    g_initialized = false;
    return 0;
}

/* ============================================================================
   PHYSICS UPDATE
   ============================================================================ */

/**
 * int thermocontrol_update(float dt)
 *
 * Update physics simulation.
 *
 * Forwards call to C++ model's update() method.
 * Adds error checking at the C boundary.
 *
 * @param dt Time step in seconds
 * @return 0 on success, -1 on error
 */
int thermocontrol_update(float dt)
{
    if (!g_initialized) {
        fprintf(stderr, "ERROR: thermocontrol_update() before init\n");
        return -1;
    }

    if (dt <= 0.0f) {
        fprintf(stderr, "ERROR: thermocontrol_update() with dt <= 0\n");
        return -1;
    }

    /* Call C++ model's update method
       The model's update() handles the actual physics (RK4 integration)
    */
    g_model.update(dt);

    return 0;
}

/* ============================================================================
   COMMAND INPUT
   ============================================================================ */

/**
 * void thermocontrol_set_command(float command)
 *
 * Set heater command from firmware.
 *
 * Simple pass-through to C++ model.
 * Firmware (C code) calls this, we forward to C++ model.
 *
 * @param command Heater command (-100 to +100)
 */
void thermocontrol_set_command(float command)
{
    if (!g_initialized) {
        fprintf(stderr, "ERROR: thermocontrol_set_command() before init\n");
        return;
    }

    /* Clamp command to valid range (safety check)
       Firmware should do this, but we protect anyway.
    */
    if (command > 100.0f) command = 100.0f;
    if (command < -100.0f) command = -100.0f;

    /* Forward to C++ model */
    g_model.set_heater_command(command);
}

/* ============================================================================
   STATE QUERIES
   ============================================================================ */

/**
 * float thermocontrol_get_temperature(void)
 *
 * Get current room temperature from physics engine.
 *
 * Firmware sensor driver calls this and adds noise to simulate
 * realistic sensor readings.
 *
 * @return Room temperature in °C
 */
float thermocontrol_get_temperature(void)
{
    if (!g_initialized) {
        fprintf(stderr, "ERROR: thermocontrol_get_temperature() before init\n");
        return 20.0f;  /* Return safe default (ambient) */
    }

    return g_model.get_temperature();
}

/**
 * float thermocontrol_get_command(void)
 *
 * Get the last heater command.
 *
 * For logging and debugging. Shows what firmware requested.
 *
 * @return Last heater command (-100 to +100)
 */
float thermocontrol_get_command(void)
{
    if (!g_initialized) {
        return 0.0f;  /* No command if not initialized */
    }

    return g_model.get_heater_command();
}

/* ============================================================================
   SIMULATION CONTROL
   ============================================================================ */

/**
 * int thermocontrol_reset(float initial_temp)
 *
 * Reset physics to initial state.
 *
 * @param initial_temp Starting temperature (0 = use ambient)
 * @return 0 on success, -1 on error
 */
int thermocontrol_reset(float initial_temp)
{
    if (!g_initialized) {
        fprintf(stderr, "ERROR: thermocontrol_reset() before init\n");
        return -1;
    }

    /* If initial_temp is 0, use ambient (default) */
    if (initial_temp == 0.0f) {
        initial_temp = ThermoModel::AMBIENT_TEMP;
    }

    /* Reset the model */
    g_model.reset(initial_temp);

    return 0;
}

/* ============================================================================
   PARAMETER QUERIES
   ============================================================================ */

/**
 * float thermocontrol_get_ambient_temp(void)
 *
 * Get ambient temperature constant.
 *
 * Useful for documentation and tests.
 *
 * @return Ambient temperature in °C
 */
float thermocontrol_get_ambient_temp(void)
{
    return ThermoModel::AMBIENT_TEMP;
}

/**
 * float thermocontrol_get_time_constant(void)
 *
 * Get thermal time constant.
 *
 * @return Time constant in seconds
 */
float thermocontrol_get_time_constant(void)
{
    return ThermoModel::TIME_CONSTANT;
}

/**
 * float thermocontrol_get_heater_gain(void)
 *
 * Get heater gain constant.
 *
 * @return Heater gain in °C/sec
 */
float thermocontrol_get_heater_gain(void)
{
    return ThermoModel::HEATER_GAIN;
}
