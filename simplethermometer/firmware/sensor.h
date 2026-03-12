/**
 * @file sensor.h
 * @brief Temperature Sensor Simulation Header
 *
 * Defines functions to read a simulated temperature sensor.
 * Each line of code is explained in the implementation.
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef SENSOR_H_INCLUDED
#define SENSOR_H_INCLUDED

#include <stdint.h>  /* For uint32_t types */

/* ============================================================================
   FUNCTION DECLARATIONS
   ============================================================================ */

/**
 * @brief Initialize the sensor (seed random number generator)
 *
 * Should be called once at firmware startup.
 */
void sensor_init(void);

/**
 * @brief Read current temperature from simulated sensor
 *
 * @return Temperature in °C (float, range 15.0 to 35.0)
 */
float sensor_read_temperature(void);

#endif /* SENSOR_H_INCLUDED */
