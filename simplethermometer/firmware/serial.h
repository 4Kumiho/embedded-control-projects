/**
 * @file serial.h
 * @brief Serial Communication Header
 *
 * Defines functions for serial (UART-like) communication.
 * In this simulation, we print to stdout instead of real serial port.
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

/* ============================================================================
   FUNCTION DECLARATIONS
   ============================================================================ */

/**
 * @brief Initialize serial port (no-op in simulation)
 *
 * In a real embedded system, this would initialize UART driver.
 * In this simulation, it does nothing since we use stdout.
 */
void serial_init(void);

/**
 * @brief Transmit temperature value as formatted string
 *
 * Sends: "TEMP:XX.X°C\r\n"
 *
 * @param temperature Temperature value in °C (float)
 */
void serial_send_temperature(float temperature);

#endif /* SERIAL_H_INCLUDED */
