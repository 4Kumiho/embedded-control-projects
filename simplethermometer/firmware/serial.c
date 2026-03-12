/**
 * @file serial.c
 * @brief Serial Communication Implementation
 *
 * Implements UART-like serial transmission.
 * In this simulation, we print formatted strings to stdout.
 * In real embedded system, this would use actual UART driver.
 *
 * Every line of code is explained.
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include "serial.h"     /* Include serial.h header file */
#include <stdio.h>      /* Include stdio for printf() function */

/* ============================================================================
   SERIAL INITIALIZATION
   ============================================================================ */

/**
 * @brief Initialize serial port
 *
 * In a real embedded system (STM32, Arduino), this would configure:
 * - Baud rate to 115200
 * - 8 data bits, 1 stop bit
 * - No parity
 *
 * In this simulation, we use stdout, so initialization is a no-op.
 */
void serial_init(void)
{
    /* In a real system, we would configure UART driver here */
    /* For simulation, stdout is already ready, so we do nothing */
}

/* ============================================================================
   SERIAL TRANSMISSION
   ============================================================================ */

/**
 * @brief Send temperature value as formatted string
 *
 * Formats temperature as: "TEMP:XX.X°C\r\n"
 * Example output: "TEMP:22.3°C\r\n"
 *
 * @param temperature Temperature value in degrees Celsius
 */
void serial_send_temperature(float temperature)
{
    /* --- Step 1: Format the temperature string --- */

    /* Use printf to format temperature with 1 decimal place */
    /* \r\n is carriage return + line feed (standard serial line ending) */
    printf("TEMP:%.1f°C\r\n", temperature);

    /* --- Step 2: In a real system, we would also flush the buffer --- */

    /* fflush() forces stdout to immediately write to output */
    /* Without it, output might be buffered and delayed */
    fflush(stdout);

    /* In a real embedded system using UART:
       - We would write temperature bytes to UART transmit register
       - Hardware would transmit each byte at configured baud rate
       - We would wait for transmit complete interrupt
       For simulation, printf handles all of this for us
    */
}
