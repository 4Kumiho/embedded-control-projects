/**
 * @file main.c
 * @brief SimpleThermometer Firmware Main Loop
 *
 * Very simple firmware that continuously:
 * 1. Reads temperature from simulated sensor
 * 2. Formats as string
 * 3. Transmits over serial
 *
 * No complex scheduler, no PID controller - just a basic loop.
 *
 * Every line has a comment explaining what it does.
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include <stdio.h>      /* Include stdio for printf() */
#include <unistd.h>     /* Include unistd for usleep() (Linux/Windows) */
#include "sensor.h"     /* Include sensor.h to read temperature */
#include "serial.h"     /* Include serial.h to send data */

/* ============================================================================
   MAIN PROGRAM
   ============================================================================ */

/**
 * @brief Main entry point of SimpleThermometer firmware
 *
 * Program flow:
 * 1. Initialize sensor and serial
 * 2. Infinite loop:
 *    - Read temperature
 *    - Send over serial
 *    - Sleep 500ms
 *
 * @return Exit code (0 = success, 1 = error)
 */
int main(void)
{
    /* --- Startup message --- */

    /* Print startup message to console */
    printf("========================================\n");
    printf("SimpleThermometer v1.0\n");
    printf("Reading temperature every 500ms\n");
    printf("========================================\n");

    /* Flush stdout buffer to ensure message is printed immediately */
    fflush(stdout);

    /* --- Initialize hardware --- */

    /* Call sensor_init() to seed random number generator */
    sensor_init();

    /* Call serial_init() to initialize serial port (no-op in simulation) */
    serial_init();

    /* --- Main infinite loop --- */

    /* Loop forever (until program is interrupted or error occurs) */
    while (1) {
        /* Step 1: Read temperature from sensor */

        /* Call sensor_read_temperature() and store result in variable */
        float temperature = sensor_read_temperature();

        /* Step 2: Send temperature over serial */

        /* Call serial_send_temperature() to format and transmit */
        serial_send_temperature(temperature);

        /* Step 3: Sleep for 500ms before next reading */

        /* usleep() sleeps for specified microseconds */
        /* 500ms = 500,000 microseconds */
        usleep(500000);

        /* Loop continues back to Step 1 */
    }

    /* This line is never reached (infinite loop above) */
    return 0;
}
