/**
 * @file main.c
 * @brief ThermoControl firmware main entry point
 *
 * This is the heart of the firmware simulation.
 * It ties together all modules and implements the main control loop.
 *
 * Architecture overview:
 * ======================
 *
 * Main Loop:
 * 1. Query current time from platform timer
 * 2. Ask scheduler "which task is due?"
 * 3. Execute task:
 *    - TASK_SENSOR: Read temperature sensor
 *    - TASK_CONTROLLER: Run PID controller
 *    - TASK_TELEMETRY: Send telemetry packet
 * 4. Repeat
 *
 * Data flow:
 * ----------
 * [Sensor] --T_measured--> [PID Controller] --command--> [Actuator/Physics]
 *                               ^
 *                               |
 *                            setpoint (from GUI via telemetry)
 *
 * [PID Output] --telemetry--> [Ground Station]
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include <stdio.h>      /* printf, fprintf */
#include <stdlib.h>     /* EXIT_SUCCESS, EXIT_FAILURE */
#include <string.h>     /* memset */

/* Include all firmware modules */
#include "platform.h"
#include "sensors.h"
#include "pid_controller.h"
#include "telemetry.h"
#include "scheduler.h"

/* ============================================================================
   FIRMWARE STATE (GLOBALS)
   ============================================================================

   In real embedded firmware, these would be local static variables
   or in a struct. For simplicity, we use globals here.
 */

/** Sensor simulation state */
static sensor_state_t g_sensor;

/** PID controller state */
static pid_state_t g_pid;

/** Task scheduler */
static scheduler_t g_scheduler;

/** System timer */
static platform_timer_t g_timer;

/** Telemetry frame buffer (for transmission) */
static telemetry_frame_t g_tx_frame;

/** Statistics for monitoring */
static struct {
    uint32_t sensor_reads;
    uint32_t pid_updates;
    uint32_t telemetry_sends;
    uint32_t total_iterations;
} g_stats = {0};

/* ============================================================================
   FUNCTION DECLARATIONS (Task Implementations)
   ============================================================================ */

/**
 * Task handler: Read temperature sensor
 * Called every 100 ms
 */
static void task_sensor_read(uint32_t current_time_ms);

/**
 * Task handler: Update PID controller
 * Called every 100 ms
 */
static void task_controller_update(uint32_t current_time_ms);

/**
 * Task handler: Send telemetry packet
 * Called every 100 ms
 */
static void task_telemetry_send(uint32_t current_time_ms);

/**
 * Print debug statistics
 */
static void print_stats(void);

/* ============================================================================
   MAIN ENTRY POINT
   ============================================================================ */

/**
 * int main(void)
 *
 * Firmware entry point.
 *
 * Initialization sequence:
 * 1. Initialize platform (timers, random seed)
 * 2. Initialize sensor simulation
 * 3. Initialize PID controller
 * 4. Initialize scheduler
 * 5. Register periodic tasks
 * 6. Enter main control loop
 *
 * The main loop:
 * - Queries current time
 * - Asks scheduler which task is due
 * - Executes task function
 * - Repeats ~10,000+ times per second
 *
 * Runs indefinitely (in real firmware, would run until power off).
 * For simulation, we run for ~30 seconds then exit.
 *
 * @return EXIT_SUCCESS on normal exit, EXIT_FAILURE on error
 */
int main(void)
{
    printf("=== ThermoControl Firmware v1.0 ===\n");

    /* ========== INITIALIZATION ========== */

    printf("Initializing platform...\n");
    if (platform_init() != 0) {
        fprintf(stderr, "ERROR: Platform init failed\n");
        return EXIT_FAILURE;
    }

    printf("Initializing sensor...\n");
    if (sensor_init(&g_sensor) != 0) {
        fprintf(stderr, "ERROR: Sensor init failed\n");
        return EXIT_FAILURE;
    }

    printf("Initializing PID controller...\n");
    if (pid_init(&g_pid, 25.0f, 0) != 0) {
        fprintf(stderr, "ERROR: PID init failed\n");
        return EXIT_FAILURE;
    }

    printf("Initializing scheduler...\n");
    if (scheduler_init(&g_scheduler, 0) != 0) {
        fprintf(stderr, "ERROR: Scheduler init failed\n");
        return EXIT_FAILURE;
    }

    printf("Registering tasks...\n");
    if (scheduler_register(&g_scheduler, TASK_SENSOR, 100) != 0 ||
        scheduler_register(&g_scheduler, TASK_CONTROLLER, 100) != 0 ||
        scheduler_register(&g_scheduler, TASK_TELEMETRY, 100) != 0) {
        fprintf(stderr, "ERROR: Task registration failed\n");
        return EXIT_FAILURE;
    }

    printf("Initializing telemetry...\n");
    if (telemetry_init() != 0) {
        fprintf(stderr, "ERROR: Telemetry init failed\n");
        return EXIT_FAILURE;
    }

    printf("\n=== Initialization complete ===\n");
    printf("Starting main control loop...\n");
    printf("(Press Ctrl+C to stop)\n\n");

    /* Start system timer */
    platform_timer_start(&g_timer);

    /* ========== MAIN CONTROL LOOP ========== */

    /* Run for approximately 30 seconds (demo duration)
       In real firmware, this would be "while(1)" forever */
    uint32_t sim_duration_ms = 30000;
    uint32_t log_interval = 0;

    while (1) {
        /* Get current time */
        uint32_t now = platform_timer_elapsed_ms(&g_timer);

        /* Safety: stop after 30 seconds of simulation */
        if (now > sim_duration_ms) {
            printf("\n=== Simulation end (30 sec reached) ===\n");
            break;
        }

        /* Ask scheduler which task is due */
        task_id_t task = scheduler_update(&g_scheduler, now);

        /* Execute task based on scheduler return value */
        switch (task) {
            case TASK_SENSOR:
                task_sensor_read(now);
                break;

            case TASK_CONTROLLER:
                task_controller_update(now);
                break;

            case TASK_TELEMETRY:
                task_telemetry_send(now);
                break;

            case TASK_COUNT:
                /* No task due - could sleep here to save CPU
                   For simulation, we just continue
                */
                break;

            default:
                fprintf(stderr, "ERROR: Invalid task ID\n");
                return EXIT_FAILURE;
        }

        /* Periodic logging (every 5 seconds) */
        if (now >= log_interval) {
            printf("Time: %5u ms | T=%.2f°C | Tset=%.2f°C | err=%.2f°C | cmd=%.1f%% | iter=%u\n",
                   now,
                   sensor_get_true_temp(&g_sensor),
                   g_pid.setpoint,
                   pid_get_error(&g_pid),
                   pid_get_output(&g_pid),
                   g_stats.total_iterations);
            log_interval += 5000;
        }

        g_stats.total_iterations++;

        /* Small sleep to prevent 100% CPU (simulation quality of life) */
        if (task == TASK_COUNT) {
            platform_sleep_ms(1);
        }
    }

    /* ========== SHUTDOWN ========== */

    print_stats();

    printf("\n=== Firmware shutdown ===\n");
    return EXIT_SUCCESS;
}

/* ============================================================================
   TASK IMPLEMENTATIONS
   ============================================================================ */

/**
 * void task_sensor_read(uint32_t current_time_ms)
 *
 * Task: Read temperature sensor
 * Frequency: 100 ms (10 Hz)
 *
 * What happens:
 * 1. Read simulated temperature sensor
 * 2. Log reading
 * 3. (In real firmware: queue sample for PID, store for telemetry)
 *
 * Note: We don't explicitly pass reading to PID here.
 * Instead, PID reads it on demand (next task).
 * This is fine for simulation; real RTOS would use message queues.
 */
static void task_sensor_read(uint32_t current_time_ms)
{
    sensor_reading_t reading;

    /* Read sensor (simulated physics response + noise) */
    if (sensor_read(&g_sensor, current_time_ms, &reading) != 0) {
        fprintf(stderr, "ERROR: Sensor read failed\n");
        return;
    }

    g_stats.sensor_reads++;

    /* Detailed logging (commented out to reduce spam)
       Uncomment to see every sensor reading:
    printf("[%u ms] SENSOR: T=%.3f°C\n", current_time_ms, reading.temperature_celsius);
    */
}

/**
 * void task_controller_update(uint32_t current_time_ms)
 *
 * Task: Update PID controller
 * Frequency: 100 ms (10 Hz)
 *
 * What happens:
 * 1. Get latest temperature measurement
 * 2. Run PID algorithm
 * 3. Get heater command output
 * 4. Apply command to sensor (closes feedback loop)
 *
 * Timing note:
 * - In real systems, sensor reading would be timestamped
 * - We assume latest reading is available (works because same frequency)
 * - Production systems would use circular buffers / message passing
 */
static void task_controller_update(uint32_t current_time_ms)
{
    /* Read latest temperature (from sensor module state) */
    float current_temperature = sensor_get_true_temp(&g_sensor);

    /* Run PID algorithm
       Calculates output based on:
       - Current measurement
       - Setpoint (configured via pid_set_setpoint)
       - PID gains
       - Time since last update
    */
    float heater_command = pid_update(&g_pid, current_temperature, current_time_ms);

    /* Send heater command to actuator/physics simulation
       This closes the feedback loop:
       Sensor reading → PID → Heater → Physics → new Temperature
    */
    sensor_set_command(&g_sensor, heater_command);

    g_stats.pid_updates++;

    /* Detailed logging (commented out to reduce spam)
    printf("[%u ms] PID: cmd=%.1f%%\n", current_time_ms, heater_command);
    */
}

/**
 * void task_telemetry_send(uint32_t current_time_ms)
 *
 * Task: Send telemetry packet to ground station
 * Frequency: 100 ms (10 Hz)
 *
 * What happens:
 * 1. Gather current system state
 * 2. Pack into telemetry packet
 * 3. Encode into binary frame
 * 4. (In real: send to GUI via socket)
 * 5. (In simulation: just show stats)
 *
 * Telemetry packet contains:
 * - Current temperature
 * - Setpoint
 * - Error
 * - Heater command
 */
static void task_telemetry_send(uint32_t current_time_ms)
{
    /* Gather current system state into packet */
    telemetry_packet_t packet;
    packet.temperature = sensor_get_true_temp(&g_sensor);
    packet.setpoint = g_pid.setpoint;
    packet.error = pid_get_error(&g_pid);
    packet.command = pid_get_output(&g_pid);
    packet.timestamp = current_time_ms;

    /* Encode packet into binary frame */
    if (telemetry_encode(&packet, &g_tx_frame) != 0) {
        fprintf(stderr, "ERROR: Telemetry encode failed\n");
        return;
    }

    g_stats.telemetry_sends++;

    /* In simulation, we don't actually send anywhere.
       In real system, would do:
       - write(socket, g_tx_frame.data, g_tx_frame.length);
       or
       - uart_write(uart_handle, g_tx_frame.data, g_tx_frame.length);

       For now, just update counter.
    */

    /* Detailed logging (commented out to reduce spam)
    printf("[%u ms] TELEMETRY: %u bytes sent\n",
           current_time_ms, (unsigned)g_tx_frame.length);
    */
}

/**
 * void print_stats(void)
 *
 * Print run statistics.
 * Called on firmware shutdown.
 */
static void print_stats(void)
{
    uint32_t total_time = platform_timer_elapsed_ms(&g_timer);

    printf("\n=== Firmware Statistics ===\n");
    printf("Total runtime: %u ms\n", total_time);
    printf("Sensor reads: %u (%.1f Hz)\n",
           g_stats.sensor_reads,
           (float)g_stats.sensor_reads * 1000.0f / total_time);
    printf("PID updates: %u (%.1f Hz)\n",
           g_stats.pid_updates,
           (float)g_stats.pid_updates * 1000.0f / total_time);
    printf("Telemetry sends: %u (%.1f Hz)\n",
           g_stats.telemetry_sends,
           (float)g_stats.telemetry_sends * 1000.0f / total_time);
    printf("Main loop iterations: %u (%.0f Hz)\n",
           g_stats.total_iterations,
           (float)g_stats.total_iterations * 1000.0f / total_time);
    printf("Average loop time: %.1f µs\n",
           (float)total_time * 1000.0f / g_stats.total_iterations);
    printf("============================\n");
}
