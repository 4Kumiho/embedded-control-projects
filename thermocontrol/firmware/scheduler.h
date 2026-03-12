/**
 * @file scheduler.h
 * @brief Simple task scheduler for timing-critical operations
 *
 * Scheduler for embedded systems: manages periodic tasks with different
 * frequencies in a cooperative (non-preemptive) way.
 *
 * Use case:
 * - Sensor reading: 100 ms period (10 Hz)
 * - PID update: 100 ms period (same as sensor)
 * - Telemetry transmission: 100 ms period (same)
 * - All run in a single "main loop" without OS threads
 *
 * Why not use threads?
 * - Firmware on bare metal (MCU) typically doesn't have threads
 * - Even on PC, single-threaded keeps things simple
 * - Cooperative scheduling: tasks yield control to each other
 * - Lower overhead than preemptive kernel
 *
 * How it works:
 * =============
 * 1. Register tasks with their period
 * 2. In main loop, call scheduler_update() repeatedly
 * 3. Scheduler checks if each task's period has elapsed
 * 4. If yes, returns task ID to run that task
 * 5. Application calls task function
 * 6. Loop continues
 *
 * Time-based vs. tick-based:
 * ==========================
 * Tick-based (old RTOS way):
 * - Timer interrupt increments tick counter
 * - Tasks run when tick % period == 0
 * - Pro: Predictable, independent of execution time
 * - Con: Requires interrupt handling
 *
 * Time-based (our approach):
 * - Query actual elapsed time (from platform.h timers)
 * - Tasks run when elapsed >= period
 * - Pro: Simple, no interrupts needed
 * - Con: Timing depends on query frequency
 *
 * Our implementation uses time-based (simpler for PC simulation).
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>    /* uint32_t */
#include <stdbool.h>   /* bool */

/**
 * @defgroup Scheduler Task Scheduler
 * @{
 */

/* ============================================================================
   CONFIGURATION
   ============================================================================ */

/** Maximum number of periodic tasks we can register
    Typical firmware: 3-10 tasks. We allow up to 10.
 */
#define SCHEDULER_MAX_TASKS 10

/** Task ID values */
typedef enum {
    TASK_SENSOR    = 0,      ///< Read temperature sensor (100 ms)
    TASK_CONTROLLER = 1,     ///< Run PID controller (100 ms)
    TASK_TELEMETRY = 2,      ///< Send telemetry packet (100 ms)
    TASK_COUNT = 3            ///< Number of tasks (used for validation)
} task_id_t;

/* ============================================================================
   DATA STRUCTURES
   ============================================================================ */

/**
 * @struct scheduler_task_t
 * @brief Represents a single periodic task
 *
 * Internal structure - don't modify directly.
 * Holds timing information for one task.
 */
typedef struct {
    uint32_t period_ms;         ///< How often to run (milliseconds)
    uint32_t last_run_time;     ///< Last time this task was executed (ms)
    bool enabled;               ///< Is this task active?
} scheduler_task_t;

/**
 * @struct scheduler_t
 * @brief Main scheduler state
 *
 * Opaque structure - use scheduler_init(), scheduler_register(), etc.
 * instead of accessing fields directly.
 */
typedef struct {
    scheduler_task_t tasks[SCHEDULER_MAX_TASKS];  ///< Array of registered tasks
    uint32_t num_tasks;                            ///< Number of registered tasks
    uint32_t reference_time;                       ///< Reference point for time calculation
} scheduler_t;

/* ============================================================================
   FUNCTION DECLARATIONS
   ============================================================================ */

/**
 * @brief Initialize the scheduler
 *
 * Must be called once at program startup before registering tasks.
 *
 * Sets up internal state:
 * - Clears task array
 * - Sets reference time to 0
 *
 * @param scheduler Pointer to scheduler_t to initialize
 * @param start_time_ms Initial reference time (usually 0 or current time)
 * @return 0 on success, -1 on error
 *
 * @example
 *   scheduler_t sched;
 *   if (scheduler_init(&sched, 0) != 0) {
 *       printf("Scheduler init failed\n");
 *       return 1;
 *   }
 */
int scheduler_init(scheduler_t *scheduler, uint32_t start_time_ms);

/**
 * @brief Register a periodic task
 *
 * Adds a task to the scheduler's task list.
 * Later, scheduler_update() will check when this task should run.
 *
 * Typical registration:
 *   scheduler_register(&sched, TASK_SENSOR, 100);    // Every 100 ms
 *   scheduler_register(&sched, TASK_CONTROLLER, 100);
 *   scheduler_register(&sched, TASK_TELEMETRY, 100);
 *
 * @param scheduler Pointer to initialized scheduler_t
 * @param task_id Task identifier (TASK_SENSOR, TASK_CONTROLLER, etc.)
 * @param period_ms Task period in milliseconds (how often to run)
 * @return 0 on success, -1 if too many tasks or invalid task_id
 *
 * @example
 *   // Register sensor task to run every 100 ms
 *   if (scheduler_register(&sched, TASK_SENSOR, 100) != 0) {
 *       printf("Failed to register task\n");
 *       return 1;
 *   }
 */
int scheduler_register(scheduler_t *scheduler, task_id_t task_id,
                       uint32_t period_ms);

/**
 * @brief Update scheduler and check if any task is due
 *
 * Main scheduler loop function. Call this repeatedly in your main loop.
 *
 * Algorithm:
 * 1. For each registered task:
 *    - Check if elapsed time since last run >= task period
 *    - If yes, mark as "due" and update last run time
 * 2. Return the first due task (or TASK_COUNT if none)
 *
 * Usage pattern:
 *   while (1) {
 *       uint32_t now = platform_timer_elapsed_ms(&timer);
 *       task_id_t task = scheduler_update(&sched, now);
 *
 *       switch (task) {
 *           case TASK_SENSOR:
 *               sensor_read(...);
 *               break;
 *           case TASK_CONTROLLER:
 *               pid_update(...);
 *               break;
 *           case TASK_TELEMETRY:
 *               telemetry_send(...);
 *               break;
 *           case TASK_COUNT:
 *               // No task due, can sleep or process other things
 *               break;
 *       }
 *   }
 *
 * Note on timing:
 * - If multiple tasks are due, we return one at a time
 * - Call scheduler_update() frequently (e.g., every 1-10 ms) for accuracy
 * - If you call once per 1000 ms, tasks will be late
 *
 * @param scheduler Pointer to scheduler_t
 * @param current_time_ms Current system time (from platform.h)
 * @return Task ID if a task is due, TASK_COUNT if no task due
 *
 * @example
 *   platform_timer_t timer;
 *   platform_timer_start(&timer);
 *
 *   while (1) {
 *       uint32_t now = platform_timer_elapsed_ms(&timer);
 *       task_id_t task = scheduler_update(&sched, now);
 *
 *       if (task == TASK_SENSOR) {
 *           // Time to read sensor
 *       } else if (task != TASK_COUNT) {
 *           // Other task due
 *       } else {
 *           // No task due, can sleep briefly
 *           platform_sleep_ms(1);
 *       }
 *   }
 */
task_id_t scheduler_update(scheduler_t *scheduler, uint32_t current_time_ms);

/**
 * @brief Enable/disable a task
 *
 * Allows turning tasks on/off at runtime without unregistering them.
 *
 * @param scheduler Pointer to scheduler_t
 * @param task_id Task to enable/disable
 * @param enabled true to enable, false to disable
 *
 * @example
 *   // Stop reading sensor temporarily
 *   scheduler_set_enabled(&sched, TASK_SENSOR, false);
 *   // Resume later
 *   scheduler_set_enabled(&sched, TASK_SENSOR, true);
 */
void scheduler_set_enabled(scheduler_t *scheduler, task_id_t task_id,
                           bool enabled);

/**
 * @brief Get period of a task
 *
 * Returns how often a task is scheduled to run.
 *
 * @param scheduler Pointer to scheduler_t
 * @param task_id Task identifier
 * @return Period in milliseconds, or 0 if task not found
 *
 * @example
 *   uint32_t sensor_period = scheduler_get_period(&sched, TASK_SENSOR);
 *   printf("Sensor runs every %u ms\n", sensor_period);
 */
uint32_t scheduler_get_period(const scheduler_t *scheduler, task_id_t task_id);

/**
 * @brief Get time until a task is due
 *
 * Calculates how long until a task will next execute.
 * Useful for adaptive sleeping or priority scheduling.
 *
 * @param scheduler Pointer to scheduler_t
 * @param task_id Task identifier
 * @param current_time_ms Current system time
 * @return Milliseconds until task is due (0 if due now, -1 if not found)
 *
 * @example
 *   int32_t until_sensor = scheduler_time_until_ready(&sched, TASK_SENSOR, now);
 *   if (until_sensor > 50) {
 *       // Safe to sleep 50 ms
 *       platform_sleep_ms(50);
 *   }
 */
int32_t scheduler_time_until_ready(const scheduler_t *scheduler,
                                    task_id_t task_id, uint32_t current_time_ms);

/**
 * @brief Get statistics about scheduler
 *
 * For debugging/monitoring.
 *
 * @param scheduler Pointer to scheduler_t
 * @return Number of registered tasks
 *
 * @example
 *   uint32_t count = scheduler_get_task_count(&sched);
 *   printf("Scheduler has %u tasks\n", count);
 */
uint32_t scheduler_get_task_count(const scheduler_t *scheduler);

/**
 * @} End of Scheduler group
 */

#endif /* SCHEDULER_H */
