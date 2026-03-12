/**
 * @file scheduler.c
 * @brief Task scheduler implementation
 *
 * Simple time-based scheduler for embedded firmware.
 * Manages periodic tasks without threads or interrupts.
 *
 * Design philosophy:
 * - Simplicity over features
 * - Predictable timing (query current time, check periods)
 * - Cooperative (tasks yield to each other)
 * - No dynamic memory allocation
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include "scheduler.h"
#include <string.h>    /* memset() */

/* ============================================================================
   INITIALIZATION
   ============================================================================ */

/**
 * int scheduler_init(scheduler_t *scheduler, uint32_t start_time_ms)
 *
 * Initialize scheduler to empty state.
 *
 * Setup:
 * 1. Zero-initialize all tasks
 * 2. Set reference time (all tasks start relative to this)
 * 3. Mark as ready for use
 *
 * @param scheduler Pointer to uninitialized scheduler_t
 * @param start_time_ms Initial time reference (usually 0)
 * @return 0 on success, -1 on error
 */
int scheduler_init(scheduler_t *scheduler, uint32_t start_time_ms)
{
    if (!scheduler) {
        return -1;
    }

    /* Zero-initialize entire structure */
    memset(scheduler, 0, sizeof(scheduler_t));

    /* Set reference time */
    scheduler->reference_time = start_time_ms;

    /* num_tasks is 0 initially (no tasks registered yet)
       All task slots are zero-initialized (not enabled) */

    return 0;
}

/* ============================================================================
   TASK REGISTRATION
   ============================================================================ */

/**
 * int scheduler_register(scheduler_t *scheduler, task_id_t task_id,
 *                        uint32_t period_ms)
 *
 * Register a periodic task.
 *
 * Algorithm:
 * 1. Validate inputs
 * 2. Check if we have room for more tasks
 * 3. Create task entry with given period
 * 4. Enable task
 * 5. Set last_run_time to current reference (task will run immediately)
 *
 * Why set last_run_time to reference?
 * - Task will be "due" on first scheduler_update()
 * - No artificial delay before first execution
 *
 * @param scheduler Pointer to initialized scheduler_t
 * @param task_id Task identifier (TASK_SENSOR, TASK_CONTROLLER, etc.)
 * @param period_ms Task period in milliseconds
 * @return 0 on success, -1 on error
 */
int scheduler_register(scheduler_t *scheduler, task_id_t task_id,
                       uint32_t period_ms)
{
    if (!scheduler || task_id < 0 || task_id >= TASK_COUNT) {
        return -1;
    }

    if (scheduler->num_tasks >= SCHEDULER_MAX_TASKS) {
        return -1;  /* No more room */
    }

    /* Create task entry
       Use task_id as index in tasks array for easy lookup */
    scheduler_task_t *task = &scheduler->tasks[task_id];

    task->period_ms = period_ms;
    task->last_run_time = scheduler->reference_time;  /* Ready to run immediately */
    task->enabled = true;

    /* Update count if this is a new task (not overwriting) */
    if (task_id >= scheduler->num_tasks) {
        scheduler->num_tasks = task_id + 1;
    }

    return 0;
}

/* ============================================================================
   SCHEDULER UPDATE
   ============================================================================

   Main scheduler logic: checks which tasks are due.
 */

/**
 * task_id_t scheduler_update(scheduler_t *scheduler, uint32_t current_time_ms)
 *
 * Check which task (if any) is due to run.
 *
 * Algorithm:
 * 1. For each registered task:
 *    - Check if enabled (skip disabled tasks)
 *    - Calculate time elapsed since last run: elapsed = current_time - last_run_time
 *    - If elapsed >= period, task is due:
 *      * Update last_run_time to current time
 *      * Return this task ID
 * 2. If no task is due, return TASK_COUNT (sentinel value)
 *
 * Design notes:
 * - Returns first due task only (caller must call again for next task)
 * - This allows other code to run between tasks (good for responsiveness)
 * - Alternatively, could return all due tasks, but loops are simpler
 *
 * Timing accuracy:
 * - If called frequently (every 1-10 ms), timing is accurate
 * - If called rarely (every 1000 ms), tasks will be late
 * - Real embedded systems: interrupt-driven (timer fires periodically)
 * - Our simulation: polling (call scheduler_update as fast as possible)
 *
 * @param scheduler Pointer to initialized scheduler_t
 * @param current_time_ms Current system time (from platform.h timer)
 * @return Task ID if task is due, TASK_COUNT if no task due
 */
task_id_t scheduler_update(scheduler_t *scheduler, uint32_t current_time_ms)
{
    if (!scheduler) {
        return TASK_COUNT;
    }

    /* Check each task in order */
    for (uint32_t i = 0; i < scheduler->num_tasks; i++) {
        scheduler_task_t *task = &scheduler->tasks[i];

        /* Skip disabled tasks */
        if (!task->enabled) {
            continue;
        }

        /* Skip tasks with zero period (not configured) */
        if (task->period_ms == 0) {
            continue;
        }

        /* Calculate elapsed time since last run
           Note: This assumes current_time_ms doesn't wrap around,
           which is valid for uint32_t up to ~49 days of uptime.
           In production, you'd handle wraparound explicitly.
        */
        uint32_t elapsed = current_time_ms - task->last_run_time;

        /* Check if task is due (elapsed time >= period) */
        if (elapsed >= task->period_ms) {
            /* Update last run time
               Why current_time_ms (not last_run_time + period)?
               - If we're late, next task runs at actual time, not ideal time
               - Prevents "bunching up" of delayed tasks
               - Alternative: last_run_time += period (maintains strict timing)
               - We choose current for stability over precision
            */
            task->last_run_time = current_time_ms;

            /* Return task ID */
            return (task_id_t)i;
        }
    }

    /* No task is due */
    return TASK_COUNT;
}

/* ============================================================================
   TASK CONTROL
   ============================================================================ */

/**
 * void scheduler_set_enabled(scheduler_t *scheduler, task_id_t task_id,
 *                            bool enabled)
 *
 * Enable or disable a task at runtime.
 *
 * Disabled tasks:
 * - Are skipped in scheduler_update()
 * - Retain their period and last_run_time
 * - Can be re-enabled later
 *
 * @param scheduler Pointer to scheduler_t
 * @param task_id Task to enable/disable
 * @param enabled true to enable, false to disable
 */
void scheduler_set_enabled(scheduler_t *scheduler, task_id_t task_id,
                           bool enabled)
{
    if (!scheduler || task_id < 0 || task_id >= scheduler->num_tasks) {
        return;
    }

    scheduler->tasks[task_id].enabled = enabled;
}

/* ============================================================================
   TASK QUERIES
   ============================================================================ */

/**
 * uint32_t scheduler_get_period(const scheduler_t *scheduler, task_id_t task_id)
 *
 * Get the period of a task.
 *
 * @param scheduler Pointer to scheduler_t
 * @param task_id Task identifier
 * @return Period in milliseconds, 0 if not found
 */
uint32_t scheduler_get_period(const scheduler_t *scheduler, task_id_t task_id)
{
    if (!scheduler || task_id < 0 || task_id >= scheduler->num_tasks) {
        return 0;
    }

    return scheduler->tasks[task_id].period_ms;
}

/**
 * int32_t scheduler_time_until_ready(const scheduler_t *scheduler,
 *                                     task_id_t task_id,
 *                                     uint32_t current_time_ms)
 *
 * Calculate time until a task will next execute.
 *
 * Formula:
 *   elapsed = current_time - last_run_time
 *   time_until = period - elapsed
 *   (clamp to 0 if elapsed >= period)
 *
 * Use case:
 *   // Sleep until next task is due
 *   int32_t sleep_time = scheduler_time_until_ready(&sched, TASK_SENSOR, now);
 *   if (sleep_time > 10) {
 *       platform_sleep_ms(sleep_time - 10);  // Leave 10 ms buffer
 *   }
 *
 * @param scheduler Pointer to scheduler_t
 * @param task_id Task identifier
 * @param current_time_ms Current system time
 * @return Milliseconds until task is due (0 if due now, -1 if error)
 */
int32_t scheduler_time_until_ready(const scheduler_t *scheduler,
                                    task_id_t task_id, uint32_t current_time_ms)
{
    if (!scheduler || task_id < 0 || task_id >= scheduler->num_tasks) {
        return -1;
    }

    const scheduler_task_t *task = &scheduler->tasks[task_id];

    if (task->period_ms == 0) {
        return -1;  /* Task not configured */
    }

    uint32_t elapsed = current_time_ms - task->last_run_time;

    if (elapsed >= task->period_ms) {
        return 0;  /* Task is due now */
    }

    /* Return time remaining */
    return (int32_t)(task->period_ms - elapsed);
}

/**
 * uint32_t scheduler_get_task_count(const scheduler_t *scheduler)
 *
 * Get number of registered tasks.
 *
 * @param scheduler Pointer to scheduler_t
 * @return Number of tasks
 */
uint32_t scheduler_get_task_count(const scheduler_t *scheduler)
{
    if (!scheduler) {
        return 0;
    }

    return scheduler->num_tasks;
}
