/**
 * @file platform.h
 * @brief Platform abstraction layer for cross-platform timing and sleep functions
 *
 * This module provides OS-independent abstractions for:
 * - Timer initialization and measurement
 * - Sleep/delay operations
 * - System time queries
 *
 * Why abstraction layer?
 * - Windows uses Sleep(ms), Linux uses usleep(us)
 * - Timer counters are different (LARGE_INTEGER vs clock_gettime)
 * - Easier to port to embedded MCU later (just replace this file!)
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>  /* uint32_t, uint64_t */
#include <time.h>    /* time_t for standard C */

/**
 * @defgroup Platform Platform Abstraction
 * @{
 */

/* ============================================================================
   TIMER TYPE DEFINITIONS
   ============================================================================

   Purpose: Store timer state in a way that's hidden from the user.
   This is an "opaque" type pattern - user doesn't need to know internal details.
 */

/**
 * @struct platform_timer_t
 * @brief Opaque timer handle for measuring elapsed time
 *
 * Usage:
 *   platform_timer_t timer;
 *   platform_timer_start(&timer);
 *   // ... do some work ...
 *   uint32_t elapsed_ms = platform_timer_elapsed_ms(&timer);
 */
typedef struct {
    uint64_t start_ticks;  ///< Internal tick count when timer started
    uint64_t frequency;    ///< System clock frequency (for Windows LARGE_INTEGER)
} platform_timer_t;

/* ============================================================================
   FUNCTION DECLARATIONS
   ============================================================================

   Note: All functions are declared here, implemented in platform.c
   The actual implementation differs between Windows and Linux.
 */

/**
 * @brief Initialize the platform timer system
 *
 * Must be called once at program startup before using any timer functions.
 *
 * - On Windows: Queries high-resolution counter frequency
 * - On Linux: Just returns (clock_gettime doesn't need init)
 *
 * @return 0 on success, -1 on failure
 *
 * @example
 *   if (platform_init() != 0) {
 *       printf("Failed to init platform\n");
 *       return 1;
 *   }
 */
int platform_init(void);

/**
 * @brief Start a timer for measuring elapsed time
 *
 * Records the current system time in the timer structure.
 * Use platform_timer_elapsed_ms() later to see how much time passed.
 *
 * Multiple timers can run independently:
 *   platform_timer_t timer1, timer2;
 *   platform_timer_start(&timer1);
 *   platform_timer_start(&timer2);
 *   // timer1 and timer2 track different intervals
 *
 * @param timer Pointer to timer structure to initialize
 *
 * @example
 *   platform_timer_t my_timer;
 *   platform_timer_start(&my_timer);
 */
void platform_timer_start(platform_timer_t *timer);

/**
 * @brief Get elapsed time in milliseconds since platform_timer_start() was called
 *
 * - Can be called multiple times; each call recalculates from start time
 * - Does NOT reset the timer (use platform_timer_start() to reset)
 * - Returns 0 if less than 1 ms has passed
 *
 * Precision:
 * - Windows: microsecond-level (converted to ms)
 * - Linux: nanosecond-level via clock_gettime(CLOCK_MONOTONIC)
 *
 * @param timer Pointer to timer that was started with platform_timer_start()
 * @return Elapsed milliseconds since timer_start was called (uint32_t)
 *
 * @example
 *   platform_timer_t timer;
 *   platform_timer_start(&timer);
 *   // ... wait 500 ms ...
 *   uint32_t elapsed = platform_timer_elapsed_ms(&timer);
 *   printf("Elapsed: %u ms\n", elapsed);  // Output: ~500
 */
uint32_t platform_timer_elapsed_ms(const platform_timer_t *timer);

/**
 * @brief Sleep for a specified number of milliseconds
 *
 * Blocks the current thread for approximately the given duration.
 * The actual duration may be slightly longer due to OS scheduling.
 *
 * Platform-specific implementation:
 * - Windows: Uses Sleep(milliseconds) from <windows.h>
 * - Linux/Unix: Uses nanosleep() from <time.h>
 *
 * NOT for high-precision timing! For that, use timers.
 * For example, don't do:
 *   while(1) {
 *       platform_sleep_ms(10);  // ❌ Accumulates error over time
 *       do_task();
 *   }
 *
 * Instead, use a scheduler (see scheduler.h) that checks elapsed time:
 *   platform_timer_t timer;
 *   platform_timer_start(&timer);
 *   while(1) {
 *       if (platform_timer_elapsed_ms(&timer) >= 10) {
 *           do_task();
 *           platform_timer_start(&timer);  // Reset for next cycle
 *       }
 *   }
 *
 * @param milliseconds Time to sleep in milliseconds (uint32_t)
 *
 * @example
 *   printf("Before sleep\n");
 *   platform_sleep_ms(1000);  // Sleep 1 second
 *   printf("After sleep\n");
 */
void platform_sleep_ms(uint32_t milliseconds);

/**
 * @brief Get current system time as a Unix timestamp
 *
 * Returns seconds since 1970-01-01 00:00:00 UTC (Unix epoch).
 * Useful for timestamping telemetry data.
 *
 * Not suitable for measuring intervals (use timers instead),
 * but good for logging absolute times in data:
 *   printf("Data logged at timestamp: %ld\n", platform_get_time_unix());
 *
 * @return Time in seconds since Unix epoch (time_t, typically int32_t or int64_t)
 *
 * @example
 *   time_t now = platform_get_time_unix();
 *   printf("Current time: %ld\n", now);
 */
time_t platform_get_time_unix(void);

/**
 * @} End of Platform group
 */

#endif /* PLATFORM_H */
