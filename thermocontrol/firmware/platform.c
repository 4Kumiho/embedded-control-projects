/**
 * @file platform.c
 * @brief Platform abstraction layer implementation
 *
 * Provides cross-platform timer and sleep functionality for Windows and Linux.
 * The "trick" is using preprocessor directives (#ifdef _WIN32) to compile
 * different code on different platforms.
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include "platform.h"
#include <time.h>   /* time() for Unix timestamp */

/* ============================================================================
   PLATFORM DETECTION
   ============================================================================

   These preprocessor macros are automatically defined by the compiler:
   - _WIN32: Defined on Windows (32-bit and 64-bit)
   - __linux__: Defined on Linux
   - __unix__: Defined on Unix-like systems

   We use #ifdef to conditionally compile different code blocks.
 */

#ifdef _WIN32
    /* Windows-specific includes */
    #include <windows.h>        /* Sleep(), QueryPerformanceCounter() */
    #include <profileapi.h>     /* Performance counter API */

#elif defined(__unix__) || defined(__linux__)
    /* Unix/Linux-specific includes */
    #include <unistd.h>         /* usleep() */
    #include <sys/time.h>       /* gettimeofday() - high-resolution timer */
    #include <errno.h>          /* Error handling for nanosleep */

#else
    #error "Unsupported platform - define Windows or Unix/Linux support"
#endif

/* ============================================================================
   GLOBAL STATE
   ============================================================================

   These variables store platform-specific initialization data.
   We keep them static (file-scope) so they're hidden from the outside world.
 */

/**
 * Stores the high-resolution timer frequency.
 * Only used on Windows; on Linux this is hardcoded.
 */
static uint64_t g_timer_frequency = 0;

/**
 * Flag to track whether platform_init() has been called.
 * Prevents undefined behavior if timer functions are used before init.
 */
static int g_platform_initialized = 0;

/* ============================================================================
   PLATFORM INITIALIZATION
   ============================================================================ */

/**
 * int platform_init(void)
 *
 * Platform initialization:
 * - Windows: Query the high-resolution counter frequency
 * - Linux: Just mark as initialized (clock_gettime is always available)
 *
 * Why this is needed:
 * - Windows' QueryPerformanceCounter() returns "ticks" in an arbitrary unit
 * - To convert ticks to time, we need to know ticks-per-second
 * - QueryPerformanceFrequency() gives us that conversion factor
 *
 * Return: 0 on success, -1 on failure
 */
int platform_init(void)
{
#ifdef _WIN32
    /* ====== WINDOWS IMPLEMENTATION ====== */

    /* QueryPerformanceFrequency() fills in a LARGE_INTEGER with
       the number of counts per second for high-resolution counter.
       Typically 1-3 million Hz on modern CPUs. */
    LARGE_INTEGER freq;
    if (!QueryPerformanceFrequency(&freq)) {
        /* If this fails, the CPU doesn't support high-resolution counters
           (very rare on modern systems) */
        return -1;
    }

    /* Store frequency in our global variable.
       LARGE_INTEGER is a union that contains QuadPart for 64-bit access. */
    g_timer_frequency = (uint64_t)freq.QuadPart;

#elif defined(__unix__) || defined(__linux__)
    /* ====== LINUX/UNIX IMPLEMENTATION ====== */

    /* On Unix, clock_gettime(CLOCK_MONOTONIC) is always available
       and returns nanosecond precision. No initialization needed,
       but we set the frequency for completeness. */
    g_timer_frequency = 1000000000ULL;  /* 1 GHz (nanoseconds) */

#endif

    /* Mark platform as initialized */
    g_platform_initialized = 1;
    return 0;
}

/* ============================================================================
   TIMER FUNCTIONS
   ============================================================================ */

/**
 * void platform_timer_start(platform_timer_t *timer)
 *
 * Captures the current system time and stores it in the timer structure.
 * This is the "zero point" - all elapsed time is measured from this moment.
 *
 * Implementation details:
 * - Windows: Uses QueryPerformanceCounter() for sub-microsecond precision
 * - Linux: Uses clock_gettime(CLOCK_MONOTONIC) for nanosecond precision
 *
 * Why CLOCK_MONOTONIC?
 * - CLOCK_REALTIME can go backward (system clock adjustment)
 * - CLOCK_MONOTONIC always moves forward, never adjusted
 * - Perfect for measuring intervals (timer.h)
 */
void platform_timer_start(platform_timer_t *timer)
{
    if (!timer) return;  /* Defensive programming: null pointer check */

#ifdef _WIN32
    /* ====== WINDOWS IMPLEMENTATION ====== */

    LARGE_INTEGER counter;
    /* Capture the current performance counter value.
       This is a 64-bit tick counter that increments at g_timer_frequency Hz. */
    QueryPerformanceCounter(&counter);

    /* Store the tick value */
    timer->start_ticks = (uint64_t)counter.QuadPart;

    /* Also store frequency for later conversion */
    timer->frequency = g_timer_frequency;

#elif defined(__unix__) || defined(__linux__)
    /* ====== LINUX/UNIX IMPLEMENTATION ====== */

    struct timespec ts;
    /* clock_gettime() reads the monotonic clock and fills in ts.
       - ts.tv_sec: seconds
       - ts.tv_nsec: nanoseconds (0-999999999)

       We convert to a single 64-bit "nanosecond ticks" value:
       total_ns = seconds * 1e9 + nanoseconds
    */
    clock_gettime(CLOCK_MONOTONIC, &ts);

    /* Convert seconds and nanoseconds to a single nanosecond tick count */
    timer->start_ticks = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

    /* On Unix, frequency is always nanoseconds */
    timer->frequency = 1000000000ULL;

#endif
}

/**
 * uint32_t platform_timer_elapsed_ms(const platform_timer_t *timer)
 *
 * Calculates how many milliseconds have elapsed since platform_timer_start().
 *
 * Algorithm:
 * 1. Get current time (same method as start)
 * 2. Subtract start time: elapsed_ticks = current_ticks - start_ticks
 * 3. Convert ticks to milliseconds using frequency:
 *    elapsed_ms = (elapsed_ticks * 1000) / frequency
 *
 * Why the *1000 and /frequency?
 * - Ticks are in "frequency Hz", so they're counts per second
 * - To convert to milliseconds: (ticks / frequency_hz) * 1000 ms
 * - This is the same as: (ticks * 1000) / frequency_hz
 *
 * Precision note:
 * - Windows: Can measure down to ~0.1 microseconds, returned as milliseconds
 * - Linux: Can measure down to 1 nanosecond, returned as milliseconds
 * - Both return uint32_t (milliseconds), so max range ~49 days
 */
uint32_t platform_timer_elapsed_ms(const platform_timer_t *timer)
{
    if (!timer) return 0;  /* Defensive: null pointer check */

#ifdef _WIN32
    /* ====== WINDOWS IMPLEMENTATION ====== */

    LARGE_INTEGER counter;
    /* Get current counter value */
    QueryPerformanceCounter(&counter);

    uint64_t current_ticks = (uint64_t)counter.QuadPart;

#elif defined(__unix__) || defined(__linux__)
    /* ====== LINUX/UNIX IMPLEMENTATION ====== */

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    /* Convert to nanoseconds */
    uint64_t current_ticks = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

#endif

    /* Calculate elapsed ticks */
    uint64_t elapsed_ticks = current_ticks - timer->start_ticks;

    /* Convert ticks to milliseconds.
       Formula: elapsed_ms = (elapsed_ticks * 1000) / frequency

       Why multiply first?
       - If we divide first, we lose precision due to integer division
       - Multiply first to shift the decimal point, then divide
       - Example: 1234 ticks / 1000000 freq = 0 (integer division!)
       -          1234 * 1000 / 1000000 = 1234000 / 1000000 = 1
       -          Which gives us the correct answer
    */
    uint32_t elapsed_ms = (uint32_t)((elapsed_ticks * 1000) / timer->frequency);

    return elapsed_ms;
}

/* ============================================================================
   SLEEP FUNCTIONS
   ============================================================================ */

/**
 * void platform_sleep_ms(uint32_t milliseconds)
 *
 * Blocks the current thread for approximately the given duration.
 *
 * Important note:
 * - NOT precise: Use timers to measure intervals
 * - Accuracy depends on OS scheduler (typically ±10-20 ms on Windows)
 * - Used mainly for delays in initialization or demo code
 *
 * Why two implementations?
 * - Windows: Sleep(milliseconds) is simple and well-known
 * - Linux: usleep(microseconds) - we convert milliseconds to microseconds
 *
 * Caveat: nanosleep() is more precise than usleep() on modern Linux,
 * but usleep() is more portable and sufficient for our needs.
 */
void platform_sleep_ms(uint32_t milliseconds)
{
#ifdef _WIN32
    /* ====== WINDOWS IMPLEMENTATION ====== */

    /* Windows Sleep() takes milliseconds directly.
       This will yield the thread to the OS scheduler, allowing
       other threads/processes to run. It's not "busy waiting",
       so it's efficient. */
    Sleep((DWORD)milliseconds);

#elif defined(__unix__) || defined(__linux__)
    /* ====== LINUX/UNIX IMPLEMENTATION ====== */

    /* usleep() takes microseconds, so we multiply by 1000.
       Example: 100 ms = 100 * 1000 = 100,000 microseconds */
    usleep(milliseconds * 1000);

#endif
}

/* ============================================================================
   TIME FUNCTIONS
   ============================================================================ */

/**
 * time_t platform_get_time_unix(void)
 *
 * Returns the current time as Unix timestamp (seconds since 1970-01-01).
 *
 * Usage:
 * - Timestamping log messages
 * - Recording when telemetry data was collected
 * - Not for measuring intervals (use timers!)
 *
 * Standard C function - same on all platforms.
 * time_t is typically int32_t or int64_t depending on platform.
 *
 * Note: Will overflow on 32-bit systems in year 2038!
 * (But that's a problem for 2038... not our concern for this project)
 */
time_t platform_get_time_unix(void)
{
    /* time(NULL) returns current time as Unix timestamp.
       NULL means "just return the time, don't store it anywhere". */
    return time(NULL);
}
