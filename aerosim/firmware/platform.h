/**
 * @file platform.h
 * @brief Astrazione cross-platform per tempo e sleep.
 *
 * In un firmware reale questo file conterrebbe le chiamate ai registri
 * hardware del MCU (es. SysTick su STM32). Qui simuliamo lo stesso
 * comportamento usando le API del sistema operativo host.
 */

#ifndef AEROSIM_PLATFORM_H
#define AEROSIM_PLATFORM_H

#include <stdint.h>

#ifdef _WIN32
    #include <windows.h>

    /** Restituisce il tempo corrente in millisecondi dall'avvio. */
    static inline uint32_t platform_get_tick_ms(void) {
        return (uint32_t)GetTickCount();
    }

    /** Sospende l'esecuzione per ms millisecondi (simula WFI su MCU). */
    static inline void platform_sleep_ms(uint32_t ms) {
        Sleep(ms);
    }

#else
    #include <time.h>

    static inline uint32_t platform_get_tick_ms(void) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }

    static inline void platform_sleep_ms(uint32_t ms) {
        struct timespec ts = { (time_t)(ms / 1000),
                               (long)((ms % 1000) * 1000000L) };
        nanosleep(&ts, NULL);
    }
#endif

#endif /* AEROSIM_PLATFORM_H */
