/**
 * @file scheduler.c
 * @brief Implementazione dello schedulatore ciclico cooperativo.
 */

#include "scheduler.h"
#include "platform.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Stato interno                                                       */
/* ------------------------------------------------------------------ */

static Task     s_tasks[SCHEDULER_MAX_TASKS];
static int      s_task_count = 0;
static uint32_t s_start_ms   = 0;

/* ------------------------------------------------------------------ */
/*  Implementazione API                                                 */
/* ------------------------------------------------------------------ */

void scheduler_init(void) {
    memset(s_tasks, 0, sizeof(s_tasks));
    s_task_count = 0;
    s_start_ms   = platform_get_tick_ms();

    printf("[SCHEDULER] Inizializzato. Tick base: %u ms\n", s_start_ms);
}

int scheduler_add_task(const char *name, uint32_t period_ms,
                       void (*callback)(void)) {
    if (s_task_count >= SCHEDULER_MAX_TASKS) {
        printf("[SCHEDULER] ERRORE: lista task piena!\n");
        return -1;
    }
    if (!callback) {
        printf("[SCHEDULER] ERRORE: callback NULL per task '%s'\n", name);
        return -1;
    }

    Task *t        = &s_tasks[s_task_count];
    t->name        = name;
    t->period_ms   = period_ms;
    t->last_run_ms = platform_get_tick_ms(); /* parte subito sincronizzato */
    t->callback    = callback;
    t->run_count   = 0;

    printf("[SCHEDULER] Task '%s' registrato (periodo: %u ms)\n",
           name, period_ms);

    return s_task_count++;
}

void scheduler_tick(void) {
    uint32_t now = platform_get_tick_ms();

    for (int i = 0; i < s_task_count; i++) {
        Task *t = &s_tasks[i];

        /*
         * Controlla se il task e' scaduto.
         * La sottrazione unsigned gestisce correttamente il wrap-around
         * del contatore a 32 bit (overflow dopo ~49 giorni).
         */
        if ((now - t->last_run_ms) >= t->period_ms) {
            t->callback();
            t->last_run_ms = now;
            t->run_count++;
        }
    }
}

void scheduler_run(uint32_t duration_ms) {
    uint32_t start = platform_get_tick_ms();

    printf("[SCHEDULER] Avvio loop. Durata: %s\n",
           duration_ms ? "limitata" : "infinita");

    while (1) {
        scheduler_tick();

        /* Pausa di 1ms per non saturare la CPU (simula WFI su MCU) */
        platform_sleep_ms(1);

        if (duration_ms > 0) {
            uint32_t elapsed = platform_get_tick_ms() - start;
            if (elapsed >= duration_ms) break;
        }
    }

    printf("[SCHEDULER] Loop terminato.\n");
    scheduler_print_stats();
}

void scheduler_print_stats(void) {
    printf("\n=== Statistiche scheduler ===\n");
    printf("%-20s %10s %10s\n", "Task", "Periodo(ms)", "Esecuzioni");
    printf("--------------------------------------------\n");
    for (int i = 0; i < s_task_count; i++) {
        Task *t = &s_tasks[i];
        printf("%-20s %10u %10u\n", t->name, t->period_ms, t->run_count);
    }
    printf("============================\n\n");
}
