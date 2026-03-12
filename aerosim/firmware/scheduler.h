/**
 * @file scheduler.h
 * @brief Schedulatore ciclico cooperativo a priorita' fissa.
 *
 * Nei sistemi embedded real-time i task vengono eseguiti a intervalli
 * precisi guidati da un timer hardware (es. SysTick su Cortex-M).
 * Questo modulo simula quel comportamento su PC usando il clock di sistema.
 *
 * Schema di funzionamento:
 *
 *   Timer ISR (ogni 1ms)
 *       |
 *       v
 *   tick_counter++
 *       |
 *   Main loop
 *       |
 *       +---> Task A (ogni 10ms)  es. lettura sensori
 *       +---> Task B (ogni 100ms) es. invio telemetria
 *       +---> Task C (ogni 1000ms) es. diagnostica
 */

#ifndef AEROSIM_SCHEDULER_H
#define AEROSIM_SCHEDULER_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Configurazione                                                      */
/* ------------------------------------------------------------------ */

#define SCHEDULER_MAX_TASKS  8   /**< Numero massimo di task registrabili */

/* ------------------------------------------------------------------ */
/*  Strutture dati                                                      */
/* ------------------------------------------------------------------ */

/**
 * @brief Descrittore di un task schedulato.
 */
typedef struct {
    const char  *name;          /**< Nome descrittivo (per debug)       */
    uint32_t     period_ms;     /**< Periodo di esecuzione [ms]         */
    uint32_t     last_run_ms;   /**< Timestamp ultima esecuzione [ms]   */
    void        (*callback)(void); /**< Funzione da eseguire            */
    uint32_t     run_count;     /**< Numero totale di esecuzioni        */
} Task;

/* ------------------------------------------------------------------ */
/*  API pubblica                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief Inizializza lo scheduler.
 */
void scheduler_init(void);

/**
 * @brief Registra un nuovo task.
 * @param name       Nome descrittivo del task.
 * @param period_ms  Periodo in millisecondi.
 * @param callback   Funzione da richiamare ad ogni scadenza.
 * @return           Indice del task, oppure -1 se la lista e' piena.
 */
int scheduler_add_task(const char *name, uint32_t period_ms,
                       void (*callback)(void));

/**
 * @brief Esegue un singolo ciclo dello scheduler.
 *
 * Da chiamare nel main loop. Controlla quali task hanno superato
 * il loro periodo e li esegue in ordine di registrazione.
 */
void scheduler_tick(void);

/**
 * @brief Avvia il main loop dello scheduler (bloccante).
 * @param duration_ms  Durata della simulazione in ms (0 = infinito).
 */
void scheduler_run(uint32_t duration_ms);

/**
 * @brief Stampa a schermo le statistiche dei task registrati.
 */
void scheduler_print_stats(void);

#endif /* AEROSIM_SCHEDULER_H */
