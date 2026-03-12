/**
 * @file test_scheduler.c
 * @brief Test unitari per il modulo scheduler.c
 *
 * Testiamo la logica di registrazione e gestione dei task.
 * Non testiamo il timing reale (dipendente dal sistema operativo)
 * ma la correttezza della struttura dati e delle guardie di sicurezza.
 */

#include "test_framework.h"
#include "scheduler.h"

/* Callback dummy usate nei test */
static int g_cb_count_a = 0;
static int g_cb_count_b = 0;

static void cb_a(void) { g_cb_count_a++; }
static void cb_b(void) { g_cb_count_b++; }

/* ------------------------------------------------------------------ */
/*  Test: registrazione task                                           */
/* ------------------------------------------------------------------ */

TEST(add_first_task_returns_zero) {
    scheduler_init();
    int idx = scheduler_add_task("task_a", 100, cb_a);
    ASSERT_EQ(0, idx);
}

TEST(add_second_task_returns_one) {
    scheduler_init();
    scheduler_add_task("task_a", 100, cb_a);
    int idx = scheduler_add_task("task_b", 200, cb_b);
    ASSERT_EQ(1, idx);
}

TEST(add_task_max_limit_returns_minus_one) {
    /*
     * SCHEDULER_MAX_TASKS è il limite massimo.
     * Il (MAX+1)-esimo task deve essere rifiutato.
     */
    scheduler_init();
    int last_valid = -1;
    for (int i = 0; i < SCHEDULER_MAX_TASKS; i++) {
        last_valid = scheduler_add_task("t", 10, cb_a);
    }
    ASSERT_EQ(SCHEDULER_MAX_TASKS - 1, last_valid);

    /* Il prossimo deve fallire */
    int overflow = scheduler_add_task("overflow", 10, cb_a);
    ASSERT_EQ(-1, overflow);
}

TEST(add_task_null_callback_rejected) {
    /*
     * Una callback NULL causerebbe un undefined behavior quando
     * lo scheduler tenta di chiamarla. Deve essere rifiutata.
     * Questo è un requisito di robustezza MISRA-C.
     */
    scheduler_init();
    int idx = scheduler_add_task("null_cb", 100, NULL);
    ASSERT_EQ(-1, idx);
}

TEST(reinit_clears_all_tasks) {
    /*
     * Dopo una reinizializzazione lo scheduler deve essere vuoto.
     * Aggiungere un task deve restituire indice 0.
     */
    scheduler_init();
    scheduler_add_task("a", 10, cb_a);
    scheduler_add_task("b", 20, cb_b);

    /* Reinit */
    scheduler_init();
    int idx = scheduler_add_task("fresh", 50, cb_a);
    ASSERT_EQ(0, idx);
}

TEST(zero_period_task_accepted) {
    /*
     * Un task con periodo 0ms viene eseguito ad ogni tick.
     * Non è un errore — può essere utile per task di diagnostica
     * che devono girare il più frequentemente possibile.
     */
    scheduler_init();
    int idx = scheduler_add_task("fast", 0, cb_a);
    ASSERT_NEQ(-1, idx);
}
