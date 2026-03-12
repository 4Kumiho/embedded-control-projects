/**
 * @file test_framework.h
 * @brief Lightweight unit test framework per firmware C.
 *
 * Non richiede dipendenze esterne (no Google Test, no Unity).
 * Sufficiente per un progetto embedded dove le librerie esterne
 * potrebbero non essere disponibili sul target MCU.
 *
 * Uso:
 *   TEST(nome_test) {
 *       ASSERT_EQ(valore_atteso, valore_reale);
 *       ASSERT_TRUE(condizione);
 *       ASSERT_NEAR(atteso, reale, tolleranza);
 *   }
 *
 *   int main(void) {
 *       RUN_TEST(nome_test);
 *       PRINT_SUMMARY();
 *       return g_failures > 0 ? 1 : 0;
 *   }
 */

#ifndef AEROSIM_TEST_FRAMEWORK_H
#define AEROSIM_TEST_FRAMEWORK_H

#include <stdio.h>
#include <math.h>

/* Contatori globali */
static int g_passes   = 0;
static int g_failures = 0;
static int g_tests    = 0;

/* ------------------------------------------------------------------ */
/*  Macro di asserzione                                                */
/* ------------------------------------------------------------------ */

#define ASSERT_TRUE(cond)                                               \
    do {                                                                \
        if (!(cond)) {                                                  \
            printf("  FAIL [%s:%d] ASSERT_TRUE(%s)\n",                 \
                   __FILE__, __LINE__, #cond);                          \
            g_failures++;                                               \
        } else {                                                        \
            g_passes++;                                                 \
        }                                                               \
    } while (0)

#define ASSERT_FALSE(cond) ASSERT_TRUE(!(cond))

#define ASSERT_EQ(expected, actual)                                     \
    do {                                                                \
        if ((expected) != (actual)) {                                   \
            printf("  FAIL [%s:%d] expected=%d, actual=%d\n",          \
                   __FILE__, __LINE__, (int)(expected), (int)(actual)); \
            g_failures++;                                               \
        } else {                                                        \
            g_passes++;                                                 \
        }                                                               \
    } while (0)

#define ASSERT_NEQ(a, b)                                                \
    do {                                                                \
        if ((a) == (b)) {                                               \
            printf("  FAIL [%s:%d] values should differ but both=%d\n",\
                   __FILE__, __LINE__, (int)(a));                       \
            g_failures++;                                               \
        } else {                                                        \
            g_passes++;                                                 \
        }                                                               \
    } while (0)

/**
 * @brief Verifica che due float siano entro una tolleranza assoluta.
 * Necessario perché il confronto diretto tra float è unreliable.
 */
#define ASSERT_NEAR(expected, actual, tol)                              \
    do {                                                                \
        double _diff = fabs((double)(expected) - (double)(actual));     \
        if (_diff > (double)(tol)) {                                    \
            printf("  FAIL [%s:%d] |%.4f - %.4f| = %.4f > tol=%.4f\n",\
                   __FILE__, __LINE__,                                  \
                   (double)(expected), (double)(actual),                \
                   _diff, (double)(tol));                               \
            g_failures++;                                               \
        } else {                                                        \
            g_passes++;                                                 \
        }                                                               \
    } while (0)

/* ------------------------------------------------------------------ */
/*  Macro di test                                                      */
/* ------------------------------------------------------------------ */

/** Dichiara e avvia una funzione di test */
#define TEST(name) static void test_##name(void)

/** Esegue un test e stampa il risultato */
#define RUN_TEST(name)                                                  \
    do {                                                                \
        int _fail_before = g_failures;                                  \
        g_tests++;                                                      \
        printf("[ RUN ] %s\n", #name);                                  \
        test_##name();                                                  \
        if (g_failures == _fail_before) {                               \
            printf("[ OK  ] %s\n", #name);                              \
        } else {                                                        \
            printf("[ FAIL] %s\n", #name);                              \
        }                                                               \
    } while (0)

/** Stampa il riepilogo finale */
#define PRINT_SUMMARY()                                                 \
    do {                                                                \
        printf("\n==========================================\n");        \
        printf("  Tests: %d  |  Pass: %d  |  Fail: %d\n",             \
               g_tests, g_passes, g_failures);                          \
        printf("  Coverage: asserts=%d\n", g_passes + g_failures);     \
        printf("==========================================\n");         \
        if (g_failures == 0) {                                          \
            printf("  TUTTI I TEST PASSATI\n");                         \
        } else {                                                        \
            printf("  %d TEST FALLITI\n", g_failures);                  \
        }                                                               \
        printf("==========================================\n\n");       \
    } while (0)

#endif /* AEROSIM_TEST_FRAMEWORK_H */
