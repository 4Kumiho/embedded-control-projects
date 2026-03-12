/**
 * @file sensors.c
 * @brief Implementazione dei driver sensori simulati.
 *
 * Il rumore gaussiano e' generato con la trasformata di Box-Muller:
 *   z = sqrt(-2 * ln(u1)) * cos(2 * pi * u2)
 * dove u1 e u2 sono numeri uniformi in (0,1].
 * Questa tecnica e' usata anche nei simulatori di volo certificati.
 */

#include "sensors.h"

#include <math.h>
#include <stdlib.h>

/* ------------------------------------------------------------------ */
/*  Costanti fisiche e parametri di rumore                             */
/* ------------------------------------------------------------------ */

#define GRAVITY          9.81f   /* m/s^2                            */
#define SEA_LEVEL_PRESS  1013.25f /* hPa                             */

/* Deviazioni standard del rumore per ogni sensore */
#define IMU_ACC_NOISE    0.05f   /* m/s^2 — tipico MEMS accelerometro */
#define IMU_GYRO_NOISE   0.02f   /* deg/s — tipico MEMS giroscopio    */
#define GPS_POS_NOISE    0.000005 /* gradi — ~0.5m di errore          */
#define GPS_ALT_NOISE    0.5     /* m                                 */
#define GPS_SPD_NOISE    0.1f    /* m/s                               */
#define BARO_PRESS_NOISE 0.1f    /* hPa                               */

/* ------------------------------------------------------------------ */
/*  Stato interno del modulo (variabili statiche = "private")          */
/* ------------------------------------------------------------------ */

static float s_altitude = 0.0f;  /* Quota corrente imposta dall'esterno */

/* ------------------------------------------------------------------ */
/*  Funzione privata: rumore gaussiano (Box-Muller)                    */
/* ------------------------------------------------------------------ */

/**
 * @brief Genera un campione gaussiano N(mean, std).
 *
 * Non e' thread-safe (usa rand()), ma per un firmware single-task
 * e' sufficiente. In un RTOS si userebbe un mutex.
 */
static double gaussian(double mean, double std) {
    double u1, u2;

    /* Evita log(0) che darebbe -inf */
    do { u1 = (double)rand() / RAND_MAX; } while (u1 == 0.0);
    u2 = (double)rand() / RAND_MAX;

    double z = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return mean + std * z;
}

/* ------------------------------------------------------------------ */
/*  Implementazione API pubblica                                        */
/* ------------------------------------------------------------------ */

void sensors_init(uint32_t seed) {
    srand(seed);
    s_altitude = 0.0f;
}

void sensors_read_imu(IMUData *out) {
    if (!out) return;

    /*
     * Stato di riposo: accelerazione = (0, 0, g) perche' il sensore
     * percepisce la reazione del suolo. In volo orizzontale stabile
     * il valore di az rimane ~9.81.
     */
    out->ax = (float)gaussian(0.0,    IMU_ACC_NOISE);
    out->ay = (float)gaussian(0.0,    IMU_ACC_NOISE);
    out->az = (float)gaussian(GRAVITY, IMU_ACC_NOISE);

    /* Velocita' angolare: a riposo e' ~0 con piccolo rumore */
    out->gx = (float)gaussian(0.0, IMU_GYRO_NOISE);
    out->gy = (float)gaussian(0.0, IMU_GYRO_NOISE);
    out->gz = (float)gaussian(0.0, IMU_GYRO_NOISE);
}

void sensors_read_gps(GPSData *out) {
    if (!out) return;

    /*
     * Punto di riferimento: ALTEN Italia, Milano.
     * In un sistema reale queste coordinate verrebbero aggiornate
     * integrando la velocita' calcolata dal navigation filter.
     */
    out->lat   = 45.4654 + gaussian(0.0, GPS_POS_NOISE);
    out->lon   =  9.1859 + gaussian(0.0, GPS_POS_NOISE);
    out->alt   = (double)s_altitude + gaussian(0.0, GPS_ALT_NOISE);
    out->speed = (float)gaussian(0.0, GPS_SPD_NOISE);
}

void sensors_read_baro(BaroData *out) {
    if (!out) return;

    /*
     * Formula barometrica internazionale semplificata:
     *   P = P0 * (1 - L*h/T0)^(g*M / R*L)
     * Approssimazione lineare valida fino a ~2000m:
     *   P ≈ P0 - 0.12 * h  [hPa]
     */
    float true_pressure = SEA_LEVEL_PRESS - 0.12f * s_altitude;
    out->pressure = true_pressure + (float)gaussian(0.0, BARO_PRESS_NOISE);

    /* Quota barometrica ricavata dalla pressione letta */
    out->baro_alt = (SEA_LEVEL_PRESS - out->pressure) / 0.12f;
}

void sensors_set_altitude(float altitude_m) {
    s_altitude = altitude_m;
}
