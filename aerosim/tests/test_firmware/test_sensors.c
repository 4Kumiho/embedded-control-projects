/**
 * @file test_sensors.c
 * @brief Test unitari per il modulo sensors.c
 *
 * Strategia di test per dati stocastici:
 * - Si eseguono N campioni e si verifica che la MEDIA sia vicina
 *   al valore atteso (legge dei grandi numeri).
 * - La varianza deve essere consistente con il rumore dichiarato.
 *
 * Questo approccio è usato nella validazione di filtri di navigazione
 * inerziale (INS) in ambito Aerospace & Defence.
 */

#include "test_framework.h"
#include "sensors.h"

#include <math.h>

#define N_SAMPLES 1000   /* campioni per i test statistici */

/* ------------------------------------------------------------------ */
/*  Test: inizializzazione                                             */
/* ------------------------------------------------------------------ */

TEST(sensors_init_no_crash) {
    /* Verifica che l'inizializzazione non causi crash con seed diversi */
    sensors_init(0);
    sensors_init(42);
    sensors_init(0xDEADBEEF);
    ASSERT_TRUE(1);   /* se arriviamo qui, non è crashato */
}

/* ------------------------------------------------------------------ */
/*  Test: IMU                                                          */
/* ------------------------------------------------------------------ */

TEST(imu_null_pointer_safe) {
    /* Il firmware reale deve tollerare puntatori NULL (MISRA-C Rule 17.5) */
    sensors_init(1);
    sensors_read_imu(NULL);   /* non deve crashare */
    ASSERT_TRUE(1);
}

TEST(imu_az_near_gravity_at_rest) {
    /*
     * A riposo (altitude=0, no movimento), az deve essere ~9.81 m/s²
     * (reazione del suolo). Verifichiamo con la media su N campioni.
     */
    sensors_init(42);
    sensors_set_altitude(0.0f);

    double sum_az = 0.0;
    IMUData d;
    for (int i = 0; i < N_SAMPLES; i++) {
        sensors_read_imu(&d);
        sum_az += d.az;
    }
    double mean_az = sum_az / N_SAMPLES;

    /* La media deve essere entro ±0.5 m/s² da 9.81 */
    ASSERT_NEAR(9.81, mean_az, 0.5);
}

TEST(imu_ax_ay_near_zero_at_rest) {
    sensors_init(99);
    sensors_set_altitude(0.0f);

    double sum_ax = 0.0, sum_ay = 0.0;
    IMUData d;
    for (int i = 0; i < N_SAMPLES; i++) {
        sensors_read_imu(&d);
        sum_ax += d.ax;
        sum_ay += d.ay;
    }
    /* Media delle componenti orizzontali deve essere ~0 */
    ASSERT_NEAR(0.0, sum_ax / N_SAMPLES, 0.3);
    ASSERT_NEAR(0.0, sum_ay / N_SAMPLES, 0.3);
}

TEST(imu_gyro_near_zero_at_rest) {
    sensors_init(7);
    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    IMUData d;
    for (int i = 0; i < N_SAMPLES; i++) {
        sensors_read_imu(&d);
        sum_gx += d.gx;
        sum_gy += d.gy;
        sum_gz += d.gz;
    }
    ASSERT_NEAR(0.0, sum_gx / N_SAMPLES, 0.1);
    ASSERT_NEAR(0.0, sum_gy / N_SAMPLES, 0.1);
    ASSERT_NEAR(0.0, sum_gz / N_SAMPLES, 0.1);
}

/* ------------------------------------------------------------------ */
/*  Test: GPS                                                          */
/* ------------------------------------------------------------------ */

TEST(gps_null_pointer_safe) {
    sensors_init(1);
    sensors_read_gps(NULL);
    ASSERT_TRUE(1);
}

TEST(gps_default_position_near_milano) {
    /*
     * La posizione di default è centrata su Milano (45.4654°N, 9.1859°E).
     * Con seed fisso, la media su N campioni deve essere vicina.
     */
    sensors_init(42);
    sensors_set_altitude(0.0f);

    double sum_lat = 0.0, sum_lon = 0.0;
    GPSData g;
    for (int i = 0; i < N_SAMPLES; i++) {
        sensors_read_gps(&g);
        sum_lat += g.lat;
        sum_lon += g.lon;
    }
    ASSERT_NEAR(45.4654, sum_lat / N_SAMPLES, 0.001);
    ASSERT_NEAR( 9.1859, sum_lon / N_SAMPLES, 0.001);
}

TEST(gps_altitude_reflects_set_value) {
    sensors_init(42);
    float test_alt = 50.0f;
    sensors_set_altitude(test_alt);

    double sum_alt = 0.0;
    GPSData g;
    for (int i = 0; i < N_SAMPLES; i++) {
        sensors_read_gps(&g);
        sum_alt += g.alt;
    }
    /* La media dell'altitudine GPS deve essere vicina al valore impostato */
    ASSERT_NEAR((double)test_alt, sum_alt / N_SAMPLES, 2.0);
}

/* ------------------------------------------------------------------ */
/*  Test: barometro                                                    */
/* ------------------------------------------------------------------ */

TEST(baro_null_pointer_safe) {
    sensors_init(1);
    sensors_read_baro(NULL);
    ASSERT_TRUE(1);
}

TEST(baro_pressure_decreases_with_altitude) {
    sensors_init(42);

    BaroData b_low, b_high;

    /* Media a quota bassa */
    sensors_set_altitude(0.0f);
    double sum_low = 0.0;
    for (int i = 0; i < N_SAMPLES; i++) {
        sensors_read_baro(&b_low);
        sum_low += b_low.pressure;
    }

    /* Media a quota alta */
    sensors_set_altitude(1000.0f);
    double sum_high = 0.0;
    for (int i = 0; i < N_SAMPLES; i++) {
        sensors_read_baro(&b_high);
        sum_high += b_high.pressure;
    }

    /* La pressione deve essere MINORE a quota maggiore */
    ASSERT_TRUE(sum_high / N_SAMPLES < sum_low / N_SAMPLES);
}

TEST(baro_altitude_correlates_with_set_altitude) {
    sensors_init(42);
    sensors_set_altitude(500.0f);

    double sum_baro_alt = 0.0;
    BaroData b;
    for (int i = 0; i < N_SAMPLES; i++) {
        sensors_read_baro(&b);
        sum_baro_alt += b.baro_alt;
    }
    /* La quota barometrica deve essere vicina a quella impostata */
    ASSERT_NEAR(500.0, sum_baro_alt / N_SAMPLES, 10.0);
}
