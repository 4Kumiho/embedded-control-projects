/**
 * @file sensors.h
 * @brief Driver simulati per IMU, GPS e barometro.
 *
 * In un firmware reale ogni funzione read_* conterrebbe la comunicazione
 * con il sensore fisico via I2C o SPI. Qui i dati vengono generati
 * matematicamente con rumore gaussiano (Box-Muller transform).
 */

#ifndef AEROSIM_SENSORS_H
#define AEROSIM_SENSORS_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Strutture dati dei sensori                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief Dati dall'Inertial Measurement Unit (IMU).
 *
 * Un IMU reale (es. MPU-6050, ICM-42688) espone questi 6 valori
 * via I2C/SPI leggendo registri interni a 16 bit.
 */
typedef struct {
    float ax;   /**< Accelerazione asse X [m/s^2] */
    float ay;   /**< Accelerazione asse Y [m/s^2] */
    float az;   /**< Accelerazione asse Z [m/s^2] */
    float gx;   /**< Velocita' angolare asse X [deg/s] */
    float gy;   /**< Velocita' angolare asse Y [deg/s] */
    float gz;   /**< Velocita' angolare asse Z [deg/s] */
} IMUData;

/**
 * @brief Dati dal ricevitore GPS.
 *
 * Un modulo GPS reale (es. u-blox NEO-M9N) invia frasi NMEA via UART
 * oppure dati binari via protocollo UBX.
 */
typedef struct {
    double lat;    /**< Latitudine  [gradi decimali, N positivo] */
    double lon;    /**< Longitudine [gradi decimali, E positivo] */
    double alt;    /**< Altitudine  [m sul livello del mare]     */
    float  speed;  /**< Velocita' al suolo [m/s]                */
} GPSData;

/**
 * @brief Dati dal barometro.
 *
 * Un barometro reale (es. BMP390) comunica via SPI o I2C e fornisce
 * pressione assoluta da cui si ricava la quota barometrica.
 */
typedef struct {
    float pressure;    /**< Pressione atmosferica [hPa] */
    float baro_alt;    /**< Quota barometrica [m]       */
} BaroData;

/* ------------------------------------------------------------------ */
/*  API pubblica                                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief Inizializza il modulo sensori e il seed del generatore random.
 * @param seed  Seed per la riproducibilita' della simulazione.
 */
void sensors_init(uint32_t seed);

/**
 * @brief Legge i dati dall'IMU simulato.
 * @param out  Puntatore alla struttura da riempire.
 */
void sensors_read_imu(IMUData *out);

/**
 * @brief Legge i dati dal GPS simulato.
 * @param out  Puntatore alla struttura da riempire.
 */
void sensors_read_gps(GPSData *out);

/**
 * @brief Legge i dati dal barometro simulato.
 * @param out  Puntatore alla struttura da riempire.
 */
void sensors_read_baro(BaroData *out);

/**
 * @brief Imposta la quota di volo simulata (usata dal physics engine).
 * @param altitude_m  Quota in metri.
 */
void sensors_set_altitude(float altitude_m);

#endif /* AEROSIM_SENSORS_H */
