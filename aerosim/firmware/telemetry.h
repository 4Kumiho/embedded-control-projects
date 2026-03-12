/**
 * @file telemetry.h
 * @brief Costruzione e trasmissione pacchetti telemetria.
 *
 * Protocollo binario definito nel SRS (sezione 5):
 *
 *   ┌────────┬────────┬──────────┬──────────────────────┬──────────┐
 *   │ START  │  TYPE  │  LENGTH  │       PAYLOAD         │ CHECKSUM │
 *   │ 1 byte │ 1 byte │  2 byte  │    0-255 bytes        │  1 byte  │
 *   └────────┴────────┴──────────┴──────────────────────┴──────────┘
 *
 * START    = 0xAE  (byte di sincronizzazione)
 * TYPE     = tipo di pacchetto
 * LENGTH   = lunghezza payload in byte (little-endian)
 * CHECKSUM = XOR di tutti i byte del payload
 *
 * La trasmissione avviene scrivendo su file binario (telemetry.bin)
 * che la Ground Station leggerà via pipe o socket nella Fase 4.
 */

#ifndef AEROSIM_TELEMETRY_H
#define AEROSIM_TELEMETRY_H

#include "sensors.h"
#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Costanti di protocollo                                              */
/* ------------------------------------------------------------------ */

#define TELEM_START_BYTE     0xAEu  /**< Byte di sincronizzazione       */
#define TELEM_MAX_PAYLOAD    255u   /**< Lunghezza massima payload       */
#define TELEM_HEADER_SIZE    4u     /**< START + TYPE + LENGTH(2)       */
#define TELEM_OVERHEAD       5u     /**< Header + checksum              */

/** Tipi di pacchetto (TYPE field) */
typedef enum {
    TELEM_TYPE_IMU    = 0x01,   /**< Dati accelerometro + giroscopio */
    TELEM_TYPE_GPS    = 0x02,   /**< Dati posizione GPS              */
    TELEM_TYPE_STATUS = 0x03,   /**< Stato sistema                   */
} TelemetryType;

/* ------------------------------------------------------------------ */
/*  Struttura pacchetto                                                 */
/* ------------------------------------------------------------------ */

/** Stato del sistema incluso nel pacchetto STATUS */
typedef struct {
    uint8_t  arm_state;    /**< 0=disarmato, 1=armato              */
    uint16_t error_flags;  /**< Bitmask errori (bit 0=IMU, bit 1=GPS, ...) */
    uint32_t uptime_ms;    /**< Millisecondi dall'avvio            */
} SystemStatus;

/* ------------------------------------------------------------------ */
/*  API pubblica                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief Inizializza il modulo telemetria e apre il file di output.
 * @param output_path  Percorso del file binario di output.
 * @return             0 in caso di successo, -1 in caso di errore.
 */
int telemetry_init(const char *output_path);

/**
 * @brief Chiude il file di output e libera le risorse.
 */
void telemetry_close(void);

/**
 * @brief Costruisce e invia un pacchetto con dati IMU.
 * @param data  Dati IMU da trasmettere.
 * @return      Numero di byte scritti, -1 in caso di errore.
 */
int telemetry_send_imu(const IMUData *data);

/**
 * @brief Costruisce e invia un pacchetto con dati GPS.
 * @param data  Dati GPS da trasmettere.
 * @return      Numero di byte scritti, -1 in caso di errore.
 */
int telemetry_send_gps(const GPSData *data);

/**
 * @brief Costruisce e invia un pacchetto di stato sistema.
 * @param status  Stato del sistema.
 * @return        Numero di byte scritti, -1 in caso di errore.
 */
int telemetry_send_status(const SystemStatus *status);

/**
 * @brief Calcola il checksum XOR di un buffer.
 * @param buf  Buffer dati.
 * @param len  Lunghezza del buffer.
 * @return     Byte di checksum.
 */
uint8_t telemetry_checksum(const uint8_t *buf, uint16_t len);

/**
 * @brief Stampa a schermo un pacchetto in formato hex (per debug).
 * @param buf  Buffer del pacchetto completo.
 * @param len  Lunghezza totale.
 */
void telemetry_print_hex(const uint8_t *buf, uint16_t len);

#endif /* AEROSIM_TELEMETRY_H */
