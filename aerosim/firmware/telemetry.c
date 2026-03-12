/**
 * @file telemetry.c
 * @brief Implementazione del modulo telemetria.
 *
 * I dati vengono serializzati in formato little-endian per compatibilita'
 * con le architetture ARM Cortex-M (little-endian nativo).
 * Il file di output puo' essere letto dalla Ground Station in Fase 4.
 */

#include "telemetry.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Stato interno                                                       */
/* ------------------------------------------------------------------ */

static FILE *s_out_file = NULL;

/* ------------------------------------------------------------------ */
/*  Funzioni private di serializzazione                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Scrive un uint16_t in little-endian nel buffer.
 */
static void write_u16_le(uint8_t *buf, uint16_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

/**
 * @brief Scrive un uint32_t in little-endian nel buffer.
 */
static void write_u32_le(uint8_t *buf, uint32_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8)  & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

/**
 * @brief Copia un float nel buffer come 4 byte raw (IEEE 754).
 *
 * Tecnica comune nel firmware embedded: evita problemi di allineamento
 * e conversioni implicite usando memcpy.
 */
static void write_float(uint8_t *buf, float val) {
    memcpy(buf, &val, sizeof(float));
}

/**
 * @brief Copia un double nel buffer come 8 byte raw (IEEE 754).
 */
static void write_double(uint8_t *buf, double val) {
    memcpy(buf, &val, sizeof(double));
}

/* ------------------------------------------------------------------ */
/*  Funzione privata: costruisce e invia un pacchetto generico          */
/* ------------------------------------------------------------------ */

static int send_packet(TelemetryType type,
                       const uint8_t *payload, uint16_t payload_len) {
    if (!s_out_file) return -1;
    if (payload_len > TELEM_MAX_PAYLOAD) return -1;

    /* Buffer del pacchetto completo: header + payload + checksum */
    uint8_t packet[TELEM_OVERHEAD + TELEM_MAX_PAYLOAD];
    uint16_t idx = 0;

    /* Header */
    packet[idx++] = TELEM_START_BYTE;
    packet[idx++] = (uint8_t)type;
    write_u16_le(&packet[idx], payload_len);
    idx += 2;

    /* Payload */
    memcpy(&packet[idx], payload, payload_len);
    idx += payload_len;

    /* Checksum (XOR di tutti i byte del payload) */
    packet[idx++] = telemetry_checksum(payload, payload_len);

    /* Scrittura su file */
    size_t written = fwrite(packet, 1, idx, s_out_file);
    fflush(s_out_file); /* Flush immediato: importante per pipe in tempo reale */

    return (written == idx) ? (int)written : -1;
}

/* ------------------------------------------------------------------ */
/*  Implementazione API pubblica                                        */
/* ------------------------------------------------------------------ */

int telemetry_init(const char *output_path) {
    s_out_file = fopen(output_path, "wb");
    if (!s_out_file) {
        printf("[TELEM] ERRORE: impossibile aprire '%s'\n", output_path);
        return -1;
    }
    printf("[TELEM] Output su file: %s\n", output_path);
    return 0;
}

void telemetry_close(void) {
    if (s_out_file) {
        fclose(s_out_file);
        s_out_file = NULL;
    }
}

int telemetry_send_imu(const IMUData *data) {
    if (!data) return -1;

    /*
     * Payload IMU: ax, ay, az, gx, gy, gz (6 float = 24 byte)
     * Ordine: X, Y, Z per accelerazione, poi X, Y, Z per giroscopio
     */
    uint8_t payload[24];
    write_float(&payload[0],  data->ax);
    write_float(&payload[4],  data->ay);
    write_float(&payload[8],  data->az);
    write_float(&payload[12], data->gx);
    write_float(&payload[16], data->gy);
    write_float(&payload[20], data->gz);

    return send_packet(TELEM_TYPE_IMU, payload, sizeof(payload));
}

int telemetry_send_gps(const GPSData *data) {
    if (!data) return -1;

    /*
     * Payload GPS: lat, lon, alt (3 double = 24 byte) + speed (float = 4 byte)
     * Totale: 28 byte
     */
    uint8_t payload[28];
    write_double(&payload[0],  data->lat);
    write_double(&payload[8],  data->lon);
    write_double(&payload[16], data->alt);
    write_float (&payload[24], data->speed);

    return send_packet(TELEM_TYPE_GPS, payload, sizeof(payload));
}

int telemetry_send_status(const SystemStatus *status) {
    if (!status) return -1;

    /*
     * Payload STATUS: arm_state(1) + error_flags(2) + uptime_ms(4) = 7 byte
     */
    uint8_t payload[7];
    payload[0] = status->arm_state;
    write_u16_le(&payload[1], status->error_flags);
    write_u32_le(&payload[3], status->uptime_ms);

    return send_packet(TELEM_TYPE_STATUS, payload, sizeof(payload));
}

uint8_t telemetry_checksum(const uint8_t *buf, uint16_t len) {
    uint8_t csum = 0;
    for (uint16_t i = 0; i < len; i++) {
        csum ^= buf[i];
    }
    return csum;
}

void telemetry_print_hex(const uint8_t *buf, uint16_t len) {
    printf("[TELEM HEX] ");
    for (uint16_t i = 0; i < len; i++) {
        printf("%02X ", buf[i]);
    }
    printf("\n");
}
