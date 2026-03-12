/**
 * @file test_telemetry.c
 * @brief Test unitari per il modulo telemetry.c
 *
 * Tecnica: black-box testing — testiamo il comportamento esterno
 * senza accedere agli internals (come farebbe un tester V&V esterno).
 * Questo segue il principio DO-178C per la verifica del software avionic.
 */

#include "test_framework.h"
#include "telemetry.h"
#include "sensors.h"

#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  Test: checksum XOR                                                 */
/* ------------------------------------------------------------------ */

TEST(checksum_empty_payload_is_zero) {
    /*
     * XOR di zero elementi = 0 per definizione.
     * Caso limite importante: pacchetto STATUS con payload vuoto
     * non dovrebbe essere inviato, ma il checksum deve essere definito.
     */
    uint8_t buf[1] = {0};
    uint8_t csum = telemetry_checksum(buf, 0);
    ASSERT_EQ(0x00, csum);
}

TEST(checksum_single_byte) {
    /* XOR di un singolo byte = il byte stesso */
    uint8_t buf[1] = {0xAB};
    ASSERT_EQ(0xAB, telemetry_checksum(buf, 1));
}

TEST(checksum_known_sequence) {
    /*
     * Sequenza nota: 0x01 ^ 0x02 ^ 0x03 = 0x00
     * Utile per verificare la correttezza dell'implementazione XOR.
     */
    uint8_t buf[3] = {0x01, 0x02, 0x03};
    ASSERT_EQ(0x00, telemetry_checksum(buf, 3));
}

TEST(checksum_self_inverse) {
    /*
     * Proprietà XOR: A ^ A = 0.
     * Se il checksum è corretto, XOR di (payload + checksum) = 0.
     * Questa proprietà è usata dal receiver per validare i pacchetti.
     */
    uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t csum = telemetry_checksum(payload, 4);

    /* Aggiungi il checksum al buffer e ricalcola */
    uint8_t full[5];
    memcpy(full, payload, 4);
    full[4] = csum;

    ASSERT_EQ(0x00, telemetry_checksum(full, 5));
}

TEST(checksum_null_pointer_safe) {
    /* NULL con len=0 non deve crashare */
    uint8_t result = telemetry_checksum(NULL, 0);
    ASSERT_EQ(0x00, result);
}

/* ------------------------------------------------------------------ */
/*  Test: init e close                                                 */
/* ------------------------------------------------------------------ */

TEST(init_valid_path_returns_zero) {
    int ret = telemetry_init("test_output.bin");
    ASSERT_EQ(0, ret);
    telemetry_close();
    remove("test_output.bin");
}

TEST(init_invalid_path_returns_minus_one) {
    /* Percorso non scrivibile */
    int ret = telemetry_init("/nonexistent/path/test.bin");
    ASSERT_EQ(-1, ret);
}

TEST(close_without_init_no_crash) {
    /* Doppia close non deve crashare */
    telemetry_close();
    telemetry_close();
    ASSERT_TRUE(1);
}

/* ------------------------------------------------------------------ */
/*  Test: invio pacchetti                                              */
/* ------------------------------------------------------------------ */

TEST(send_imu_returns_correct_size) {
    /*
     * Dimensione attesa:
     *   header: 4 byte (START + TYPE + LENGTH_LO + LENGTH_HI)
     *   payload: 24 byte (6 float × 4 byte)
     *   checksum: 1 byte
     *   totale: 29 byte
     */
    telemetry_init("test_imu.bin");

    IMUData d = {0.1f, 0.2f, 9.81f, 0.01f, -0.02f, 0.0f};
    int written = telemetry_send_imu(&d);

    ASSERT_EQ(4 + 24 + 1, written);   /* 29 byte */
    telemetry_close();
    remove("test_imu.bin");
}

TEST(send_gps_returns_correct_size) {
    /*
     * GPS payload: 3 double (24 B) + 1 float (4 B) = 28 byte
     * Totale pacchetto: 4 + 28 + 1 = 33 byte
     */
    telemetry_init("test_gps.bin");

    GPSData g = {45.4654, 9.1859, 100.0, 5.0f};
    int written = telemetry_send_gps(&g);

    ASSERT_EQ(4 + 28 + 1, written);   /* 33 byte */
    telemetry_close();
    remove("test_gps.bin");
}

TEST(send_status_returns_correct_size) {
    /*
     * STATUS payload: uint8(1) + uint16(2) + uint32(4) = 7 byte
     * Totale pacchetto: 4 + 7 + 1 = 12 byte
     */
    telemetry_init("test_status.bin");

    SystemStatus s = {1, 0x0000, 5000};
    int written = telemetry_send_status(&s);

    ASSERT_EQ(4 + 7 + 1, written);   /* 12 byte */
    telemetry_close();
    remove("test_status.bin");
}

TEST(send_null_pointer_returns_minus_one) {
    telemetry_init("test_null.bin");

    ASSERT_EQ(-1, telemetry_send_imu(NULL));
    ASSERT_EQ(-1, telemetry_send_gps(NULL));
    ASSERT_EQ(-1, telemetry_send_status(NULL));

    telemetry_close();
    remove("test_null.bin");
}

TEST(packet_starts_with_sync_byte) {
    /*
     * Verifica che il primo byte del file sia 0xAE.
     * Test di integrazione: scriviamo un pacchetto e rileggiamo il file.
     */
    const char *path = "test_sync.bin";
    telemetry_init(path);

    IMUData d = {0};
    telemetry_send_imu(&d);
    telemetry_close();

    FILE *f = fopen(path, "rb");
    if (f) {
        uint8_t first_byte;
        fread(&first_byte, 1, 1, f);
        fclose(f);
        ASSERT_EQ(0xAE, first_byte);
    } else {
        ASSERT_TRUE(0);   /* file non aperto = fail */
    }
    remove(path);
}
