/**
 * @file test_runner.c
 * @brief Entry point del test suite firmware.
 *
 * Raccoglie ed esegue tutti i test unitari dei moduli firmware.
 * Il codice di uscita è 0 se tutti i test passano, 1 altrimenti —
 * compatibile con i sistemi CI/CD (GitHub Actions, Jenkins).
 */

#include "test_framework.h"

/* Dichiarazioni esterne dei test (definiti negli altri file) */

/* test_sensors.c */
extern void test_sensors_init_no_crash(void);
extern void test_imu_null_pointer_safe(void);
extern void test_imu_az_near_gravity_at_rest(void);
extern void test_imu_ax_ay_near_zero_at_rest(void);
extern void test_imu_gyro_near_zero_at_rest(void);
extern void test_gps_null_pointer_safe(void);
extern void test_gps_default_position_near_milano(void);
extern void test_gps_altitude_reflects_set_value(void);
extern void test_baro_null_pointer_safe(void);
extern void test_baro_pressure_decreases_with_altitude(void);
extern void test_baro_altitude_correlates_with_set_altitude(void);

/* test_telemetry.c */
extern void test_checksum_empty_payload_is_zero(void);
extern void test_checksum_single_byte(void);
extern void test_checksum_known_sequence(void);
extern void test_checksum_self_inverse(void);
extern void test_checksum_null_pointer_safe(void);
extern void test_init_valid_path_returns_zero(void);
extern void test_init_invalid_path_returns_minus_one(void);
extern void test_close_without_init_no_crash(void);
extern void test_send_imu_returns_correct_size(void);
extern void test_send_gps_returns_correct_size(void);
extern void test_send_status_returns_correct_size(void);
extern void test_send_null_pointer_returns_minus_one(void);
extern void test_packet_starts_with_sync_byte(void);

/* test_scheduler.c */
extern void test_add_first_task_returns_zero(void);
extern void test_add_second_task_returns_one(void);
extern void test_add_task_max_limit_returns_minus_one(void);
extern void test_add_task_null_callback_rejected(void);
extern void test_reinit_clears_all_tasks(void);
extern void test_zero_period_task_accepted(void);

int main(void) {
    printf("==========================================\n");
    printf("  AeroSim Firmware Test Suite\n");
    printf("  Competenza: Verifica e Validazione\n");
    printf("==========================================\n\n");

    /* ---- Sensori ---- */
    printf("--- Modulo: sensors ---\n");
    RUN_TEST(sensors_init_no_crash);
    RUN_TEST(imu_null_pointer_safe);
    RUN_TEST(imu_az_near_gravity_at_rest);
    RUN_TEST(imu_ax_ay_near_zero_at_rest);
    RUN_TEST(imu_gyro_near_zero_at_rest);
    RUN_TEST(gps_null_pointer_safe);
    RUN_TEST(gps_default_position_near_milano);
    RUN_TEST(gps_altitude_reflects_set_value);
    RUN_TEST(baro_null_pointer_safe);
    RUN_TEST(baro_pressure_decreases_with_altitude);
    RUN_TEST(baro_altitude_correlates_with_set_altitude);

    /* ---- Telemetria ---- */
    printf("\n--- Modulo: telemetry ---\n");
    RUN_TEST(checksum_empty_payload_is_zero);
    RUN_TEST(checksum_single_byte);
    RUN_TEST(checksum_known_sequence);
    RUN_TEST(checksum_self_inverse);
    RUN_TEST(checksum_null_pointer_safe);
    RUN_TEST(init_valid_path_returns_zero);
    RUN_TEST(init_invalid_path_returns_minus_one);
    RUN_TEST(close_without_init_no_crash);
    RUN_TEST(send_imu_returns_correct_size);
    RUN_TEST(send_gps_returns_correct_size);
    RUN_TEST(send_status_returns_correct_size);
    RUN_TEST(send_null_pointer_returns_minus_one);
    RUN_TEST(packet_starts_with_sync_byte);

    /* ---- Scheduler ---- */
    printf("\n--- Modulo: scheduler ---\n");
    RUN_TEST(add_first_task_returns_zero);
    RUN_TEST(add_second_task_returns_one);
    RUN_TEST(add_task_max_limit_returns_minus_one);
    RUN_TEST(add_task_null_callback_rejected);
    RUN_TEST(reinit_clears_all_tasks);
    RUN_TEST(zero_period_task_accepted);

    PRINT_SUMMARY();
    return (g_failures > 0) ? 1 : 0;
}
