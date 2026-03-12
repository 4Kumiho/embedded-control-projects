/**
 * @file main.c
 * @brief Entry point del firmware simulator.
 *
 * Simula il comportamento di un firmware embedded su MCU:
 *   1. Inizializzazione periferiche (sensori, telemetria)
 *   2. Registrazione task sullo scheduler
 *   3. Avvio del main loop (simula il loop infinito del firmware)
 *
 * Su un MCU reale (es. STM32) il main sarebbe identico a questo;
 * cambierebbe solo l'implementazione di platform.h.
 */

#include "sensors.h"
#include "scheduler.h"
#include "telemetry.h"
#include "platform.h"

#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  Stato globale condiviso tra i task                                  */
/* ------------------------------------------------------------------ */

static IMUData      g_imu;
static GPSData      g_gps;
static BaroData     g_baro;
static SystemStatus g_status;

/* Quota simulata: sale linearmente per testare i sensori */
static float g_sim_altitude = 0.0f;

/* ------------------------------------------------------------------ */
/*  Callback dei task                                                   */
/* ------------------------------------------------------------------ */

/**
 * @brief Task 1 — Lettura sensori (ogni 10ms = 100 Hz).
 *
 * Frequenza tipica per un IMU in applicazioni di controllo assetto.
 * Il barometro e il GPS girano piu' lenti (vedi task dedicati).
 */
static void task_read_imu(void) {
    sensors_set_altitude(g_sim_altitude);
    sensors_read_imu(&g_imu);
}

/**
 * @brief Task 2 — Lettura GPS (ogni 200ms = 5 Hz).
 *
 * I moduli GPS consumer tipicamente aggiornano a 1-10 Hz.
 * Moduli ad alta precisione (es. u-blox F9P) arrivano a 20 Hz.
 */
static void task_read_gps(void) {
    sensors_read_gps(&g_gps);
    sensors_read_baro(&g_baro);

    /* Simulazione volo: quota sale di 0.2m ogni 200ms = 1 m/s */
    g_sim_altitude += 0.2f;
    if (g_sim_altitude > 50.0f) g_sim_altitude = 0.0f; /* reset dopo 50m */
}

/**
 * @brief Task 3 — Trasmissione telemetria (ogni 100ms = 10 Hz).
 *
 * Invia in sequenza: IMU, GPS, STATUS.
 * In un sistema reale questo potrebbe essere guidato da DMA + UART.
 */
static void task_send_telemetry(void) {
    static uint32_t uptime = 0;
    uptime += 100; /* incrementa di 100ms ad ogni chiamata */

    g_status.arm_state   = 1;      /* simuliamo drone armato */
    g_status.error_flags = 0x0000; /* nessun errore          */
    g_status.uptime_ms   = uptime;

    int imu_bytes    = telemetry_send_imu(&g_imu);
    int gps_bytes    = telemetry_send_gps(&g_gps);
    int status_bytes = telemetry_send_status(&g_status);

    printf("[TELEM] Inviato: IMU=%d B, GPS=%d B, STATUS=%d B | "
           "Alt=%.1fm Up=%ums\n",
           imu_bytes, gps_bytes, status_bytes,
           g_sim_altitude, uptime);
}

/**
 * @brief Task 4 — Watchdog (ogni 500ms).
 *
 * Su MCU reale refresha il registro WWDG/IWDG per evitare il reset.
 * Qui stampa solo un heartbeat.
 */
static void task_watchdog(void) {
    static uint32_t wd_count = 0;
    wd_count++;
    printf("[WDG] Heartbeat #%u | Quota: %.1f m\n", wd_count, g_sim_altitude);
}

/* ------------------------------------------------------------------ */
/*  Main                                                                */
/* ------------------------------------------------------------------ */

int main(void) {
    printf("=============================================\n");
    printf("  AeroSim Firmware Simulator v1.0\n");
    printf("  Competenza: SW Embedded (C) + FW\n");
    printf("=============================================\n\n");

    /* 1. Init sensori */
    sensors_init(42); /* seed fisso per riproducibilita' */

    /* 2. Init telemetria (scrive su file binario) */
    if (telemetry_init("telemetry.bin") != 0) {
        printf("ERRORE: impossibile aprire file telemetria\n");
        return 1;
    }

    /* 3. Init scheduler */
    scheduler_init();

    /* 4. Registrazione task (ordine = priorita' implicita) */
    scheduler_add_task("imu_read",    10,  task_read_imu);
    scheduler_add_task("gps_read",    200, task_read_gps);
    scheduler_add_task("telemetry",   100, task_send_telemetry);
    scheduler_add_task("watchdog",    500, task_watchdog);

    printf("\n[MAIN] Avvio simulazione per 5 secondi...\n\n");

    /* 5. Main loop — gira per 5000ms poi stampa statistiche ed esce */
    scheduler_run(5000);

    /* 6. Cleanup */
    telemetry_close();
    printf("[MAIN] File telemetria: telemetry.bin\n");
    printf("[MAIN] Simulazione completata.\n");

    return 0;
}
