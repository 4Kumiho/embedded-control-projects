/**
 * @file drone_c_api.h
 * @brief Interfaccia C per il physics engine C++.
 *
 * Il firmware è scritto in C puro e non può includere header C++.
 * Questo file espone le funzioni del Drone come funzioni C normali,
 * usando il meccanismo extern "C" di C++.
 *
 * Pattern usato nei sistemi embedded reali per separare:
 *   - Logica applicativa in C++ (OOP, template, STL)
 *   - Interfaccia di sistema in C (ABI stabile, compatibilità)
 *
 * Esempio reale: FreeRTOS è scritto in C ma ha wrapper C++ per C++11/14.
 */

#ifndef AEROSIM_DRONE_C_API_H
#define AEROSIM_DRONE_C_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Strutture C-compatible (senza classi o riferimenti C++)            */
/* ------------------------------------------------------------------ */

/** Stato del drone esposto al firmware C */
typedef struct {
    float pos_x, pos_y, pos_z;       /**< Posizione [m]              */
    float vel_x, vel_y, vel_z;       /**< Velocita' [m/s]            */
    float roll, pitch, yaw;          /**< Angoli di Eulero [gradi]   */
    float roll_rate, pitch_rate, yaw_rate; /**< Velocita' angolari [deg/s] */
} CDroneState;

/** Comandi motori esposti al firmware C */
typedef struct {
    float omega[4];                   /**< Velocita' angolare [rad/s] */
} CMotorCommands;

/* ------------------------------------------------------------------ */
/*  API C                                                               */
/* ------------------------------------------------------------------ */

/**
 * @brief Inizializza il physics engine con parametri di default.
 * @param wind_x, wind_y  Componenti del vento costante [m/s].
 */
void drone_init(float wind_x, float wind_y);

/**
 * @brief Avanza la simulazione di dt_ms millisecondi.
 * @param dt_ms  Passo di integrazione in millisecondi.
 */
void drone_step(uint32_t dt_ms);

/**
 * @brief Imposta i comandi ai 4 motori.
 * @param cmd  Puntatore ai comandi motore.
 */
void drone_set_motors(const CMotorCommands *cmd);

/**
 * @brief Legge lo stato corrente del drone.
 * @param out  Puntatore alla struttura da riempire.
 */
void drone_get_state(CDroneState *out);

/**
 * @brief Controlla se il drone e' in stato di crash.
 * @return  1 se crashato, 0 altrimenti.
 */
int drone_is_crashed(void);

/**
 * @brief Resetta il drone alla posizione iniziale.
 */
void drone_reset(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* AEROSIM_DRONE_C_API_H */
