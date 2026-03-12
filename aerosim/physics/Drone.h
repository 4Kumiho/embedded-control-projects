/**
 * @file Drone.h
 * @brief Modello dinamico di un quadricottore — corpo rigido 6DOF.
 *
 * Equazioni del moto (Newton-Euler):
 *
 *   Traslazione (frame mondo):
 *     p_dot = v
 *     v_dot = (1/m) * R * F_body + g
 *
 *   Rotazione (frame corpo — equazioni di Eulero):
 *     eta_dot = W(eta) * omega
 *     omega_dot = I^-1 * (M_body - omega x (I * omega))
 *
 * dove:
 *   p     = posizione nel frame mondo [m]
 *   v     = velocità nel frame mondo [m/s]
 *   eta   = angoli di Eulero ZYX: (phi=roll, theta=pitch, psi=yaw) [rad]
 *   omega = velocità angolare nel frame corpo (p, q, r) [rad/s]
 *   R     = matrice di rotazione corpo->mondo
 *   W     = matrice di trasformazione omega->eta_dot
 *   I     = tensore di inerzia del drone [kg*m²]
 *
 * Configurazione motori (vista dall'alto, X-configuration):
 *
 *       M0(CCW)  M1(CW)
 *           \   /
 *            \ /
 *    Fronte --+-- Retro
 *            / \
 *           /   \
 *       M3(CW)  M2(CCW)
 *
 *   M0: front-left  (+x, +y)   CCW → coppia yaw negativa
 *   M1: front-right (+x, -y)   CW  → coppia yaw positiva
 *   M2: rear-right  (-x, -y)   CCW → coppia yaw negativa
 *   M3: rear-left   (-x, +y)   CW  → coppia yaw positiva
 */

#ifndef AEROSIM_DRONE_H
#define AEROSIM_DRONE_H

#include "Vec3.h"
#include "Environment.h"

/* ------------------------------------------------------------------ */
/*  Strutture dati                                                      */
/* ------------------------------------------------------------------ */

/**
 * @brief Stato completo del drone — vettore di stato 12D.
 */
struct DroneState {
    Vec3 position;      /**< Posizione nel frame mondo ENU [m]        */
    Vec3 velocity;      /**< Velocità nel frame mondo ENU [m/s]       */
    Vec3 euler;         /**< Angoli di Eulero: phi, theta, psi [rad]  */
    Vec3 omega;         /**< Velocità angolare frame corpo [rad/s]    */
};

/**
 * @brief Comandi ai motori (input del controllore PID).
 */
struct MotorCommands {
    double omega[4];    /**< Velocità angolare motori [rad/s], indici 0-3 */
};

/**
 * @brief Parametri fisici del drone (da datasheet o CAD).
 */
struct DroneParams {
    double mass;        /**< Massa totale [kg]                        */
    double arm_len;     /**< Lunghezza braccio motore-centro [m]      */
    double Ixx;         /**< Inerzia attorno asse X [kg*m²]           */
    double Iyy;         /**< Inerzia attorno asse Y [kg*m²]           */
    double Izz;         /**< Inerzia attorno asse Z [kg*m²]           */
    double kT;          /**< Coefficiente di spinta [N/(rad/s)²]      */
    double kD;          /**< Coefficiente di coppia [Nm/(rad/s)²]     */
    double drag_coeff;  /**< Coefficiente aerodinamico di resistenza  */

    /** Parametri tipici di un quadricottero da 250mm (racing) */
    static DroneParams default_params() {
        return { 0.8, 0.175, 0.008, 0.008, 0.016,
                 3.0e-6, 7.5e-8, 0.25 };
    }
};

/* ------------------------------------------------------------------ */
/*  Classe Drone                                                        */
/* ------------------------------------------------------------------ */

class Drone {
public:
    /**
     * @brief Costruisce il drone con i parametri fisici specificati.
     * @param params  Parametri del drone.
     * @param env     Riferimento all'ambiente (gravità, vento).
     */
    Drone(const DroneParams &params, Environment &env);

    /**
     * @brief Avanza la simulazione di dt secondi (integratore RK4).
     * @param dt  Passo di integrazione [s]. Tipicamente 0.001 (1 ms).
     */
    void step(double dt);

    /**
     * @brief Imposta i comandi ai 4 motori.
     * @param cmd  Struttura con le 4 velocità angolari [rad/s].
     */
    void set_motors(const MotorCommands &cmd);

    /**
     * @brief Restituisce lo stato corrente del drone.
     */
    const DroneState &state() const { return m_state; }

    /**
     * @brief Controlla se il drone è in condizione di crash.
     *
     * Crash = quota < 0 oppure angolo > 75°.
     */
    bool is_crashed() const;

    /**
     * @brief Resetta il drone alla posizione iniziale.
     */
    void reset(const DroneState &initial_state = {});

private:
    DroneParams  m_params;
    Environment &m_env;
    DroneState   m_state;
    MotorCommands m_motors;

    /* ----------------------------------------------------------------
     * Metodi privati del motore fisico
     * ---------------------------------------------------------------- */

    /**
     * @brief Calcola la derivata dello stato dato lo stato corrente.
     *
     * Cuore del physics engine: implementa le equazioni di Newton-Euler.
     * Chiamata 4 volte per ogni step RK4.
     */
    DroneState compute_derivative(const DroneState &s,
                                  const MotorCommands &motors) const;

    /**
     * @brief Matrice di rotazione corpo -> mondo (ZYX Euler).
     * @param euler  Angoli phi (roll), theta (pitch), psi (yaw) [rad].
     */
    void rotation_matrix(const Vec3 &euler,
                         double R[3][3]) const;

    /**
     * @brief Trasforma omega (body) in eta_dot (Euler rates).
     *
     * eta_dot = W(eta) * omega
     * La matrice W è singolare per theta = ±90° (gimbal lock).
     */
    Vec3 omega_to_euler_rate(const Vec3 &euler,
                             const Vec3 &omega) const;

    /**
     * @brief Calcola forza totale e coppia dai comandi motore.
     * @param motors     Comandi motori.
     * @param[out] force   Forza totale nel frame corpo [N].
     * @param[out] torque  Coppia totale nel frame corpo [Nm].
     */
    void compute_motor_wrench(const MotorCommands &motors,
                              Vec3 &force,
                              Vec3 &torque) const;

    /** Somma vettoriale pesata per RK4: s + ds*scale */
    static DroneState state_add(const DroneState &s,
                                const DroneState &ds,
                                double scale);
};

#endif /* AEROSIM_DRONE_H */
