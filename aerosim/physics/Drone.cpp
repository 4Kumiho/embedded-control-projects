/**
 * @file Drone.cpp
 * @brief Implementazione del modello fisico del quadricottero.
 *
 * Integrazione numerica con Runge-Kutta 4° ordine (RK4):
 *
 *   k1 = f(t,  y)
 *   k2 = f(t + dt/2,  y + dt/2 * k1)
 *   k3 = f(t + dt/2,  y + dt/2 * k2)
 *   k4 = f(t + dt,    y + dt   * k3)
 *
 *   y(t+dt) = y(t) + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
 *
 * RK4 ha errore locale O(dt^5) — molto più accurato di Eulero O(dt^2).
 * È il metodo standard in simulatori di volo (X-Plane, JSBSim, ArduPilot SITL).
 */

#include "Drone.h"

#include <cmath>
#include <algorithm>

/* ------------------------------------------------------------------ */
/*  Costruttore e reset                                                */
/* ------------------------------------------------------------------ */

Drone::Drone(const DroneParams &params, Environment &env)
    : m_params(params)
    , m_env(env)
{
    reset();
}

void Drone::reset(const DroneState &initial_state) {
    m_state = initial_state;

    /* Motori a regime di hovering: spinta = peso / 4 motori
     * kT * omega_hover^2 = m*g/4  → omega_hover = sqrt(m*g / (4*kT))
     */
    double omega_hover = std::sqrt(m_params.mass * 9.81 /
                                   (4.0 * m_params.kT));
    for (int i = 0; i < 4; i++) {
        m_motors.omega[i] = omega_hover;
    }
}

/* ------------------------------------------------------------------ */
/*  API pubblica                                                        */
/* ------------------------------------------------------------------ */

void Drone::set_motors(const MotorCommands &cmd) {
    for (int i = 0; i < 4; i++) {
        /* Clamp: velocità motore non può essere negativa */
        m_motors.omega[i] = std::max(0.0, cmd.omega[i]);
    }
}

bool Drone::is_crashed() const {
    /* Quota sotto il suolo */
    if (m_state.position.z < 0.0) return true;

    /* Angoli eccessivi (capovolgimento) */
    constexpr double MAX_ANGLE_RAD = 75.0 * M_PI / 180.0;
    if (std::abs(m_state.euler.x) > MAX_ANGLE_RAD) return true;  /* roll  */
    if (std::abs(m_state.euler.y) > MAX_ANGLE_RAD) return true;  /* pitch */
    return false;
}

/* ------------------------------------------------------------------ */
/*  Integratore RK4                                                    */
/* ------------------------------------------------------------------ */

void Drone::step(double dt) {
    /*
     * RK4: valutiamo la derivata f(stato) in 4 punti diversi
     * e combiniamo con pesi (1/6, 1/3, 1/3, 1/6).
     */
    DroneState k1 = compute_derivative(m_state,          m_motors);
    DroneState k2 = compute_derivative(state_add(m_state, k1, dt*0.5), m_motors);
    DroneState k3 = compute_derivative(state_add(m_state, k2, dt*0.5), m_motors);
    DroneState k4 = compute_derivative(state_add(m_state, k3, dt),     m_motors);

    /* Aggiorna lo stato: y += dt/6 * (k1 + 2*k2 + 2*k3 + k4) */
    m_state.position += (k1.position + 2.0*k2.position +
                         2.0*k3.position + k4.position) * (dt / 6.0);
    m_state.velocity += (k1.velocity + 2.0*k2.velocity +
                         2.0*k3.velocity + k4.velocity) * (dt / 6.0);
    m_state.euler    += (k1.euler    + 2.0*k2.euler    +
                         2.0*k3.euler    + k4.euler)    * (dt / 6.0);
    m_state.omega    += (k1.omega    + 2.0*k2.omega    +
                         2.0*k3.omega    + k4.omega)    * (dt / 6.0);

    /* Clamp quota: il suolo è inelastico (nessun rimbalzo) */
    if (m_state.position.z < 0.0) {
        m_state.position.z = 0.0;
        if (m_state.velocity.z < 0.0) m_state.velocity.z = 0.0;
    }
}

/* ------------------------------------------------------------------ */
/*  Derivata dello stato (cuore del physics engine)                    */
/* ------------------------------------------------------------------ */

DroneState Drone::compute_derivative(const DroneState &s,
                                     const MotorCommands &motors) const {
    DroneState ds;

    /* --- 1. Matrice di rotazione corpo -> mondo --- */
    double R[3][3];
    rotation_matrix(s.euler, R);

    /* --- 2. Forza e coppia dai motori (frame corpo) --- */
    Vec3 motor_force, motor_torque;
    compute_motor_wrench(motors, motor_force, motor_torque);

    /* --- 3. Resistenza aerodinamica (drag lineare) ---
     *
     * F_drag = -0.5 * rho * Cd * A * |v|^2 * v_hat
     * Semplificato come forza proporzionale alla velocità (drag lineare):
     * F_drag = -k_drag * v
     * Valido per numeri di Reynolds bassi (volo lento).
     */
    Vec3 wind    = m_env.wind_velocity();
    Vec3 rel_vel = s.velocity - wind;   /* velocità relativa all'aria */
    Vec3 drag    = rel_vel * (-m_params.drag_coeff);

    /* --- 4. Forza totale nel frame mondo ---
     *
     * F_world = R * F_body + m*g + F_drag
     * La spinta (motor_force) è nel frame corpo → moltiplico per R.
     */
    Vec3 thrust_world{
        R[0][0]*motor_force.x + R[0][1]*motor_force.y + R[0][2]*motor_force.z,
        R[1][0]*motor_force.x + R[1][1]*motor_force.y + R[1][2]*motor_force.z,
        R[2][0]*motor_force.x + R[2][1]*motor_force.y + R[2][2]*motor_force.z
    };

    Vec3 gravity = m_env.gravity() * m_params.mass;
    Vec3 F_total = thrust_world + gravity + drag;

    /* --- 5. Equazioni di Newton: traslazione ---
     *
     * p_dot = v
     * v_dot = F_total / m
     */
    ds.position = s.velocity;
    ds.velocity = F_total * (1.0 / m_params.mass);

    /* --- 6. Equazioni di Eulero: rotazione ---
     *
     * eta_dot = W(eta) * omega
     */
    ds.euler = omega_to_euler_rate(s.euler, s.omega);

    /* --- 7. Dinamica angolare (equazioni di Eulero per corpo rigido) ---
     *
     * I * omega_dot = M - omega x (I * omega)
     *
     * omega_dot = I^-1 * [M - omega x (I*omega)]
     *
     * Per un drone simmetrico I è diagonale, quindi I^-1 è banale.
     */
    Vec3 I_omega{ m_params.Ixx * s.omega.x,
                  m_params.Iyy * s.omega.y,
                  m_params.Izz * s.omega.z };

    Vec3 gyro = s.omega.cross(I_omega);   /* omega x (I*omega) */
    Vec3 net_torque = motor_torque - gyro;

    ds.omega = Vec3{
        net_torque.x / m_params.Ixx,
        net_torque.y / m_params.Iyy,
        net_torque.z / m_params.Izz
    };

    return ds;
}

/* ------------------------------------------------------------------ */
/*  Geometria e cinematica                                             */
/* ------------------------------------------------------------------ */

void Drone::rotation_matrix(const Vec3 &euler, double R[3][3]) const {
    /*
     * Convenzione ZYX (aerospace): prima yaw, poi pitch, poi roll.
     * R = Rz(psi) * Ry(theta) * Rx(phi)
     */
    double phi   = euler.x;   /* roll  */
    double theta = euler.y;   /* pitch */
    double psi   = euler.z;   /* yaw   */

    double cphi = std::cos(phi),   sphi = std::sin(phi);
    double cth  = std::cos(theta), sth  = std::sin(theta);
    double cpsi = std::cos(psi),   spsi = std::sin(psi);

    R[0][0] =  cpsi*cth;
    R[0][1] =  cpsi*sth*sphi - spsi*cphi;
    R[0][2] =  cpsi*sth*cphi + spsi*sphi;

    R[1][0] =  spsi*cth;
    R[1][1] =  spsi*sth*sphi + cpsi*cphi;
    R[1][2] =  spsi*sth*cphi - cpsi*sphi;

    R[2][0] = -sth;
    R[2][1] =  cth*sphi;
    R[2][2] =  cth*cphi;
}

Vec3 Drone::omega_to_euler_rate(const Vec3 &euler, const Vec3 &omega) const {
    /*
     * Matrice di trasformazione W (da velocità angolari corpo a rate Eulero):
     *
     *   phi_dot   = p + (q*sin(phi) + r*cos(phi)) * tan(theta)
     *   theta_dot = q*cos(phi) - r*sin(phi)
     *   psi_dot   = (q*sin(phi) + r*cos(phi)) / cos(theta)
     *
     * ATTENZIONE: singolare per theta = ±90° (gimbal lock).
     * In un sistema reale si usano i quaternioni per evitare questo problema.
     */
    double phi   = euler.x;
    double theta = euler.y;

    double sphi = std::sin(phi), cphi = std::cos(phi);
    double tth  = std::tan(theta);
    double cth  = std::cos(theta);

    /* Evita divisione per zero vicino a gimbal lock */
    if (std::abs(cth) < 1e-6) cth = 1e-6;

    double p = omega.x, q = omega.y, r = omega.z;

    return Vec3{
        p + (q*sphi + r*cphi) * tth,
        q*cphi - r*sphi,
        (q*sphi + r*cphi) / cth
    };
}

void Drone::compute_motor_wrench(const MotorCommands &motors,
                                  Vec3 &force,
                                  Vec3 &torque) const {
    /*
     * Ogni motore i produce:
     *   Ti = kT * omega_i^2   (spinta verso +Z corpo)
     *   Qi = kD * omega_i^2   (coppia di reazione, segno dipende dalla rotazione)
     *
     * Rotazioni (vista dall'alto):
     *   M0 (front-left,  +x+y): CCW → Qi negativa
     *   M1 (front-right, +x-y): CW  → Qi positiva
     *   M2 (rear-right,  -x-y): CCW → Qi negativa
     *   M3 (rear-left,   -x+y): CW  → Qi positiva
     *
     * Convenzione segni coppie per il yaw:
     *   CCW in vista dall'alto → coppia yaw su drone negativa (reazione)
     *   CW  in vista dall'alto → coppia yaw su drone positiva (reazione)
     */
    double l  = m_params.arm_len;
    double kT = m_params.kT;
    double kD = m_params.kD;

    double T[4];
    for (int i = 0; i < 4; i++) {
        T[i] = kT * motors.omega[i] * motors.omega[i];
    }

    /* Spinta totale: somma delle 4 spinte verso +Z corpo */
    force = Vec3{0.0, 0.0, T[0] + T[1] + T[2] + T[3]};

    /*
     * Coppie (torques) nel frame corpo:
     *
     *   Roll  (attorno X): differenza spinta sinistra vs destra
     *     L = l * (T[1] + T[2] - T[0] - T[3])
     *     Positivo = rollio a destra (roll right)
     *
     *   Pitch (attorno Y): differenza spinta posteriore vs anteriore
     *     M = l * (T[0] + T[1] - T[2] - T[3])
     *     Positivo = cabrata (pitch up)
     *
     *   Yaw   (attorno Z): differenza coppia CW vs CCW
     *     N = kD * (T[1] + T[3] - T[0] - T[2])
     *     Positivo = imbardata destra (yaw right)
     */
    torque = Vec3{
        l * (T[1] + T[2] - T[0] - T[3]),   /* roll  */
        l * (T[0] + T[1] - T[2] - T[3]),   /* pitch */
        kD/kT * (T[1] + T[3] - T[0] - T[2]) /* yaw  */
    };
}

/* ------------------------------------------------------------------ */
/*  Utilità RK4                                                        */
/* ------------------------------------------------------------------ */

DroneState Drone::state_add(const DroneState &s,
                             const DroneState &ds,
                             double scale) {
    DroneState result;
    result.position = s.position + ds.position * scale;
    result.velocity = s.velocity + ds.velocity * scale;
    result.euler    = s.euler    + ds.euler    * scale;
    result.omega    = s.omega    + ds.omega    * scale;
    return result;
}
