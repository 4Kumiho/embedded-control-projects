/**
 * @file drone_c_api.cpp
 * @brief Implementazione dell'interfaccia C per il physics engine.
 *
 * Questo file è compilato come C++ (ha accesso alle classi),
 * ma espone simboli con linkage C (nessun name mangling).
 */

#include "drone_c_api.h"
#include "Drone.h"
#include "Environment.h"

#include <cmath>

/* ------------------------------------------------------------------ */
/*  Istanze globali (singleton — pattern comune nei sistemi embedded)  */
/* ------------------------------------------------------------------ */

static Environment *g_env  = nullptr;
static Drone       *g_drone = nullptr;

/* ------------------------------------------------------------------ */
/*  Implementazione                                                    */
/* ------------------------------------------------------------------ */

void drone_init(float wind_x, float wind_y) {
    /* Pulizia istanze precedenti (nel caso di un reset completo) */
    delete g_drone;
    delete g_env;

    g_env   = new Environment();
    g_env->set_base_wind(Vec3{(double)wind_x, (double)wind_y, 0.0});
    g_env->set_gust_intensity(0.3);

    DroneParams params = DroneParams::default_params();
    g_drone = new Drone(params, *g_env);
}

void drone_step(uint32_t dt_ms) {
    if (!g_drone) return;
    double dt = (double)dt_ms / 1000.0;   /* ms → s */
    g_drone->step(dt);
}

void drone_set_motors(const CMotorCommands *cmd) {
    if (!g_drone || !cmd) return;

    MotorCommands mc;
    for (int i = 0; i < 4; i++) {
        mc.omega[i] = (double)cmd->omega[i];
    }
    g_drone->set_motors(mc);
}

void drone_get_state(CDroneState *out) {
    if (!g_drone || !out) return;

    const DroneState &s = g_drone->state();

    out->pos_x = (float)s.position.x;
    out->pos_y = (float)s.position.y;
    out->pos_z = (float)s.position.z;

    out->vel_x = (float)s.velocity.x;
    out->vel_y = (float)s.velocity.y;
    out->vel_z = (float)s.velocity.z;

    /* Conversione da radianti a gradi per comodità del firmware */
    constexpr double RAD2DEG = 180.0 / M_PI;
    out->roll  = (float)(s.euler.x * RAD2DEG);
    out->pitch = (float)(s.euler.y * RAD2DEG);
    out->yaw   = (float)(s.euler.z * RAD2DEG);

    out->roll_rate  = (float)(s.omega.x * RAD2DEG);
    out->pitch_rate = (float)(s.omega.y * RAD2DEG);
    out->yaw_rate   = (float)(s.omega.z * RAD2DEG);
}

int drone_is_crashed(void) {
    if (!g_drone) return 0;
    return g_drone->is_crashed() ? 1 : 0;
}

void drone_reset(void) {
    if (!g_drone) return;
    g_drone->reset();
}
