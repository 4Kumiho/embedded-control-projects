/**
 * @file Environment.cpp
 * @brief Implementazione del modello ambientale.
 */

#include "Environment.h"

#include <cmath>
#include <cstdlib>

/* ------------------------------------------------------------------ */
/*  Funzione privata: rumore gaussiano (Box-Muller)                    */
/*  Stessa tecnica usata in sensors.c — pattern ricorrente in sim.    */
/* ------------------------------------------------------------------ */

static double gaussian_noise(double std) {
    double u1, u2;
    do { u1 = (double)rand() / RAND_MAX; } while (u1 == 0.0);
    u2 = (double)rand() / RAND_MAX;
    double z = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
    return std * z;
}

/* ------------------------------------------------------------------ */
/*  Implementazione                                                    */
/* ------------------------------------------------------------------ */

Environment::Environment()
    : m_base_wind{0.0, 0.0, 0.0}
    , m_gust_intensity{0.5}              /* raffica leggera di default */
{}

Vec3 Environment::gravity() const {
    /*
     * Frame ENU (East-North-Up): Z positivo verso l'alto.
     * La gravità agisce verso il basso → componente Z negativa.
     */
    return {0.0, 0.0, -GRAVITY};
}

double Environment::air_density(double altitude_m) const {
    /*
     * Modello esponenziale ISA (ottima approssimazione < 10 km):
     *   rho(h) = rho0 * exp(-h / H_scale)
     *
     * A 0m:    1.225 kg/m³
     * A 1000m: 1.112 kg/m³
     * A 3000m: 0.909 kg/m³
     */
    if (altitude_m < 0.0) altitude_m = 0.0;
    return RHO0 * std::exp(-altitude_m / SCALE_H);
}

Vec3 Environment::wind_velocity() {
    /*
     * Modello Dryden semplificato:
     * Il vento totale = componente stazionaria + raffica casuale.
     * Le raffiche sono campionate da distribuzione gaussiana —
     * in un modello completo andrebbero filtrate con un filtro passa-basso
     * per simulare la correlazione temporale delle turbolenze.
     */
    Vec3 gust{
        gaussian_noise(m_gust_intensity),
        gaussian_noise(m_gust_intensity),
        gaussian_noise(m_gust_intensity * 0.3)   /* vento verticale più debole */
    };
    return m_base_wind + gust;
}

void Environment::set_base_wind(const Vec3 &wind) {
    m_base_wind = wind;
}

void Environment::set_gust_intensity(double intensity) {
    m_gust_intensity = intensity;
}
