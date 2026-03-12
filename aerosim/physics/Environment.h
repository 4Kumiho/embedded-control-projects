/**
 * @file Environment.h
 * @brief Modello ambientale: gravità, atmosfera, vento.
 *
 * Fornisce al physics engine le forze esterne che agiscono sul drone
 * indipendentemente dalla sua dinamica interna.
 */

#ifndef AEROSIM_ENVIRONMENT_H
#define AEROSIM_ENVIRONMENT_H

#include "Vec3.h"

class Environment {
public:
    /**
     * @brief Costruisce l'ambiente con parametri standard ISA
     *        (International Standard Atmosphere).
     */
    Environment();

    /**
     * @brief Restituisce l'accelerazione gravitazionale [m/s²].
     *        Vettore nel frame mondo (ENU): (0, 0, -9.81).
     */
    Vec3 gravity() const;

    /**
     * @brief Calcola la densità dell'aria in funzione dell'altitudine.
     *
     * Modello ISA semplificato (troposfera, valido 0–11 km):
     *   rho(h) = rho0 * (T0 / (T0 - L*h))^(1 + g*M / R*L)
     * Approssimazione: rho ≈ rho0 * exp(-h / H)  con H = 8500 m.
     *
     * @param altitude_m  Quota in metri.
     * @return            Densità aria [kg/m³].
     */
    double air_density(double altitude_m) const;

    /**
     * @brief Restituisce la velocità del vento nel frame mondo [m/s].
     *
     * Il vento include:
     *  - componente costante (direzione e intensità medie)
     *  - raffica casuale aggiornata a ogni chiamata (Dryden model semplificato)
     */
    Vec3 wind_velocity();

    /**
     * @brief Imposta vento costante di fondo.
     * @param wind  Vettore vento [m/s] nel frame mondo.
     */
    void set_base_wind(const Vec3 &wind);

    /**
     * @brief Imposta l'intensità massima delle raffiche [m/s].
     */
    void set_gust_intensity(double intensity);

private:
    Vec3   m_base_wind;       /**< Vento costante di fondo             */
    double m_gust_intensity;  /**< Intensità massima raffica [m/s]     */

    static constexpr double GRAVITY    = 9.81;    /* m/s²   */
    static constexpr double RHO0       = 1.225;   /* kg/m³  — al livello del mare */
    static constexpr double SCALE_H    = 8500.0;  /* m      — altezza scala ISA   */
};

#endif /* AEROSIM_ENVIRONMENT_H */
