/**
 * @file Vec3.h
 * @brief Vettore 3D a virgola mobile con operatori algebrici.
 *
 * Header-only: non richiede file .cpp.
 * Usato ovunque nel physics engine per posizioni, velocità, forze e momenti.
 */

#ifndef AEROSIM_VEC3_H
#define AEROSIM_VEC3_H

#include <cmath>
#include <ostream>

struct Vec3 {
    double x, y, z;

    /* Costruttori */
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    /* Operatori aritmetici */
    Vec3 operator+(const Vec3 &o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3 &o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s)      const { return {x*s,   y*s,   z*s  }; }
    Vec3 operator/(double s)      const { return {x/s,   y/s,   z/s  }; }

    Vec3 &operator+=(const Vec3 &o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3 &operator*=(double s)      { x*=s;   y*=s;   z*=s;   return *this; }

    /* Prodotto scalare (dot product) */
    double dot(const Vec3 &o) const { return x*o.x + y*o.y + z*o.z; }

    /* Prodotto vettoriale (cross product) */
    Vec3 cross(const Vec3 &o) const {
        return { y*o.z - z*o.y,
                 z*o.x - x*o.z,
                 x*o.y - y*o.x };
    }

    /* Modulo (norma euclidea) */
    double norm() const { return std::sqrt(x*x + y*y + z*z); }

    /* Versore (vettore normalizzato) */
    Vec3 normalized() const {
        double n = norm();
        return (n > 1e-10) ? (*this / n) : Vec3{};
    }

    /* Stampa */
    friend std::ostream &operator<<(std::ostream &os, const Vec3 &v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
};

/* Prodotto scalare * vettore (commutatività) */
inline Vec3 operator*(double s, const Vec3 &v) { return v * s; }

#endif /* AEROSIM_VEC3_H */
