/**
 * @file Vec3.h
 * @brief 3D Vector mathematics (header-only library)
 *
 * Provides basic 3D vector operations for physics calculations.
 *
 * Why header-only?
 * ================
 * Vector operations are simple and frequently called.
 * Making them inline (in header) avoids function call overhead.
 * Modern compilers inline these automatically.
 *
 * Usage:
 *   #include "Vec3.h"
 *   Vec3 pos = {0, 0, 0};
 *   Vec3 vel = {1, 2, 3};
 *   Vec3 accel = Vec3_scale(vel, 0.5f);
 *
 * Math library dependency:
 * - Uses <cmath> for sqrt, sin, cos, etc.
 * - Requires C++11 or later
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef VEC3_H
#define VEC3_H

#include <cmath>
#include <cstring>

/**
 * @struct Vec3
 * @brief 3D vector: (x, y, z)
 *
 * Simple POD (Plain Old Data) structure.
 * Can be used in arrays, passed by value, etc.
 *
 * Layout in memory:
 *   Offset 0: float x
 *   Offset 4: float y
 *   Offset 8: float z
 *   Total: 12 bytes
 */
struct Vec3 {
    float x, y, z;

    /* ========== CONSTRUCTORS ========== */

    /** Default constructor: zero vector */
    Vec3() : x(0.0f), y(0.0f), z(0.0f) {}

    /** Construct from components */
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    /** Construct from array [x, y, z] */
    explicit Vec3(const float* arr) : x(arr[0]), y(arr[1]), z(arr[2]) {}

    /* ========== BASIC OPERATIONS ========== */

    /**
     * Vector addition: this + other
     * Element-wise: (x1+x2, y1+y2, z1+z2)
     */
    inline Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    /**
     * Vector subtraction: this - other
     * Element-wise: (x1-x2, y1-y2, z1-z2)
     */
    inline Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    /**
     * Scalar multiplication: this * scalar
     * Each component multiplied by scalar
     * Example: (1, 2, 3) * 2 = (2, 4, 6)
     */
    inline Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    /**
     * Scalar division: this / scalar
     * Each component divided by scalar
     * WARNING: No division-by-zero check!
     * Example: (4, 6, 2) / 2 = (2, 3, 1)
     */
    inline Vec3 operator/(float scalar) const {
        float inv = 1.0f / scalar;  /* Multiply by reciprocal (faster) */
        return Vec3(x * inv, y * inv, z * inv);
    }

    /**
     * Negation: -this
     * Flips all signs: -(x, y, z) = (-x, -y, -z)
     * Used for opposite force, etc.
     */
    inline Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    /**
     * In-place addition: this += other
     * Modifies this vector
     */
    inline Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    /**
     * In-place subtraction: this -= other
     */
    inline Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    /**
     * In-place scalar multiplication: this *= scalar
     */
    inline Vec3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    /**
     * In-place scalar division: this /= scalar
     */
    inline Vec3& operator/=(float scalar) {
        float inv = 1.0f / scalar;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }

    /* ========== DOT AND CROSS PRODUCTS ========== */

    /**
     * Dot product (scalar product): this · other
     *
     * Formula: x1*x2 + y1*y2 + z1*z2
     *
     * Properties:
     * - Returns scalar (float)
     * - Commutative: a·b = b·a
     * - Related to angle: a·b = |a||b|cos(θ)
     * - Positive = vectors point similar directions
     * - Zero = perpendicular
     * - Negative = opposite directions
     *
     * Use cases:
     * - Check if vectors are perpendicular (dot = 0)
     * - Project one vector onto another
     * - Calculate angle between vectors
     *
     * @param other Vector to dot with
     * @return Dot product scalar
     */
    inline float dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    /**
     * Cross product: this × other
     *
     * Formula:
     *   (y1*z2 - z1*y2, z1*x2 - x1*z2, x1*y2 - y1*x2)
     *
     * Properties:
     * - Returns vector (perpendicular to both inputs)
     * - Non-commutative: a×b = -(b×a)
     * - Magnitude: |a×b| = |a||b|sin(θ)
     * - Right-hand rule: curl fingers from a to b, thumb points to a×b
     *
     * Use cases:
     * - Find vector perpendicular to two others
     * - Calculate torque: τ = r × F
     * - Determine if vectors are parallel (cross = 0)
     * - Surface normal in 3D graphics
     *
     * @param other Vector to cross with
     * @return Cross product vector (perpendicular)
     */
    inline Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    /* ========== MAGNITUDE AND NORMALIZATION ========== */

    /**
     * Magnitude (length) of vector: |this|
     *
     * Formula: sqrt(x² + y² + z²)
     *
     * Represents the "length" of the vector in 3D space.
     * Always non-negative.
     *
     * Use cases:
     * - Speed = magnitude of velocity vector
     * - Distance = magnitude of position difference
     * - Force magnitude
     *
     * Performance note:
     * - sqrt() is expensive (~10-100 CPU cycles)
     * - If only comparing magnitudes, use magnitude_squared() instead
     * - Example: |a| > |b| ≡ |a|² > |b|²
     *
     * @return Length of vector (float >= 0)
     */
    inline float magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    /**
     * Squared magnitude: |this|²
     *
     * Formula: x² + y² + z²
     *
     * Faster than magnitude() because no sqrt.
     * Useful when you only need to compare lengths.
     *
     * Example:
     *   // Don't do this (expensive):
     *   if (vec.magnitude() < 1.0f) { ... }
     *
     *   // Do this instead (fast):
     *   if (vec.magnitude_squared() < 1.0f) { ... }
     *
     * @return |this|² (always >= 0)
     */
    inline float magnitude_squared() const {
        return x * x + y * y + z * z;
    }

    /**
     * Unit vector (normalized): this / |this|
     *
     * Returns vector with same direction but magnitude = 1.
     *
     * Formula: v_normalized = v / |v|
     *
     * WARNING: Returns (0,0,0) if magnitude is ~0!
     *          You should check before calling, or handle carefully.
     *
     * Use cases:
     * - Direction vectors (discard magnitude, keep direction)
     * - Unit force directions
     * - Basis vectors for coordinate systems
     *
     * Performance:
     * - Expensive: need sqrt + division
     * - Cache result if used multiple times
     *
     * @return Unit vector (magnitude ≈ 1)
     */
    inline Vec3 normalized() const {
        float mag = magnitude();
        if (mag < 1e-6f) return Vec3(0, 0, 0);  /* Avoid division by zero */
        return *this / mag;
    }

    /**
     * Normalize in-place: make this a unit vector
     *
     * Modifies this vector to have magnitude 1.
     * More efficient than normalized() if you only need the result.
     *
     * @return Reference to this (for chaining)
     */
    inline Vec3& normalize() {
        float mag = magnitude();
        if (mag < 1e-6f) {
            x = y = z = 0;
        } else {
            *this /= mag;
        }
        return *this;
    }

    /* ========== UTILITY FUNCTIONS ========== */

    /**
     * Component-wise multiplication (Hadamard product): this ⊙ other
     *
     * NOT standard dot product!
     * Each component multiplied independently:
     * (x1*x2, y1*y2, z1*z2)
     *
     * Use case: Apply different scaling to each axis
     * Example: (1, 2, 3) ⊙ (2, 0.5, 1) = (2, 1, 3)
     *
     * @param other Vector to multiply component-wise
     * @return Component-wise product
     */
    inline Vec3 component_mult(const Vec3& other) const {
        return Vec3(x * other.x, y * other.y, z * other.z);
    }

    /**
     * Distance to another point
     *
     * Treats both vectors as positions in 3D space.
     * Calculates Euclidean distance: |this - other|
     *
     * Formula: sqrt((x1-x2)² + (y1-y2)² + (z1-z2)²)
     *
     * Use case:
     *   Vec3 pos1 = {0, 0, 0};
     *   Vec3 pos2 = {3, 4, 0};
     *   float dist = pos1.distance_to(pos2);  // = 5
     *
     * @param other Another position
     * @return Distance between points
     */
    inline float distance_to(const Vec3& other) const {
        return (*this - other).magnitude();
    }

    /**
     * Angle between two vectors (in radians)
     *
     * Uses dot product formula: cos(θ) = (a·b) / (|a||b|)
     * Then: θ = acos(...)
     *
     * WARNING: Returns 0 if either vector has ~0 magnitude!
     *
     * Result is always in [0, π] radians ([0°, 180°])
     *
     * @param other Vector to measure angle to
     * @return Angle in radians
     */
    inline float angle_to(const Vec3& other) const {
        float mag_prod = magnitude() * other.magnitude();
        if (mag_prod < 1e-6f) return 0.0f;

        float cos_angle = dot(other) / mag_prod;
        /* Clamp to [-1, 1] to handle floating point errors */
        cos_angle = (cos_angle < -1.0f) ? -1.0f : (cos_angle > 1.0f ? 1.0f : cos_angle);
        return std::acos(cos_angle);
    }

    /**
     * Clamp each component to range [min, max]
     *
     * Limits each component independently.
     *
     * @param min_val Minimum value for each component
     * @param max_val Maximum value for each component
     * @return Clamped vector
     */
    inline Vec3 clamp(float min_val, float max_val) const {
        return Vec3(
            (x < min_val) ? min_val : (x > max_val ? max_val : x),
            (y < min_val) ? min_val : (y > max_val ? max_val : y),
            (z < min_val) ? min_val : (z > max_val ? max_val : z)
        );
    }

    /**
     * String representation for debugging
     *
     * @return Static buffer with "Vec3(x, y, z)" format
     * WARNING: Static buffer, not thread-safe
     */
    inline const char* to_string() const {
        static char buf[64];
        std::snprintf(buf, sizeof(buf), "Vec3(%.2f, %.2f, %.2f)", x, y, z);
        return buf;
    }
};

/* ========== FREE FUNCTIONS ========== */

/**
 * Scalar * Vector (reverse order from Vec3 * scalar)
 * Useful for: 0.5f * velocity
 */
inline Vec3 operator*(float scalar, const Vec3& v) {
    return v * scalar;
}

/**
 * Lerp (linear interpolation) between two vectors
 *
 * Formula: result = a + t * (b - a) = (1-t)*a + t*b
 *
 * When t=0: result = a
 * When t=1: result = b
 * When t=0.5: result = midpoint
 *
 * Use case: Smoothly transition between positions
 *
 * @param a Start vector
 * @param b End vector
 * @param t Parameter [0, 1]
 * @return Interpolated vector
 */
inline Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
    return a + (b - a) * t;
}

#endif /* VEC3_H */
