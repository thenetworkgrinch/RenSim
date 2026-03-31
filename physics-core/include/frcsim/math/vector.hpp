
#pragma once
#include <cmath>
#include <iostream>
#include <cassert>
#include <limits>
#include <iomanip>

namespace frcsim {

struct alignas(16) Vector3 {
    double x, y, z;

    // Constructors
    constexpr Vector3() noexcept : x(0.0), y(0.0), z(0.0) {}
    constexpr Vector3(double x_, double y_, double z_) noexcept : x(x_), y(y_), z(z_) {}
    constexpr Vector3(const Vector3&) noexcept = default;
    constexpr Vector3(Vector3&&) noexcept = default;
    constexpr Vector3& operator=(const Vector3&) noexcept = default;
    constexpr Vector3& operator=(Vector3&&) noexcept = default;

    // Arithmetic operators
    constexpr Vector3 operator+(const Vector3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vector3 operator-(const Vector3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vector3 operator*(double s) const noexcept { return {x * s, y * s, z * s}; }
    constexpr Vector3 operator/(double s) const noexcept {
        return (std::abs(s) > std::numeric_limits<double>::epsilon()) ? Vector3{x / s, y / s, z / s} : Vector3{};
    }
    Vector3& operator+=(const Vector3& o) noexcept { x += o.x; y += o.y; z += o.z; return *this; }
    Vector3& operator-=(const Vector3& o) noexcept { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vector3& operator*=(double s) noexcept { x *= s; y *= s; z *= s; return *this; }
    Vector3& operator/=(double s) noexcept {
        if (std::abs(s) > std::numeric_limits<double>::epsilon()) { x /= s; y /= s; z /= s; }
        else { x = y = z = 0.0; }
        return *this;
    }

    // Comparison operators
    constexpr bool operator==(const Vector3& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
    constexpr bool operator!=(const Vector3& o) const noexcept { return !(*this == o); }

    // Norms
    double norm() const noexcept { return std::sqrt(x * x + y * y + z * z); }
    constexpr double norm2() const noexcept { return x * x + y * y + z * z; }
    Vector3 normalized() const noexcept {
        double n = norm();
        return (n > std::numeric_limits<double>::epsilon()) ? (*this) / n : Vector3{};
    }

    // Dot and cross products
    constexpr double dot(const Vector3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }
    Vector3 cross(const Vector3& o) const noexcept {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }

    // Planar helpers
    double planarSpeed() const noexcept { return std::sqrt(x * x + y * y); }
    Vector3 xy() const noexcept { return {x, y, 0.0}; }
    Vector3 planarDir() const noexcept {
        double mag = planarSpeed();
        return (mag > std::numeric_limits<double>::epsilon()) ? Vector3{x / mag, y / mag, 0.0} : Vector3{};
    }

    // Projection onto another vector
    Vector3 projectOnto(const Vector3& axis) const noexcept {
        double denom = axis.norm2();
        if (denom < std::numeric_limits<double>::epsilon()) return Vector3{};
        return axis * (dot(axis) / denom);
    }

    // Reflection across a plane normal (normalized)
    Vector3 reflect(const Vector3& n) const noexcept {
        return *this - n * (2.0 * dot(n));
    }

    // Torque at a point
    Vector3 torque(const Vector3& r) const noexcept {
        return r.cross(*this);
    }

    // Magnus force (omega x v)
    static Vector3 magnusForce(const Vector3& v, const Vector3& omega, double k = 1e-4) noexcept {
        return omega.cross(v) * k;
    }
    // Instance Magnus force (for legacy API)
    Vector3 magnusForce(double spinRate, double spinCoeff=1e-4) const noexcept {
        return cross({0,0,spinRate}) * spinCoeff;
    }

    // Drag force (-1/2 * rho * Cd * A * |v|^2 * v_hat)
    static Vector3 dragForce(const Vector3& v, double Cd, double A, double rho = 1.225) noexcept {
        double speed = v.norm();
        if (speed < std::numeric_limits<double>::epsilon()) return Vector3{};
        return v.normalized() * (-0.5 * rho * Cd * A * speed * speed);
    }
    // Instance drag force (for legacy API)
    Vector3 dragForce(double airDensity, double dragCoeff, double area) const noexcept {
        double v = norm();
        if(v < std::numeric_limits<double>::epsilon()) return Vector3{};
        double f = 0.5*airDensity*dragCoeff*area*v*v;
        return (*this)*(-f/v);
    }

    // Dynamic gravity (with optional Magnus effect)
    static Vector3 dynamicGravity(double g = 9.81, const Vector3& omega = Vector3{}, double magnusCoeff = 1e-4) noexcept {
        return Vector3(0.0, 0.0, -g) + omega * magnusCoeff;
    }
    // Instance dynamic gravity (for legacy API)
    Vector3 dynamicGravity(double baseG=9.81, const Vector3& spin=Vector3{}, double magnusCoeff=1e-4) const noexcept {
        Vector3 magnus = cross(spin, *this) * magnusCoeff;
        return {0, 0, -baseG} + magnus;
    }

    // Zero vector constant
    static constexpr Vector3 zero() noexcept { return Vector3(0.0, 0.0, 0.0); }

    // Element access
    double& operator[](size_t i) { assert(i < 3); return i == 0 ? x : i == 1 ? y : z; }
    const double& operator[](size_t i) const { assert(i < 3); return i == 0 ? x : i == 1 ? y : z; }

    // Output stream
    friend std::ostream& operator<<(std::ostream& os, const Vector3& v) {
        os << std::fixed << std::setprecision(4) << "[" << v.x << ", " << v.y << ", " << v.z << "]";
        return os;
    }
};

// Scalar multiplication (left)
inline Vector3 operator*(double s, const Vector3& v) noexcept { return v * s; }

} // namespace frcsim