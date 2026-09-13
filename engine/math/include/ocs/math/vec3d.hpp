#pragma once

#include <cmath>

#include "ocs/math/vec3f.hpp"

namespace ocs::math {

struct Vec3d {
    double x{};
    double y{};
    double z{};

    [[nodiscard]] constexpr Vec3d operator+(const Vec3d& rhs) const noexcept {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    [[nodiscard]] constexpr Vec3d operator-(const Vec3d& rhs) const noexcept {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    [[nodiscard]] constexpr Vec3d operator-() const noexcept {
        return {-x, -y, -z};
    }

    [[nodiscard]] constexpr Vec3d operator*(const double scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }

    [[nodiscard]] constexpr Vec3d operator/(const double scalar) const noexcept {
        return {x / scalar, y / scalar, z / scalar};
    }

    constexpr Vec3d& operator+=(const Vec3d& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    constexpr Vec3d& operator-=(const Vec3d& rhs) noexcept {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    constexpr Vec3d& operator*=(const double scalar) noexcept {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    [[nodiscard]] constexpr double dot(const Vec3d& rhs) const noexcept {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    [[nodiscard]] constexpr Vec3d cross(const Vec3d& rhs) const noexcept {
        return {
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        };
    }

    [[nodiscard]] constexpr double length_squared() const noexcept {
        return dot(*this);
    }

    [[nodiscard]] double length() const noexcept {
        return std::sqrt(length_squared());
    }

    [[nodiscard]] Vec3d normalized() const noexcept {
        const double magnitude = length();
        if (magnitude <= 0.0) {
            return {};
        }
        return *this / magnitude;
    }

    [[nodiscard]] constexpr Vec3f to_vec3f() const noexcept {
        return {
            static_cast<float>(x),
            static_cast<float>(y),
            static_cast<float>(z)
        };
    }

    [[nodiscard]] static constexpr Vec3d from_vec3f(const Vec3f value) noexcept {
        return {
            static_cast<double>(value.x),
            static_cast<double>(value.y),
            static_cast<double>(value.z)
        };
    }
};

} // namespace ocs::math
