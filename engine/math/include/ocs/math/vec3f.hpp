#pragma once

#include <cmath>

namespace ocs::math {

struct Vec3f {
    float x{};
    float y{};
    float z{};

    [[nodiscard]] constexpr Vec3f operator+(const Vec3f& rhs) const noexcept {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    [[nodiscard]] constexpr Vec3f operator-(const Vec3f& rhs) const noexcept {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    [[nodiscard]] constexpr Vec3f operator-() const noexcept {
        return {-x, -y, -z};
    }

    [[nodiscard]] constexpr Vec3f operator*(const float scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }

    [[nodiscard]] constexpr float dot(const Vec3f& rhs) const noexcept {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    [[nodiscard]] constexpr Vec3f cross(const Vec3f& rhs) const noexcept {
        return {
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        };
    }

    [[nodiscard]] constexpr float length_squared() const noexcept {
        return dot(*this);
    }

    [[nodiscard]] float length() const noexcept {
        return std::sqrt(length_squared());
    }

    [[nodiscard]] Vec3f normalized() const noexcept {
        const float magnitude = length();
        if (magnitude <= 0.0F) {
            return {};
        }
        return *this * (1.0F / magnitude);
    }
};

} // namespace ocs::math
