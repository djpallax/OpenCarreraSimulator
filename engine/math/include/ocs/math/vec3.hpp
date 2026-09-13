#pragma once

#include <cmath>

namespace ocs::math {

struct Vec3 {
    double x{};
    double y{};
    double z{};

    [[nodiscard]] constexpr Vec3 operator+(const Vec3& rhs) const noexcept {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    [[nodiscard]] constexpr Vec3 operator-(const Vec3& rhs) const noexcept {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    [[nodiscard]] constexpr Vec3 operator*(const double scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }

    [[nodiscard]] constexpr double dot(const Vec3& rhs) const noexcept {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    [[nodiscard]] constexpr double length_squared() const noexcept {
        return dot(*this);
    }

    [[nodiscard]] double length() const noexcept {
        return std::sqrt(length_squared());
    }
};

} // namespace ocs::math
