#pragma once

#include <cmath>

#include "ocs/math/vec3f.hpp"

namespace ocs::math {

struct Quatf {
    float x{};
    float y{};
    float z{};
    float w{1.0F};

    [[nodiscard]] static Quatf identity() noexcept {
        return {};
    }

    [[nodiscard]] static Quatf from_axis_angle(const Vec3f axis, const float radians) noexcept {
        const Vec3f normalized_axis = axis.normalized();
        const float half_angle = radians * 0.5F;
        const float sine = std::sin(half_angle);
        return {
            normalized_axis.x * sine,
            normalized_axis.y * sine,
            normalized_axis.z * sine,
            std::cos(half_angle)
        };
    }

    [[nodiscard]] constexpr Quatf operator*(const Quatf& rhs) const noexcept {
        return {
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w,
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z
        };
    }

    [[nodiscard]] constexpr Quatf conjugate() const noexcept {
        return {-x, -y, -z, w};
    }

    [[nodiscard]] Vec3f rotate(const Vec3f vector) const noexcept {
        const Quatf q = normalized();
        const Quatf pure{vector.x, vector.y, vector.z, 0.0F};
        const Quatf rotated = q * pure * q.conjugate();
        return {rotated.x, rotated.y, rotated.z};
    }


    [[nodiscard]] static Quatf normalized_lerp(const Quatf& a, const Quatf& b, const float alpha) noexcept {
        Quatf target = b;
        const float dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        if (dot < 0.0F) {
            target = {-b.x, -b.y, -b.z, -b.w};
        }

        const float one_minus_alpha = 1.0F - alpha;
        return Quatf{
            a.x * one_minus_alpha + target.x * alpha,
            a.y * one_minus_alpha + target.y * alpha,
            a.z * one_minus_alpha + target.z * alpha,
            a.w * one_minus_alpha + target.w * alpha
        }.normalized();
    }

    [[nodiscard]] float length_squared() const noexcept {
        return x * x + y * y + z * z + w * w;
    }

    [[nodiscard]] Quatf normalized() const noexcept {
        const float magnitude = std::sqrt(length_squared());
        if (magnitude <= 0.0F) {
            return identity();
        }

        const float inv = 1.0F / magnitude;
        return {x * inv, y * inv, z * inv, w * inv};
    }
};

} // namespace ocs::math
