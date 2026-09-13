#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "ocs/math/quatf.hpp"
#include "ocs/math/vec3f.hpp"
#include "ocs/math/vec4f.hpp"

namespace ocs::math {

[[nodiscard]] constexpr float radians(const float degrees) noexcept {
    return degrees * std::numbers::pi_v<float> / 180.0F;
}

struct Mat4f {
    std::array<float, 16> values{};

    [[nodiscard]] static constexpr Mat4f identity() noexcept {
        Mat4f result{};
        result(0, 0) = 1.0F;
        result(1, 1) = 1.0F;
        result(2, 2) = 1.0F;
        result(3, 3) = 1.0F;
        return result;
    }

    [[nodiscard]] static constexpr Mat4f translation(const Vec3f offset) noexcept {
        Mat4f result = identity();
        result(0, 3) = offset.x;
        result(1, 3) = offset.y;
        result(2, 3) = offset.z;
        return result;
    }

    [[nodiscard]] static constexpr Mat4f scale(const Vec3f scale_factor) noexcept {
        Mat4f result = identity();
        result(0, 0) = scale_factor.x;
        result(1, 1) = scale_factor.y;
        result(2, 2) = scale_factor.z;
        return result;
    }

    [[nodiscard]] static Mat4f rotation(const Quatf input) noexcept {
        const Quatf q = input.normalized();

        const float xx = q.x * q.x;
        const float yy = q.y * q.y;
        const float zz = q.z * q.z;
        const float xy = q.x * q.y;
        const float xz = q.x * q.z;
        const float yz = q.y * q.z;
        const float wx = q.w * q.x;
        const float wy = q.w * q.y;
        const float wz = q.w * q.z;

        Mat4f result = identity();
        result(0, 0) = 1.0F - 2.0F * (yy + zz);
        result(0, 1) = 2.0F * (xy - wz);
        result(0, 2) = 2.0F * (xz + wy);

        result(1, 0) = 2.0F * (xy + wz);
        result(1, 1) = 1.0F - 2.0F * (xx + zz);
        result(1, 2) = 2.0F * (yz - wx);

        result(2, 0) = 2.0F * (xz - wy);
        result(2, 1) = 2.0F * (yz + wx);
        result(2, 2) = 1.0F - 2.0F * (xx + yy);
        return result;
    }

    [[nodiscard]] static Mat4f look_at(const Vec3f eye,
                                       const Vec3f target,
                                       const Vec3f up) noexcept {
        const Vec3f forward = (target - eye).normalized();
        const Vec3f right = forward.cross(up).normalized();
        const Vec3f corrected_up = right.cross(forward);

        Mat4f result = identity();
        result(0, 0) = right.x;
        result(0, 1) = right.y;
        result(0, 2) = right.z;
        result(0, 3) = -right.dot(eye);

        result(1, 0) = corrected_up.x;
        result(1, 1) = corrected_up.y;
        result(1, 2) = corrected_up.z;
        result(1, 3) = -corrected_up.dot(eye);

        result(2, 0) = -forward.x;
        result(2, 1) = -forward.y;
        result(2, 2) = -forward.z;
        result(2, 3) = forward.dot(eye);
        return result;
    }

    // Right-handed perspective matrix for Vulkan's [0, 1] depth range.
    // Y is flipped here so a positive-height Vulkan viewport preserves a
    // conventional world-space "up".
    [[nodiscard]] static Mat4f perspective_vulkan(const float vertical_fov_radians,
                                                  const float aspect_ratio,
                                                  const float near_plane,
                                                  const float far_plane) noexcept {
        Mat4f result{};
        const float focal = 1.0F / std::tan(vertical_fov_radians * 0.5F);

        result(0, 0) = focal / aspect_ratio;
        result(1, 1) = -focal;
        result(2, 2) = far_plane / (near_plane - far_plane);
        result(2, 3) = (far_plane * near_plane) / (near_plane - far_plane);
        result(3, 2) = -1.0F;
        return result;
    }

    [[nodiscard]] constexpr float& operator()(const std::size_t row,
                                               const std::size_t column) noexcept {
        return values[column * 4U + row];
    }

    [[nodiscard]] constexpr float operator()(const std::size_t row,
                                              const std::size_t column) const noexcept {
        return values[column * 4U + row];
    }

    [[nodiscard]] constexpr const float* data() const noexcept {
        return values.data();
    }

    [[nodiscard]] constexpr float* data() noexcept {
        return values.data();
    }

    [[nodiscard]] constexpr Mat4f operator*(const Mat4f& rhs) const noexcept {
        Mat4f result{};
        for (std::size_t row = 0; row < 4U; ++row) {
            for (std::size_t column = 0; column < 4U; ++column) {
                float value = 0.0F;
                for (std::size_t inner = 0; inner < 4U; ++inner) {
                    value += (*this)(row, inner) * rhs(inner, column);
                }
                result(row, column) = value;
            }
        }
        return result;
    }

    [[nodiscard]] constexpr Vec4f operator*(const Vec4f vector) const noexcept {
        return {
            (*this)(0, 0) * vector.x + (*this)(0, 1) * vector.y +
                (*this)(0, 2) * vector.z + (*this)(0, 3) * vector.w,
            (*this)(1, 0) * vector.x + (*this)(1, 1) * vector.y +
                (*this)(1, 2) * vector.z + (*this)(1, 3) * vector.w,
            (*this)(2, 0) * vector.x + (*this)(2, 1) * vector.y +
                (*this)(2, 2) * vector.z + (*this)(2, 3) * vector.w,
            (*this)(3, 0) * vector.x + (*this)(3, 1) * vector.y +
                (*this)(3, 2) * vector.z + (*this)(3, 3) * vector.w
        };
    }
};

static_assert(sizeof(Mat4f) == sizeof(float) * 16U);

} // namespace ocs::math
