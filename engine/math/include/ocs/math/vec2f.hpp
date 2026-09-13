#pragma once

#include <cmath>

namespace ocs::math {

struct Vec2f {
    float x{};
    float y{};

    [[nodiscard]] constexpr Vec2f operator+(const Vec2f& rhs) const noexcept {
        return {x + rhs.x, y + rhs.y};
    }

    [[nodiscard]] constexpr Vec2f operator-(const Vec2f& rhs) const noexcept {
        return {x - rhs.x, y - rhs.y};
    }

    [[nodiscard]] constexpr Vec2f operator*(const float scalar) const noexcept {
        return {x * scalar, y * scalar};
    }

    [[nodiscard]] constexpr float dot(const Vec2f& rhs) const noexcept {
        return x * rhs.x + y * rhs.y;
    }

    [[nodiscard]] constexpr float length_squared() const noexcept {
        return dot(*this);
    }

    [[nodiscard]] float length() const noexcept {
        return std::sqrt(length_squared());
    }
};

} // namespace ocs::math
