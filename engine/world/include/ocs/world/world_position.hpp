#pragma once

#include "ocs/math/vec3f.hpp"

namespace ocs::world {

struct WorldPosition {
    double x{};
    double y{};
    double z{};

    [[nodiscard]] constexpr WorldPosition operator+(const WorldPosition& rhs) const noexcept {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    [[nodiscard]] constexpr WorldPosition operator-(const WorldPosition& rhs) const noexcept {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    constexpr WorldPosition& operator+=(const WorldPosition& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    [[nodiscard]] math::Vec3f relative_to(const WorldPosition& origin) const noexcept {
        return {
            static_cast<float>(x - origin.x),
            static_cast<float>(y - origin.y),
            static_cast<float>(z - origin.z)
        };
    }
};

} // namespace ocs::world
