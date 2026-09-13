#pragma once

#include "ocs/math/quatf.hpp"
#include "ocs/math/vec3f.hpp"
#include "ocs/world/world_position.hpp"

namespace ocs::world {

struct Transform {
    WorldPosition position{};
    math::Quatf rotation = math::Quatf::identity();
    math::Vec3f scale{1.0F, 1.0F, 1.0F};
};

} // namespace ocs::world
