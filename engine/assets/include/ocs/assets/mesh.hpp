#pragma once

#include <cstdint>
#include <vector>
#include <type_traits>

#include "ocs/math/vec2f.hpp"
#include "ocs/math/vec3f.hpp"

namespace ocs::assets {

struct MeshVertex {
    math::Vec3f position{};
    math::Vec3f normal{0.0F, 0.0F, 1.0F};
    math::Vec2f texcoord{};
};

static_assert(sizeof(MeshVertex) == sizeof(float) * 8U);
static_assert(std::is_trivially_copyable_v<MeshVertex>);

struct Aabb3f {
    math::Vec3f min{};
    math::Vec3f max{};
};

struct MeshData {
    std::vector<MeshVertex> vertices;
    std::vector<std::uint32_t> indices;
    Aabb3f bounds{};
};

} // namespace ocs::assets
