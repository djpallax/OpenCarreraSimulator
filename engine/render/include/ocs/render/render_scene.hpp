#pragma once

#include <vector>

#include "ocs/math/mat4f.hpp"
#include "ocs/render/debug_draw.hpp"
#include "ocs/math/vec3f.hpp"
#include "ocs/render/handles.hpp"

namespace ocs::render {

struct CameraView {
    math::Vec3f forward{1.0F, 0.0F, 0.0F};
    math::Vec3f up{0.0F, 0.0F, 1.0F};
    float vertical_fov_radians = 1.01229097F; // 58 degrees
    float near_plane = 0.05F;
    float far_plane = 5000.0F;
};

struct RenderInstance {
    ModelHandle model{};
    math::Mat4f model_matrix = math::Mat4f::identity();
};

struct RenderScene {
    std::vector<RenderInstance> instances{};
    DebugDrawList debug_draw{};
};

} // namespace ocs::render
