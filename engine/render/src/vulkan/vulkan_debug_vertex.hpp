#pragma once

#include <vulkan/vulkan.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "ocs/math/vec3f.hpp"
#include "ocs/math/vec4f.hpp"

namespace ocs::render::vulkan {

struct DebugLineVertex {
    math::Vec3f position{};
    math::Vec4f color{};
};

[[nodiscard]] inline VkVertexInputBindingDescription debug_line_vertex_binding_description() noexcept {
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = static_cast<std::uint32_t>(sizeof(DebugLineVertex));
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return binding;
}

[[nodiscard]] inline std::array<VkVertexInputAttributeDescription, 2>
debug_line_vertex_attribute_descriptions() noexcept {
    return {{
        {
            .location = 0,
            .binding = 0,
            .format = VK_FORMAT_R32G32B32_SFLOAT,
            .offset = static_cast<std::uint32_t>(offsetof(DebugLineVertex, position))
        },
        {
            .location = 1,
            .binding = 0,
            .format = VK_FORMAT_R32G32B32A32_SFLOAT,
            .offset = static_cast<std::uint32_t>(offsetof(DebugLineVertex, color))
        }
    }};
}

static_assert(std::is_standard_layout_v<DebugLineVertex>);

} // namespace ocs::render::vulkan
