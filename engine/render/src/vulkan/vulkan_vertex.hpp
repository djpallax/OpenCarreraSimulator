#pragma once

#include <vulkan/vulkan.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "ocs/assets/mesh.hpp"

namespace ocs::render::vulkan {

using Vertex = assets::MeshVertex;

[[nodiscard]] inline VkVertexInputBindingDescription vertex_binding_description() noexcept {
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = static_cast<std::uint32_t>(sizeof(Vertex));
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return binding;
}

[[nodiscard]] inline std::array<VkVertexInputAttributeDescription, 3>
vertex_attribute_descriptions() noexcept {
    return {{
        {
            .location = 0,
            .binding = 0,
            .format = VK_FORMAT_R32G32B32_SFLOAT,
            .offset = static_cast<std::uint32_t>(offsetof(Vertex, position))
        },
        {
            .location = 1,
            .binding = 0,
            .format = VK_FORMAT_R32G32B32_SFLOAT,
            .offset = static_cast<std::uint32_t>(offsetof(Vertex, normal))
        },
        {
            .location = 2,
            .binding = 0,
            .format = VK_FORMAT_R32G32_SFLOAT,
            .offset = static_cast<std::uint32_t>(offsetof(Vertex, texcoord))
        }
    }};
}

static_assert(std::is_standard_layout_v<Vertex>);

} // namespace ocs::render::vulkan
