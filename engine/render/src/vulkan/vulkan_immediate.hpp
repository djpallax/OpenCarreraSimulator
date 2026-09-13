#pragma once

#include <vulkan/vulkan.h>

#include <cstdint>
#include <functional>

namespace ocs::render::vulkan {

[[nodiscard]] bool immediate_submit(VkDevice device,
                                    VkQueue queue,
                                    std::uint32_t queue_family,
                                    const std::function<void(VkCommandBuffer)>& recorder);

} // namespace ocs::render::vulkan
