#pragma once

#include <vulkan/vulkan.h>

#include <cstdint>

#include "vulkan_memory.hpp"

namespace ocs::render::vulkan {

[[nodiscard]] VkFormat choose_depth_format(VkPhysicalDevice physical_device);

class VulkanDepthTarget {
public:
    VulkanDepthTarget() = default;
    ~VulkanDepthTarget();

    VulkanDepthTarget(const VulkanDepthTarget&) = delete;
    VulkanDepthTarget& operator=(const VulkanDepthTarget&) = delete;

    VulkanDepthTarget(VulkanDepthTarget&& other) noexcept;
    VulkanDepthTarget& operator=(VulkanDepthTarget&& other) noexcept;

    [[nodiscard]] bool initialize(VulkanMemoryAllocator& allocator,
                                  VkQueue graphics_queue,
                                  std::uint32_t graphics_queue_family,
                                  VkExtent2D extent,
                                  VkFormat format);
    void shutdown() noexcept;

    [[nodiscard]] VkImage image() const noexcept { return image_; }
    [[nodiscard]] VkImageView view() const noexcept { return view_; }
    [[nodiscard]] VkFormat format() const noexcept { return format_; }

private:
    VkDevice device_ = VK_NULL_HANDLE;
    VkImage image_ = VK_NULL_HANDLE;
    VkDeviceMemory memory_ = VK_NULL_HANDLE;
    VkImageView view_ = VK_NULL_HANDLE;
    VkFormat format_ = VK_FORMAT_UNDEFINED;
};

} // namespace ocs::render::vulkan
