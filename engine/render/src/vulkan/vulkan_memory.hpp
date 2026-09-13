#pragma once

#include <vulkan/vulkan.h>

#include <cstdint>

namespace ocs::render::vulkan {

class VulkanMemoryAllocator {
public:
    VulkanMemoryAllocator() = default;

    void initialize(VkPhysicalDevice physical_device, VkDevice device) noexcept;
    void shutdown() noexcept;

    [[nodiscard]] bool allocate(const VkMemoryRequirements& requirements,
                                VkMemoryPropertyFlags properties,
                                VkDeviceMemory& out_memory) const;

    [[nodiscard]] VkDevice device() const noexcept { return device_; }
    [[nodiscard]] bool initialized() const noexcept {
        return physical_device_ != VK_NULL_HANDLE && device_ != VK_NULL_HANDLE;
    }

private:
    [[nodiscard]] bool find_memory_type(std::uint32_t type_filter,
                                        VkMemoryPropertyFlags properties,
                                        std::uint32_t& out_index) const noexcept;

    VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties memory_properties_{};
};

} // namespace ocs::render::vulkan
