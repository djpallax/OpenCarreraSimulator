#pragma once

#include <vulkan/vulkan.h>

#include <cstddef>
#include <span>

#include "vulkan_memory.hpp"

namespace ocs::render::vulkan {

class VulkanBuffer {
public:
    VulkanBuffer() = default;
    ~VulkanBuffer();

    VulkanBuffer(const VulkanBuffer&) = delete;
    VulkanBuffer& operator=(const VulkanBuffer&) = delete;

    VulkanBuffer(VulkanBuffer&& other) noexcept;
    VulkanBuffer& operator=(VulkanBuffer&& other) noexcept;

    [[nodiscard]] bool initialize(VulkanMemoryAllocator& allocator,
                                  VkDeviceSize size,
                                  VkBufferUsageFlags usage,
                                  VkMemoryPropertyFlags memory_properties);

    [[nodiscard]] bool initialize_device_local(VulkanMemoryAllocator& allocator,
                                               VkQueue transfer_queue,
                                               std::uint32_t transfer_queue_family,
                                               std::span<const std::byte> data,
                                               VkBufferUsageFlags final_usage);

    [[nodiscard]] bool write(std::span<const std::byte> data,
                             VkDeviceSize offset = 0);

    void shutdown() noexcept;

    [[nodiscard]] VkBuffer handle() const noexcept { return buffer_; }
    [[nodiscard]] VkDeviceSize size() const noexcept { return size_; }
    [[nodiscard]] bool valid() const noexcept { return buffer_ != VK_NULL_HANDLE; }

private:
    VkDevice device_ = VK_NULL_HANDLE;
    VkBuffer buffer_ = VK_NULL_HANDLE;
    VkDeviceMemory memory_ = VK_NULL_HANDLE;
    VkDeviceSize size_ = 0;
    VkMemoryPropertyFlags memory_properties_ = 0;
};

} // namespace ocs::render::vulkan
