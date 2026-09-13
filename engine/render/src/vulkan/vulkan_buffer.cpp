#include "vulkan_buffer.hpp"

#include <cstring>
#include <utility>

#include "ocs/core/log.hpp"
#include "vulkan_common.hpp"
#include "vulkan_immediate.hpp"

namespace ocs::render::vulkan {

VulkanBuffer::~VulkanBuffer() {
    shutdown();
}

VulkanBuffer::VulkanBuffer(VulkanBuffer&& other) noexcept
    : device_(std::exchange(other.device_, VK_NULL_HANDLE)),
      buffer_(std::exchange(other.buffer_, VK_NULL_HANDLE)),
      memory_(std::exchange(other.memory_, VK_NULL_HANDLE)),
      size_(std::exchange(other.size_, 0)),
      memory_properties_(std::exchange(other.memory_properties_, 0)) {}

VulkanBuffer& VulkanBuffer::operator=(VulkanBuffer&& other) noexcept {
    if (this != &other) {
        shutdown();
        device_ = std::exchange(other.device_, VK_NULL_HANDLE);
        buffer_ = std::exchange(other.buffer_, VK_NULL_HANDLE);
        memory_ = std::exchange(other.memory_, VK_NULL_HANDLE);
        size_ = std::exchange(other.size_, 0);
        memory_properties_ = std::exchange(other.memory_properties_, 0);
    }
    return *this;
}

bool VulkanBuffer::initialize(VulkanMemoryAllocator& allocator,
                              const VkDeviceSize size,
                              const VkBufferUsageFlags usage,
                              const VkMemoryPropertyFlags memory_properties) {
    shutdown();

    if (size == 0) {
        OCS_LOG_ERROR("Cannot create a zero-sized Vulkan buffer");
        return false;
    }

    device_ = allocator.device();
    size_ = size;
    memory_properties_ = memory_properties;

    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = size;
    buffer_info.usage = usage;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (!vk_check(vkCreateBuffer(device_, &buffer_info, nullptr, &buffer_),
                  "vkCreateBuffer")) {
        shutdown();
        return false;
    }

    VkMemoryRequirements requirements{};
    vkGetBufferMemoryRequirements(device_, buffer_, &requirements);

    if (!allocator.allocate(requirements, memory_properties_, memory_)) {
        shutdown();
        return false;
    }

    if (!vk_check(vkBindBufferMemory(device_, buffer_, memory_, 0),
                  "vkBindBufferMemory")) {
        shutdown();
        return false;
    }

    return true;
}

bool VulkanBuffer::write(const std::span<const std::byte> data,
                         const VkDeviceSize offset) {
    if (!valid()) {
        OCS_LOG_ERROR("Attempted to write to an invalid Vulkan buffer");
        return false;
    }

    if ((memory_properties_ & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) == 0U) {
        OCS_LOG_ERROR("Attempted CPU write to non-host-visible Vulkan memory");
        return false;
    }

    const VkDeviceSize byte_count = static_cast<VkDeviceSize>(data.size_bytes());
    if (offset > size_ || byte_count > size_ - offset) {
        OCS_LOG_ERROR("Vulkan buffer write exceeds allocation size");
        return false;
    }

    void* mapped = nullptr;
    if (!vk_check(vkMapMemory(device_, memory_, offset, byte_count, 0, &mapped),
                  "vkMapMemory")) {
        return false;
    }

    std::memcpy(mapped, data.data(), data.size_bytes());

    if ((memory_properties_ & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) == 0U) {
        VkMappedMemoryRange range{};
        range.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
        range.memory = memory_;
        range.offset = offset;
        range.size = byte_count;
        if (!vk_check(vkFlushMappedMemoryRanges(device_, 1, &range),
                      "vkFlushMappedMemoryRanges")) {
            vkUnmapMemory(device_, memory_);
            return false;
        }
    }

    vkUnmapMemory(device_, memory_);
    return true;
}

bool VulkanBuffer::initialize_device_local(VulkanMemoryAllocator& allocator,
                                           const VkQueue transfer_queue,
                                           const std::uint32_t transfer_queue_family,
                                           const std::span<const std::byte> data,
                                           const VkBufferUsageFlags final_usage) {
    if (data.empty()) {
        OCS_LOG_ERROR("Cannot upload an empty Vulkan buffer");
        return false;
    }

    const VkDeviceSize byte_count = static_cast<VkDeviceSize>(data.size_bytes());

    VulkanBuffer staging;
    if (!staging.initialize(allocator,
                            byte_count,
                            VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) {
        return false;
    }

    if (!staging.write(data)) {
        return false;
    }

    if (!initialize(allocator,
                    byte_count,
                    VK_BUFFER_USAGE_TRANSFER_DST_BIT | final_usage,
                    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
        return false;
    }

    const bool copied = immediate_submit(
        device_,
        transfer_queue,
        transfer_queue_family,
        [&](const VkCommandBuffer command_buffer) {
            VkBufferCopy region{};
            region.size = byte_count;
            vkCmdCopyBuffer(command_buffer, staging.handle(), buffer_, 1, &region);
        });

    if (!copied) {
        shutdown();
        return false;
    }

    return true;
}

void VulkanBuffer::shutdown() noexcept {
    if (device_ != VK_NULL_HANDLE && buffer_ != VK_NULL_HANDLE) {
        vkDestroyBuffer(device_, buffer_, nullptr);
    }
    if (device_ != VK_NULL_HANDLE && memory_ != VK_NULL_HANDLE) {
        vkFreeMemory(device_, memory_, nullptr);
    }

    device_ = VK_NULL_HANDLE;
    buffer_ = VK_NULL_HANDLE;
    memory_ = VK_NULL_HANDLE;
    size_ = 0;
    memory_properties_ = 0;
}

} // namespace ocs::render::vulkan
