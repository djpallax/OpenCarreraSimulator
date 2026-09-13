#pragma once

#include <vulkan/vulkan.h>

#include <cstdint>

#include "ocs/assets/texture.hpp"
#include "ocs/core/bytes.hpp"
#include "vulkan_memory.hpp"

namespace ocs::render::vulkan {

class VulkanTexture {
public:
    VulkanTexture() = default;
    ~VulkanTexture();

    VulkanTexture(const VulkanTexture&) = delete;
    VulkanTexture& operator=(const VulkanTexture&) = delete;

    VulkanTexture(VulkanTexture&& other) noexcept;
    VulkanTexture& operator=(VulkanTexture&& other) noexcept;

    [[nodiscard]] bool initialize(VulkanMemoryAllocator& allocator,
                                  VkQueue transfer_queue,
                                  std::uint32_t transfer_queue_family,
                                  const assets::TextureData& texture);
    void shutdown() noexcept;

    [[nodiscard]] VkImage image() const noexcept { return image_; }
    [[nodiscard]] VkImageView view() const noexcept { return view_; }
    [[nodiscard]] VkFormat format() const noexcept { return format_; }
    [[nodiscard]] std::uint32_t width() const noexcept { return width_; }
    [[nodiscard]] std::uint32_t height() const noexcept { return height_; }
    [[nodiscard]] std::uint32_t mip_levels() const noexcept { return mip_levels_; }
    [[nodiscard]] core::ByteSize allocation_size() const noexcept { return allocation_size_; }
    [[nodiscard]] bool valid() const noexcept { return image_ != VK_NULL_HANDLE && view_ != VK_NULL_HANDLE; }

private:
    VkDevice device_ = VK_NULL_HANDLE;
    VkImage image_ = VK_NULL_HANDLE;
    VkDeviceMemory memory_ = VK_NULL_HANDLE;
    VkImageView view_ = VK_NULL_HANDLE;
    VkFormat format_ = VK_FORMAT_UNDEFINED;
    std::uint32_t width_ = 0;
    std::uint32_t height_ = 0;
    std::uint32_t mip_levels_ = 0;
    core::ByteSize allocation_size_{};
};

} // namespace ocs::render::vulkan
