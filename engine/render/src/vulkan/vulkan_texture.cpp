#include "vulkan_texture.hpp"

#include <span>
#include <utility>
#include <vector>

#include "ocs/core/log.hpp"
#include "vulkan_buffer.hpp"
#include "vulkan_common.hpp"
#include "vulkan_immediate.hpp"

namespace ocs::render::vulkan {

VulkanTexture::~VulkanTexture() {
    shutdown();
}

VulkanTexture::VulkanTexture(VulkanTexture&& other) noexcept
    : device_(std::exchange(other.device_, VK_NULL_HANDLE)),
      image_(std::exchange(other.image_, VK_NULL_HANDLE)),
      memory_(std::exchange(other.memory_, VK_NULL_HANDLE)),
      view_(std::exchange(other.view_, VK_NULL_HANDLE)),
      format_(std::exchange(other.format_, VK_FORMAT_UNDEFINED)),
      width_(std::exchange(other.width_, 0)),
      height_(std::exchange(other.height_, 0)),
      mip_levels_(std::exchange(other.mip_levels_, 0)),
      allocation_size_(std::exchange(other.allocation_size_, core::ByteSize{})) {}

VulkanTexture& VulkanTexture::operator=(VulkanTexture&& other) noexcept {
    if (this != &other) {
        shutdown();
        device_ = std::exchange(other.device_, VK_NULL_HANDLE);
        image_ = std::exchange(other.image_, VK_NULL_HANDLE);
        memory_ = std::exchange(other.memory_, VK_NULL_HANDLE);
        view_ = std::exchange(other.view_, VK_NULL_HANDLE);
        format_ = std::exchange(other.format_, VK_FORMAT_UNDEFINED);
        width_ = std::exchange(other.width_, 0);
        height_ = std::exchange(other.height_, 0);
        mip_levels_ = std::exchange(other.mip_levels_, 0);
        allocation_size_ = std::exchange(other.allocation_size_, core::ByteSize{});
    }
    return *this;
}

bool VulkanTexture::initialize(VulkanMemoryAllocator& allocator,
                               const VkQueue transfer_queue,
                               const std::uint32_t transfer_queue_family,
                               const assets::TextureData& texture) {
    shutdown();

    if (texture.width == 0U || texture.height == 0U || texture.mip_levels == 0U ||
        texture.pixels.empty() ||
        texture.pixels.size() != assets::texture_expected_byte_size(texture)) {
        OCS_LOG_ERROR("Cannot upload invalid texture data");
        return false;
    }

    device_ = allocator.device();
    width_ = texture.width;
    height_ = texture.height;
    mip_levels_ = texture.mip_levels;
    format_ = texture.color_space == assets::TextureColorSpace::srgb
        ? VK_FORMAT_R8G8B8A8_SRGB
        : VK_FORMAT_R8G8B8A8_UNORM;

    VulkanBuffer staging;
    if (!staging.initialize(allocator,
                            static_cast<VkDeviceSize>(texture.pixels.size()),
                            VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) ||
        !staging.write(std::span<const std::byte>{texture.pixels.data(), texture.pixels.size()})) {
        shutdown();
        return false;
    }

    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.format = format_;
    image_info.extent = {width_, height_, 1U};
    image_info.mipLevels = mip_levels_;
    image_info.arrayLayers = 1U;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    if (!vk_check(vkCreateImage(device_, &image_info, nullptr, &image_),
                  "vkCreateImage(texture)")) {
        shutdown();
        return false;
    }

    VkMemoryRequirements requirements{};
    vkGetImageMemoryRequirements(device_, image_, &requirements);
    allocation_size_ = core::ByteSize::from_bytes(static_cast<std::uint64_t>(requirements.size));
    if (!allocator.allocate(requirements, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, memory_) ||
        !vk_check(vkBindImageMemory(device_, image_, memory_, 0), "vkBindImageMemory(texture)")) {
        shutdown();
        return false;
    }

    std::vector<VkBufferImageCopy> regions;
    regions.reserve(mip_levels_);
    for (std::uint32_t level = 0; level < mip_levels_; ++level) {
        VkBufferImageCopy region{};
        region.bufferOffset = static_cast<VkDeviceSize>(assets::texture_mip_offset(texture, level));
        region.bufferRowLength = 0;
        region.bufferImageHeight = 0;
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.mipLevel = level;
        region.imageSubresource.baseArrayLayer = 0;
        region.imageSubresource.layerCount = 1;
        region.imageOffset = {0, 0, 0};
        region.imageExtent = {
            assets::mip_extent(width_, level),
            assets::mip_extent(height_, level),
            1U
        };
        regions.push_back(region);
    }

    const bool uploaded = immediate_submit(
        device_, transfer_queue, transfer_queue_family,
        [&](const VkCommandBuffer command_buffer) {
            VkImageMemoryBarrier2 to_transfer{};
            to_transfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
            to_transfer.srcStageMask = VK_PIPELINE_STAGE_2_NONE;
            to_transfer.srcAccessMask = VK_ACCESS_2_NONE;
            to_transfer.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
            to_transfer.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
            to_transfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            to_transfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            to_transfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            to_transfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            to_transfer.image = image_;
            to_transfer.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            to_transfer.subresourceRange.baseMipLevel = 0;
            to_transfer.subresourceRange.levelCount = mip_levels_;
            to_transfer.subresourceRange.baseArrayLayer = 0;
            to_transfer.subresourceRange.layerCount = 1;

            VkDependencyInfo transfer_dependency{};
            transfer_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
            transfer_dependency.imageMemoryBarrierCount = 1;
            transfer_dependency.pImageMemoryBarriers = &to_transfer;
            vkCmdPipelineBarrier2(command_buffer, &transfer_dependency);

            vkCmdCopyBufferToImage(command_buffer,
                                   staging.handle(),
                                   image_,
                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                   static_cast<std::uint32_t>(regions.size()),
                                   regions.data());

            VkImageMemoryBarrier2 to_shader{};
            to_shader.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
            to_shader.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
            to_shader.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
            to_shader.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
            to_shader.dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
            to_shader.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            to_shader.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            to_shader.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            to_shader.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            to_shader.image = image_;
            to_shader.subresourceRange = to_transfer.subresourceRange;

            VkDependencyInfo shader_dependency{};
            shader_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
            shader_dependency.imageMemoryBarrierCount = 1;
            shader_dependency.pImageMemoryBarriers = &to_shader;
            vkCmdPipelineBarrier2(command_buffer, &shader_dependency);
        });
    if (!uploaded) {
        shutdown();
        return false;
    }

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = image_;
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = format_;
    view_info.components = {
        VK_COMPONENT_SWIZZLE_IDENTITY,
        VK_COMPONENT_SWIZZLE_IDENTITY,
        VK_COMPONENT_SWIZZLE_IDENTITY,
        VK_COMPONENT_SWIZZLE_IDENTITY
    };
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = mip_levels_;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 1;
    if (!vk_check(vkCreateImageView(device_, &view_info, nullptr, &view_),
                  "vkCreateImageView(texture)")) {
        shutdown();
        return false;
    }
    return true;
}

void VulkanTexture::shutdown() noexcept {
    if (device_ != VK_NULL_HANDLE && view_ != VK_NULL_HANDLE) {
        vkDestroyImageView(device_, view_, nullptr);
    }
    if (device_ != VK_NULL_HANDLE && image_ != VK_NULL_HANDLE) {
        vkDestroyImage(device_, image_, nullptr);
    }
    if (device_ != VK_NULL_HANDLE && memory_ != VK_NULL_HANDLE) {
        vkFreeMemory(device_, memory_, nullptr);
    }

    device_ = VK_NULL_HANDLE;
    image_ = VK_NULL_HANDLE;
    memory_ = VK_NULL_HANDLE;
    view_ = VK_NULL_HANDLE;
    format_ = VK_FORMAT_UNDEFINED;
    width_ = 0;
    height_ = 0;
    mip_levels_ = 0;
    allocation_size_ = {};
}

} // namespace ocs::render::vulkan
