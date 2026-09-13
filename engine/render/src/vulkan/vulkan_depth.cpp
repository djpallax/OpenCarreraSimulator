#include "vulkan_depth.hpp"

#include <array>
#include <utility>

#include "ocs/core/log.hpp"
#include "vulkan_common.hpp"
#include "vulkan_immediate.hpp"

namespace ocs::render::vulkan {

VkFormat choose_depth_format(const VkPhysicalDevice physical_device) {
    constexpr std::array<VkFormat, 3> candidates = {
        VK_FORMAT_D32_SFLOAT,
        VK_FORMAT_D32_SFLOAT_S8_UINT,
        VK_FORMAT_D24_UNORM_S8_UINT
    };

    for (const VkFormat format : candidates) {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(physical_device, format, &properties);
        if ((properties.optimalTilingFeatures &
             VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) != 0U) {
            return format;
        }
    }

    return VK_FORMAT_UNDEFINED;
}

VulkanDepthTarget::~VulkanDepthTarget() {
    shutdown();
}

VulkanDepthTarget::VulkanDepthTarget(VulkanDepthTarget&& other) noexcept
    : device_(std::exchange(other.device_, VK_NULL_HANDLE)),
      image_(std::exchange(other.image_, VK_NULL_HANDLE)),
      memory_(std::exchange(other.memory_, VK_NULL_HANDLE)),
      view_(std::exchange(other.view_, VK_NULL_HANDLE)),
      format_(std::exchange(other.format_, VK_FORMAT_UNDEFINED)) {}

VulkanDepthTarget& VulkanDepthTarget::operator=(VulkanDepthTarget&& other) noexcept {
    if (this != &other) {
        shutdown();
        device_ = std::exchange(other.device_, VK_NULL_HANDLE);
        image_ = std::exchange(other.image_, VK_NULL_HANDLE);
        memory_ = std::exchange(other.memory_, VK_NULL_HANDLE);
        view_ = std::exchange(other.view_, VK_NULL_HANDLE);
        format_ = std::exchange(other.format_, VK_FORMAT_UNDEFINED);
    }
    return *this;
}

bool VulkanDepthTarget::initialize(VulkanMemoryAllocator& allocator,
                                   const VkQueue graphics_queue,
                                   const std::uint32_t graphics_queue_family,
                                   const VkExtent2D extent,
                                   const VkFormat format) {
    shutdown();

    if (format == VK_FORMAT_UNDEFINED || extent.width == 0U || extent.height == 0U) {
        OCS_LOG_ERROR("Invalid depth target configuration");
        return false;
    }

    device_ = allocator.device();
    format_ = format;

    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.format = format_;
    image_info.extent = {extent.width, extent.height, 1U};
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    if (!vk_check(vkCreateImage(device_, &image_info, nullptr, &image_),
                  "vkCreateImage(depth)")) {
        shutdown();
        return false;
    }

    VkMemoryRequirements requirements{};
    vkGetImageMemoryRequirements(device_, image_, &requirements);
    if (!allocator.allocate(requirements,
                            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                            memory_)) {
        shutdown();
        return false;
    }

    if (!vk_check(vkBindImageMemory(device_, image_, memory_, 0),
                  "vkBindImageMemory(depth)")) {
        shutdown();
        return false;
    }

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = image_;
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = format_;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 1;

    if (!vk_check(vkCreateImageView(device_, &view_info, nullptr, &view_),
                  "vkCreateImageView(depth)")) {
        shutdown();
        return false;
    }

    const bool transitioned = immediate_submit(
        device_,
        graphics_queue,
        graphics_queue_family,
        [&](const VkCommandBuffer command_buffer) {
            VkImageMemoryBarrier2 barrier{};
            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
            barrier.srcStageMask = VK_PIPELINE_STAGE_2_NONE;
            barrier.srcAccessMask = VK_ACCESS_2_NONE;
            barrier.dstStageMask =
                VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
            barrier.dstAccessMask =
                VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
            barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            barrier.newLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.image = image_;
            barrier.subresourceRange = view_info.subresourceRange;

            VkDependencyInfo dependency{};
            dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
            dependency.imageMemoryBarrierCount = 1;
            dependency.pImageMemoryBarriers = &barrier;
            vkCmdPipelineBarrier2(command_buffer, &dependency);
        });

    if (!transitioned) {
        shutdown();
        return false;
    }

    return true;
}

void VulkanDepthTarget::shutdown() noexcept {
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
}

} // namespace ocs::render::vulkan
