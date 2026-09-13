#include "vulkan_swapchain.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

#include "ocs/core/log.hpp"
#include "ocs/platform/window.hpp"
#include "vulkan_common.hpp"

namespace ocs::render::vulkan {
namespace {

VkSurfaceFormatKHR choose_surface_format(const std::vector<VkSurfaceFormatKHR>& formats) {
    const auto preferred = std::ranges::find_if(formats, [](const VkSurfaceFormatKHR& format) {
        return format.format == VK_FORMAT_B8G8R8A8_SRGB &&
               format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    });
    return preferred != formats.end() ? *preferred : formats.front();
}

VkPresentModeKHR choose_present_mode(const std::vector<VkPresentModeKHR>& modes, const bool vsync) {
    if (!vsync) {
        if (std::ranges::find(modes, VK_PRESENT_MODE_MAILBOX_KHR) != modes.end()) {
            return VK_PRESENT_MODE_MAILBOX_KHR;
        }
        if (std::ranges::find(modes, VK_PRESENT_MODE_IMMEDIATE_KHR) != modes.end()) {
            return VK_PRESENT_MODE_IMMEDIATE_KHR;
        }
    }
    return VK_PRESENT_MODE_FIFO_KHR;
}


VkCompositeAlphaFlagBitsKHR choose_composite_alpha(const VkSurfaceCapabilitiesKHR& capabilities) {
    constexpr std::array<VkCompositeAlphaFlagBitsKHR, 4> preferred = {
        VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
        VK_COMPOSITE_ALPHA_PRE_MULTIPLIED_BIT_KHR,
        VK_COMPOSITE_ALPHA_POST_MULTIPLIED_BIT_KHR,
        VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR
    };

    for (const VkCompositeAlphaFlagBitsKHR mode : preferred) {
        if ((capabilities.supportedCompositeAlpha & mode) != 0U) {
            return mode;
        }
    }

    return VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
}

VkExtent2D choose_extent(const VkSurfaceCapabilitiesKHR& capabilities,
                         const platform::PixelSize pixel_size) {
    if (capabilities.currentExtent.width != std::numeric_limits<std::uint32_t>::max()) {
        return capabilities.currentExtent;
    }

    const auto width = static_cast<std::uint32_t>(std::max(pixel_size.width, 0));
    const auto height = static_cast<std::uint32_t>(std::max(pixel_size.height, 0));
    return {
        std::clamp(width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width),
        std::clamp(height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height)
    };
}

} // namespace

VulkanSwapchain::~VulkanSwapchain() {
    shutdown();
}

bool VulkanSwapchain::initialize(const VkPhysicalDevice physical_device,
                                 const VkDevice device,
                                 const VkSurfaceKHR surface,
                                 platform::Window& window,
                                 const std::uint32_t graphics_family,
                                 const std::uint32_t present_family,
                                 const bool vsync) {
    shutdown();
    physical_device_ = physical_device;
    device_ = device;
    surface_ = surface;
    window_ = &window;
    graphics_family_ = graphics_family;
    present_family_ = present_family;
    vsync_ = vsync;
    return create(VK_NULL_HANDLE);
}

bool VulkanSwapchain::recreate() {
    if (device_ == VK_NULL_HANDLE || window_ == nullptr) {
        return false;
    }
    return create(swapchain_);
}

bool VulkanSwapchain::create(const VkSwapchainKHR old_swapchain) {
    VkSurfaceCapabilitiesKHR capabilities{};
    if (!vk_check(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical_device_, surface_, &capabilities),
                  "vkGetPhysicalDeviceSurfaceCapabilitiesKHR")) {
        return false;
    }

    std::uint32_t format_count = 0;
    if (!vk_check(vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device_, surface_, &format_count, nullptr),
                  "vkGetPhysicalDeviceSurfaceFormatsKHR(count)")) {
        return false;
    }
    std::vector<VkSurfaceFormatKHR> formats(format_count);
    if (!vk_check(vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device_, surface_, &format_count, formats.data()),
                  "vkGetPhysicalDeviceSurfaceFormatsKHR")) {
        return false;
    }

    std::uint32_t mode_count = 0;
    if (!vk_check(vkGetPhysicalDeviceSurfacePresentModesKHR(physical_device_, surface_, &mode_count, nullptr),
                  "vkGetPhysicalDeviceSurfacePresentModesKHR(count)")) {
        return false;
    }
    std::vector<VkPresentModeKHR> modes(mode_count);
    if (!vk_check(vkGetPhysicalDeviceSurfacePresentModesKHR(physical_device_, surface_, &mode_count, modes.data()),
                  "vkGetPhysicalDeviceSurfacePresentModesKHR")) {
        return false;
    }

    if (formats.empty() || modes.empty()) {
        OCS_LOG_ERROR("Surface has no usable swapchain formats or present modes");
        return false;
    }

    platform::PixelSize pixel_size{};
    if (!window_->pixel_size(pixel_size) || pixel_size.width <= 0 || pixel_size.height <= 0) {
        return false;
    }

    const VkSurfaceFormatKHR surface_format = choose_surface_format(formats);
    const VkPresentModeKHR present_mode = choose_present_mode(modes, vsync_);
    const VkExtent2D extent = choose_extent(capabilities, pixel_size);

    std::uint32_t image_count = capabilities.minImageCount + 1U;
    if (capabilities.maxImageCount > 0 && image_count > capabilities.maxImageCount) {
        image_count = capabilities.maxImageCount;
    }

    std::array<std::uint32_t, 2> queue_indices = {graphics_family_, present_family_};

    VkSwapchainCreateInfoKHR create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    create_info.surface = surface_;
    create_info.minImageCount = image_count;
    create_info.imageFormat = surface_format.format;
    create_info.imageColorSpace = surface_format.colorSpace;
    create_info.imageExtent = extent;
    create_info.imageArrayLayers = 1;
    create_info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    create_info.preTransform = capabilities.currentTransform;
    create_info.compositeAlpha = choose_composite_alpha(capabilities);
    create_info.presentMode = present_mode;
    create_info.clipped = VK_TRUE;
    create_info.oldSwapchain = old_swapchain;

    if (graphics_family_ != present_family_) {
        create_info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        create_info.queueFamilyIndexCount = static_cast<std::uint32_t>(queue_indices.size());
        create_info.pQueueFamilyIndices = queue_indices.data();
    } else {
        create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }

    VkSwapchainKHR new_swapchain = VK_NULL_HANDLE;
    if (!vk_check(vkCreateSwapchainKHR(device_, &create_info, nullptr, &new_swapchain),
                  "vkCreateSwapchainKHR")) {
        return false;
    }

    std::uint32_t actual_image_count = 0;
    if (!vk_check(vkGetSwapchainImagesKHR(device_, new_swapchain, &actual_image_count, nullptr),
                  "vkGetSwapchainImagesKHR(count)")) {
        vkDestroySwapchainKHR(device_, new_swapchain, nullptr);
        return false;
    }

    std::vector<VkImage> new_images(actual_image_count);
    if (!vk_check(vkGetSwapchainImagesKHR(device_, new_swapchain, &actual_image_count, new_images.data()),
                  "vkGetSwapchainImagesKHR")) {
        vkDestroySwapchainKHR(device_, new_swapchain, nullptr);
        return false;
    }

    std::vector<VkImageView> new_views;
    new_views.reserve(new_images.size());
    for (const VkImage image : new_images) {
        VkImageViewCreateInfo view_info{};
        view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        view_info.image = image;
        view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
        view_info.format = surface_format.format;
        view_info.components = {
            VK_COMPONENT_SWIZZLE_IDENTITY,
            VK_COMPONENT_SWIZZLE_IDENTITY,
            VK_COMPONENT_SWIZZLE_IDENTITY,
            VK_COMPONENT_SWIZZLE_IDENTITY
        };
        view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        view_info.subresourceRange.baseMipLevel = 0;
        view_info.subresourceRange.levelCount = 1;
        view_info.subresourceRange.baseArrayLayer = 0;
        view_info.subresourceRange.layerCount = 1;

        VkImageView view = VK_NULL_HANDLE;
        if (!vk_check(vkCreateImageView(device_, &view_info, nullptr, &view), "vkCreateImageView")) {
            for (const VkImageView created : new_views) {
                vkDestroyImageView(device_, created, nullptr);
            }
            vkDestroySwapchainKHR(device_, new_swapchain, nullptr);
            return false;
        }
        new_views.push_back(view);
    }

    destroy_image_views();
    if (swapchain_ != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(device_, swapchain_, nullptr);
    }

    swapchain_ = new_swapchain;
    images_ = std::move(new_images);
    image_views_ = std::move(new_views);
    format_ = surface_format.format;
    extent_ = extent;
    return true;
}

void VulkanSwapchain::destroy_image_views() noexcept {
    if (device_ == VK_NULL_HANDLE) {
        image_views_.clear();
        return;
    }

    for (const VkImageView view : image_views_) {
        vkDestroyImageView(device_, view, nullptr);
    }
    image_views_.clear();
}

void VulkanSwapchain::shutdown() noexcept {
    destroy_image_views();
    images_.clear();

    if (device_ != VK_NULL_HANDLE && swapchain_ != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(device_, swapchain_, nullptr);
    }

    swapchain_ = VK_NULL_HANDLE;
    format_ = VK_FORMAT_UNDEFINED;
    extent_ = {};
    physical_device_ = VK_NULL_HANDLE;
    device_ = VK_NULL_HANDLE;
    surface_ = VK_NULL_HANDLE;
    window_ = nullptr;
    graphics_family_ = UINT32_MAX;
    present_family_ = UINT32_MAX;
}

} // namespace ocs::render::vulkan
