#pragma once

#include <vulkan/vulkan.h>

#include <cstdint>
#include <vector>

namespace ocs::platform {
class Window;
}

namespace ocs::render::vulkan {

class VulkanSwapchain {
public:
    VulkanSwapchain() = default;
    ~VulkanSwapchain();

    VulkanSwapchain(const VulkanSwapchain&) = delete;
    VulkanSwapchain& operator=(const VulkanSwapchain&) = delete;

    bool initialize(VkPhysicalDevice physical_device,
                    VkDevice device,
                    VkSurfaceKHR surface,
                    platform::Window& window,
                    std::uint32_t graphics_family,
                    std::uint32_t present_family,
                    bool vsync);

    bool recreate();
    void shutdown() noexcept;

    [[nodiscard]] VkSwapchainKHR handle() const noexcept { return swapchain_; }
    [[nodiscard]] VkFormat format() const noexcept { return format_; }
    [[nodiscard]] VkExtent2D extent() const noexcept { return extent_; }
    [[nodiscard]] const std::vector<VkImage>& images() const noexcept { return images_; }
    [[nodiscard]] const std::vector<VkImageView>& image_views() const noexcept { return image_views_; }

private:
    bool create(VkSwapchainKHR old_swapchain);
    void destroy_image_views() noexcept;

    VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    platform::Window* window_ = nullptr;
    std::uint32_t graphics_family_ = UINT32_MAX;
    std::uint32_t present_family_ = UINT32_MAX;
    bool vsync_ = true;

    VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
    VkFormat format_ = VK_FORMAT_UNDEFINED;
    VkExtent2D extent_{};
    std::vector<VkImage> images_;
    std::vector<VkImageView> image_views_;
};

} // namespace ocs::render::vulkan
