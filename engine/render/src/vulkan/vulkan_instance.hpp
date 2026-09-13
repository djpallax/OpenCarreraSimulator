#pragma once

#include <vulkan/vulkan.h>

#include <cstdint>

struct SDL_Window;

namespace ocs::render::vulkan {

class VulkanInstance {
public:
    VulkanInstance() = default;
    ~VulkanInstance();

    VulkanInstance(const VulkanInstance&) = delete;
    VulkanInstance& operator=(const VulkanInstance&) = delete;

    bool initialize(SDL_Window* window, bool request_validation);
    void shutdown() noexcept;

    [[nodiscard]] VkInstance handle() const noexcept { return instance_; }
    [[nodiscard]] VkSurfaceKHR surface() const noexcept { return surface_; }
    [[nodiscard]] bool validation_enabled() const noexcept { return validation_enabled_; }
    [[nodiscard]] std::uint32_t loader_version() const noexcept { return loader_version_; }
    [[nodiscard]] std::uint32_t instance_api_version() const noexcept { return instance_api_version_; }

private:
    bool create_debug_messenger();

    VkInstance instance_ = VK_NULL_HANDLE;
    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debug_messenger_ = VK_NULL_HANDLE;
    std::uint32_t loader_version_ = VK_API_VERSION_1_0;
    std::uint32_t instance_api_version_ = VK_API_VERSION_1_0;
    bool validation_enabled_ = false;
    bool debug_utils_enabled_ = false;
};

} // namespace ocs::render::vulkan
