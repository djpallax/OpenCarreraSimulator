#pragma once

#include <vulkan/vulkan.h>

#include <cstdint>
#include <string>

#include "ocs/render/gpu_info.hpp"

namespace ocs::render::vulkan {

struct QueueFamilies {
    std::uint32_t graphics = UINT32_MAX;
    std::uint32_t present = UINT32_MAX;

    [[nodiscard]] bool complete() const noexcept {
        return graphics != UINT32_MAX && present != UINT32_MAX;
    }
};

class VulkanDevice {
public:
    VulkanDevice() = default;
    ~VulkanDevice();

    VulkanDevice(const VulkanDevice&) = delete;
    VulkanDevice& operator=(const VulkanDevice&) = delete;

    bool initialize(VkInstance instance,
                    VkSurfaceKHR surface,
                    const std::string& gpu_selector,
                    bool allow_software_gpu);
    void shutdown() noexcept;

    [[nodiscard]] VkPhysicalDevice physical() const noexcept { return physical_device_; }
    [[nodiscard]] VkDevice handle() const noexcept { return device_; }
    [[nodiscard]] VkQueue graphics_queue() const noexcept { return graphics_queue_; }
    [[nodiscard]] VkQueue present_queue() const noexcept { return present_queue_; }
    [[nodiscard]] const QueueFamilies& queue_families() const noexcept { return queue_families_; }
    [[nodiscard]] const GPUInfo& gpu_info() const noexcept { return gpu_info_; }
    [[nodiscard]] float timestamp_period_ns() const noexcept { return timestamp_period_ns_; }
    [[nodiscard]] std::uint32_t timestamp_valid_bits() const noexcept { return timestamp_valid_bits_; }

private:
    VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue graphics_queue_ = VK_NULL_HANDLE;
    VkQueue present_queue_ = VK_NULL_HANDLE;
    QueueFamilies queue_families_{};
    GPUInfo gpu_info_{};
    float timestamp_period_ns_ = 0.0F;
    std::uint32_t timestamp_valid_bits_ = 0;
};

} // namespace ocs::render::vulkan
