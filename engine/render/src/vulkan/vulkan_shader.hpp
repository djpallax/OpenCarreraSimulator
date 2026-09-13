#pragma once

#include <vulkan/vulkan.h>

#include <filesystem>

namespace ocs::render::vulkan {

class VulkanShaderModule {
public:
    VulkanShaderModule() = default;
    ~VulkanShaderModule();

    VulkanShaderModule(const VulkanShaderModule&) = delete;
    VulkanShaderModule& operator=(const VulkanShaderModule&) = delete;

    VulkanShaderModule(VulkanShaderModule&& other) noexcept;
    VulkanShaderModule& operator=(VulkanShaderModule&& other) noexcept;

    [[nodiscard]] bool load(VkDevice device, const std::filesystem::path& path);
    void shutdown() noexcept;

    [[nodiscard]] VkShaderModule handle() const noexcept { return module_; }

private:
    VkDevice device_ = VK_NULL_HANDLE;
    VkShaderModule module_ = VK_NULL_HANDLE;
};

} // namespace ocs::render::vulkan
