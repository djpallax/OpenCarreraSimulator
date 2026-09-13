#pragma once

#include <vulkan/vulkan.h>

#include <cstdint>
#include <filesystem>
#include <span>

namespace ocs::render::vulkan {

class VulkanGraphicsPipeline {
public:
    VulkanGraphicsPipeline() = default;
    ~VulkanGraphicsPipeline();

    VulkanGraphicsPipeline(const VulkanGraphicsPipeline&) = delete;
    VulkanGraphicsPipeline& operator=(const VulkanGraphicsPipeline&) = delete;

    VulkanGraphicsPipeline(VulkanGraphicsPipeline&& other) noexcept;
    VulkanGraphicsPipeline& operator=(VulkanGraphicsPipeline&& other) noexcept;

    [[nodiscard]] bool initialize(VkDevice device,
                                  VkFormat color_format,
                                  VkFormat depth_format,
                                  const std::filesystem::path& vertex_shader,
                                  const std::filesystem::path& fragment_shader,
                                  std::span<const VkDescriptorSetLayout> descriptor_set_layouts,
                                  std::uint32_t push_constant_size,
                                  bool cull_backfaces);

    [[nodiscard]] bool initialize_debug_lines(
        VkDevice device,
        VkFormat color_format,
        VkFormat depth_format,
        const std::filesystem::path& vertex_shader,
        const std::filesystem::path& fragment_shader,
        std::span<const VkDescriptorSetLayout> descriptor_set_layouts);

    void shutdown() noexcept;

    [[nodiscard]] VkPipeline handle() const noexcept { return pipeline_; }
    [[nodiscard]] VkPipelineLayout layout() const noexcept { return layout_; }
    [[nodiscard]] bool valid() const noexcept { return pipeline_ != VK_NULL_HANDLE; }

private:
    [[nodiscard]] bool initialize_internal(
        VkDevice device,
        VkFormat color_format,
        VkFormat depth_format,
        const std::filesystem::path& vertex_shader,
        const std::filesystem::path& fragment_shader,
        std::span<const VkDescriptorSetLayout> descriptor_set_layouts,
        std::uint32_t push_constant_size,
        const VkVertexInputBindingDescription& binding,
        std::span<const VkVertexInputAttributeDescription> attributes,
        VkPrimitiveTopology topology,
        bool cull_backfaces,
        bool depth_write,
        VkCompareOp depth_compare);

    VkDevice device_ = VK_NULL_HANDLE;
    VkPipelineLayout layout_ = VK_NULL_HANDLE;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
};

} // namespace ocs::render::vulkan
