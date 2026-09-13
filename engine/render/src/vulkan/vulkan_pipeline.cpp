#include "vulkan_pipeline.hpp"

#include <array>
#include <utility>

#include "vulkan_common.hpp"
#include "vulkan_debug_vertex.hpp"
#include "vulkan_shader.hpp"
#include "vulkan_vertex.hpp"

namespace ocs::render::vulkan {

VulkanGraphicsPipeline::~VulkanGraphicsPipeline() {
    shutdown();
}

VulkanGraphicsPipeline::VulkanGraphicsPipeline(VulkanGraphicsPipeline&& other) noexcept
    : device_(std::exchange(other.device_, VK_NULL_HANDLE)),
      layout_(std::exchange(other.layout_, VK_NULL_HANDLE)),
      pipeline_(std::exchange(other.pipeline_, VK_NULL_HANDLE)) {}

VulkanGraphicsPipeline& VulkanGraphicsPipeline::operator=(VulkanGraphicsPipeline&& other) noexcept {
    if (this != &other) {
        shutdown();
        device_ = std::exchange(other.device_, VK_NULL_HANDLE);
        layout_ = std::exchange(other.layout_, VK_NULL_HANDLE);
        pipeline_ = std::exchange(other.pipeline_, VK_NULL_HANDLE);
    }
    return *this;
}

bool VulkanGraphicsPipeline::initialize(const VkDevice device,
                                        const VkFormat color_format,
                                        const VkFormat depth_format,
                                        const std::filesystem::path& vertex_shader,
                                        const std::filesystem::path& fragment_shader,
                                        const std::span<const VkDescriptorSetLayout> descriptor_set_layouts,
                                        const std::uint32_t push_constant_size,
                                        const bool cull_backfaces) {
    const VkVertexInputBindingDescription binding = vertex_binding_description();
    const auto attributes = vertex_attribute_descriptions();
    return initialize_internal(device,
                               color_format,
                               depth_format,
                               vertex_shader,
                               fragment_shader,
                               descriptor_set_layouts,
                               push_constant_size,
                               binding,
                               attributes,
                               VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,
                               cull_backfaces,
                               true,
                               VK_COMPARE_OP_LESS);
}

bool VulkanGraphicsPipeline::initialize_debug_lines(
    const VkDevice device,
    const VkFormat color_format,
    const VkFormat depth_format,
    const std::filesystem::path& vertex_shader,
    const std::filesystem::path& fragment_shader,
    const std::span<const VkDescriptorSetLayout> descriptor_set_layouts) {
    const VkVertexInputBindingDescription binding = debug_line_vertex_binding_description();
    const auto attributes = debug_line_vertex_attribute_descriptions();
    return initialize_internal(device,
                               color_format,
                               depth_format,
                               vertex_shader,
                               fragment_shader,
                               descriptor_set_layouts,
                               0U,
                               binding,
                               attributes,
                               VK_PRIMITIVE_TOPOLOGY_LINE_LIST,
                               false,
                               false,
                               VK_COMPARE_OP_LESS_OR_EQUAL);
}

bool VulkanGraphicsPipeline::initialize_internal(
    const VkDevice device,
    const VkFormat color_format,
    const VkFormat depth_format,
    const std::filesystem::path& vertex_shader,
    const std::filesystem::path& fragment_shader,
    const std::span<const VkDescriptorSetLayout> descriptor_set_layouts,
    const std::uint32_t push_constant_size,
    const VkVertexInputBindingDescription& binding,
    const std::span<const VkVertexInputAttributeDescription> attributes,
    const VkPrimitiveTopology topology,
    const bool cull_backfaces,
    const bool depth_write,
    const VkCompareOp depth_compare) {
    shutdown();

    VulkanShaderModule vertex_module;
    VulkanShaderModule fragment_module;
    if (!vertex_module.load(device, vertex_shader) ||
        !fragment_module.load(device, fragment_shader)) {
        return false;
    }

    std::array<VkPipelineShaderStageCreateInfo, 2> shader_stages{};
    shader_stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    shader_stages[0].module = vertex_module.handle();
    shader_stages[0].pName = "main";

    shader_stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    shader_stages[1].module = fragment_module.handle();
    shader_stages[1].pName = "main";

    VkPipelineVertexInputStateCreateInfo vertex_input{};
    vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertex_input.vertexBindingDescriptionCount = 1;
    vertex_input.pVertexBindingDescriptions = &binding;
    vertex_input.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attributes.size());
    vertex_input.pVertexAttributeDescriptions = attributes.data();

    VkPipelineInputAssemblyStateCreateInfo input_assembly{};
    input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    input_assembly.topology = topology;
    input_assembly.primitiveRestartEnable = VK_FALSE;

    VkPipelineViewportStateCreateInfo viewport_state{};
    viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewport_state.viewportCount = 1;
    viewport_state.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterization{};
    rasterization.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterization.depthClampEnable = VK_FALSE;
    rasterization.rasterizerDiscardEnable = VK_FALSE;
    rasterization.polygonMode = VK_POLYGON_MODE_FILL;
    rasterization.cullMode = cull_backfaces ? VK_CULL_MODE_BACK_BIT : VK_CULL_MODE_NONE;
    // Engine geometry is authored counter-clockwise when viewed from the front.
    // Our positive-height Vulkan viewport preserves that convention.
    rasterization.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterization.depthBiasEnable = VK_FALSE;
    rasterization.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisample{};
    multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisample.sampleShadingEnable = VK_FALSE;

    VkPipelineDepthStencilStateCreateInfo depth_stencil{};
    depth_stencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depth_stencil.depthTestEnable = VK_TRUE;
    depth_stencil.depthWriteEnable = depth_write ? VK_TRUE : VK_FALSE;
    depth_stencil.depthCompareOp = depth_compare;
    depth_stencil.depthBoundsTestEnable = VK_FALSE;
    depth_stencil.stencilTestEnable = VK_FALSE;

    VkPipelineColorBlendAttachmentState blend_attachment{};
    blend_attachment.blendEnable = VK_FALSE;
    blend_attachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT |
        VK_COLOR_COMPONENT_G_BIT |
        VK_COLOR_COMPONENT_B_BIT |
        VK_COLOR_COMPONENT_A_BIT;

    VkPipelineColorBlendStateCreateInfo color_blend{};
    color_blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    color_blend.logicOpEnable = VK_FALSE;
    color_blend.attachmentCount = 1;
    color_blend.pAttachments = &blend_attachment;

    constexpr std::array<VkDynamicState, 2> dynamic_states = {
        VK_DYNAMIC_STATE_VIEWPORT,
        VK_DYNAMIC_STATE_SCISSOR
    };

    VkPipelineDynamicStateCreateInfo dynamic_state{};
    dynamic_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamic_state.dynamicStateCount = static_cast<std::uint32_t>(dynamic_states.size());
    dynamic_state.pDynamicStates = dynamic_states.data();

    VkPushConstantRange push_constant{};
    push_constant.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    push_constant.offset = 0;
    push_constant.size = push_constant_size;

    VkPipelineLayoutCreateInfo layout_info{};
    layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layout_info.setLayoutCount = static_cast<std::uint32_t>(descriptor_set_layouts.size());
    layout_info.pSetLayouts = descriptor_set_layouts.empty() ? nullptr : descriptor_set_layouts.data();
    layout_info.pushConstantRangeCount = push_constant_size > 0U ? 1U : 0U;
    layout_info.pPushConstantRanges = push_constant_size > 0U ? &push_constant : nullptr;

    VkPipelineLayout new_layout = VK_NULL_HANDLE;
    if (!vk_check(vkCreatePipelineLayout(device, &layout_info, nullptr, &new_layout),
                  "vkCreatePipelineLayout")) {
        return false;
    }

    VkPipelineRenderingCreateInfo rendering_info{};
    rendering_info.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
    rendering_info.colorAttachmentCount = 1;
    rendering_info.pColorAttachmentFormats = &color_format;
    rendering_info.depthAttachmentFormat = depth_format;

    VkGraphicsPipelineCreateInfo pipeline_info{};
    pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipeline_info.pNext = &rendering_info;
    pipeline_info.stageCount = static_cast<std::uint32_t>(shader_stages.size());
    pipeline_info.pStages = shader_stages.data();
    pipeline_info.pVertexInputState = &vertex_input;
    pipeline_info.pInputAssemblyState = &input_assembly;
    pipeline_info.pViewportState = &viewport_state;
    pipeline_info.pRasterizationState = &rasterization;
    pipeline_info.pMultisampleState = &multisample;
    pipeline_info.pDepthStencilState = &depth_stencil;
    pipeline_info.pColorBlendState = &color_blend;
    pipeline_info.pDynamicState = &dynamic_state;
    pipeline_info.layout = new_layout;

    VkPipeline new_pipeline = VK_NULL_HANDLE;
    const VkResult result = vkCreateGraphicsPipelines(device,
                                                       VK_NULL_HANDLE,
                                                       1,
                                                       &pipeline_info,
                                                       nullptr,
                                                       &new_pipeline);
    if (!vk_check(result, "vkCreateGraphicsPipelines")) {
        vkDestroyPipelineLayout(device, new_layout, nullptr);
        return false;
    }

    device_ = device;
    layout_ = new_layout;
    pipeline_ = new_pipeline;
    return true;
}

void VulkanGraphicsPipeline::shutdown() noexcept {
    if (device_ != VK_NULL_HANDLE && pipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pipeline_, nullptr);
    }
    if (device_ != VK_NULL_HANDLE && layout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, layout_, nullptr);
    }

    device_ = VK_NULL_HANDLE;
    layout_ = VK_NULL_HANDLE;
    pipeline_ = VK_NULL_HANDLE;
}

} // namespace ocs::render::vulkan
