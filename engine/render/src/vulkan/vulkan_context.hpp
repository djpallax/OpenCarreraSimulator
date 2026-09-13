#pragma once

#include <vulkan/vulkan.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ocs/assets/mesh.hpp"
#include "ocs/render/gpu_info.hpp"
#include "ocs/render/handles.hpp"
#include "ocs/render/render_scene.hpp"
#include "ocs/render/renderer.hpp"
#include "vulkan_buffer.hpp"
#include "vulkan_depth.hpp"
#include "vulkan_device.hpp"
#include "vulkan_instance.hpp"
#include "vulkan_memory.hpp"
#include "vulkan_pipeline.hpp"
#include "vulkan_raii.hpp"
#include "vulkan_resource_manager.hpp"
#include "vulkan_swapchain.hpp"

namespace ocs::platform {
class Window;
}

namespace ocs::render::vulkan {

class VulkanContext {
public:
    VulkanContext() = default;
    ~VulkanContext();

    VulkanContext(const VulkanContext&) = delete;
    VulkanContext& operator=(const VulkanContext&) = delete;

    bool initialize(platform::Window& window, const RendererConfig& config);
    void shutdown() noexcept;

    [[nodiscard]] ModelHandle load_model(const std::string& path);
    bool unload_model(ModelHandle handle) noexcept;
    [[nodiscard]] ModelHandle default_model() const noexcept { return default_model_; }
    [[nodiscard]] std::optional<assets::Aabb3f> model_bounds(ModelHandle handle) const noexcept;

    [[nodiscard]] bool draw_frame(const RenderScene& scene, const CameraView& camera);
    void notify_framebuffer_resized() noexcept { framebuffer_resized_ = true; }

    [[nodiscard]] const GPUInfo& gpu_info() const noexcept { return device_.gpu_info(); }
    [[nodiscard]] const RendererStats& stats() const noexcept { return stats_; }
    [[nodiscard]] bool initialized() const noexcept { return initialized_; }

private:
    struct SceneUniforms {
        std::array<float, 16> view_projection{};
        std::array<float, 4> camera_position{};
    };

    static_assert(sizeof(SceneUniforms) == sizeof(float) * 20U);

    struct FrameContext {
        UniqueCommandPool command_pool{};
        VkCommandBuffer command_buffer = VK_NULL_HANDLE;
        UniqueSemaphore image_available{};
        UniqueSemaphore render_finished{};
        UniqueFence in_flight{};
        UniqueQueryPool timestamp_queries{};
        VulkanBuffer scene_uniform{};
        VulkanBuffer debug_line_buffer{};
        VkDescriptorSet scene_set = VK_NULL_HANDLE;
        bool timestamp_pending = false;
    };

    struct PushConstants {
        std::array<float, 16> model{};
        std::array<float, 4> base_color{};
        float metallic = 1.0F;
        float roughness = 1.0F;
        std::uint32_t material_flags = 0;
        float alpha_cutoff = 0.5F;
    };

    static_assert(sizeof(PushConstants) == sizeof(float) * 24U);

    static constexpr std::size_t kFramesInFlight = 2;

    bool create_descriptor_resources();
    void destroy_descriptor_resources() noexcept;
    bool create_frame_resources();
    void destroy_frame_resources() noexcept;

    bool create_default_model_resource();
    void destroy_model_resources() noexcept;
    [[nodiscard]] ModelHandle load_model_from_path(const std::string& path);

    bool create_surface_render_resources();
    void destroy_surface_render_resources() noexcept;

    bool record_render_commands(FrameContext& frame,
                                std::uint32_t image_index,
                                const RenderScene& scene,
                                const CameraView& camera);
    bool recreate_swapchain_if_possible();

    [[nodiscard]] SceneUniforms build_scene_uniforms(const CameraView& camera) const;
    void refresh_resource_stats(const RenderScene* scene = nullptr) noexcept;

    VulkanInstance instance_{};
    VulkanDevice device_{};
    VulkanMemoryAllocator memory_allocator_{};
    VulkanResourceManager resources_{};
    VulkanSwapchain swapchain_{};

    UniqueDescriptorSetLayout scene_set_layout_{};
    UniqueDescriptorSetLayout material_set_layout_{};
    UniqueDescriptorPool descriptor_pool_{};
    std::array<FrameContext, kFramesInFlight> frames_{};
    std::vector<VkFence> images_in_flight_{};

    ModelHandle default_model_{};

    VkFormat depth_format_ = VK_FORMAT_UNDEFINED;
    std::vector<VulkanDepthTarget> depth_targets_{};
    VulkanGraphicsPipeline culled_pipeline_{};
    VulkanGraphicsPipeline double_sided_pipeline_{};
    VulkanGraphicsPipeline debug_line_pipeline_{};

    std::chrono::steady_clock::time_point last_frame_time_{};
    RendererStats stats_{};

    std::size_t current_frame_ = 0;
    platform::Window* window_ = nullptr;
    RendererConfig config_{};
    bool framebuffer_resized_ = false;
    bool initialized_ = false;
};

} // namespace ocs::render::vulkan
