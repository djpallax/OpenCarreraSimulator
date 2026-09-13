#include "vulkan_context.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <span>
#include <string>
#include <utility>

#include "ocs/assets/mesh_io.hpp"
#include "ocs/assets/model_io.hpp"
#include "ocs/core/log.hpp"
#include "ocs/math/mat4f.hpp"
#include "ocs/math/quatf.hpp"
#include "ocs/math/vec3f.hpp"
#include "ocs/platform/window.hpp"
#include "vulkan_common.hpp"
#include "vulkan_debug_vertex.hpp"

#ifndef OCS_SHADER_BINARY_DIR
#error "OCS_SHADER_BINARY_DIR must be provided by CMake"
#endif

namespace ocs::render::vulkan {
namespace {

constexpr std::uint32_t kMaterialUnlit = 1U << 0U;
constexpr std::uint32_t kMaterialAlphaMask = 1U << 1U;
constexpr std::uint32_t kMaterialDoubleSided = 1U << 2U;

assets::ModelData make_fallback_model() {
    assets::MeshData mesh{};
    mesh.vertices = {
        {{-1.0F, -1.0F, -1.0F}, {-0.577F, -0.577F, -0.577F}, {}},
        {{ 1.0F, -1.0F, -1.0F}, { 0.577F, -0.577F, -0.577F}, {}},
        {{ 1.0F,  1.0F, -1.0F}, { 0.577F,  0.577F, -0.577F}, {}},
        {{-1.0F,  1.0F, -1.0F}, {-0.577F,  0.577F, -0.577F}, {}},
        {{-1.0F, -1.0F,  1.0F}, {-0.577F, -0.577F,  0.577F}, {}},
        {{ 1.0F, -1.0F,  1.0F}, { 0.577F, -0.577F,  0.577F}, {}},
        {{ 1.0F,  1.0F,  1.0F}, { 0.577F,  0.577F,  0.577F}, {}},
        {{-1.0F,  1.0F,  1.0F}, {-0.577F,  0.577F,  0.577F}, {}}
    };
    mesh.indices = {
        0, 2, 1, 0, 3, 2,
        4, 5, 6, 4, 6, 7,
        1, 2, 6, 1, 6, 5,
        0, 7, 3, 0, 4, 7,
        2, 3, 7, 2, 7, 6,
        0, 1, 5, 0, 5, 4
    };
    mesh.bounds.min = {-1.0F, -1.0F, -1.0F};
    mesh.bounds.max = {1.0F, 1.0F, 1.0F};
    auto model = assets::model_from_mesh(std::move(mesh));
    model.materials[0].base_color_factor = {0.35F, 0.58F, 0.86F, 1.0F};
    model.materials[0].metallic_factor = 0.05F;
    model.materials[0].roughness_factor = 0.55F;
    return model;
}

std::filesystem::path shader_path(const char* relative_path) {
    return std::filesystem::path{OCS_SHADER_BINARY_DIR} / relative_path;
}

} // namespace

VulkanContext::~VulkanContext() {
    shutdown();
}

bool VulkanContext::initialize(platform::Window& window, const RendererConfig& config) {
    shutdown();
    window_ = &window;
    config_ = config;

    if (!instance_.initialize(window.native_handle(), config.enable_validation)) {
        shutdown();
        return false;
    }

    if (!device_.initialize(instance_.handle(),
                            instance_.surface(),
                            config.gpu_selector,
                            config.allow_software_gpu)) {
        shutdown();
        return false;
    }

    memory_allocator_.initialize(device_.physical(), device_.handle());

    if (!swapchain_.initialize(device_.physical(),
                               device_.handle(),
                               instance_.surface(),
                               window,
                               device_.queue_families().graphics,
                               device_.queue_families().present,
                               config.vsync)) {
        shutdown();
        return false;
    }

    if (!create_descriptor_resources() ||
        !resources_.initialize(memory_allocator_,
                               device_.graphics_queue(),
                               device_.queue_families().graphics,
                               device_.handle(),
                               material_set_layout_.get()) ||
        !create_frame_resources() ||
        !create_default_model_resource() ||
        !create_surface_render_resources()) {
        shutdown();
        return false;
    }

    last_frame_time_ = std::chrono::steady_clock::now();
    initialized_ = true;
    OCS_LOG_INFO("Vulkan Step 7 world/scene renderer initialized");
    return true;
}

bool VulkanContext::create_descriptor_resources() {
    VkDescriptorSetLayoutBinding scene_binding{};
    scene_binding.binding = 0;
    scene_binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    scene_binding.descriptorCount = 1;
    scene_binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo scene_layout_info{};
    scene_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    scene_layout_info.bindingCount = 1;
    scene_layout_info.pBindings = &scene_binding;
    if (!vk_check(vkCreateDescriptorSetLayout(device_.handle(),
                                               &scene_layout_info,
                                               nullptr,
                                               scene_set_layout_.put(device_.handle())),
                  "vkCreateDescriptorSetLayout(scene)")) {
        return false;
    }

    VkDescriptorSetLayoutBinding material_binding{};
    material_binding.binding = 0;
    material_binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    material_binding.descriptorCount = 1;
    material_binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo material_layout_info{};
    material_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    material_layout_info.bindingCount = 1;
    material_layout_info.pBindings = &material_binding;
    if (!vk_check(vkCreateDescriptorSetLayout(device_.handle(),
                                               &material_layout_info,
                                               nullptr,
                                               material_set_layout_.put(device_.handle())),
                  "vkCreateDescriptorSetLayout(material)")) {
        return false;
    }

    VkDescriptorPoolSize pool_size{};
    pool_size.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    pool_size.descriptorCount = static_cast<std::uint32_t>(kFramesInFlight);

    VkDescriptorPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.maxSets = static_cast<std::uint32_t>(kFramesInFlight);
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = &pool_size;
    return vk_check(vkCreateDescriptorPool(device_.handle(),
                                            &pool_info,
                                            nullptr,
                                            descriptor_pool_.put(device_.handle())),
                    "vkCreateDescriptorPool(scene)");
}

void VulkanContext::destroy_descriptor_resources() noexcept {
    descriptor_pool_.reset();
    material_set_layout_.reset();
    scene_set_layout_.reset();
}

bool VulkanContext::create_frame_resources() {
    for (FrameContext& frame : frames_) {
        VkCommandPoolCreateInfo pool_info{};
        pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        pool_info.queueFamilyIndex = device_.queue_families().graphics;
        if (!vk_check(vkCreateCommandPool(device_.handle(),
                                           &pool_info,
                                           nullptr,
                                           frame.command_pool.put(device_.handle())),
                      "vkCreateCommandPool")) {
            return false;
        }

        VkCommandBufferAllocateInfo command_info{};
        command_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        command_info.commandPool = frame.command_pool.get();
        command_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        command_info.commandBufferCount = 1;
        if (!vk_check(vkAllocateCommandBuffers(device_.handle(),
                                                &command_info,
                                                &frame.command_buffer),
                      "vkAllocateCommandBuffers")) {
            return false;
        }

        VkSemaphoreCreateInfo semaphore_info{};
        semaphore_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
        if (!vk_check(vkCreateSemaphore(device_.handle(),
                                         &semaphore_info,
                                         nullptr,
                                         frame.image_available.put(device_.handle())),
                      "vkCreateSemaphore(image available)")) {
            return false;
        }
        if (!vk_check(vkCreateSemaphore(device_.handle(),
                                         &semaphore_info,
                                         nullptr,
                                         frame.render_finished.put(device_.handle())),
                      "vkCreateSemaphore(render finished)")) {
            return false;
        }

        VkFenceCreateInfo fence_info{};
        fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fence_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        if (!vk_check(vkCreateFence(device_.handle(),
                                     &fence_info,
                                     nullptr,
                                     frame.in_flight.put(device_.handle())),
                      "vkCreateFence")) {
            return false;
        }

        if (device_.timestamp_valid_bits() > 0U) {
            VkQueryPoolCreateInfo query_info{};
            query_info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
            query_info.queryType = VK_QUERY_TYPE_TIMESTAMP;
            query_info.queryCount = 2;
            if (!vk_check(vkCreateQueryPool(device_.handle(),
                                             &query_info,
                                             nullptr,
                                             frame.timestamp_queries.put(device_.handle())),
                          "vkCreateQueryPool(timestamp)")) {
                return false;
            }
        }

        if (!frame.scene_uniform.initialize(
                memory_allocator_,
                sizeof(SceneUniforms),
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) {
            return false;
        }

        const VkDescriptorSetLayout layout = scene_set_layout_.get();
        VkDescriptorSetAllocateInfo allocate_info{};
        allocate_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocate_info.descriptorPool = descriptor_pool_.get();
        allocate_info.descriptorSetCount = 1;
        allocate_info.pSetLayouts = &layout;
        if (!vk_check(vkAllocateDescriptorSets(device_.handle(),
                                                &allocate_info,
                                                &frame.scene_set),
                      "vkAllocateDescriptorSets(scene)")) {
            return false;
        }

        VkDescriptorBufferInfo buffer_info{};
        buffer_info.buffer = frame.scene_uniform.handle();
        buffer_info.offset = 0;
        buffer_info.range = sizeof(SceneUniforms);

        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = frame.scene_set;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        write.pBufferInfo = &buffer_info;
        vkUpdateDescriptorSets(device_.handle(), 1, &write, 0, nullptr);
    }
    return true;
}

ModelHandle VulkanContext::load_model_from_path(const std::string& path_string) {
    if (path_string.empty()) {
        return {};
    }

    assets::ModelData model{};
    const std::filesystem::path path = path_string;
    if (path.extension() == ".ocsmodel") {
        auto loaded = assets::load_model(path);
        if (!loaded) {
            OCS_LOG_ERROR("Failed to load runtime model '" + path.string() + "': " +
                          loaded.error().message);
            return {};
        }
        model = std::move(*loaded);
        OCS_LOG_INFO("Loaded runtime model: " + path.string());
    } else if (path.extension() == ".ocsmesh") {
        auto loaded = assets::load_mesh(path);
        if (!loaded) {
            OCS_LOG_ERROR("Failed to load legacy runtime mesh '" + path.string() + "': " +
                          loaded.error().message);
            return {};
        }
        model = assets::model_from_mesh(std::move(*loaded));
        OCS_LOG_WARN("Loaded legacy .ocsmesh through the compatibility path");
    } else {
        OCS_LOG_ERROR("Runtime only accepts .ocsmodel/.ocsmesh. Compile source assets first.");
        return {};
    }

    for (const assets::MaterialData& material : model.materials) {
        if (material.alpha_mode == assets::MaterialAlphaMode::blend) {
            OCS_LOG_WARN("Alpha-blend materials are stored but currently rendered through the opaque path");
            break;
        }
    }

    return resources_.upload_model(model);
}

bool VulkanContext::create_default_model_resource() {
    if (!config_.asset_path.empty()) {
        default_model_ = load_model_from_path(config_.asset_path);
    } else {
        default_model_ = resources_.upload_model(make_fallback_model());
        OCS_LOG_WARN("No runtime asset supplied; using built-in fallback model");
    }

    if (!default_model_) {
        return false;
    }
    refresh_resource_stats();
    return true;
}

ModelHandle VulkanContext::load_model(const std::string& path) {
    if (!initialized_) {
        return {};
    }
    const ModelHandle handle = load_model_from_path(path);
    refresh_resource_stats();
    return handle;
}

bool VulkanContext::unload_model(const ModelHandle handle) noexcept {
    if (!handle || handle == default_model_ || device_.handle() == VK_NULL_HANDLE) {
        return false;
    }
    if (vkDeviceWaitIdle(device_.handle()) != VK_SUCCESS) {
        return false;
    }
    const bool removed = resources_.destroy_model(handle);
    refresh_resource_stats();
    return removed;
}

std::optional<assets::Aabb3f> VulkanContext::model_bounds(const ModelHandle handle) const noexcept {
    const GpuModelResource* model = resources_.model(handle);
    if (model == nullptr) {
        return std::nullopt;
    }
    return model->bounds;
}


void VulkanContext::destroy_model_resources() noexcept {
    // All model resources are owned by VulkanResourceManager. Clearing it during
    // shutdown releases default and dynamically loaded world assets together.
    default_model_ = {};
    refresh_resource_stats();
}

bool VulkanContext::create_surface_render_resources() {
    depth_format_ = choose_depth_format(device_.physical());
    if (depth_format_ == VK_FORMAT_UNDEFINED) {
        OCS_LOG_ERROR("No supported depth attachment format found");
        return false;
    }

    depth_targets_.clear();
    depth_targets_.resize(swapchain_.images().size());
    for (VulkanDepthTarget& depth_target : depth_targets_) {
        if (!depth_target.initialize(memory_allocator_,
                                     device_.graphics_queue(),
                                     device_.queue_families().graphics,
                                     swapchain_.extent(),
                                     depth_format_)) {
            destroy_surface_render_resources();
            return false;
        }
    }

    const auto vertex_shader = shader_path("material/simple.vert.spv");
    const auto fragment_shader = shader_path("material/simple.frag.spv");
    const std::array<VkDescriptorSetLayout, 2> descriptor_layouts = {
        scene_set_layout_.get(), material_set_layout_.get()
    };
    const std::array<VkDescriptorSetLayout, 1> debug_descriptor_layouts = {
        scene_set_layout_.get()
    };
    if (!culled_pipeline_.initialize(device_.handle(),
                                     swapchain_.format(),
                                     depth_format_,
                                     vertex_shader,
                                     fragment_shader,
                                     descriptor_layouts,
                                     static_cast<std::uint32_t>(sizeof(PushConstants)),
                                     true) ||
        !double_sided_pipeline_.initialize(device_.handle(),
                                           swapchain_.format(),
                                           depth_format_,
                                           vertex_shader,
                                           fragment_shader,
                                           descriptor_layouts,
                                           static_cast<std::uint32_t>(sizeof(PushConstants)),
                                           false) ||
        !debug_line_pipeline_.initialize_debug_lines(
            device_.handle(),
            swapchain_.format(),
            depth_format_,
            shader_path("debug/physics_lines.vert.spv"),
            shader_path("debug/physics_lines.frag.spv"),
            debug_descriptor_layouts)) {
        destroy_surface_render_resources();
        return false;
    }

    images_in_flight_.assign(swapchain_.images().size(), VK_NULL_HANDLE);
    return true;
}

void VulkanContext::destroy_surface_render_resources() noexcept {
    debug_line_pipeline_.shutdown();
    double_sided_pipeline_.shutdown();
    culled_pipeline_.shutdown();
    depth_targets_.clear();
    depth_format_ = VK_FORMAT_UNDEFINED;
    images_in_flight_.clear();
}



VulkanContext::SceneUniforms VulkanContext::build_scene_uniforms(const CameraView& camera) const {
    using namespace math;

    Vec3f forward = camera.forward.normalized();
    if (forward.length_squared() <= 0.0F) {
        forward = {1.0F, 0.0F, 0.0F};
    }
    Vec3f up = camera.up.normalized();
    if (up.length_squared() <= 0.0F || std::abs(forward.dot(up)) > 0.999F) {
        up = {0.0F, 0.0F, 1.0F};
    }

    // RenderScene transforms are camera-relative, therefore the camera lives at
    // the float origin and only orientation/projection remain in the view matrix.
    const Vec3f eye{0.0F, 0.0F, 0.0F};
    const Mat4f view = Mat4f::look_at(eye, forward, up);

    const VkExtent2D extent = swapchain_.extent();
    const float aspect = static_cast<float>(extent.width) / static_cast<float>(extent.height);
    const float near_plane = std::max(0.001F, camera.near_plane);
    const float far_plane = std::max(near_plane + 1.0F, camera.far_plane);
    const Mat4f projection = Mat4f::perspective_vulkan(
        camera.vertical_fov_radians,
        aspect,
        near_plane,
        far_plane);

    SceneUniforms scene{};
    scene.view_projection = (projection * view).values;
    scene.camera_position = {0.0F, 0.0F, 0.0F, 1.0F};
    return scene;
}

void VulkanContext::refresh_resource_stats(const RenderScene* scene) noexcept {
    const ResourceManagerStats resource_stats = resources_.stats();
    stats_.models = static_cast<std::uint32_t>(resource_stats.model_count);
    stats_.meshes = static_cast<std::uint32_t>(resource_stats.mesh_count);
    stats_.materials = static_cast<std::uint32_t>(resource_stats.material_count);
    stats_.textures = static_cast<std::uint32_t>(resource_stats.texture_count);
    stats_.samplers = static_cast<std::uint32_t>(resource_stats.sampler_count);
    stats_.vertex_memory = resource_stats.vertex_memory;
    stats_.index_memory = resource_stats.index_memory;
    stats_.texture_memory = resource_stats.texture_memory;
    stats_.draw_calls = 0;
    stats_.triangles = 0;
    stats_.render_instances = 0;
    stats_.debug_draw_calls = 0;
    stats_.debug_lines = 0;

    if (scene == nullptr) {
        return;
    }

    stats_.render_instances = static_cast<std::uint32_t>(scene->instances.size());
    stats_.debug_lines = static_cast<std::uint32_t>(scene->debug_draw.size());
    if (!scene->debug_draw.empty()) {
        stats_.debug_draw_calls = 1U;
        ++stats_.draw_calls;
    }
    for (const RenderInstance& instance : scene->instances) {
        const GpuModelResource* model = resources_.model(instance.model);
        if (model == nullptr) {
            continue;
        }
        stats_.draw_calls += static_cast<std::uint32_t>(model->submeshes.size());
        for (const GpuSubmesh& submesh : model->submeshes) {
            stats_.triangles += submesh.index_count / 3U;
        }
    }
}

bool VulkanContext::record_render_commands(FrameContext& frame,
                                           const std::uint32_t image_index,
                                           const RenderScene& render_scene,
                                           const CameraView& camera) {
    if (image_index >= depth_targets_.size()) {
        OCS_LOG_ERROR("Swapchain image index has no matching depth target");
        return false;
    }

    const SceneUniforms scene_uniforms = build_scene_uniforms(camera);
    if (!frame.scene_uniform.write(
            std::as_bytes(std::span<const SceneUniforms>(&scene_uniforms, 1)))) {
        return false;
    }

    if (!vk_check(vkResetCommandPool(device_.handle(), frame.command_pool.get(), 0),
                  "vkResetCommandPool")) {
        return false;
    }

    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (!vk_check(vkBeginCommandBuffer(frame.command_buffer, &begin_info),
                  "vkBeginCommandBuffer")) {
        return false;
    }

    if (frame.timestamp_queries) {
        vkCmdResetQueryPool(frame.command_buffer, frame.timestamp_queries.get(), 0, 2);
        vkCmdWriteTimestamp2(frame.command_buffer,
                             VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                             frame.timestamp_queries.get(),
                             0);
    }

    VkImageMemoryBarrier2 to_color{};
    to_color.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    to_color.srcStageMask = VK_PIPELINE_STAGE_2_NONE;
    to_color.srcAccessMask = VK_ACCESS_2_NONE;
    to_color.dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    to_color.dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
    to_color.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    to_color.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    to_color.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_color.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_color.image = swapchain_.images()[image_index];
    to_color.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    to_color.subresourceRange.baseMipLevel = 0;
    to_color.subresourceRange.levelCount = 1;
    to_color.subresourceRange.baseArrayLayer = 0;
    to_color.subresourceRange.layerCount = 1;

    VkDependencyInfo to_color_dependency{};
    to_color_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    to_color_dependency.imageMemoryBarrierCount = 1;
    to_color_dependency.pImageMemoryBarriers = &to_color;
    vkCmdPipelineBarrier2(frame.command_buffer, &to_color_dependency);

    VkImageMemoryBarrier2 depth_barrier{};
    depth_barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    depth_barrier.srcStageMask = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                                 VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
    depth_barrier.srcAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    depth_barrier.dstStageMask = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                                 VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
    depth_barrier.dstAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    depth_barrier.oldLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
    depth_barrier.newLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
    depth_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    depth_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    depth_barrier.image = depth_targets_[image_index].image();
    depth_barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    depth_barrier.subresourceRange.baseMipLevel = 0;
    depth_barrier.subresourceRange.levelCount = 1;
    depth_barrier.subresourceRange.baseArrayLayer = 0;
    depth_barrier.subresourceRange.layerCount = 1;

    VkDependencyInfo depth_dependency{};
    depth_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    depth_dependency.imageMemoryBarrierCount = 1;
    depth_dependency.pImageMemoryBarriers = &depth_barrier;
    vkCmdPipelineBarrier2(frame.command_buffer, &depth_dependency);

    VkRenderingAttachmentInfo color_attachment{};
    color_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
    color_attachment.imageView = swapchain_.image_views()[image_index];
    color_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color_attachment.clearValue.color = {{0.025F, 0.035F, 0.050F, 1.0F}};

    VkRenderingAttachmentInfo depth_attachment{};
    depth_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
    depth_attachment.imageView = depth_targets_[image_index].view();
    depth_attachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
    depth_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth_attachment.clearValue.depthStencil = {1.0F, 0};

    VkRenderingInfo rendering_info{};
    rendering_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    rendering_info.renderArea.offset = {0, 0};
    rendering_info.renderArea.extent = swapchain_.extent();
    rendering_info.layerCount = 1;
    rendering_info.colorAttachmentCount = 1;
    rendering_info.pColorAttachments = &color_attachment;
    rendering_info.pDepthAttachment = &depth_attachment;
    vkCmdBeginRendering(frame.command_buffer, &rendering_info);

    const VkExtent2D extent = swapchain_.extent();
    VkViewport viewport{};
    viewport.width = static_cast<float>(extent.width);
    viewport.height = static_cast<float>(extent.height);
    viewport.minDepth = 0.0F;
    viewport.maxDepth = 1.0F;
    vkCmdSetViewport(frame.command_buffer, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.extent = extent;
    vkCmdSetScissor(frame.command_buffer, 0, 1, &scissor);

    VkPipeline last_pipeline = VK_NULL_HANDLE;
    MeshHandle last_mesh{};

    for (const RenderInstance& instance : render_scene.instances) {
        const GpuModelResource* model = resources_.model(instance.model);
        if (model == nullptr) {
            OCS_LOG_ERROR("RenderScene contains a stale model handle");
            return false;
        }
        const GpuMeshResource* mesh = resources_.mesh(model->mesh);
        if (mesh == nullptr) {
            OCS_LOG_ERROR("RenderScene model contains a stale mesh handle");
            return false;
        }

        if (model->mesh != last_mesh) {
            const VkBuffer vertex_buffers[] = {mesh->vertex_buffer.handle()};
            constexpr VkDeviceSize offsets[] = {0};
            vkCmdBindVertexBuffers(frame.command_buffer, 0, 1, vertex_buffers, offsets);
            vkCmdBindIndexBuffer(frame.command_buffer,
                                 mesh->index_buffer.handle(),
                                 0,
                                 VK_INDEX_TYPE_UINT32);
            last_mesh = model->mesh;
        }

        for (const GpuSubmesh& submesh : model->submeshes) {
            const GpuMaterialResource* material = resources_.material(submesh.material);
            if (material == nullptr) {
                OCS_LOG_ERROR("Material handle became stale while recording a draw");
                return false;
            }

            const VulkanGraphicsPipeline& selected_pipeline = material->data.double_sided
                ? double_sided_pipeline_
                : culled_pipeline_;
            if (selected_pipeline.handle() != last_pipeline) {
                vkCmdBindPipeline(frame.command_buffer,
                                  VK_PIPELINE_BIND_POINT_GRAPHICS,
                                  selected_pipeline.handle());
                vkCmdBindDescriptorSets(frame.command_buffer,
                                        VK_PIPELINE_BIND_POINT_GRAPHICS,
                                        selected_pipeline.layout(),
                                        0,
                                        1,
                                        &frame.scene_set,
                                        0,
                                        nullptr);
                last_pipeline = selected_pipeline.handle();
            }

            if (material->descriptor_set == VK_NULL_HANDLE) {
                OCS_LOG_ERROR("Material descriptor set is invalid while recording a draw");
                return false;
            }
            vkCmdBindDescriptorSets(frame.command_buffer,
                                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    selected_pipeline.layout(),
                                    1,
                                    1,
                                    &material->descriptor_set,
                                    0,
                                    nullptr);

            PushConstants push{};
            push.model = instance.model_matrix.values;
            push.base_color = {
                material->data.base_color_factor.x,
                material->data.base_color_factor.y,
                material->data.base_color_factor.z,
                material->data.base_color_factor.w
            };
            push.metallic = material->data.metallic_factor;
            push.roughness = material->data.roughness_factor;
            push.alpha_cutoff = material->data.alpha_cutoff;
            if (material->data.unlit) {
                push.material_flags |= kMaterialUnlit;
            }
            if (material->data.alpha_mode == assets::MaterialAlphaMode::mask) {
                push.material_flags |= kMaterialAlphaMask;
            }
            if (material->data.double_sided) {
                push.material_flags |= kMaterialDoubleSided;
            }

            vkCmdPushConstants(frame.command_buffer,
                               selected_pipeline.layout(),
                               VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                               0,
                               static_cast<std::uint32_t>(sizeof(PushConstants)),
                               &push);
            vkCmdDrawIndexed(frame.command_buffer,
                             submesh.index_count,
                             1,
                             submesh.first_index,
                             0,
                             0);
        }
    }

    if (!render_scene.debug_draw.empty()) {
        std::vector<DebugLineVertex> debug_vertices;
        debug_vertices.reserve(render_scene.debug_draw.size() * 2U);
        for (const DebugLine& line : render_scene.debug_draw.lines()) {
            debug_vertices.push_back({line.start, line.color});
            debug_vertices.push_back({line.end, line.color});
        }

        const auto debug_bytes = std::as_bytes(std::span<const DebugLineVertex>(debug_vertices));
        const VkDeviceSize required_size = static_cast<VkDeviceSize>(debug_bytes.size_bytes());
        if (!frame.debug_line_buffer.valid() || frame.debug_line_buffer.size() < required_size) {
            frame.debug_line_buffer.shutdown();
            VkDeviceSize allocation_size = 4096U;
            while (allocation_size < required_size) {
                allocation_size *= 2U;
            }
            if (!frame.debug_line_buffer.initialize(
                    memory_allocator_,
                    allocation_size,
                    VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) {
                return false;
            }
        }
        if (!frame.debug_line_buffer.write(debug_bytes)) {
            return false;
        }

        vkCmdBindPipeline(frame.command_buffer,
                          VK_PIPELINE_BIND_POINT_GRAPHICS,
                          debug_line_pipeline_.handle());
        vkCmdBindDescriptorSets(frame.command_buffer,
                                VK_PIPELINE_BIND_POINT_GRAPHICS,
                                debug_line_pipeline_.layout(),
                                0,
                                1,
                                &frame.scene_set,
                                0,
                                nullptr);
        const VkBuffer debug_buffer = frame.debug_line_buffer.handle();
        constexpr VkDeviceSize debug_offset = 0;
        vkCmdBindVertexBuffers(frame.command_buffer, 0, 1, &debug_buffer, &debug_offset);
        vkCmdDraw(frame.command_buffer,
                  static_cast<std::uint32_t>(debug_vertices.size()),
                  1,
                  0,
                  0);
    }

    vkCmdEndRendering(frame.command_buffer);

    VkImageMemoryBarrier2 to_present{};
    to_present.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    to_present.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    to_present.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
    to_present.dstStageMask = VK_PIPELINE_STAGE_2_NONE;
    to_present.dstAccessMask = VK_ACCESS_2_NONE;
    to_present.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    to_present.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    to_present.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_present.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_present.image = swapchain_.images()[image_index];
    to_present.subresourceRange = to_color.subresourceRange;

    VkDependencyInfo to_present_dependency{};
    to_present_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    to_present_dependency.imageMemoryBarrierCount = 1;
    to_present_dependency.pImageMemoryBarriers = &to_present;
    vkCmdPipelineBarrier2(frame.command_buffer, &to_present_dependency);

    if (frame.timestamp_queries) {
        vkCmdWriteTimestamp2(frame.command_buffer,
                             VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,
                             frame.timestamp_queries.get(),
                             1);
    }

    return vk_check(vkEndCommandBuffer(frame.command_buffer), "vkEndCommandBuffer");
}

bool VulkanContext::recreate_swapchain_if_possible() {
    platform::PixelSize pixel_size{};
    if (window_ == nullptr || !window_->pixel_size(pixel_size)) {
        return false;
    }
    if (pixel_size.width <= 0 || pixel_size.height <= 0) {
        framebuffer_resized_ = true;
        return true;
    }

    if (!vk_check(vkDeviceWaitIdle(device_.handle()),
                  "vkDeviceWaitIdle before swapchain recreation")) {
        return false;
    }

    destroy_surface_render_resources();
    if (!swapchain_.recreate() || !create_surface_render_resources()) {
        return false;
    }

    framebuffer_resized_ = false;
    OCS_LOG_INFO("Vulkan swapchain + depth + material pipelines recreated");
    return true;
}

bool VulkanContext::draw_frame(const RenderScene& scene, const CameraView& camera) {
    using Clock = std::chrono::steady_clock;
    const auto cpu_begin = Clock::now();
    double cpu_sync_ms = 0.0;
    double acquire_ms = 0.0;
    double present_ms = 0.0;

    const auto elapsed_ms = [](const Clock::time_point begin,
                               const Clock::time_point end) noexcept {
        return std::chrono::duration<double, std::milli>(end - begin).count();
    };

    if (!initialized_) {
        return false;
    }

    platform::PixelSize pixel_size{};
    if (window_ == nullptr || !window_->pixel_size(pixel_size)) {
        return false;
    }
    if (pixel_size.width <= 0 || pixel_size.height <= 0) {
        return true;
    }

    if (framebuffer_resized_ && !recreate_swapchain_if_possible()) {
        return false;
    }

    FrameContext& frame = frames_[current_frame_];
    const VkFence in_flight_fence = frame.in_flight.get();
    const auto frame_fence_wait_begin = Clock::now();
    const VkResult frame_fence_wait_result = vkWaitForFences(
        device_.handle(),
        1,
        &in_flight_fence,
        VK_TRUE,
        std::numeric_limits<std::uint64_t>::max());
    cpu_sync_ms += elapsed_ms(frame_fence_wait_begin, Clock::now());
    if (!vk_check(frame_fence_wait_result, "vkWaitForFences")) {
        return false;
    }

    if (frame.timestamp_pending && frame.timestamp_queries) {
        std::array<std::uint64_t, 2> timestamps{};
        const VkResult query_result = vkGetQueryPoolResults(
            device_.handle(),
            frame.timestamp_queries.get(),
            0,
            2,
            sizeof(timestamps),
            timestamps.data(),
            sizeof(std::uint64_t),
            VK_QUERY_RESULT_64_BIT);

        if (query_result == VK_SUCCESS) {
            const std::uint32_t valid_bits = device_.timestamp_valid_bits();
            std::uint64_t delta = timestamps[1] - timestamps[0];
            if (valid_bits > 0U && valid_bits < 64U) {
                const std::uint64_t mask = (std::uint64_t{1} << valid_bits) - 1U;
                delta &= mask;
            }
            const double gpu_ms = static_cast<double>(delta) *
                                  static_cast<double>(device_.timestamp_period_ns()) /
                                  1'000'000.0;
            stats_.gpu_ms = stats_.frame_index <= 1U
                ? gpu_ms
                : stats_.gpu_ms * 0.9 + gpu_ms * 0.1;
        } else if (query_result != VK_NOT_READY) {
            if (!vk_check(query_result, "vkGetQueryPoolResults(timestamp)")) {
                return false;
            }
        }
        frame.timestamp_pending = false;
    }

    std::uint32_t image_index = 0;
    const auto acquire_begin = Clock::now();
    const VkResult acquire_result = vkAcquireNextImageKHR(
        device_.handle(),
        swapchain_.handle(),
        std::numeric_limits<std::uint64_t>::max(),
        frame.image_available.get(),
        VK_NULL_HANDLE,
        &image_index);
    acquire_ms = elapsed_ms(acquire_begin, Clock::now());
    cpu_sync_ms += acquire_ms;

    if (acquire_result == VK_ERROR_OUT_OF_DATE_KHR) {
        framebuffer_resized_ = true;
        return recreate_swapchain_if_possible();
    }
    if (acquire_result != VK_SUCCESS && acquire_result != VK_SUBOPTIMAL_KHR) {
        return vk_check(acquire_result, "vkAcquireNextImageKHR");
    }

    if (image_index >= images_in_flight_.size()) {
        OCS_LOG_ERROR("Swapchain returned an invalid image index");
        return false;
    }

    if (images_in_flight_[image_index] != VK_NULL_HANDLE) {
        const VkFence previous_fence = images_in_flight_[image_index];
        const auto image_fence_wait_begin = Clock::now();
        const VkResult image_fence_wait_result = vkWaitForFences(
            device_.handle(),
            1,
            &previous_fence,
            VK_TRUE,
            std::numeric_limits<std::uint64_t>::max());
        cpu_sync_ms += elapsed_ms(image_fence_wait_begin, Clock::now());
        if (!vk_check(image_fence_wait_result, "vkWaitForFences(swapchain image)")) {
            return false;
        }
    }
    images_in_flight_[image_index] = in_flight_fence;

    if (!vk_check(vkResetFences(device_.handle(), 1, &in_flight_fence),
                  "vkResetFences")) {
        return false;
    }

    refresh_resource_stats(&scene);

    if (!record_render_commands(frame, image_index, scene, camera)) {
        return false;
    }

    VkSemaphoreSubmitInfo wait_info{};
    wait_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
    wait_info.semaphore = frame.image_available.get();
    wait_info.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;

    VkCommandBufferSubmitInfo command_info{};
    command_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
    command_info.commandBuffer = frame.command_buffer;

    VkSemaphoreSubmitInfo signal_info{};
    signal_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
    signal_info.semaphore = frame.render_finished.get();
    signal_info.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;

    VkSubmitInfo2 submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
    submit_info.waitSemaphoreInfoCount = 1;
    submit_info.pWaitSemaphoreInfos = &wait_info;
    submit_info.commandBufferInfoCount = 1;
    submit_info.pCommandBufferInfos = &command_info;
    submit_info.signalSemaphoreInfoCount = 1;
    submit_info.pSignalSemaphoreInfos = &signal_info;

    if (!vk_check(vkQueueSubmit2(device_.graphics_queue(),
                                  1,
                                  &submit_info,
                                  frame.in_flight.get()),
                  "vkQueueSubmit2")) {
        return false;
    }
    frame.timestamp_pending = static_cast<bool>(frame.timestamp_queries);

    VkSwapchainKHR swapchains[] = {swapchain_.handle()};
    const VkSemaphore render_finished_semaphore = frame.render_finished.get();
    VkPresentInfoKHR present_info{};
    present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    present_info.waitSemaphoreCount = 1;
    present_info.pWaitSemaphores = &render_finished_semaphore;
    present_info.swapchainCount = 1;
    present_info.pSwapchains = swapchains;
    present_info.pImageIndices = &image_index;

    const auto present_begin = Clock::now();
    const VkResult present_result = vkQueuePresentKHR(device_.present_queue(), &present_info);
    present_ms = elapsed_ms(present_begin, Clock::now());
    cpu_sync_ms += present_ms;

    const bool must_recreate =
        present_result == VK_ERROR_OUT_OF_DATE_KHR ||
        present_result == VK_SUBOPTIMAL_KHR ||
        acquire_result == VK_SUBOPTIMAL_KHR ||
        framebuffer_resized_;

    if (present_result != VK_SUCCESS &&
        present_result != VK_ERROR_OUT_OF_DATE_KHR &&
        present_result != VK_SUBOPTIMAL_KHR) {
        return vk_check(present_result, "vkQueuePresentKHR");
    }

    current_frame_ = (current_frame_ + 1U) % kFramesInFlight;

    const auto frame_end = Clock::now();
    const double frame_ms = elapsed_ms(last_frame_time_, frame_end);
    const double cpu_total_ms = elapsed_ms(cpu_begin, frame_end);
    const double cpu_work_ms = std::max(0.0, cpu_total_ms - cpu_sync_ms);
    last_frame_time_ = frame_end;

    const auto smooth = [first_frame = stats_.frame_index == 0U](
                            const double previous,
                            const double sample) noexcept {
        constexpr double kSampleWeight = 0.1;
        return first_frame
            ? sample
            : previous * (1.0 - kSampleWeight) + sample * kSampleWeight;
    };

    if (frame_ms > 0.0) {
        stats_.frame_ms = smooth(stats_.frame_ms, frame_ms);
        stats_.fps = 1000.0 / stats_.frame_ms;
    }
    stats_.cpu_total_ms = smooth(stats_.cpu_total_ms, cpu_total_ms);
    stats_.cpu_work_ms = smooth(stats_.cpu_work_ms, cpu_work_ms);
    stats_.cpu_sync_ms = smooth(stats_.cpu_sync_ms, cpu_sync_ms);
    stats_.acquire_ms = smooth(stats_.acquire_ms, acquire_ms);
    stats_.present_ms = smooth(stats_.present_ms, present_ms);
    ++stats_.frame_index;

    if (must_recreate) {
        framebuffer_resized_ = true;
        return recreate_swapchain_if_possible();
    }
    return true;
}

void VulkanContext::destroy_frame_resources() noexcept {
    for (FrameContext& frame : frames_) {
        frame.scene_set = VK_NULL_HANDLE;
        // Every per-frame VulkanBuffer must be released while VkDevice is still alive.
        // debug_line_buffer is allocated lazily by the Step 8.10 debug draw path.
        frame.debug_line_buffer.shutdown();
        frame.scene_uniform.shutdown();
        frame.command_buffer = VK_NULL_HANDLE;
        frame.timestamp_pending = false;
        frame.timestamp_queries.reset();
        frame.in_flight.reset();
        frame.render_finished.reset();
        frame.image_available.reset();
        frame.command_pool.reset();
    }
}

void VulkanContext::shutdown() noexcept {
    if (device_.handle() != VK_NULL_HANDLE) {
        static_cast<void>(vkDeviceWaitIdle(device_.handle()));
    }

    destroy_surface_render_resources();
    destroy_model_resources();
    destroy_frame_resources();
    resources_.shutdown();
    destroy_descriptor_resources();
    swapchain_.shutdown();
    memory_allocator_.shutdown();
    device_.shutdown();
    instance_.shutdown();

    current_frame_ = 0;
    window_ = nullptr;
    framebuffer_resized_ = false;
    initialized_ = false;
    last_frame_time_ = {};
    stats_ = {};
}

} // namespace ocs::render::vulkan
