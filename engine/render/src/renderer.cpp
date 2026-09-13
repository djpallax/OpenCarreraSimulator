#include "ocs/render/renderer.hpp"

#include <utility>

#include "vulkan/vulkan_context.hpp"

namespace ocs::render {

struct Renderer::Impl {
    vulkan::VulkanContext context;
};

Renderer::Renderer() : impl_(std::make_unique<Impl>()) {}
Renderer::~Renderer() = default;
Renderer::Renderer(Renderer&&) noexcept = default;
Renderer& Renderer::operator=(Renderer&&) noexcept = default;

bool Renderer::initialize(platform::Window& window, const RendererConfig& config) {
    return impl_->context.initialize(window, config);
}

void Renderer::shutdown() noexcept {
    impl_->context.shutdown();
}

ModelHandle Renderer::load_model(const std::string& path) {
    return impl_->context.load_model(path);
}

bool Renderer::unload_model(const ModelHandle handle) noexcept {
    return impl_->context.unload_model(handle);
}

ModelHandle Renderer::default_model() const noexcept {
    return impl_->context.default_model();
}

std::optional<assets::Aabb3f> Renderer::model_bounds(const ModelHandle handle) const noexcept {
    return impl_->context.model_bounds(handle);
}

bool Renderer::draw_frame(const RenderScene& scene, const CameraView& camera) {
    return impl_->context.draw_frame(scene, camera);
}

void Renderer::notify_framebuffer_resized() noexcept {
    impl_->context.notify_framebuffer_resized();
}

const GPUInfo& Renderer::gpu_info() const noexcept {
    return impl_->context.gpu_info();
}

const RendererStats& Renderer::stats() const noexcept {
    return impl_->context.stats();
}

bool Renderer::initialized() const noexcept {
    return impl_->context.initialized();
}

} // namespace ocs::render
