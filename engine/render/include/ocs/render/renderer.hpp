#pragma once

#include <memory>
#include <optional>
#include <string>

#include "ocs/assets/mesh.hpp"
#include "ocs/render/gpu_info.hpp"
#include "ocs/render/handles.hpp"
#include "ocs/render/render_scene.hpp"
#include "ocs/render/render_stats.hpp"

namespace ocs::platform {
class Window;
}

namespace ocs::render {

struct RendererConfig {
#ifndef NDEBUG
    bool enable_validation = true;
#else
    bool enable_validation = false;
#endif
    bool vsync = true;
    bool allow_software_gpu = false;
    std::string gpu_selector;
    std::string asset_path;
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer(Renderer&&) noexcept;
    Renderer& operator=(Renderer&&) noexcept;

    bool initialize(platform::Window& window, const RendererConfig& config = {});
    void shutdown() noexcept;

    [[nodiscard]] ModelHandle load_model(const std::string& path);
    bool unload_model(ModelHandle handle) noexcept;
    [[nodiscard]] ModelHandle default_model() const noexcept;
    [[nodiscard]] std::optional<assets::Aabb3f> model_bounds(ModelHandle handle) const noexcept;

    [[nodiscard]] bool draw_frame(const RenderScene& scene, const CameraView& camera);
    void notify_framebuffer_resized() noexcept;

    [[nodiscard]] const GPUInfo& gpu_info() const noexcept;
    [[nodiscard]] const RendererStats& stats() const noexcept;
    [[nodiscard]] bool initialized() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ocs::render
