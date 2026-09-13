#pragma once

#include <cstdint>
#include <string>

struct SDL_Window;

namespace ocs::platform {

enum class WindowGraphicsApi {
    none,
    vulkan
};

struct WindowDesc {
    std::string title = "OpenCarreraSimulator";
    int width = 1280;
    int height = 720;
    bool resizable = true;
    bool high_pixel_density = true;
    WindowGraphicsApi graphics_api = WindowGraphicsApi::vulkan;
};

struct PixelSize {
    int width = 0;
    int height = 0;
};

class Window {
public:
    Window() = default;
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
    Window(Window&& other) noexcept;
    Window& operator=(Window&& other) noexcept;

    bool create(const WindowDesc& desc);
    void destroy() noexcept;

    [[nodiscard]] bool pixel_size(PixelSize& out_size) const noexcept;
    bool set_relative_mouse_mode(bool enabled) noexcept;
    [[nodiscard]] SDL_Window* native_handle() const noexcept { return window_; }
    [[nodiscard]] std::uint32_t id() const noexcept;
    [[nodiscard]] bool valid() const noexcept { return window_ != nullptr; }

private:
    SDL_Window* window_ = nullptr;
};

} // namespace ocs::platform
