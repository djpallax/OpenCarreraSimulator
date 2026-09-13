#include "ocs/platform/platform.hpp"
#include "ocs/platform/window.hpp"

#include <SDL3/SDL.h>

#include "ocs/core/log.hpp"

namespace ocs::platform {

bool Platform::initialize() {
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC)) {
        OCS_LOG_ERROR(SDL_GetError());
        return false;
    }

    OCS_LOG_INFO("SDL3 platform initialized");
    return true;
}

void Platform::shutdown() noexcept {
    SDL_Quit();
}

PlatformEvents Platform::poll_events() {
    PlatformEvents result{};
    SDL_Event event{};

    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_EVENT_QUIT) {
            result.quit_requested = true;
        }

        if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) {
            result.close_requested_window_id = event.window.windowID;
        }

        if (event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
            result.framebuffer_resized_window_id = event.window.windowID;
        }

        if (event.type == SDL_EVENT_MOUSE_MOTION) {
            result.mouse_delta_x += event.motion.xrel;
            result.mouse_delta_y += event.motion.yrel;
            result.mouse_window_id = event.motion.windowID;
        }
    }

    if (SDL_Window* keyboard_focus = SDL_GetKeyboardFocus(); keyboard_focus != nullptr) {
        result.keyboard_focus_window_id = SDL_GetWindowID(keyboard_focus);
    }
    if (SDL_Window* mouse_focus = SDL_GetMouseFocus(); mouse_focus != nullptr) {
        result.mouse_focus_window_id = SDL_GetWindowID(mouse_focus);
    }

    int key_count = 0;
    const bool* keyboard = SDL_GetKeyboardState(&key_count);
    const auto key_down = [&](const SDL_Scancode scancode) noexcept {
        const int index = static_cast<int>(scancode);
        return keyboard != nullptr && index >= 0 && index < key_count && keyboard[index];
    };

    result.move_forward = key_down(SDL_SCANCODE_W);
    result.move_backward = key_down(SDL_SCANCODE_S);
    result.move_left = key_down(SDL_SCANCODE_A);
    result.move_right = key_down(SDL_SCANCODE_D);
    result.move_down = key_down(SDL_SCANCODE_Q);
    result.move_up = key_down(SDL_SCANCODE_E);
    result.speed_boost = key_down(SDL_SCANCODE_LSHIFT) || key_down(SDL_SCANCODE_RSHIFT);

    const bool toggle_down = key_down(SDL_SCANCODE_C);
    result.toggle_control_mode = toggle_down && !previous_toggle_down_;
    previous_toggle_down_ = toggle_down;

    const bool pause_down = key_down(SDL_SCANCODE_SPACE);
    result.toggle_physics_pause = pause_down && !previous_pause_down_;
    previous_pause_down_ = pause_down;

    const bool step_down = key_down(SDL_SCANCODE_N);
    result.single_step_physics = step_down && !previous_step_down_;
    previous_step_down_ = step_down;

    const bool reset_down = key_down(SDL_SCANCODE_R);
    result.reset_physics = reset_down && !previous_reset_down_;
    previous_reset_down_ = reset_down;

    const bool debug_down = key_down(SDL_SCANCODE_V);
    result.toggle_physics_debug = debug_down && !previous_debug_down_;
    previous_debug_down_ = debug_down;

    result.apply_test_force = key_down(SDL_SCANCODE_F);
    result.apply_test_torque = key_down(SDL_SCANCODE_T);

    const SDL_MouseButtonFlags mouse_buttons = SDL_GetMouseState(nullptr, nullptr);
    result.rotate_active = (mouse_buttons & SDL_BUTTON_RMASK) != 0U;
    return result;
}

Window::~Window() {
    destroy();
}

Window::Window(Window&& other) noexcept : window_(other.window_) {
    other.window_ = nullptr;
}

Window& Window::operator=(Window&& other) noexcept {
    if (this != &other) {
        destroy();
        window_ = other.window_;
        other.window_ = nullptr;
    }
    return *this;
}

bool Window::create(const WindowDesc& desc) {
    destroy();

    SDL_WindowFlags flags = 0;
    if (desc.resizable) {
        flags |= SDL_WINDOW_RESIZABLE;
    }
    if (desc.high_pixel_density) {
        flags |= SDL_WINDOW_HIGH_PIXEL_DENSITY;
    }
    if (desc.graphics_api == WindowGraphicsApi::vulkan) {
        flags |= SDL_WINDOW_VULKAN;
    }

    window_ = SDL_CreateWindow(desc.title.c_str(), desc.width, desc.height, flags);
    if (window_ == nullptr) {
        OCS_LOG_ERROR(SDL_GetError());
        return false;
    }

    return true;
}

void Window::destroy() noexcept {
    if (window_ != nullptr) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
}

std::uint32_t Window::id() const noexcept {
    return window_ != nullptr ? SDL_GetWindowID(window_) : 0U;
}

bool Window::pixel_size(PixelSize& out_size) const noexcept {
    if (window_ == nullptr) {
        return false;
    }

    if (!SDL_GetWindowSizeInPixels(window_, &out_size.width, &out_size.height)) {
        OCS_LOG_ERROR(SDL_GetError());
        return false;
    }

    return true;
}

bool Window::set_relative_mouse_mode(const bool enabled) noexcept {
    if (window_ == nullptr) {
        return false;
    }
    if (!SDL_SetWindowRelativeMouseMode(window_, enabled)) {
        OCS_LOG_WARN(SDL_GetError());
        return false;
    }
    return true;
}

} // namespace ocs::platform
