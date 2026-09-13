#pragma once

#include <cstdint>

namespace ocs::platform {

struct PlatformEvents {
    bool quit_requested = false;
    std::uint32_t close_requested_window_id = 0;
    std::uint32_t framebuffer_resized_window_id = 0;
    std::uint32_t keyboard_focus_window_id = 0;
    std::uint32_t mouse_window_id = 0;
    std::uint32_t mouse_focus_window_id = 0;

    // Engine-level navigation input. These names deliberately describe intent
    // rather than SDL scancodes so the app/world layer stays backend-agnostic.
    bool move_forward = false;
    bool move_backward = false;
    bool move_left = false;
    bool move_right = false;
    bool move_down = false;
    bool move_up = false;
    bool speed_boost = false;

    // Edge-triggered C key cycles CAMERA -> OBJECT -> DRIVE control modes.
    bool toggle_control_mode = false;

    // Step 8 physics-laboratory controls.
    bool toggle_physics_pause = false;
    bool single_step_physics = false;
    bool reset_physics = false;
    bool toggle_physics_debug = false;
    bool apply_test_force = false;
    bool apply_test_torque = false;

    // Mouse deltas are accumulated from SDL motion events. Rotation is active
    // while the right mouse button is held.
    float mouse_delta_x = 0.0F;
    float mouse_delta_y = 0.0F;
    bool rotate_active = false;
};

class Platform {
public:
    bool initialize();
    void shutdown() noexcept;
    [[nodiscard]] PlatformEvents poll_events();

private:
    bool previous_toggle_down_ = false;
    bool previous_pause_down_ = false;
    bool previous_step_down_ = false;
    bool previous_reset_down_ = false;
    bool previous_debug_down_ = false;
};

} // namespace ocs::platform
