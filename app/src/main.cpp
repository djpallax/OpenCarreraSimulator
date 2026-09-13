#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ocs/assets/mesh_io.hpp"
#include "ocs/core/log.hpp"
#include "ocs/math/quatf.hpp"
#include "ocs/math/vec3d.hpp"
#include "ocs/math/vec3f.hpp"
#include "ocs/physics/fixed_step.hpp"
#include "ocs/physics/physics_world.hpp"
#include "ocs/physics/physics_thread.hpp"
#include "ocs/platform/debug_text_window.hpp"
#include "ocs/platform/platform.hpp"
#include "ocs/platform/window.hpp"
#include "ocs/render/renderer.hpp"
#include "ocs/vehicle/vehicle.hpp"
#include "ocs/world/world.hpp"

#ifndef OCS_DEFAULT_ASSET_PATH
#define OCS_DEFAULT_ASSET_PATH ""
#endif

#ifndef OCS_DEVELOPMENT_TRACK_PATH
#define OCS_DEVELOPMENT_TRACK_PATH ""
#endif

#ifndef OCS_TEST_CIRCUIT_PATH
#define OCS_TEST_CIRCUIT_PATH OCS_DEVELOPMENT_TRACK_PATH
#endif

#ifndef OCS_TEST_CIRCUIT_COLLISION_PATH
#define OCS_TEST_CIRCUIT_COLLISION_PATH ""
#endif

#ifndef OCS_VEHICLE_WHEEL_PATH
#define OCS_VEHICLE_WHEEL_PATH ""
#endif

namespace {

struct AppConfig {
    ocs::render::RendererConfig renderer{};
    std::filesystem::path track_path = OCS_TEST_CIRCUIT_PATH;
    std::filesystem::path track_collision_path = OCS_TEST_CIRCUIT_COLLISION_PATH;
    std::filesystem::path wheel_path = OCS_VEHICLE_WHEEL_PATH;
    bool log_stats = false;
    std::optional<std::filesystem::path> metrics_file{};
    double physics_hz = 500.0;
    bool single_thread_physics = false;
};

struct PhysicsFrameTelemetry {
    bool threaded = true;
    bool paused = false;
    double frequency_hz = 500.0;
    double actual_hz = 0.0;
    double fixed_dt_ms = 2.0;
    std::uint32_t steps = 0;
    double alpha = 0.0;
    double accumulator_ms = 0.0;
    double simulation_time_s = 0.0;
    double dropped_time_ms = 0.0;
    double cpu_ms = 0.0;
    double step_us = 0.0;
    double peak_step_us = 0.0;
    double scheduler_lag_us = 0.0;
    double utilization_percent = 0.0;
    double snapshot_age_ms = 0.0;
    std::uint64_t total_steps = 0;
    std::uint64_t catchup_steps = 0;
    std::uint64_t dropped_ticks = 0;
    std::uint64_t deadline_misses = 0;
    std::size_t bodies = 0;
    std::size_t static_triangles = 0;
    std::uint64_t triangle_tests = 0;
    std::uint32_t contacts = 0;
};

struct DriveFrameTelemetry {
    float throttle = 0.0F;
    float brake = 0.0F;
    float steering = 0.0F;
    double longitudinal_speed_mps = 0.0;
    double lateral_speed_mps = 0.0;
    double applied_longitudinal_tire_force_n = 0.0;
    double applied_drive_torque_nm = 0.0;
    double applied_brake_torque_nm = 0.0;
    double applied_suspension_force_n = 0.0;
    double target_yaw_rate_rps = 0.0;
    double current_yaw_rate_rps = 0.0;
    float applied_steer_torque_nm = 0.0F;
    ocs::vehicle::DriveLayout drive_layout = ocs::vehicle::DriveLayout::rear_wheel_drive;
    ocs::vehicle::VehicleContactState wheel_contacts{};
};

[[nodiscard]] constexpr const char* drive_layout_name(const ocs::vehicle::DriveLayout layout) noexcept {
    switch (layout) {
    case ocs::vehicle::DriveLayout::front_wheel_drive: return "FWD";
    case ocs::vehicle::DriveLayout::rear_wheel_drive: return "RWD";
    case ocs::vehicle::DriveLayout::all_wheel_drive: return "AWD";
    }
    return "UNKNOWN";
}

[[nodiscard]] constexpr const char* wheel_name(const std::size_t index) noexcept {
    constexpr std::array<const char*, 4> names{"FL", "FR", "RL", "RR"};
    return index < names.size() ? names[index] : "?";
}

class DriveTelemetryMailbox {
public:
    void publish(const DriveFrameTelemetry& telemetry) noexcept {
        const std::scoped_lock lock(mutex_);
        telemetry_ = telemetry;
    }

    [[nodiscard]] DriveFrameTelemetry read() const noexcept {
        const std::scoped_lock lock(mutex_);
        return telemetry_;
    }

private:
    mutable std::mutex mutex_{};
    DriveFrameTelemetry telemetry_{};
};


class PhysicsRuntime {
public:
    using Input = ocs::physics::PhysicsThreadInput;
    using StepCallback = ocs::physics::PhysicsThread::StepCallback;
    using Clock = std::chrono::steady_clock;

    PhysicsRuntime(const bool threaded, const double frequency_hz)
        : threaded_(threaded),
          threaded_runner_({
              .frequency_hz = frequency_hz,
              .max_catchup_steps = 4,
              .max_lag_seconds = 0.050,
              .start_paused = false
          }),
          inline_clock_({
              .frequency_hz = frequency_hz,
              .max_substeps = 32,
              .max_frame_delta_seconds = 0.1
          }) {}

    [[nodiscard]] ocs::physics::PhysicsWorld& setup_world() noexcept {
        return threaded_ ? threaded_runner_.setup_world() : inline_world_;
    }

    void set_monitored_body(const ocs::physics::RigidBodyHandle handle) noexcept {
        monitored_body_ = handle;
        if (threaded_) {
            threaded_runner_.set_monitored_body(handle);
        }
    }

    bool start(StepCallback callback) {
        callback_ = std::move(callback);
        if (threaded_) {
            return threaded_runner_.start(callback_);
        }
        inline_started_ = true;
        return true;
    }

    void stop() noexcept {
        if (threaded_) {
            threaded_runner_.stop();
        }
        inline_started_ = false;
    }

    [[nodiscard]] bool threaded() const noexcept { return threaded_; }

    void set_input(const Input& input) noexcept {
        if (threaded_) {
            threaded_runner_.set_input(input);
        } else {
            inline_input_ = input;
        }
    }

    void set_paused(const bool paused) {
        if (threaded_) {
            threaded_runner_.request_pause(paused);
        } else {
            inline_paused_ = paused;
        }
    }

    void request_single_step() {
        if (threaded_) {
            threaded_runner_.request_single_step();
        } else if (inline_paused_) {
            inline_single_step_pending_ = true;
        }
    }

    void reset(const ocs::physics::RigidBodyState& state) {
        if (threaded_) {
            threaded_runner_.request_reset(monitored_body_, state);
        } else {
            (void)inline_world_.reset_state(monitored_body_, state);
            inline_clock_.reset();
            inline_total_steps_ = 0;
            inline_last_result_ = {};
        }
    }

    void set_pose(const ocs::math::Vec3d position,
                  const ocs::math::Quatf orientation,
                  const bool reset_motion = true) {
        if (threaded_) {
            threaded_runner_.request_pose(monitored_body_, position, orientation, reset_motion);
        } else {
            (void)inline_world_.set_pose(monitored_body_, position, orientation, reset_motion);
        }
    }

    void update(const double delta_seconds) {
        if (threaded_ || !inline_started_) {
            return;
        }

        const auto begin = Clock::now();
        auto step = [&](const double dt) {
            if (callback_) {
                callback_(inline_world_, dt, inline_input_);
            }
            inline_world_.step(dt);
        };

        if (inline_paused_) {
            if (inline_single_step_pending_) {
                inline_last_result_ = inline_clock_.single_step(step);
                inline_single_step_pending_ = false;
            } else {
                inline_last_result_ = inline_clock_.advance(0.0, step);
            }
        } else {
            inline_last_result_ = inline_clock_.advance(delta_seconds, step);
        }
        const auto end = Clock::now();
        inline_cpu_ms_ = std::chrono::duration<double, std::milli>(end - begin).count();
        inline_total_steps_ += inline_last_result_.steps;
        inline_average_step_us_ = inline_last_result_.steps > 0U
            ? inline_cpu_ms_ * 1000.0 / static_cast<double>(inline_last_result_.steps)
            : 0.0;
    }

    [[nodiscard]] ocs::physics::PhysicsThreadSnapshot snapshot(const Clock::time_point now) const noexcept {
        if (threaded_) {
            return threaded_runner_.snapshot();
        }

        ocs::physics::PhysicsThreadSnapshot snapshot{};
        snapshot.sequence = inline_total_steps_ + 1U;
        snapshot.thread_running = false;
        snapshot.paused = inline_paused_;
        snapshot.target_hz = inline_clock_.frequency_hz();
        snapshot.actual_hz = inline_clock_.frequency_hz();
        snapshot.fixed_dt_seconds = inline_clock_.fixed_dt_seconds();
        snapshot.simulation_time_seconds = inline_last_result_.simulation_time_seconds;
        snapshot.total_steps = inline_total_steps_;
        snapshot.dropped_ticks = static_cast<std::uint64_t>(
            inline_last_result_.dropped_time_seconds / std::max(snapshot.fixed_dt_seconds, 1.0e-12));
        snapshot.last_step_us = inline_average_step_us_;
        snapshot.average_step_us = inline_average_step_us_;
        snapshot.peak_step_us = inline_average_step_us_;
        snapshot.utilization_percent = snapshot.fixed_dt_seconds > 0.0
            ? inline_average_step_us_ / (snapshot.fixed_dt_seconds * 1.0e6) * 100.0
            : 0.0;
        snapshot.step_stats = inline_world_.last_step_stats();
        snapshot.body_count = inline_world_.body_count();
        snapshot.static_triangle_count = inline_world_.static_triangle_count();
        snapshot.gravity = inline_world_.gravity();
        snapshot.monitored_body = monitored_body_;
        if (const auto* body = inline_world_.get(monitored_body_); body != nullptr) {
            snapshot.has_monitored_body = true;
            snapshot.body = *body;
        }
        snapshot.published_at = now;
        return snapshot;
    }

    [[nodiscard]] double interpolation_alpha(const ocs::physics::PhysicsThreadSnapshot& snapshot,
                                             const Clock::time_point now) const noexcept {
        return threaded_
            ? ocs::physics::physics_snapshot_interpolation_alpha(snapshot, now)
            : inline_last_result_.alpha;
    }

    [[nodiscard]] ocs::physics::RigidBodyState render_state(
        const ocs::physics::PhysicsThreadSnapshot& snapshot,
        const double alpha) const noexcept {
        if (threaded_) {
            return ocs::physics::interpolated_snapshot_state(snapshot, alpha);
        }
        return inline_world_.interpolated_state(monitored_body_, alpha);
    }

    [[nodiscard]] double inline_cpu_ms() const noexcept { return inline_cpu_ms_; }
    [[nodiscard]] double inline_accumulator_ms() const noexcept {
        return inline_last_result_.accumulator_seconds * 1000.0;
    }
    [[nodiscard]] double inline_dropped_ms() const noexcept {
        return inline_last_result_.dropped_time_seconds * 1000.0;
    }

private:
    bool threaded_ = true;
    ocs::physics::PhysicsThread threaded_runner_;
    ocs::physics::PhysicsWorld inline_world_{};
    ocs::physics::FixedStepAccumulator inline_clock_{};
    StepCallback callback_{};
    Input inline_input_{};
    ocs::physics::RigidBodyHandle monitored_body_{};
    bool inline_started_ = false;
    bool inline_paused_ = false;
    bool inline_single_step_pending_ = false;
    ocs::physics::FixedStepResult inline_last_result_{};
    std::uint64_t inline_total_steps_ = 0;
    double inline_cpu_ms_ = 0.0;
    double inline_average_step_us_ = 0.0;
};

struct MotionTracker {
    bool initialized = false;
    ocs::math::Vec3d previous_position{};
    ocs::math::Vec3d previous_velocity{};
    ocs::math::Vec3d velocity{};
    ocs::math::Vec3d acceleration{};

    void update(const ocs::math::Vec3d position, const double dt_seconds) noexcept {
        if (!initialized || !(dt_seconds > 1.0e-6)) {
            initialized = true;
            previous_position = position;
            previous_velocity = {};
            velocity = {};
            acceleration = {};
            return;
        }

        velocity = (position - previous_position) / dt_seconds;
        acceleration = (velocity - previous_velocity) / dt_seconds;
        previous_position = position;
        previous_velocity = velocity;
    }
};

AppConfig parse_config(const int argc, char** argv) {
    AppConfig config{};
    config.renderer.asset_path = OCS_DEFAULT_ASSET_PATH;

    for (int index = 1; index < argc; ++index) {
        const std::string_view argument = argv[index];
        if (argument == "--gpu" && index + 1 < argc) {
            config.renderer.gpu_selector = argv[++index];
        } else if ((argument == "--asset" || argument == "--model" || argument == "--mesh") && index + 1 < argc) {
            config.renderer.asset_path = argv[++index];
        } else if (argument == "--track" && index + 1 < argc) {
            config.track_path = argv[++index];
        } else if (argument == "--track-collision" && index + 1 < argc) {
            config.track_collision_path = argv[++index];
        } else if (argument == "--wheel" && index + 1 < argc) {
            config.wheel_path = argv[++index];
        } else if (argument == "--physics-hz" && index + 1 < argc) {
            try {
                config.physics_hz = std::clamp(std::stod(argv[++index]), 100.0, 2000.0);
            } catch (...) {
                config.physics_hz = 500.0;
            }
        } else if (argument == "--single-thread-physics") {
            config.single_thread_physics = true;
        } else if (argument == "--dedicated-physics") {
            config.single_thread_physics = false;
        } else if (argument == "--allow-software-gpu") {
            config.renderer.allow_software_gpu = true;
        } else if (argument == "--no-vsync") {
            config.renderer.vsync = false;
        } else if (argument == "--no-validation") {
            config.renderer.enable_validation = false;
        } else if (argument == "--validation") {
            config.renderer.enable_validation = true;
        } else if (argument == "--stats") {
            config.log_stats = true;
        } else if (argument == "--metrics") {
            config.metrics_file = "/tmp/opencarrera_metrics.csv";
        } else if (argument == "--metrics-file" && index + 1 < argc) {
            config.metrics_file = argv[++index];
        }
    }

    return config;
}

class MetricsWriter {
public:
    bool initialize(const std::optional<std::filesystem::path>& path) {
        if (!path.has_value()) {
            return true;
        }

        path_ = *path;
        output_.open(path_, std::ios::out | std::ios::trunc);
        if (!output_) {
            OCS_LOG_ERROR("Could not open metrics file: " + path_.string());
            return false;
        }

        output_ << "time_s,frame,fps,frame_ms,cpu_total_ms,cpu_work_ms,cpu_sync_ms,"
                   "acquire_ms,present_ms,gpu_ms,draw_calls,triangles,render_instances,"
                   "models,meshes,materials,textures,samplers,debug_lines,vertex_mib,index_mib,texture_mib,"
                   "physics_mode,physics_hz,physics_actual_hz,physics_steps,physics_total_steps,"
                   "physics_cpu_ms,physics_step_us,physics_peak_step_us,physics_utilization_pct,"
                   "physics_scheduler_lag_us,physics_snapshot_age_ms,physics_catchup_steps,"
                   "physics_dropped_ticks,physics_deadline_misses,physics_bodies,"
                   "physics_static_triangles,physics_triangle_tests,physics_contacts,"
                   "physics_sim_time_s,physics_alpha,physics_dropped_ms\n";
        output_.flush();
        start_ = std::chrono::steady_clock::now();
        OCS_LOG_INFO("Runtime metrics: " + path_.string());
        return true;
    }

    void write(const ocs::render::RendererStats& stats, const PhysicsFrameTelemetry& physics) {
        if (!output_) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_write_ < std::chrono::seconds(1)) {
            return;
        }
        last_write_ = now;

        const double time_s = std::chrono::duration<double>(now - start_).count();
        output_ << std::fixed << std::setprecision(3)
                << time_s << ',' << stats.frame_index << ',' << stats.fps << ',' << stats.frame_ms << ','
                << stats.cpu_total_ms << ',' << stats.cpu_work_ms << ',' << stats.cpu_sync_ms << ','
                << stats.acquire_ms << ',' << stats.present_ms << ',' << stats.gpu_ms << ','
                << stats.draw_calls << ',' << stats.triangles << ',' << stats.render_instances << ','
                << stats.models << ',' << stats.meshes << ',' << stats.materials << ','
                << stats.textures << ',' << stats.samplers << ',' << stats.debug_lines << ','
                << stats.vertex_memory.mebibytes_f64() << ',' << stats.index_memory.mebibytes_f64() << ','
                << stats.texture_memory.mebibytes_f64() << ','
                << (physics.threaded ? "threaded" : "single") << ','
                << physics.frequency_hz << ',' << physics.actual_hz << ',' << physics.steps << ','
                << physics.total_steps << ',' << physics.cpu_ms << ',' << physics.step_us << ','
                << physics.peak_step_us << ',' << physics.utilization_percent << ','
                << physics.scheduler_lag_us << ',' << physics.snapshot_age_ms << ','
                << physics.catchup_steps << ',' << physics.dropped_ticks << ','
                << physics.deadline_misses << ',' << physics.bodies << ',' << physics.static_triangles << ','
                << physics.triangle_tests << ',' << physics.contacts << ','
                << physics.simulation_time_s << ',' << physics.alpha << ',' << physics.dropped_time_ms << '\n';
        output_.flush();
    }

private:
    std::filesystem::path path_{};
    std::ofstream output_{};
    std::chrono::steady_clock::time_point start_{};
    std::chrono::steady_clock::time_point last_write_{};
};

std::string stats_line(const ocs::render::RendererStats& stats, const PhysicsFrameTelemetry& physics) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "FPS " << stats.fps
           << " | frame " << stats.frame_ms << " ms"
           << " | CPU " << stats.cpu_work_ms << " ms"
           << " | GPU " << stats.gpu_ms << " ms"
           << " | physics " << (physics.threaded ? "THR" : "MAIN")
           << ' ' << physics.frequency_hz << "/" << physics.actual_hz << " Hz"
           << " | step " << physics.step_us << " us"
           << " | physFrame " << physics.cpu_ms << " ms"
           << " | util " << physics.utilization_percent << "%"
           << " | contacts " << physics.contacts
           << " | debug " << stats.debug_lines << " lines"
           << " | instances " << stats.render_instances
           << " | draws " << stats.draw_calls;
    return stream.str();
}

enum class ControlMode {
    camera,
    object,
    drive
};

[[nodiscard]] constexpr const char* control_mode_name(const ControlMode mode) noexcept {
    switch (mode) {
    case ControlMode::camera: return "CAMERA";
    case ControlMode::object: return "OBJECT";
    case ControlMode::drive: return "DRIVE";
    }
    return "UNKNOWN";
}

[[nodiscard]] constexpr ControlMode next_control_mode(const ControlMode mode) noexcept {
    switch (mode) {
    case ControlMode::camera: return ControlMode::object;
    case ControlMode::object: return ControlMode::drive;
    case ControlMode::drive: return ControlMode::camera;
    }
    return ControlMode::camera;
}

struct ObjectManipulationTarget {
    bool initialized = false;
    ocs::math::Vec3d position{};
    ocs::math::Quatf orientation = ocs::math::Quatf::identity();
};

struct FreeCamera {
    ocs::world::WorldPosition position{-12.0, -16.0, 8.0};
    float yaw = 0.46F;
    float pitch = -0.22F;

    [[nodiscard]] ocs::math::Vec3f forward() const noexcept {
        const float cos_pitch = std::cos(pitch);
        return {cos_pitch * std::cos(yaw), cos_pitch * std::sin(yaw), std::sin(pitch)};
    }

    [[nodiscard]] ocs::math::Vec3f right() const noexcept {
        return forward().cross({0.0F, 0.0F, 1.0F}).normalized();
    }

    [[nodiscard]] ocs::render::CameraView render_view() const noexcept {
        return {
            .forward = forward(),
            .up = {0.0F, 0.0F, 1.0F},
            .vertical_fov_radians = ocs::math::radians(62.0F),
            .near_plane = 0.05F,
            .far_plane = 2000.0F
        };
    }
};

ocs::math::Vec3d to_vec3d(const ocs::world::WorldPosition value) noexcept {
    return {value.x, value.y, value.z};
}

ocs::world::WorldPosition to_world_position(const ocs::math::Vec3d value) noexcept {
    return {value.x, value.y, value.z};
}

void move_position(ocs::world::WorldPosition& position,
                   const ocs::math::Vec3f direction,
                   const double distance) noexcept {
    position.x += static_cast<double>(direction.x) * distance;
    position.y += static_cast<double>(direction.y) * distance;
    position.z += static_cast<double>(direction.z) * distance;
}

void update_camera(FreeCamera& camera,
                   const ocs::platform::PlatformEvents& input,
                   const double delta_seconds) {
    const double speed = (input.speed_boost ? 45.0 : 12.0) * delta_seconds;
    const ocs::math::Vec3f forward = camera.forward();
    const ocs::math::Vec3f right = camera.right();

    if (input.move_forward) move_position(camera.position, forward, speed);
    if (input.move_backward) move_position(camera.position, forward, -speed);
    if (input.move_left) move_position(camera.position, right, -speed);
    if (input.move_right) move_position(camera.position, right, speed);
    if (input.move_down) camera.position.z -= speed;
    if (input.move_up) camera.position.z += speed;

    if (input.rotate_active) {
        constexpr float kSensitivity = 0.0030F;
        camera.yaw -= input.mouse_delta_x * kSensitivity;
        camera.pitch -= input.mouse_delta_y * kSensitivity;
        camera.pitch = std::clamp(camera.pitch,
                                  ocs::math::radians(-89.0F),
                                  ocs::math::radians(89.0F));
    }
}


struct ChaseCameraState {
    bool initialized = false;
    ocs::math::Vec3d position{};
    ocs::math::Vec3d target{};
};

[[nodiscard]] ocs::math::Vec3d lerp_vec3d(const ocs::math::Vec3d a,
                                           const ocs::math::Vec3d b,
                                           const double alpha) noexcept {
    return a + (b - a) * alpha;
}

void update_drive_chase_camera(FreeCamera& camera,
                               ChaseCameraState& chase,
                               const ocs::physics::RigidBodyState& body,
                               const double delta_seconds) noexcept {
    using ocs::math::Vec3d;

    Vec3d forward = Vec3d::from_vec3f(body.orientation.rotate({1.0F, 0.0F, 0.0F}));
    forward.z = 0.0;
    if (forward.length_squared() < 1.0e-8) {
        forward = {1.0, 0.0, 0.0};
    } else {
        forward = forward.normalized();
    }

    constexpr Vec3d kWorldUp{0.0, 0.0, 1.0};
    constexpr double kDistanceBehind = 7.5;
    constexpr double kHeight = 3.0;
    constexpr double kLookAhead = 3.2;
    constexpr double kLookHeight = 0.65;
    constexpr double kVelocityLeadSeconds = 0.08;

    const Vec3d desired_position =
        body.position - forward * kDistanceBehind + kWorldUp * kHeight;
    const Vec3d desired_target =
        body.position + forward * kLookAhead + kWorldUp * kLookHeight +
        body.linear_velocity * kVelocityLeadSeconds;

    if (!chase.initialized) {
        chase.initialized = true;
        chase.position = desired_position;
        chase.target = desired_target;
    } else {
        const double dt = std::clamp(delta_seconds, 0.0, 0.1);
        const double position_alpha = 1.0 - std::exp(-8.0 * dt);
        const double target_alpha = 1.0 - std::exp(-12.0 * dt);
        chase.position = lerp_vec3d(chase.position, desired_position, position_alpha);
        chase.target = lerp_vec3d(chase.target, desired_target, target_alpha);
    }

    camera.position = to_world_position(chase.position);
    const Vec3d look = chase.target - chase.position;
    const double look_length = look.length();
    if (look_length > 1.0e-8) {
        const Vec3d direction = look / look_length;
        camera.yaw = static_cast<float>(std::atan2(direction.y, direction.x));
        camera.pitch = static_cast<float>(std::asin(std::clamp(direction.z, -1.0, 1.0)));
    }
}

struct VisualWheelRig {
    static constexpr float maximum_steer_radians = 0.42F;
    std::array<ocs::world::ObjectHandle, 4> objects{};
};

bool compute_physics_object_pose(const ocs::physics::RigidBody& body,
                                 const ocs::platform::PlatformEvents& input,
                                 const double delta_seconds,
                                 ocs::math::Vec3d& position,
                                 ocs::math::Quatf& rotation) {
    position = body.current.position;
    rotation = body.current.orientation;
    const double speed = (input.speed_boost ? 35.0 : 7.0) * delta_seconds;
    bool changed = false;

    if (input.move_forward) { position.x += speed; changed = true; }
    if (input.move_backward) { position.x -= speed; changed = true; }
    if (input.move_left) { position.y += speed; changed = true; }
    if (input.move_right) { position.y -= speed; changed = true; }
    if (input.move_down) { position.z -= speed; changed = true; }
    if (input.move_up) { position.z += speed; changed = true; }

    if (input.rotate_active) {
        constexpr float kSensitivity = 0.0040F;
        const auto yaw_rotation = ocs::math::Quatf::from_axis_angle(
            {0.0F, 0.0F, 1.0F}, -input.mouse_delta_x * kSensitivity);
        const auto pitch_rotation = ocs::math::Quatf::from_axis_angle(
            {0.0F, 1.0F, 0.0F}, -input.mouse_delta_y * kSensitivity);
        rotation = (yaw_rotation * rotation * pitch_rotation).normalized();
        changed = true;
    }

    return changed;
}


struct TrackSpawn {
    ocs::math::Vec3d surface_point{};
    ocs::math::Vec3d surface_normal{0.0, 0.0, 1.0};
    ocs::math::Vec3d forward{1.0, 0.0, 0.0};
};

[[nodiscard]] std::vector<ocs::physics::StaticTriangle> collision_triangles(
    const ocs::assets::MeshData& mesh) {
    std::vector<ocs::physics::StaticTriangle> triangles;
    triangles.reserve(mesh.indices.size() / 3U);
    for (std::size_t index = 0; index + 2U < mesh.indices.size(); index += 3U) {
        const std::uint32_t i0 = mesh.indices[index + 0U];
        const std::uint32_t i1 = mesh.indices[index + 1U];
        const std::uint32_t i2 = mesh.indices[index + 2U];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size()) {
            continue;
        }
        triangles.push_back({
            .a = ocs::math::Vec3d::from_vec3f(mesh.vertices[i0].position),
            .b = ocs::math::Vec3d::from_vec3f(mesh.vertices[i1].position),
            .c = ocs::math::Vec3d::from_vec3f(mesh.vertices[i2].position)
        });
    }
    return triangles;
}

[[nodiscard]] TrackSpawn choose_track_spawn(const ocs::assets::MeshData& mesh) noexcept {
    struct FloorTriangle {
        ocs::math::Vec3d a{};
        ocs::math::Vec3d b{};
        ocs::math::Vec3d c{};
        ocs::math::Vec3d normal{};
    };

    std::vector<FloorTriangle> floor_triangles;
    floor_triangles.reserve(mesh.indices.size() / 3U);
    std::map<int, std::size_t> edge_length_histogram;

    const auto register_edge = [&](const ocs::math::Vec3d a, const ocs::math::Vec3d b) {
        const double length = (b - a).length();
        if (length > 1.0) {
            const int decimeters = static_cast<int>(std::lround(length * 10.0));
            ++edge_length_histogram[decimeters];
        }
    };

    for (std::size_t index = 0; index + 2U < mesh.indices.size(); index += 3U) {
        const std::uint32_t i0 = mesh.indices[index + 0U];
        const std::uint32_t i1 = mesh.indices[index + 1U];
        const std::uint32_t i2 = mesh.indices[index + 2U];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size()) {
            continue;
        }

        const ocs::math::Vec3d a = ocs::math::Vec3d::from_vec3f(mesh.vertices[i0].position);
        const ocs::math::Vec3d b = ocs::math::Vec3d::from_vec3f(mesh.vertices[i1].position);
        const ocs::math::Vec3d c = ocs::math::Vec3d::from_vec3f(mesh.vertices[i2].position);
        const ocs::math::Vec3d cross = (b - a).cross(c - a);
        const double magnitude = cross.length();
        if (!(magnitude > 1.0e-8)) {
            continue;
        }
        const ocs::math::Vec3d normal = cross / magnitude;
        if (normal.z < 0.85) {
            continue;
        }

        floor_triangles.push_back({a, b, c, normal});
        register_edge(a, b);
        register_edge(b, c);
        register_edge(c, a);
    }

    if (floor_triangles.empty()) {
        return {};
    }

    // A Blender curve/extruded test track has one nearly constant cross-track
    // width while segment lengths and diagonals vary. The most frequent edge
    // length is therefore a good centerline cue without hard-coding 20 m.
    int modal_edge_decimeters = 0;
    std::size_t modal_count = 0;
    for (const auto& [length_bin, count] : edge_length_histogram) {
        if (count > modal_count) {
            modal_count = count;
            modal_edge_decimeters = length_bin;
        }
    }
    const double modal_edge_length = static_cast<double>(modal_edge_decimeters) / 10.0;

    TrackSpawn best{};
    double best_distance_squared = std::numeric_limits<double>::max();
    bool found_width_edge = false;

    const auto consider_edge = [&](const ocs::math::Vec3d a,
                                   const ocs::math::Vec3d b,
                                   const ocs::math::Vec3d normal) {
        const ocs::math::Vec3d edge = b - a;
        const double length = edge.length();
        if (!(length > 1.0e-8) || std::abs(length - modal_edge_length) > 0.16) {
            return;
        }

        const ocs::math::Vec3d midpoint = (a + b) * 0.5;
        const double distance_squared = midpoint.x * midpoint.x + midpoint.y * midpoint.y;
        if (found_width_edge && distance_squared >= best_distance_squared) {
            return;
        }

        ocs::math::Vec3d width_direction = edge / length;
        ocs::math::Vec3d forward = normal.cross(width_direction).normalized();
        if (forward.x < -1.0e-6 || (std::abs(forward.x) <= 1.0e-6 && forward.y < 0.0)) {
            forward = -forward;
        }

        found_width_edge = true;
        best_distance_squared = distance_squared;
        best = {.surface_point = midpoint, .surface_normal = normal, .forward = forward};
    };

    for (const FloorTriangle& triangle : floor_triangles) {
        consider_edge(triangle.a, triangle.b, triangle.normal);
        consider_edge(triangle.b, triangle.c, triangle.normal);
        consider_edge(triangle.c, triangle.a, triangle.normal);
    }

    if (found_width_edge) {
        return best;
    }

    // Fallback for arbitrary meshes: choose the upward-facing triangle nearest
    // the origin and use its longest horizontal edge to establish a tangent.
    for (const FloorTriangle& triangle : floor_triangles) {
        const ocs::math::Vec3d centroid = (triangle.a + triangle.b + triangle.c) / 3.0;
        const double distance_squared = centroid.x * centroid.x + centroid.y * centroid.y;
        if (distance_squared < best_distance_squared) {
            best_distance_squared = distance_squared;
            const ocs::math::Vec3d edge = triangle.b - triangle.a;
            ocs::math::Vec3d forward = triangle.normal.cross(edge.normalized()).normalized();
            if (forward.x < 0.0) {
                forward = -forward;
            }
            best = {.surface_point = centroid, .surface_normal = triangle.normal, .forward = forward};
        }
    }
    return best;
}

[[nodiscard]] ocs::math::Vec3f visual_center_from_bounds(
    const std::optional<ocs::assets::Aabb3f>& bounds,
    const float scale) noexcept {
    if (!bounds.has_value()) {
        return {};
    }
    return (bounds->min + bounds->max) * (0.5F * scale);
}

[[nodiscard]] ocs::physics::BoxCollisionShape collision_box_from_bounds(
    const std::optional<ocs::assets::Aabb3f>& bounds,
    const float scale) noexcept {
    if (!bounds.has_value()) {
        return {.center_local = {}, .half_extents = {0.5F, 0.5F, 0.5F}};
    }
    const ocs::math::Vec3f half = (bounds->max - bounds->min) * (0.5F * scale);
    return {
        .center_local = {},
        .half_extents = {
            std::max(half.x, 0.05F),
            std::max(half.y, 0.05F),
            std::max(half.z, 0.05F)
        }
    };
}

[[nodiscard]] ocs::math::Vec3f box_inertia_diagonal(const double mass,
                                                      const ocs::math::Vec3f half) noexcept {
    const float mass_f = static_cast<float>(mass);
    return {
        mass_f * (half.y * half.y + half.z * half.z) / 3.0F,
        mass_f * (half.x * half.x + half.z * half.z) / 3.0F,
        mass_f * (half.x * half.x + half.y * half.y) / 3.0F
    };
}

void apply_drive_lab_control(ocs::physics::PhysicsWorld& physics,
                             const ocs::physics::RigidBodyHandle handle,
                             const ocs::vehicle::VehicleConfig& vehicle,
                             ocs::vehicle::VehicleRuntimeState& vehicle_runtime,
                             const ocs::physics::PhysicsThreadInput& input,
                             const double fixed_dt_seconds,
                             DriveFrameTelemetry& telemetry) noexcept {
    ocs::physics::RigidBody* body = physics.get(handle);
    if (body == nullptr || !(fixed_dt_seconds > 0.0)) {
        return;
    }

    const ocs::math::Vec3d forward = ocs::math::Vec3d::from_vec3f(
        body->current.orientation.rotate({1.0F, 0.0F, 0.0F})).normalized();
    const ocs::math::Vec3d left = ocs::math::Vec3d::from_vec3f(
        body->current.orientation.rotate({0.0F, 1.0F, 0.0F})).normalized();
    const ocs::math::Vec3d body_up = ocs::math::Vec3d::from_vec3f(
        body->current.orientation.rotate({0.0F, 0.0F, 1.0F})).normalized();
    const ocs::vehicle::VehicleContactState& wheel_contacts = vehicle_runtime.contacts;
    const ocs::math::Vec3d steer_axis = wheel_contacts.wheels_in_contact > 0U
        ? wheel_contacts.average_contact_normal
        : body_up;

    const double longitudinal_speed = body->current.linear_velocity.dot(forward);
    const double lateral_speed = body->current.linear_velocity.dot(left);
    const float throttle = input.drive_enabled && input.move_forward ? 1.0F : 0.0F;
    const float brake = input.drive_enabled && input.move_backward ? 1.0F : 0.0F;
    const float steering = input.drive_enabled
        ? (input.move_left ? 1.0F : 0.0F) - (input.move_right ? 1.0F : 0.0F)
        : 0.0F;
    const double boost = input.drive_enabled && input.speed_boost ? 1.6 : 1.0;

    constexpr double kLateralResponse = 7.5;
    constexpr double kMaximumLateralAcceleration = 8.5;
    constexpr double kYawRateResponse = 7.0;
    constexpr double kMaximumYawAcceleration = 2.5;

    // Step 9.4/9.5: throttle and braking now act on wheel rotational inertia.
    // Longitudinal chassis force exists only as a tire reaction to wheel slip at
    // a real contact patch; an airborne driven wheel can spin but cannot propel.
    const ocs::vehicle::LongitudinalTireResult tire_result =
        ocs::vehicle::update_longitudinal_tire_dynamics(
            physics, handle, *body, vehicle, vehicle_runtime,
            static_cast<double>(throttle), static_cast<double>(brake), boost,
            static_cast<double>(steering), fixed_dt_seconds);

    // Temporary lateral grip helper until tire slip-angle forces arrive. Scale it
    // by wheel contact count so one remaining wheel cannot magically provide the
    // full-car lateral authority.
    if (input.drive_enabled && wheel_contacts.wheels_in_contact > 0U) {
        const double contact_factor =
            static_cast<double>(wheel_contacts.wheels_in_contact) /
            static_cast<double>(ocs::vehicle::kWheelCount);
        const double lateral_acceleration = std::clamp(
            -lateral_speed * kLateralResponse,
            -kMaximumLateralAcceleration,
            kMaximumLateralAcceleration) * contact_factor;
        physics.add_force(handle, left * (body->mass.mass * lateral_acceleration));
    }

    std::uint32_t steerable_contacts = 0U;
    std::uint32_t steerable_wheels = 0U;
    for (const auto& wheel : wheel_contacts.wheels) {
        if (wheel.steerable) {
            ++steerable_wheels;
            if (wheel.in_contact) {
                ++steerable_contacts;
            }
        }
    }

    // Step 9.3.1 handling hotfix: track a physically meaningful yaw-rate target
    // instead of continuously adding yaw torque. Bicycle geometry makes target yaw
    // proportional to longitudinal speed, so the vehicle cannot rotate in place.
    // The controller also remains active with centred steering, damping residual yaw
    // rather than letting the chassis keep spinning as if it were on ice.
    double target_yaw_rate = 0.0;
    const double current_yaw_rate = ocs::math::Vec3d::from_vec3f(
        body->current.angular_velocity).dot(steer_axis);
    float steer_torque = 0.0F;
    if (input.drive_enabled && steerable_contacts > 0U) {
        const double contact_factor = steerable_wheels > 0U
            ? static_cast<double>(steerable_contacts) / static_cast<double>(steerable_wheels)
            : 0.0;
        target_yaw_rate = ocs::vehicle::provisional_steering_yaw_rate(
            vehicle, longitudinal_speed, static_cast<double>(steering)) * contact_factor;
        const double desired_yaw_acceleration = std::clamp(
            (target_yaw_rate - current_yaw_rate) * kYawRateResponse,
            -kMaximumYawAcceleration,
            kMaximumYawAcceleration);
        const float yaw_inertia = std::max(body->mass.inertia_diagonal.z, 1.0F);
        steer_torque = static_cast<float>(desired_yaw_acceleration) * yaw_inertia;
        physics.add_torque(handle, (steer_axis * static_cast<double>(steer_torque)).to_vec3f());
    }

    const double suspension_force = telemetry.applied_suspension_force_n;
    telemetry = {
        .throttle = throttle,
        .brake = brake,
        .steering = steering,
        .longitudinal_speed_mps = longitudinal_speed,
        .lateral_speed_mps = lateral_speed,
        .applied_longitudinal_tire_force_n = tire_result.total_longitudinal_force_n,
        .applied_drive_torque_nm = tire_result.total_drive_torque_nm,
        .applied_brake_torque_nm = tire_result.total_brake_torque_nm,
        .applied_suspension_force_n = suspension_force,
        .target_yaw_rate_rps = target_yaw_rate,
        .current_yaw_rate_rps = current_yaw_rate,
        .applied_steer_torque_nm = steer_torque,
        .drive_layout = vehicle.drive_layout,
        .wheel_contacts = wheel_contacts
    };
}

ocs::world::Transform visual_transform_from_body(const ocs::math::Vec3d body_position,
                                                    const ocs::math::Quatf orientation,
                                                    const ocs::math::Vec3f visual_center_local,
                                                    const float scale) noexcept {
    const ocs::math::Vec3d rotated_center = ocs::math::Vec3d::from_vec3f(
        orientation.rotate(visual_center_local));
    ocs::world::Transform transform{};
    transform.position = to_world_position(body_position - rotated_center);
    transform.rotation = orientation;
    transform.scale = {scale, scale, scale};
    return transform;
}

[[nodiscard]] ocs::world::Transform wheel_transform_from_body(
    const ocs::physics::RigidBodyState& body,
    const ocs::math::Vec3f local_center,
    const float steering_radians,
    const float rolling_radians,
    const ocs::math::Vec3f visual_center_local) noexcept {
    const auto steer = ocs::math::Quatf::from_axis_angle({0.0F, 0.0F, 1.0F}, steering_radians);
    const auto roll = ocs::math::Quatf::from_axis_angle({0.0F, 1.0F, 0.0F}, rolling_radians);
    const ocs::math::Quatf wheel_orientation = (body.orientation * steer * roll).normalized();

    const ocs::math::Vec3d center_world = body.position + ocs::math::Vec3d::from_vec3f(
        body.orientation.rotate(local_center));
    const ocs::math::Vec3d rotated_visual_center = ocs::math::Vec3d::from_vec3f(
        wheel_orientation.rotate(visual_center_local));

    ocs::world::Transform transform{};
    transform.position = to_world_position(center_world - rotated_visual_center);
    transform.rotation = wheel_orientation;
    return transform;
}

void update_visual_wheels(ocs::world::World& world,
                          VisualWheelRig& wheels,
                          const ocs::vehicle::VehicleConfig& vehicle,
                          const ocs::vehicle::VehicleContactState& contacts,
                          const ocs::physics::RigidBodyState& body,
                          const ocs::math::Vec3f wheel_visual_center,
                          const float steering_input) noexcept {
    const float steer = std::clamp(steering_input, -1.0F, 1.0F) *
        VisualWheelRig::maximum_steer_radians;

    for (std::size_t index = 0; index < wheels.objects.size(); ++index) {
        if (ocs::world::WorldObject* object = world.get(wheels.objects[index]); object != nullptr) {
            const auto& wheel_config = vehicle.wheels[index];
            const auto& contact = contacts.wheels[index];
            const double length = contact.suspension_length > 0.0
                ? contact.suspension_length
                : wheel_config.rest_length;
            ocs::math::Vec3f wheel_center_local = wheel_config.suspension_mount_local;
            wheel_center_local.z -= static_cast<float>(length);
            object->transform = wheel_transform_from_body(
                body,
                wheel_center_local,
                wheel_config.steerable ? steer : 0.0F,
                static_cast<float>(-contact.wheel_rotation_radians),
                wheel_visual_center);
        }
    }
}

ocs::math::Vec3f camera_relative_point(const ocs::math::Vec3d world_point,
                                             const ocs::world::WorldPosition camera_position) noexcept {
    return {
        static_cast<float>(world_point.x - camera_position.x),
        static_cast<float>(world_point.y - camera_position.y),
        static_cast<float>(world_point.z - camera_position.z)
    };
}

ocs::math::Vec3d clamped_vector(const ocs::math::Vec3d value, const double maximum_length) noexcept {
    const double length = value.length();
    if (!(length > maximum_length) || !(maximum_length > 0.0)) {
        return value;
    }
    return value * (maximum_length / length);
}

void append_physics_debug_draw(ocs::render::RenderScene& scene,
                               const ocs::physics::RigidBody& body,
                               const ocs::physics::RigidBodyState& state,
                               const ocs::world::WorldPosition camera_position) {
    using ocs::math::Vec3d;
    using ocs::math::Vec3f;
    using ocs::math::Vec4f;

    auto& debug = scene.debug_draw;
    debug.reserve(64U);

    constexpr Vec4f kComColor{1.0F, 0.85F, 0.10F, 1.0F};
    constexpr Vec4f kObbColor{0.10F, 0.90F, 1.0F, 1.0F};
    constexpr Vec4f kVelocityColor{0.20F, 1.0F, 0.25F, 1.0F};
    constexpr Vec4f kAngularColor{1.0F, 0.25F, 1.0F, 1.0F};
    constexpr Vec4f kForceColor{1.0F, 0.45F, 0.10F, 1.0F};
    constexpr Vec4f kSupportColor{0.20F, 0.55F, 1.0F, 1.0F};
    constexpr Vec4f kContactColor{1.0F, 0.15F, 0.15F, 1.0F};
    constexpr Vec4f kContactNormalColor{1.0F, 0.95F, 0.20F, 1.0F};

    const Vec3d com_world = state.position;
    const Vec3f com = camera_relative_point(com_world, camera_position);
    debug.cross(com, 0.28F, kComColor);

    // Body-local axes are intentionally RGB: +X forward, +Y left, +Z up.
    const Vec3d forward = Vec3d::from_vec3f(state.orientation.rotate({1.0F, 0.0F, 0.0F}));
    const Vec3d left = Vec3d::from_vec3f(state.orientation.rotate({0.0F, 1.0F, 0.0F}));
    const Vec3d up = Vec3d::from_vec3f(state.orientation.rotate({0.0F, 0.0F, 1.0F}));
    debug.line(com, camera_relative_point(com_world + forward * 1.4, camera_position),
               {1.0F, 0.15F, 0.15F, 1.0F});
    debug.line(com, camera_relative_point(com_world + left * 1.4, camera_position),
               {0.15F, 1.0F, 0.15F, 1.0F});
    debug.line(com, camera_relative_point(com_world + up * 1.4, camera_position),
               {0.15F, 0.35F, 1.0F, 1.0F});

    std::array<Vec3f, 8> corners{};
    const Vec3f center = body.collision_box.center_local;
    const Vec3f half = body.collision_box.half_extents;
    std::size_t corner_index = 0;
    for (const float sx : {-1.0F, 1.0F}) {
        for (const float sy : {-1.0F, 1.0F}) {
            for (const float sz : {-1.0F, 1.0F}) {
                const Vec3f local{
                    center.x + sx * half.x,
                    center.y + sy * half.y,
                    center.z + sz * half.z
                };
                const Vec3d world = com_world + Vec3d::from_vec3f(state.orientation.rotate(local));
                corners[corner_index++] = camera_relative_point(world, camera_position);
            }
        }
    }
    debug.box(corners, kObbColor);

    const Vec3d velocity_vector = clamped_vector(state.linear_velocity * 0.45, 10.0);
    if (velocity_vector.length_squared() > 1.0e-8) {
        debug.line(com,
                   camera_relative_point(com_world + velocity_vector, camera_position),
                   kVelocityColor);
    }

    const Vec3d angular_world = Vec3d::from_vec3f(state.angular_velocity);
    if (angular_world.length_squared() > 1.0e-8) {
        const Vec3d angular_vector = clamped_vector(angular_world * 0.9, 4.0);
        debug.line(com,
                   camera_relative_point(com_world + angular_vector, camera_position),
                   kAngularColor);
    }

    const Vec3d force_vector = clamped_vector(body.last_external_force / 5000.0, 6.0);
    if (force_vector.length_squared() > 1.0e-8) {
        debug.line(com,
                   camera_relative_point(com_world + force_vector, camera_position),
                   kForceColor);
    }

    if (body.grounded) {
        debug.line(com,
                   camera_relative_point(com_world + body.support_normal * 1.8, camera_position),
                   kSupportColor);
    }

    for (std::uint32_t index = 0; index < body.debug_contact_count; ++index) {
        const auto& contact = body.debug_contacts[index];
        const Vec3f point = camera_relative_point(contact.point, camera_position);
        debug.cross(point, 0.14F, kContactColor);
        debug.line(point,
                   camera_relative_point(contact.point + contact.normal * 1.25, camera_position),
                   kContactNormalColor);
    }
}

void append_wheel_contact_debug_draw(
    ocs::render::RenderScene& scene,
    const ocs::vehicle::VehicleContactState& contacts,
    const ocs::world::WorldPosition camera_position) {
    using ocs::math::Vec4f;

    constexpr Vec4f kContactColor{0.20F, 1.0F, 0.30F, 1.0F};
    constexpr Vec4f kMissColor{1.0F, 0.20F, 0.20F, 1.0F};
    constexpr Vec4f kDrivenColor{1.0F, 0.55F, 0.10F, 1.0F};
    constexpr Vec4f kNormalColor{0.95F, 0.95F, 0.20F, 1.0F};
    constexpr Vec4f kLongitudinalForceColor{0.20F, 0.75F, 1.0F, 1.0F};

    for (const auto& wheel : contacts.wheels) {
        const auto start = camera_relative_point(wheel.mount_world, camera_position);
        const auto hub = camera_relative_point(wheel.wheel_center_world, camera_position);
        const auto end_world = wheel.in_contact ? wheel.contact_point : wheel.probe_end_world;
        const auto end = camera_relative_point(end_world, camera_position);
        const Vec4f line_color = wheel.driven && wheel.in_contact
            ? kDrivenColor
            : (wheel.in_contact ? kContactColor : kMissColor);
        scene.debug_draw.line(start, hub, line_color);
        scene.debug_draw.line(hub, end, line_color);
        if (wheel.in_contact) {
            scene.debug_draw.cross(end, 0.09F, line_color);
            const double force_scale = std::clamp(wheel.normal_force / 5000.0, 0.0, 1.5);
            scene.debug_draw.line(
                end,
                camera_relative_point(
                    wheel.contact_point + wheel.contact_normal * (0.55 + force_scale),
                    camera_position),
                kNormalColor);
            if (wheel.longitudinal_direction_world.length_squared() > 1.0e-12 &&
                std::abs(wheel.longitudinal_tire_force_n) > 1.0) {
                const double tire_scale = std::clamp(
                    wheel.longitudinal_tire_force_n / 3500.0, -1.5, 1.5);
                scene.debug_draw.line(
                    end,
                    camera_relative_point(
                        wheel.contact_point + wheel.longitudinal_direction_world * tire_scale,
                        camera_position),
                    kLongitudinalForceColor);
            }
        }
    }
}

ocs::math::Vec3f quaternion_euler_degrees(const ocs::math::Quatf q_raw) noexcept {
    const ocs::math::Quatf q = q_raw.normalized();
    const float sinr_cosp = 2.0F * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0F - 2.0F * (q.x * q.x + q.y * q.y);
    const float roll = std::atan2(sinr_cosp, cosr_cosp);

    const float sinp = 2.0F * (q.w * q.y - q.z * q.x);
    const float pitch = std::abs(sinp) >= 1.0F
        ? std::copysign(ocs::math::radians(90.0F), sinp)
        : std::asin(sinp);

    const float siny_cosp = 2.0F * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0F - 2.0F * (q.y * q.y + q.z * q.z);
    const float yaw = std::atan2(siny_cosp, cosy_cosp);
    constexpr float kRadToDeg = 57.29577951308232F;
    return {roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg};
}

std::string vec3d_text(const ocs::math::Vec3d value, const int precision = 3) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision)
           << '(' << value.x << ", " << value.y << ", " << value.z << ')';
    return stream.str();
}

std::string vec3f_text(const ocs::math::Vec3f value, const int precision = 3) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision)
           << '(' << value.x << ", " << value.y << ", " << value.z << ')';
    return stream.str();
}

std::vector<std::string> build_debug_lines(
    const ocs::render::Renderer& renderer,
    const ocs::world::World& world,
    const ocs::physics::PhysicsThreadSnapshot& physics_snapshot,
    const ocs::world::ObjectHandle movable_object,
    const FreeCamera& camera,
    const MotionTracker& camera_motion,
    const ControlMode control_mode,
    const PhysicsFrameTelemetry& physics_frame,
    const DriveFrameTelemetry& drive_frame,
    const bool physics_debug_enabled) {

    const auto& stats = renderer.stats();
    const auto& gpu = renderer.gpu_info();
    std::vector<std::string> lines;
    lines.reserve(64);

    lines.push_back("OpenCarreraSimulator - Vehicle Debug Telemetry");
    lines.push_back("============================================================");
    lines.push_back("[ENGINE]");
    lines.push_back("Frame: " + std::to_string(stats.frame_index) +
                    " | FPS: " + std::to_string(stats.fps) +
                    " | mode: " + control_mode_name(control_mode));
    lines.push_back("GPU: " + gpu.name + " | " + gpu.vendor + " | " +
                    std::string(ocs::render::gpu_type_name(gpu.type)));

    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3)
               << "Render ms: frame=" << stats.frame_ms
               << " cpuWork=" << stats.cpu_work_ms
               << " sync=" << stats.cpu_sync_ms
               << " gpu=" << stats.gpu_ms;
        lines.push_back(stream.str());
    }
    lines.push_back("Render: instances=" + std::to_string(stats.render_instances) +
                    " draws=" + std::to_string(stats.draw_calls) +
                    " triangles=" + std::to_string(stats.triangles));
    lines.push_back("3D physics debug: " + std::string(physics_debug_enabled ? "ON" : "OFF") +
                    " | lines=" + std::to_string(stats.debug_lines) +
                    " | drawCalls=" + std::to_string(stats.debug_draw_calls));
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2)
               << "Resources: models=" << stats.models << " meshes=" << stats.meshes
               << " materials=" << stats.materials << " textures=" << stats.textures
               << " samplers=" << stats.samplers << " texMiB="
               << stats.texture_memory.mebibytes_f64();
        lines.push_back(stream.str());
    }

    lines.push_back("");
    lines.push_back("[PHYSICS]");
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3)
               << "State: " << (physics_frame.paused ? "PAUSED" : "RUNNING")
               << " | target=" << physics_frame.frequency_hz << " Hz"
               << " | fixedDt=" << physics_frame.fixed_dt_ms << " ms"
               << " | steps/frame=" << physics_frame.steps;
        lines.push_back(stream.str());
    }
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3)
               << "Timing: mode=" << (physics_frame.threaded ? "DEDICATED THREAD" : "SINGLE THREAD")
               << " actual=" << physics_frame.actual_hz << " Hz"
               << " stepAvg=" << physics_frame.step_us << " us"
               << " peak=" << physics_frame.peak_step_us << " us"
               << " util=" << physics_frame.utilization_percent << "%";
        lines.push_back(stream.str());
    }
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3)
               << "Scheduler: lag=" << physics_frame.scheduler_lag_us << " us"
               << " snapshotAge=" << physics_frame.snapshot_age_ms << " ms"
               << " catchup=" << physics_frame.catchup_steps
               << " droppedTicks=" << physics_frame.dropped_ticks
               << " deadlineMiss=" << physics_frame.deadline_misses;
        lines.push_back(stream.str());
    }
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3)
               << "Simulation: t=" << physics_frame.simulation_time_s << " s"
               << " dropped=" << physics_frame.dropped_time_ms << " ms"
               << " totalSteps=" << physics_frame.total_steps
               << " bodies=" << physics_frame.bodies
               << " contacts=" << physics_frame.contacts
               << " staticTris=" << physics_frame.static_triangles
               << " triTests=" << physics_frame.triangle_tests;
        lines.push_back(stream.str());
    }
    lines.push_back("Gravity: " + vec3d_text(physics_snapshot.gravity, 5) + " m/s^2");
    lines.push_back("Controls: C CAMERA/OBJECT/DRIVE | DRIVE=chase camera | V 3D debug | SPACE pause | N step | R reset");

    lines.push_back("");
    lines.push_back("[DRIVE LAB CONTROLLER]");
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3)
               << "input throttle=" << drive_frame.throttle
               << " brake=" << drive_frame.brake
               << " steering=" << drive_frame.steering;
        lines.push_back(stream.str());
    }
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3)
               << "speed longitudinal=" << drive_frame.longitudinal_speed_mps << " m/s"
               << " lateral=" << drive_frame.lateral_speed_mps << " m/s";
        lines.push_back(stream.str());
    }
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(1)
               << "longitudinal tire Fx=" << drive_frame.applied_longitudinal_tire_force_n << " N"
               << " driveTorque=" << drive_frame.applied_drive_torque_nm << " Nm"
               << " brakeTorque=" << drive_frame.applied_brake_torque_nm << " Nm"
               << " suspension=" << drive_frame.applied_suspension_force_n << " N"
               << " steerTorque=" << drive_frame.applied_steer_torque_nm << " Nm";
        lines.push_back(stream.str());
    }
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3)
               << "yaw rate target=" << drive_frame.target_yaw_rate_rps << " rad/s"
               << " actual=" << drive_frame.current_yaw_rate_rps << " rad/s";
        lines.push_back(stream.str());
    }
    lines.push_back(std::string("drivetrain=") + drive_layout_name(drive_frame.drive_layout) +
                    " | wheelContacts=" + std::to_string(drive_frame.wheel_contacts.wheels_in_contact) + "/4" +
                    " | drivenContacts=" + std::to_string(drive_frame.wheel_contacts.driven_wheels_in_contact) +
                    "/" + std::to_string(drive_frame.wheel_contacts.driven_wheels));
    for (std::size_t index = 0; index < drive_frame.wheel_contacts.wheels.size(); ++index) {
        const auto& wheel = drive_frame.wheel_contacts.wheels[index];
        std::ostringstream stream;
        stream << " " << wheel_name(index)
               << " driven=" << (wheel.driven ? "yes" : "no")
               << " contact=" << (wheel.in_contact ? "yes" : "no");
        if (wheel.in_contact) {
            stream << std::fixed << std::setprecision(3)
                   << " len=" << wheel.suspension_length << " m"
                   << " comp=" << wheel.compression * 1000.0 << " mm"
                   << " vel=" << wheel.compression_velocity << " m/s"
                   << " Fz=" << wheel.normal_force << " N"
                   << " Vx=" << wheel.longitudinal_ground_speed_mps << " m/s"
                   << " omega=" << wheel.wheel_angular_velocity_rad_s << " rad/s"
                   << " kappa=" << wheel.slip_ratio
                   << " Fx=" << wheel.longitudinal_tire_force_n << " N"
                   << " normal=" << vec3d_text(wheel.contact_normal, 2);
        }
        lines.push_back(stream.str());
    }
    lines.push_back("W throttle | S brake | A/D steer | Shift debug boost | camera=CHASE");

    lines.push_back("");
    lines.push_back("[CAMERA]");
    lines.push_back("Position world m: " + vec3d_text(to_vec3d(camera.position)));
    lines.push_back("Velocity m/s:      " + vec3d_text(camera_motion.velocity));
    lines.push_back("Acceleration m/s2: " + vec3d_text(camera_motion.acceleration));
    {
        std::ostringstream stream;
        constexpr float kRadToDeg = 57.29577951308232F;
        stream << std::fixed << std::setprecision(2)
               << "Yaw/Pitch deg: " << camera.yaw * kRadToDeg << " / " << camera.pitch * kRadToDeg
               << " | forward=" << vec3f_text(camera.forward());
        lines.push_back(stream.str());
    }

    lines.push_back("");
    lines.push_back("[WORLD] objects=" + std::to_string(world.object_count()));
    world.for_each_object([&](const ocs::world::ObjectHandle handle, const ocs::world::WorldObject& object) {
        const bool is_physics_object = handle == movable_object;
        const ocs::physics::RigidBody* body =
            is_physics_object && physics_snapshot.has_monitored_body ? &physics_snapshot.body : nullptr;
        const ocs::math::Vec3f euler = quaternion_euler_degrees(object.transform.rotation);

        lines.push_back("-- object " + std::to_string(handle.index) + ':' + std::to_string(handle.generation) +
                        "  " + object.name + " --");
        lines.push_back(" model=" + std::to_string(object.model.index) + ':' + std::to_string(object.model.generation) +
                        " visible=" + (object.visible ? "yes" : "no") +
                        " movable=" + (object.movable ? "yes" : "no") +
                        " physics=" + (body != nullptr ? "yes" : "no"));
        lines.push_back(" pos m:   " + vec3d_text(to_vec3d(object.transform.position)));
        lines.push_back(" rot deg: " + vec3f_text(euler, 2) +
                        " | scale=" + vec3f_text(object.transform.scale, 2));
        if (body != nullptr) {
            lines.push_back(" vel m/s: " + vec3d_text(body->current.linear_velocity));
            lines.push_back(" acc m/s2:" + vec3d_text(body->current.linear_acceleration));
            lines.push_back(" omega rad/s: " + vec3f_text(body->current.angular_velocity));
            lines.push_back(" alpha rad/s2:" + vec3f_text(body->current.angular_acceleration));
            std::ostringstream body_line;
            body_line << std::fixed << std::setprecision(2)
                      << " mass=" << body->mass.mass << " kg"
                      << " grounded=" << (body->grounded ? "yes" : "no")
                      << " contacts=" << body->contact_count
                      << " debugContacts=" << body->debug_contact_count
                      << " force=" << vec3d_text(body->last_external_force)
                      << " torque=" << vec3f_text(body->last_external_torque);
            lines.push_back(body_line.str());
            lines.push_back(" support normal: " + vec3d_text(body->support_normal) +
                            " | last contact: " + vec3d_text(body->last_contact_point));
            lines.push_back(" collision box center: " + vec3f_text(body->collision_box.center_local) +
                            " half: " + vec3f_text(body->collision_box.half_extents));
        } else {
            lines.push_back(" vel/acc/angular: static (0)");
        }
    });

    return lines;
}

} // namespace

int main(const int argc, char** argv) {
    const AppConfig config = parse_config(argc, argv);

    ocs::platform::Platform platform;
    if (!platform.initialize()) return 1;

    ocs::platform::Window window;
    if (!window.create({
            .title = "OpenCarreraSimulator - Step 9.5 Longitudinal Tire Prototype",
            .width = 1280,
            .height = 720,
            .resizable = true,
            .high_pixel_density = true,
            .graphics_api = ocs::platform::WindowGraphicsApi::vulkan})) {
        platform.shutdown();
        return 1;
    }

    ocs::render::Renderer renderer;
    if (!renderer.initialize(window, config.renderer)) {
        window.destroy();
        platform.shutdown();
        return 1;
    }

    const auto vehicle_model = renderer.default_model();
    const auto wheel_model = renderer.load_model(config.wheel_path.string());
    const auto track_model = renderer.load_model(config.track_path.string());
    if (!vehicle_model || !wheel_model || !track_model) {
        OCS_LOG_ERROR("Step 9.5 could not load the vehicle body, wheel or Blender test circuit");
        renderer.shutdown();
        window.destroy();
        platform.shutdown();
        return 1;
    }

    const auto collision_mesh_result = ocs::assets::load_mesh(config.track_collision_path);
    if (!collision_mesh_result) {
        OCS_LOG_ERROR("Step 9.5 could not load static collision mesh: " +
                      collision_mesh_result.error().message);
        renderer.shutdown();
        window.destroy();
        platform.shutdown();
        return 1;
    }
    const ocs::assets::MeshData& collision_mesh = *collision_mesh_result;
    const TrackSpawn track_spawn = choose_track_spawn(collision_mesh);

    const auto bounds = renderer.model_bounds(vehicle_model);
    const auto wheel_bounds = renderer.model_bounds(wheel_model);
    constexpr float kVehicleScale = 1.0F;
    const ocs::math::Vec3f visual_center_local = visual_center_from_bounds(bounds, kVehicleScale);
    const ocs::math::Vec3f wheel_visual_center = visual_center_from_bounds(wheel_bounds, 1.0F);

    // Step 9.2/9.3: the OBB returns to the actual body shell. Normal ride-height
    // support now comes from four spring/damper suspension stations; the OBB remains
    // as chassis collision/bottom-out protection for walls, hard landings and rollover.
    constexpr float kWheelRadius = 0.34F;
    constexpr float kRideClearance = 0.22F;
    constexpr double kSuspensionRestLength = 0.30;
    constexpr double kStaticSuspensionLength = 0.22;
    constexpr double kSuspensionCompressionTravel = 0.14;
    constexpr double kSuspensionDroopTravel = 0.12;
    constexpr double kSpringRate = 36000.0;
    constexpr double kDamperRate = 4500.0;
    ocs::physics::BoxCollisionShape collision_box = collision_box_from_bounds(bounds, kVehicleScale);
    const float static_wheel_center_z =
        collision_box.center_local.z - collision_box.half_extents.z - kRideClearance + kWheelRadius;
    const float suspension_mount_z = static_wheel_center_z + static_cast<float>(kStaticSuspensionLength);

    ocs::vehicle::VehicleConfig vehicle_config{};
    vehicle_config.drive_layout = ocs::vehicle::DriveLayout::rear_wheel_drive;
    vehicle_config.drive_acceleration = 9.0; // legacy compatibility helper only
    vehicle_config.maximum_drive_torque_nm = 2200.0;
    vehicle_config.maximum_brake_torque_nm_per_wheel = 1200.0;
    vehicle_config.longitudinal_slip_reference_speed_mps = 2.0;
    vehicle_config.maximum_steer_angle_radians = 0.42;
    vehicle_config.provisional_max_lateral_acceleration = 8.5;
    vehicle_config.wheels = {{
        {.suspension_mount_local = { 1.36F,  0.79F, suspension_mount_z}, .radius = kWheelRadius, .rest_length = kSuspensionRestLength, .max_compression = kSuspensionCompressionTravel, .max_droop = kSuspensionDroopTravel, .spring_rate = kSpringRate, .damper_rate = kDamperRate, .steerable = true},
        {.suspension_mount_local = { 1.36F, -0.79F, suspension_mount_z}, .radius = kWheelRadius, .rest_length = kSuspensionRestLength, .max_compression = kSuspensionCompressionTravel, .max_droop = kSuspensionDroopTravel, .spring_rate = kSpringRate, .damper_rate = kDamperRate, .steerable = true},
        {.suspension_mount_local = {-1.33F,  0.79F, suspension_mount_z}, .radius = kWheelRadius, .rest_length = kSuspensionRestLength, .max_compression = kSuspensionCompressionTravel, .max_droop = kSuspensionDroopTravel, .spring_rate = kSpringRate, .damper_rate = kDamperRate, .steerable = false},
        {.suspension_mount_local = {-1.33F, -0.79F, suspension_mount_z}, .radius = kWheelRadius, .rest_length = kSuspensionRestLength, .max_compression = kSuspensionCompressionTravel, .max_droop = kSuspensionDroopTravel, .spring_rate = kSpringRate, .damper_rate = kDamperRate, .steerable = false}
    }};

    const double support_height = static_cast<double>(
        collision_box.half_extents.z - collision_box.center_local.z) + kRideClearance;
    const ocs::math::Vec3d initial_body_position = track_spawn.surface_point +
        ocs::math::Vec3d{0.0, 0.0, 0.05 + support_height};
    const float initial_yaw = static_cast<float>(std::atan2(track_spawn.forward.y, track_spawn.forward.x));
    const ocs::math::Quatf initial_body_orientation = ocs::math::Quatf::from_axis_angle(
        {0.0F, 0.0F, 1.0F}, initial_yaw);
    const ocs::world::Transform initial_transform = visual_transform_from_body(
        initial_body_position, initial_body_orientation, visual_center_local, kVehicleScale);

    ocs::world::World world;
    const auto track_object = world.spawn("test_circuit", track_model, {}, false);
    const auto movable_object = world.spawn("vehicle_lab_gt", vehicle_model, initial_transform, true);
    VisualWheelRig visual_wheels{};
    visual_wheels.objects = {{
        world.spawn("wheel_FL", wheel_model, {}, false),
        world.spawn("wheel_FR", wheel_model, {}, false),
        world.spawn("wheel_RL", wheel_model, {}, false),
        world.spawn("wheel_RR", wheel_model, {}, false)
    }};
    const bool wheels_valid = std::all_of(
        visual_wheels.objects.begin(), visual_wheels.objects.end(),
        [](const ocs::world::ObjectHandle handle) { return static_cast<bool>(handle); });
    if (!track_object || !movable_object || !wheels_valid) {
        OCS_LOG_ERROR("Step 9.5 failed to construct the laboratory vehicle world");
        renderer.shutdown();
        window.destroy();
        platform.shutdown();
        return 1;
    }

    PhysicsRuntime physics_runtime(!config.single_thread_physics, config.physics_hz);
    ocs::physics::PhysicsWorld& physics_setup = physics_runtime.setup_world();
    physics_setup.set_static_triangles(collision_triangles(collision_mesh));

    ocs::physics::RigidBodyDesc body_desc{};
    body_desc.state.position = initial_body_position;
    body_desc.state.orientation = initial_body_orientation;
    body_desc.mass = 1200.0;
    body_desc.inertia_diagonal = box_inertia_diagonal(body_desc.mass, collision_box.half_extents);
    body_desc.dynamic = true;
    body_desc.ground_contact_enabled = false;
    body_desc.static_mesh_contact_enabled = true;
    body_desc.collision_box = collision_box;
    body_desc.restitution = 0.04;
    body_desc.friction_coefficient = 0.70;
    body_desc.linear_damping = 0.015;
    body_desc.angular_damping = 0.12;
    const auto movable_body = physics_setup.create_body(body_desc);
    if (!movable_body) {
        OCS_LOG_ERROR("Step 9.5 failed to create the laboratory vehicle rigid body");
        renderer.shutdown();
        window.destroy();
        platform.shutdown();
        return 1;
    }
    const std::size_t static_triangle_count = physics_setup.static_triangle_count();
    const ocs::physics::RigidBodyState initial_body_state = body_desc.state;
    ocs::vehicle::VehicleContactState initial_wheel_contacts{};
    if (const ocs::physics::RigidBody* body = physics_setup.get(movable_body); body != nullptr) {
        initial_wheel_contacts = ocs::vehicle::sample_wheel_contacts(physics_setup, *body, vehicle_config);
    }
    update_visual_wheels(
        world, visual_wheels, vehicle_config, initial_wheel_contacts, initial_body_state,
        wheel_visual_center, 0.0F);
    physics_runtime.set_monitored_body(movable_body);

    DriveTelemetryMailbox drive_mailbox;
    std::atomic<std::uint64_t> vehicle_state_epoch{0U};
    if (!physics_runtime.start([movable_body, &drive_mailbox, &vehicle_state_epoch, vehicle_config,
                                vehicle_runtime = ocs::vehicle::VehicleRuntimeState{},
                                observed_vehicle_state_epoch = std::uint64_t{0}](
            ocs::physics::PhysicsWorld& physics,
            const double dt,
            const ocs::physics::PhysicsThreadInput& input) mutable {
            DriveFrameTelemetry telemetry{};
            telemetry.drive_layout = vehicle_config.drive_layout;
            const std::uint64_t requested_epoch = vehicle_state_epoch.load(std::memory_order_acquire);
            if (requested_epoch != observed_vehicle_state_epoch) {
                vehicle_runtime = ocs::vehicle::VehicleRuntimeState{};
                observed_vehicle_state_epoch = requested_epoch;
            }
            if (ocs::physics::RigidBody* body = physics.get(movable_body); body != nullptr) {
                body->dynamic = !input.object_manipulation_enabled;
                if (input.object_manipulation_enabled) {
                    vehicle_runtime.initialized = false;
                    (void)physics.set_pose(
                        movable_body,
                        input.object_target_position,
                        input.object_target_orientation,
                        true);
                } else {
                    telemetry.wheel_contacts = ocs::vehicle::update_suspension_contacts(
                        physics, *body, vehicle_config, vehicle_runtime, dt);
                    telemetry.applied_suspension_force_n = ocs::vehicle::apply_suspension_forces(
                        physics, movable_body, vehicle_config, vehicle_runtime);
                }
            }
            if (!input.object_manipulation_enabled) {
                apply_drive_lab_control(
                    physics, movable_body, vehicle_config, vehicle_runtime, input, dt, telemetry);
                telemetry.wheel_contacts = vehicle_runtime.contacts;
            }
            if (input.apply_test_force) {
                physics.add_force(movable_body, {0.0, 18000.0, 0.0});
            }
            if (input.apply_test_torque) {
                physics.add_torque(movable_body, {0.0F, 0.0F, 4500.0F});
            }
            drive_mailbox.publish(telemetry);
        })) {
        OCS_LOG_ERROR("Step 9.5 failed to start physics execution path");
        renderer.shutdown();
        window.destroy();
        platform.shutdown();
        return 1;
    }

    MetricsWriter metrics;
    if (!metrics.initialize(config.metrics_file)) {
        physics_runtime.stop();
        renderer.shutdown();
        window.destroy();
        platform.shutdown();
        return 1;
    }

    ocs::platform::DebugTextWindow debug_window;
    if (config.metrics_file.has_value() && !debug_window.create("OpenCarreraSimulator - Debug Telemetry")) {
        OCS_LOG_WARN("Metrics CSV remains enabled, but the debug telemetry window could not be created");
    }

    OCS_LOG_INFO("OpenCarreraSimulator Step 9.5 running | wheel inertia + slip-ratio longitudinal tires + suspension + RWD | world objects " + std::to_string(world.object_count()));
    OCS_LOG_INFO("Static track collision: " + std::to_string(static_triangle_count) + " triangles");
    OCS_LOG_INFO("Vehicle: OCS Lab GT | RWD | wheel angular inertia | load-limited longitudinal slip tires | spring/damper support | provisional bicycle yaw helper");
    OCS_LOG_INFO("Controls: C CAMERA/OBJECT/DRIVE | DRIVE camera: CHASE | V 3D physics debug | DRIVE: W throttle, S brake, A/D steer | SPACE pause | N step | R reset");
    OCS_LOG_INFO("Physics target: " + std::to_string(config.physics_hz) + " Hz fixed step | execution: " +
                 std::string(physics_runtime.threaded()
                     ? "DEDICATED THREAD"
                     : "SINGLE THREAD (explicit --single-thread-physics)"));
    if (!physics_runtime.threaded()) {
        OCS_LOG_WARN("Single-thread physics was explicitly requested. Physics work runs in main-thread bursts and may affect frame pacing; omit --single-thread-physics (or pass --dedicated-physics) for the default dedicated thread.");
    }

    FreeCamera camera{};
    const ocs::math::Vec3d spawn_left{-track_spawn.forward.y, track_spawn.forward.x, 0.0};
    const ocs::math::Vec3d camera_position = track_spawn.surface_point - track_spawn.forward * 18.0 -
        spawn_left * 9.0 + ocs::math::Vec3d{0.0, 0.0, 11.0};
    camera.position = to_world_position(camera_position);
    camera.yaw = static_cast<float>(std::atan2(
        track_spawn.surface_point.y - camera.position.y,
        track_spawn.surface_point.x - camera.position.x));
    camera.pitch = -0.30F;
    MotionTracker camera_motion{};
    ChaseCameraState chase_camera{};
    ControlMode control_mode = ControlMode::camera;
    ObjectManipulationTarget object_target{};
    bool physics_paused = false;
    bool physics_debug_enabled = true;
    bool relative_mouse_enabled = false;
    PhysicsFrameTelemetry physics_frame{};
    DriveFrameTelemetry drive_frame{};
    auto previous_frame = std::chrono::steady_clock::now();
    auto last_stats_log = previous_frame;
    auto last_debug_refresh = previous_frame - std::chrono::seconds(1);
    auto physics_snapshot = physics_runtime.snapshot(previous_frame);
    std::uint64_t previous_physics_total_steps = physics_snapshot.total_steps;

    bool running = true;
    while (running) {
        const auto frame_begin = std::chrono::steady_clock::now();
        const double delta_seconds = std::clamp(
            std::chrono::duration<double>(frame_begin - previous_frame).count(), 0.0, 0.1);
        previous_frame = frame_begin;

        const ocs::platform::PlatformEvents events = platform.poll_events();
        if (events.quit_requested || events.close_requested_window_id == window.id()) {
            running = false;
        }
        if (debug_window.valid() && events.close_requested_window_id == debug_window.id()) {
            debug_window.destroy();
        }
        if (events.framebuffer_resized_window_id == window.id()) {
            renderer.notify_framebuffer_resized();
        }

        const bool main_keyboard_focus = events.keyboard_focus_window_id == 0U || events.keyboard_focus_window_id == window.id();
        const bool main_mouse_focus = events.mouse_focus_window_id == 0U || events.mouse_focus_window_id == window.id();
        const bool rotate_active = events.rotate_active && main_mouse_focus;
        if (rotate_active != relative_mouse_enabled) {
            if (window.set_relative_mouse_mode(rotate_active)) {
                relative_mouse_enabled = rotate_active;
            }
        }

        ocs::platform::PlatformEvents control_events = events;
        control_events.rotate_active = rotate_active;
        if (!main_keyboard_focus) {
            control_events.move_forward = false;
            control_events.move_backward = false;
            control_events.move_left = false;
            control_events.move_right = false;
            control_events.move_down = false;
            control_events.move_up = false;
            control_events.speed_boost = false;
            control_events.toggle_control_mode = false;
            control_events.toggle_physics_pause = false;
            control_events.single_step_physics = false;
            control_events.reset_physics = false;
            control_events.toggle_physics_debug = false;
            control_events.apply_test_force = false;
            control_events.apply_test_torque = false;
        }

        if (control_events.toggle_control_mode) {
            control_mode = next_control_mode(control_mode);
            if (control_mode == ControlMode::object && physics_snapshot.has_monitored_body) {
                vehicle_state_epoch.fetch_add(1U, std::memory_order_release);
                object_target.initialized = true;
                object_target.position = physics_snapshot.body.current.position;
                object_target.orientation = physics_snapshot.body.current.orientation;
            }
            if (control_mode == ControlMode::drive) {
                chase_camera.initialized = false;
            }
            OCS_LOG_INFO(std::string("Control mode: ") + control_mode_name(control_mode) +
                         (control_mode == ControlMode::object ? " (vehicle_lab_gt, KINEMATIC)" :
                          control_mode == ControlMode::drive ? " (CHASE CAMERA)" : ""));
        }
        if (control_events.toggle_physics_pause) {
            physics_paused = !physics_paused;
            physics_runtime.set_paused(physics_paused);
            OCS_LOG_INFO(physics_paused ? "Physics: PAUSED" : "Physics: RUNNING");
        }
        if (control_events.toggle_physics_debug) {
            physics_debug_enabled = !physics_debug_enabled;
            OCS_LOG_INFO(physics_debug_enabled ? "3D physics debug: ON" : "3D physics debug: OFF");
        }
        if (control_events.reset_physics) {
            vehicle_state_epoch.fetch_add(1U, std::memory_order_release);
            physics_runtime.reset(initial_body_state);
            object_target.initialized = false;
            chase_camera.initialized = false;
            OCS_LOG_INFO("Vehicle laboratory body reset");
        }
        if (control_events.single_step_physics && physics_paused) {
            physics_runtime.request_single_step();
        }

        if (control_mode == ControlMode::camera) {
            update_camera(camera, control_events, delta_seconds);
        } else if (control_mode == ControlMode::object && physics_snapshot.has_monitored_body) {
            if (!object_target.initialized) {
                object_target.initialized = true;
                object_target.position = physics_snapshot.body.current.position;
                object_target.orientation = physics_snapshot.body.current.orientation;
            }
            ocs::physics::RigidBody target_body = physics_snapshot.body;
            target_body.current.position = object_target.position;
            target_body.current.orientation = object_target.orientation;
            ocs::math::Vec3d requested_position{};
            ocs::math::Quatf requested_rotation{};
            if (compute_physics_object_pose(
                    target_body,
                    control_events,
                    delta_seconds,
                    requested_position,
                    requested_rotation)) {
                object_target.position = requested_position;
                object_target.orientation = requested_rotation;
            }
        }

        ocs::physics::PhysicsThreadInput physics_input{};
        physics_input.drive_enabled = control_mode == ControlMode::drive;
        physics_input.move_forward = control_events.move_forward;
        physics_input.move_backward = control_events.move_backward;
        physics_input.move_left = control_events.move_left;
        physics_input.move_right = control_events.move_right;
        physics_input.speed_boost = control_events.speed_boost;
        physics_input.apply_test_force = control_events.apply_test_force;
        physics_input.apply_test_torque = control_events.apply_test_torque;
        physics_input.object_manipulation_enabled =
            control_mode == ControlMode::object && object_target.initialized;
        physics_input.object_target_position = object_target.position;
        physics_input.object_target_orientation = object_target.orientation;
        physics_runtime.set_input(physics_input);

        // In dedicated-thread mode this is intentionally a no-op. In fallback mode
        // it executes the exact same fixed-step callback on the main thread.
        physics_runtime.update(delta_seconds);

        const auto physics_now = std::chrono::steady_clock::now();
        physics_snapshot = physics_runtime.snapshot(physics_now);
        const double render_alpha = physics_runtime.interpolation_alpha(physics_snapshot, physics_now);
        const std::uint64_t step_delta_u64 = physics_snapshot.total_steps >= previous_physics_total_steps
            ? physics_snapshot.total_steps - previous_physics_total_steps
            : 0U;
        previous_physics_total_steps = physics_snapshot.total_steps;
        const std::uint32_t frame_physics_steps = static_cast<std::uint32_t>(
            std::min<std::uint64_t>(step_delta_u64, std::numeric_limits<std::uint32_t>::max()));
        const double snapshot_age_ms = physics_snapshot.published_at.time_since_epoch().count() != 0
            ? std::max(0.0, std::chrono::duration<double, std::milli>(physics_now - physics_snapshot.published_at).count())
            : 0.0;
        const double dropped_time_ms = physics_runtime.threaded()
            ? static_cast<double>(physics_snapshot.dropped_ticks) * physics_snapshot.fixed_dt_seconds * 1000.0
            : physics_runtime.inline_dropped_ms();
        const double physics_work_ms = physics_runtime.threaded()
            ? physics_snapshot.average_step_us * static_cast<double>(frame_physics_steps) / 1000.0
            : physics_runtime.inline_cpu_ms();

        physics_frame = {
            .threaded = physics_runtime.threaded(),
            .paused = physics_snapshot.paused,
            .frequency_hz = physics_snapshot.target_hz,
            .actual_hz = physics_snapshot.actual_hz,
            .fixed_dt_ms = physics_snapshot.fixed_dt_seconds * 1000.0,
            .steps = frame_physics_steps,
            .alpha = render_alpha,
            .accumulator_ms = physics_runtime.threaded() ? 0.0 : physics_runtime.inline_accumulator_ms(),
            .simulation_time_s = physics_snapshot.simulation_time_seconds,
            .dropped_time_ms = dropped_time_ms,
            .cpu_ms = physics_work_ms,
            .step_us = physics_snapshot.average_step_us,
            .peak_step_us = physics_snapshot.peak_step_us,
            .scheduler_lag_us = physics_snapshot.scheduler_lag_us,
            .utilization_percent = physics_snapshot.utilization_percent,
            .snapshot_age_ms = snapshot_age_ms,
            .total_steps = physics_snapshot.total_steps,
            .catchup_steps = physics_snapshot.catchup_steps,
            .dropped_ticks = physics_snapshot.dropped_ticks,
            .deadline_misses = physics_snapshot.deadline_misses,
            .bodies = physics_snapshot.body_count,
            .static_triangles = physics_snapshot.static_triangle_count,
            .triangle_tests = physics_snapshot.step_stats.triangle_tests,
            .contacts = physics_snapshot.step_stats.contacts
        };
        drive_frame = drive_mailbox.read();

        auto render_state = physics_runtime.render_state(physics_snapshot, render_alpha);
        if (control_mode == ControlMode::object && object_target.initialized) {
            render_state.position = object_target.position;
            render_state.orientation = object_target.orientation;
            render_state.linear_velocity = {};
            render_state.linear_acceleration = {};
            render_state.angular_velocity = {};
            render_state.angular_acceleration = {};
        }
        if (ocs::world::WorldObject* object = world.get(movable_object); object != nullptr) {
            object->transform = visual_transform_from_body(
                render_state.position, render_state.orientation, visual_center_local, kVehicleScale);
        }
        update_visual_wheels(
            world, visual_wheels, vehicle_config, drive_frame.wheel_contacts, render_state,
            wheel_visual_center, control_mode == ControlMode::drive ? drive_frame.steering : 0.0F);

        if (control_mode == ControlMode::drive) {
            update_drive_chase_camera(camera, chase_camera, render_state, delta_seconds);
        }
        camera_motion.update(to_vec3d(camera.position), delta_seconds);

        ocs::render::RenderScene scene = world.extract_render_scene(camera.position);
        if (physics_debug_enabled && physics_snapshot.has_monitored_body) {
            append_physics_debug_draw(scene, physics_snapshot.body, render_state, camera.position);
            append_wheel_contact_debug_draw(scene, drive_frame.wheel_contacts, camera.position);
        }
        if (running && !renderer.draw_frame(scene, camera.render_view())) {
            OCS_LOG_ERROR("Renderer failed while drawing a world frame");
            running = false;
            continue;
        }

        const auto& stats = renderer.stats();
        metrics.write(stats, physics_frame);

        const auto now = std::chrono::steady_clock::now();
        if (debug_window.valid() && now - last_debug_refresh >= std::chrono::milliseconds(100)) {
            const auto lines = build_debug_lines(renderer, world, physics_snapshot, movable_object,
                                                 camera, camera_motion, control_mode, physics_frame, drive_frame,
                                                 physics_debug_enabled);
            (void)debug_window.render_lines(lines);
            last_debug_refresh = now;
        }

        if (config.log_stats && now - last_stats_log >= std::chrono::seconds(1)) {
            OCS_LOG_INFO(stats_line(stats, physics_frame));
            last_stats_log = now;
        }
    }

    physics_runtime.stop();
    debug_window.destroy();
    renderer.shutdown();
    window.destroy();
    platform.shutdown();
    return 0;
}
