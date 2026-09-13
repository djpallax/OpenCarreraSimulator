#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

#include "ocs/physics/physics_world.hpp"

namespace ocs::physics {

struct PhysicsThreadConfig {
    double frequency_hz = 500.0;
    std::uint32_t max_catchup_steps = 4;
    double max_lag_seconds = 0.050;
    bool start_paused = false;
};

struct PhysicsThreadInput {
    bool drive_enabled = false;
    bool move_forward = false;
    bool move_backward = false;
    bool move_left = false;
    bool move_right = false;
    bool speed_boost = false;
    bool apply_test_force = false;
    bool apply_test_torque = false;

    // Editor/debug object manipulation. The latest target is consumed by the
    // physics owner thread; render may preview the same target immediately.
    bool object_manipulation_enabled = false;
    math::Vec3d object_target_position{};
    math::Quatf object_target_orientation = math::Quatf::identity();
};

struct PhysicsThreadSnapshot {
    using Clock = std::chrono::steady_clock;

    std::uint64_t sequence = 0;
    bool thread_running = false;
    bool paused = false;
    double target_hz = 500.0;
    double actual_hz = 0.0;
    double fixed_dt_seconds = 0.002;
    double simulation_time_seconds = 0.0;
    std::uint64_t total_steps = 0;
    std::uint64_t catchup_steps = 0;
    std::uint64_t dropped_ticks = 0;
    std::uint64_t deadline_misses = 0;
    double last_step_us = 0.0;
    double average_step_us = 0.0;
    double peak_step_us = 0.0;
    double scheduler_lag_us = 0.0;
    double utilization_percent = 0.0;
    PhysicsStepStats step_stats{};
    std::size_t body_count = 0;
    std::size_t static_triangle_count = 0;
    math::Vec3d gravity{0.0, 0.0, -9.80665};
    RigidBodyHandle monitored_body{};
    bool has_monitored_body = false;
    RigidBody body{};
    Clock::time_point published_at{};
};

class PhysicsThread {
public:
    using StepCallback = std::function<void(PhysicsWorld&, double, const PhysicsThreadInput&)>;

    explicit PhysicsThread(PhysicsThreadConfig config = {}) noexcept;
    ~PhysicsThread();

    PhysicsThread(const PhysicsThread&) = delete;
    PhysicsThread& operator=(const PhysicsThread&) = delete;
    PhysicsThread(PhysicsThread&&) = delete;
    PhysicsThread& operator=(PhysicsThread&&) = delete;

    // Setup-only access. Call before start(); after start(), the physics thread
    // exclusively owns PhysicsWorld until stop() has joined it.
    [[nodiscard]] PhysicsWorld& setup_world() noexcept { return world_; }
    void set_monitored_body(RigidBodyHandle handle) noexcept { monitored_body_ = handle; }

    bool start(StepCallback callback = {});
    void stop() noexcept;
    [[nodiscard]] bool running() const noexcept { return running_.load(std::memory_order_acquire); }

    void set_input(const PhysicsThreadInput& input) noexcept;
    void request_pause(bool paused);
    void request_single_step();
    void request_reset(RigidBodyHandle handle, const RigidBodyState& state);
    void request_pose(RigidBodyHandle handle,
                      math::Vec3d position,
                      math::Quatf orientation,
                      bool reset_motion = true);

    [[nodiscard]] PhysicsThreadSnapshot snapshot() const noexcept;

private:
    enum class CommandType {
        pause,
        single_step,
        reset,
        set_pose
    };

    struct Command {
        CommandType type = CommandType::pause;
        RigidBodyHandle handle{};
        RigidBodyState state{};
        math::Vec3d position{};
        math::Quatf orientation = math::Quatf::identity();
        bool flag = false;
    };

    static PhysicsThreadConfig sanitize(PhysicsThreadConfig config) noexcept;
    void run(std::stop_token stop_token) noexcept;
    bool process_commands() noexcept;
    void execute_step(double fixed_dt_seconds) noexcept;
    void publish_snapshot(PhysicsThreadSnapshot::Clock::time_point now) noexcept;
    [[nodiscard]] PhysicsThreadInput input_snapshot() const noexcept;
    void queue_command(Command command);

    PhysicsThreadConfig config_{};
    PhysicsWorld world_{};
    RigidBodyHandle monitored_body_{};
    StepCallback step_callback_{};

    mutable std::mutex input_mutex_{};
    PhysicsThreadInput input_{};

    mutable std::mutex command_mutex_{};
    std::condition_variable command_cv_{};
    std::deque<Command> commands_{};

    mutable std::mutex snapshot_mutex_{};
    PhysicsThreadSnapshot published_{};

    std::jthread thread_{};
    std::atomic<bool> running_{false};
    bool paused_ = false;
    std::uint32_t pending_single_steps_ = 0;
    bool resync_schedule_ = false;

    double simulation_time_seconds_ = 0.0;
    std::uint64_t total_steps_ = 0;
    std::uint64_t catchup_steps_ = 0;
    std::uint64_t dropped_ticks_ = 0;
    std::uint64_t deadline_misses_ = 0;
    double last_step_us_ = 0.0;
    double average_step_us_ = 0.0;
    double peak_step_us_ = 0.0;
    double scheduler_lag_us_ = 0.0;
    double measured_hz_ = 0.0;
    std::uint64_t measured_window_steps_ = 0;
    PhysicsThreadSnapshot::Clock::time_point measured_window_start_{};
};

[[nodiscard]] double physics_snapshot_interpolation_alpha(
    const PhysicsThreadSnapshot& snapshot,
    PhysicsThreadSnapshot::Clock::time_point now = PhysicsThreadSnapshot::Clock::now()) noexcept;

[[nodiscard]] RigidBodyState interpolated_snapshot_state(
    const PhysicsThreadSnapshot& snapshot,
    double alpha) noexcept;

} // namespace ocs::physics
