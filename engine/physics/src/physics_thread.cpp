#include "ocs/physics/physics_thread.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ocs::physics {
namespace {

using Clock = PhysicsThreadSnapshot::Clock;

[[nodiscard]] math::Vec3d lerp(const math::Vec3d a, const math::Vec3d b, const double alpha) noexcept {
    return a + (b - a) * alpha;
}

[[nodiscard]] math::Vec3f lerp(const math::Vec3f a, const math::Vec3f b, const float alpha) noexcept {
    return a + (b - a) * alpha;
}

} // namespace

PhysicsThreadConfig PhysicsThread::sanitize(PhysicsThreadConfig config) noexcept {
    config.frequency_hz = std::clamp(config.frequency_hz, 1.0, 5000.0);
    config.max_catchup_steps = std::clamp<std::uint32_t>(config.max_catchup_steps, 1U, 64U);
    config.max_lag_seconds = std::clamp(config.max_lag_seconds, 0.001, 1.0);
    return config;
}

PhysicsThread::PhysicsThread(PhysicsThreadConfig config) noexcept
    : config_(sanitize(config)) {
    published_.target_hz = config_.frequency_hz;
    published_.fixed_dt_seconds = 1.0 / config_.frequency_hz;
}

PhysicsThread::~PhysicsThread() {
    stop();
}

bool PhysicsThread::start(StepCallback callback) {
    if (running_.load(std::memory_order_acquire) || thread_.joinable()) {
        return false;
    }

    step_callback_ = std::move(callback);
    paused_ = config_.start_paused;
    pending_single_steps_ = 0;
    resync_schedule_ = false;
    simulation_time_seconds_ = 0.0;
    total_steps_ = 0;
    catchup_steps_ = 0;
    dropped_ticks_ = 0;
    deadline_misses_ = 0;
    last_step_us_ = 0.0;
    average_step_us_ = 0.0;
    peak_step_us_ = 0.0;
    scheduler_lag_us_ = 0.0;
    measured_hz_ = 0.0;
    measured_window_steps_ = 0;
    measured_window_start_ = Clock::now();
    running_.store(true, std::memory_order_release);
    publish_snapshot(measured_window_start_);
    thread_ = std::jthread([this](const std::stop_token token) { run(token); });
    return true;
}

void PhysicsThread::stop() noexcept {
    if (!thread_.joinable()) {
        running_.store(false, std::memory_order_release);
        return;
    }
    thread_.request_stop();
    command_cv_.notify_all();
    thread_.join();
    running_.store(false, std::memory_order_release);
    publish_snapshot(Clock::now());
}

void PhysicsThread::set_input(const PhysicsThreadInput& input) noexcept {
    const std::scoped_lock lock(input_mutex_);
    input_ = input;
}

PhysicsThreadInput PhysicsThread::input_snapshot() const noexcept {
    const std::scoped_lock lock(input_mutex_);
    return input_;
}

void PhysicsThread::queue_command(Command command) {
    {
        const std::scoped_lock lock(command_mutex_);
        commands_.push_back(std::move(command));
    }
    command_cv_.notify_all();
}

void PhysicsThread::request_pause(const bool paused) {
    Command command{};
    command.type = CommandType::pause;
    command.flag = paused;
    queue_command(command);
}

void PhysicsThread::request_single_step() {
    Command command{};
    command.type = CommandType::single_step;
    queue_command(command);
}

void PhysicsThread::request_reset(const RigidBodyHandle handle, const RigidBodyState& state) {
    Command command{};
    command.type = CommandType::reset;
    command.handle = handle;
    command.state = state;
    queue_command(command);
}

void PhysicsThread::request_pose(const RigidBodyHandle handle,
                                 const math::Vec3d position,
                                 const math::Quatf orientation,
                                 const bool reset_motion) {
    Command command{};
    command.type = CommandType::set_pose;
    command.handle = handle;
    command.position = position;
    command.orientation = orientation;
    command.flag = reset_motion;
    queue_command(command);
}

bool PhysicsThread::process_commands() noexcept {
    std::deque<Command> local;
    {
        const std::scoped_lock lock(command_mutex_);
        local.swap(commands_);
    }

    bool changed = false;
    for (const Command& command : local) {
        switch (command.type) {
        case CommandType::pause:
            if (paused_ != command.flag) {
                paused_ = command.flag;
                resync_schedule_ = true;
                changed = true;
            }
            break;
        case CommandType::single_step:
            if (paused_) {
                pending_single_steps_ = std::min<std::uint32_t>(pending_single_steps_ + 1U, 64U);
                changed = true;
            }
            break;
        case CommandType::reset:
            if (world_.reset_state(command.handle, command.state)) {
                simulation_time_seconds_ = 0.0;
                total_steps_ = 0;
                catchup_steps_ = 0;
                dropped_ticks_ = 0;
                deadline_misses_ = 0;
                last_step_us_ = 0.0;
                average_step_us_ = 0.0;
                peak_step_us_ = 0.0;
                scheduler_lag_us_ = 0.0;
                measured_hz_ = 0.0;
                measured_window_steps_ = 0;
                measured_window_start_ = Clock::now();
                changed = true;
            }
            resync_schedule_ = true;
            break;
        case CommandType::set_pose:
            changed = world_.set_pose(
                command.handle,
                command.position,
                command.orientation,
                command.flag) || changed;
            resync_schedule_ = true;
            break;
        }
    }
    return changed;
}

void PhysicsThread::execute_step(const double fixed_dt_seconds) noexcept {
    const PhysicsThreadInput controls = input_snapshot();
    const auto begin = Clock::now();
    if (step_callback_) {
        step_callback_(world_, fixed_dt_seconds, controls);
    }
    world_.step(fixed_dt_seconds);
    const auto end = Clock::now();

    last_step_us_ = std::chrono::duration<double, std::micro>(end - begin).count();
    average_step_us_ = total_steps_ == 0U
        ? last_step_us_
        : average_step_us_ * 0.95 + last_step_us_ * 0.05;
    peak_step_us_ = std::max(peak_step_us_, last_step_us_);
    simulation_time_seconds_ += fixed_dt_seconds;
    ++total_steps_;
    ++measured_window_steps_;
}

void PhysicsThread::publish_snapshot(const Clock::time_point now) noexcept {
    PhysicsThreadSnapshot snapshot{};
    snapshot.sequence = total_steps_ + 1U;
    snapshot.thread_running = running_.load(std::memory_order_acquire);
    snapshot.paused = paused_;
    snapshot.target_hz = config_.frequency_hz;
    snapshot.actual_hz = measured_hz_;
    snapshot.fixed_dt_seconds = 1.0 / config_.frequency_hz;
    snapshot.simulation_time_seconds = simulation_time_seconds_;
    snapshot.total_steps = total_steps_;
    snapshot.catchup_steps = catchup_steps_;
    snapshot.dropped_ticks = dropped_ticks_;
    snapshot.deadline_misses = deadline_misses_;
    snapshot.last_step_us = last_step_us_;
    snapshot.average_step_us = average_step_us_;
    snapshot.peak_step_us = peak_step_us_;
    snapshot.scheduler_lag_us = scheduler_lag_us_;
    snapshot.utilization_percent = snapshot.fixed_dt_seconds > 0.0
        ? average_step_us_ / (snapshot.fixed_dt_seconds * 1.0e6) * 100.0
        : 0.0;
    snapshot.step_stats = world_.last_step_stats();
    snapshot.body_count = world_.body_count();
    snapshot.static_triangle_count = world_.static_triangle_count();
    snapshot.gravity = world_.gravity();
    snapshot.monitored_body = monitored_body_;
    if (const RigidBody* body = world_.get(monitored_body_); body != nullptr) {
        snapshot.has_monitored_body = true;
        snapshot.body = *body;
    }
    snapshot.published_at = now;

    const std::scoped_lock lock(snapshot_mutex_);
    snapshot.sequence = published_.sequence + 1U;
    published_ = std::move(snapshot);
}

PhysicsThreadSnapshot PhysicsThread::snapshot() const noexcept {
    const std::scoped_lock lock(snapshot_mutex_);
    return published_;
}

void PhysicsThread::run(const std::stop_token stop_token) noexcept {
    const double fixed_dt_seconds = 1.0 / config_.frequency_hz;
    const auto fixed_dt = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(fixed_dt_seconds));
    auto next_tick = Clock::now() + fixed_dt;
    measured_window_start_ = Clock::now();

    while (!stop_token.stop_requested()) {
        {
            std::unique_lock lock(command_mutex_);
            command_cv_.wait_until(lock, next_tick, [&] {
                return stop_token.stop_requested() || !commands_.empty();
            });
        }
        if (stop_token.stop_requested()) {
            break;
        }

        const bool commands_changed_state = process_commands();
        auto now = Clock::now();

        if (resync_schedule_) {
            next_tick = now + fixed_dt;
            resync_schedule_ = false;
        }

        if (paused_) {
            while (pending_single_steps_ > 0U && !stop_token.stop_requested()) {
                execute_step(fixed_dt_seconds);
                --pending_single_steps_;
            }
            if (commands_changed_state || pending_single_steps_ == 0U) {
                publish_snapshot(Clock::now());
            }

            std::unique_lock lock(command_mutex_);
            command_cv_.wait(lock, [&] {
                return stop_token.stop_requested() || !commands_.empty();
            });
            next_tick = Clock::now() + fixed_dt;
            continue;
        }

        if (now < next_tick) {
            if (commands_changed_state) {
                publish_snapshot(now);
            }
            continue;
        }

        // A very large scheduling stall is never paid back as an unbounded burst.
        // Skip old deadlines first, leaving at most one current tick to execute.
        const double initial_lag_seconds = std::chrono::duration<double>(now - next_tick).count();
        if (initial_lag_seconds > config_.max_lag_seconds) {
            const auto skipped = static_cast<std::uint64_t>(initial_lag_seconds / fixed_dt_seconds);
            dropped_ticks_ += skipped;
            next_tick += fixed_dt * static_cast<Clock::duration::rep>(skipped);
        }

        std::uint32_t steps_this_wake = 0;
        while (now >= next_tick && steps_this_wake < config_.max_catchup_steps && !stop_token.stop_requested()) {
            const double lag_us = std::chrono::duration<double, std::micro>(now - next_tick).count();
            scheduler_lag_us_ = std::max(0.0, lag_us);
            if (scheduler_lag_us_ > fixed_dt_seconds * 500000.0) {
                ++deadline_misses_;
            }
            execute_step(fixed_dt_seconds);
            ++steps_this_wake;
            if (steps_this_wake > 1U) {
                ++catchup_steps_;
            }
            next_tick += fixed_dt;
            now = Clock::now();
        }

        // max_catchup_steps is a hard budget. If we are still late, drop the
        // remaining old deadlines and rejoin the wall-clock schedule.
        if (now >= next_tick) {
            const double behind_seconds = std::chrono::duration<double>(now - next_tick).count();
            const auto skipped = static_cast<std::uint64_t>(behind_seconds / fixed_dt_seconds) + 1U;
            dropped_ticks_ += skipped;
            next_tick += fixed_dt * static_cast<Clock::duration::rep>(skipped);
        }

        const double measurement_seconds = std::chrono::duration<double>(now - measured_window_start_).count();
        if (measurement_seconds >= 0.5) {
            measured_hz_ = static_cast<double>(measured_window_steps_) / measurement_seconds;
            measured_window_steps_ = 0;
            measured_window_start_ = now;
            peak_step_us_ = last_step_us_;
        }
        publish_snapshot(now);
    }

    running_.store(false, std::memory_order_release);
}

double physics_snapshot_interpolation_alpha(const PhysicsThreadSnapshot& snapshot,
                                            const Clock::time_point now) noexcept {
    if (!snapshot.has_monitored_body || snapshot.paused || !(snapshot.fixed_dt_seconds > 0.0)) {
        return 1.0;
    }
    const double age_seconds = std::max(0.0, std::chrono::duration<double>(now - snapshot.published_at).count());
    return std::clamp(age_seconds / snapshot.fixed_dt_seconds, 0.0, 1.0);
}

RigidBodyState interpolated_snapshot_state(const PhysicsThreadSnapshot& snapshot,
                                           const double alpha) noexcept {
    if (!snapshot.has_monitored_body) {
        return {};
    }
    const double clamped = std::clamp(alpha, 0.0, 1.0);
    const float clamped_f = static_cast<float>(clamped);
    const RigidBody& body = snapshot.body;
    return {
        .position = lerp(body.previous.position, body.current.position, clamped),
        .orientation = math::Quatf::normalized_lerp(body.previous.orientation, body.current.orientation, clamped_f),
        .linear_velocity = lerp(body.previous.linear_velocity, body.current.linear_velocity, clamped),
        .linear_acceleration = lerp(body.previous.linear_acceleration, body.current.linear_acceleration, clamped),
        .angular_velocity = lerp(body.previous.angular_velocity, body.current.angular_velocity, clamped_f),
        .angular_acceleration = lerp(body.previous.angular_acceleration, body.current.angular_acceleration, clamped_f)
    };
}

} // namespace ocs::physics
