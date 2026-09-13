#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ocs::physics {

struct FixedStepConfig {
    double frequency_hz = 500.0;
    std::uint32_t max_substeps = 16;
    double max_frame_delta_seconds = 0.1;
};

struct FixedStepResult {
    std::uint32_t steps = 0;
    double alpha = 0.0;
    double accumulator_seconds = 0.0;
    double simulation_time_seconds = 0.0;
    double dropped_time_seconds = 0.0;
};

class FixedStepAccumulator {
public:
    explicit FixedStepAccumulator(FixedStepConfig config = {}) noexcept
        : config_(sanitize(config)) {}

    void reset() noexcept {
        accumulator_seconds_ = 0.0;
        simulation_time_seconds_ = 0.0;
        dropped_time_seconds_ = 0.0;
    }

    void set_frequency_hz(const double frequency_hz) noexcept {
        config_.frequency_hz = std::clamp(frequency_hz, 1.0, 5000.0);
        const double dt = fixed_dt_seconds();
        accumulator_seconds_ = std::min(accumulator_seconds_, dt);
    }

    [[nodiscard]] double frequency_hz() const noexcept { return config_.frequency_hz; }
    [[nodiscard]] double fixed_dt_seconds() const noexcept { return 1.0 / config_.frequency_hz; }
    [[nodiscard]] const FixedStepConfig& config() const noexcept { return config_; }

    template <typename StepFunction>
    [[nodiscard]] FixedStepResult single_step(StepFunction&& step_function) {
        const double fixed_dt = fixed_dt_seconds();
        step_function(fixed_dt);
        simulation_time_seconds_ += fixed_dt;
        return {
            .steps = 1,
            .alpha = accumulator_seconds_ / fixed_dt,
            .accumulator_seconds = accumulator_seconds_,
            .simulation_time_seconds = simulation_time_seconds_,
            .dropped_time_seconds = dropped_time_seconds_
        };
    }

    template <typename StepFunction>
    [[nodiscard]] FixedStepResult advance(const double frame_delta_seconds,
                                          StepFunction&& step_function) {
        const double fixed_dt = fixed_dt_seconds();
        const double clamped_delta = std::clamp(
            frame_delta_seconds,
            0.0,
            config_.max_frame_delta_seconds);
        accumulator_seconds_ += clamped_delta;

        std::uint32_t steps = 0;
        while (accumulator_seconds_ + 1.0e-12 >= fixed_dt && steps < config_.max_substeps) {
            step_function(fixed_dt);
            accumulator_seconds_ -= fixed_dt;
            simulation_time_seconds_ += fixed_dt;
            ++steps;
        }

        if (accumulator_seconds_ >= fixed_dt) {
            const double kept = std::fmod(accumulator_seconds_, fixed_dt);
            dropped_time_seconds_ += accumulator_seconds_ - kept;
            accumulator_seconds_ = kept;
        }

        return {
            .steps = steps,
            .alpha = std::clamp(accumulator_seconds_ / fixed_dt, 0.0, 1.0),
            .accumulator_seconds = accumulator_seconds_,
            .simulation_time_seconds = simulation_time_seconds_,
            .dropped_time_seconds = dropped_time_seconds_
        };
    }

private:
    static FixedStepConfig sanitize(FixedStepConfig config) noexcept {
        config.frequency_hz = std::clamp(config.frequency_hz, 1.0, 5000.0);
        config.max_substeps = std::max(config.max_substeps, 1U);
        config.max_frame_delta_seconds = std::clamp(config.max_frame_delta_seconds, 0.001, 1.0);
        return config;
    }

    FixedStepConfig config_{};
    double accumulator_seconds_ = 0.0;
    double simulation_time_seconds_ = 0.0;
    double dropped_time_seconds_ = 0.0;
};

} // namespace ocs::physics
