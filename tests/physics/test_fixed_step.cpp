#include <gtest/gtest.h>

#include <array>

#include "ocs/physics/fixed_step.hpp"

TEST(FixedStep, ProducesSamePhysicsStepCountAcrossRenderCadences) {
    constexpr double duration = 1.0;

    for (const double render_hz : std::array{60.0, 144.0, 240.0}) {
        ocs::physics::FixedStepAccumulator clock({
            .frequency_hz = 500.0,
            .max_substeps = 32,
            .max_frame_delta_seconds = 0.1
        });

        std::uint32_t steps = 0;
        const int frames = static_cast<int>(render_hz * duration);
        for (int frame = 0; frame < frames; ++frame) {
            const auto result = clock.advance(1.0 / render_hz, [&](const double) { ++steps; });
            (void)result;
        }
        EXPECT_NEAR(static_cast<double>(steps), 500.0, 1.0);
    }
}

TEST(FixedStep, CapsCatchUpAndTracksDroppedTime) {
    ocs::physics::FixedStepAccumulator clock({
        .frequency_hz = 500.0,
        .max_substeps = 4,
        .max_frame_delta_seconds = 0.1
    });

    const auto result = clock.advance(0.1, [](const double) {});
    EXPECT_EQ(result.steps, 4U);
    EXPECT_GT(result.dropped_time_seconds, 0.0);
    EXPECT_GE(result.alpha, 0.0);
    EXPECT_LT(result.alpha, 1.0);
}
