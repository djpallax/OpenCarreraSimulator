#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <thread>

#include "ocs/physics/physics_thread.hpp"

namespace {

using namespace std::chrono_literals;

bool wait_for(const std::function<bool()>& predicate, const std::chrono::milliseconds timeout = 1000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(1ms);
    }
    return predicate();
}

ocs::physics::RigidBodyDesc moving_body() {
    ocs::physics::RigidBodyDesc desc{};
    desc.dynamic = true;
    desc.ground_contact_enabled = false;
    desc.state.linear_velocity = {10.0, 0.0, 0.0};
    return desc;
}

} // namespace

TEST(PhysicsThread, PausedSingleStepAdvancesExactlyOneFixedTick) {
    ocs::physics::PhysicsThread runner({
        .frequency_hz = 500.0,
        .max_catchup_steps = 4,
        .max_lag_seconds = 0.05,
        .start_paused = true
    });
    runner.setup_world().set_gravity({});
    const auto body = runner.setup_world().create_body(moving_body());
    runner.set_monitored_body(body);
    ASSERT_TRUE(runner.start());

    ASSERT_TRUE(wait_for([&] { return runner.snapshot().paused; }));
    runner.request_single_step();
    ASSERT_TRUE(wait_for([&] { return runner.snapshot().total_steps >= 1U; }));

    const auto snapshot = runner.snapshot();
    EXPECT_EQ(snapshot.total_steps, 1U);
    ASSERT_TRUE(snapshot.has_monitored_body);
    EXPECT_NEAR(snapshot.body.current.position.x, 0.02, 1.0e-9);
    runner.stop();
}

TEST(PhysicsThread, ResetCommandIsAppliedOnlyByPhysicsOwnerThread) {
    ocs::physics::PhysicsThread runner({
        .frequency_hz = 1000.0,
        .max_catchup_steps = 4,
        .max_lag_seconds = 0.05,
        .start_paused = true
    });
    runner.setup_world().set_gravity({});
    const auto body = runner.setup_world().create_body(moving_body());
    runner.set_monitored_body(body);
    ASSERT_TRUE(runner.start());

    ocs::physics::RigidBodyState reset{};
    reset.position = {42.0, -3.0, 7.0};
    runner.request_reset(body, reset);
    ASSERT_TRUE(wait_for([&] {
        const auto s = runner.snapshot();
        return s.has_monitored_body && s.body.current.position.x == 42.0;
    }));

    const auto snapshot = runner.snapshot();
    EXPECT_DOUBLE_EQ(snapshot.body.current.position.x, 42.0);
    EXPECT_DOUBLE_EQ(snapshot.body.current.position.y, -3.0);
    EXPECT_DOUBLE_EQ(snapshot.body.current.position.z, 7.0);
    EXPECT_EQ(snapshot.total_steps, 0U);
    runner.stop();
}

TEST(PhysicsThread, SnapshotInterpolationUsesOneTickOfRenderLatency) {
    ocs::physics::PhysicsThreadSnapshot snapshot{};
    snapshot.has_monitored_body = true;
    snapshot.fixed_dt_seconds = 0.002;
    snapshot.body.previous.position = {0.0, 0.0, 0.0};
    snapshot.body.current.position = {2.0, 0.0, 0.0};

    const auto state = ocs::physics::interpolated_snapshot_state(snapshot, 0.25);
    EXPECT_NEAR(state.position.x, 0.5, 1.0e-12);
}
