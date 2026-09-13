#include <gtest/gtest.h>

#include <cmath>

#include "ocs/physics/physics_world.hpp"

namespace {

ocs::physics::RigidBodyDesc dynamic_body() {
    ocs::physics::RigidBodyDesc desc{};
    desc.mass = 2.0;
    desc.inertia_diagonal = {2.0F, 3.0F, 4.0F};
    desc.dynamic = true;
    desc.ground_contact_enabled = false;
    return desc;
}

} // namespace

TEST(PhysicsWorld, GravityFreeFallMatchesExpectedVelocity) {
    ocs::physics::PhysicsWorld world;
    auto desc = dynamic_body();
    desc.state.position = {0.0, 0.0, 100.0};
    const auto handle = world.create_body(desc);

    constexpr double dt = 0.002;
    for (int i = 0; i < 500; ++i) {
        world.step(dt);
    }

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_NEAR(body->current.linear_velocity.z, -9.80665, 1.0e-4);
}

TEST(PhysicsWorld, ZeroForcePreservesVelocityWithoutGravity) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    auto desc = dynamic_body();
    desc.state.linear_velocity = {3.0, -2.0, 1.0};
    const auto handle = world.create_body(desc);

    for (int i = 0; i < 1000; ++i) {
        world.step(0.001);
    }

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_NEAR(body->current.linear_velocity.x, 3.0, 1.0e-9);
    EXPECT_NEAR(body->current.linear_velocity.y, -2.0, 1.0e-9);
    EXPECT_NEAR(body->current.linear_velocity.z, 1.0, 1.0e-9);
}

TEST(PhysicsWorld, ConstantForceProducesExpectedAcceleration) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    const auto handle = world.create_body(dynamic_body());

    world.add_force(handle, {10.0, 0.0, 0.0});
    world.step(0.01);

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_NEAR(body->current.linear_acceleration.x, 5.0, 1.0e-9);
    EXPECT_NEAR(body->current.linear_velocity.x, 0.05, 1.0e-9);
}

TEST(PhysicsWorld, ForceAtCenterProducesNoTorque) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    auto desc = dynamic_body();
    desc.state.position = {4.0, 2.0, 1.0};
    const auto handle = world.create_body(desc);

    world.add_force_at_point(handle, {0.0, 10.0, 0.0}, desc.state.position);
    world.step(0.01);

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_NEAR(body->current.angular_velocity.length(), 0.0F, 1.0e-6F);
}

TEST(PhysicsWorld, OffCenterForceProducesAngularAcceleration) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    const auto handle = world.create_body(dynamic_body());

    world.add_force_at_point(handle, {0.0, 10.0, 0.0}, {1.0, 0.0, 0.0});
    world.step(0.01);

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_GT(std::abs(body->current.angular_acceleration.z), 0.0F);
}

TEST(PhysicsWorld, QuaternionRemainsNormalizedOverLongRun) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    auto desc = dynamic_body();
    desc.state.angular_velocity = {0.7F, -0.4F, 1.2F};
    const auto handle = world.create_body(desc);

    for (int i = 0; i < 20000; ++i) {
        world.step(0.001);
    }

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_NEAR(body->current.orientation.length_squared(), 1.0F, 1.0e-4F);
}

TEST(PhysicsWorld, GroundPlanePreventsFallingThrough) {
    ocs::physics::PhysicsWorld world;
    auto desc = dynamic_body();
    desc.ground_contact_enabled = true;
    desc.ground_clearance = 1.25;
    desc.state.position = {0.0, 0.0, 3.0};
    const auto handle = world.create_body(desc);

    for (int i = 0; i < 3000; ++i) {
        world.step(0.001);
    }

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_GE(body->current.position.z, 1.25);
    EXPECT_TRUE(body->grounded);
}

TEST(PhysicsWorld, InterpolatesBetweenPreviousAndCurrentStates) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    auto desc = dynamic_body();
    desc.state.linear_velocity = {10.0, 0.0, 0.0};
    const auto handle = world.create_body(desc);

    world.step(0.1);
    const auto state = world.interpolated_state(handle, 0.5);
    EXPECT_NEAR(state.position.x, 0.5, 1.0e-9);
}

namespace {

std::vector<ocs::physics::StaticTriangle> flat_floor() {
    return {
        {{-20.0, -20.0, 0.0}, {20.0, -20.0, 0.0}, {20.0, 20.0, 0.0}},
        {{-20.0, -20.0, 0.0}, {20.0, 20.0, 0.0}, {-20.0, 20.0, 0.0}}
    };
}

ocs::physics::RigidBodyDesc dynamic_box() {
    auto desc = dynamic_body();
    desc.mass = 10.0;
    desc.inertia_diagonal = {1.6667F, 1.6667F, 1.6667F};
    desc.static_mesh_contact_enabled = true;
    desc.collision_box.half_extents = {0.5F, 0.5F, 0.5F};
    desc.restitution = 0.03;
    desc.friction_coefficient = 0.9;
    desc.linear_damping = 0.02;
    desc.angular_damping = 0.25;
    return desc;
}

} // namespace

TEST(PhysicsWorld, StaticTriangleFloorSupportsOrientedBox) {
    ocs::physics::PhysicsWorld world;
    world.set_static_triangles(flat_floor());
    auto desc = dynamic_box();
    desc.state.position = {0.0, 0.0, 3.0};
    const auto handle = world.create_body(desc);

    for (int i = 0; i < 6000; ++i) {
        world.step(0.001);
    }

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_TRUE(body->grounded);
    EXPECT_GE(body->current.position.z, 0.49);
    EXPECT_GT(body->contact_count, 0U);
    EXPECT_NEAR(body->support_normal.z, 1.0, 1.0e-4);
    EXPECT_EQ(world.static_triangle_count(), 2U);
}

TEST(PhysicsWorld, TiltedBoxSettlesParallelToStaticFloor) {
    ocs::physics::PhysicsWorld world;
    world.set_static_triangles(flat_floor());
    auto desc = dynamic_box();
    desc.state.position = {0.0, 0.0, 3.0};
    desc.state.orientation = (
        ocs::math::Quatf::from_axis_angle({1.0F, 0.0F, 0.0F}, 0.70F) *
        ocs::math::Quatf::from_axis_angle({0.0F, 1.0F, 0.0F}, 0.35F)).normalized();
    const auto handle = world.create_body(desc);

    for (int i = 0; i < 10000; ++i) {
        world.step(0.001);
    }

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    const ocs::math::Vec3f up = body->current.orientation.rotate({0.0F, 0.0F, 1.0F});
    EXPECT_TRUE(body->grounded);
    EXPECT_GT(up.z, 0.999F);
    EXPECT_LT(body->current.angular_velocity.length(), 0.03F);
    EXPECT_NEAR(body->current.position.z, 0.5, 0.02);
}

TEST(PhysicsWorld, StaticWallRejectsBoxFromBackSideOfInwardNormal) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    world.set_static_triangles({
        // x = 2 plane, winding gives normal -X: legal/drivable side is x < 2.
        {{2.0, -10.0, 0.0}, {2.0, -10.0, 4.0}, {2.0, 10.0, 4.0}},
        {{2.0, -10.0, 0.0}, {2.0, 10.0, 4.0}, {2.0, 10.0, 0.0}}
    });
    auto desc = dynamic_box();
    desc.state.position = {0.0, 0.0, 1.0};
    desc.state.linear_velocity = {8.0, 0.0, 0.0};
    const auto handle = world.create_body(desc);

    for (int i = 0; i < 1000; ++i) {
        world.step(0.001);
    }

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_LE(body->current.position.x, 1.51);
    EXPECT_LT(body->current.linear_velocity.x, 0.1);
}

TEST(PhysicsWorld, CapturesLastExternalForceForDebugVisualization) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    const auto handle = world.create_body(dynamic_body());

    world.add_force(handle, {12.0, -4.0, 3.0});
    world.add_torque(handle, {1.0F, 2.0F, -3.0F});
    world.step(0.01);

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_DOUBLE_EQ(body->last_external_force.x, 12.0);
    EXPECT_DOUBLE_EQ(body->last_external_force.y, -4.0);
    EXPECT_DOUBLE_EQ(body->last_external_force.z, 3.0);
    EXPECT_FLOAT_EQ(body->last_external_torque.x, 1.0F);
    EXPECT_FLOAT_EQ(body->last_external_torque.y, 2.0F);
    EXPECT_FLOAT_EQ(body->last_external_torque.z, -3.0F);
}

TEST(PhysicsWorld, RetainsContactSamplesForSpatialDebugging) {
    ocs::physics::PhysicsWorld world;
    world.set_static_triangles(flat_floor());
    auto desc = dynamic_box();
    desc.state.position = {0.0, 0.0, 1.5};
    const auto handle = world.create_body(desc);

    for (int i = 0; i < 2500; ++i) {
        world.step(0.001);
    }

    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    ASSERT_GT(body->debug_contact_count, 0U);
    ASSERT_LE(body->debug_contact_count, body->debug_contacts.size());
    for (std::uint32_t index = 0; index < body->debug_contact_count; ++index) {
        EXPECT_GT(body->debug_contacts[index].normal.z, 0.99);
        EXPECT_NEAR(body->debug_contacts[index].point.z, 0.0, 1.0e-3);
    }
}

TEST(PhysicsWorld, StaticRaycastHitsAuthoredFrontFace) {
    ocs::physics::PhysicsWorld world;
    world.set_static_triangles(flat_floor());

    const auto hit = world.raycast_static({0.0, 0.0, 2.0}, {0.0, 0.0, -2.0}, 3.0);
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->distance, 2.0, 1.0e-9);
    EXPECT_NEAR(hit->point.z, 0.0, 1.0e-9);
    EXPECT_NEAR(hit->normal.z, 1.0, 1.0e-9);

    const auto backface = world.raycast_static({0.0, 0.0, -1.0}, {0.0, 0.0, 1.0}, 3.0);
    EXPECT_FALSE(backface.has_value());
}

namespace {

std::vector<ocs::physics::StaticTriangle> tiled_floor(const int cells_x, const int cells_y) {
    std::vector<ocs::physics::StaticTriangle> triangles;
    triangles.reserve(static_cast<std::size_t>(cells_x * cells_y * 2));
    for (int y = 0; y < cells_y; ++y) {
        for (int x = 0; x < cells_x; ++x) {
            const double x0 = static_cast<double>(x);
            const double x1 = static_cast<double>(x + 1);
            const double y0 = static_cast<double>(y);
            const double y1 = static_cast<double>(y + 1);
            triangles.push_back({{x0, y0, 0.0}, {x1, y0, 0.0}, {x1, y1, 0.0}});
            triangles.push_back({{x0, y0, 0.0}, {x1, y1, 0.0}, {x0, y1, 0.0}});
        }
    }
    return triangles;
}

} // namespace

TEST(PhysicsWorld, StaticBvhRaycastPrunesLargeTriangleSet) {
    ocs::physics::PhysicsWorld world;
    world.set_static_triangles(tiled_floor(128, 128));
    ASSERT_EQ(world.static_triangle_count(), 32768U);
    EXPECT_GT(world.static_bvh_node_count(), 1U);

    const auto hit = world.raycast_static({63.25, 71.75, 4.0}, {0.0, 0.0, -1.0}, 10.0);
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->distance, 4.0, 1.0e-9);
    EXPECT_NEAR(hit->normal.z, 1.0, 1.0e-9);

    // Fold the const ray-query counters into a published step snapshot.
    world.step(0.001);
    const auto& stats = world.last_step_stats();
    EXPECT_GT(stats.bvh_node_visits, 0U);
    EXPECT_LT(stats.triangle_tests, 256U);
}

TEST(PhysicsWorld, StaticBvhBroadphasePrunesBoxContacts) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    world.set_static_triangles(tiled_floor(128, 128));
    auto desc = dynamic_box();
    desc.state.position = {64.5, 64.5, 0.40};
    const auto handle = world.create_body(desc);

    world.step(0.001);
    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_GT(body->contact_count, 0U);
    EXPECT_LT(world.last_step_stats().triangle_tests, 5000U);
}

TEST(PhysicsWorld, StaticBvhRaycastPreservesOriginalTriangleIndex) {
    ocs::physics::PhysicsWorld world;
    world.set_static_triangles({
        // Degenerate source triangle is discarded from the BVH.
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
        {{-2.0, -2.0, 0.0}, {2.0, -2.0, 0.0}, {0.0, 2.0, 0.0}}
    });

    const auto hit = world.raycast_static({0.0, 0.0, 2.0}, {0.0, 0.0, -1.0}, 4.0);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->triangle_index, 1U);
}

TEST(PhysicsWorld, DeepPreexistingStaticOverlapDoesNotTeleportBody) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    world.set_static_triangles(flat_floor());
    auto desc = dynamic_box();
    desc.state.position = {0.0, 0.0, 0.0}; // lower corners start 0.5 m below floor
    desc.static_contact_max_penetration = 0.35;
    desc.static_contact_persistence_depth = 0.08;
    desc.upright_ground_contact_max_penetration = 0.12;
    desc.maximum_depenetration_speed = 3.0;
    const auto handle = world.create_body(desc);

    world.step(0.002);
    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    // Imported scenery can contain overlapping decorative shells. A body already
    // deeply behind one at tick start must not be projected half a metre away.
    EXPECT_NEAR(body->current.position.z, 0.0, 1.0e-9);
    EXPECT_EQ(body->contact_count, 0U);
}

TEST(PhysicsWorld, SweptImpactDepenetrationIsSpeedLimited) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    world.set_static_triangles(flat_floor());
    auto desc = dynamic_box();
    desc.state.position = {0.0, 0.0, 0.51};
    desc.state.linear_velocity = {0.0, 0.0, -100.0};
    desc.static_contact_max_penetration = 0.35;
    desc.static_contact_persistence_depth = 0.35;
    desc.upright_ground_contact_max_penetration = 0.25;
    desc.maximum_depenetration_speed = 3.0;
    const auto handle = world.create_body(desc);

    world.step(0.002);
    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_GT(body->contact_count, 0U);
    // Integration reaches z=0.31. Positional correction is capped to 3 m/s *
    // 0.002 s = 0.006 m for the entire tick, rather than teleporting near z=0.5.
    EXPECT_GE(body->current.position.z, 0.31);
    EXPECT_LE(body->current.position.z, 0.316001);
}
