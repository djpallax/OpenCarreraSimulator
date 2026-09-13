#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "ocs/vehicle/vehicle.hpp"
#include "ocs/vehicle/vehicle_definition.hpp"

namespace {

ocs::vehicle::VehicleConfig rwd_config() {
    ocs::vehicle::VehicleConfig config{};
    config.drive_layout = ocs::vehicle::DriveLayout::rear_wheel_drive;
    config.drive_acceleration = 10.0;
    config.wheels = {{
        {.suspension_mount_local = { 1.0F,  0.8F, 0.64F}, .radius = 0.34, .rest_length = 0.30, .max_compression = 0.14, .max_droop = 0.12, .spring_rate = 36000.0, .damper_rate = 4500.0, .steerable = true},
        {.suspension_mount_local = { 1.0F, -0.8F, 0.64F}, .radius = 0.34, .rest_length = 0.30, .max_compression = 0.14, .max_droop = 0.12, .spring_rate = 36000.0, .damper_rate = 4500.0, .steerable = true},
        {.suspension_mount_local = {-1.0F,  0.8F, 0.64F}, .radius = 0.34, .rest_length = 0.30, .max_compression = 0.14, .max_droop = 0.12, .spring_rate = 36000.0, .damper_rate = 4500.0, .steerable = false},
        {.suspension_mount_local = {-1.0F, -0.8F, 0.64F}, .radius = 0.34, .rest_length = 0.30, .max_compression = 0.14, .max_droop = 0.12, .spring_rate = 36000.0, .damper_rate = 4500.0, .steerable = false}
    }};
    return config;
}

ocs::physics::RigidBodyDesc body_desc() {
    ocs::physics::RigidBodyDesc desc{};
    desc.mass = 1000.0;
    desc.inertia_diagonal = {1000.0F, 1000.0F, 1000.0F};
    desc.dynamic = true;
    desc.state.position = {0.0, 0.0, 0.0};
    return desc;
}

std::vector<ocs::physics::StaticTriangle> wide_floor() {
    return {
        {{-5.0, -5.0, 0.0}, {5.0, -5.0, 0.0}, {5.0, 5.0, 0.0}},
        {{-5.0, -5.0, 0.0}, {5.0, 5.0, 0.0}, {-5.0, 5.0, 0.0}}
    };
}

} // namespace

TEST(Vehicle, RearWheelDriveMarksOnlyRearWheelsDriven) {
    const auto config = rwd_config();
    EXPECT_FALSE(ocs::vehicle::wheel_is_driven(config.drive_layout, ocs::vehicle::WheelPosition::front_left));
    EXPECT_FALSE(ocs::vehicle::wheel_is_driven(config.drive_layout, ocs::vehicle::WheelPosition::front_right));
    EXPECT_TRUE(ocs::vehicle::wheel_is_driven(config.drive_layout, ocs::vehicle::WheelPosition::rear_left));
    EXPECT_TRUE(ocs::vehicle::wheel_is_driven(config.drive_layout, ocs::vehicle::WheelPosition::rear_right));
}

TEST(Vehicle, SuspensionContactReportsLengthAndCompression) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    world.set_static_triangles(wide_floor());
    auto desc = body_desc();
    // mount z = 0.64 m and radius = 0.34 m -> suspension length = 0.30 m
    const auto handle = world.create_body(desc);
    const auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);

    auto config = rwd_config();
    ocs::vehicle::VehicleRuntimeState runtime{};
    const auto contacts = ocs::vehicle::update_suspension_contacts(world, *body, config, runtime, 0.002);
    EXPECT_EQ(contacts.wheels_in_contact, 4U);
    for (const auto& wheel : contacts.wheels) {
        EXPECT_TRUE(wheel.in_contact);
        // Suspension mounts are authored as Vec3f, so metre-scale geometry enters
        // the double-precision solver with normal float quantization (~1e-8 m).
        EXPECT_NEAR(wheel.suspension_length, 0.30, 1.0e-6);
        EXPECT_NEAR(wheel.compression, 0.0, 1.0e-6);
        EXPECT_NEAR(wheel.normal_force, 0.0, 1.0e-2);
    }
}

TEST(Vehicle, DamperVelocityIsPositiveWhileSuspensionCompresses) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    world.set_static_triangles(wide_floor());
    const auto handle = world.create_body(body_desc());
    auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);

    const auto config = rwd_config();
    ocs::vehicle::VehicleRuntimeState runtime{};
    (void)ocs::vehicle::update_suspension_contacts(world, *body, config, runtime, 0.01);
    ASSERT_TRUE(world.set_pose(handle, {0.0, 0.0, -0.02}, body->current.orientation, true));
    body = world.get(handle);
    ASSERT_NE(body, nullptr);
    const auto contacts = ocs::vehicle::update_suspension_contacts(world, *body, config, runtime, 0.01);
    EXPECT_GT(contacts.wheels[0].compression_velocity, 0.0);
    EXPECT_GT(contacts.wheels[0].damper_force, 0.0);
}

TEST(Vehicle, SpringDamperSupportAppliesUpwardForceAtFourStations) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    world.set_static_triangles(wide_floor());
    auto desc = body_desc();
    desc.state.position.z = -0.08; // 80 mm compression relative to rest.
    const auto handle = world.create_body(desc);
    auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);

    const auto config = rwd_config();
    ocs::vehicle::VehicleRuntimeState runtime{};
    const auto contacts = ocs::vehicle::update_suspension_contacts(world, *body, config, runtime, 0.002);
    EXPECT_EQ(contacts.wheels_in_contact, 4U);
    const double applied = ocs::vehicle::apply_suspension_forces(world, handle, config, runtime);
    // 80 mm comes through float-authored mount geometry; millinewton-scale
    // differences are numerical representation, not a physics regression.
    EXPECT_NEAR(applied, 4.0 * 36000.0 * 0.08, 1.0e-2);

    world.step(0.002);
    body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_GT(body->current.linear_acceleration.z, 0.0);
}

TEST(Vehicle, AirborneDrivenWheelContributesNoPropulsion) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    // Floor exists only under +Y, so RL contacts while RR is airborne.
    world.set_static_triangles({
        {{-5.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {5.0, 2.0, 0.0}},
        {{-5.0, 0.0, 0.0}, {5.0, 2.0, 0.0}, {-5.0, 2.0, 0.0}}
    });
    const auto handle = world.create_body(body_desc());
    auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);

    const auto config = rwd_config();
    const auto contacts = ocs::vehicle::sample_wheel_contacts(world, *body, config);
    EXPECT_EQ(contacts.driven_wheels, 2U);
    EXPECT_EQ(contacts.driven_wheels_in_contact, 1U);
    EXPECT_TRUE(contacts.wheels[2].in_contact);
    EXPECT_FALSE(contacts.wheels[3].in_contact);

    const double applied = ocs::vehicle::apply_driven_wheel_force(
        world, handle, *body, config, contacts, 1.0, 1.0);
    EXPECT_NEAR(applied, 5000.0, 1.0e-9);

    world.step(0.01);
    body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_NEAR(body->current.linear_acceleration.x, 5.0, 1.0e-9);
}

TEST(Vehicle, NoDrivenWheelContactMeansNoDriveForce) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    const auto handle = world.create_body(body_desc());
    auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);

    const auto config = rwd_config();
    const auto contacts = ocs::vehicle::sample_wheel_contacts(world, *body, config);
    EXPECT_EQ(contacts.driven_wheels_in_contact, 0U);
    EXPECT_DOUBLE_EQ(ocs::vehicle::apply_driven_wheel_force(
        world, handle, *body, config, contacts, 1.0, 1.0), 0.0);
}

TEST(Vehicle, WheelbaseComesFromFrontAndRearSuspensionMounts) {
    const auto config = rwd_config();
    EXPECT_NEAR(ocs::vehicle::vehicle_wheelbase(config), 2.0, 1.0e-9);
}

TEST(Vehicle, BicycleSteeringCannotGenerateYawAtZeroLongitudinalSpeed) {
    auto config = rwd_config();
    config.maximum_steer_angle_radians = 0.42;
    EXPECT_DOUBLE_EQ(ocs::vehicle::provisional_steering_yaw_rate(config, 0.0, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(ocs::vehicle::provisional_steering_yaw_rate(config, 0.0, -1.0), 0.0);
}

TEST(Vehicle, BicycleSteeringYawRateScalesWithSpeedAndSteeringDirection) {
    auto config = rwd_config();
    config.maximum_steer_angle_radians = 0.30;
    config.provisional_max_lateral_acceleration = 100.0; // keep this test on pure bicycle geometry

    const double slow = ocs::vehicle::provisional_steering_yaw_rate(config, 1.0, 1.0);
    const double fast = ocs::vehicle::provisional_steering_yaw_rate(config, 2.0, 1.0);
    const double right = ocs::vehicle::provisional_steering_yaw_rate(config, 2.0, -1.0);

    EXPECT_GT(slow, 0.0);
    EXPECT_NEAR(fast, slow * 2.0, 1.0e-9);
    EXPECT_NEAR(right, -fast, 1.0e-9);
}


TEST(Vehicle, LongitudinalSlipIsZeroAtPureRolling) {
    EXPECT_NEAR(ocs::vehicle::longitudinal_slip_ratio(10.0, 10.0, 2.0), 0.0, 1.0e-12);
    EXPECT_GT(ocs::vehicle::longitudinal_slip_ratio(11.0, 10.0, 2.0), 0.0);
    EXPECT_LT(ocs::vehicle::longitudinal_slip_ratio(9.0, 10.0, 2.0), 0.0);
}

TEST(Vehicle, LongitudinalTireForceIsLoadLimited) {
    auto wheel = rwd_config().wheels[2];
    wheel.longitudinal_stiffness = 100000.0;
    wheel.peak_friction_coefficient = 1.2;
    const double load = 3000.0;
    const double positive = ocs::vehicle::longitudinal_tire_force(wheel, 10.0, load);
    const double negative = ocs::vehicle::longitudinal_tire_force(wheel, -10.0, load);
    EXPECT_LE(positive, 3600.0 + 1.0e-9);
    EXPECT_GE(negative, -3600.0 - 1.0e-9);
    EXPECT_GT(positive, 3500.0);
    EXPECT_LT(negative, -3500.0);
}

TEST(Vehicle, AirborneDrivenWheelSpinsButCannotPushChassis) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    const auto handle = world.create_body(body_desc());
    auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);

    auto config = rwd_config();
    config.maximum_drive_torque_nm = 2000.0;
    ocs::vehicle::VehicleRuntimeState runtime{};
    runtime.contacts = ocs::vehicle::sample_wheel_contacts(world, *body, config);

    const auto result = ocs::vehicle::update_longitudinal_tire_dynamics(
        world, handle, *body, config, runtime, 1.0, 0.0, 1.0, 0.0, 0.002);
    EXPECT_DOUBLE_EQ(result.total_longitudinal_force_n, 0.0);
    EXPECT_GT(runtime.wheel_angular_velocity_rad_s[2], 0.0);
    EXPECT_GT(runtime.wheel_angular_velocity_rad_s[3], 0.0);

    world.step(0.002);
    body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_NEAR(body->current.linear_acceleration.x, 0.0, 1.0e-12);
}

TEST(Vehicle, DrivenWheelSlipAppliesForwardTireForceAtRoadContact) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    world.set_static_triangles(wide_floor());
    auto desc = body_desc();
    desc.state.position.z = -0.08;
    const auto handle = world.create_body(desc);
    auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);

    auto config = rwd_config();
    ocs::vehicle::VehicleRuntimeState runtime{};
    runtime.contacts = ocs::vehicle::update_suspension_contacts(world, *body, config, runtime, 0.002);
    runtime.wheel_angular_velocity_rad_s[2] = 12.0;
    runtime.wheel_angular_velocity_rad_s[3] = 12.0;

    const auto result = ocs::vehicle::update_longitudinal_tire_dynamics(
        world, handle, *body, config, runtime, 0.0, 0.0, 1.0, 0.0, 0.002);
    EXPECT_GT(result.total_longitudinal_force_n, 0.0);
    EXPECT_GT(runtime.contacts.wheels[2].slip_ratio, 0.0);
    EXPECT_GT(runtime.contacts.wheels[2].longitudinal_tire_force_n, 0.0);

    world.step(0.002);
    body = world.get(handle);
    ASSERT_NE(body, nullptr);
    EXPECT_GT(body->current.linear_acceleration.x, 0.0);
}

TEST(Vehicle, BrakeTorqueOpposesForwardWheelRotation) {
    ocs::physics::PhysicsWorld world;
    world.set_gravity({});
    auto desc = body_desc();
    const auto handle = world.create_body(desc);
    auto* body = world.get(handle);
    ASSERT_NE(body, nullptr);

    auto config = rwd_config();
    ocs::vehicle::VehicleRuntimeState runtime{};
    runtime.contacts = ocs::vehicle::sample_wheel_contacts(world, *body, config);
    runtime.wheel_angular_velocity_rad_s.fill(20.0);

    (void)ocs::vehicle::update_longitudinal_tire_dynamics(
        world, handle, *body, config, runtime, 0.0, 1.0, 1.0, 0.0, 0.002);
    for (double omega : runtime.wheel_angular_velocity_rad_s) {
        EXPECT_LT(omega, 20.0);
        EXPECT_GE(omega, 0.0);
    }
}


TEST(VehicleDefinition, MissingConfigFileUsesEngineDefaults) {
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "ocs_vehicle_definition_missing_cfg";
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);

    const auto definition = ocs::vehicle::load_vehicle_definition(directory);
    EXPECT_FALSE(definition.config_file_found);
    EXPECT_DOUBLE_EQ(definition.mass_kg, 1200.0);
    EXPECT_DOUBLE_EQ(definition.dynamics.maximum_brake_torque_nm_per_wheel, 1200.0);
    EXPECT_NEAR(definition.dynamics.maximum_steer_angle_radians, 0.42, 1.0e-12);
    EXPECT_EQ(definition.body_model_path(), directory / "body.ocsmodel");
    EXPECT_EQ(definition.wheel_model_path(), directory / "wheel.ocsmodel");

    std::filesystem::remove_all(directory);
}

TEST(VehicleDefinition, PartialConfigOverridesOnlyPresentAttributes) {
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "ocs_vehicle_definition_partial_cfg";
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    {
        std::ofstream output(directory / "vehicle.cfg");
        output << "name = Test Car\n"
               << "mass_kg = 1337\n"
               << "maximum_brake_torque_nm_per_wheel = 1900\n"
               << "maximum_steer_angle_degrees = 30\n"
               << "spring_rate_n_m = 42000\n"
               << "unknown_future_key = 123\n";
    }

    const auto definition = ocs::vehicle::load_vehicle_definition(directory);
    EXPECT_TRUE(definition.config_file_found);
    EXPECT_EQ(definition.name, "Test Car");
    EXPECT_DOUBLE_EQ(definition.mass_kg, 1337.0);
    EXPECT_DOUBLE_EQ(definition.dynamics.maximum_brake_torque_nm_per_wheel, 1900.0);
    EXPECT_NEAR(definition.dynamics.maximum_steer_angle_radians,
                3.14159265358979323846 / 6.0, 1.0e-12);
    EXPECT_DOUBLE_EQ(definition.dynamics.wheels[0].spring_rate, 42000.0);
    EXPECT_DOUBLE_EQ(definition.dynamics.wheels[3].spring_rate, 42000.0);
    // Omitted values retain engine defaults.
    EXPECT_DOUBLE_EQ(definition.dynamics.maximum_drive_torque_nm, 2200.0);
    EXPECT_DOUBLE_EQ(definition.dynamics.wheels[0].damper_rate, 4500.0);
    EXPECT_EQ(definition.applied_attribute_count, 5U);

    std::filesystem::remove_all(directory);
}
