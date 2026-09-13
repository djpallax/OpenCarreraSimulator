#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "ocs/math/vec3d.hpp"
#include "ocs/math/vec3f.hpp"
#include "ocs/physics/physics_world.hpp"

namespace ocs::vehicle {

enum class DriveLayout : std::uint8_t {
    front_wheel_drive,
    rear_wheel_drive,
    all_wheel_drive
};

enum class WheelPosition : std::uint8_t {
    front_left = 0,
    front_right = 1,
    rear_left = 2,
    rear_right = 3
};

inline constexpr std::size_t kWheelCount = 4U;

struct WheelConfig {
    // Suspension attachment in body-local coordinates. The wheel hub travels
    // along body-local -Z from this point.
    math::Vec3f suspension_mount_local{};
    double radius = 0.34;
    double rest_length = 0.30;
    double max_compression = 0.14;
    double max_droop = 0.12;
    double spring_rate = 36000.0;       // N / m
    double damper_rate = 4500.0;        // N s / m

    // Step 9.4/9.5 wheel/tire rotational state. These are intentionally simple
    // physical parameters rather than a hidden chassis acceleration helper.
    double rotational_inertia = 1.50;               // kg m^2
    double longitudinal_stiffness = 18000.0;         // N per unit slip ratio
    double peak_friction_coefficient = 1.20;         // longitudinal mu
    bool steerable = false;
};

struct VehicleConfig {
    DriveLayout drive_layout = DriveLayout::rear_wheel_drive;
    std::array<WheelConfig, kWheelCount> wheels{};

    // Legacy Step 9.1 laboratory force request. Kept only for the compatibility
    // helper below; DRIVE no longer uses it after Step 9.5.
    double drive_acceleration = 9.0;

    // Total wheel torque after the (not-yet-modelled) gearbox/final drive. It is
    // split across configured driven wheels, including airborne wheels; an
    // airborne wheel may spin but cannot create chassis force without road load.
    double maximum_drive_torque_nm = 2200.0;
    double maximum_brake_torque_nm_per_wheel = 1200.0;
    double longitudinal_slip_reference_speed_mps = 2.0;

    // Temporary Step 9 steering model. These values feed a bicycle-model yaw
    // target until lateral tire forces replace direct yaw control. Keeping them
    // vehicle-specific avoids burying handling behaviour in the app layer.
    double maximum_steer_angle_radians = 0.42;
    double provisional_max_lateral_acceleration = 8.5;
};

struct WheelContactState {
    bool driven = false;
    bool steerable = false;
    bool in_contact = false;
    math::Vec3d mount_world{};
    math::Vec3d wheel_center_world{};
    math::Vec3d probe_end_world{};
    math::Vec3d contact_point{};
    math::Vec3d contact_normal{0.0, 0.0, 1.0};
    double probe_distance = 0.0;
    double suspension_length = 0.0;
    double compression = 0.0;
    double compression_velocity = 0.0;
    double spring_force = 0.0;
    double damper_force = 0.0;
    double normal_force = 0.0;

    // Step 9.4/9.5 wheel angular/tire state, published with the contact snapshot
    // so rendering and telemetry never read mutable physics-thread state.
    double wheel_angular_velocity_rad_s = 0.0;
    double wheel_rotation_radians = 0.0;
    math::Vec3d longitudinal_direction_world{};
    double longitudinal_ground_speed_mps = 0.0;
    double circumferential_speed_mps = 0.0;
    double slip_ratio = 0.0;
    double longitudinal_tire_force_n = 0.0;
    double drive_torque_nm = 0.0;
    double brake_torque_nm = 0.0;
};

struct VehicleContactState {
    std::array<WheelContactState, kWheelCount> wheels{};
    std::uint32_t wheels_in_contact = 0;
    std::uint32_t driven_wheels = 0;
    std::uint32_t driven_wheels_in_contact = 0;
    math::Vec3d average_contact_normal{0.0, 0.0, 1.0};
};

// Persistent per-vehicle suspension state owned by the physics execution path.
// Keeping previous lengths here makes damper velocity deterministic and avoids
// coupling suspension history to render snapshots.
struct VehicleRuntimeState {
    VehicleContactState contacts{};
    std::array<double, kWheelCount> previous_lengths{};
    std::array<double, kWheelCount> wheel_angular_velocity_rad_s{};
    std::array<double, kWheelCount> wheel_rotation_radians{};
    bool initialized = false;
};

struct LongitudinalTireResult {
    double total_longitudinal_force_n = 0.0;
    double total_drive_torque_nm = 0.0;
    double total_brake_torque_nm = 0.0;
};

[[nodiscard]] constexpr std::size_t wheel_index(const WheelPosition position) noexcept {
    return static_cast<std::size_t>(position);
}

[[nodiscard]] constexpr bool wheel_is_driven(const DriveLayout layout,
                                              const WheelPosition position) noexcept {
    switch (layout) {
    case DriveLayout::front_wheel_drive:
        return position == WheelPosition::front_left || position == WheelPosition::front_right;
    case DriveLayout::rear_wheel_drive:
        return position == WheelPosition::rear_left || position == WheelPosition::rear_right;
    case DriveLayout::all_wheel_drive:
        return true;
    }
    return false;
}


[[nodiscard]] double vehicle_wheelbase(const VehicleConfig& config) noexcept;

// Returns the temporary bicycle-model yaw-rate target used before the real
// lateral tire model lands. At zero longitudinal speed the target is zero, so
// steering cannot rotate the vehicle in place. The result is also capped by a
// provisional lateral-acceleration envelope to avoid absurd yaw rates at speed.
[[nodiscard]] double provisional_steering_yaw_rate(
    const VehicleConfig& config,
    double longitudinal_speed_mps,
    double steering_input) noexcept;

[[nodiscard]] double minimum_suspension_length(const WheelConfig& wheel) noexcept;
[[nodiscard]] double maximum_suspension_length(const WheelConfig& wheel) noexcept;

// Samples suspension rays and advances persistent compression/damper state.
// A hit is considered suspension contact only while it is within configured
// travel. Wheel centre therefore follows real suspension length rather than a
// fixed body-local offset.
[[nodiscard]] VehicleContactState update_suspension_contacts(
    const physics::PhysicsWorld& physics,
    const physics::RigidBody& body,
    const VehicleConfig& config,
    VehicleRuntimeState& runtime,
    double dt_seconds) noexcept;

// Applies spring + damper support at each contacted wheel station. The force is
// one-sided (never pulls the chassis toward the road) and follows the authored
// track normal so ramps naturally pitch/roll the chassis.
[[nodiscard]] double apply_suspension_forces(
    physics::PhysicsWorld& physics,
    physics::RigidBodyHandle body_handle,
    const VehicleConfig& config,
    VehicleRuntimeState& runtime) noexcept;


// Dimensionless SAE-style longitudinal slip: positive when tread speed exceeds
// road speed (driving), negative when the wheel is slower than the road (braking).
// A finite low-speed reference keeps the ratio well behaved around standstill.
[[nodiscard]] double longitudinal_slip_ratio(
    double circumferential_speed_mps,
    double ground_speed_mps,
    double reference_speed_mps) noexcept;

// Smooth linear-to-saturated longitudinal tire curve. The low-slip slope is the
// configured stiffness and the peak is load dependent: |Fx| <= mu * Fz.
[[nodiscard]] double longitudinal_tire_force(
    const WheelConfig& wheel,
    double slip_ratio,
    double normal_force_n) noexcept;

// Advances all four wheel angular states and applies their longitudinal tire
// forces at real road contacts. Drive torque is sent only to configured driven
// wheels; airborne wheels can spin but generate zero chassis force. Basic brake
// torque is included here because braking must act through wheel slip once wheel
// rotational inertia exists.
[[nodiscard]] LongitudinalTireResult update_longitudinal_tire_dynamics(
    physics::PhysicsWorld& physics,
    physics::RigidBodyHandle body_handle,
    const physics::RigidBody& body,
    const VehicleConfig& config,
    VehicleRuntimeState& runtime,
    double throttle,
    double brake,
    double boost,
    double steering_input,
    double dt_seconds) noexcept;

// Step 9.1 compatibility helper: stateless contact sampling. New vehicle code
// should prefer update_suspension_contacts().
[[nodiscard]] VehicleContactState sample_wheel_contacts(
    const physics::PhysicsWorld& physics,
    const physics::RigidBody& body,
    const VehicleConfig& config) noexcept;

// Applies a simple per-driven-wheel longitudinal force at each real track contact.
// Requested force is split by configured driven-wheel count; an airborne driven
// wheel contributes zero rather than redistributing its share to wheels on ground.
[[nodiscard]] double apply_driven_wheel_force(
    physics::PhysicsWorld& physics,
    physics::RigidBodyHandle body_handle,
    const physics::RigidBody& body,
    const VehicleConfig& config,
    const VehicleContactState& contacts,
    double throttle,
    double boost = 1.0) noexcept;

} // namespace ocs::vehicle
