#include "ocs/vehicle/vehicle.hpp"

#include <algorithm>
#include <cmath>

namespace ocs::vehicle {
namespace {

[[nodiscard]] WheelPosition wheel_position_from_index(const std::size_t index) noexcept {
    return static_cast<WheelPosition>(index);
}

[[nodiscard]] math::Vec3d suspension_down(const physics::RigidBody& body) noexcept {
    return -math::Vec3d::from_vec3f(
        body.current.orientation.rotate({0.0F, 0.0F, 1.0F})).normalized();
}

[[nodiscard]] math::Vec3d wheel_forward_world(const physics::RigidBody& body,
                                               const WheelConfig& wheel,
                                               const VehicleConfig& config,
                                               const double steering_input) noexcept {
    math::Vec3f local_forward{1.0F, 0.0F, 0.0F};
    if (wheel.steerable) {
        const float angle = static_cast<float>(
            std::clamp(steering_input, -1.0, 1.0) *
            std::max(0.0, config.maximum_steer_angle_radians));
        local_forward = math::Quatf::from_axis_angle({0.0F, 0.0F, 1.0F}, angle)
                            .rotate(local_forward);
    }
    return math::Vec3d::from_vec3f(body.current.orientation.rotate(local_forward)).normalized();
}

[[nodiscard]] double sign_nonzero(const double value) noexcept {
    return value > 0.0 ? 1.0 : (value < 0.0 ? -1.0 : 0.0);
}

[[nodiscard]] VehicleContactState sample_impl(const physics::PhysicsWorld& physics,
                                              const physics::RigidBody& body,
                                              const VehicleConfig& config,
                                              VehicleRuntimeState* runtime,
                                              const double dt_seconds) noexcept {
    VehicleContactState state{};
    const math::Vec3d down = suspension_down(body);
    math::Vec3d normal_sum{};

    for (std::size_t index = 0; index < config.wheels.size(); ++index) {
        const WheelConfig& wheel = config.wheels[index];
        WheelContactState& contact = state.wheels[index];
        const WheelPosition position = wheel_position_from_index(index);
        contact.driven = wheel_is_driven(config.drive_layout, position);
        contact.steerable = wheel.steerable;
        if (contact.driven) {
            ++state.driven_wheels;
        }

        const double min_length = minimum_suspension_length(wheel);
        const double max_length = maximum_suspension_length(wheel);
        contact.suspension_length = std::clamp(wheel.rest_length, min_length, max_length);
        contact.mount_world = body.current.position + math::Vec3d::from_vec3f(
            body.current.orientation.rotate(wheel.suspension_mount_local));
        const double probe_length = std::max(0.0, wheel.radius + max_length);
        contact.probe_end_world = contact.mount_world + down * probe_length;
        if (runtime != nullptr) {
            contact.wheel_angular_velocity_rad_s = runtime->wheel_angular_velocity_rad_s[index];
            contact.wheel_rotation_radians = runtime->wheel_rotation_radians[index];
        }

        const auto hit = physics.raycast_static(contact.mount_world, down, probe_length);
        if (!hit.has_value()) {
            contact.suspension_length = max_length;
            contact.wheel_center_world = contact.mount_world + down * max_length;
            if (runtime != nullptr) {
                runtime->previous_lengths[index] = max_length;
            }
            continue;
        }

        const double raw_length = hit->distance - std::max(0.0, wheel.radius);
        contact.suspension_length = std::clamp(raw_length, min_length, max_length);
        contact.wheel_center_world = contact.mount_world + down * contact.suspension_length;

        // A hit beyond max droop is outside suspension reach. raycast max distance
        // already handles the usual case; this epsilon only guards numerical edge cases.
        if (raw_length > max_length + 1.0e-6) {
            if (runtime != nullptr) {
                runtime->previous_lengths[index] = max_length;
            }
            continue;
        }

        contact.in_contact = true;
        contact.contact_point = hit->point;
        contact.contact_normal = hit->normal;
        contact.probe_distance = hit->distance;
        contact.compression = std::max(0.0, wheel.rest_length - contact.suspension_length);

        if (runtime != nullptr && runtime->initialized && dt_seconds > 0.0) {
            // Positive when suspension is compressing (length decreasing).
            contact.compression_velocity =
                (runtime->previous_lengths[index] - contact.suspension_length) / dt_seconds;
        }

        contact.spring_force = std::max(0.0, wheel.spring_rate) * contact.compression;
        contact.damper_force = std::max(0.0, wheel.damper_rate) * contact.compression_velocity;
        contact.normal_force = std::max(0.0, contact.spring_force + contact.damper_force);

        ++state.wheels_in_contact;
        if (contact.driven) {
            ++state.driven_wheels_in_contact;
        }
        normal_sum += contact.contact_normal;
        if (runtime != nullptr) {
            runtime->previous_lengths[index] = contact.suspension_length;
        }
    }

    if (state.wheels_in_contact > 0U && normal_sum.length_squared() > 1.0e-12) {
        state.average_contact_normal = normal_sum.normalized();
    }
    return state;
}

} // namespace

double vehicle_wheelbase(const VehicleConfig& config) noexcept {
    double front_sum = 0.0;
    double rear_sum = 0.0;
    std::size_t front_count = 0U;
    std::size_t rear_count = 0U;

    for (std::size_t index = 0; index < config.wheels.size(); ++index) {
        const double x = static_cast<double>(config.wheels[index].suspension_mount_local.x);
        if (index == wheel_index(WheelPosition::front_left) ||
            index == wheel_index(WheelPosition::front_right)) {
            front_sum += x;
            ++front_count;
        } else {
            rear_sum += x;
            ++rear_count;
        }
    }

    if (front_count == 0U || rear_count == 0U) {
        return 0.0;
    }
    const double front_x = front_sum / static_cast<double>(front_count);
    const double rear_x = rear_sum / static_cast<double>(rear_count);
    return std::abs(front_x - rear_x);
}

double provisional_steering_yaw_rate(const VehicleConfig& config,
                                      const double longitudinal_speed_mps,
                                      const double steering_input) noexcept {
    const double wheelbase = vehicle_wheelbase(config);
    const double steer = std::clamp(steering_input, -1.0, 1.0);
    if (!(wheelbase > 1.0e-6) || std::abs(steer) <= 1.0e-9 ||
        std::abs(longitudinal_speed_mps) <= 1.0e-6) {
        return 0.0;
    }

    const double angle = steer * std::max(0.0, config.maximum_steer_angle_radians);
    double target = longitudinal_speed_mps * std::tan(angle) / wheelbase;

    const double max_lateral_acceleration =
        std::max(0.0, config.provisional_max_lateral_acceleration);
    if (max_lateral_acceleration > 0.0) {
        const double speed = std::abs(longitudinal_speed_mps);
        const double max_yaw_rate = max_lateral_acceleration / std::max(speed, 0.5);
        target = std::clamp(target, -max_yaw_rate, max_yaw_rate);
    }
    return target;
}

double minimum_suspension_length(const WheelConfig& wheel) noexcept {
    return std::max(0.0, wheel.rest_length - std::max(0.0, wheel.max_compression));
}

double maximum_suspension_length(const WheelConfig& wheel) noexcept {
    return std::max(minimum_suspension_length(wheel),
                    wheel.rest_length + std::max(0.0, wheel.max_droop));
}

VehicleContactState update_suspension_contacts(const physics::PhysicsWorld& physics,
                                               const physics::RigidBody& body,
                                               const VehicleConfig& config,
                                               VehicleRuntimeState& runtime,
                                               const double dt_seconds) noexcept {
    runtime.contacts = sample_impl(physics, body, config, &runtime, dt_seconds);
    runtime.initialized = true;
    return runtime.contacts;
}

double apply_suspension_forces(physics::PhysicsWorld& physics,
                               const physics::RigidBodyHandle body_handle,
                               const VehicleConfig& config,
                               VehicleRuntimeState& runtime) noexcept {
    double total_force = 0.0;
    for (std::size_t index = 0; index < config.wheels.size(); ++index) {
        WheelContactState& wheel = runtime.contacts.wheels[index];
        if (!wheel.in_contact || !(wheel.normal_force > 0.0)) {
            continue;
        }
        physics.add_force_at_point(
            body_handle,
            wheel.contact_normal * wheel.normal_force,
            wheel.wheel_center_world);
        total_force += wheel.normal_force;
    }
    return total_force;
}

double longitudinal_slip_ratio(const double circumferential_speed_mps,
                               const double ground_speed_mps,
                               const double reference_speed_mps) noexcept {
    const double reference = std::max(std::abs(reference_speed_mps), 1.0e-3);
    const double denominator = std::max({
        std::abs(circumferential_speed_mps),
        std::abs(ground_speed_mps),
        reference
    });
    return (circumferential_speed_mps - ground_speed_mps) / denominator;
}

double longitudinal_tire_force(const WheelConfig& wheel,
                               const double slip,
                               const double normal_force_n) noexcept {
    const double load = std::max(0.0, normal_force_n);
    const double peak = std::max(0.0, wheel.peak_friction_coefficient) * load;
    const double stiffness = std::max(0.0, wheel.longitudinal_stiffness);
    if (!(peak > 0.0) || !(stiffness > 0.0)) {
        return 0.0;
    }

    // tanh gives a smooth low-slip linear region while enforcing the load-based
    // friction limit without a discontinuous hard clamp.
    return peak * std::tanh((stiffness * slip) / peak);
}

LongitudinalTireResult update_longitudinal_tire_dynamics(
    physics::PhysicsWorld& physics,
    const physics::RigidBodyHandle body_handle,
    const physics::RigidBody& body,
    const VehicleConfig& config,
    VehicleRuntimeState& runtime,
    const double throttle,
    const double brake,
    const double boost,
    const double steering_input,
    const double dt_seconds) noexcept {

    LongitudinalTireResult result{};
    if (!(dt_seconds > 0.0)) {
        return result;
    }

    const double throttle_input = std::clamp(throttle, 0.0, 1.0);
    const double brake_input = std::clamp(brake, 0.0, 1.0);
    const double drive_boost = std::max(0.0, boost);
    const double total_drive_torque =
        std::max(0.0, config.maximum_drive_torque_nm) * throttle_input * drive_boost;
    const double drive_torque_per_wheel = runtime.contacts.driven_wheels > 0U
        ? total_drive_torque / static_cast<double>(runtime.contacts.driven_wheels)
        : 0.0;

    const math::Vec3d chassis_omega = math::Vec3d::from_vec3f(body.current.angular_velocity);

    for (std::size_t index = 0; index < config.wheels.size(); ++index) {
        const WheelConfig& wheel = config.wheels[index];
        WheelContactState& contact = runtime.contacts.wheels[index];
        const double radius = std::max(wheel.radius, 1.0e-4);
        const double inertia = std::max(wheel.rotational_inertia, 1.0e-4);
        double& angular_velocity = runtime.wheel_angular_velocity_rad_s[index];
        double& rotation_angle = runtime.wheel_rotation_radians[index];

        const double drive_torque = contact.driven ? drive_torque_per_wheel : 0.0;
        double road_speed = 0.0;
        double tire_force = 0.0;
        math::Vec3d tangent{};

        if (contact.in_contact) {
            tangent = wheel_forward_world(body, wheel, config, steering_input);
            tangent -= contact.contact_normal * tangent.dot(contact.contact_normal);
            const double tangent_length = tangent.length();
            if (tangent_length > 1.0e-8) {
                tangent = tangent / tangent_length;
                const math::Vec3d lever = contact.contact_point - body.current.position;
                const math::Vec3d point_velocity =
                    body.current.linear_velocity + chassis_omega.cross(lever);
                contact.longitudinal_direction_world = tangent;
                road_speed = point_velocity.dot(tangent);
                const double surface_speed = angular_velocity * radius;
                contact.longitudinal_ground_speed_mps = road_speed;
                contact.circumferential_speed_mps = surface_speed;
                contact.slip_ratio = longitudinal_slip_ratio(
                    surface_speed, road_speed, config.longitudinal_slip_reference_speed_mps);
                tire_force = longitudinal_tire_force(wheel, contact.slip_ratio, contact.normal_force);
                contact.longitudinal_tire_force_n = tire_force;
                physics.add_force_at_point(body_handle, tangent * tire_force, contact.contact_point);
                result.total_longitudinal_force_n += tire_force;
            }
        } else {
            contact.longitudinal_direction_world = {};
            contact.longitudinal_ground_speed_mps = 0.0;
            contact.circumferential_speed_mps = angular_velocity * radius;
            contact.slip_ratio = 0.0;
            contact.longitudinal_tire_force_n = 0.0;
        }

        // Basic service-brake torque is needed now that wheel inertia exists; the
        // final brake balance/ABS model remains a later Step 9 concern. Near zero
        // wheel speed, road speed selects the opposing sign so brakes do not
        // accidentally accelerate a slowly rolling wheel.
        double brake_reference = angular_velocity;
        if (std::abs(brake_reference) < 0.05) {
            brake_reference = road_speed / radius;
        }
        const double brake_torque = -sign_nonzero(brake_reference) *
            std::max(0.0, config.maximum_brake_torque_nm_per_wheel) * brake_input;

        const double reaction_torque = -tire_force * radius;
        const double net_torque = drive_torque + brake_torque + reaction_torque;
        angular_velocity += (net_torque / inertia) * dt_seconds;

        // Do not let brake torque alone numerically reverse a nearly stopped wheel.
        if (brake_input > 0.0 && std::abs(road_speed) < 0.05 &&
            sign_nonzero(brake_reference) != 0.0 &&
            sign_nonzero(angular_velocity) != sign_nonzero(brake_reference)) {
            angular_velocity = 0.0;
        }

        rotation_angle = std::remainder(
            rotation_angle + angular_velocity * dt_seconds,
            6.28318530717958647692);

        contact.wheel_angular_velocity_rad_s = angular_velocity;
        contact.wheel_rotation_radians = rotation_angle;
        contact.circumferential_speed_mps = angular_velocity * radius;
        contact.drive_torque_nm = drive_torque;
        contact.brake_torque_nm = brake_torque;
        result.total_drive_torque_nm += std::abs(drive_torque);
        result.total_brake_torque_nm += std::abs(brake_torque);
    }

    return result;
}

VehicleContactState sample_wheel_contacts(const physics::PhysicsWorld& physics,
                                          const physics::RigidBody& body,
                                          const VehicleConfig& config) noexcept {
    return sample_impl(physics, body, config, nullptr, 0.0);
}

double apply_driven_wheel_force(physics::PhysicsWorld& physics,
                                const physics::RigidBodyHandle body_handle,
                                const physics::RigidBody& body,
                                const VehicleConfig& config,
                                const VehicleContactState& contacts,
                                const double throttle,
                                const double boost) noexcept {
    const double input = std::clamp(throttle, 0.0, 1.0);
    if (!(input > 0.0) || contacts.driven_wheels == 0U || !(body.mass.mass > 0.0)) {
        return 0.0;
    }

    const math::Vec3d body_forward = math::Vec3d::from_vec3f(
        body.current.orientation.rotate({1.0F, 0.0F, 0.0F})).normalized();
    const double requested_total_force =
        body.mass.mass * std::max(0.0, config.drive_acceleration) * std::max(0.0, boost) * input;
    const double force_per_driven_wheel =
        requested_total_force / static_cast<double>(contacts.driven_wheels);

    double applied_total = 0.0;
    for (const WheelContactState& wheel : contacts.wheels) {
        if (!wheel.driven || !wheel.in_contact) {
            continue;
        }

        math::Vec3d tangent = body_forward - wheel.contact_normal * body_forward.dot(wheel.contact_normal);
        const double tangent_length = tangent.length();
        if (!(tangent_length > 1.0e-8)) {
            continue;
        }
        tangent = tangent / tangent_length;
        physics.add_force_at_point(body_handle, tangent * force_per_driven_wheel, wheel.contact_point);
        applied_total += force_per_driven_wheel;
    }
    return applied_total;
}

} // namespace ocs::vehicle
