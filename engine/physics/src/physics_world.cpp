#include "ocs/physics/physics_world.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace ocs::physics {
namespace {

constexpr double kMinimumMass = 1.0e-6;
constexpr float kMinimumInertia = 1.0e-6F;
constexpr double kContactSlop = 1.0e-4;
constexpr double kPositionCorrection = 0.90;
constexpr std::uint32_t kContactIterations = 8U;

math::Vec3f multiply_components(const math::Vec3f a, const math::Vec3f b) noexcept {
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}

math::Vec3d lerp(const math::Vec3d a, const math::Vec3d b, const double alpha) noexcept {
    return a * (1.0 - alpha) + b * alpha;
}

math::Vec3f lerp(const math::Vec3f a, const math::Vec3f b, const float alpha) noexcept {
    return a * (1.0F - alpha) + b * alpha;
}

math::Vec3d angular_velocity_at_point(const RigidBody& body, const math::Vec3d lever) noexcept {
    return math::Vec3d::from_vec3f(body.current.angular_velocity).cross(lever);
}

} // namespace

MassProperties PhysicsWorld::build_mass_properties(const RigidBodyDesc& desc) noexcept {
    MassProperties result{};
    if (!desc.dynamic) {
        result.mass = 0.0;
        result.inverse_mass = 0.0;
        result.inertia_diagonal = desc.inertia_diagonal;
        result.inverse_inertia_diagonal = {};
        return result;
    }

    result.mass = std::max(desc.mass, kMinimumMass);
    result.inverse_mass = 1.0 / result.mass;
    result.inertia_diagonal = {
        std::max(desc.inertia_diagonal.x, kMinimumInertia),
        std::max(desc.inertia_diagonal.y, kMinimumInertia),
        std::max(desc.inertia_diagonal.z, kMinimumInertia)
    };
    result.inverse_inertia_diagonal = {
        1.0F / result.inertia_diagonal.x,
        1.0F / result.inertia_diagonal.y,
        1.0F / result.inertia_diagonal.z
    };
    return result;
}

RigidBodyHandle PhysicsWorld::create_body(const RigidBodyDesc& desc) {
    RigidBody body{};
    body.previous = desc.state;
    body.current = desc.state;
    body.current.orientation = body.current.orientation.normalized();
    body.previous.orientation = body.current.orientation;
    body.mass = build_mass_properties(desc);
    body.dynamic = desc.dynamic;

    body.ground_contact_enabled = desc.ground_contact_enabled;
    body.ground_clearance = desc.ground_clearance;
    body.restitution = std::clamp(desc.restitution, 0.0, 1.0);
    body.ground_friction = std::max(desc.ground_friction, 0.0);

    body.static_mesh_contact_enabled = desc.static_mesh_contact_enabled;
    body.collision_box = desc.collision_box;
    body.collision_box.half_extents = {
        std::max(std::abs(body.collision_box.half_extents.x), 1.0e-4F),
        std::max(std::abs(body.collision_box.half_extents.y), 1.0e-4F),
        std::max(std::abs(body.collision_box.half_extents.z), 1.0e-4F)
    };
    body.friction_coefficient = std::max(desc.friction_coefficient, 0.0);
    body.linear_damping = std::max(desc.linear_damping, 0.0);
    body.angular_damping = std::max(desc.angular_damping, 0.0);
    return bodies_.emplace(body);
}

bool PhysicsWorld::destroy_body(const RigidBodyHandle handle) noexcept {
    return bodies_.erase(handle);
}

RigidBody* PhysicsWorld::get(const RigidBodyHandle handle) noexcept {
    return bodies_.get(handle);
}

const RigidBody* PhysicsWorld::get(const RigidBodyHandle handle) const noexcept {
    return bodies_.get(handle);
}

void PhysicsWorld::set_static_triangles(std::vector<StaticTriangle> triangles) {
    static_triangles_.clear();
    static_triangles_.reserve(triangles.size());

    for (const StaticTriangle& source : triangles) {
        const math::Vec3d cross = (source.b - source.a).cross(source.c - source.a);
        const double magnitude = cross.length();
        if (!(magnitude > 1.0e-10) || !std::isfinite(magnitude)) {
            continue;
        }
        static_triangles_.push_back({
            .a = source.a,
            .b = source.b,
            .c = source.c,
            .normal = cross / magnitude
        });
    }
}

std::optional<StaticRaycastHit> PhysicsWorld::raycast_static(
    const math::Vec3d origin,
    const math::Vec3d direction,
    const double max_distance) const noexcept {
    if (!(max_distance > 0.0) || !std::isfinite(max_distance)) {
        return std::nullopt;
    }

    const double direction_length = direction.length();
    if (!(direction_length > 1.0e-10) || !std::isfinite(direction_length)) {
        return std::nullopt;
    }
    const math::Vec3d ray = direction / direction_length;

    std::optional<StaticRaycastHit> closest;
    constexpr double kParallelEpsilon = 1.0e-9;
    for (std::size_t index = 0; index < static_triangles_.size(); ++index) {
        const CollisionTriangle& triangle = static_triangles_[index];
        const double denominator = ray.dot(triangle.normal);

        // Static collision is one-sided. A suspension ray must approach the
        // authored front face (+Z floor, ramp normal, inward wall normal).
        if (denominator >= -kParallelEpsilon) {
            continue;
        }

        const double distance = (triangle.a - origin).dot(triangle.normal) / denominator;
        if (distance < 0.0 || distance > max_distance) {
            continue;
        }
        if (closest.has_value() && distance >= closest->distance) {
            continue;
        }

        const math::Vec3d point = origin + ray * distance;
        if (!point_in_triangle(point, triangle)) {
            continue;
        }

        closest = StaticRaycastHit{
            .point = point,
            .normal = triangle.normal,
            .distance = distance,
            .triangle_index = index
        };
    }
    return closest;
}

void PhysicsWorld::add_force(const RigidBodyHandle handle, const math::Vec3d force) noexcept {
    if (RigidBody* body = get(handle); body != nullptr && body->dynamic) {
        body->accumulated_force += force;
    }
}

void PhysicsWorld::add_torque(const RigidBodyHandle handle, const math::Vec3f torque) noexcept {
    if (RigidBody* body = get(handle); body != nullptr && body->dynamic) {
        body->accumulated_torque = body->accumulated_torque + torque;
    }
}

void PhysicsWorld::add_force_at_point(const RigidBodyHandle handle,
                                      const math::Vec3d force,
                                      const math::Vec3d world_point) noexcept {
    RigidBody* body = get(handle);
    if (body == nullptr || !body->dynamic) {
        return;
    }

    body->accumulated_force += force;
    const math::Vec3d lever = world_point - body->current.position;
    const math::Vec3d torque = lever.cross(force);
    body->accumulated_torque = body->accumulated_torque + torque.to_vec3f();
}

bool PhysicsWorld::set_pose(const RigidBodyHandle handle,
                            const math::Vec3d position,
                            const math::Quatf orientation,
                            const bool reset_motion) noexcept {
    RigidBody* body = get(handle);
    if (body == nullptr) {
        return false;
    }

    body->current.position = position;
    body->current.orientation = orientation.normalized();
    body->previous.position = body->current.position;
    body->previous.orientation = body->current.orientation;
    if (reset_motion) {
        body->current.linear_velocity = {};
        body->current.linear_acceleration = {};
        body->current.angular_velocity = {};
        body->current.angular_acceleration = {};
        body->previous.linear_velocity = {};
        body->previous.linear_acceleration = {};
        body->previous.angular_velocity = {};
        body->previous.angular_acceleration = {};
    }
    body->accumulated_force = {};
    body->accumulated_torque = {};
    body->last_external_force = {};
    body->last_external_torque = {};
    body->grounded = false;
    body->contact_count = 0;
    body->debug_contact_count = 0;
    body->support_normal = {0.0, 0.0, 1.0};
    return true;
}

bool PhysicsWorld::reset_state(const RigidBodyHandle handle, const RigidBodyState& state) noexcept {
    RigidBody* body = get(handle);
    if (body == nullptr) {
        return false;
    }
    body->current = state;
    body->current.orientation = body->current.orientation.normalized();
    body->previous = body->current;
    body->accumulated_force = {};
    body->accumulated_torque = {};
    body->last_external_force = {};
    body->last_external_torque = {};
    body->grounded = false;
    body->contact_count = 0;
    body->debug_contact_count = 0;
    body->support_normal = {0.0, 0.0, 1.0};
    return true;
}

math::Vec3f PhysicsWorld::angular_acceleration_world(const RigidBody& body) noexcept {
    const math::Quatf orientation = body.current.orientation.normalized();
    const math::Quatf inverse = orientation.conjugate();

    const math::Vec3f omega_body = inverse.rotate(body.current.angular_velocity);
    const math::Vec3f torque_body = inverse.rotate(body.accumulated_torque);
    const math::Vec3f angular_momentum_body = multiply_components(
        body.mass.inertia_diagonal,
        omega_body);
    const math::Vec3f gyroscopic = omega_body.cross(angular_momentum_body);
    const math::Vec3f net_body = torque_body - gyroscopic;
    const math::Vec3f acceleration_body = multiply_components(
        body.mass.inverse_inertia_diagonal,
        net_body);
    return orientation.rotate(acceleration_body);
}

void PhysicsWorld::integrate_orientation(RigidBody& body, const double dt_seconds) noexcept {
    const math::Vec3f omega = body.current.angular_velocity;
    const math::Quatf omega_quat{omega.x, omega.y, omega.z, 0.0F};
    const math::Quatf derivative = omega_quat * body.current.orientation;
    const float half_dt = static_cast<float>(0.5 * dt_seconds);

    body.current.orientation = math::Quatf{
        body.current.orientation.x + derivative.x * half_dt,
        body.current.orientation.y + derivative.y * half_dt,
        body.current.orientation.z + derivative.z * half_dt,
        body.current.orientation.w + derivative.w * half_dt
    }.normalized();
}

void PhysicsWorld::solve_ground_plane(RigidBody& body, const double dt_seconds) noexcept {
    if (!body.ground_contact_enabled) {
        return;
    }

    if (body.current.position.z >= body.ground_clearance) {
        return;
    }

    body.current.position.z = body.ground_clearance;
    if (body.current.linear_velocity.z < 0.0) {
        body.current.linear_velocity.z = -body.current.linear_velocity.z * body.restitution;
        if (std::abs(body.current.linear_velocity.z) < 0.02) {
            body.current.linear_velocity.z = 0.0;
        }
    }

    const double friction_factor = std::max(0.0, 1.0 - body.ground_friction * dt_seconds);
    body.current.linear_velocity.x *= friction_factor;
    body.current.linear_velocity.y *= friction_factor;
    body.grounded = true;
    body.contact_count = 1;
    body.support_normal = {0.0, 0.0, 1.0};
    body.last_contact_point = {body.current.position.x, body.current.position.y, body.ground_clearance};
    body.last_contact_normal = body.support_normal;
    body.debug_contact_count = 1U;
    body.debug_contacts[0] = {body.last_contact_point, body.last_contact_normal};
}

void PhysicsWorld::apply_damping(RigidBody& body, const double dt_seconds) noexcept {
    const double linear_factor = std::exp(-body.linear_damping * dt_seconds);
    const float angular_factor = static_cast<float>(std::exp(-body.angular_damping * dt_seconds));
    body.current.linear_velocity *= linear_factor;
    body.current.angular_velocity = body.current.angular_velocity * angular_factor;
}

math::Vec3d PhysicsWorld::inverse_inertia_world(const RigidBody& body,
                                                const math::Vec3d value) noexcept {
    const math::Quatf orientation = body.current.orientation.normalized();
    const math::Vec3f body_value = orientation.conjugate().rotate(value.to_vec3f());
    const math::Vec3f body_result = multiply_components(body.mass.inverse_inertia_diagonal, body_value);
    return math::Vec3d::from_vec3f(orientation.rotate(body_result));
}

void PhysicsWorld::apply_impulse(RigidBody& body,
                                 const math::Vec3d impulse,
                                 const math::Vec3d lever) noexcept {
    body.current.linear_velocity += impulse * body.mass.inverse_mass;
    const math::Vec3d angular_impulse = inverse_inertia_world(body, lever.cross(impulse));
    body.current.angular_velocity = body.current.angular_velocity + angular_impulse.to_vec3f();
}

bool PhysicsWorld::point_in_triangle(const math::Vec3d point,
                                     const CollisionTriangle& triangle) noexcept {
    const math::Vec3d v0 = triangle.b - triangle.a;
    const math::Vec3d v1 = triangle.c - triangle.a;
    const math::Vec3d v2 = point - triangle.a;
    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);
    const double denominator = d00 * d11 - d01 * d01;
    if (std::abs(denominator) <= 1.0e-14) {
        return false;
    }

    const double v = (d11 * d20 - d01 * d21) / denominator;
    const double w = (d00 * d21 - d01 * d20) / denominator;
    const double u = 1.0 - v - w;
    constexpr double tolerance = -1.0e-6;
    return u >= tolerance && v >= tolerance && w >= tolerance;
}

PhysicsWorld::ContactCandidate PhysicsWorld::deepest_static_contact(RigidBody& body) noexcept {
    ContactCandidate deepest{};
    const math::Vec3f center = body.collision_box.center_local;
    const math::Vec3f half = body.collision_box.half_extents;
    const double collision_radius = std::sqrt(
        static_cast<double>(half.x * half.x + half.y * half.y + half.z * half.z));
    const double maximum_penetration = std::max(0.5, collision_radius * 1.25);
    constexpr std::array<float, 2> signs{-1.0F, 1.0F};
    std::uint8_t corner_index = 0;

    for (const float sx : signs) {
        for (const float sy : signs) {
            for (const float sz : signs) {
                const math::Vec3f local_corner{
                    center.x + sx * half.x,
                    center.y + sy * half.y,
                    center.z + sz * half.z
                };
                const math::Vec3d corner = body.current.position +
                    math::Vec3d::from_vec3f(body.current.orientation.rotate(local_corner));

                for (const CollisionTriangle& triangle : static_triangles_) {
                    ++last_step_stats_.triangle_tests;
                    const double signed_distance = (corner - triangle.a).dot(triangle.normal);
                    if (signed_distance >= -kContactSlop || -signed_distance > maximum_penetration) {
                        continue;
                    }

                    const math::Vec3d projected = corner - triangle.normal * signed_distance;
                    if (!point_in_triangle(projected, triangle)) {
                        continue;
                    }

                    const double penetration = -signed_distance;
                    if (!deepest.valid || penetration > deepest.penetration) {
                        deepest = {
                            .valid = true,
                            .point = projected,
                            .normal = triangle.normal,
                            .penetration = penetration,
                            .corner_index = corner_index
                        };
                    }
                }
                ++corner_index;
            }
        }
    }
    return deepest;
}

void PhysicsWorld::solve_static_mesh(RigidBody& body, const double dt_seconds) noexcept {
    if (!body.static_mesh_contact_enabled || static_triangles_.empty()) {
        return;
    }

    math::Vec3d support_sum{};
    std::uint32_t solved_contacts = 0;
    std::uint8_t unique_corner_mask = 0U;
    std::uint32_t unique_contacts = 0U;

    for (std::uint32_t iteration = 0; iteration < kContactIterations; ++iteration) {
        const ContactCandidate contact = deepest_static_contact(body);
        if (!contact.valid) {
            break;
        }

        const double correction = std::max(0.0, contact.penetration - kContactSlop) * kPositionCorrection;
        body.current.position += contact.normal * correction;

        // After positional correction, use the actual surface point as the lever arm.
        const math::Vec3d lever = contact.point - body.current.position;
        math::Vec3d contact_velocity = body.current.linear_velocity + angular_velocity_at_point(body, lever);
        const double normal_speed = contact_velocity.dot(contact.normal);

        double normal_impulse_magnitude = 0.0;
        if (normal_speed < 0.0) {
            const math::Vec3d rxn = lever.cross(contact.normal);
            const math::Vec3d inertial = inverse_inertia_world(body, rxn).cross(lever);
            const double effective_mass = body.mass.inverse_mass + contact.normal.dot(inertial);
            if (effective_mass > 1.0e-10) {
                // Suppress tiny resting bounces while retaining restitution for real impacts.
                const double restitution = normal_speed < -1.0 ? body.restitution : 0.0;
                normal_impulse_magnitude = -(1.0 + restitution) * normal_speed / effective_mass;
                apply_impulse(body, contact.normal * normal_impulse_magnitude, lever);
            }
        }

        // Coulomb-style kinetic/static approximation at the same point. This is
        // intentionally small and deterministic; tire friction remains Step 9 work.
        contact_velocity = body.current.linear_velocity + angular_velocity_at_point(body, lever);
        const double velocity_along_normal = contact_velocity.dot(contact.normal);
        const math::Vec3d tangent_velocity = contact_velocity - contact.normal * velocity_along_normal;
        const double tangent_speed = tangent_velocity.length();
        if (tangent_speed > 1.0e-8 && normal_impulse_magnitude > 0.0) {
            const math::Vec3d tangent = tangent_velocity / tangent_speed;
            const math::Vec3d rxt = lever.cross(tangent);
            const math::Vec3d inertial = inverse_inertia_world(body, rxt).cross(lever);
            const double effective_mass = body.mass.inverse_mass + tangent.dot(inertial);
            if (effective_mass > 1.0e-10) {
                double tangent_impulse = -contact_velocity.dot(tangent) / effective_mass;
                const double limit = body.friction_coefficient * normal_impulse_magnitude;
                tangent_impulse = std::clamp(tangent_impulse, -limit, limit);
                apply_impulse(body, tangent * tangent_impulse, lever);
            }
        }

        ++solved_contacts;
        const std::uint8_t corner_bit = static_cast<std::uint8_t>(1U << contact.corner_index);
        if ((unique_corner_mask & corner_bit) == 0U) {
            unique_corner_mask = static_cast<std::uint8_t>(unique_corner_mask | corner_bit);
            if (unique_contacts < body.debug_contacts.size()) {
                body.debug_contacts[unique_contacts] = {contact.point, contact.normal};
            }
            ++unique_contacts;
            support_sum += contact.normal;
        }
        body.last_contact_point = contact.point;
        body.last_contact_normal = contact.normal;
    }

    if (solved_contacts > 0U) {
        body.contact_count = unique_contacts;
        body.debug_contact_count = std::min<std::uint32_t>(
            unique_contacts, static_cast<std::uint32_t>(body.debug_contacts.size()));
        const math::Vec3d support = support_sum.normalized();
        body.support_normal = support.length_squared() > 0.0 ? support : math::Vec3d{0.0, 0.0, 1.0};
        body.grounded = body.support_normal.z > 0.35;

    }

    (void)dt_seconds;
}

void PhysicsWorld::step(const double dt_seconds) noexcept {
    if (!(dt_seconds > 0.0) || !std::isfinite(dt_seconds)) {
        return;
    }

    last_step_stats_ = {
        .body_count = bodies_.size(),
        .static_triangle_count = static_triangles_.size(),
        .triangle_tests = 0,
        .contacts = 0
    };

    bodies_.for_each([&](const RigidBodyHandle, RigidBody& body) {
        body.previous = body.current;
        body.grounded = false;
        body.contact_count = 0;
        body.debug_contact_count = 0;
        body.support_normal = {0.0, 0.0, 1.0};
        body.last_external_force = body.accumulated_force;
        body.last_external_torque = body.accumulated_torque;

        if (!body.dynamic) {
            body.current.linear_acceleration = {};
            body.current.angular_acceleration = {};
            body.accumulated_force = {};
            body.accumulated_torque = {};
            return;
        }

        body.current.linear_acceleration = gravity_ + body.accumulated_force * body.mass.inverse_mass;
        body.current.angular_acceleration = angular_acceleration_world(body);

        body.current.linear_velocity += body.current.linear_acceleration * dt_seconds;
        body.current.position += body.current.linear_velocity * dt_seconds;

        body.current.angular_velocity = body.current.angular_velocity +
            body.current.angular_acceleration * static_cast<float>(dt_seconds);
        integrate_orientation(body, dt_seconds);
        apply_damping(body, dt_seconds);

        solve_static_mesh(body, dt_seconds);
        solve_ground_plane(body, dt_seconds);
        last_step_stats_.contacts += body.contact_count;

        body.accumulated_force = {};
        body.accumulated_torque = {};
    });
}

RigidBodyState PhysicsWorld::interpolated_state(const RigidBodyHandle handle,
                                                const double alpha) const noexcept {
    const RigidBody* body = get(handle);
    if (body == nullptr) {
        return {};
    }

    const double clamped = std::clamp(alpha, 0.0, 1.0);
    const float clamped_f = static_cast<float>(clamped);
    return {
        .position = lerp(body->previous.position, body->current.position, clamped),
        .orientation = math::Quatf::normalized_lerp(
            body->previous.orientation,
            body->current.orientation,
            clamped_f),
        .linear_velocity = lerp(body->previous.linear_velocity, body->current.linear_velocity, clamped),
        .linear_acceleration = lerp(body->previous.linear_acceleration, body->current.linear_acceleration, clamped),
        .angular_velocity = lerp(body->previous.angular_velocity, body->current.angular_velocity, clamped_f),
        .angular_acceleration = lerp(body->previous.angular_acceleration, body->current.angular_acceleration, clamped_f)
    };
}

} // namespace ocs::physics
