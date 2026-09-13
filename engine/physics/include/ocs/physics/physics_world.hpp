#pragma once

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "ocs/core/slot_map.hpp"
#include "ocs/math/quatf.hpp"
#include "ocs/math/vec3d.hpp"
#include "ocs/math/vec3f.hpp"

namespace ocs::physics {

struct RigidBodyHandle {
    static constexpr std::uint32_t invalid_index = std::numeric_limits<std::uint32_t>::max();

    std::uint32_t index = invalid_index;
    std::uint32_t generation = 0;

    [[nodiscard]] constexpr bool valid() const noexcept { return index != invalid_index; }
    [[nodiscard]] constexpr explicit operator bool() const noexcept { return valid(); }
    auto operator<=>(const RigidBodyHandle&) const = default;
};

struct MassProperties {
    double mass = 1.0;
    double inverse_mass = 1.0;
    math::Vec3f inertia_diagonal{1.0F, 1.0F, 1.0F};
    math::Vec3f inverse_inertia_diagonal{1.0F, 1.0F, 1.0F};
};

struct RigidBodyState {
    math::Vec3d position{};
    math::Quatf orientation = math::Quatf::identity();
    math::Vec3d linear_velocity{};
    math::Vec3d linear_acceleration{};
    math::Vec3f angular_velocity{};
    math::Vec3f angular_acceleration{};
};

struct BoxCollisionShape {
    // Offset and half extents are body-local. Keeping the offset is important
    // because render assets are not required to have their origin at the AABB centre.
    math::Vec3f center_local{};
    math::Vec3f half_extents{0.5F, 0.5F, 0.5F};
};

struct StaticTriangle {
    math::Vec3d a{};
    math::Vec3d b{};
    math::Vec3d c{};
};


struct StaticRaycastHit {
    math::Vec3d point{};
    math::Vec3d normal{0.0, 0.0, 1.0};
    double distance = 0.0;
    std::size_t triangle_index = 0;
};

struct RigidBodyDesc {
    RigidBodyState state{};
    double mass = 1.0;
    math::Vec3f inertia_diagonal{1.0F, 1.0F, 1.0F};
    bool dynamic = true;

    // Legacy Step 8.4 infinite-plane contact. Kept for regression tests and
    // isolated experiments. Track collision should use static_mesh_contact_enabled.
    bool ground_contact_enabled = false;
    double ground_clearance = 0.0;
    double restitution = 0.05;
    double ground_friction = 3.0;

    // Step 8.8 oriented-box vs static-triangle-mesh laboratory collision.
    bool static_mesh_contact_enabled = false;
    BoxCollisionShape collision_box{};
    double friction_coefficient = 0.85;
    double linear_damping = 0.0;
    double angular_damping = 0.0;

    // Static-mesh stabilization. Large pre-existing overlaps are usually scenery
    // details or an invalid spawn and must not teleport a body out of geometry.
    double static_contact_max_penetration = 0.35;
    double static_contact_persistence_depth = 0.35;
    double upright_ground_contact_max_penetration = 0.12;
    double maximum_depenetration_speed = 3.0;
};

struct ContactDebugSample {
    math::Vec3d point{};
    math::Vec3d normal{0.0, 0.0, 1.0};
};

struct RigidBody {
    RigidBodyState previous{};
    RigidBodyState current{};
    MassProperties mass{};
    math::Vec3d accumulated_force{};
    math::Vec3f accumulated_torque{};
    math::Vec3d last_external_force{};
    math::Vec3f last_external_torque{};
    bool dynamic = true;

    bool ground_contact_enabled = false;
    double ground_clearance = 0.0;
    double restitution = 0.05;
    double ground_friction = 3.0;

    bool static_mesh_contact_enabled = false;
    BoxCollisionShape collision_box{};
    double friction_coefficient = 0.85;
    double linear_damping = 0.0;
    double angular_damping = 0.0;
    double static_contact_max_penetration = 0.35;
    double static_contact_persistence_depth = 0.35;
    double upright_ground_contact_max_penetration = 0.12;
    double maximum_depenetration_speed = 3.0;

    bool grounded = false;
    std::uint32_t contact_count = 0;
    math::Vec3d support_normal{0.0, 0.0, 1.0};
    math::Vec3d last_contact_point{};
    math::Vec3d last_contact_normal{0.0, 0.0, 1.0};
    std::array<ContactDebugSample, 8> debug_contacts{};
    std::uint32_t debug_contact_count = 0;
};

struct PhysicsStepStats {
    std::size_t body_count = 0;
    std::size_t static_triangle_count = 0;
    std::size_t static_bvh_node_count = 0;
    std::uint64_t bvh_node_visits = 0;
    std::uint64_t triangle_tests = 0;
    std::uint32_t contacts = 0;
};

class PhysicsWorld {
public:
    [[nodiscard]] RigidBodyHandle create_body(const RigidBodyDesc& desc);
    bool destroy_body(RigidBodyHandle handle) noexcept;

    [[nodiscard]] RigidBody* get(RigidBodyHandle handle) noexcept;
    [[nodiscard]] const RigidBody* get(RigidBodyHandle handle) const noexcept;

    void set_gravity(math::Vec3d gravity) noexcept { gravity_ = gravity; }
    [[nodiscard]] math::Vec3d gravity() const noexcept { return gravity_; }

    // Replaces the current static collision world. Degenerate triangles are
    // discarded and normals are derived from winding, so +Z floor / inward wall
    // winding from the asset compiler remains authoritative.
    void set_static_triangles(std::vector<StaticTriangle> triangles);
    void clear_static_triangles() noexcept;
    [[nodiscard]] std::size_t static_triangle_count() const noexcept { return static_triangles_.size(); }
    [[nodiscard]] std::size_t static_bvh_node_count() const noexcept { return static_bvh_nodes_.size(); }

    // Front-face static-mesh query used by wheel/suspension probes. Direction
    // need not be normalized; distance is reported in world metres.
    [[nodiscard]] std::optional<StaticRaycastHit> raycast_static(
        math::Vec3d origin,
        math::Vec3d direction,
        double max_distance) const noexcept;

    void add_force(RigidBodyHandle handle, math::Vec3d force) noexcept;
    void add_torque(RigidBodyHandle handle, math::Vec3f torque) noexcept;
    void add_force_at_point(RigidBodyHandle handle,
                            math::Vec3d force,
                            math::Vec3d world_point) noexcept;

    bool set_pose(RigidBodyHandle handle,
                  math::Vec3d position,
                  math::Quatf orientation,
                  bool reset_motion) noexcept;
    bool reset_state(RigidBodyHandle handle, const RigidBodyState& state) noexcept;

    void step(double dt_seconds) noexcept;

    [[nodiscard]] RigidBodyState interpolated_state(RigidBodyHandle handle,
                                                    double alpha) const noexcept;
    [[nodiscard]] const PhysicsStepStats& last_step_stats() const noexcept { return last_step_stats_; }
    [[nodiscard]] std::size_t body_count() const noexcept { return bodies_.size(); }

private:
    struct CollisionTriangle {
        math::Vec3d a{};
        math::Vec3d b{};
        math::Vec3d c{};
        math::Vec3d normal{};
        std::size_t source_index = 0;
    };

    struct Aabb {
        math::Vec3d minimum{};
        math::Vec3d maximum{};
    };

    struct StaticBvhNode {
        Aabb bounds{};
        std::uint32_t left = std::numeric_limits<std::uint32_t>::max();
        std::uint32_t right = std::numeric_limits<std::uint32_t>::max();
        std::uint32_t first = 0;
        std::uint32_t count = 0;

        [[nodiscard]] bool leaf() const noexcept { return count != 0U; }
    };

    struct ContactCandidate {
        bool valid = false;
        math::Vec3d point{};
        math::Vec3d normal{};
        double penetration = 0.0;
        std::uint8_t corner_index = 0;
    };

    static MassProperties build_mass_properties(const RigidBodyDesc& desc) noexcept;
    static math::Vec3f angular_acceleration_world(const RigidBody& body) noexcept;
    static void integrate_orientation(RigidBody& body, double dt_seconds) noexcept;
    static void solve_ground_plane(RigidBody& body, double dt_seconds) noexcept;
    static void apply_damping(RigidBody& body, double dt_seconds) noexcept;
    static math::Vec3d inverse_inertia_world(const RigidBody& body, math::Vec3d value) noexcept;
    static void apply_impulse(RigidBody& body, math::Vec3d impulse, math::Vec3d lever) noexcept;
    static bool point_in_triangle(math::Vec3d point, const CollisionTriangle& triangle) noexcept;
    static Aabb triangle_bounds(const CollisionTriangle& triangle) noexcept;
    static Aabb merge_bounds(const Aabb& a, const Aabb& b) noexcept;
    static bool overlaps(const Aabb& a, const Aabb& b) noexcept;
    static bool ray_intersects_aabb(math::Vec3d origin,
                                    math::Vec3d direction,
                                    double max_distance,
                                    const Aabb& bounds,
                                    double* entry_distance = nullptr) noexcept;

    void build_static_bvh();
    [[nodiscard]] std::uint32_t build_static_bvh_node(std::uint32_t first, std::uint32_t count);
    [[nodiscard]] ContactCandidate deepest_static_contact(RigidBody& body) noexcept;
    void solve_static_mesh(RigidBody& body, double dt_seconds) noexcept;

    core::SlotMap<RigidBodyHandle, RigidBody> bodies_{};
    std::vector<CollisionTriangle> static_triangles_{};
    std::vector<std::uint32_t> static_bvh_triangle_indices_{};
    std::vector<StaticBvhNode> static_bvh_nodes_{};
    math::Vec3d gravity_{0.0, 0.0, -9.80665};
    mutable std::uint64_t pending_query_triangle_tests_ = 0;
    mutable std::uint64_t pending_query_bvh_node_visits_ = 0;
    PhysicsStepStats last_step_stats_{};
};

} // namespace ocs::physics
