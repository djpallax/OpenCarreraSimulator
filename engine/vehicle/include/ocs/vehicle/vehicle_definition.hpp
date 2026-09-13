#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

#include "ocs/math/vec3f.hpp"
#include "ocs/vehicle/vehicle.hpp"

namespace ocs::vehicle {

struct VehicleDefinition {
    std::string name = "OCS Lab GT";
    std::filesystem::path directory{};
    std::filesystem::path body_model = "body.ocsmodel";
    std::filesystem::path wheel_model = "wheel.ocsmodel";

    double body_scale = 1.0;
    double wheel_visual_scale = 1.0;
    double mass_kg = 1200.0;
    math::Vec3f inertia_scale{1.0F, 1.0F, 1.0F};

    double ride_clearance_m = 0.22;
    double static_suspension_length_m = 0.22;
    double front_axle_x_m = 1.36;
    double rear_axle_x_m = -1.33;
    double half_track_m = 0.79;
    bool front_steerable = true;
    bool rear_steerable = false;

    double restitution = 0.04;
    double chassis_friction = 0.70;
    double linear_damping = 0.015;
    double angular_damping = 0.12;

    // Static-mesh robustness controls. Deep pre-existing overlap with arbitrary
    // scenery is ignored, while legitimate swept impacts remain resolvable.
    double static_contact_max_penetration_m = 0.35;
    double static_contact_persistence_depth_m = 0.35;
    double upright_ground_contact_max_penetration_m = 0.12;
    double maximum_depenetration_speed_mps = 3.0;

    VehicleConfig dynamics{};

    bool config_file_found = false;
    std::size_t applied_attribute_count = 0;

    [[nodiscard]] std::filesystem::path body_model_path() const;
    [[nodiscard]] std::filesystem::path wheel_model_path() const;
};

[[nodiscard]] VehicleDefinition default_vehicle_definition();

// Loads <directory>/vehicle.cfg. The parser is intentionally permissive:
// unknown keys and malformed values are ignored, and every omitted attribute
// retains the engine default from default_vehicle_definition().
[[nodiscard]] VehicleDefinition load_vehicle_definition(const std::filesystem::path& directory);

} // namespace ocs::vehicle
