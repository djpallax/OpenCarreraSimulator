#include "ocs/vehicle/vehicle_definition.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>

namespace ocs::vehicle {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::string trim(std::string value) {
    const auto not_space = [](const unsigned char ch) { return std::isspace(ch) == 0; };
    const auto first = std::find_if(value.begin(), value.end(), not_space);
    if (first == value.end()) {
        return {};
    }
    const auto last = std::find_if(value.rbegin(), value.rend(), not_space).base();
    return std::string(first, last);
}

std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

std::optional<double> parse_double(const std::string& text) {
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

std::optional<bool> parse_bool(const std::string& text) {
    const std::string value = lowercase(trim(text));
    if (value == "1" || value == "true" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off") {
        return false;
    }
    return std::nullopt;
}

std::optional<DriveLayout> parse_drive_layout(const std::string& text) {
    const std::string value = lowercase(trim(text));
    if (value == "fwd" || value == "front" || value == "front_wheel_drive") {
        return DriveLayout::front_wheel_drive;
    }
    if (value == "rwd" || value == "rear" || value == "rear_wheel_drive") {
        return DriveLayout::rear_wheel_drive;
    }
    if (value == "awd" || value == "4wd" || value == "all_wheel_drive") {
        return DriveLayout::all_wheel_drive;
    }
    return std::nullopt;
}

template <typename Setter>
bool apply_number(const std::string& value, Setter&& setter) {
    const auto parsed = parse_double(value);
    if (!parsed.has_value()) {
        return false;
    }
    setter(*parsed);
    return true;
}

void apply_common_wheel_value(VehicleDefinition& definition,
                              const std::string& key,
                              const std::string& value,
                              bool& applied) {
    auto apply_to_all = [&](auto member, const double parsed) {
        for (auto& wheel : definition.dynamics.wheels) {
            wheel.*member = parsed;
        }
    };

    if (key == "wheel_radius_m") {
        applied = apply_number(value, [&](const double v) {
            apply_to_all(&WheelConfig::radius, std::max(v, 0.01));
        });
    } else if (key == "suspension_rest_length_m") {
        applied = apply_number(value, [&](const double v) {
            apply_to_all(&WheelConfig::rest_length, std::max(v, 0.0));
        });
    } else if (key == "suspension_max_compression_m") {
        applied = apply_number(value, [&](const double v) {
            apply_to_all(&WheelConfig::max_compression, std::max(v, 0.0));
        });
    } else if (key == "suspension_max_droop_m") {
        applied = apply_number(value, [&](const double v) {
            apply_to_all(&WheelConfig::max_droop, std::max(v, 0.0));
        });
    } else if (key == "spring_rate_n_m") {
        applied = apply_number(value, [&](const double v) {
            apply_to_all(&WheelConfig::spring_rate, std::max(v, 0.0));
        });
    } else if (key == "damper_rate_n_s_m") {
        applied = apply_number(value, [&](const double v) {
            apply_to_all(&WheelConfig::damper_rate, std::max(v, 0.0));
        });
    } else if (key == "wheel_rotational_inertia_kg_m2") {
        applied = apply_number(value, [&](const double v) {
            apply_to_all(&WheelConfig::rotational_inertia, std::max(v, 1.0e-4));
        });
    } else if (key == "longitudinal_stiffness_n") {
        applied = apply_number(value, [&](const double v) {
            apply_to_all(&WheelConfig::longitudinal_stiffness, std::max(v, 0.0));
        });
    } else if (key == "peak_friction_coefficient") {
        applied = apply_number(value, [&](const double v) {
            apply_to_all(&WheelConfig::peak_friction_coefficient, std::max(v, 0.0));
        });
    }
}

bool apply_attribute(VehicleDefinition& definition,
                     const std::string& raw_key,
                     const std::string& raw_value) {
    const std::string key = lowercase(trim(raw_key));
    const std::string value = trim(raw_value);
    bool applied = false;

    apply_common_wheel_value(definition, key, value, applied);
    if (applied) {
        return true;
    }

    if (key == "name") {
        if (!value.empty()) {
            definition.name = value;
            return true;
        }
    } else if (key == "body_model") {
        if (!value.empty()) {
            definition.body_model = value;
            return true;
        }
    } else if (key == "wheel_model") {
        if (!value.empty()) {
            definition.wheel_model = value;
            return true;
        }
    } else if (key == "drive_layout") {
        if (const auto layout = parse_drive_layout(value); layout.has_value()) {
            definition.dynamics.drive_layout = *layout;
            return true;
        }
    } else if (key == "front_steerable") {
        if (const auto parsed = parse_bool(value); parsed.has_value()) {
            definition.front_steerable = *parsed;
            return true;
        }
    } else if (key == "rear_steerable") {
        if (const auto parsed = parse_bool(value); parsed.has_value()) {
            definition.rear_steerable = *parsed;
            return true;
        }
    } else if (key == "body_scale") {
        return apply_number(value, [&](const double v) { definition.body_scale = std::max(v, 0.01); });
    } else if (key == "wheel_visual_scale") {
        return apply_number(value, [&](const double v) { definition.wheel_visual_scale = std::max(v, 0.01); });
    } else if (key == "mass_kg") {
        return apply_number(value, [&](const double v) { definition.mass_kg = std::max(v, 1.0); });
    } else if (key == "inertia_scale_x") {
        return apply_number(value, [&](const double v) { definition.inertia_scale.x = static_cast<float>(std::max(v, 0.01)); });
    } else if (key == "inertia_scale_y") {
        return apply_number(value, [&](const double v) { definition.inertia_scale.y = static_cast<float>(std::max(v, 0.01)); });
    } else if (key == "inertia_scale_z") {
        return apply_number(value, [&](const double v) { definition.inertia_scale.z = static_cast<float>(std::max(v, 0.01)); });
    } else if (key == "ride_clearance_m") {
        return apply_number(value, [&](const double v) { definition.ride_clearance_m = std::max(v, 0.0); });
    } else if (key == "static_suspension_length_m") {
        return apply_number(value, [&](const double v) { definition.static_suspension_length_m = std::max(v, 0.0); });
    } else if (key == "front_axle_x_m") {
        return apply_number(value, [&](const double v) { definition.front_axle_x_m = v; });
    } else if (key == "rear_axle_x_m") {
        return apply_number(value, [&](const double v) { definition.rear_axle_x_m = v; });
    } else if (key == "half_track_m") {
        return apply_number(value, [&](const double v) { definition.half_track_m = std::max(v, 0.01); });
    } else if (key == "restitution") {
        return apply_number(value, [&](const double v) { definition.restitution = std::clamp(v, 0.0, 1.0); });
    } else if (key == "chassis_friction") {
        return apply_number(value, [&](const double v) { definition.chassis_friction = std::max(v, 0.0); });
    } else if (key == "linear_damping") {
        return apply_number(value, [&](const double v) { definition.linear_damping = std::max(v, 0.0); });
    } else if (key == "angular_damping") {
        return apply_number(value, [&](const double v) { definition.angular_damping = std::max(v, 0.0); });
    } else if (key == "static_contact_max_penetration_m") {
        return apply_number(value, [&](const double v) { definition.static_contact_max_penetration_m = std::max(v, 0.001); });
    } else if (key == "static_contact_persistence_depth_m") {
        return apply_number(value, [&](const double v) { definition.static_contact_persistence_depth_m = std::max(v, 0.0); });
    } else if (key == "upright_ground_contact_max_penetration_m") {
        return apply_number(value, [&](const double v) { definition.upright_ground_contact_max_penetration_m = std::max(v, 0.001); });
    } else if (key == "maximum_depenetration_speed_mps") {
        return apply_number(value, [&](const double v) { definition.maximum_depenetration_speed_mps = std::max(v, 0.0); });
    } else if (key == "legacy_drive_acceleration_mps2") {
        return apply_number(value, [&](const double v) { definition.dynamics.drive_acceleration = std::max(v, 0.0); });
    } else if (key == "maximum_drive_torque_nm") {
        return apply_number(value, [&](const double v) { definition.dynamics.maximum_drive_torque_nm = std::max(v, 0.0); });
    } else if (key == "maximum_brake_torque_nm_per_wheel") {
        return apply_number(value, [&](const double v) { definition.dynamics.maximum_brake_torque_nm_per_wheel = std::max(v, 0.0); });
    } else if (key == "longitudinal_slip_reference_speed_mps") {
        return apply_number(value, [&](const double v) { definition.dynamics.longitudinal_slip_reference_speed_mps = std::max(v, 0.01); });
    } else if (key == "maximum_steer_angle_radians") {
        return apply_number(value, [&](const double v) { definition.dynamics.maximum_steer_angle_radians = std::max(v, 0.0); });
    } else if (key == "maximum_steer_angle_degrees") {
        return apply_number(value, [&](const double v) {
            definition.dynamics.maximum_steer_angle_radians = std::max(v, 0.0) * kPi / 180.0;
        });
    } else if (key == "provisional_max_lateral_acceleration_mps2") {
        return apply_number(value, [&](const double v) { definition.dynamics.provisional_max_lateral_acceleration = std::max(v, 0.0); });
    } else if (key == "provisional_lateral_response") {
        return apply_number(value, [&](const double v) { definition.dynamics.provisional_lateral_response = std::max(v, 0.0); });
    } else if (key == "provisional_yaw_rate_response") {
        return apply_number(value, [&](const double v) { definition.dynamics.provisional_yaw_rate_response = std::max(v, 0.0); });
    } else if (key == "provisional_max_yaw_acceleration_rps2") {
        return apply_number(value, [&](const double v) { definition.dynamics.provisional_max_yaw_acceleration = std::max(v, 0.0); });
    } else if (key == "speed_boost_multiplier") {
        return apply_number(value, [&](const double v) { definition.dynamics.speed_boost_multiplier = std::max(v, 0.0); });
    }

    return false;
}

} // namespace

VehicleDefinition default_vehicle_definition() {
    VehicleDefinition result{};
    result.dynamics.drive_layout = DriveLayout::rear_wheel_drive;
    for (auto& wheel : result.dynamics.wheels) {
        wheel.radius = 0.34;
        wheel.rest_length = 0.30;
        wheel.max_compression = 0.14;
        wheel.max_droop = 0.12;
        wheel.spring_rate = 36000.0;
        wheel.damper_rate = 4500.0;
        wheel.rotational_inertia = 1.50;
        wheel.longitudinal_stiffness = 18000.0;
        wheel.peak_friction_coefficient = 1.20;
    }
    return result;
}

std::filesystem::path VehicleDefinition::body_model_path() const {
    return body_model.is_absolute() ? body_model : directory / body_model;
}

std::filesystem::path VehicleDefinition::wheel_model_path() const {
    return wheel_model.is_absolute() ? wheel_model : directory / wheel_model;
}

VehicleDefinition load_vehicle_definition(const std::filesystem::path& directory) {
    VehicleDefinition result = default_vehicle_definition();
    result.directory = directory;

    const std::filesystem::path config_path = directory / "vehicle.cfg";
    std::ifstream input(config_path);
    if (!input) {
        return result;
    }
    result.config_file_found = true;

    std::string line;
    while (std::getline(input, line)) {
        const std::size_t comment = line.find_first_of("#;");
        if (comment != std::string::npos) {
            line.resize(comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }

        const std::size_t separator = line.find('=');
        if (separator == std::string::npos) {
            continue;
        }
        if (apply_attribute(result, line.substr(0, separator), line.substr(separator + 1U))) {
            ++result.applied_attribute_count;
        }
    }

    result.static_contact_persistence_depth_m = std::min(
        result.static_contact_persistence_depth_m,
        result.static_contact_max_penetration_m);
    result.upright_ground_contact_max_penetration_m = std::min(
        result.upright_ground_contact_max_penetration_m,
        result.static_contact_max_penetration_m);
    return result;
}

} // namespace ocs::vehicle
