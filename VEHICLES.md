# Vehicle profiles

Step 9.6.1 moves vehicle-specific assets and tuning out of `app/src/main.cpp`.
A runtime vehicle is a directory containing at least the compiled body and wheel
models; `vehicle.cfg` is optional.

Default build layout:

```text
build/gcc-debug/assets/vehicle/ocs_lab_gt/
├── body.ocsmodel
├── wheel.ocsmodel
└── vehicle.cfg
```

Select another vehicle without recompiling:

```bash
./build/gcc-debug/app/OpenCarreraSimulator \
  --vehicle-dir /path/to/my_car \
  --track /path/to/track.ocsmodel \
  --track-collision /path/to/track_collision.ocsmesh
```

`--asset` and `--wheel` remain available as explicit model overrides.

## vehicle.cfg

The format is `key = value`; `#` and `;` start comments. Every key is optional.
Missing, malformed, or unknown attributes leave the engine default unchanged.
This makes partial vehicle profiles valid and allows new attributes to be added
without breaking older cars.

Current keys:

```ini
name = My Car
body_model = body.ocsmodel
wheel_model = wheel.ocsmodel

body_scale = 1.0
wheel_visual_scale = 1.0
mass_kg = 1200
inertia_scale_x = 1.0
inertia_scale_y = 1.0
inertia_scale_z = 1.0
ride_clearance_m = 0.22
static_suspension_length_m = 0.22
front_axle_x_m = 1.36
rear_axle_x_m = -1.33
half_track_m = 0.79
front_steerable = true
rear_steerable = false

restitution = 0.02
chassis_friction = 0.70
linear_damping = 0.015
angular_damping = 0.12
static_contact_max_penetration_m = 0.35
static_contact_persistence_depth_m = 0.35
upright_ground_contact_max_penetration_m = 0.12
maximum_depenetration_speed_mps = 3.0

wheel_radius_m = 0.34
suspension_rest_length_m = 0.30
suspension_max_compression_m = 0.14
suspension_max_droop_m = 0.12
spring_rate_n_m = 36000
damper_rate_n_s_m = 4500
wheel_rotational_inertia_kg_m2 = 1.50
longitudinal_stiffness_n = 18000
peak_friction_coefficient = 1.20

legacy_drive_acceleration_mps2 = 9.0
drive_layout = RWD
maximum_drive_torque_nm = 1800
maximum_brake_torque_nm_per_wheel = 1800
longitudinal_slip_reference_speed_mps = 2.0
speed_boost_multiplier = 1.6

maximum_steer_angle_degrees = 29.0
provisional_max_lateral_acceleration_mps2 = 12.0
provisional_lateral_response = 9.0
provisional_yaw_rate_response = 8.0
provisional_max_yaw_acceleration_rps2 = 3.5
```

`maximum_steer_angle_radians` is also accepted when radians are preferred.

## Static collision stabilization

The chassis solver remains one-sided triangle collision, but Step 9.6.1 makes
imported high-detail tracks safer:

- positional depenetration is capped by `maximum_depenetration_speed_mps * dt`;
- penetration beyond `static_contact_max_penetration_m` is not treated as a
  plausible discrete contact;
- while the car is upright, floor-like triangles only test lower chassis corners
  and use the tighter `upright_ground_contact_max_penetration_m` limit;
- upright floor-like contacts suppress restitution, avoiding bounce from seams;
- wall, rollover, and crash contacts retain the configured restitution.

These protections make a full visual mesh less explosive as temporary collision,
but a dedicated/filtered track collision mesh is still the preferred long-term
asset for real circuits.
