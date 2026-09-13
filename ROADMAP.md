# OpenCarreraSimulator — Engineering Roadmap

This roadmap preserves the intent of the original project plan while updating the
milestones to match the architecture that now exists.

## Original high-level roadmap

1. Core: CMake, C++23, Linux, logging, math, tests, SDL3, window.
2. Renderer: Vulkan, triangle, buffers, shaders, camera, glTF.
3. Asset pipeline: asset compiler, GLB -> runtime format, KTX2, hot reload.
4. First circuit: mesh, free camera, basic streaming, materials.
5. Physics sandbox: rigid body, integrator, debug drawing.
6. First car: chassis, four wheels, suspension, basic tire model.
7. Drivetrain: engine, clutch, gearbox, differential.
8. Wheel input: G923, calibration, pedals and steering.
9. Physical FFB: rack torque and filters.
10. Vehicle physics v2: temperature, wear, aero and suspension geometry.
11. Track physics: materials, grip, kerbs, temperature and rubber.
12. AI: line, speed profile, controller and traffic.
13. UI: custom frontend, GT-inspired interaction and presentation.
14. Race engine: laps, penalties, pits, grids and sessions.
15. Audio: engine, transmission, tires and environment.
16. Weather: rain, wetness, drying and temperature.
17. Replay/telemetry: deterministic regression and analysis.
18. Windows port using the same engine and Vulkan first.
19. Optional D3D12 backend when there is a concrete reason.
20. Multiplayer / VR / triple-screen after the single-player simulation is solid.

## Revised roadmap after Step 4

### Step 1 — Core foundation ✅
C++23, CMake, tests, logging, math, SDL3/platform abstraction.

### Step 2 — Vulkan platform foundation ✅
Instance/device selection, NVIDIA/AMD support, swapchain, synchronization,
validation and resize handling.

### Step 2.5 — Build/engine hardening ✅
Static engine libraries, target aliases, project warning/sanitizer targets,
GPU classification, memory budgets, RAII and `std::expected` primitives.

### Step 3 — Render foundation ✅
Device-local buffers, staging, shaders, pipeline, depth, camera math, indexed 3D
geometry and Vulkan GPU timestamps.

### Step 4 — Offline asset pipeline ✅
glTF/GLB -> `.ocsmesh`, coordinate conversion, versioned runtime format,
integrity validation and runtime metrics.

### Step 4.1 — Cleanup / trustworthy telemetry ✅
Third-party warning isolation, deprecated API cleanup, internally consistent FPS
and frame-time, CPU work vs synchronization/pacing timing.

### Step 5 — Resource & material foundation ✅
- Generational `ModelHandle`, `MeshHandle` and `MaterialHandle` registries.
- Versioned `.ocsmodel` with submeshes and material assignments.
- Descriptor-set foundation and per-frame scene UBO.
- PBR metallic/roughness material factors and double-sided/unlit metadata.
- Real Blender reference asset compiled through the offline pipeline.
- Resource counts and stale-handle tests.

### Step 5.1 — Texture resource extension ✅
- Generational `TextureHandle` and `SamplerHandle` registries.
- Offline PNG/JPEG decode from glTF/GLB; no runtime image codec.
- Versioned `.ocstex` RGBA8 sidecars with offline mip generation.
- Vulkan images, image views, samplers and staging upload.
- Base-color material texture descriptor binding and white fallback texture.
- Texture resource/memory telemetry.
- KTX2/Basis, normal maps and metallic/roughness textures stay deferred until needed.

### Step 6 — Cross-platform gate ⏸ deferred until Windows is available
The architecture remains ready for this gate, but runtime validation is more valuable than speculative cross-platform code:
- Build and run the same Vulkan backend on Windows.
- GCC/Clang remain primary on Linux; MSVC/clang-cl build coverage on Windows.
- CI compile/test matrix where practical.
- Keep D3D12 deferred; the goal is platform validation, not another renderer.

### Step 7 — World/scene + development straight ✅
- Double-precision persistent world positions with camera-relative float rendering.
- Generational world objects and model instances.
- RenderScene extraction boundary between world/game state and Vulkan.
- Free/debug camera plus `C` toggle to movable-object control.
- Build-time generated 500 m development straight.
- Visual `.ocsmodel` and collision `.ocsmesh` separated from day one.
- Spatial chunks, culling, scene files, surface metadata and timing sectors remain follow-up work.

### Step 8 — Physics laboratory ✅
- Fixed-step simulation independent from rendering.
- Configurable 500–1000 Hz physics experiments.
- Rigid-body 6DOF, force/torque accumulation and integrator validation.
- Blender test circuit compiled into separate visual and static collision resources.
- OBB-vs-static-triangle contacts with rotational impulses so tilted bodies settle onto support surfaces naturally.
- CAMERA / OBJECT / DRIVE laboratory control modes; DRIVE remains temporary and is not the Step 9 tire model.
- Numeric physics/collision telemetry plus in-scene 3D COM/OBB/vector/contact debug drawing are implemented.
- Dedicated fixed-step physics thread is now the default, with command/input mailboxes, render snapshots and a `--single-thread-physics` A/B fallback.
- Long-run stability/performance validation remains the final item before the physics laboratory can be considered closed.

### Step 9 — First vehicle ← current
- Step 9.0: original low-poly GT body + reusable wheel GLBs, four visual wheel instances and DRIVE chase camera.
- Step 9.1: `engine/vehicle` configuration, RWD wheel layout, static-track wheel raycasts, contact-gated rear-wheel propulsion, corrected wheel ride height and sawtooth test ramp.
- Step 9.2: persistent four-corner suspension state with mount/hub separation, bump/droop travel, compression and compression velocity.
- Step 9.3: spring/damper support at each wheel station; chassis OBB returns to body-shell collision duty instead of normal road support.
- Step 9.3.1: low-speed steering hotfix replaces open-loop yaw torque with a speed-coupled bicycle yaw-rate target and centred-steering yaw damping.
- Step 9.3.2: suspension regression tolerances corrected to match float-authored geometry precision.
- Step 9.4: persistent per-wheel angular velocity/rotation and rotational inertia.
- Step 9.5: first load-limited longitudinal slip-ratio tire force; throttle/braking now act through wheel torque and tire contact instead of direct chassis force.
- Wheel/contact model.
- Basic suspension. ✅ spring/damper raycast prototype complete; geometry/anti-roll refinement remains.
- Basic tire forces: ✅ longitudinal slip/load coupling; slip angle and combined-slip path remain.
- Steering and brakes.
- Keyboard/gamepad controls sufficient for physics development.

### Step 10 — Drivetrain & basic aero
- Torque curve.
- Engine inertia.
- Clutch.
- Gearbox/final drive.
- Open/LSD differential foundations.
- Basic drag/downforce model with an interface designed for later aero maps.

### Step 11 — Device input / G923
- Generic device/input action layer.
- Hotplug and per-device calibration/profiles.
- Steering wheel, pedals, H-pattern shifter and handbrake abstractions.
- Linux input backend refinements when SDL/HID coverage is insufficient.

### Step 12 — Physical force feedback
- Steering rack/column torque derived from vehicle physics.
- Aligning torque, caster/trail effects and configurable filtering.
- 500–1000 Hz output path where hardware/backend permits.
- TrueForce/vendor extensions remain optional and isolated.

### Step 13 — Vehicle physics v2
- Suspension geometry and kinematics.
- Advanced tire model experiments.
- Tire pressure, thermal state and wear.
- Aero maps/ride-height sensitivity.
- ABS/TCS and other electronics.

### Step 14 — Track physics
- Per-surface friction and roughness.
- Kerbs, bumps and contact detail.
- Rubber evolution and temperature hooks.
- Wetness/drainage hooks for later weather.

### Step 15 — AI
Strategy -> tactical planner -> trajectory planner -> vehicle controller, using the
same physical vehicle model as the player.

### Step 16 — Custom UI / GT-inspired frontend
Retained-mode UI, controller navigation, transitions, garage/dealer/race flows and
3D presentation. ImGui remains debug tooling only.

### Step 17 — Race engine
Sessions, grids, timing, sectors, laps, flags, penalties, pits and championship/event
rules. This is a natural point to evaluate a lightweight scripting layer.

### Step 18 — Audio
Engine/drivetrain/tire/environment model, realtime mixer and spatial audio.

### Step 19 — Weather and dynamic track state
Rain, wetness, drying, temperatures and coupling to track/tire physics.

### Step 20 — Replay, regression and advanced telemetry
The telemetry foundation already exists. Expand it into input/state replays,
comparison across commits, 1%/0.1% lows, physics-channel recording and automated
regression runs.

### Step 21 — Optional renderer/platform expansion
D3D12 if it delivers a concrete Windows benefit; Metal if macOS becomes a target.
The RHI boundary must justify these backends before implementation starts.

### Step 22 — Multiplayer / VR / triple-screen
Only after the single-player physics, timing and resource systems are stable.

## Guiding rule

Do not optimize the roadmap around visually impressive screenshots. Every milestone
should either validate a subsystem boundary, improve measurable simulation quality,
or remove a future scaling risk.
