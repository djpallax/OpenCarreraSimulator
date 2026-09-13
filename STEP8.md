# Step 8 — Physics Laboratory

Step 8 builds the deterministic rigid-body and collision laboratory that Step 9 vehicle dynamics will use. Physics remains independent from Vulkan and runs at a fixed frequency; the World/RenderScene boundary only receives interpolated snapshots.

This document is the implementation record. A substep is only marked implemented when it has code, telemetry and/or a regression test appropriate to that layer.

## 8.1 — Fixed-step simulation clock — IMPLEMENTED

Tasks:
- [x] Physics frequency independent from render frequency.
- [x] Default target: 500 Hz (`2 ms`).
- [x] Configurable with `--physics-hz`; initial intended range 500–1000 Hz.
- [x] Accumulator-based stepping.
- [x] Maximum catch-up/substep count to prevent spiral-of-death behavior.
- [x] Dropped/capped simulation-time telemetry.
- [x] Render interpolation alpha exposed every frame.
- [x] Pause and single-step support.

Acceptance:
- Different render cadences produce the same number of physics steps for the same elapsed time.
- A large frame hitch cannot create an unbounded number of physics steps.

## 8.2 — Rigid-body 6DOF core — IMPLEMENTED

Tasks:
- [x] Generational `RigidBodyHandle`.
- [x] Double-precision persistent linear position and velocity.
- [x] Quaternion orientation.
- [x] Linear/angular velocity and acceleration.
- [x] Mass and inverse mass.
- [x] Body-space diagonal inertia and inverse inertia.
- [x] Semi-implicit Euler translation integration.
- [x] Quaternion angular integration with normalization.
- [x] Gyroscopic term included in angular acceleration.
- [x] Linear/angular damping parameters, disabled by default and opt-in per body.

Acceptance:
- Long-running quaternion integration remains normalized.
- Uniform motion remains uniform when no net force or damping acts.

## 8.3 — Force / torque accumulation — IMPLEMENTED

Tasks:
- [x] `add_force()`.
- [x] `add_torque()`.
- [x] `add_force_at_point()` using `r x F`.
- [x] Accumulators consumed and cleared every fixed step.
- [x] Force-at-center and off-center-force regression tests.
- [x] Temporary lab controls: hold `F` for lateral test force, hold `T` for yaw test torque.

This is the API Step 9 suspension/tire code will build on. Vehicle code must not be embedded in `RigidBody`.

## 8.4 — Infinite laboratory ground plane — IMPLEMENTED / LEGACY TEST PATH

Tasks:
- [x] Optional infinite plane contact.
- [x] Penetration clamp.
- [x] Basic restitution.
- [x] Basic tangential damping.
- [x] Ground-plane regression test retained.

Status:
- The application no longer uses this as its primary track contact path after 8.8.
- It remains useful for isolated solver tests and regression coverage.

## 8.5 — Physics → World → Render interpolation — IMPLEMENTED

Tasks:
- [x] Physics owns the dynamic body's authoritative state.
- [x] Previous/current physics states retained.
- [x] Render transform interpolated using fixed-step alpha.
- [x] World remains the extraction boundary to `RenderScene`.
- [x] Vulkan never sees or owns a `RigidBody`.
- [x] OBJECT mode edits the physics body instead of fighting a render-only transform.
- [x] Physics position now represents the rigid body's centre of mass / OBB centre.
- [x] Non-centred artist-model origins are handled as a render-only local offset.

Expected flow:

`Input -> Physics fixed steps -> interpolated body state -> visual-origin offset -> World -> RenderScene -> Vulkan`

## 8.6 — Debug telemetry window — IMPLEMENTED

When `--metrics` or `--metrics-file` is present, a second SDL software-rendered telemetry window opens and refreshes at approximately 10 Hz.

Displayed data includes:
- [x] frame/FPS/render CPU/sync/GPU timing;
- [x] render instances, draws, triangles and resource counts;
- [x] GPU identity and texture memory;
- [x] physics target Hz, fixed dt, steps/frame, interpolation alpha and accumulator;
- [x] physics CPU time and average step time;
- [x] simulation time and dropped time;
- [x] body count, static-triangle count, triangle tests and contacts;
- [x] gravity;
- [x] camera world position, yaw/pitch, forward vector, velocity and acceleration;
- [x] world object count;
- [x] per-object handles, model handle, visibility/movability, position, rotation and scale;
- [x] per-physics-object mass, grounded state, linear/angular velocity and acceleration;
- [x] collision OBB centre/half-extents;
- [x] support normal and latest contact point;
- [x] DRIVE input, longitudinal/lateral speed and applied lab-controller forces/torque.

Window events remain separated from the Vulkan window.

## 8.7 — Physics telemetry CSV + core regression tests — IMPLEMENTED

Tasks:
- [x] Existing CSV preserved.
- [x] Physics Hz/steps/CPU/step time/bodies/contacts/sim-time/alpha/dropped-time.
- [x] Static-triangle and triangle-test counters.
- [x] Fixed-step render-cadence regression.
- [x] Catch-up cap regression.
- [x] Gravity free-fall regression.
- [x] Constant-force regression.
- [x] Centre/off-centre force regressions.
- [x] Quaternion normalization regression.
- [x] Ground-plane regression.
- [x] Render-state interpolation regression.

## 8.8 — Static track collision + support dynamics — IMPLEMENTED IN THIS SUBSTEP

Source asset:
- `assets_src/world/test_circuit.glb` — Blender circuit supplied for the physics laboratory.

Build pipeline:
- [x] Compile the GLB to `test_circuit.ocsmodel` for rendering.
- [x] Compile the same source independently to `test_circuit_collision.ocsmesh` for physics.
- [x] Keep visual and collision runtime resources separate even while source geometry is currently identical.

Static collision:
- [x] `PhysicsWorld` accepts a static triangle set without depending on asset-file code.
- [x] Degenerate triangles rejected.
- [x] Contact normals derived from triangle winding.
- [x] One-sided floor/wall semantics retained: track floor normals point up and walls inward.
- [x] Proximity guard rejects false contacts against distant triangle planes.
- [x] Triangle-test telemetry exposes brute-force cost before a broadphase is justified.

Dynamic body collision:
- [x] Oriented-box (OBB) collision shape derived from model bounds.
- [x] Physics body centred at the OBB/COM rather than assuming the artist origin is the COM.
- [x] Eight transformed OBB support vertices tested against track triangles.
- [x] Normal impulse includes rotational effective mass.
- [x] Contact impulse at a corner generates real angular response.
- [x] Coulomb-style tangential impulse approximation.
- [x] Positional penetration correction.
- [x] Low-speed restitution suppression to avoid resting bounce.
- [x] Support normal / latest contact telemetry.

Support/equilibrium requirement:
- [x] A tilted box dropped onto a flat triangle floor rotates naturally through corner contacts until its face becomes parallel to the support plane.
- [x] No `orientation = identity` or artificial snap-to-flat operation is used.
- [x] The same mechanism is intended to settle against future ramps according to their actual surface normal.

Track spawn:
- [x] Detect the repeated cross-track width from floor edges.
- [x] Spawn on the inferred centreline rather than a triangle centroid near an edge.
- [x] Infer the local track tangent and align the body forward axis to it.
- [x] Position the debug camera relative to the generated spawn.

Regression tests added:
- [x] Static triangle floor supports an OBB.
- [x] Tilted box settles parallel to the floor.
- [x] Inward-facing static wall blocks the OBB.

Expected suite after 8.8/8.9: **42 tests**.

## 8.9 — DRIVE laboratory control mode — IMPLEMENTED IN THIS SUBSTEP

`C` now cycles:

`CAMERA -> OBJECT -> DRIVE -> CAMERA`

DRIVE controls:
- `W`: longitudinal acceleration request.
- `S`: brake request; clamps braking so low-speed braking does not numerically reverse the body.
- `A / D`: left/right steering request.
- `Shift`: temporary debug drive-force/steering boost.

Implementation tasks:
- [x] Longitudinal drive force.
- [x] Braking force based on current body-forward speed.
- [x] Steering yaw torque scaled by longitudinal speed.
- [x] Steering axis uses current support normal while grounded.
- [x] Bounded lateral-velocity stabilizer so yaw changes trajectory instead of leaving the body sliding in its old world direction.
- [x] DRIVE values exposed in the telemetry window.

Important scope boundary:
- This is deliberately a **laboratory controller**, not a tire model.
- The lateral stabilizer is not Pacejka, brush, slip-angle or combined-slip physics.
- Step 9 replaces this temporary behavior with wheel contacts, suspension and tire forces applied through `add_force_at_point()`.

## 8.10 — 3D physics debug drawing — IMPLEMENTED IN THIS SUBSTEP

Tasks:
- [x] Generic `DebugDrawList` carried by `RenderScene`; physics never depends on the renderer.
- [x] Dedicated Vulkan `LINE_LIST` pipeline with per-vertex color.
- [x] Host-visible per-frame debug vertex buffers that grow on demand.
- [x] Depth-tested debug lines with depth writes disabled.
- [x] Centre-of-mass marker.
- [x] OBB wireframe.
- [x] Body-local +X/+Y/+Z axes.
- [x] Linear velocity vector.
- [x] Angular velocity vector/axis.
- [x] Last external-force vector.
- [x] Support-normal vector while grounded.
- [x] Per-contact point markers and contact normals.
- [x] `V` toggles 3D physics debug rendering without changing physics/gameplay state.
- [x] Debug line count/draw-call telemetry and CSV field.

Color convention:
- COM: yellow; OBB: cyan; velocity: green; angular velocity: magenta.
- External force: orange; support normal: blue.
- Contact points: red; contact normals: yellow.
- Body axes: +X red, +Y green, +Z blue.

The numeric telemetry window remains authoritative for exact values; this layer exists only for spatial diagnosis.

## 8.11 — Dedicated physics execution thread — IMPLEMENTED IN THIS SUBSTEP

Default runtime mode now executes the fixed-step solver on one dedicated `std::jthread`.
The renderer/game loop does not call or mutate `PhysicsWorld` after the thread starts.

Ownership / synchronization rules:
- [x] `PhysicsWorld` has one owner: the physics thread while threaded execution is active.
- [x] Main/render never holds pointers or references into live physics state.
- [x] Continuous DRIVE/test input uses a latest-value input mailbox copied at each physics tick.
- [x] Pause/resume, reset and debug pose edits use an ordered command queue.
- [x] Immutable-by-copy physics snapshots cross back to the render thread under a very short mutex.
- [x] Snapshot contains previous/current body state, contact/debug data and solver counters.
- [x] Render interpolation uses one physics tick of intentional latency rather than racing the current write.
- [x] Shutdown requests stop, wakes the scheduler and joins before renderer/platform teardown.
- [x] Paused mode sleeps on a condition variable; it does not busy-spin.
- [x] Single-step while paused executes exactly one fixed tick on the physics owner thread.
- [x] Reset also clears simulation time, scheduler/catch-up/drop counters and timing history.

Scheduler behavior:
- [x] Absolute `steady_clock` deadlines rather than render-frame deltas.
- [x] Small bounded catch-up window for ordinary OS scheduling jitter.
- [x] Severe lag is dropped/re-aligned instead of entering an unbounded spiral of death.
- [x] No CPU affinity or elevated OS priority yet; those are platform-specific optimizations that require measurements first.

Diagnostics added:
- target vs measured physics Hz;
- average/peak step time and percentage of the fixed-step budget;
- scheduler lag and snapshot age;
- catch-up steps, dropped ticks and deadline misses;
- cumulative fixed-step count.

Fallback / A-B testing:
- `--single-thread-physics` restores the former accumulator-on-main-thread path.
- Both execution modes call the same `PhysicsWorld`, DRIVE callback and collision solver.
- The fallback exists for debugging, deterministic comparisons, sanitizer work and platform bring-up; it is not a second physics implementation.

Regression coverage:
- paused threaded single-step advances exactly one tick;
- reset command is applied by the physics owner thread without stepping;
- snapshot interpolation preserves the intended one-tick render-latency model.

Expected suite after 8.11: **48 tests**.

## Runtime controls

- `C`: cycle CAMERA / OBJECT / DRIVE.
- CAMERA: `W/S`, `A/D`, `Q/E`, `Shift`, `RMB + mouse`.
- OBJECT: `W/S`, `A/D`, `Q/E`, `Shift`, `RMB + mouse` directly reposition/rotate the rigid body for tests.
- DRIVE: `W` throttle, `S` brake, `A/D` steer, `Shift` debug boost.
- `Space`: pause/resume physics.
- `N`: one fixed physics step while paused.
- `R`: reset the laboratory rigid body and physics clock.
- `V`: toggle 3D physics debug drawing.
- `F`: hold to apply +Y test force.
- `T`: hold to apply +Z test torque.
- `--physics-hz 500`: set fixed-step target (`1000` tests a 1 ms step).
- `--single-thread-physics`: diagnostic fallback that runs the same fixed-step solver on the main thread.
- `--track <visual.ocsmodel>`: override visual track.
- `--track-collision <collision.ocsmesh>`: override collision track; override both paths together.

## Step 8 completion gate

Do not mark Step 8 complete until:
1. all tests pass;
2. Vulkan validation stays clean;
3. 500 Hz and 1000 Hz both run correctly;
4. pause/single-step/reset behave correctly;
5. Blender test-circuit floor and walls collide correctly;
6. a deliberately tilted body settles on the support surface without orientation snapping;
7. DRIVE acceleration/braking/steering remain stable enough for laboratory use;
8. telemetry window reports coherent collision/drive state;
9. 3D physics debug drawing (8.10) is implemented and validated;
10. dedicated-thread telemetry shows no sustained deadline misses/dropped ticks at the selected physics rate;
11. a long headless stability run shows no NaN/Inf or explosive quaternion drift.

Current automated suite after 8.11: **48 tests**.


## Step 8.11.1 — threading/shutdown hotfix

Status: **complete**

- Fixed lifetime ordering for the per-frame Vulkan debug-line buffers: both debug `VkBuffer`/`VkDeviceMemory` allocations are now destroyed before `VkDevice`.
- Added explicit `--dedicated-physics` override; dedicated physics remains the default.
- Startup logging now distinguishes an explicitly requested single-thread diagnostic run from the normal dedicated-thread path.
- Console stats now include `physFrame` so main-thread physics burst cost is visible instead of being hidden behind renderer-only CPU timing.
- `--single-thread-physics` remains an A/B/debug path, not the recommended normal execution mode.


## Step 8.11.2 — Kinematic object manipulation hotfix

- OBJECT mode now uses a persistent kinematic target instead of incremental teleports from delayed physics snapshots.
- The render thread previews that target immediately for smooth manipulation.
- The physics owner thread consumes the same latest target and temporarily marks the body non-dynamic while OBJECT mode is active.
- Leaving OBJECT mode restores dynamic simulation with zeroed motion, preventing gravity/contact from fighting editor manipulation.
- This specifically removes the threaded-mode jitter/rubber-banding seen while manually moving the object.
