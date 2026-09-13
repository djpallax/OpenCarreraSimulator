# Step 9 — First Vehicle

Status: **in progress**

Step 8 established the generic rigid-body laboratory, static track collision,
dedicated fixed-step execution, debug telemetry and DRIVE laboratory controls.
Step 9 progressively moves support, steering and propulsion from temporary
chassis-level helpers to wheel/contact physics while keeping `RigidBody` generic.

## 9.0 — Vehicle visual prototype + DRIVE chase camera ✅

Two original, texture-free GLB resources live under `assets_src/vehicle/`:

- `ocs_lab_gt_body.glb`: low-poly GT/laboratory body, authored in metres;
- `ocs_lab_gt_wheel.glb`: reusable tire/rim/disc/hub wheel resource.

The body is one world instance and the wheel resource is instanced four times as
FL / FR / RL / RR. DRIVE owns a frame-rate-independent chase camera and the front
wheel visuals follow steering input while all four wheels roll with longitudinal
speed.

## 9.1 — Vehicle layout + contact-gated RWD ✅

This substep creates the first real `engine/vehicle` boundary and moves the wheel
layout out of presentation-only constants.

### Vehicle configuration

`VehicleConfig` contains:

- `DriveLayout` (`FWD`, `RWD`, `AWD`);
- four named wheel stations;
- body-local wheel centre;
- wheel radius;
- static-track contact-probe extension;
- steering flag;
- requested laboratory drive acceleration.

The OCS Lab GT is configured as **RWD**. Only RL and RR are driven. The drivetrain
layout is therefore a property of the vehicle rather than a hard-coded `W -> body
force` rule.

Current geometry:

- wheelbase: approximately 2.69 m;
- track width: approximately 1.58 m;
- wheel radius: 0.34 m;
- visual chassis ride clearance: approximately 0.22 m.

The temporary Step 8 OBB is extended downward to the tire contact plane so the
body shell is visibly above the ground and the wheel bottoms coincide with the
current support plane. This OBB support is temporary until spring/damper support.

### Static-track raycast primitive

`PhysicsWorld::raycast_static()` now returns the closest authored front-face hit:

- hit point;
- surface normal;
- distance in world metres;
- triangle index.

Each wheel casts along chassis local `-Z`. The probe length is wheel radius plus a
small extension so a wheel can lose contact instead of having infinite reach.
This query is the foundation for Step 9 suspension.

### Contact-gated propulsion

Throttle no longer applies a force directly at the chassis COM.

The requested drive force is split by the number of configured driven wheels. For
this RWD vehicle:

- RL gets 50% of requested propulsion when in contact;
- RR gets 50% of requested propulsion when in contact;
- a rear wheel with no track hit contributes **0 N**;
- if both rear wheels are airborne, throttle contributes **0 N**.

The missing share is deliberately **not redistributed** yet. This is a simple
per-wheel traction gate, not a differential model. A real open/LSD/locked
differential belongs in the drivetrain milestone.

Forces are applied at the wheel contact points and projected onto the contacted
surface tangent, so ramp contact can naturally introduce chassis pitch torque.

The temporary lateral stabilizer and steering yaw helper are also disabled or
scaled down when the relevant wheels are not in contact, preventing artificial
mid-air grip/steering.

### Wheel contact debug

With 3D physics debug enabled, each wheel probe is visible:

- green: non-driven wheel with contact;
- orange: driven wheel with contact;
- red: no contact;
- yellow short line: contact normal.

The telemetry window reports drivetrain layout plus FL/FR/RL/RR contact status,
probe distance and normal.

### Wheel-height correction

Step 9.0 aligned the wheel bottom with the body OBB bottom, which made the tires
look too high relative to the body shell. Step 9.1 introduces an explicit ride
clearance and moves the temporary OBB support plane down to the tire bottoms. The
body is therefore raised relative to the wheels without visually burying tires in
the track.


## 9.1.1 — Ride-height visual tuning ✅

After track/RWD validation, the temporary visual chassis ride clearance was increased
from approximately **0.14 m to 0.22 m**. This raises the body shell by another 8 cm
relative to the wheel centres while preserving the tire contact plane, temporary OBB
support plane, wheel raycasts and RWD contact-gating behavior. No drivetrain, solver or
track-collision logic changes in this hotfix.

## 9.1 track update — sawtooth ramp ✅

The test circuit now includes an orange sawtooth ramp shortly after the spawn:

- approximately 5.0 m long;
- approximately 6.0 m wide;
- approximately 0.55 m high;
- slope rises in the expected lap direction, followed by a sharp drop.

Visual and collision sources are now separate:

- `assets_src/world/test_circuit.glb` contains the full visible wedge;
- `assets_src/world/test_circuit_collision.glb` contains the original track plus
  only the drivable sloped ramp face.

The vertical end cap is intentionally absent from collision so the one-sided
static solver cannot turn the sawtooth drop into an invisible wall. This ramp is a
laboratory feature for wheel-contact loss/recovery and later suspension travel.

## Tests added

The Step 9.1 suite gains four regressions (total at that substep: **52**):

- static-mesh raycast hits the authored front face and ignores the back face;
- RWD marks only RL/RR as driven;
- an airborne driven wheel contributes no propulsion;
- no driven-wheel contact produces zero drive force.

## 9.2 — Suspension contact state ✅

The four Step 9.1 wheel probes are now persistent suspension stations rather than
fixed wheel-centre traction probes. Each `WheelConfig` owns:

- body-local suspension attachment (`suspension_mount_local`);
- wheel radius;
- rest length;
- maximum compression (bump travel);
- maximum droop;
- spring rate;
- damper rate;
- steering capability and drivetrain role.

The OCS Lab GT baseline is intentionally simple and symmetric:

- rest length: **0.30 m**;
- nominal/static length: about **0.22 m**;
- bump travel: **0.14 m**;
- droop travel: **0.12 m**;
- spring rate: **36 kN/m** per corner;
- damper rate: **4.5 kN·s/m** per corner.

`VehicleRuntimeState` is owned by the physics execution path and retains previous
suspension lengths so compression velocity is deterministic at the fixed physics
rate. Each wheel reports:

- mount and hub world positions;
- hit/contact point and surface normal;
- current suspension length;
- compression;
- compression velocity;
- spring force;
- damper force;
- final one-sided normal force.

A missed ray moves the visual hub to maximum droop and supplies no support or
propulsion. Contact normals are averaged for temporary steering/lateral helpers,
so ramp steering uses the contacted surface rather than assuming world +Z.

## 9.3 — Spring + damper support ✅

Normal vehicle support now comes from four suspension forces applied at the wheel
stations. For each contacted wheel:

`F_spring = k * compression`

`F_damper = c * compression_velocity`

`F_normal = max(0, F_spring + F_damper)`

The final force follows the authored track contact normal and is applied at the
wheel hub/vehicle station. Because the four forces are off-centre they naturally
produce pitch and roll moments; no explicit `tilt` rule is used. The damper is
one-sided at the road interface: rebound can reduce the support force but the road
never pulls the chassis downward.

### OBB role change

The Step 9.1 temporary OBB extension to the tire plane has been removed. The OBB
again wraps the actual body shell. At normal ride height the chassis is therefore
clear of the track and the four springs carry the vehicle. The OBB remains active
for:

- wall/body impacts;
- suspension bottom-out / hard landings;
- rollover and roof/body collision.

This is the intended transition from a generic Step 8 rigid-body box to a vehicle
whose normal support comes from wheels while retaining a chassis crash collider.

### Visual suspension travel

Wheel instances are no longer rigidly attached to one body-local centre. Their hub
position is reconstructed from suspension mount minus current suspension length, so
the four wheel GLBs visibly move through bump/droop travel. Steering and wheel roll
remain layered on top of that hub transform.

### Debug / telemetry

The second window now reports FL / FR / RL / RR:

- suspension length;
- compression in mm;
- compression velocity in m/s;
- normal load in N;
- contact normal;
- driven/contact flags.

3D debug draws mount -> hub -> road contact. The contact-normal line grows with
normal load, making pitch/roll load transfer and ramp compression visible.

### Regression additions

Step 9.2/9.3 adds three vehicle regressions (expected total: **55**):

- rest-length contact reports zero compression/force;
- shortening suspension produces positive compression velocity and damper force;
- four compressed suspension stations apply the expected upward support force.

A separate headless long-run smoke test was also used while developing this step:
with a 1200 kg chassis on flat ground the car settles at about 80 mm static
compression per corner, approximately 2.9 kN/corner, with the body OBB remaining
clear of the road.

## Later Step 9 work

- wheel angular state and rotational inertia;
- steering geometry;
- longitudinal slip / tire force;
- lateral slip angle / tire force;
- wheel brakes;
- normal-load coupling and combined-slip path;
- drivetrain / differential;
- vehicle-specific stability and deterministic regression tests.

The existing DRIVE yaw/lateral helper remains explicitly temporary and is removed
progressively once real tire forces can steer the chassis.

## Step 9.3.1 — Low-speed steering / yaw stability hotfix

The original DRIVE helper still added direct yaw torque with a minimum authority at
very low speed, then stopped correcting yaw when steering returned to centre. Once
real suspension support arrived this became easy to feel: the car could rotate too
quickly at walking speed and retain yaw like it was driving on ice.

This hotfix replaces that open-loop torque with a temporary closed-loop yaw-rate
controller. The requested yaw rate is derived from bicycle geometry:

- wheelbase comes from the front/rear suspension mount positions;
- steering input maps to the configured maximum road-wheel angle;
- target yaw rate tends to zero with longitudinal speed, so the vehicle cannot steer
  in place;
- a provisional lateral-acceleration envelope limits unreasonable high-speed yaw;
- the controller remains active with centred steering, so residual yaw is damped
  toward zero while the front wheels remain in contact;
- lateral stabilization is scaled by total wheel contact count, preventing a single
  contacted wheel from providing full-car grip.

This remains a laboratory helper. It is intentionally replaced later by actual front
and rear tire slip-angle forces rather than becoming the final steering model.

Telemetry now reports target and actual yaw rate in rad/s.

Regression additions bring the expected suite to **58 tests**:

- wheelbase is derived from suspension geometry;
- zero longitudinal speed produces zero steering yaw target;
- yaw target scales with speed and steering sign under pure bicycle geometry.


## Step 9.3.2 — Suspension regression tolerance fix ✅

The Step 9.2/9.3 geometry tests initially used `1e-9` metre/Newton tolerances even
though suspension attachment coordinates are authored as `Vec3f`. On the real GCC
build this produced harmless differences of roughly **1.4e-8 m** and **0.002 N**.
The tests now use tolerances appropriate to the precision of their inputs
(`1e-6 m` and `1e-2 N`). No suspension equations, runtime constants or solver
behaviour changed.

## Step 9.4 — Wheel angular state ✅

Each wheel now owns persistent physics-thread state:

- angular velocity in rad/s;
- integrated rotation angle;
- rotational inertia;
- drive torque and brake torque;
- tread/circumferential speed.

The visual wheel GLBs use the published per-wheel rotation angle rather than the
old chassis-speed animation. An airborne driven wheel can therefore visibly spin
up independently of vehicle speed. Auxiliary wheel state is reset with vehicle
reset/object-manipulation discontinuities so stale wheel speed cannot survive a
teleport or `R` reset.

The OCS Lab GT baseline uses a simple **1.5 kg·m²** rotational inertia per wheel.
This is deliberately a vehicle configuration value rather than an engine-wide
constant.

## Step 9.5 — First longitudinal slip tire force ✅

Throttle no longer requests a chassis acceleration in DRIVE. The configured
post-gearing wheel torque is split across the driven wheels (RWD for the Lab GT),
then wheel angular dynamics create longitudinal slip at real contact patches.

The regularized slip ratio is:

`kappa = (omega * R - Vx) / max(|omega * R|, |Vx|, V_ref)`

where `V_ref` prevents singular behaviour around standstill. Positive slip is
traction; negative slip is braking.

The first longitudinal tire curve is intentionally compact:

`Fx = (mu * Fz) * tanh(Cx * kappa / (mu * Fz))`

This provides:

- linear low-slip stiffness `Cx`;
- smooth saturation;
- explicit normal-load coupling;
- `|Fx| <= mu * Fz`;
- zero chassis force when `Fz == 0`.

Current Lab GT baseline values:

- total driven-wheel torque: about **2200 N·m** before the debug boost;
- brake torque: about **1200 N·m per wheel**;
- longitudinal stiffness: **18 kN / unit slip**;
- peak longitudinal friction coefficient: **1.20**;
- low-speed slip reference: **2.0 m/s**.

Basic service braking now also acts through wheel torque/tire slip instead of a
direct body force. Brake balance, hydraulic model and ABS remain later work.

The temporary lateral-speed helper and bicycle yaw controller remain in place for
now; they will be removed when lateral slip-angle tire forces arrive.

### Debug / telemetry

Per wheel the telemetry window now adds:

- `omega` (rad/s);
- longitudinal ground speed `Vx`;
- slip ratio `kappa`;
- longitudinal tire force `Fx`;
- drive/brake torque.

3D debug adds a blue longitudinal-force vector at each active contact patch. It
vanishes when the tire loses road load, even if the wheel continues spinning in
the air.

### Regression coverage

Five longitudinal-wheel regressions are added, bringing the expected suite to
**63 tests**:

- pure rolling has zero longitudinal slip;
- tire force remains inside the `mu * Fz` envelope;
- an airborne driven wheel spins without propelling the chassis;
- positive driven-wheel slip applies forward contact force;
- brake torque opposes forward wheel rotation.
