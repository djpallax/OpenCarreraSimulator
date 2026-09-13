# OpenCarreraSimulator — Step 7

Linux-first C++23 racing-simulator engine built without a third-party game
engine. Step 7 introduces a persistent world/scene layer, camera-relative
rendering and a generated 500 m development straight.

## Build

```bash
rm -rf build/gcc-debug
cmake --preset gcc-debug
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

The build still compiles the textured Blender reference GLB and additionally
runs `ocs_trackgen` to produce separate visual/collision straight-track assets.

## Run

```bash
./build/gcc-debug/app/OpenCarreraSimulator --gpu 0 --stats --metrics
```

Controls:

```text
C             camera/object mode
W/S           forward/back
A/D           left/right
Q/E           down/up
Shift         faster
RMB + mouse   rotate active camera/object
```

The default world contains the 500 m straight plus the textured Blender
reference object from Step 5.1. In OBJECT mode that reference object is the
selected movable object.

Use a different source-compiled reference model with `--asset`, or a different
runtime straight/world visual with `--track`.

See [`STEP7.md`](STEP7.md) and [`ROADMAP.md`](ROADMAP.md).


## Step 8 physics laboratory

Step 8 uses a 500 Hz default fixed-step physics laboratory (`--physics-hz 1000` tests a 1 ms step).
The current default world is the Blender-authored `test_circuit.glb`, compiled independently to visual `.ocsmodel` and static-collision `.ocsmesh` resources. The laboratory rigid body uses an OBB, real corner contact impulses and can settle naturally from a tilted landing.

`C` cycles CAMERA -> OBJECT -> DRIVE. DRIVE uses `W` throttle, `S` brake and `A/D` steering. Propulsion is RWD and contact-gated per rear wheel. Four independent raycast suspensions now carry the chassis with spring/damper forces; steering/lateral response remains a temporary laboratory helper until the tire model replaces it.

When `--metrics` (or `--metrics-file`) is enabled, a second software-rendered SDL telemetry window opens with renderer, physics, collision, drive, camera and per-object state.

Step 8.10 also renders spatial physics diagnostics directly in the Vulkan scene. Press `V` to toggle COM/OBB/contact/vector debug lines without changing the simulation.

Step 8.11 runs physics on a dedicated `std::jthread` by default. Render/game state consumes synchronized snapshots and never mutates the live `PhysicsWorld`. Use `--single-thread-physics` for A/B regression testing against the previous main-thread fixed-step path. The telemetry window exposes measured physics Hz, step budget utilization, scheduler lag, snapshot age, catch-up steps, dropped ticks and deadline misses.

See `STEP8.md` for the detailed substep checklist and completion gate.


### Physics execution mode
Dedicated physics is the default. Use `--dedicated-physics` to force it explicitly. `--single-thread-physics` is retained only for A/B diagnostics and can affect frame pacing because all fixed steps run on the render/main thread.

## Step 9 vehicle prototype

The current Lab GT uses four raycast spring/damper suspensions, RWD, persistent
wheel angular state and a first load-limited longitudinal tire model. Throttle
and braking now act through wheel torque and longitudinal slip (`kappa`) rather
than direct chassis forces; airborne driven wheels can spin but cannot propel the
vehicle. The temporary bicycle yaw/lateral helper remains until lateral tire
forces are implemented.

See `STEP9.md` for the vehicle substep history and current tire/suspension values.
