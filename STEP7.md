# Step 7 — World / Scene Foundation

Step 7 moves OpenCarreraSimulator from rendering one active model to rendering a
camera-relative world containing multiple model instances.

## Coordinate contract

- right handed;
- +X forward;
- +Y left;
- +Z up;
- meters;
- persistent world positions use `double`;
- render transforms are converted to camera-relative `float` matrices each frame.

This lets the world grow to long circuits without forcing the Vulkan path to use
double-precision vertex math.

## Controls

- `C`: toggle CAMERA / OBJECT control mode.
- `W/S`: forward/backward.
- `A/D`: left/right.
- `Q/E`: down/up.
- `Shift`: speed boost.
- hold right mouse button + move mouse: rotate camera or selected object.

In OBJECT mode Step 7 controls the `reference_object`. The generated development
track is intentionally non-movable.

## Development straight

`ocs_trackgen` generates at build time:

```text
build/.../assets/world/development_straight.ocsmodel
build/.../assets/world/development_straight_collision.ocsmesh
```

The visual road is 500 m long and 12 m wide with edge and center markings. The
collision source is a separate 2-triangle plane. Physics does not consume it yet;
the separation is established now so visual-detail growth cannot dictate the
future collision representation.

## World/render boundary

```text
World (double positions)
   |
   | extract relative to camera origin
   v
RenderScene
   |- RenderInstance(ModelHandle, float Mat4)
   `- ...
   |
   v
Renderer / Vulkan
```

The Vulkan backend no longer owns camera/object gameplay state. It receives a
`RenderScene` and a `CameraView`.

## Scope intentionally deferred

- frustum culling;
- GPU instancing/batching;
- world-file compiler;
- streaming/chunk residency;
- physics/collision use;
- timing sectors and surface physics metadata;
- object selection UI/gizmos.

Those features can now be layered on the World/RenderScene boundary without
restructuring the Vulkan backend.
