# OpenCarreraSimulator — Step 5 Resource & Material Foundation

Step 5 turns the single-mesh Step 4 renderer into the first real resource/material
runtime. It deliberately prioritizes architecture and functionality over graphics
quality.

## Reference Blender asset

The default sample is the user-created `assets_src/reference/test1.glb`.

Its current structure is useful for this milestone:

- glTF 2.0 / binary GLB exported by Blender
- 1 scene / 1 node / 1 mesh / 1 primitive
- 206 vertices
- 396 indices / 132 triangles
- POSITION + NORMAL + TEXCOORD_0
- 1 glTF PBR metallic-roughness material
- base color factor approximately `(0.8, 0.8, 0.8, 1.0)`
- metallic `0.0`
- roughness `0.5`
- double-sided material
- no image textures yet

That last point is intentional: Step 5 validates resource handles, submeshes,
materials and descriptors before Step 5.1 introduces image/texture lifetime.

## Offline content flow

```text
Blender
  |
  v
GLB / glTF                 source / interchange format
  |
  v
ocs_assetc                 offline tool only
  |
  v
.ocsmodel                  runtime model container
  |
  +-- vertices / indices
  +-- submeshes
  +-- material factors
  +-- bounds
  `-- payload integrity hash
  |
  v
ocs::assets
  |
  v
VulkanResourceManager
  |
  +-- ModelHandle
  +-- MeshHandle
  `-- MaterialHandle
```

`fastgltf` and `simdjson` remain build-tool dependencies only. The simulator does
not parse Blender/glTF files at runtime.

## New runtime model format

Step 5 adds `.ocsmodel` (`OCMD`, version 1) while keeping `.ocsmesh` (`OCSM`,
version 1) readable for backwards compatibility.

`.ocsmodel` stores:

- the Step 4 32-byte vertex layout (position, normal, TEXCOORD_0)
- uint32 indices
- submesh index ranges
- material indices
- PBR factor-only material data
- model AABB
- format version / structural metadata
- payload hash

The loader rejects invalid ranges, stale material references, incompatible
headers and corrupted payloads.

## Material scope

The Step 5 material representation imports:

- `baseColorFactor`
- `metallicFactor`
- `roughnessFactor`
- `alphaCutoff`
- opaque / mask / blend metadata
- `doubleSided`
- `unlit`

The renderer uses a deliberately simple directional-light shader. It is not the
final car-paint/PBR model; the goal is to exercise material state correctly.

Alpha-mask is supported. Alpha-blend metadata is preserved but rendered through
the opaque path for now because correct transparent sorting/blending belongs to a
later rendering milestone.

## Generational resource handles

The reusable `ocs::core::SlotMap` backs the first real resource registries:

```text
ModelHandle    index + generation
MeshHandle     index + generation
MaterialHandle index + generation
```

A handle from a destroyed resource cannot silently refer to a new resource that
reused the same slot. This is the pattern that future textures, audio resources
and world assets can reuse.

## Descriptor foundation

Each frame now owns a host-visible scene uniform buffer and descriptor set.
The current scene UBO contains view-projection and camera position.

Per-draw model/material values remain push constants because the payload is small.
This gives us both descriptor and push-constant paths before textures arrive.

## Build

```bash
rm -rf build/gcc-debug
cmake --preset gcc-debug
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

The build automatically performs:

```text
assets_src/reference/test1.glb
        |
        v
     ocs_assetc
        |
        v
build/gcc-debug/assets/reference/test1.ocsmodel
```

## Run

Use the default reference model:

```bash
./build/gcc-debug/app/OpenCarreraSimulator --gpu 0 --stats --metrics
```

Load another compiled model:

```bash
./build/gcc-debug/app/OpenCarreraSimulator \
    --gpu 0 \
    --asset ./my_model.ocsmodel
```

`--model` remains an alias and `--mesh` remains accepted for legacy `.ocsmesh`
files.

Compile another Blender export manually:

```bash
./build/gcc-debug/tools/assetc/ocs_assetc \
    model.glb \
    model.ocsmodel
```

## Expected runtime resource stats

For `test1.glb`, the asset compiler stores a fallback material plus the imported
Blender material. Therefore a typical Step 5 line contains one model, one mesh and
two material resources:

```text
... | draws 1 | triangles 132 | resources 1/1/2
```

## Tests added

Step 5 adds coverage for:

- generational handle reuse and stale-handle rejection
- `.ocsmodel` round-trip
- invalid submesh/material references
- corrupted `.ocsmodel` payload rejection

The previous `.ocsmesh`, math, core and renderer-support tests remain.

## Deliberately deferred to Step 5.1

- image decoding/transcoding
- KTX2 / Basis Universal
- Vulkan image resource handles
- samplers and texture descriptors
- base-color / metallic-roughness / normal textures
- mip generation / texture streaming

This keeps Step 5 focused on the resource lifetime and model/material architecture
that those features will depend on.
