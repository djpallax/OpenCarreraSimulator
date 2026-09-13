# Step 5.1 — Texture Resource Extension

## Acceptance goal

The Blender reference model must render using its embedded base-color texture while
Vulkan validation remains clean. Source image codecs remain an offline-tool concern.

## Runtime formats

### `.ocsmodel` v2
Stores geometry, submeshes, material factors, sampler metadata and texture references.
Texture references are integrity-bound to deterministic sidecars.

### `.ocstex` v1
Stores one RGBA8 2D texture with its complete mip chain. Header metadata includes
width, height, mip count, color space and FNV-1a payload hash.

Sidecars are named:

```text
<model-stem>.tex<N>.ocstex
```

Example:

```text
test1.ocsmodel
test1.tex0.ocstex
```

## Vulkan ownership

```text
ModelHandle
  +-- MeshHandle
  +-- MaterialHandle[]
  +-- TextureHandle[]
  `-- SamplerHandle[]

MaterialHandle
  `-- descriptor set -> VkImageView + VkSampler
```

A global 1x1 white fallback makes the material shader path uniform: even an
untextured material has a valid descriptor set.

## Deferred intentionally

- normal maps and tangent generation;
- metallic/roughness texture sampling;
- anisotropic filtering policy;
- compressed KTX2/Basis payloads;
- streaming and residency management;
- bindless/descriptor indexing.

Those should be introduced only when a real scene or vehicle requires them.
