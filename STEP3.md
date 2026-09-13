# Step 3 — Render Foundation

## Acceptance criteria

Step 3 is complete when all of the following are true:

1. `glslc` is invoked automatically by CMake.
2. `basic.vert` and `basic.frag` are compiled to SPIR-V.
3. Vertex data uploads through a host-visible staging buffer.
4. Vertex and index data live in device-local memory.
5. The graphics pipeline uses Vulkan dynamic rendering.
6. A depth buffer is active.
7. There is one depth target per swapchain image.
8. Swapchain images are tracked with fences before per-image resources are reused.
9. The renderer draws an indexed cube.
10. The cube rotates using a C++ model-view-projection matrix.
11. Window resize recreates swapchain-dependent resources.
12. `VK_LAYER_KHRONOS_validation` reports no errors.
13. Unit tests for render math pass.
14. The application layer does not include Vulkan types.

## Build

```bash
rm -rf build/gcc-debug
cmake --preset gcc-debug
cmake --build --preset gcc-debug
ctest --preset gcc-debug
./build/gcc-debug/app/OpenCarreraSimulator --gpu 0
```

## Why dedicated allocations for now?

The Step 3 `VulkanMemoryAllocator` intentionally performs simple dedicated allocations. The abstraction exists now, but block allocation/suballocation is postponed until real asset sizes and allocation patterns can be measured.

This prevents premature allocator complexity while keeping Vulkan allocation calls centralized.

## Why depth per swapchain image?

The renderer supports multiple frames in flight. A single global depth image could be reused while a previous frame is still referencing it.

Associating a depth target with each swapchain image, plus tracking the fence last associated with each image, gives a simple and correct lifetime model for this stage.

## Why push constants?

The debug cube needs only a 64-byte MVP matrix. Push constants avoid introducing descriptor sets before there is a real camera/material resource model.

Descriptor architecture should be designed when Step 4/5 provides real meshes, textures, materials and per-frame scene data.
