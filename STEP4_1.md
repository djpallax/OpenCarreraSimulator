# OpenCarreraSimulator — Step 4.1 Cleanup

Step 4.1 is intentionally a maintenance milestone. It does not add rendering
features; it makes the Step 4 build and telemetry trustworthy before the resource
and material system grows.

## Changes

- Marks fetched `fastgltf` and `simdjson` targets as CMake `SYSTEM` dependencies so
  OpenCarreraSimulator's strict warning policy applies to project code without
  flooding the build with warnings from third-party headers.
- Removes deprecated `fastgltf::Options::LoadGLBBuffers` (GLB buffers are loaded by
  default in the pinned fastgltf version).
- Makes FPS mathematically consistent with the reported frame time: `fps =
  1000 / frame_ms` after smoothing.
- Splits CPU renderer timing into:
  - `cpu_total_ms`: total wall time inside `Renderer::draw_frame()`
  - `cpu_work_ms`: CPU-side renderer work excluding measured Vulkan synchronization
  - `cpu_sync_ms`: fence waits + image acquisition + present call time
  - `acquire_ms`: `vkAcquireNextImageKHR` call time
  - `present_ms`: `vkQueuePresentKHR` call time
- Keeps `gpu_ms` sourced from Vulkan timestamp queries.
- Extends CSV metrics with all timing buckets.

## Why the split matters

With VSync enabled a light scene can report roughly 16.67 ms inside `draw_frame()`
even when the CPU only performs a fraction of a millisecond of actual renderer
work. On many Vulkan presentation paths the pacing can happen in image acquisition,
present, or a fence wait. Step 4.1 measures those calls instead of labeling the
whole interval as CPU work.

`cpu_sync_ms` is a diagnostic timing bucket, not a promise that every nanosecond in
it is literally VSync sleep. Driver/compositor scheduling is implementation
dependent.

## Expected log

```text
FPS 60.00 | frame 16.67 ms | CPU work 0.xx ms | sync 16.xx ms | GPU 0.xx ms | draws 1 | triangles 12
```

During window movement the wall frame time may legitimately change, but FPS and
frame time will remain internally consistent.

## Build

```bash
rm -rf build/gcc-debug
cmake --preset gcc-debug
cmake --build --preset gcc-debug
ctest --preset gcc-debug
./build/gcc-debug/app/OpenCarreraSimulator --gpu 0 --stats --metrics
```
