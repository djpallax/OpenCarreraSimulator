# OpenCarreraSimulator — Step 2.5

Step 2.5 hardens the build and Vulkan foundation before the render-resource layer is added.

## What changes

- Keeps engine modules as explicit `STATIC` libraries.
- Adds canonical CMake aliases: `ocs::core`, `ocs::math`, `ocs::platform`, `ocs::render`.
- Adds CMake `INTERFACE` targets:
  - `ocs::project_options`
  - `ocs::project_warnings`
  - `ocs::sanitizers`
- Adds `ocs::core::ByteSize` so memory quantities are not bare integers.
- Adds `ocs::core::Result<T>` / `Status` based on C++23 `std::expected`.
- Classifies Vulkan devices as discrete/integrated/virtual/software/other.
- Stops reporting unified memory as if it were dedicated VRAM.
- Reads `VK_EXT_memory_budget` when the driver exposes it.
- Software Vulkan implementations (for example llvmpipe) are no longer auto-selected.
- Adds `--allow-software-gpu` for explicit software-renderer testing/CI.
- Reports Vulkan loader, requested API, instance API, physical-device API and engine API separately.
- Adds move-only RAII wrappers for per-frame Vulkan command pools, semaphores and fences.

## Configure and build

```bash
cmake --preset gcc-debug
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

Run:

```bash
./build/gcc-debug/app/OpenCarreraSimulator
```

Select a real GPU:

```bash
./build/gcc-debug/app/OpenCarreraSimulator --gpu 0
./build/gcc-debug/app/OpenCarreraSimulator --gpu 1
```

Explicitly test a software Vulkan device such as llvmpipe:

```bash
./build/gcc-debug/app/OpenCarreraSimulator --gpu 2 --allow-software-gpu
```

## Recommended clean rebuild

Because the target graph changes in this step, do a clean configure once:

```bash
rm -rf build/gcc-debug
cmake --preset gcc-debug
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

## Expected diagnostic style

Example only; actual values depend on the driver:

```text
[INFO] Vulkan loader 1.3.xxx | requested API 1.3.0 | instance API 1.3.0 | validation on
[INFO] GPU[0] NVIDIA ... | NVIDIA | Discrete | device API 1.4.xxx | dedicated 8192 MiB | budget ... MiB | usage ... MiB | score ...
[INFO] GPU[1] AMD ... | AMD | Integrated | device API 1.4.xxx | unified device-local ... MiB | budget ... MiB | usage ... MiB | score ...
[INFO] GPU[2] llvmpipe ... | Unknown | CPU/Software | ...
[INFO] Selected GPU: NVIDIA ... (NVIDIA, Discrete, device API 1.4.xxx, engine API 1.3.0)
```
