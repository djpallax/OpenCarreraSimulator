#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "ocs/core/bytes.hpp"

namespace ocs::render {

enum class GPUType {
    discrete,
    integrated,
    virtual_gpu,
    cpu,
    other
};

[[nodiscard]] constexpr std::string_view gpu_type_name(const GPUType type) noexcept {
    switch (type) {
        case GPUType::discrete: return "Discrete";
        case GPUType::integrated: return "Integrated";
        case GPUType::virtual_gpu: return "Virtual";
        case GPUType::cpu: return "CPU/Software";
        case GPUType::other: return "Other";
    }
    return "Other";
}

struct GPUMemoryInfo {
    core::ByteSize device_local{};
    core::ByteSize dedicated{};
    core::ByteSize budget{};
    core::ByteSize usage{};
    bool unified = false;
    bool budget_available = false;
};

struct GPUCapabilities {
    bool dynamic_rendering = false;
    bool synchronization2 = false;
    bool timeline_semaphores = false;
    bool descriptor_indexing = false;
    bool mesh_shaders = false;
    bool ray_tracing = false;
    bool variable_rate_shading = false;
    bool memory_budget = false;
};

struct GPUInfo {
    std::string name;
    std::string vendor;
    std::uint32_t vendor_id = 0;
    std::uint32_t device_id = 0;
    std::uint32_t api_version = 0;
    GPUType type = GPUType::other;
    GPUMemoryInfo memory{};
    GPUCapabilities capabilities{};

    [[nodiscard]] bool is_software() const noexcept { return type == GPUType::cpu; }
};

} // namespace ocs::render
