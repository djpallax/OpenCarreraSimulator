#pragma once

#include <cstdint>

#include "ocs/core/bytes.hpp"

namespace ocs::render {

struct RendererStats {
    std::uint64_t frame_index = 0;

    // Smoothed wall-clock frame interval. FPS is always derived from this value
    // so the two metrics cannot contradict each other.
    double fps = 0.0;
    double frame_ms = 0.0;

    // CPU timing for Renderer::draw_frame(). cpu_total_ms is split into work
    // and synchronization/pacing time. The sync bucket includes Vulkan fence
    // waits, image acquisition and vkQueuePresentKHR call time.
    double cpu_total_ms = 0.0;
    double cpu_work_ms = 0.0;
    double cpu_sync_ms = 0.0;
    double acquire_ms = 0.0;
    double present_ms = 0.0;

    // Timestamp-query duration measured on the GPU.
    double gpu_ms = 0.0;

    std::uint32_t draw_calls = 0;
    std::uint64_t triangles = 0;
    std::uint32_t render_instances = 0;
    std::uint32_t debug_draw_calls = 0;
    std::uint32_t debug_lines = 0;
    std::uint32_t models = 0;
    std::uint32_t meshes = 0;
    std::uint32_t materials = 0;
    std::uint32_t textures = 0;
    std::uint32_t samplers = 0;
    core::ByteSize vertex_memory{};
    core::ByteSize index_memory{};
    core::ByteSize texture_memory{};
};

} // namespace ocs::render
