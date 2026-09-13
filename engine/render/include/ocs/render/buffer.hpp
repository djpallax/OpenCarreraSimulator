#pragma once

#include <cstddef>
#include <cstdint>

namespace ocs::render {

enum class BufferUsage : std::uint8_t {
    vertex,
    index,
    uniform,
    storage,
    staging
};

struct BufferDesc {
    std::size_t size = 0;
    BufferUsage usage = BufferUsage::vertex;
};

} // namespace ocs::render
