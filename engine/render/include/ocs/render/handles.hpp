#pragma once

#include <compare>
#include <cstdint>
#include <limits>

namespace ocs::render {

template <typename Tag>
struct Handle {
    static constexpr std::uint32_t invalid_index = std::numeric_limits<std::uint32_t>::max();

    std::uint32_t index = invalid_index;
    std::uint32_t generation = 0;

    [[nodiscard]] constexpr bool valid() const noexcept {
        return index != invalid_index;
    }

    [[nodiscard]] constexpr explicit operator bool() const noexcept {
        return valid();
    }

    auto operator<=>(const Handle&) const = default;
};

struct BufferTag;
struct ShaderTag;
struct PipelineTag;
struct TextureTag;
struct SamplerTag;
struct MeshTag;
struct MaterialTag;
struct ModelTag;

using BufferHandle = Handle<BufferTag>;
using ShaderHandle = Handle<ShaderTag>;
using PipelineHandle = Handle<PipelineTag>;
using TextureHandle = Handle<TextureTag>;
using SamplerHandle = Handle<SamplerTag>;
using MeshHandle = Handle<MeshTag>;
using MaterialHandle = Handle<MaterialTag>;
using ModelHandle = Handle<ModelTag>;

} // namespace ocs::render
