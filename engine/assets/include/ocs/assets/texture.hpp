#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace ocs::assets {

inline constexpr std::uint32_t kInvalidAssetIndex =
    std::numeric_limits<std::uint32_t>::max();

enum class TexturePixelFormat : std::uint8_t {
    rgba8_unorm = 1
};

enum class TextureColorSpace : std::uint8_t {
    linear = 0,
    srgb = 1
};

enum class SamplerFilter : std::uint8_t {
    nearest = 0,
    linear = 1
};

enum class SamplerMipFilter : std::uint8_t {
    nearest = 0,
    linear = 1
};

enum class SamplerAddressMode : std::uint8_t {
    repeat = 0,
    clamp_to_edge = 1,
    mirrored_repeat = 2
};

struct SamplerData {
    SamplerFilter min_filter = SamplerFilter::linear;
    SamplerFilter mag_filter = SamplerFilter::linear;
    SamplerMipFilter mip_filter = SamplerMipFilter::linear;
    SamplerAddressMode address_u = SamplerAddressMode::repeat;
    SamplerAddressMode address_v = SamplerAddressMode::repeat;
    bool use_mipmaps = true;
};

struct MaterialTextureBinding {
    std::uint32_t texture_index = kInvalidAssetIndex;
    std::uint32_t sampler_index = kInvalidAssetIndex;
    std::uint32_t texcoord_index = 0;

    [[nodiscard]] constexpr bool valid() const noexcept {
        return texture_index != kInvalidAssetIndex && sampler_index != kInvalidAssetIndex;
    }
};

struct TextureData {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t mip_levels = 0;
    TexturePixelFormat format = TexturePixelFormat::rgba8_unorm;
    TextureColorSpace color_space = TextureColorSpace::srgb;
    std::vector<std::byte> pixels{};
};

[[nodiscard]] constexpr std::uint32_t mip_extent(const std::uint32_t base,
                                                  const std::uint32_t level) noexcept {
    return std::max(1U, base >> level);
}

[[nodiscard]] constexpr std::size_t rgba8_mip_byte_size(const std::uint32_t width,
                                                         const std::uint32_t height,
                                                         const std::uint32_t level) noexcept {
    return static_cast<std::size_t>(mip_extent(width, level)) *
           static_cast<std::size_t>(mip_extent(height, level)) * 4U;
}

[[nodiscard]] constexpr std::size_t texture_expected_byte_size(const TextureData& texture) noexcept {
    std::size_t total = 0;
    for (std::uint32_t level = 0; level < texture.mip_levels; ++level) {
        total += rgba8_mip_byte_size(texture.width, texture.height, level);
    }
    return total;
}

[[nodiscard]] constexpr std::size_t texture_mip_offset(const TextureData& texture,
                                                        const std::uint32_t level) noexcept {
    std::size_t offset = 0;
    for (std::uint32_t current = 0; current < level; ++current) {
        offset += rgba8_mip_byte_size(texture.width, texture.height, current);
    }
    return offset;
}

} // namespace ocs::assets
