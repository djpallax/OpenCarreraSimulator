#pragma once

#include <cstdint>
#include <filesystem>

#include "ocs/assets/texture.hpp"
#include "ocs/core/result.hpp"

namespace ocs::assets {

inline constexpr std::uint32_t kTextureFormatVersion = 1;

[[nodiscard]] std::uint64_t texture_payload_hash(const TextureData& texture) noexcept;
[[nodiscard]] core::Result<TextureData> load_texture(const std::filesystem::path& path);
[[nodiscard]] core::Status write_texture(const std::filesystem::path& path,
                                         const TextureData& texture);

} // namespace ocs::assets
