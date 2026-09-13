#pragma once

#include <cstdint>
#include <filesystem>

#include "ocs/assets/model.hpp"
#include "ocs/core/result.hpp"

namespace ocs::assets {

inline constexpr std::uint32_t kModelFormatVersion = 2;

[[nodiscard]] std::filesystem::path model_texture_sidecar_path(
    const std::filesystem::path& model_path,
    std::uint32_t texture_index);

[[nodiscard]] core::Result<ModelData> load_model(const std::filesystem::path& path);
[[nodiscard]] core::Status write_model(const std::filesystem::path& path,
                                       const ModelData& model);

} // namespace ocs::assets
