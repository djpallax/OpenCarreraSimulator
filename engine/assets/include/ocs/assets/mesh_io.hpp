#pragma once

#include <filesystem>

#include "ocs/assets/mesh.hpp"
#include "ocs/core/result.hpp"

namespace ocs::assets {

inline constexpr std::uint32_t kMeshFormatVersion = 1;

[[nodiscard]] core::Result<MeshData> load_mesh(const std::filesystem::path& path);
[[nodiscard]] core::Status write_mesh(const std::filesystem::path& path,
                                      const MeshData& mesh);

} // namespace ocs::assets
