#include "ocs/assets/mesh_io.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace ocs::assets {
namespace {

constexpr std::array<char, 4> kMagic = {'O', 'C', 'S', 'M'};
constexpr std::uint32_t kHeaderBytes = 64;
constexpr std::uint32_t kVertexStride = sizeof(MeshVertex);

struct DiskHeader {
    std::array<char, 4> magic{};
    std::uint32_t version = 0;
    std::uint64_t payload_hash = 0;
    std::uint32_t header_bytes = 0;
    std::uint32_t vertex_stride = 0;
    std::uint32_t vertex_count = 0;
    std::uint32_t index_count = 0;
    std::uint32_t flags = 0;
    std::array<float, 3> bounds_min{};
    std::array<float, 3> bounds_max{};
    std::uint32_t reserved = 0;
};

static_assert(sizeof(DiskHeader) == kHeaderBytes);

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

[[nodiscard]] std::uint64_t fnv1a_append(std::uint64_t hash,
                                         const std::span<const std::byte> bytes) noexcept {
    for (const std::byte byte : bytes) {
        hash ^= static_cast<std::uint8_t>(byte);
        hash *= kFnvPrime;
    }
    return hash;
}

[[nodiscard]] std::uint64_t payload_hash(const MeshData& mesh) noexcept {
    std::uint64_t hash = kFnvOffset;
    hash = fnv1a_append(hash, std::as_bytes(std::span{mesh.vertices}));
    hash = fnv1a_append(hash, std::as_bytes(std::span{mesh.indices}));
    return hash;
}

[[nodiscard]] core::Error io_error(std::string message) {
    return core::Error{.code = core::ErrorCode::io_error, .message = std::move(message)};
}

[[nodiscard]] core::Error invalid_data(std::string message) {
    return core::Error{.code = core::ErrorCode::invalid_data, .message = std::move(message)};
}

[[nodiscard]] bool host_is_little_endian() noexcept {
    return std::endian::native == std::endian::little;
}

} // namespace

core::Status write_mesh(const std::filesystem::path& path, const MeshData& mesh) {
    if (!host_is_little_endian()) {
        return std::unexpected(invalid_data("OCSM v1 currently requires a little-endian host"));
    }
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        return std::unexpected(invalid_data("Refusing to write an empty mesh"));
    }
    if (mesh.vertices.size() > std::numeric_limits<std::uint32_t>::max() ||
        mesh.indices.size() > std::numeric_limits<std::uint32_t>::max()) {
        return std::unexpected(invalid_data("Mesh exceeds OCSM v1 32-bit element limits"));
    }

    DiskHeader header{};
    header.magic = kMagic;
    header.version = kMeshFormatVersion;
    header.header_bytes = kHeaderBytes;
    header.vertex_stride = kVertexStride;
    header.vertex_count = static_cast<std::uint32_t>(mesh.vertices.size());
    header.index_count = static_cast<std::uint32_t>(mesh.indices.size());
    header.bounds_min = {mesh.bounds.min.x, mesh.bounds.min.y, mesh.bounds.min.z};
    header.bounds_max = {mesh.bounds.max.x, mesh.bounds.max.y, mesh.bounds.max.z};
    header.payload_hash = payload_hash(mesh);

    std::error_code error;
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            return std::unexpected(io_error("Failed to create mesh output directory: " + error.message()));
        }
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        return std::unexpected(io_error("Failed to open mesh for writing: " + path.string()));
    }

    output.write(reinterpret_cast<const char*>(&header), sizeof(header));
    output.write(reinterpret_cast<const char*>(mesh.vertices.data()),
                 static_cast<std::streamsize>(mesh.vertices.size() * sizeof(MeshVertex)));
    output.write(reinterpret_cast<const char*>(mesh.indices.data()),
                 static_cast<std::streamsize>(mesh.indices.size() * sizeof(std::uint32_t)));

    if (!output) {
        return std::unexpected(io_error("Failed while writing mesh: " + path.string()));
    }

    return {};
}

core::Result<MeshData> load_mesh(const std::filesystem::path& path) {
    if (!host_is_little_endian()) {
        return std::unexpected(invalid_data("OCSM v1 currently requires a little-endian host"));
    }

    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input) {
        return std::unexpected(io_error("Failed to open mesh: " + path.string()));
    }

    const std::streamoff file_size = input.tellg();
    if (file_size < static_cast<std::streamoff>(sizeof(DiskHeader))) {
        return std::unexpected(invalid_data("Mesh file is smaller than the OCSM header"));
    }
    input.seekg(0, std::ios::beg);

    DiskHeader header{};
    input.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!input) {
        return std::unexpected(io_error("Failed to read OCSM header"));
    }

    if (header.magic != kMagic) {
        return std::unexpected(invalid_data("Invalid OCSM magic"));
    }
    if (header.version != kMeshFormatVersion) {
        return std::unexpected(invalid_data("Unsupported OCSM mesh version"));
    }
    if (header.header_bytes != kHeaderBytes || header.vertex_stride != kVertexStride) {
        return std::unexpected(invalid_data("OCSM layout does not match this engine build"));
    }
    if (header.vertex_count == 0U || header.index_count == 0U) {
        return std::unexpected(invalid_data("OCSM contains no renderable geometry"));
    }

    const std::uint64_t vertex_bytes =
        static_cast<std::uint64_t>(header.vertex_count) * sizeof(MeshVertex);
    const std::uint64_t index_bytes =
        static_cast<std::uint64_t>(header.index_count) * sizeof(std::uint32_t);
    const std::uint64_t expected_size =
        static_cast<std::uint64_t>(header.header_bytes) + vertex_bytes + index_bytes;

    if (expected_size != static_cast<std::uint64_t>(file_size)) {
        return std::unexpected(invalid_data("OCSM file size does not match its header"));
    }

    MeshData mesh{};
    mesh.vertices.resize(header.vertex_count);
    mesh.indices.resize(header.index_count);
    mesh.bounds.min = {header.bounds_min[0], header.bounds_min[1], header.bounds_min[2]};
    mesh.bounds.max = {header.bounds_max[0], header.bounds_max[1], header.bounds_max[2]};

    input.read(reinterpret_cast<char*>(mesh.vertices.data()),
               static_cast<std::streamsize>(vertex_bytes));
    input.read(reinterpret_cast<char*>(mesh.indices.data()),
               static_cast<std::streamsize>(index_bytes));
    if (!input) {
        return std::unexpected(io_error("Failed while reading OCSM payload"));
    }

    for (const std::uint32_t index : mesh.indices) {
        if (index >= mesh.vertices.size()) {
            return std::unexpected(invalid_data("OCSM contains an out-of-range index"));
        }
    }

    if (payload_hash(mesh) != header.payload_hash) {
        return std::unexpected(invalid_data("OCSM payload hash mismatch"));
    }

    return mesh;
}

} // namespace ocs::assets
