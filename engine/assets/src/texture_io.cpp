#include "ocs/assets/texture_io.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <span>
#include <string>
#include <utility>

namespace ocs::assets {
namespace {

constexpr std::array<char, 4> kMagic = {'O', 'C', 'T', 'X'};
constexpr std::uint32_t kHeaderBytes = 64;
constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

struct DiskHeader {
    std::array<char, 4> magic{};
    std::uint32_t version = 0;
    std::uint64_t payload_hash = 0;
    std::uint32_t header_bytes = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t mip_levels = 0;
    std::uint32_t format = 0;
    std::uint32_t color_space = 0;
    std::uint64_t payload_bytes = 0;
    std::array<std::uint32_t, 4> reserved{};
};

static_assert(sizeof(DiskHeader) == kHeaderBytes);

[[nodiscard]] core::Error io_error(std::string message) {
    return core::Error{.code = core::ErrorCode::io_error, .message = std::move(message)};
}

[[nodiscard]] core::Error invalid_data(std::string message) {
    return core::Error{.code = core::ErrorCode::invalid_data, .message = std::move(message)};
}

[[nodiscard]] bool host_is_little_endian() noexcept {
    return std::endian::native == std::endian::little;
}

[[nodiscard]] std::uint64_t fnv1a(const std::span<const std::byte> bytes) noexcept {
    std::uint64_t hash = kFnvOffset;
    for (const std::byte byte : bytes) {
        hash ^= static_cast<std::uint8_t>(byte);
        hash *= kFnvPrime;
    }
    return hash;
}

[[nodiscard]] core::Status validate_texture(const TextureData& texture) {
    if (texture.width == 0U || texture.height == 0U || texture.mip_levels == 0U) {
        return std::unexpected(invalid_data("OCTX requires non-zero dimensions and mip count"));
    }
    if (texture.format != TexturePixelFormat::rgba8_unorm) {
        return std::unexpected(invalid_data("OCTX v1 only supports RGBA8 textures"));
    }

    std::uint32_t max_mips = 1U;
    for (std::uint32_t extent = std::max(texture.width, texture.height); extent > 1U; extent >>= 1U) {
        ++max_mips;
    }
    if (texture.mip_levels > max_mips) {
        return std::unexpected(invalid_data("OCTX mip count exceeds the image dimensions"));
    }

    const std::size_t expected = texture_expected_byte_size(texture);
    if (texture.pixels.size() != expected) {
        return std::unexpected(invalid_data("OCTX RGBA8 payload size does not match dimensions/mips"));
    }
    return {};
}

} // namespace

std::uint64_t texture_payload_hash(const TextureData& texture) noexcept {
    return fnv1a(std::span<const std::byte>{texture.pixels.data(), texture.pixels.size()});
}

core::Status write_texture(const std::filesystem::path& path, const TextureData& texture) {
    if (!host_is_little_endian()) {
        return std::unexpected(invalid_data("OCTX v1 currently requires a little-endian host"));
    }
    if (auto status = validate_texture(texture); !status) {
        return status;
    }

    std::error_code error;
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            return std::unexpected(io_error("Failed to create texture output directory: " + error.message()));
        }
    }

    DiskHeader header{};
    header.magic = kMagic;
    header.version = kTextureFormatVersion;
    header.payload_hash = texture_payload_hash(texture);
    header.header_bytes = kHeaderBytes;
    header.width = texture.width;
    header.height = texture.height;
    header.mip_levels = texture.mip_levels;
    header.format = static_cast<std::uint32_t>(texture.format);
    header.color_space = static_cast<std::uint32_t>(texture.color_space);
    header.payload_bytes = static_cast<std::uint64_t>(texture.pixels.size());

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        return std::unexpected(io_error("Failed to open texture for writing: " + path.string()));
    }

    output.write(reinterpret_cast<const char*>(&header), sizeof(header));
    output.write(reinterpret_cast<const char*>(texture.pixels.data()),
                 static_cast<std::streamsize>(texture.pixels.size()));
    if (!output) {
        return std::unexpected(io_error("Failed while writing texture: " + path.string()));
    }
    return {};
}

core::Result<TextureData> load_texture(const std::filesystem::path& path) {
    if (!host_is_little_endian()) {
        return std::unexpected(invalid_data("OCTX v1 currently requires a little-endian host"));
    }

    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input) {
        return std::unexpected(io_error("Failed to open texture: " + path.string()));
    }
    const std::streamoff file_size = input.tellg();
    if (file_size < static_cast<std::streamoff>(sizeof(DiskHeader))) {
        return std::unexpected(invalid_data("Texture file is smaller than the OCTX header"));
    }
    input.seekg(0, std::ios::beg);

    DiskHeader header{};
    input.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!input) {
        return std::unexpected(io_error("Failed to read OCTX header"));
    }
    if (header.magic != kMagic || header.version != kTextureFormatVersion ||
        header.header_bytes != kHeaderBytes ||
        header.format != static_cast<std::uint32_t>(TexturePixelFormat::rgba8_unorm) ||
        header.color_space > static_cast<std::uint32_t>(TextureColorSpace::srgb)) {
        return std::unexpected(invalid_data("Unsupported or incompatible OCTX header"));
    }
    if (header.payload_bytes > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
        return std::unexpected(invalid_data("OCTX payload is too large for this host"));
    }

    const std::uint64_t expected_file_size =
        static_cast<std::uint64_t>(header.header_bytes) + header.payload_bytes;
    if (expected_file_size != static_cast<std::uint64_t>(file_size)) {
        return std::unexpected(invalid_data("OCTX file size does not match its header"));
    }

    TextureData texture{};
    texture.width = header.width;
    texture.height = header.height;
    texture.mip_levels = header.mip_levels;
    texture.format = static_cast<TexturePixelFormat>(header.format);
    texture.color_space = static_cast<TextureColorSpace>(header.color_space);
    texture.pixels.resize(static_cast<std::size_t>(header.payload_bytes));
    input.read(reinterpret_cast<char*>(texture.pixels.data()),
               static_cast<std::streamsize>(texture.pixels.size()));
    if (!input) {
        return std::unexpected(io_error("Failed while reading OCTX payload"));
    }

    if (auto status = validate_texture(texture); !status) {
        return std::unexpected(status.error());
    }
    if (texture_payload_hash(texture) != header.payload_hash) {
        return std::unexpected(invalid_data("OCTX payload hash mismatch"));
    }
    return texture;
}

} // namespace ocs::assets
