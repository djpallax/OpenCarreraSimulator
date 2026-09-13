#include "ocs/assets/model_io.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <span>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "ocs/assets/texture_io.hpp"

namespace ocs::assets {
namespace {

constexpr std::array<char, 4> kMagic = {'O', 'C', 'M', 'D'};
constexpr std::uint32_t kHeaderBytes = 96;
constexpr std::uint32_t kVertexStride = sizeof(MeshVertex);
constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

struct DiskHeader {
    std::array<char, 4> magic{};
    std::uint32_t version = 0;
    std::uint64_t payload_hash = 0;
    std::uint32_t header_bytes = 0;
    std::uint32_t vertex_stride = 0;
    std::uint32_t vertex_count = 0;
    std::uint32_t index_count = 0;
    std::uint32_t submesh_count = 0;
    std::uint32_t material_count = 0;
    std::uint32_t texture_count = 0;
    std::uint32_t sampler_count = 0;
    std::uint32_t flags = 0;
    std::array<float, 3> bounds_min{};
    std::array<float, 3> bounds_max{};
    std::array<std::uint32_t, 5> reserved{};
};

struct DiskSubmesh {
    std::uint32_t first_index = 0;
    std::uint32_t index_count = 0;
    std::uint32_t material_index = 0;
    std::uint32_t reserved = 0;
};

struct DiskMaterial {
    std::array<float, 4> base_color{};
    float metallic = 1.0F;
    float roughness = 1.0F;
    float alpha_cutoff = 0.5F;
    std::uint32_t flags = 0;
    std::uint32_t base_color_texture = kInvalidAssetIndex;
    std::uint32_t base_color_sampler = kInvalidAssetIndex;
    std::uint32_t base_color_texcoord = 0;
    std::uint32_t reserved = 0;
};

struct DiskSampler {
    std::uint32_t min_filter = 0;
    std::uint32_t mag_filter = 0;
    std::uint32_t mip_filter = 0;
    std::uint32_t address_u = 0;
    std::uint32_t address_v = 0;
    std::uint32_t flags = 0;
};

struct DiskTextureRef {
    std::uint64_t payload_hash = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t mip_levels = 0;
    std::uint32_t flags = 0;
};

static_assert(sizeof(DiskHeader) == kHeaderBytes);
static_assert(sizeof(DiskSubmesh) == 16U);
static_assert(sizeof(DiskMaterial) == 48U);
static_assert(sizeof(DiskSampler) == 24U);
static_assert(sizeof(DiskTextureRef) == 24U);
static_assert(std::is_trivially_copyable_v<DiskSubmesh>);
static_assert(std::is_trivially_copyable_v<DiskMaterial>);
static_assert(std::is_trivially_copyable_v<DiskSampler>);
static_assert(std::is_trivially_copyable_v<DiskTextureRef>);

enum MaterialFlags : std::uint32_t {
    kDoubleSided = 1U << 0U,
    kUnlit = 1U << 1U,
    kAlphaMask = 1U << 2U,
    kAlphaBlend = 1U << 3U
};

enum TextureFlags : std::uint32_t {
    kTextureSrgb = 1U << 0U
};

[[nodiscard]] core::Error io_error(std::string message) {
    return core::Error{.code = core::ErrorCode::io_error, .message = std::move(message)};
}

[[nodiscard]] core::Error invalid_data(std::string message) {
    return core::Error{.code = core::ErrorCode::invalid_data, .message = std::move(message)};
}

[[nodiscard]] bool host_is_little_endian() noexcept {
    return std::endian::native == std::endian::little;
}

[[nodiscard]] std::uint64_t fnv1a_append(std::uint64_t hash,
                                         const std::span<const std::byte> bytes) noexcept {
    for (const std::byte byte : bytes) {
        hash ^= static_cast<std::uint8_t>(byte);
        hash *= kFnvPrime;
    }
    return hash;
}

[[nodiscard]] DiskMaterial to_disk(const MaterialData& material) noexcept {
    DiskMaterial disk{};
    disk.base_color = {
        material.base_color_factor.x,
        material.base_color_factor.y,
        material.base_color_factor.z,
        material.base_color_factor.w
    };
    disk.metallic = material.metallic_factor;
    disk.roughness = material.roughness_factor;
    disk.alpha_cutoff = material.alpha_cutoff;
    if (material.double_sided) {
        disk.flags |= kDoubleSided;
    }
    if (material.unlit) {
        disk.flags |= kUnlit;
    }
    if (material.alpha_mode == MaterialAlphaMode::mask) {
        disk.flags |= kAlphaMask;
    } else if (material.alpha_mode == MaterialAlphaMode::blend) {
        disk.flags |= kAlphaBlend;
    }
    disk.base_color_texture = material.base_color_texture.texture_index;
    disk.base_color_sampler = material.base_color_texture.sampler_index;
    disk.base_color_texcoord = material.base_color_texture.texcoord_index;
    return disk;
}

[[nodiscard]] MaterialData from_disk(const DiskMaterial& disk) noexcept {
    MaterialData material{};
    material.base_color_factor = {
        disk.base_color[0], disk.base_color[1], disk.base_color[2], disk.base_color[3]
    };
    material.metallic_factor = disk.metallic;
    material.roughness_factor = disk.roughness;
    material.alpha_cutoff = disk.alpha_cutoff;
    material.double_sided = (disk.flags & kDoubleSided) != 0U;
    material.unlit = (disk.flags & kUnlit) != 0U;
    material.alpha_mode = (disk.flags & kAlphaBlend) != 0U
        ? MaterialAlphaMode::blend
        : ((disk.flags & kAlphaMask) != 0U ? MaterialAlphaMode::mask
                                           : MaterialAlphaMode::opaque);
    material.base_color_texture = {
        .texture_index = disk.base_color_texture,
        .sampler_index = disk.base_color_sampler,
        .texcoord_index = disk.base_color_texcoord
    };
    return material;
}

[[nodiscard]] DiskSampler to_disk(const SamplerData& sampler) noexcept {
    return {
        .min_filter = static_cast<std::uint32_t>(sampler.min_filter),
        .mag_filter = static_cast<std::uint32_t>(sampler.mag_filter),
        .mip_filter = static_cast<std::uint32_t>(sampler.mip_filter),
        .address_u = static_cast<std::uint32_t>(sampler.address_u),
        .address_v = static_cast<std::uint32_t>(sampler.address_v),
        .flags = sampler.use_mipmaps ? 1U : 0U
    };
}

[[nodiscard]] SamplerData from_disk(const DiskSampler& disk) noexcept {
    return {
        .min_filter = static_cast<SamplerFilter>(disk.min_filter),
        .mag_filter = static_cast<SamplerFilter>(disk.mag_filter),
        .mip_filter = static_cast<SamplerMipFilter>(disk.mip_filter),
        .address_u = static_cast<SamplerAddressMode>(disk.address_u),
        .address_v = static_cast<SamplerAddressMode>(disk.address_v),
        .use_mipmaps = (disk.flags & 1U) != 0U
    };
}

[[nodiscard]] std::vector<DiskSubmesh> disk_submeshes(const ModelData& model) {
    std::vector<DiskSubmesh> result;
    result.reserve(model.submeshes.size());
    for (const SubmeshData& submesh : model.submeshes) {
        result.push_back({
            .first_index = submesh.first_index,
            .index_count = submesh.index_count,
            .material_index = submesh.material_index,
            .reserved = 0
        });
    }
    return result;
}

[[nodiscard]] std::vector<DiskMaterial> disk_materials(const ModelData& model) {
    std::vector<DiskMaterial> result;
    result.reserve(model.materials.size());
    for (const MaterialData& material : model.materials) {
        result.push_back(to_disk(material));
    }
    return result;
}

[[nodiscard]] std::vector<DiskSampler> disk_samplers(const ModelData& model) {
    std::vector<DiskSampler> result;
    result.reserve(model.samplers.size());
    for (const SamplerData& sampler : model.samplers) {
        result.push_back(to_disk(sampler));
    }
    return result;
}

[[nodiscard]] std::vector<DiskTextureRef> disk_textures(const ModelData& model) {
    std::vector<DiskTextureRef> result;
    result.reserve(model.textures.size());
    for (const TextureData& texture : model.textures) {
        result.push_back({
            .payload_hash = texture_payload_hash(texture),
            .width = texture.width,
            .height = texture.height,
            .mip_levels = texture.mip_levels,
            .flags = texture.color_space == TextureColorSpace::srgb ? kTextureSrgb : 0U
        });
    }
    return result;
}

[[nodiscard]] std::uint64_t payload_hash(const ModelData& model,
                                         const std::span<const DiskSubmesh> submeshes,
                                         const std::span<const DiskMaterial> materials,
                                         const std::span<const DiskSampler> samplers,
                                         const std::span<const DiskTextureRef> textures) noexcept {
    std::uint64_t hash = kFnvOffset;
    hash = fnv1a_append(hash, std::as_bytes(std::span{model.vertices}));
    hash = fnv1a_append(hash, std::as_bytes(std::span{model.indices}));
    hash = fnv1a_append(hash, std::as_bytes(submeshes));
    hash = fnv1a_append(hash, std::as_bytes(materials));
    hash = fnv1a_append(hash, std::as_bytes(samplers));
    hash = fnv1a_append(hash, std::as_bytes(textures));
    return hash;
}

[[nodiscard]] bool valid_sampler_enum(const SamplerData& sampler) noexcept {
    return static_cast<std::uint32_t>(sampler.min_filter) <=
               static_cast<std::uint32_t>(SamplerFilter::linear) &&
           static_cast<std::uint32_t>(sampler.mag_filter) <=
               static_cast<std::uint32_t>(SamplerFilter::linear) &&
           static_cast<std::uint32_t>(sampler.mip_filter) <=
               static_cast<std::uint32_t>(SamplerMipFilter::linear) &&
           static_cast<std::uint32_t>(sampler.address_u) <=
               static_cast<std::uint32_t>(SamplerAddressMode::mirrored_repeat) &&
           static_cast<std::uint32_t>(sampler.address_v) <=
               static_cast<std::uint32_t>(SamplerAddressMode::mirrored_repeat);
}

[[nodiscard]] core::Status validate_model(const ModelData& model) {
    if (model.vertices.empty() || model.indices.empty() || model.submeshes.empty()) {
        return std::unexpected(invalid_data("OCMD requires vertices, indices and at least one submesh"));
    }
    if (model.materials.empty()) {
        return std::unexpected(invalid_data("OCMD requires at least the default material"));
    }
    if (model.vertices.size() > std::numeric_limits<std::uint32_t>::max() ||
        model.indices.size() > std::numeric_limits<std::uint32_t>::max() ||
        model.submeshes.size() > std::numeric_limits<std::uint32_t>::max() ||
        model.materials.size() > std::numeric_limits<std::uint32_t>::max() ||
        model.textures.size() > std::numeric_limits<std::uint32_t>::max() ||
        model.samplers.size() > std::numeric_limits<std::uint32_t>::max()) {
        return std::unexpected(invalid_data("OCMD v2 count exceeds 32-bit limits"));
    }

    for (const std::uint32_t index : model.indices) {
        if (index >= model.vertices.size()) {
            return std::unexpected(invalid_data("OCMD contains an out-of-range vertex index"));
        }
    }

    for (const SubmeshData& submesh : model.submeshes) {
        const std::uint64_t end = static_cast<std::uint64_t>(submesh.first_index) +
                                  static_cast<std::uint64_t>(submesh.index_count);
        if (submesh.index_count == 0U || end > model.indices.size()) {
            return std::unexpected(invalid_data("OCMD submesh index range is invalid"));
        }
        if ((submesh.index_count % 3U) != 0U) {
            return std::unexpected(invalid_data("OCMD submesh is not a triangle list"));
        }
        if (submesh.material_index >= model.materials.size()) {
            return std::unexpected(invalid_data("OCMD submesh refers to an invalid material"));
        }
    }

    for (const SamplerData& sampler : model.samplers) {
        if (!valid_sampler_enum(sampler)) {
            return std::unexpected(invalid_data("OCMD contains an unsupported sampler enum value"));
        }
    }

    for (const TextureData& texture : model.textures) {
        if (texture.width == 0U || texture.height == 0U || texture.mip_levels == 0U ||
            texture.format != TexturePixelFormat::rgba8_unorm ||
            texture.pixels.size() != texture_expected_byte_size(texture)) {
            return std::unexpected(invalid_data("OCMD contains invalid texture metadata/payload"));
        }
    }

    for (const MaterialData& material : model.materials) {
        const MaterialTextureBinding& binding = material.base_color_texture;
        if (!binding.valid()) {
            const bool both_invalid = binding.texture_index == kInvalidAssetIndex &&
                                      binding.sampler_index == kInvalidAssetIndex;
            if (!both_invalid) {
                return std::unexpected(invalid_data("OCMD material has a partial texture binding"));
            }
            continue;
        }
        if (binding.texture_index >= model.textures.size() ||
            binding.sampler_index >= model.samplers.size()) {
            return std::unexpected(invalid_data("OCMD material texture binding is out of range"));
        }
        if (binding.texcoord_index != 0U) {
            return std::unexpected(invalid_data("OCMD v2 currently supports TEXCOORD_0 only"));
        }
    }
    return {};
}

} // namespace

std::filesystem::path model_texture_sidecar_path(const std::filesystem::path& model_path,
                                                 const std::uint32_t texture_index) {
    const std::string filename = model_path.stem().string() + ".tex" +
                                 std::to_string(texture_index) + ".ocstex";
    return model_path.parent_path() / filename;
}

core::Status write_model(const std::filesystem::path& path, const ModelData& model) {
    if (!host_is_little_endian()) {
        return std::unexpected(invalid_data("OCMD v2 currently requires a little-endian host"));
    }
    if (auto status = validate_model(model); !status) {
        return status;
    }

    std::error_code error;
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            return std::unexpected(io_error("Failed to create model output directory: " + error.message()));
        }
    }

    // Write texture sidecars first. The model file is committed last so a valid
    // OCMD never references sidecars that failed to build in the same invocation.
    for (std::uint32_t index = 0; index < model.textures.size(); ++index) {
        const auto texture_path = model_texture_sidecar_path(path, index);
        if (auto status = write_texture(texture_path, model.textures[index]); !status) {
            return std::unexpected(status.error());
        }
    }

    const auto submeshes = disk_submeshes(model);
    const auto materials = disk_materials(model);
    const auto samplers = disk_samplers(model);
    const auto textures = disk_textures(model);

    DiskHeader header{};
    header.magic = kMagic;
    header.version = kModelFormatVersion;
    header.header_bytes = kHeaderBytes;
    header.vertex_stride = kVertexStride;
    header.vertex_count = static_cast<std::uint32_t>(model.vertices.size());
    header.index_count = static_cast<std::uint32_t>(model.indices.size());
    header.submesh_count = static_cast<std::uint32_t>(submeshes.size());
    header.material_count = static_cast<std::uint32_t>(materials.size());
    header.texture_count = static_cast<std::uint32_t>(textures.size());
    header.sampler_count = static_cast<std::uint32_t>(samplers.size());
    header.bounds_min = {model.bounds.min.x, model.bounds.min.y, model.bounds.min.z};
    header.bounds_max = {model.bounds.max.x, model.bounds.max.y, model.bounds.max.z};
    header.payload_hash = payload_hash(model, submeshes, materials, samplers, textures);

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        return std::unexpected(io_error("Failed to open model for writing: " + path.string()));
    }

    output.write(reinterpret_cast<const char*>(&header), sizeof(header));
    output.write(reinterpret_cast<const char*>(model.vertices.data()),
                 static_cast<std::streamsize>(model.vertices.size() * sizeof(MeshVertex)));
    output.write(reinterpret_cast<const char*>(model.indices.data()),
                 static_cast<std::streamsize>(model.indices.size() * sizeof(std::uint32_t)));
    output.write(reinterpret_cast<const char*>(submeshes.data()),
                 static_cast<std::streamsize>(submeshes.size() * sizeof(DiskSubmesh)));
    output.write(reinterpret_cast<const char*>(materials.data()),
                 static_cast<std::streamsize>(materials.size() * sizeof(DiskMaterial)));
    output.write(reinterpret_cast<const char*>(samplers.data()),
                 static_cast<std::streamsize>(samplers.size() * sizeof(DiskSampler)));
    output.write(reinterpret_cast<const char*>(textures.data()),
                 static_cast<std::streamsize>(textures.size() * sizeof(DiskTextureRef)));

    if (!output) {
        return std::unexpected(io_error("Failed while writing model: " + path.string()));
    }
    return {};
}

core::Result<ModelData> load_model(const std::filesystem::path& path) {
    if (!host_is_little_endian()) {
        return std::unexpected(invalid_data("OCMD v2 currently requires a little-endian host"));
    }

    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input) {
        return std::unexpected(io_error("Failed to open model: " + path.string()));
    }
    const std::streamoff file_size = input.tellg();
    if (file_size < static_cast<std::streamoff>(sizeof(DiskHeader))) {
        return std::unexpected(invalid_data("Model file is smaller than the OCMD header"));
    }
    input.seekg(0, std::ios::beg);

    DiskHeader header{};
    input.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!input) {
        return std::unexpected(io_error("Failed to read OCMD header"));
    }
    if (header.magic != kMagic) {
        return std::unexpected(invalid_data("File is not an OCMD runtime model"));
    }
    if (header.version != kModelFormatVersion) {
        return std::unexpected(invalid_data(
            "Unsupported OCMD version; rebuild source assets with the current ocs_assetc"));
    }
    if (header.header_bytes != kHeaderBytes || header.vertex_stride != kVertexStride) {
        return std::unexpected(invalid_data("Incompatible OCMD v2 header layout"));
    }
    if (header.vertex_count == 0U || header.index_count == 0U ||
        header.submesh_count == 0U || header.material_count == 0U) {
        return std::unexpected(invalid_data("OCMD contains empty required sections"));
    }

    const std::uint64_t vertex_bytes = static_cast<std::uint64_t>(header.vertex_count) * sizeof(MeshVertex);
    const std::uint64_t index_bytes = static_cast<std::uint64_t>(header.index_count) * sizeof(std::uint32_t);
    const std::uint64_t submesh_bytes = static_cast<std::uint64_t>(header.submesh_count) * sizeof(DiskSubmesh);
    const std::uint64_t material_bytes = static_cast<std::uint64_t>(header.material_count) * sizeof(DiskMaterial);
    const std::uint64_t sampler_bytes = static_cast<std::uint64_t>(header.sampler_count) * sizeof(DiskSampler);
    const std::uint64_t texture_ref_bytes = static_cast<std::uint64_t>(header.texture_count) * sizeof(DiskTextureRef);
    const std::uint64_t expected_size = static_cast<std::uint64_t>(header.header_bytes) +
                                        vertex_bytes + index_bytes + submesh_bytes + material_bytes +
                                        sampler_bytes + texture_ref_bytes;
    if (expected_size != static_cast<std::uint64_t>(file_size)) {
        return std::unexpected(invalid_data("OCMD file size does not match its header"));
    }

    ModelData model{};
    model.vertices.resize(header.vertex_count);
    model.indices.resize(header.index_count);
    std::vector<DiskSubmesh> disk_submesh_data(header.submesh_count);
    std::vector<DiskMaterial> disk_material_data(header.material_count);
    std::vector<DiskSampler> disk_sampler_data(header.sampler_count);
    std::vector<DiskTextureRef> disk_texture_data(header.texture_count);

    input.read(reinterpret_cast<char*>(model.vertices.data()), static_cast<std::streamsize>(vertex_bytes));
    input.read(reinterpret_cast<char*>(model.indices.data()), static_cast<std::streamsize>(index_bytes));
    input.read(reinterpret_cast<char*>(disk_submesh_data.data()), static_cast<std::streamsize>(submesh_bytes));
    input.read(reinterpret_cast<char*>(disk_material_data.data()), static_cast<std::streamsize>(material_bytes));
    input.read(reinterpret_cast<char*>(disk_sampler_data.data()), static_cast<std::streamsize>(sampler_bytes));
    input.read(reinterpret_cast<char*>(disk_texture_data.data()), static_cast<std::streamsize>(texture_ref_bytes));
    if (!input) {
        return std::unexpected(io_error("Failed while reading OCMD payload"));
    }

    model.bounds.min = {header.bounds_min[0], header.bounds_min[1], header.bounds_min[2]};
    model.bounds.max = {header.bounds_max[0], header.bounds_max[1], header.bounds_max[2]};

    model.submeshes.reserve(disk_submesh_data.size());
    for (const DiskSubmesh& disk : disk_submesh_data) {
        model.submeshes.push_back({disk.first_index, disk.index_count, disk.material_index});
    }
    model.materials.reserve(disk_material_data.size());
    for (const DiskMaterial& disk : disk_material_data) {
        model.materials.push_back(from_disk(disk));
    }
    model.samplers.reserve(disk_sampler_data.size());
    for (const DiskSampler& disk : disk_sampler_data) {
        model.samplers.push_back(from_disk(disk));
    }

    if (payload_hash(model, disk_submesh_data, disk_material_data, disk_sampler_data,
                     disk_texture_data) != header.payload_hash) {
        return std::unexpected(invalid_data("OCMD payload hash mismatch"));
    }

    model.textures.reserve(disk_texture_data.size());
    for (std::uint32_t index = 0; index < header.texture_count; ++index) {
        const auto sidecar_path = model_texture_sidecar_path(path, index);
        auto texture = load_texture(sidecar_path);
        if (!texture) {
            return std::unexpected(core::Error{
                .code = texture.error().code,
                .message = "Failed to load model texture sidecar '" + sidecar_path.string() +
                           "': " + texture.error().message
            });
        }

        const DiskTextureRef& reference = disk_texture_data[index];
        const bool expected_srgb = (reference.flags & kTextureSrgb) != 0U;
        if (texture->width != reference.width || texture->height != reference.height ||
            texture->mip_levels != reference.mip_levels ||
            (texture->color_space == TextureColorSpace::srgb) != expected_srgb ||
            texture_payload_hash(*texture) != reference.payload_hash) {
            return std::unexpected(invalid_data(
                "OCMD texture sidecar does not match the model texture reference"));
        }
        model.textures.push_back(std::move(*texture));
    }

    if (auto status = validate_model(model); !status) {
        return std::unexpected(status.error());
    }
    return model;
}

} // namespace ocs::assets
