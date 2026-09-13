#include <fastgltf/core.hpp>
#include <fastgltf/math.hpp>
#include <fastgltf/tools.hpp>
#include <fastgltf/types.hpp>
#include <stb_image.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

#include "ocs/assets/mesh_io.hpp"
#include "ocs/assets/model_io.hpp"
#include "ocs/assets/texture.hpp"
#include "ocs/math/vec2f.hpp"
#include "ocs/math/vec3f.hpp"

namespace {

using ocs::assets::MaterialAlphaMode;
using ocs::assets::MaterialData;
using ocs::assets::MeshData;
using ocs::assets::MeshVertex;
using ocs::assets::ModelData;
using ocs::assets::SubmeshData;
using ocs::assets::TextureColorSpace;
using ocs::assets::TextureData;
using ocs::assets::SamplerAddressMode;
using ocs::assets::SamplerData;
using ocs::assets::SamplerFilter;
using ocs::assets::SamplerMipFilter;
using ocs::math::Vec2f;
using ocs::math::Vec3f;

struct Mat3 {
    std::array<float, 9> m{};

    [[nodiscard]] float& at(const std::size_t row, const std::size_t column) noexcept {
        return m[row * 3U + column];
    }
    [[nodiscard]] float at(const std::size_t row, const std::size_t column) const noexcept {
        return m[row * 3U + column];
    }
};

[[nodiscard]] Vec3f gltf_to_engine(const Vec3f value) noexcept {
    // glTF: +Y up, +Z forward, +X left.
    // OCS:  +Z up, +X forward, +Y left.
    // This cyclic permutation preserves handedness and winding.
    return {value.z, value.x, value.y};
}

[[nodiscard]] Vec3f transform_point(const fastgltf::math::fmat4x4& matrix,
                                    const fastgltf::math::fvec3& value) noexcept {
    const float x = value.x();
    const float y = value.y();
    const float z = value.z();
    return {
        matrix[0][0] * x + matrix[1][0] * y + matrix[2][0] * z + matrix[3][0],
        matrix[0][1] * x + matrix[1][1] * y + matrix[2][1] * z + matrix[3][1],
        matrix[0][2] * x + matrix[1][2] * y + matrix[2][2] * z + matrix[3][2]
    };
}

[[nodiscard]] std::optional<Mat3> inverse_transpose_linear(
    const fastgltf::math::fmat4x4& matrix) noexcept {
    const float a00 = matrix[0][0];
    const float a01 = matrix[1][0];
    const float a02 = matrix[2][0];
    const float a10 = matrix[0][1];
    const float a11 = matrix[1][1];
    const float a12 = matrix[2][1];
    const float a20 = matrix[0][2];
    const float a21 = matrix[1][2];
    const float a22 = matrix[2][2];

    const float c00 = a11 * a22 - a12 * a21;
    const float c01 = -(a10 * a22 - a12 * a20);
    const float c02 = a10 * a21 - a11 * a20;
    const float c10 = -(a01 * a22 - a02 * a21);
    const float c11 = a00 * a22 - a02 * a20;
    const float c12 = -(a00 * a21 - a01 * a20);
    const float c20 = a01 * a12 - a02 * a11;
    const float c21 = -(a00 * a12 - a02 * a10);
    const float c22 = a00 * a11 - a01 * a10;

    const float determinant = a00 * c00 + a01 * c01 + a02 * c02;
    if (std::abs(determinant) < 1.0e-8F) {
        return std::nullopt;
    }

    const float inv_det = 1.0F / determinant;
    Mat3 result{};
    result.at(0, 0) = c00 * inv_det;
    result.at(0, 1) = c01 * inv_det;
    result.at(0, 2) = c02 * inv_det;
    result.at(1, 0) = c10 * inv_det;
    result.at(1, 1) = c11 * inv_det;
    result.at(1, 2) = c12 * inv_det;
    result.at(2, 0) = c20 * inv_det;
    result.at(2, 1) = c21 * inv_det;
    result.at(2, 2) = c22 * inv_det;
    return result;
}

[[nodiscard]] Vec3f transform_normal(const Mat3& matrix,
                                     const fastgltf::math::fvec3& value) noexcept {
    const Vec3f gltf_normal{
        matrix.at(0, 0) * value.x() + matrix.at(0, 1) * value.y() + matrix.at(0, 2) * value.z(),
        matrix.at(1, 0) * value.x() + matrix.at(1, 1) * value.y() + matrix.at(1, 2) * value.z(),
        matrix.at(2, 0) * value.x() + matrix.at(2, 1) * value.y() + matrix.at(2, 2) * value.z()
    };
    return gltf_to_engine(gltf_normal).normalized();
}

void extend_bounds(ModelData& model, const Vec3f position, bool& initialized) noexcept {
    if (!initialized) {
        model.bounds.min = position;
        model.bounds.max = position;
        initialized = true;
        return;
    }
    model.bounds.min.x = std::min(model.bounds.min.x, position.x);
    model.bounds.min.y = std::min(model.bounds.min.y, position.y);
    model.bounds.min.z = std::min(model.bounds.min.z, position.z);
    model.bounds.max.x = std::max(model.bounds.max.x, position.x);
    model.bounds.max.y = std::max(model.bounds.max.y, position.y);
    model.bounds.max.z = std::max(model.bounds.max.z, position.z);
}

void generate_normals(ModelData& model,
                      const std::size_t base_vertex,
                      const std::size_t first_index,
                      const std::size_t index_count) {
    for (std::size_t index = 0; index + 2U < index_count; index += 3U) {
        const std::uint32_t i0 = model.indices[first_index + index + 0U];
        const std::uint32_t i1 = model.indices[first_index + index + 1U];
        const std::uint32_t i2 = model.indices[first_index + index + 2U];
        const Vec3f p0 = model.vertices[i0].position;
        const Vec3f p1 = model.vertices[i1].position;
        const Vec3f p2 = model.vertices[i2].position;
        const Vec3f normal = (p1 - p0).cross(p2 - p0);
        model.vertices[i0].normal = model.vertices[i0].normal + normal;
        model.vertices[i1].normal = model.vertices[i1].normal + normal;
        model.vertices[i2].normal = model.vertices[i2].normal + normal;
    }

    for (std::size_t index = base_vertex; index < model.vertices.size(); ++index) {
        const Vec3f normal = model.vertices[index].normal.normalized();
        model.vertices[index].normal = normal.length_squared() > 0.0F
            ? normal
            : Vec3f{0.0F, 0.0F, 1.0F};
    }
}

[[nodiscard]] MaterialAlphaMode alpha_mode(const fastgltf::AlphaMode mode) noexcept {
    switch (mode) {
    case fastgltf::AlphaMode::Mask:
        return MaterialAlphaMode::mask;
    case fastgltf::AlphaMode::Blend:
        return MaterialAlphaMode::blend;
    case fastgltf::AlphaMode::Opaque:
    default:
        return MaterialAlphaMode::opaque;
    }
}

template <typename... Ts>
struct Overloaded : Ts... {
    using Ts::operator()...;
};
template <typename... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

[[nodiscard]] std::optional<std::span<const std::byte>> encoded_image_bytes(
    const fastgltf::Asset& asset,
    const fastgltf::Image& image) {
    return std::visit(
        Overloaded{
            [](const fastgltf::sources::Array& source)
                -> std::optional<std::span<const std::byte>> {
                return std::span<const std::byte>{source.bytes.data(), source.bytes.size()};
            },
            [](const fastgltf::sources::Vector& source)
                -> std::optional<std::span<const std::byte>> {
                return std::span<const std::byte>{source.bytes.data(), source.bytes.size()};
            },
            [](const fastgltf::sources::ByteView& source)
                -> std::optional<std::span<const std::byte>> {
                return std::span<const std::byte>{source.bytes.data(), source.bytes.size()};
            },
            [&](const fastgltf::sources::BufferView& source)
                -> std::optional<std::span<const std::byte>> {
                if (source.bufferViewIndex >= asset.bufferViews.size()) {
                    return std::nullopt;
                }
                const auto data = fastgltf::DefaultBufferDataAdapter{}(
                    asset, source.bufferViewIndex);
                return std::span<const std::byte>{data.data(), data.size()};
            },
            [](const auto&) -> std::optional<std::span<const std::byte>> {
                return std::nullopt;
            }},
        image.data);
}

void append_bytes(std::vector<std::byte>& destination,
                  const std::span<const std::uint8_t> bytes) {
    const std::size_t offset = destination.size();
    destination.resize(offset + bytes.size());
    std::memcpy(destination.data() + offset, bytes.data(), bytes.size());
}

[[nodiscard]] TextureData make_mipped_rgba8(const std::uint32_t width,
                                             const std::uint32_t height,
                                             const std::span<const std::uint8_t> rgba,
                                             const TextureColorSpace color_space) {
    TextureData texture{};
    texture.width = width;
    texture.height = height;
    texture.color_space = color_space;

    std::uint32_t mip_levels = 1U;
    for (std::uint32_t extent = std::max(width, height); extent > 1U; extent >>= 1U) {
        ++mip_levels;
    }
    texture.mip_levels = mip_levels;
    texture.pixels.reserve(static_cast<std::size_t>(width) * height * 4U * 4U / 3U + 16U);

    std::vector<std::uint8_t> previous(rgba.begin(), rgba.end());
    std::uint32_t previous_width = width;
    std::uint32_t previous_height = height;
    append_bytes(texture.pixels, std::span<const std::uint8_t>{previous.data(), previous.size()});

    for (std::uint32_t level = 1; level < mip_levels; ++level) {
        const std::uint32_t next_width = std::max(1U, previous_width / 2U);
        const std::uint32_t next_height = std::max(1U, previous_height / 2U);
        std::vector<std::uint8_t> next(
            static_cast<std::size_t>(next_width) * next_height * 4U, 0U);

        for (std::uint32_t y = 0; y < next_height; ++y) {
            for (std::uint32_t x = 0; x < next_width; ++x) {
                for (std::uint32_t channel = 0; channel < 4U; ++channel) {
                    std::uint32_t sum = 0;
                    std::uint32_t samples = 0;
                    for (std::uint32_t dy = 0; dy < 2U; ++dy) {
                        for (std::uint32_t dx = 0; dx < 2U; ++dx) {
                            const std::uint32_t source_x = std::min(previous_width - 1U, x * 2U + dx);
                            const std::uint32_t source_y = std::min(previous_height - 1U, y * 2U + dy);
                            const std::size_t source_index =
                                (static_cast<std::size_t>(source_y) * previous_width + source_x) * 4U + channel;
                            sum += previous[source_index];
                            ++samples;
                        }
                    }
                    const std::size_t target_index =
                        (static_cast<std::size_t>(y) * next_width + x) * 4U + channel;
                    next[target_index] = static_cast<std::uint8_t>(sum / samples);
                }
            }
        }

        append_bytes(texture.pixels, std::span<const std::uint8_t>{next.data(), next.size()});
        previous = std::move(next);
        previous_width = next_width;
        previous_height = next_height;
    }
    return texture;
}

[[nodiscard]] std::optional<TextureData> decode_image_rgba8(const fastgltf::Asset& asset,
                                                             const std::size_t image_index) {
    if (image_index >= asset.images.size()) {
        std::cerr << "[assetc] texture refers to an invalid image index\n";
        return std::nullopt;
    }

    const auto encoded = encoded_image_bytes(asset, asset.images[image_index]);
    if (!encoded.has_value() || encoded->empty()) {
        std::cerr << "[assetc] unsupported/unloaded glTF image source\n";
        return std::nullopt;
    }
    if (encoded->size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        std::cerr << "[assetc] encoded source image is too large for stb_image\n";
        return std::nullopt;
    }

    int width = 0;
    int height = 0;
    int channels = 0;
    stbi_uc* decoded = stbi_load_from_memory(
        reinterpret_cast<const stbi_uc*>(encoded->data()),
        static_cast<int>(encoded->size()),
        &width,
        &height,
        &channels,
        STBI_rgb_alpha);
    if (decoded == nullptr || width <= 0 || height <= 0) {
        std::cerr << "[assetc] failed to decode image: "
                  << (stbi_failure_reason() != nullptr ? stbi_failure_reason() : "unknown stb_image error")
                  << '\n';
        if (decoded != nullptr) {
            stbi_image_free(decoded);
        }
        return std::nullopt;
    }

    const std::size_t byte_count = static_cast<std::size_t>(width) *
                                   static_cast<std::size_t>(height) * 4U;
    const std::span<const std::uint8_t> rgba{
        reinterpret_cast<const std::uint8_t*>(decoded), byte_count};
    TextureData texture = make_mipped_rgba8(
        static_cast<std::uint32_t>(width),
        static_cast<std::uint32_t>(height),
        rgba,
        TextureColorSpace::srgb);
    stbi_image_free(decoded);
    return texture;
}

[[nodiscard]] SamplerAddressMode sampler_address_mode(const fastgltf::Wrap wrap) noexcept {
    switch (wrap) {
    case fastgltf::Wrap::ClampToEdge:
        return SamplerAddressMode::clamp_to_edge;
    case fastgltf::Wrap::MirroredRepeat:
        return SamplerAddressMode::mirrored_repeat;
    case fastgltf::Wrap::Repeat:
    default:
        return SamplerAddressMode::repeat;
    }
}

[[nodiscard]] SamplerData sampler_data(const fastgltf::Sampler& source) noexcept {
    SamplerData result{};
    result.address_u = sampler_address_mode(source.wrapS);
    result.address_v = sampler_address_mode(source.wrapT);

    if (source.magFilter.has_value()) {
        result.mag_filter = *source.magFilter == fastgltf::Filter::Nearest
            ? SamplerFilter::nearest
            : SamplerFilter::linear;
    }

    if (source.minFilter.has_value()) {
        switch (*source.minFilter) {
        case fastgltf::Filter::Nearest:
            result.min_filter = SamplerFilter::nearest;
            result.use_mipmaps = false;
            break;
        case fastgltf::Filter::Linear:
            result.min_filter = SamplerFilter::linear;
            result.use_mipmaps = false;
            break;
        case fastgltf::Filter::NearestMipMapNearest:
            result.min_filter = SamplerFilter::nearest;
            result.mip_filter = SamplerMipFilter::nearest;
            break;
        case fastgltf::Filter::LinearMipMapNearest:
            result.min_filter = SamplerFilter::linear;
            result.mip_filter = SamplerMipFilter::nearest;
            break;
        case fastgltf::Filter::NearestMipMapLinear:
            result.min_filter = SamplerFilter::nearest;
            result.mip_filter = SamplerMipFilter::linear;
            break;
        case fastgltf::Filter::LinearMipMapLinear:
        default:
            result.min_filter = SamplerFilter::linear;
            result.mip_filter = SamplerMipFilter::linear;
            break;
        }
    }
    return result;
}

void import_samplers(const fastgltf::Asset& asset, ModelData& output) {
    // Slot zero is OCS's deterministic default sampler for glTF textures that
    // omit an explicit sampler object.
    output.samplers.emplace_back();
    output.samplers.reserve(asset.samplers.size() + 1U);
    for (const fastgltf::Sampler& sampler : asset.samplers) {
        output.samplers.push_back(sampler_data(sampler));
    }
}

[[nodiscard]] std::optional<std::uint32_t> import_base_color_texture(
    const fastgltf::Asset& asset,
    const std::size_t gltf_texture_index,
    ModelData& output,
    std::vector<std::uint32_t>& texture_map) {
    if (gltf_texture_index >= asset.textures.size() || gltf_texture_index >= texture_map.size()) {
        std::cerr << "[assetc] material refers to an invalid glTF texture\n";
        return std::nullopt;
    }
    if (texture_map[gltf_texture_index] != ocs::assets::kInvalidAssetIndex) {
        return texture_map[gltf_texture_index];
    }

    const fastgltf::Texture& source_texture = asset.textures[gltf_texture_index];
    if (!source_texture.imageIndex.has_value()) {
        std::cerr << "[assetc] Step 5.1 expects a regular glTF image source for base color\n";
        return std::nullopt;
    }

    auto decoded = decode_image_rgba8(asset, *source_texture.imageIndex);
    if (!decoded.has_value()) {
        return std::nullopt;
    }
    if (output.textures.size() >= static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
        std::cerr << "[assetc] runtime texture count exceeds 32-bit indexing\n";
        return std::nullopt;
    }

    const std::uint32_t runtime_index = static_cast<std::uint32_t>(output.textures.size());
    std::cout << "[assetc] decoded base-color texture " << gltf_texture_index
              << " | " << decoded->width << 'x' << decoded->height
              << " | mips " << decoded->mip_levels << '\n';
    output.textures.push_back(std::move(*decoded));
    texture_map[gltf_texture_index] = runtime_index;
    return runtime_index;
}

[[nodiscard]] bool import_materials(const fastgltf::Asset& asset, ModelData& output) {
    // Slot zero is always a valid fallback for primitives without a glTF material.
    output.materials.emplace_back();
    std::vector<std::uint32_t> texture_map(asset.textures.size(), ocs::assets::kInvalidAssetIndex);

    bool warned_future_textures = false;
    for (const fastgltf::Material& source : asset.materials) {
        MaterialData material{};
        const auto& base = source.pbrData.baseColorFactor;
        material.base_color_factor = {base.x(), base.y(), base.z(), base.w()};
        material.metallic_factor = source.pbrData.metallicFactor;
        material.roughness_factor = source.pbrData.roughnessFactor;
        material.alpha_cutoff = source.alphaCutoff;
        material.alpha_mode = alpha_mode(source.alphaMode);
        material.double_sided = source.doubleSided;
        material.unlit = source.unlit;

        if (source.pbrData.baseColorTexture.has_value()) {
            const auto& texture_info = *source.pbrData.baseColorTexture;
            if (texture_info.texCoordIndex != 0U) {
                std::cerr << "[assetc] Step 5.1 supports TEXCOORD_0 for base-color textures only\n";
                return false;
            }
            const auto runtime_texture = import_base_color_texture(
                asset, texture_info.textureIndex, output, texture_map);
            if (!runtime_texture.has_value()) {
                return false;
            }

            const fastgltf::Texture& source_texture = asset.textures[texture_info.textureIndex];
            const std::uint32_t runtime_sampler = source_texture.samplerIndex.has_value()
                ? static_cast<std::uint32_t>(*source_texture.samplerIndex + 1U)
                : 0U;
            if (runtime_sampler >= output.samplers.size()) {
                std::cerr << "[assetc] texture refers to an invalid sampler\n";
                return false;
            }
            material.base_color_texture = {
                .texture_index = *runtime_texture,
                .sampler_index = runtime_sampler,
                .texcoord_index = 0U
            };
        }

        if (!warned_future_textures &&
            (source.pbrData.metallicRoughnessTexture.has_value() || source.normalTexture.has_value())) {
            std::cerr << "[assetc] note: Step 5.1 consumes base-color textures; normal and "
                         "metallic-roughness image slots are reserved for a later material extension\n";
            warned_future_textures = true;
        }
        output.materials.push_back(material);
    }
    return true;
}

[[nodiscard]] bool append_primitive(const fastgltf::Asset& asset,
                                    const fastgltf::Primitive& primitive,
                                    const fastgltf::math::fmat4x4& world,
                                    ModelData& model,
                                    bool& bounds_initialized) {
    if (primitive.type != fastgltf::PrimitiveType::Triangles) {
        std::cerr << "[assetc] skipping non-triangle primitive\n";
        return true;
    }

    const auto position_attribute = primitive.findAttribute("POSITION");
    if (position_attribute == primitive.attributes.end()) {
        std::cerr << "[assetc] primitive has no POSITION attribute\n";
        return false;
    }

    const auto& position_accessor = asset.accessors[position_attribute->accessorIndex];
    if (position_accessor.count == 0U) {
        return true;
    }
    if (model.vertices.size() + position_accessor.count >
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
        std::cerr << "[assetc] merged model exceeds 32-bit vertex addressing\n";
        return false;
    }

    const std::size_t base_vertex = model.vertices.size();
    model.vertices.resize(base_vertex + position_accessor.count);

    fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec3>(
        asset,
        position_accessor,
        [&](const fastgltf::math::fvec3 position, const std::size_t index) {
            const Vec3f converted = gltf_to_engine(transform_point(world, position));
            MeshVertex& vertex = model.vertices[base_vertex + index];
            vertex.position = converted;
            vertex.normal = {};
            vertex.texcoord = {};
            extend_bounds(model, converted, bounds_initialized);
        });

    bool has_normals = false;
    if (const auto normal_attribute = primitive.findAttribute("NORMAL");
        normal_attribute != primitive.attributes.end()) {
        const auto normal_matrix = inverse_transpose_linear(world);
        if (!normal_matrix.has_value()) {
            std::cerr << "[assetc] node transform has a singular normal matrix\n";
            return false;
        }

        const auto& accessor = asset.accessors[normal_attribute->accessorIndex];
        if (accessor.count != position_accessor.count) {
            std::cerr << "[assetc] NORMAL count differs from POSITION count\n";
            return false;
        }
        fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec3>(
            asset,
            accessor,
            [&](const fastgltf::math::fvec3 normal, const std::size_t index) {
                model.vertices[base_vertex + index].normal =
                    transform_normal(*normal_matrix, normal);
            });
        has_normals = true;
    }

    if (const auto texcoord_attribute = primitive.findAttribute("TEXCOORD_0");
        texcoord_attribute != primitive.attributes.end()) {
        const auto& accessor = asset.accessors[texcoord_attribute->accessorIndex];
        if (accessor.count != position_accessor.count) {
            std::cerr << "[assetc] TEXCOORD_0 count differs from POSITION count\n";
            return false;
        }
        fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec2>(
            asset,
            accessor,
            [&](const fastgltf::math::fvec2 uv, const std::size_t index) {
                model.vertices[base_vertex + index].texcoord = {uv.x(), uv.y()};
            });
    }

    const std::size_t first_index = model.indices.size();
    if (primitive.indicesAccessor.has_value()) {
        const auto& accessor = asset.accessors[*primitive.indicesAccessor];
        std::vector<std::uint32_t> local_indices(accessor.count);
        fastgltf::copyFromAccessor<std::uint32_t>(asset, accessor, local_indices.data());
        model.indices.reserve(model.indices.size() + local_indices.size());
        for (const std::uint32_t local_index : local_indices) {
            if (local_index >= position_accessor.count) {
                std::cerr << "[assetc] primitive contains an out-of-range index\n";
                return false;
            }
            model.indices.push_back(static_cast<std::uint32_t>(base_vertex) + local_index);
        }
    } else {
        for (std::size_t index = 0; index < position_accessor.count; ++index) {
            model.indices.push_back(static_cast<std::uint32_t>(base_vertex + index));
        }
    }

    const std::size_t added_indices = model.indices.size() - first_index;
    if ((added_indices % 3U) != 0U) {
        std::cerr << "[assetc] triangle primitive index count is not divisible by 3\n";
        return false;
    }
    if (!has_normals) {
        generate_normals(model, base_vertex, first_index, added_indices);
    }

    const std::uint32_t material_index = primitive.materialIndex.has_value()
        ? static_cast<std::uint32_t>(*primitive.materialIndex + 1U)
        : 0U;
    if (material_index >= model.materials.size()) {
        std::cerr << "[assetc] primitive refers to an invalid material index\n";
        return false;
    }

    model.submeshes.push_back({
        .first_index = static_cast<std::uint32_t>(first_index),
        .index_count = static_cast<std::uint32_t>(added_indices),
        .material_index = material_index
    });
    return true;
}

[[nodiscard]] bool compile_gltf(const std::filesystem::path& source,
                                ModelData& output) {
    auto file = fastgltf::GltfDataBuffer::FromPath(source);
    if (file.error() != fastgltf::Error::None) {
        std::cerr << "[assetc] failed to read " << source << ": "
                  << fastgltf::getErrorMessage(file.error()) << '\n';
        return false;
    }

    fastgltf::Parser parser;
    constexpr auto options =
        fastgltf::Options::LoadExternalBuffers |
        fastgltf::Options::LoadExternalImages |
        fastgltf::Options::GenerateMeshIndices;

    auto parsed = parser.loadGltf(file.get(), source.parent_path(), options);
    if (parsed.error() != fastgltf::Error::None) {
        std::cerr << "[assetc] failed to parse " << source << ": "
                  << fastgltf::getErrorMessage(parsed.error()) << '\n';
        return false;
    }

    fastgltf::Asset& asset = parsed.get();
    if (asset.meshes.empty()) {
        std::cerr << "[assetc] glTF has no meshes\n";
        return false;
    }

    import_samplers(asset, output);
    if (!import_materials(asset, output)) {
        return false;
    }
    bool bounds_initialized = false;
    bool ok = true;

    if (!asset.scenes.empty()) {
        const std::size_t scene_index = asset.defaultScene.value_or(0U);
        fastgltf::iterateSceneNodes(
            asset,
            scene_index,
            fastgltf::math::fmat4x4(1.0F),
            [&](fastgltf::Node& node, const fastgltf::math::fmat4x4& world) {
                if (!ok || !node.meshIndex.has_value()) {
                    return;
                }
                const auto& gltf_mesh = asset.meshes[*node.meshIndex];
                for (const fastgltf::Primitive& primitive : gltf_mesh.primitives) {
                    if (!append_primitive(asset, primitive, world, output, bounds_initialized)) {
                        ok = false;
                        return;
                    }
                }
            });
    } else {
        const fastgltf::math::fmat4x4 identity(1.0F);
        for (const fastgltf::Mesh& gltf_mesh : asset.meshes) {
            for (const fastgltf::Primitive& primitive : gltf_mesh.primitives) {
                if (!append_primitive(asset, primitive, identity, output, bounds_initialized)) {
                    return false;
                }
            }
        }
    }

    if (!ok || output.vertices.empty() || output.indices.empty() || output.submeshes.empty()) {
        std::cerr << "[assetc] no supported triangle geometry was produced\n";
        return false;
    }
    return true;
}

[[nodiscard]] MeshData flatten_mesh(const ModelData& model) {
    MeshData mesh{};
    mesh.vertices = model.vertices;
    mesh.indices = model.indices;
    mesh.bounds = model.bounds;
    return mesh;
}

} // namespace

int main(const int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: ocs_assetc <input.gltf|input.glb> <output.ocsmodel|output.ocsmesh>\n";
        return 2;
    }

    const std::filesystem::path source = argv[1];
    const std::filesystem::path destination = argv[2];

    ModelData model{};
    if (!compile_gltf(source, model)) {
        return 1;
    }

    ocs::core::Status status{};
    if (destination.extension() == ".ocsmesh") {
        status = ocs::assets::write_mesh(destination, flatten_mesh(model));
    } else {
        status = ocs::assets::write_model(destination, model);
    }

    if (!status) {
        std::cerr << "[assetc] " << status.error().message << '\n';
        return 1;
    }

    std::cout << "[assetc] " << source.filename().string()
              << " -> " << destination.string()
              << " | vertices " << model.vertices.size()
              << " | indices " << model.indices.size()
              << " | triangles " << model.indices.size() / 3U
              << " | submeshes " << model.submeshes.size()
              << " | materials " << model.materials.size()
              << " | textures " << model.textures.size()
              << " | samplers " << model.samplers.size()
              << '\n';
    return 0;
}
