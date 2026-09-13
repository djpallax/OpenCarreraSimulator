#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>

#include "ocs/assets/model_io.hpp"

namespace {

ocs::assets::ModelData sample_model() {
    ocs::assets::ModelData model{};
    model.vertices = {
        {{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F}},
        {{1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {1.0F, 0.0F}},
        {{0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 1.0F}}
    };
    model.indices = {0, 1, 2};
    model.bounds.min = {0.0F, 0.0F, 0.0F};
    model.bounds.max = {1.0F, 1.0F, 0.0F};

    ocs::assets::MaterialData material{};
    material.base_color_factor = {0.25F, 0.5F, 0.75F, 1.0F};
    material.metallic_factor = 0.2F;
    material.roughness_factor = 0.6F;
    material.double_sided = true;
    model.materials.push_back(material);
    model.submeshes.push_back({0, 3, 0});
    return model;
}

std::filesystem::path temp_path(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}

} // namespace

TEST(ModelIo, RoundTripsMaterialsAndSubmeshes) {
    const auto path = temp_path("ocs_model_roundtrip.ocsmodel");
    const auto original = sample_model();
    ASSERT_TRUE(ocs::assets::write_model(path, original));

    const auto loaded = ocs::assets::load_model(path);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    ASSERT_EQ(loaded->vertices.size(), 3U);
    ASSERT_EQ(loaded->indices.size(), 3U);
    ASSERT_EQ(loaded->submeshes.size(), 1U);
    ASSERT_EQ(loaded->materials.size(), 1U);
    EXPECT_EQ(loaded->submeshes[0].material_index, 0U);
    EXPECT_FLOAT_EQ(loaded->materials[0].base_color_factor.y, 0.5F);
    EXPECT_FLOAT_EQ(loaded->materials[0].roughness_factor, 0.6F);
    EXPECT_TRUE(loaded->materials[0].double_sided);

    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

TEST(ModelIo, RejectsInvalidMaterialReferenceBeforeWrite) {
    const auto path = temp_path("ocs_model_invalid.ocsmodel");
    auto model = sample_model();
    model.submeshes[0].material_index = 99;
    const auto status = ocs::assets::write_model(path, model);
    EXPECT_FALSE(status.has_value());
}

TEST(ModelIo, RejectsCorruptedPayload) {
    const auto path = temp_path("ocs_model_corrupt.ocsmodel");
    ASSERT_TRUE(ocs::assets::write_model(path, sample_model()));

    std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
    ASSERT_TRUE(file.good());
    file.seekg(-1, std::ios::end);
    char value = 0;
    file.read(&value, 1);
    file.clear();
    file.seekp(-1, std::ios::end);
    value ^= 0x5A;
    file.write(&value, 1);
    file.close();

    const auto loaded = ocs::assets::load_model(path);
    EXPECT_FALSE(loaded.has_value());

    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

TEST(ModelIo, RoundTripsBaseColorTextureBindingAndSidecar) {
    const auto path = temp_path("ocs_model_textured.ocsmodel");
    auto model = sample_model();

    ocs::assets::TextureData texture{};
    texture.width = 1U;
    texture.height = 1U;
    texture.mip_levels = 1U;
    texture.color_space = ocs::assets::TextureColorSpace::srgb;
    texture.pixels = {
        std::byte{0x10}, std::byte{0x40}, std::byte{0x80}, std::byte{0xFF}
    };
    model.textures.push_back(texture);
    model.samplers.emplace_back();
    model.materials[0].base_color_texture = {
        .texture_index = 0U,
        .sampler_index = 0U,
        .texcoord_index = 0U
    };

    ASSERT_TRUE(ocs::assets::write_model(path, model));
    const auto sidecar = ocs::assets::model_texture_sidecar_path(path, 0U);
    EXPECT_TRUE(std::filesystem::exists(sidecar));

    const auto loaded = ocs::assets::load_model(path);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    ASSERT_EQ(loaded->textures.size(), 1U);
    ASSERT_EQ(loaded->samplers.size(), 1U);
    EXPECT_EQ(loaded->materials[0].base_color_texture.texture_index, 0U);
    EXPECT_EQ(loaded->materials[0].base_color_texture.sampler_index, 0U);
    EXPECT_EQ(loaded->textures[0].pixels, texture.pixels);

    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    std::filesystem::remove(sidecar, ignored);
}

TEST(ModelIo, RejectsOutOfRangeTextureBindingBeforeWrite) {
    const auto path = temp_path("ocs_model_invalid_texture.ocsmodel");
    auto model = sample_model();
    model.samplers.emplace_back();
    model.materials[0].base_color_texture = {
        .texture_index = 5U,
        .sampler_index = 0U,
        .texcoord_index = 0U
    };
    EXPECT_FALSE(ocs::assets::write_model(path, model).has_value());
}
