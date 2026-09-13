#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>

#include "ocs/assets/texture_io.hpp"

namespace {

ocs::assets::TextureData sample_texture() {
    ocs::assets::TextureData texture{};
    texture.width = 2U;
    texture.height = 2U;
    texture.mip_levels = 2U;
    texture.color_space = ocs::assets::TextureColorSpace::srgb;
    // 2x2 RGBA8 base level (16 bytes), then 1x1 mip (4 bytes).
    texture.pixels = {
        std::byte{0xFF}, std::byte{0x00}, std::byte{0x00}, std::byte{0xFF},
        std::byte{0x00}, std::byte{0xFF}, std::byte{0x00}, std::byte{0xFF},
        std::byte{0x00}, std::byte{0x00}, std::byte{0xFF}, std::byte{0xFF},
        std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF},
        std::byte{0x80}, std::byte{0x80}, std::byte{0x80}, std::byte{0xFF}
    };
    return texture;
}

std::filesystem::path temp_path(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}

} // namespace

TEST(TextureIo, RoundTripsMipChain) {
    const auto path = temp_path("ocs_texture_roundtrip.ocstex");
    const auto original = sample_texture();
    ASSERT_TRUE(ocs::assets::write_texture(path, original));

    const auto loaded = ocs::assets::load_texture(path);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    EXPECT_EQ(loaded->width, 2U);
    EXPECT_EQ(loaded->height, 2U);
    EXPECT_EQ(loaded->mip_levels, 2U);
    EXPECT_EQ(loaded->color_space, ocs::assets::TextureColorSpace::srgb);
    EXPECT_EQ(loaded->pixels, original.pixels);
    EXPECT_EQ(ocs::assets::texture_mip_offset(*loaded, 1U), 16U);

    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

TEST(TextureIo, RejectsCorruptedPayload) {
    const auto path = temp_path("ocs_texture_corrupt.ocstex");
    ASSERT_TRUE(ocs::assets::write_texture(path, sample_texture()));

    std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
    ASSERT_TRUE(file.good());
    file.seekg(-1, std::ios::end);
    char value = 0;
    file.read(&value, 1);
    file.clear();
    file.seekp(-1, std::ios::end);
    value ^= 0x31;
    file.write(&value, 1);
    file.close();

    EXPECT_FALSE(ocs::assets::load_texture(path).has_value());

    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}
