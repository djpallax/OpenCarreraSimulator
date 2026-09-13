#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>

#include "ocs/assets/mesh_io.hpp"

namespace {

ocs::assets::MeshData triangle_mesh() {
    ocs::assets::MeshData source{};
    source.vertices = {
        {{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F}},
        {{1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {1.0F, 0.0F}},
        {{0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 1.0F}}
    };
    source.indices = {0, 1, 2};
    source.bounds.min = {0.0F, 0.0F, 0.0F};
    source.bounds.max = {1.0F, 1.0F, 0.0F};
    return source;
}

} // namespace

TEST(MeshIo, RoundTripsVersionedRuntimeMesh) {
    const auto source = triangle_mesh();
    const auto path = std::filesystem::temp_directory_path() / "ocs_mesh_io_test.ocsmesh";

    const auto write = ocs::assets::write_mesh(path, source);
    if (!write) {
        FAIL() << write.error().message;
    }

    const auto loaded = ocs::assets::load_mesh(path);
    if (!loaded) {
        FAIL() << loaded.error().message;
    }

    ASSERT_EQ(loaded->vertices.size(), source.vertices.size());
    ASSERT_EQ(loaded->indices, source.indices);
    EXPECT_FLOAT_EQ(loaded->vertices[1].position.x, 1.0F);
    EXPECT_FLOAT_EQ(loaded->vertices[2].texcoord.y, 1.0F);

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(MeshIo, RejectsCorruptedPayload) {
    const auto source = triangle_mesh();
    const auto path = std::filesystem::temp_directory_path() / "ocs_mesh_io_corrupt.ocsmesh";

    const auto write = ocs::assets::write_mesh(path, source);
    if (!write) {
        FAIL() << write.error().message;
    }

    {
        std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
        ASSERT_TRUE(file.good());
        file.seekg(64, std::ios::beg); // first payload byte, after the v1 header
        char byte = 0;
        file.read(&byte, 1);
        ASSERT_TRUE(file.good());
        byte = static_cast<char>(static_cast<unsigned char>(byte) ^ 0x01U);
        file.seekp(64, std::ios::beg);
        file.write(&byte, 1);
        ASSERT_TRUE(file.good());
    }

    const auto loaded = ocs::assets::load_mesh(path);
    EXPECT_FALSE(loaded.has_value());
    if (!loaded) {
        EXPECT_EQ(loaded.error().code, ocs::core::ErrorCode::invalid_data);
    }

    std::error_code error;
    std::filesystem::remove(path, error);
}
