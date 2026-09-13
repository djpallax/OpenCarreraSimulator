#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>

#include "ocs/assets/mesh_io.hpp"
#include "ocs/assets/model_io.hpp"

namespace {

using ocs::assets::MeshData;
using ocs::assets::MeshVertex;
using ocs::assets::ModelData;
using ocs::math::Vec2f;
using ocs::math::Vec3f;

void append_quad(ModelData& model,
                 const float x0,
                 const float x1,
                 const float y0,
                 const float y1,
                 const float z) {
    const std::uint32_t base = static_cast<std::uint32_t>(model.vertices.size());
    model.vertices.push_back({{x0, y0, z}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F}});
    model.vertices.push_back({{x1, y0, z}, {0.0F, 0.0F, 1.0F}, {1.0F, 0.0F}});
    model.vertices.push_back({{x1, y1, z}, {0.0F, 0.0F, 1.0F}, {1.0F, 1.0F}});
    model.vertices.push_back({{x0, y1, z}, {0.0F, 0.0F, 1.0F}, {0.0F, 1.0F}});
    model.indices.insert(model.indices.end(), {
        base + 0U, base + 1U, base + 2U,
        base + 0U, base + 2U, base + 3U
    });
}

void append_quad(MeshData& mesh,
                 const float x0,
                 const float x1,
                 const float y0,
                 const float y1,
                 const float z) {
    const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back({{x0, y0, z}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F}});
    mesh.vertices.push_back({{x1, y0, z}, {0.0F, 0.0F, 1.0F}, {1.0F, 0.0F}});
    mesh.vertices.push_back({{x1, y1, z}, {0.0F, 0.0F, 1.0F}, {1.0F, 1.0F}});
    mesh.vertices.push_back({{x0, y1, z}, {0.0F, 0.0F, 1.0F}, {0.0F, 1.0F}});
    mesh.indices.insert(mesh.indices.end(), {
        base + 0U, base + 1U, base + 2U,
        base + 0U, base + 2U, base + 3U
    });
}

ModelData make_visual_straight() {
    constexpr float kLength = 500.0F;
    constexpr float kHalfWidth = 6.0F;

    ModelData model{};
    model.materials.resize(2);
    model.materials[0].base_color_factor = {0.055F, 0.060F, 0.066F, 1.0F};
    model.materials[0].metallic_factor = 0.0F;
    model.materials[0].roughness_factor = 0.95F;
    model.materials[1].base_color_factor = {0.92F, 0.92F, 0.88F, 1.0F};
    model.materials[1].metallic_factor = 0.0F;
    model.materials[1].roughness_factor = 0.70F;

    const std::uint32_t asphalt_first = static_cast<std::uint32_t>(model.indices.size());
    append_quad(model, 0.0F, kLength, -kHalfWidth, kHalfWidth, 0.0F);
    model.submeshes.push_back({
        .first_index = asphalt_first,
        .index_count = static_cast<std::uint32_t>(model.indices.size()) - asphalt_first,
        .material_index = 0U
    });

    const std::uint32_t marking_first = static_cast<std::uint32_t>(model.indices.size());
    append_quad(model, 0.0F, kLength, -5.82F, -5.64F, 0.012F);
    append_quad(model, 0.0F, kLength, 5.64F, 5.82F, 0.012F);
    for (float x = 10.0F; x < kLength; x += 20.0F) {
        append_quad(model, x, x + 8.0F, -0.08F, 0.08F, 0.015F);
    }
    model.submeshes.push_back({
        .first_index = marking_first,
        .index_count = static_cast<std::uint32_t>(model.indices.size()) - marking_first,
        .material_index = 1U
    });

    model.bounds.min = {0.0F, -kHalfWidth, 0.0F};
    model.bounds.max = {kLength, kHalfWidth, 0.015F};
    return model;
}

MeshData make_collision_straight() {
    MeshData mesh{};
    append_quad(mesh, 0.0F, 500.0F, -6.0F, 6.0F, 0.0F);
    mesh.bounds.min = {0.0F, -6.0F, 0.0F};
    mesh.bounds.max = {500.0F, 6.0F, 0.0F};
    return mesh;
}

} // namespace

int main(const int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: ocs_trackgen <visual.ocsmodel> <collision.ocsmesh>\n";
        return 2;
    }

    const std::filesystem::path visual_path = argv[1];
    const std::filesystem::path collision_path = argv[2];
    std::filesystem::create_directories(visual_path.parent_path());
    std::filesystem::create_directories(collision_path.parent_path());

    const ModelData visual = make_visual_straight();
    if (auto status = ocs::assets::write_model(visual_path, visual); !status) {
        std::cerr << "trackgen visual failed: " << status.error().message << '\n';
        return 1;
    }

    const MeshData collision = make_collision_straight();
    if (auto status = ocs::assets::write_mesh(collision_path, collision); !status) {
        std::cerr << "trackgen collision failed: " << status.error().message << '\n';
        return 1;
    }

    std::cout << "[trackgen] 500 m straight | width 12 m | visual "
              << visual.vertices.size() << " vertices / "
              << visual.indices.size() / 3U << " triangles | collision "
              << collision.indices.size() / 3U << " triangles\n";
    return 0;
}
