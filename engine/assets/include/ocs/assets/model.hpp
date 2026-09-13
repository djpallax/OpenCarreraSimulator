#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "ocs/assets/material.hpp"
#include "ocs/assets/mesh.hpp"
#include "ocs/assets/texture.hpp"

namespace ocs::assets {

struct SubmeshData {
    std::uint32_t first_index = 0;
    std::uint32_t index_count = 0;
    std::uint32_t material_index = 0;
};

struct ModelData {
    std::vector<MeshVertex> vertices;
    std::vector<std::uint32_t> indices;
    std::vector<SubmeshData> submeshes;
    std::vector<MaterialData> materials;
    std::vector<TextureData> textures;
    std::vector<SamplerData> samplers;
    Aabb3f bounds{};
};

[[nodiscard]] inline ModelData model_from_mesh(MeshData mesh) {
    ModelData model{};
    model.vertices = std::move(mesh.vertices);
    model.indices = std::move(mesh.indices);
    model.bounds = mesh.bounds;
    model.materials.emplace_back();
    model.submeshes.push_back({
        .first_index = 0,
        .index_count = static_cast<std::uint32_t>(model.indices.size()),
        .material_index = 0
    });
    return model;
}

} // namespace ocs::assets
