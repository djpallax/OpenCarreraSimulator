#pragma once

#include <cstdint>

#include "ocs/assets/texture.hpp"
#include "ocs/math/vec4f.hpp"

namespace ocs::assets {

enum class MaterialAlphaMode : std::uint8_t {
    opaque,
    mask,
    blend
};

struct MaterialData {
    math::Vec4f base_color_factor{1.0F, 1.0F, 1.0F, 1.0F};
    float metallic_factor = 1.0F;
    float roughness_factor = 1.0F;
    float alpha_cutoff = 0.5F;
    MaterialAlphaMode alpha_mode = MaterialAlphaMode::opaque;
    bool double_sided = false;
    bool unlit = false;

    // Step 5.1 consumes base-color textures. The binding structure is already
    // generic enough for additional material texture slots in later milestones.
    MaterialTextureBinding base_color_texture{};
};

} // namespace ocs::assets
