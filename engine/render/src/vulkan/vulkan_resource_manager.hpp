#pragma once

#include <vulkan/vulkan.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ocs/assets/material.hpp"
#include "ocs/assets/model.hpp"
#include "ocs/assets/texture.hpp"
#include "ocs/core/bytes.hpp"
#include "ocs/core/slot_map.hpp"
#include "ocs/render/handles.hpp"
#include "vulkan_buffer.hpp"
#include "vulkan_memory.hpp"
#include "vulkan_raii.hpp"
#include "vulkan_texture.hpp"

namespace ocs::render::vulkan {

struct GpuSubmesh {
    std::uint32_t first_index = 0;
    std::uint32_t index_count = 0;
    MaterialHandle material{};
};

struct GpuMeshResource {
    VulkanBuffer vertex_buffer{};
    VulkanBuffer index_buffer{};
    std::uint32_t index_count = 0;
    core::ByteSize vertex_memory{};
    core::ByteSize index_memory{};
};

struct GpuTextureResource {
    VulkanTexture texture{};
};

struct GpuSamplerResource {
    UniqueSampler sampler{};
    assets::SamplerData data{};
};

struct GpuMaterialResource {
    assets::MaterialData data{};
    TextureHandle base_color_texture{};
    SamplerHandle base_color_sampler{};
    VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
};

struct GpuModelResource {
    MeshHandle mesh{};
    std::vector<GpuSubmesh> submeshes{};
    std::vector<MaterialHandle> owned_materials{};
    std::vector<TextureHandle> owned_textures{};
    std::vector<SamplerHandle> owned_samplers{};
    assets::Aabb3f bounds{};
};

struct ResourceManagerStats {
    std::size_t model_count = 0;
    std::size_t mesh_count = 0;
    std::size_t material_count = 0;
    std::size_t texture_count = 0;
    std::size_t sampler_count = 0;
    core::ByteSize vertex_memory{};
    core::ByteSize index_memory{};
    core::ByteSize texture_memory{};
};

class VulkanResourceManager {
public:
    VulkanResourceManager() = default;
    ~VulkanResourceManager();

    VulkanResourceManager(const VulkanResourceManager&) = delete;
    VulkanResourceManager& operator=(const VulkanResourceManager&) = delete;

    [[nodiscard]] bool initialize(VulkanMemoryAllocator& allocator,
                                  VkQueue transfer_queue,
                                  std::uint32_t transfer_queue_family,
                                  VkDevice device,
                                  VkDescriptorSetLayout material_set_layout);
    void shutdown() noexcept;

    [[nodiscard]] ModelHandle upload_model(const assets::ModelData& model);
    bool destroy_model(ModelHandle handle) noexcept;

    [[nodiscard]] GpuModelResource* model(ModelHandle handle) noexcept;
    [[nodiscard]] const GpuModelResource* model(ModelHandle handle) const noexcept;
    [[nodiscard]] GpuMeshResource* mesh(MeshHandle handle) noexcept;
    [[nodiscard]] const GpuMeshResource* mesh(MeshHandle handle) const noexcept;
    [[nodiscard]] GpuMaterialResource* material(MaterialHandle handle) noexcept;
    [[nodiscard]] const GpuMaterialResource* material(MaterialHandle handle) const noexcept;
    [[nodiscard]] GpuTextureResource* texture(TextureHandle handle) noexcept;
    [[nodiscard]] const GpuTextureResource* texture(TextureHandle handle) const noexcept;
    [[nodiscard]] GpuSamplerResource* sampler(SamplerHandle handle) noexcept;
    [[nodiscard]] const GpuSamplerResource* sampler(SamplerHandle handle) const noexcept;

    [[nodiscard]] ResourceManagerStats stats() const noexcept;

private:
    [[nodiscard]] TextureHandle create_texture(const assets::TextureData& data);
    [[nodiscard]] SamplerHandle create_sampler(const assets::SamplerData& data);
    [[nodiscard]] MaterialHandle create_material(const assets::MaterialData& data,
                                                 TextureHandle texture,
                                                 SamplerHandle sampler);
    void destroy_material(MaterialHandle handle) noexcept;

    [[nodiscard]] bool create_descriptor_pool();
    [[nodiscard]] bool create_fallback_resources();

    VulkanMemoryAllocator* allocator_ = nullptr;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue transfer_queue_ = VK_NULL_HANDLE;
    std::uint32_t transfer_queue_family_ = 0;
    VkDescriptorSetLayout material_set_layout_ = VK_NULL_HANDLE;
    UniqueDescriptorPool material_descriptor_pool_{};

    TextureHandle fallback_texture_{};
    SamplerHandle fallback_sampler_{};

    core::SlotMap<MeshHandle, GpuMeshResource> meshes_{};
    core::SlotMap<TextureHandle, GpuTextureResource> textures_{};
    core::SlotMap<SamplerHandle, GpuSamplerResource> samplers_{};
    core::SlotMap<MaterialHandle, GpuMaterialResource> materials_{};
    core::SlotMap<ModelHandle, GpuModelResource> models_{};
};

} // namespace ocs::render::vulkan
