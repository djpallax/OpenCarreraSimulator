#include "vulkan_resource_manager.hpp"

#include <algorithm>
#include <array>
#include <span>
#include <string>
#include <utility>

#include "ocs/core/log.hpp"
#include "vulkan_common.hpp"

namespace ocs::render::vulkan {
namespace {

[[nodiscard]] VkFilter vk_filter(const assets::SamplerFilter filter) noexcept {
    return filter == assets::SamplerFilter::nearest ? VK_FILTER_NEAREST : VK_FILTER_LINEAR;
}

[[nodiscard]] VkSamplerMipmapMode vk_mipmap_mode(const assets::SamplerMipFilter filter) noexcept {
    return filter == assets::SamplerMipFilter::nearest
        ? VK_SAMPLER_MIPMAP_MODE_NEAREST
        : VK_SAMPLER_MIPMAP_MODE_LINEAR;
}

[[nodiscard]] VkSamplerAddressMode vk_address_mode(const assets::SamplerAddressMode mode) noexcept {
    switch (mode) {
    case assets::SamplerAddressMode::clamp_to_edge:
        return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    case assets::SamplerAddressMode::mirrored_repeat:
        return VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT;
    case assets::SamplerAddressMode::repeat:
    default:
        return VK_SAMPLER_ADDRESS_MODE_REPEAT;
    }
}

} // namespace

VulkanResourceManager::~VulkanResourceManager() {
    shutdown();
}

bool VulkanResourceManager::initialize(VulkanMemoryAllocator& allocator,
                                       const VkQueue transfer_queue,
                                       const std::uint32_t transfer_queue_family,
                                       const VkDevice device,
                                       const VkDescriptorSetLayout material_set_layout) {
    shutdown();
    allocator_ = &allocator;
    device_ = device;
    transfer_queue_ = transfer_queue;
    transfer_queue_family_ = transfer_queue_family;
    material_set_layout_ = material_set_layout;

    if (device_ == VK_NULL_HANDLE || material_set_layout_ == VK_NULL_HANDLE ||
        !create_descriptor_pool() || !create_fallback_resources()) {
        shutdown();
        return false;
    }
    return true;
}

bool VulkanResourceManager::create_descriptor_pool() {
    VkDescriptorPoolSize pool_size{};
    pool_size.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    pool_size.descriptorCount = 4096U;

    VkDescriptorPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    pool_info.maxSets = 4096U;
    pool_info.poolSizeCount = 1U;
    pool_info.pPoolSizes = &pool_size;
    return vk_check(vkCreateDescriptorPool(device_,
                                            &pool_info,
                                            nullptr,
                                            material_descriptor_pool_.put(device_)),
                    "vkCreateDescriptorPool(material)");
}

bool VulkanResourceManager::create_fallback_resources() {
    assets::TextureData white{};
    white.width = 1U;
    white.height = 1U;
    white.mip_levels = 1U;
    white.color_space = assets::TextureColorSpace::srgb;
    white.pixels = {
        std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}
    };
    fallback_texture_ = create_texture(white);
    fallback_sampler_ = create_sampler(assets::SamplerData{});
    return static_cast<bool>(fallback_texture_) && static_cast<bool>(fallback_sampler_);
}

void VulkanResourceManager::shutdown() noexcept {
    // The renderer waits for device idle before shutting the resource manager
    // down, so model-owned descriptors/images can be released immediately here.
    models_.clear();

    // Descriptor sets are owned by the descriptor pool. Clearing material slots
    // before destroying the pool drops only CPU-side metadata; the pool release
    // destroys every remaining set, including fallbacks.
    materials_.clear();
    meshes_.clear();
    textures_.clear();
    samplers_.clear();
    material_descriptor_pool_.reset();

    fallback_texture_ = {};
    fallback_sampler_ = {};
    allocator_ = nullptr;
    device_ = VK_NULL_HANDLE;
    transfer_queue_ = VK_NULL_HANDLE;
    transfer_queue_family_ = 0;
    material_set_layout_ = VK_NULL_HANDLE;
}

TextureHandle VulkanResourceManager::create_texture(const assets::TextureData& data) {
    if (allocator_ == nullptr || transfer_queue_ == VK_NULL_HANDLE) {
        return {};
    }
    GpuTextureResource resource{};
    if (!resource.texture.initialize(*allocator_, transfer_queue_, transfer_queue_family_, data)) {
        return {};
    }
    return textures_.emplace(std::move(resource));
}

SamplerHandle VulkanResourceManager::create_sampler(const assets::SamplerData& data) {
    if (device_ == VK_NULL_HANDLE) {
        return {};
    }

    GpuSamplerResource resource{};
    resource.data = data;

    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = vk_filter(data.mag_filter);
    sampler_info.minFilter = vk_filter(data.min_filter);
    sampler_info.mipmapMode = vk_mipmap_mode(data.mip_filter);
    sampler_info.addressModeU = vk_address_mode(data.address_u);
    sampler_info.addressModeV = vk_address_mode(data.address_v);
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler_info.mipLodBias = 0.0F;
    sampler_info.anisotropyEnable = VK_FALSE;
    sampler_info.maxAnisotropy = 1.0F;
    sampler_info.compareEnable = VK_FALSE;
    sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    sampler_info.minLod = 0.0F;
    sampler_info.maxLod = data.use_mipmaps ? VK_LOD_CLAMP_NONE : 0.0F;
    sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampler_info.unnormalizedCoordinates = VK_FALSE;

    if (!vk_check(vkCreateSampler(device_,
                                   &sampler_info,
                                   nullptr,
                                   resource.sampler.put(device_)),
                  "vkCreateSampler")) {
        return {};
    }
    return samplers_.emplace(std::move(resource));
}

MaterialHandle VulkanResourceManager::create_material(const assets::MaterialData& data,
                                                       const TextureHandle texture_handle,
                                                       const SamplerHandle sampler_handle) {
    GpuTextureResource* texture_resource = textures_.get(texture_handle);
    GpuSamplerResource* sampler_resource = samplers_.get(sampler_handle);
    if (texture_resource == nullptr || sampler_resource == nullptr ||
        !texture_resource->texture.valid() || !sampler_resource->sampler) {
        OCS_LOG_ERROR("ResourceManager cannot create material from stale texture/sampler handles");
        return {};
    }

    VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
    const VkDescriptorSetLayout layout = material_set_layout_;
    VkDescriptorSetAllocateInfo allocate_info{};
    allocate_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocate_info.descriptorPool = material_descriptor_pool_.get();
    allocate_info.descriptorSetCount = 1U;
    allocate_info.pSetLayouts = &layout;
    if (!vk_check(vkAllocateDescriptorSets(device_, &allocate_info, &descriptor_set),
                  "vkAllocateDescriptorSets(material)")) {
        return {};
    }

    VkDescriptorImageInfo image_info{};
    image_info.sampler = sampler_resource->sampler.get();
    image_info.imageView = texture_resource->texture.view();
    image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkWriteDescriptorSet write{};
    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write.dstSet = descriptor_set;
    write.dstBinding = 0U;
    write.descriptorCount = 1U;
    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    write.pImageInfo = &image_info;
    vkUpdateDescriptorSets(device_, 1U, &write, 0U, nullptr);

    const MaterialHandle handle = materials_.emplace(GpuMaterialResource{
        .data = data,
        .base_color_texture = texture_handle,
        .base_color_sampler = sampler_handle,
        .descriptor_set = descriptor_set
    });
    if (!handle) {
        static_cast<void>(vkFreeDescriptorSets(device_,
                                                material_descriptor_pool_.get(),
                                                1U,
                                                &descriptor_set));
    }
    return handle;
}

void VulkanResourceManager::destroy_material(const MaterialHandle handle) noexcept {
    GpuMaterialResource* material_resource = materials_.get(handle);
    if (material_resource == nullptr) {
        return;
    }
    if (material_resource->descriptor_set != VK_NULL_HANDLE &&
        material_descriptor_pool_) {
        const VkDescriptorSet set = material_resource->descriptor_set;
        static_cast<void>(vkFreeDescriptorSets(device_,
                                                material_descriptor_pool_.get(),
                                                1U,
                                                &set));
    }
    static_cast<void>(materials_.erase(handle));
}

ModelHandle VulkanResourceManager::upload_model(const assets::ModelData& source) {
    if (allocator_ == nullptr || transfer_queue_ == VK_NULL_HANDLE ||
        source.vertices.empty() || source.indices.empty() ||
        source.submeshes.empty() || source.materials.empty()) {
        OCS_LOG_ERROR("ResourceManager received an invalid model upload request");
        return {};
    }

    std::vector<TextureHandle> texture_handles;
    texture_handles.reserve(source.textures.size());
    for (const assets::TextureData& texture_data : source.textures) {
        const TextureHandle handle = create_texture(texture_data);
        if (!handle) {
            for (const TextureHandle created : texture_handles) {
                static_cast<void>(textures_.erase(created));
            }
            OCS_LOG_ERROR("ResourceManager failed to create a model texture");
            return {};
        }
        texture_handles.push_back(handle);
    }

    std::vector<SamplerHandle> sampler_handles;
    sampler_handles.reserve(source.samplers.size());
    for (const assets::SamplerData& sampler_data : source.samplers) {
        const SamplerHandle handle = create_sampler(sampler_data);
        if (!handle) {
            for (const TextureHandle created : texture_handles) {
                static_cast<void>(textures_.erase(created));
            }
            for (const SamplerHandle created : sampler_handles) {
                static_cast<void>(samplers_.erase(created));
            }
            OCS_LOG_ERROR("ResourceManager failed to create a model sampler");
            return {};
        }
        sampler_handles.push_back(handle);
    }

    std::vector<MaterialHandle> material_handles;
    material_handles.reserve(source.materials.size());
    for (const assets::MaterialData& material_data : source.materials) {
        TextureHandle texture_handle = fallback_texture_;
        SamplerHandle sampler_handle = fallback_sampler_;
        if (material_data.base_color_texture.valid()) {
            const auto texture_index = material_data.base_color_texture.texture_index;
            const auto sampler_index = material_data.base_color_texture.sampler_index;
            if (texture_index >= texture_handles.size() || sampler_index >= sampler_handles.size()) {
                OCS_LOG_ERROR("ResourceManager material contains an invalid texture binding");
                for (const MaterialHandle created : material_handles) {
                    destroy_material(created);
                }
                for (const TextureHandle created : texture_handles) {
                    static_cast<void>(textures_.erase(created));
                }
                for (const SamplerHandle created : sampler_handles) {
                    static_cast<void>(samplers_.erase(created));
                }
                return {};
            }
            texture_handle = texture_handles[texture_index];
            sampler_handle = sampler_handles[sampler_index];
        }

        const MaterialHandle handle = create_material(material_data, texture_handle, sampler_handle);
        if (!handle) {
            for (const MaterialHandle created : material_handles) {
                destroy_material(created);
            }
            for (const TextureHandle created : texture_handles) {
                static_cast<void>(textures_.erase(created));
            }
            for (const SamplerHandle created : sampler_handles) {
                static_cast<void>(samplers_.erase(created));
            }
            OCS_LOG_ERROR("ResourceManager exhausted/failed material resources");
            return {};
        }
        material_handles.push_back(handle);
    }

    GpuMeshResource gpu_mesh{};
    const auto vertex_bytes = std::as_bytes(std::span{source.vertices});
    if (!gpu_mesh.vertex_buffer.initialize_device_local(
            *allocator_, transfer_queue_, transfer_queue_family_, vertex_bytes,
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT)) {
        for (const MaterialHandle created : material_handles) {
            destroy_material(created);
        }
        for (const TextureHandle created : texture_handles) {
            static_cast<void>(textures_.erase(created));
        }
        for (const SamplerHandle created : sampler_handles) {
            static_cast<void>(samplers_.erase(created));
        }
        return {};
    }

    const auto index_bytes = std::as_bytes(std::span{source.indices});
    if (!gpu_mesh.index_buffer.initialize_device_local(
            *allocator_, transfer_queue_, transfer_queue_family_, index_bytes,
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT)) {
        for (const MaterialHandle created : material_handles) {
            destroy_material(created);
        }
        for (const TextureHandle created : texture_handles) {
            static_cast<void>(textures_.erase(created));
        }
        for (const SamplerHandle created : sampler_handles) {
            static_cast<void>(samplers_.erase(created));
        }
        return {};
    }

    gpu_mesh.index_count = static_cast<std::uint32_t>(source.indices.size());
    gpu_mesh.vertex_memory = core::ByteSize::from_bytes(vertex_bytes.size_bytes());
    gpu_mesh.index_memory = core::ByteSize::from_bytes(index_bytes.size_bytes());

    const MeshHandle mesh_handle = meshes_.emplace(std::move(gpu_mesh));
    if (!mesh_handle) {
        for (const MaterialHandle created : material_handles) {
            destroy_material(created);
        }
        for (const TextureHandle created : texture_handles) {
            static_cast<void>(textures_.erase(created));
        }
        for (const SamplerHandle created : sampler_handles) {
            static_cast<void>(samplers_.erase(created));
        }
        OCS_LOG_ERROR("ResourceManager exhausted mesh handle slots");
        return {};
    }

    GpuModelResource gpu_model{};
    gpu_model.mesh = mesh_handle;
    gpu_model.bounds = source.bounds;
    gpu_model.owned_materials = material_handles;
    gpu_model.owned_textures = texture_handles;
    gpu_model.owned_samplers = sampler_handles;
    gpu_model.submeshes.reserve(source.submeshes.size());
    for (const assets::SubmeshData& submesh : source.submeshes) {
        if (submesh.material_index >= material_handles.size()) {
            static_cast<void>(meshes_.erase(mesh_handle));
            for (const MaterialHandle created : material_handles) {
                destroy_material(created);
            }
            for (const TextureHandle created : texture_handles) {
                static_cast<void>(textures_.erase(created));
            }
            for (const SamplerHandle created : sampler_handles) {
                static_cast<void>(samplers_.erase(created));
            }
            OCS_LOG_ERROR("ResourceManager model contains an invalid material reference");
            return {};
        }
        gpu_model.submeshes.push_back({
            .first_index = submesh.first_index,
            .index_count = submesh.index_count,
            .material = material_handles[submesh.material_index]
        });
    }

    const ModelHandle model_handle = models_.emplace(std::move(gpu_model));
    if (!model_handle) {
        static_cast<void>(meshes_.erase(mesh_handle));
        for (const MaterialHandle created : material_handles) {
            destroy_material(created);
        }
        for (const TextureHandle created : texture_handles) {
            static_cast<void>(textures_.erase(created));
        }
        for (const SamplerHandle created : sampler_handles) {
            static_cast<void>(samplers_.erase(created));
        }
        OCS_LOG_ERROR("ResourceManager exhausted model handle slots");
        return {};
    }

    OCS_LOG_INFO("ResourceManager uploaded model | model handle " +
                 std::to_string(model_handle.index) + ":" +
                 std::to_string(model_handle.generation) + " | mesh handle " +
                 std::to_string(mesh_handle.index) + ":" +
                 std::to_string(mesh_handle.generation) + " | submeshes " +
                 std::to_string(source.submeshes.size()) + " | materials " +
                 std::to_string(source.materials.size()) + " | textures " +
                 std::to_string(source.textures.size()) + " | samplers " +
                 std::to_string(source.samplers.size()));
    return model_handle;
}

bool VulkanResourceManager::destroy_model(const ModelHandle handle) noexcept {
    GpuModelResource* resource = models_.get(handle);
    if (resource == nullptr) {
        return false;
    }

    const MeshHandle mesh_handle = resource->mesh;
    const std::vector<MaterialHandle> material_handles = resource->owned_materials;
    const std::vector<TextureHandle> texture_handles = resource->owned_textures;
    const std::vector<SamplerHandle> sampler_handles = resource->owned_samplers;
    static_cast<void>(models_.erase(handle));
    static_cast<void>(meshes_.erase(mesh_handle));
    for (const MaterialHandle material_handle : material_handles) {
        destroy_material(material_handle);
    }
    for (const TextureHandle texture_handle : texture_handles) {
        static_cast<void>(textures_.erase(texture_handle));
    }
    for (const SamplerHandle sampler_handle : sampler_handles) {
        static_cast<void>(samplers_.erase(sampler_handle));
    }
    return true;
}

GpuModelResource* VulkanResourceManager::model(const ModelHandle handle) noexcept {
    return models_.get(handle);
}

const GpuModelResource* VulkanResourceManager::model(const ModelHandle handle) const noexcept {
    return models_.get(handle);
}

GpuMeshResource* VulkanResourceManager::mesh(const MeshHandle handle) noexcept {
    return meshes_.get(handle);
}

const GpuMeshResource* VulkanResourceManager::mesh(const MeshHandle handle) const noexcept {
    return meshes_.get(handle);
}

GpuMaterialResource* VulkanResourceManager::material(const MaterialHandle handle) noexcept {
    return materials_.get(handle);
}

const GpuMaterialResource* VulkanResourceManager::material(const MaterialHandle handle) const noexcept {
    return materials_.get(handle);
}

GpuTextureResource* VulkanResourceManager::texture(const TextureHandle handle) noexcept {
    return textures_.get(handle);
}

const GpuTextureResource* VulkanResourceManager::texture(const TextureHandle handle) const noexcept {
    return textures_.get(handle);
}

GpuSamplerResource* VulkanResourceManager::sampler(const SamplerHandle handle) noexcept {
    return samplers_.get(handle);
}

const GpuSamplerResource* VulkanResourceManager::sampler(const SamplerHandle handle) const noexcept {
    return samplers_.get(handle);
}

ResourceManagerStats VulkanResourceManager::stats() const noexcept {
    ResourceManagerStats result{};
    result.model_count = models_.size();
    result.mesh_count = meshes_.size();
    result.material_count = materials_.size();
    result.texture_count = textures_.size() > 0U ? textures_.size() - 1U : 0U;
    result.sampler_count = samplers_.size() > 0U ? samplers_.size() - 1U : 0U;

    meshes_.for_each([&](const MeshHandle, const GpuMeshResource& mesh) {
        result.vertex_memory += mesh.vertex_memory;
        result.index_memory += mesh.index_memory;
    });
    textures_.for_each([&](const TextureHandle handle, const GpuTextureResource& texture) {
        if (handle != fallback_texture_) {
            result.texture_memory += texture.texture.allocation_size();
        }
    });
    return result;
}

} // namespace ocs::render::vulkan
