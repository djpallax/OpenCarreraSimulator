#include "vulkan_shader.hpp"

#include <cstdint>
#include <fstream>
#include <sstream>
#include <utility>
#include <vector>

#include "ocs/core/log.hpp"
#include "vulkan_common.hpp"

namespace ocs::render::vulkan {

VulkanShaderModule::~VulkanShaderModule() {
    shutdown();
}

VulkanShaderModule::VulkanShaderModule(VulkanShaderModule&& other) noexcept
    : device_(std::exchange(other.device_, VK_NULL_HANDLE)),
      module_(std::exchange(other.module_, VK_NULL_HANDLE)) {}

VulkanShaderModule& VulkanShaderModule::operator=(VulkanShaderModule&& other) noexcept {
    if (this != &other) {
        shutdown();
        device_ = std::exchange(other.device_, VK_NULL_HANDLE);
        module_ = std::exchange(other.module_, VK_NULL_HANDLE);
    }
    return *this;
}

bool VulkanShaderModule::load(const VkDevice device,
                              const std::filesystem::path& path) {
    shutdown();

    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        OCS_LOG_ERROR("Unable to open SPIR-V shader: " + path.string());
        return false;
    }

    const std::streampos end_position = file.tellg();
    if (end_position <= 0) {
        OCS_LOG_ERROR("SPIR-V shader is empty: " + path.string());
        return false;
    }

    const std::streamoff signed_byte_count = end_position;
    const auto byte_count = static_cast<std::size_t>(signed_byte_count);
    if ((byte_count % sizeof(std::uint32_t)) != 0U) {
        OCS_LOG_ERROR("SPIR-V shader size is not aligned to 32 bits: " + path.string());
        return false;
    }

    std::vector<std::uint32_t> words(byte_count / sizeof(std::uint32_t));
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char*>(words.data()),
              static_cast<std::streamsize>(byte_count));
    if (!file) {
        OCS_LOG_ERROR("Failed while reading SPIR-V shader: " + path.string());
        return false;
    }

    VkShaderModuleCreateInfo create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    create_info.codeSize = byte_count;
    create_info.pCode = words.data();

    VkShaderModule new_module = VK_NULL_HANDLE;
    if (!vk_check(vkCreateShaderModule(device, &create_info, nullptr, &new_module),
                  "vkCreateShaderModule")) {
        return false;
    }

    device_ = device;
    module_ = new_module;
    return true;
}

void VulkanShaderModule::shutdown() noexcept {
    if (device_ != VK_NULL_HANDLE && module_ != VK_NULL_HANDLE) {
        vkDestroyShaderModule(device_, module_, nullptr);
    }
    device_ = VK_NULL_HANDLE;
    module_ = VK_NULL_HANDLE;
}

} // namespace ocs::render::vulkan
