#include "vulkan_memory.hpp"

#include <iomanip>
#include <sstream>

#include "ocs/core/log.hpp"
#include "vulkan_common.hpp"

namespace ocs::render::vulkan {

void VulkanMemoryAllocator::initialize(const VkPhysicalDevice physical_device,
                                       const VkDevice device) noexcept {
    physical_device_ = physical_device;
    device_ = device;
    vkGetPhysicalDeviceMemoryProperties(physical_device_, &memory_properties_);
}

void VulkanMemoryAllocator::shutdown() noexcept {
    physical_device_ = VK_NULL_HANDLE;
    device_ = VK_NULL_HANDLE;
    memory_properties_ = {};
}

bool VulkanMemoryAllocator::find_memory_type(const std::uint32_t type_filter,
                                              const VkMemoryPropertyFlags properties,
                                              std::uint32_t& out_index) const noexcept {
    for (std::uint32_t index = 0; index < memory_properties_.memoryTypeCount; ++index) {
        const bool supported = (type_filter & (1U << index)) != 0U;
        const VkMemoryPropertyFlags available = memory_properties_.memoryTypes[index].propertyFlags;
        if (supported && (available & properties) == properties) {
            out_index = index;
            return true;
        }
    }

    return false;
}

bool VulkanMemoryAllocator::allocate(const VkMemoryRequirements& requirements,
                                     const VkMemoryPropertyFlags properties,
                                     VkDeviceMemory& out_memory) const {
    if (!initialized()) {
        OCS_LOG_ERROR("VulkanMemoryAllocator used before initialization");
        return false;
    }

    std::uint32_t memory_type_index = 0;
    if (!find_memory_type(requirements.memoryTypeBits, properties, memory_type_index)) {
        std::ostringstream stream;
        stream << "No compatible Vulkan memory type found for properties 0x"
               << std::hex << properties;
        OCS_LOG_ERROR(stream.str());
        return false;
    }

    VkMemoryAllocateInfo allocate_info{};
    allocate_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocate_info.allocationSize = requirements.size;
    allocate_info.memoryTypeIndex = memory_type_index;

    return vk_check(vkAllocateMemory(device_, &allocate_info, nullptr, &out_memory),
                    "vkAllocateMemory");
}

} // namespace ocs::render::vulkan
