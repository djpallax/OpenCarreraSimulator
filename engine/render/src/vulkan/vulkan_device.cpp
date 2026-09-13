#include "vulkan_device.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>
#include <utility>
#include <vector>

#include "ocs/core/bytes.hpp"
#include "ocs/core/log.hpp"
#include "vulkan_common.hpp"

namespace ocs::render::vulkan {
namespace {

constexpr std::array<const char*, 1> kRequiredDeviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME
};

struct Candidate {
    VkPhysicalDevice device = VK_NULL_HANDLE;
    QueueFamilies queues{};
    GPUInfo info{};
    std::int64_t score = std::numeric_limits<std::int64_t>::min();
    std::uint32_t enumeration_index = 0;
    bool technically_suitable = false;
};

std::string lowercase(std::string value) {
    std::ranges::transform(value, value.begin(), [](const unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::string vendor_name(const std::uint32_t vendor_id) {
    switch (vendor_id) {
        case 0x10DE: return "NVIDIA";
        case 0x1002: return "AMD";
        case 0x8086: return "Intel";
        default: return "Unknown";
    }
}

GPUType gpu_type(const VkPhysicalDeviceType type) noexcept {
    switch (type) {
        case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU: return GPUType::discrete;
        case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: return GPUType::integrated;
        case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU: return GPUType::virtual_gpu;
        case VK_PHYSICAL_DEVICE_TYPE_CPU: return GPUType::cpu;
        default: return GPUType::other;
    }
}

QueueFamilies find_queue_families(const VkPhysicalDevice device, const VkSurfaceKHR surface) {
    QueueFamilies result{};
    std::uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);
    std::vector<VkQueueFamilyProperties> properties(count);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, properties.data());

    for (std::uint32_t index = 0; index < count; ++index) {
        if ((properties[index].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0U) {
            result.graphics = index;
        }

        VkBool32 present_supported = VK_FALSE;
        if (vkGetPhysicalDeviceSurfaceSupportKHR(device, index, surface, &present_supported) == VK_SUCCESS &&
            present_supported == VK_TRUE) {
            result.present = index;
        }

        if (result.complete()) {
            break;
        }
    }

    return result;
}

std::vector<VkExtensionProperties> enumerate_device_extensions(const VkPhysicalDevice device) {
    std::uint32_t count = 0;
    if (vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr) != VK_SUCCESS) {
        return {};
    }

    std::vector<VkExtensionProperties> extensions(count);
    if (count > 0 && vkEnumerateDeviceExtensionProperties(device, nullptr, &count, extensions.data()) != VK_SUCCESS) {
        return {};
    }
    return extensions;
}

bool has_extension(const std::vector<VkExtensionProperties>& extensions, const char* name) {
    return std::ranges::any_of(extensions, [name](const VkExtensionProperties& extension) {
        return std::strcmp(extension.extensionName, name) == 0;
    });
}

bool supports_required_extensions(const std::vector<VkExtensionProperties>& extensions) {
    return std::ranges::all_of(kRequiredDeviceExtensions, [&extensions](const char* required) {
        return has_extension(extensions, required);
    });
}

bool has_swapchain_support(const VkPhysicalDevice device, const VkSurfaceKHR surface) {
    std::uint32_t format_count = 0;
    std::uint32_t present_mode_count = 0;
    if (vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &format_count, nullptr) != VK_SUCCESS) {
        return false;
    }
    if (vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &present_mode_count, nullptr) != VK_SUCCESS) {
        return false;
    }
    return format_count > 0 && present_mode_count > 0;
}

GPUMemoryInfo query_memory_info(const VkPhysicalDevice device,
                                const GPUType type,
                                const bool supports_memory_budget) {
    VkPhysicalDeviceMemoryBudgetPropertiesEXT budget{};
    budget.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_BUDGET_PROPERTIES_EXT;

    VkPhysicalDeviceMemoryProperties2 properties{};
    properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_PROPERTIES_2;
    properties.pNext = supports_memory_budget ? &budget : nullptr;
    vkGetPhysicalDeviceMemoryProperties2(device, &properties);

    GPUMemoryInfo info{};
    info.unified = type == GPUType::integrated || type == GPUType::cpu;
    info.budget_available = supports_memory_budget;

    for (std::uint32_t index = 0; index < properties.memoryProperties.memoryHeapCount; ++index) {
        const VkMemoryHeap& heap = properties.memoryProperties.memoryHeaps[index];
        if ((heap.flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) == 0U) {
            continue;
        }

        info.device_local += core::ByteSize::from_bytes(heap.size);
        if (type == GPUType::discrete) {
            info.dedicated += core::ByteSize::from_bytes(heap.size);
        }

        if (supports_memory_budget) {
            info.budget += core::ByteSize::from_bytes(budget.heapBudget[index]);
            info.usage += core::ByteSize::from_bytes(budget.heapUsage[index]);
        }
    }

    return info;
}

Candidate inspect_candidate(const VkPhysicalDevice device,
                            const VkSurfaceKHR surface,
                            const std::uint32_t enumeration_index) {
    Candidate candidate{};
    candidate.device = device;
    candidate.enumeration_index = enumeration_index;
    candidate.queues = find_queue_families(device, surface);

    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(device, &properties);

    const auto extensions = enumerate_device_extensions(device);
    const bool supports_memory_budget = has_extension(extensions, VK_EXT_MEMORY_BUDGET_EXTENSION_NAME);

    VkPhysicalDeviceVulkan13Features vulkan13{};
    vulkan13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
    VkPhysicalDeviceVulkan12Features vulkan12{};
    vulkan12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
    vulkan13.pNext = &vulkan12;

    VkPhysicalDeviceFeatures2 features{};
    features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    features.pNext = &vulkan13;
    vkGetPhysicalDeviceFeatures2(device, &features);

    candidate.info.name = properties.deviceName;
    candidate.info.vendor_id = properties.vendorID;
    candidate.info.device_id = properties.deviceID;
    candidate.info.vendor = vendor_name(properties.vendorID);
    candidate.info.api_version = properties.apiVersion;
    candidate.info.type = gpu_type(properties.deviceType);
    candidate.info.memory = query_memory_info(device, candidate.info.type, supports_memory_budget);
    candidate.info.capabilities.dynamic_rendering = vulkan13.dynamicRendering == VK_TRUE;
    candidate.info.capabilities.synchronization2 = vulkan13.synchronization2 == VK_TRUE;
    candidate.info.capabilities.timeline_semaphores = vulkan12.timelineSemaphore == VK_TRUE;
    candidate.info.capabilities.descriptor_indexing = vulkan12.runtimeDescriptorArray == VK_TRUE &&
                                                       vulkan12.descriptorBindingPartiallyBound == VK_TRUE;
    candidate.info.capabilities.mesh_shaders = has_extension(extensions, "VK_EXT_mesh_shader");
    candidate.info.capabilities.ray_tracing = has_extension(extensions, "VK_KHR_ray_tracing_pipeline") &&
                                               has_extension(extensions, "VK_KHR_acceleration_structure");
    candidate.info.capabilities.variable_rate_shading =
        has_extension(extensions, "VK_KHR_fragment_shading_rate");
    candidate.info.capabilities.memory_budget = supports_memory_budget;

    candidate.technically_suitable = properties.apiVersion >= kEngineVulkanApiVersion &&
                                     candidate.queues.complete() &&
                                     supports_required_extensions(extensions) &&
                                     has_swapchain_support(device, surface) &&
                                     candidate.info.capabilities.dynamic_rendering &&
                                     candidate.info.capabilities.synchronization2;

    if (!candidate.technically_suitable) {
        return candidate;
    }

    candidate.score = 100;
    switch (candidate.info.type) {
        case GPUType::discrete: candidate.score += 10'000; break;
        case GPUType::integrated: candidate.score += 2'000; break;
        case GPUType::virtual_gpu: candidate.score += 500; break;
        case GPUType::cpu: candidate.score -= 10'000; break;
        case GPUType::other: break;
    }

    candidate.score += static_cast<std::int64_t>(candidate.info.memory.device_local.mebibytes() / 256ULL);
    candidate.score += static_cast<std::int64_t>(properties.limits.maxImageDimension2D / 1024U);
    return candidate;
}

bool selector_matches(const Candidate& candidate, const std::string& selector) {
    if (selector.empty()) {
        return true;
    }

    try {
        std::size_t consumed = 0;
        const unsigned long numeric = std::stoul(selector, &consumed, 10);
        if (consumed == selector.size()) {
            return numeric == candidate.enumeration_index;
        }
    } catch (...) {
        // Non-numeric selectors are matched by device-name substring below.
    }

    const std::string device_name = lowercase(candidate.info.name);
    const std::string wanted = lowercase(selector);
    return device_name.find(wanted) != std::string::npos;
}

void log_candidate(const Candidate& candidate) {
    std::ostringstream stream;
    stream << "GPU[" << candidate.enumeration_index << "] " << candidate.info.name
           << " | " << candidate.info.vendor
           << " | " << gpu_type_name(candidate.info.type)
           << " | device API " << version_string(candidate.info.api_version);

    if (candidate.info.memory.unified) {
        stream << " | unified device-local " << candidate.info.memory.device_local.mebibytes() << " MiB";
    } else {
        stream << " | dedicated " << candidate.info.memory.dedicated.mebibytes() << " MiB";
    }

    if (candidate.info.memory.budget_available) {
        stream << " | budget " << candidate.info.memory.budget.mebibytes() << " MiB"
               << " | usage " << candidate.info.memory.usage.mebibytes() << " MiB";
    }

    if (!candidate.technically_suitable) {
        stream << " | unsuitable";
    } else {
        stream << " | score " << candidate.score;
    }

    OCS_LOG_INFO(stream.str());
}

} // namespace

VulkanDevice::~VulkanDevice() {
    shutdown();
}

bool VulkanDevice::initialize(const VkInstance instance,
                              const VkSurfaceKHR surface,
                              const std::string& gpu_selector,
                              const bool allow_software_gpu) {
    shutdown();

    std::uint32_t device_count = 0;
    if (!vk_check(vkEnumeratePhysicalDevices(instance, &device_count, nullptr),
                  "vkEnumeratePhysicalDevices(count)")) {
        return false;
    }
    if (device_count == 0) {
        OCS_LOG_ERROR("No Vulkan-capable GPU found");
        return false;
    }

    std::vector<VkPhysicalDevice> devices(device_count);
    if (!vk_check(vkEnumeratePhysicalDevices(instance, &device_count, devices.data()),
                  "vkEnumeratePhysicalDevices")) {
        return false;
    }

    std::optional<Candidate> selected;
    for (std::uint32_t index = 0; index < device_count; ++index) {
        Candidate candidate = inspect_candidate(devices[index], surface, index);
        log_candidate(candidate);

        if (!candidate.technically_suitable) {
            continue;
        }
        if (candidate.info.is_software() && !allow_software_gpu) {
            continue;
        }
        if (!selector_matches(candidate, gpu_selector)) {
            continue;
        }
        if (!selected.has_value() || candidate.score > selected->score) {
            selected = std::move(candidate);
        }
    }

    if (!selected.has_value()) {
        if (gpu_selector.empty()) {
            OCS_LOG_ERROR("No suitable Vulkan 1.3 GPU found");
        } else if (!allow_software_gpu) {
            OCS_LOG_ERROR("Requested GPU was not suitable. Software devices require --allow-software-gpu: " +
                          gpu_selector);
        } else {
            OCS_LOG_ERROR("Requested GPU selector did not match a suitable Vulkan 1.3 device: " + gpu_selector);
        }
        return false;
    }

    physical_device_ = selected->device;
    queue_families_ = selected->queues;
    gpu_info_ = selected->info;

    VkPhysicalDeviceProperties selected_properties{};
    vkGetPhysicalDeviceProperties(physical_device_, &selected_properties);
    timestamp_period_ns_ = selected_properties.limits.timestampPeriod;

    std::uint32_t queue_family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device_, &queue_family_count, nullptr);
    std::vector<VkQueueFamilyProperties> queue_properties(queue_family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device_, &queue_family_count, queue_properties.data());
    if (queue_families_.graphics < queue_properties.size()) {
        timestamp_valid_bits_ = queue_properties[queue_families_.graphics].timestampValidBits;
    }

    std::set<std::uint32_t> unique_families = {
        queue_families_.graphics,
        queue_families_.present
    };

    constexpr float kQueuePriority = 1.0F;
    std::vector<VkDeviceQueueCreateInfo> queue_infos;
    queue_infos.reserve(unique_families.size());
    for (const std::uint32_t family : unique_families) {
        VkDeviceQueueCreateInfo queue_info{};
        queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queue_info.queueFamilyIndex = family;
        queue_info.queueCount = 1;
        queue_info.pQueuePriorities = &kQueuePriority;
        queue_infos.push_back(queue_info);
    }

    VkPhysicalDeviceVulkan13Features vulkan13{};
    vulkan13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
    vulkan13.dynamicRendering = VK_TRUE;
    vulkan13.synchronization2 = VK_TRUE;

    VkDeviceCreateInfo create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    create_info.pNext = &vulkan13;
    create_info.queueCreateInfoCount = static_cast<std::uint32_t>(queue_infos.size());
    create_info.pQueueCreateInfos = queue_infos.data();
    create_info.enabledExtensionCount = static_cast<std::uint32_t>(kRequiredDeviceExtensions.size());
    create_info.ppEnabledExtensionNames = kRequiredDeviceExtensions.data();

    if (!vk_check(vkCreateDevice(physical_device_, &create_info, nullptr, &device_), "vkCreateDevice")) {
        shutdown();
        return false;
    }

    vkGetDeviceQueue(device_, queue_families_.graphics, 0, &graphics_queue_);
    vkGetDeviceQueue(device_, queue_families_.present, 0, &present_queue_);

    std::ostringstream stream;
    stream << "Selected GPU: " << gpu_info_.name
           << " (" << gpu_info_.vendor
           << ", " << gpu_type_name(gpu_info_.type)
           << ", device API " << version_string(gpu_info_.api_version)
           << ", engine API " << version_string(kEngineVulkanApiVersion) << ')';
    OCS_LOG_INFO(stream.str());
    return true;
}

void VulkanDevice::shutdown() noexcept {
    if (device_ != VK_NULL_HANDLE) {
        vkDestroyDevice(device_, nullptr);
        device_ = VK_NULL_HANDLE;
    }

    physical_device_ = VK_NULL_HANDLE;
    graphics_queue_ = VK_NULL_HANDLE;
    present_queue_ = VK_NULL_HANDLE;
    queue_families_ = {};
    gpu_info_ = {};
    timestamp_period_ns_ = 0.0F;
    timestamp_valid_bits_ = 0;
}

} // namespace ocs::render::vulkan
