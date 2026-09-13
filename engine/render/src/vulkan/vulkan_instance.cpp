#include "vulkan_instance.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>

#include <algorithm>
#include <cstring>
#include <sstream>
#include <vector>

#include "ocs/core/log.hpp"
#include "vulkan_common.hpp"

namespace ocs::render::vulkan {
namespace {

constexpr const char* kValidationLayer = "VK_LAYER_KHRONOS_validation";

bool has_layer(const char* name) {
    std::uint32_t count = 0;
    if (vkEnumerateInstanceLayerProperties(&count, nullptr) != VK_SUCCESS) {
        return false;
    }

    std::vector<VkLayerProperties> layers(count);
    if (count > 0 && vkEnumerateInstanceLayerProperties(&count, layers.data()) != VK_SUCCESS) {
        return false;
    }

    return std::ranges::any_of(layers, [name](const VkLayerProperties& layer) {
        return std::strcmp(layer.layerName, name) == 0;
    });
}

bool has_instance_extension(const char* name) {
    std::uint32_t count = 0;
    if (vkEnumerateInstanceExtensionProperties(nullptr, &count, nullptr) != VK_SUCCESS) {
        return false;
    }

    std::vector<VkExtensionProperties> extensions(count);
    if (count > 0 && vkEnumerateInstanceExtensionProperties(nullptr, &count, extensions.data()) != VK_SUCCESS) {
        return false;
    }

    return std::ranges::any_of(extensions, [name](const VkExtensionProperties& extension) {
        return std::strcmp(extension.extensionName, name) == 0;
    });
}

VKAPI_ATTR VkBool32 VKAPI_CALL debug_callback(
    const VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT,
    const VkDebugUtilsMessengerCallbackDataEXT* callback_data,
    void*) {

    if (callback_data == nullptr || callback_data->pMessage == nullptr) {
        return VK_FALSE;
    }

    if ((severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) != 0U) {
        OCS_LOG_ERROR(callback_data->pMessage);
    } else if ((severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) != 0U) {
        OCS_LOG_WARN(callback_data->pMessage);
    } else {
        OCS_LOG_TRACE(callback_data->pMessage);
    }

    return VK_FALSE;
}

VkDebugUtilsMessengerCreateInfoEXT debug_messenger_create_info() {
    VkDebugUtilsMessengerCreateInfoEXT info{};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    info.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                           VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                       VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                       VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    info.pfnUserCallback = debug_callback;
    return info;
}

} // namespace

VulkanInstance::~VulkanInstance() {
    shutdown();
}

bool VulkanInstance::initialize(SDL_Window* window, const bool request_validation) {
    shutdown();

    if (window == nullptr) {
        OCS_LOG_ERROR("VulkanInstance received a null SDL window");
        return false;
    }

    loader_version_ = VK_API_VERSION_1_0;
    const auto enumerate_instance_version = reinterpret_cast<PFN_vkEnumerateInstanceVersion>(
        vkGetInstanceProcAddr(VK_NULL_HANDLE, "vkEnumerateInstanceVersion"));
    if (enumerate_instance_version != nullptr) {
        const VkResult result = enumerate_instance_version(&loader_version_);
        if (result != VK_SUCCESS) {
            OCS_LOG_ERROR("Could not query Vulkan loader version");
            return false;
        }
    }

    if (loader_version_ < kEngineVulkanApiVersion) {
        std::ostringstream stream;
        stream << "Vulkan " << version_string(kEngineVulkanApiVersion)
               << " is required; loader exposes " << version_string(loader_version_);
        OCS_LOG_ERROR(stream.str());
        return false;
    }

    instance_api_version_ = kEngineVulkanApiVersion;

    validation_enabled_ = request_validation && has_layer(kValidationLayer);
    if (request_validation && !validation_enabled_) {
        OCS_LOG_WARN("VK_LAYER_KHRONOS_validation not found; continuing without validation layers");
    }

    Uint32 sdl_extension_count = 0;
    const char* const* sdl_extensions = SDL_Vulkan_GetInstanceExtensions(&sdl_extension_count);
    if (sdl_extensions == nullptr) {
        OCS_LOG_ERROR(SDL_GetError());
        return false;
    }

    std::vector<const char*> extensions;
    extensions.reserve(static_cast<std::size_t>(sdl_extension_count) + 1U);
    for (Uint32 index = 0; index < sdl_extension_count; ++index) {
        extensions.push_back(sdl_extensions[index]);
    }

    debug_utils_enabled_ = validation_enabled_ && has_instance_extension(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    if (debug_utils_enabled_) {
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    } else if (validation_enabled_) {
        OCS_LOG_WARN("VK_EXT_debug_utils not available; validation callback disabled");
    }

    VkApplicationInfo application_info{};
    application_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    application_info.pApplicationName = "OpenCarreraSimulator";
    application_info.applicationVersion = VK_MAKE_VERSION(0, 2, 5);
    application_info.pEngineName = "OpenCarreraSimulator Engine";
    application_info.engineVersion = VK_MAKE_VERSION(0, 2, 5);
    application_info.apiVersion = instance_api_version_;

    VkDebugUtilsMessengerCreateInfoEXT debug_info = debug_messenger_create_info();

    VkInstanceCreateInfo create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    create_info.pApplicationInfo = &application_info;
    create_info.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
    create_info.ppEnabledExtensionNames = extensions.data();

    const char* layers[] = {kValidationLayer};
    if (validation_enabled_) {
        create_info.enabledLayerCount = 1;
        create_info.ppEnabledLayerNames = layers;
        if (debug_utils_enabled_) {
            create_info.pNext = &debug_info;
        }
    }

    if (!vk_check(vkCreateInstance(&create_info, nullptr, &instance_), "vkCreateInstance")) {
        return false;
    }

    if (debug_utils_enabled_ && !create_debug_messenger()) {
        shutdown();
        return false;
    }

    if (!SDL_Vulkan_CreateSurface(window, instance_, nullptr, &surface_)) {
        OCS_LOG_ERROR(SDL_GetError());
        shutdown();
        return false;
    }

    std::ostringstream stream;
    stream << "Vulkan loader " << version_string(loader_version_)
           << " | requested API " << version_string(kEngineVulkanApiVersion)
           << " | instance API " << version_string(instance_api_version_)
           << " | validation " << (validation_enabled_ ? "on" : "off");
    OCS_LOG_INFO(stream.str());
    return true;
}

bool VulkanInstance::create_debug_messenger() {
    const auto create_fn = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(instance_, "vkCreateDebugUtilsMessengerEXT"));
    if (create_fn == nullptr) {
        OCS_LOG_ERROR("vkCreateDebugUtilsMessengerEXT not found");
        return false;
    }

    const VkDebugUtilsMessengerCreateInfoEXT info = debug_messenger_create_info();
    return vk_check(create_fn(instance_, &info, nullptr, &debug_messenger_),
                    "vkCreateDebugUtilsMessengerEXT");
}

void VulkanInstance::shutdown() noexcept {
    if (instance_ == VK_NULL_HANDLE) {
        loader_version_ = VK_API_VERSION_1_0;
        instance_api_version_ = VK_API_VERSION_1_0;
        validation_enabled_ = false;
        debug_utils_enabled_ = false;
        return;
    }

    if (surface_ != VK_NULL_HANDLE) {
        SDL_Vulkan_DestroySurface(instance_, surface_, nullptr);
        surface_ = VK_NULL_HANDLE;
    }

    if (debug_messenger_ != VK_NULL_HANDLE) {
        const auto destroy_fn = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(instance_, "vkDestroyDebugUtilsMessengerEXT"));
        if (destroy_fn != nullptr) {
            destroy_fn(instance_, debug_messenger_, nullptr);
        }
        debug_messenger_ = VK_NULL_HANDLE;
    }

    vkDestroyInstance(instance_, nullptr);
    instance_ = VK_NULL_HANDLE;
    loader_version_ = VK_API_VERSION_1_0;
    instance_api_version_ = VK_API_VERSION_1_0;
    validation_enabled_ = false;
    debug_utils_enabled_ = false;
}

} // namespace ocs::render::vulkan
