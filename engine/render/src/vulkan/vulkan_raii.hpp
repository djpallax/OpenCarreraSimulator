#pragma once

#include <vulkan/vulkan.h>

#include <utility>

namespace ocs::render::vulkan {

template <typename Handle, auto DestroyFn>
class UniqueDeviceHandle {
public:
    UniqueDeviceHandle() = default;
    ~UniqueDeviceHandle() { reset(); }

    UniqueDeviceHandle(const UniqueDeviceHandle&) = delete;
    UniqueDeviceHandle& operator=(const UniqueDeviceHandle&) = delete;

    UniqueDeviceHandle(UniqueDeviceHandle&& other) noexcept
        : device_(std::exchange(other.device_, VK_NULL_HANDLE)),
          handle_(std::exchange(other.handle_, VK_NULL_HANDLE)) {}

    UniqueDeviceHandle& operator=(UniqueDeviceHandle&& other) noexcept {
        if (this != &other) {
            reset();
            device_ = std::exchange(other.device_, VK_NULL_HANDLE);
            handle_ = std::exchange(other.handle_, VK_NULL_HANDLE);
        }
        return *this;
    }

    [[nodiscard]] Handle get() const noexcept { return handle_; }
    [[nodiscard]] explicit operator bool() const noexcept { return handle_ != VK_NULL_HANDLE; }

    Handle* put(const VkDevice device) noexcept {
        reset();
        device_ = device;
        return &handle_;
    }

    void reset() noexcept {
        if (device_ != VK_NULL_HANDLE && handle_ != VK_NULL_HANDLE) {
            DestroyFn(device_, handle_, nullptr);
        }
        handle_ = VK_NULL_HANDLE;
        device_ = VK_NULL_HANDLE;
    }

private:
    VkDevice device_ = VK_NULL_HANDLE;
    Handle handle_ = VK_NULL_HANDLE;
};

using UniqueCommandPool = UniqueDeviceHandle<VkCommandPool, vkDestroyCommandPool>;
using UniqueSemaphore = UniqueDeviceHandle<VkSemaphore, vkDestroySemaphore>;
using UniqueFence = UniqueDeviceHandle<VkFence, vkDestroyFence>;
using UniqueQueryPool = UniqueDeviceHandle<VkQueryPool, vkDestroyQueryPool>;
using UniqueDescriptorPool = UniqueDeviceHandle<VkDescriptorPool, vkDestroyDescriptorPool>;
using UniqueDescriptorSetLayout = UniqueDeviceHandle<VkDescriptorSetLayout, vkDestroyDescriptorSetLayout>;
using UniqueSampler = UniqueDeviceHandle<VkSampler, vkDestroySampler>;

} // namespace ocs::render::vulkan
