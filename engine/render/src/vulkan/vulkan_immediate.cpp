#include "vulkan_immediate.hpp"

#include <limits>

#include "vulkan_common.hpp"
#include "vulkan_raii.hpp"

namespace ocs::render::vulkan {

bool immediate_submit(const VkDevice device,
                      const VkQueue queue,
                      const std::uint32_t queue_family,
                      const std::function<void(VkCommandBuffer)>& recorder) {
    UniqueCommandPool command_pool{};
    VkCommandPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    pool_info.queueFamilyIndex = queue_family;
    if (!vk_check(vkCreateCommandPool(device, &pool_info, nullptr, command_pool.put(device)),
                  "vkCreateCommandPool(immediate)")) {
        return false;
    }

    VkCommandBuffer command_buffer = VK_NULL_HANDLE;
    VkCommandBufferAllocateInfo allocate_info{};
    allocate_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocate_info.commandPool = command_pool.get();
    allocate_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocate_info.commandBufferCount = 1;
    if (!vk_check(vkAllocateCommandBuffers(device, &allocate_info, &command_buffer),
                  "vkAllocateCommandBuffers(immediate)")) {
        return false;
    }

    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (!vk_check(vkBeginCommandBuffer(command_buffer, &begin_info),
                  "vkBeginCommandBuffer(immediate)")) {
        return false;
    }

    recorder(command_buffer);

    if (!vk_check(vkEndCommandBuffer(command_buffer), "vkEndCommandBuffer(immediate)")) {
        return false;
    }

    UniqueFence fence{};
    VkFenceCreateInfo fence_info{};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (!vk_check(vkCreateFence(device, &fence_info, nullptr, fence.put(device)),
                  "vkCreateFence(immediate)")) {
        return false;
    }

    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &command_buffer;
    if (!vk_check(vkQueueSubmit(queue, 1, &submit_info, fence.get()),
                  "vkQueueSubmit(immediate)")) {
        return false;
    }

    const VkFence wait_fence = fence.get();
    return vk_check(
        vkWaitForFences(device,
                        1,
                        &wait_fence,
                        VK_TRUE,
                        std::numeric_limits<std::uint64_t>::max()),
        "vkWaitForFences(immediate)");
}

} // namespace ocs::render::vulkan
