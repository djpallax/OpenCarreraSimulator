add_library(ocs_render STATIC
    src/renderer.cpp

    src/vulkan/vulkan_buffer.cpp
    src/vulkan/vulkan_context.cpp
    src/vulkan/vulkan_depth.cpp
    src/vulkan/vulkan_device.cpp
    src/vulkan/vulkan_immediate.cpp
    src/vulkan/vulkan_instance.cpp
    src/vulkan/vulkan_memory.cpp
    src/vulkan/vulkan_pipeline.cpp
    src/vulkan/vulkan_resource_manager.cpp
    src/vulkan/vulkan_shader.cpp
    src/vulkan/vulkan_swapchain.cpp
    src/vulkan/vulkan_texture.cpp
)

add_library(ocs::render ALIAS ocs_render)
add_library(OpenCarreraSimulator::Render ALIAS ocs_render)

target_include_directories(ocs_render
    PUBLIC include
    PRIVATE src
)

target_compile_definitions(ocs_render
    PRIVATE
        OCS_SHADER_BINARY_DIR="${OCS_SHADER_OUTPUT_DIR}"
)

target_link_libraries(ocs_render
    PUBLIC
        ocs::project_options
        ocs::core
        ocs::math
        ocs::platform
        ocs::assets
    PRIVATE
        ocs::project_warnings
        ocs::sanitizers
        SDL3::SDL3
        Vulkan::Vulkan
)

add_dependencies(ocs_render ocs_shaders)
