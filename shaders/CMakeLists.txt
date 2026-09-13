if(DEFINED Vulkan_GLSLC_EXECUTABLE AND EXISTS "${Vulkan_GLSLC_EXECUTABLE}")
    set(OCS_GLSLC_EXECUTABLE "${Vulkan_GLSLC_EXECUTABLE}")
else()
    find_program(OCS_GLSLC_EXECUTABLE NAMES glslc REQUIRED)
endif()

set(OCS_SHADER_OUTPUT_DIR "${CMAKE_BINARY_DIR}/shaders" CACHE INTERNAL
    "OpenCarreraSimulator compiled shader directory")

function(ocs_compile_shader SOURCE RELATIVE_OUTPUT)
    set(INPUT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE}")
    set(OUTPUT_FILE "${OCS_SHADER_OUTPUT_DIR}/${RELATIVE_OUTPUT}")
    get_filename_component(OUTPUT_DIRECTORY "${OUTPUT_FILE}" DIRECTORY)

    add_custom_command(
        OUTPUT "${OUTPUT_FILE}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${OUTPUT_DIRECTORY}"
        COMMAND "${OCS_GLSLC_EXECUTABLE}"
                --target-env=vulkan1.3
                -g
                "${INPUT_FILE}"
                -o "${OUTPUT_FILE}"
        DEPENDS "${INPUT_FILE}"
        COMMENT "Compiling shader ${SOURCE}"
        VERBATIM
    )

    set(OCS_COMPILED_SHADERS
        ${OCS_COMPILED_SHADERS} "${OUTPUT_FILE}"
        PARENT_SCOPE
    )
endfunction()

set(OCS_COMPILED_SHADERS)
ocs_compile_shader("debug/basic.vert" "debug/basic.vert.spv")
ocs_compile_shader("debug/basic.frag" "debug/basic.frag.spv")
ocs_compile_shader("debug/physics_lines.vert" "debug/physics_lines.vert.spv")
ocs_compile_shader("debug/physics_lines.frag" "debug/physics_lines.frag.spv")
ocs_compile_shader("material/simple.vert" "material/simple.vert.spv")
ocs_compile_shader("material/simple.frag" "material/simple.frag.spv")

add_custom_target(ocs_shaders ALL DEPENDS ${OCS_COMPILED_SHADERS})
