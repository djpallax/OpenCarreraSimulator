include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(ocs_tests
    core/test_bytes.cpp
    core/test_result.cpp
    core/test_slot_map.cpp
    core/test_types.cpp
    math/test_mat4f.cpp
    math/test_quatf.cpp
    math/test_vec3.cpp
    math/test_vec3f.cpp
    assets/test_mesh_io.cpp
    assets/test_model_io.cpp
    assets/test_texture_io.cpp
    world/test_world.cpp
    physics/test_fixed_step.cpp
    physics/test_physics_world.cpp
    physics/test_physics_thread.cpp
    render/test_debug_draw.cpp
)

target_link_libraries(ocs_tests
    PRIVATE
        ocs::project_options
        ocs::project_warnings
        ocs::sanitizers
        ocs::core
        ocs::math
        ocs::assets
        ocs::world
        ocs::physics
        ocs::render
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(ocs_tests)
