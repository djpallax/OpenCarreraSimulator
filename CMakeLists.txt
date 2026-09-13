cmake_minimum_required(VERSION 3.25)

project(OpenCarreraSimulator
    VERSION 0.8.11
    DESCRIPTION "Linux-first, cross-platform racing simulator engine"
    LANGUAGES CXX
)

option(OCS_BUILD_TESTS "Build tests" ON)
option(OCS_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(OCS_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(OCS_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(OCS_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build OpenCarreraSimulator engine modules as static libraries" FORCE)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(ProjectOptions)
ocs_create_project_targets()

find_package(SDL3 CONFIG REQUIRED)
find_package(Vulkan REQUIRED)

add_subdirectory(shaders)
add_subdirectory(engine/core)
add_subdirectory(engine/math)
add_subdirectory(engine/platform)
add_subdirectory(engine/assets)
add_subdirectory(engine/render)
add_subdirectory(engine/world)
add_subdirectory(engine/physics)
add_subdirectory(tools/assetc)
add_subdirectory(tools/trackgen)
add_subdirectory(assets_src)
add_subdirectory(app)

if(OCS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
