include(FetchContent)

# Pin the parser and its JSON backend. fastgltf remains an offline build-tool
# dependency only; OpenCarreraSimulator itself never links either library.
FetchContent_Declare(
    simdjson
    URL https://github.com/simdjson/simdjson/archive/refs/tags/v4.2.2.tar.gz
    URL_HASH SHA256=3efae22cb41f83299fe0b2e8a187af543d3dda93abbb910586f897df670f9eaa
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(simdjson)

set(FASTGLTF_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(FASTGLTF_ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
set(FASTGLTF_ENABLE_DOCS OFF CACHE BOOL "" FORCE)
set(FASTGLTF_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(FASTGLTF_COMPILE_AS_CPP20 ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    fastgltf
    URL https://github.com/spnda/fastgltf/archive/refs/tags/v0.9.0.tar.gz
    URL_HASH SHA256=0bb564e127b14c22f062db50f89381dd2e0a20dbaf4987ca138a4ae8728712f9
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(fastgltf)

# stb_image is used only by the offline asset compiler to decode source PNG/JPEG
# images. Runtime code consumes .ocstex and therefore has no image-codec dependency.
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG 2c980bb59875b0d32144a71867fbdebb2f77cd20
)
FetchContent_MakeAvailable(stb)

# Third-party headers must not inherit OpenCarreraSimulator's intentionally
# strict warning policy. CMake's SYSTEM target property makes the dependency's
# INTERFACE include directories system headers for consumers, while warnings in
# our own ocs_assetc sources remain enabled.
if(TARGET fastgltf)
    set_property(TARGET fastgltf PROPERTY SYSTEM TRUE)
endif()
if(TARGET simdjson)
    set_property(TARGET simdjson PROPERTY SYSTEM TRUE)
endif()

add_executable(ocs_assetc
    main.cpp
    stb_image_impl.cpp
)

target_link_libraries(ocs_assetc
    PRIVATE
        ocs::project_options
        ocs::project_warnings
        ocs::sanitizers
        ocs::core
        ocs::math
        ocs::assets
        fastgltf::fastgltf
)

# Treat the third-party stb header as a system include so our warning policy
# remains focused on OpenCarreraSimulator code.
target_include_directories(ocs_assetc SYSTEM PRIVATE "${stb_SOURCE_DIR}")
