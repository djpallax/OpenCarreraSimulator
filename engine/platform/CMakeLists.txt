add_library(ocs_platform STATIC
    src/sdl_platform.cpp
    src/sdl_debug_text_window.cpp
)

add_library(ocs::platform ALIAS ocs_platform)
add_library(OpenCarreraSimulator::Platform ALIAS ocs_platform)

target_include_directories(ocs_platform PUBLIC include)
target_link_libraries(ocs_platform
    PUBLIC
        ocs::project_options
        ocs::core
        SDL3::SDL3
    PRIVATE
        ocs::project_warnings
        ocs::sanitizers
)
