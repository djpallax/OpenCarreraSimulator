add_library(ocs_assets STATIC
    src/mesh_io.cpp
    src/model_io.cpp
    src/texture_io.cpp
)

add_library(ocs::assets ALIAS ocs_assets)
add_library(OpenCarreraSimulator::Assets ALIAS ocs_assets)

target_include_directories(ocs_assets PUBLIC include)
target_link_libraries(ocs_assets
    PUBLIC
        ocs::project_options
        ocs::core
        ocs::math
    PRIVATE
        ocs::project_warnings
        ocs::sanitizers
)
