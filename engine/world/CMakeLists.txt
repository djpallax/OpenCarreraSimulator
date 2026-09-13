add_library(ocs_world STATIC
    src/world.cpp
)

add_library(ocs::world ALIAS ocs_world)
add_library(OpenCarreraSimulator::World ALIAS ocs_world)

target_include_directories(ocs_world PUBLIC include)

target_link_libraries(ocs_world
    PUBLIC
        ocs::project_options
        ocs::core
        ocs::math
        ocs::render
    PRIVATE
        ocs::project_warnings
        ocs::sanitizers
)
