find_package(Threads REQUIRED)

add_library(ocs_physics STATIC
    src/physics_world.cpp
    src/physics_thread.cpp
)

add_library(ocs::physics ALIAS ocs_physics)
add_library(OpenCarreraSimulator::Physics ALIAS ocs_physics)

target_include_directories(ocs_physics PUBLIC include)

target_link_libraries(ocs_physics
    PUBLIC
        ocs::project_options
        ocs::core
        ocs::math
        Threads::Threads
    PRIVATE
        ocs::project_warnings
        ocs::sanitizers
)
