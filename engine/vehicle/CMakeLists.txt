add_library(ocs_vehicle STATIC
    src/vehicle.cpp
    src/vehicle_definition.cpp
)

add_library(ocs::vehicle ALIAS ocs_vehicle)
add_library(OpenCarreraSimulator::Vehicle ALIAS ocs_vehicle)

target_include_directories(ocs_vehicle PUBLIC include)

target_link_libraries(ocs_vehicle
    PUBLIC
        ocs::project_options
        ocs::math
        ocs::physics
    PRIVATE
        ocs::project_warnings
        ocs::sanitizers
)
