add_executable(OpenCarreraSimulator
    src/main.cpp
)

target_link_libraries(OpenCarreraSimulator
    PRIVATE
        ocs::project_options
        ocs::project_warnings
        ocs::sanitizers
        ocs::core
        ocs::math
        ocs::platform
        ocs::render
        ocs::world
        ocs::physics
        ocs::vehicle
        ocs::assets
)

add_dependencies(OpenCarreraSimulator ocs_sample_assets)

target_compile_definitions(OpenCarreraSimulator
    PRIVATE
        OCS_DEFAULT_ASSET_PATH="${OCS_VEHICLE_BODY_PATH}"
        OCS_DEVELOPMENT_TRACK_PATH="${OCS_DEVELOPMENT_TRACK_PATH}"
        OCS_TEST_CIRCUIT_PATH="${OCS_TEST_CIRCUIT_PATH}"
        OCS_TEST_CIRCUIT_COLLISION_PATH="${OCS_TEST_CIRCUIT_COLLISION_PATH}"
        OCS_VEHICLE_WHEEL_PATH="${OCS_VEHICLE_WHEEL_PATH}"
)
