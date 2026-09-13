set(OCS_SAMPLE_ASSET_OUTPUT_DIR "${CMAKE_BINARY_DIR}/assets/reference")
set(OCS_SAMPLE_MODEL_PATH "${OCS_SAMPLE_ASSET_OUTPUT_DIR}/test1.ocsmodel"
    CACHE INTERNAL "OpenCarreraSimulator Step 7 default runtime model")
set(OCS_SAMPLE_TEXTURE_PATH "${OCS_SAMPLE_ASSET_OUTPUT_DIR}/test1.tex0.ocstex"
    CACHE INTERNAL "OpenCarreraSimulator Step 7 default runtime texture")

add_custom_command(
    OUTPUT
        "${OCS_SAMPLE_MODEL_PATH}"
        "${OCS_SAMPLE_TEXTURE_PATH}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${OCS_SAMPLE_ASSET_OUTPUT_DIR}"
    COMMAND $<TARGET_FILE:ocs_assetc>
            "${CMAKE_CURRENT_SOURCE_DIR}/reference/test1.glb"
            "${OCS_SAMPLE_MODEL_PATH}"
    DEPENDS
        ocs_assetc
        "${CMAKE_CURRENT_SOURCE_DIR}/reference/test1.glb"
    COMMENT "Compiling Step 7 textured Blender reference model"
    VERBATIM
)

set(OCS_WORLD_ASSET_OUTPUT_DIR "${CMAKE_BINARY_DIR}/assets/world")
set(OCS_DEVELOPMENT_TRACK_PATH "${OCS_WORLD_ASSET_OUTPUT_DIR}/development_straight.ocsmodel"
    CACHE INTERNAL "Step 7 visual development straight")
set(OCS_DEVELOPMENT_COLLISION_PATH "${OCS_WORLD_ASSET_OUTPUT_DIR}/development_straight_collision.ocsmesh"
    CACHE INTERNAL "Step 7 collision development straight")

add_custom_command(
    OUTPUT
        "${OCS_DEVELOPMENT_TRACK_PATH}"
        "${OCS_DEVELOPMENT_COLLISION_PATH}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${OCS_WORLD_ASSET_OUTPUT_DIR}"
    COMMAND $<TARGET_FILE:ocs_trackgen>
            "${OCS_DEVELOPMENT_TRACK_PATH}"
            "${OCS_DEVELOPMENT_COLLISION_PATH}"
    DEPENDS ocs_trackgen
    COMMENT "Generating Step 7 500 m development straight"
    VERBATIM
)



# Step 9.1 physics test circuit. Visual source now includes the orange sawtooth
# ramp while collision uses a dedicated source with only the drivable ramp face.
# This keeps presentation and collision independently authorable.
set(OCS_TEST_CIRCUIT_PATH "${OCS_WORLD_ASSET_OUTPUT_DIR}/test_circuit.ocsmodel"
    CACHE INTERNAL "Step 9.1 visual physics test circuit + sawtooth ramp")
set(OCS_TEST_CIRCUIT_COLLISION_PATH "${OCS_WORLD_ASSET_OUTPUT_DIR}/test_circuit_collision.ocsmesh"
    CACHE INTERNAL "Step 9.1 static collision mesh")

add_custom_command(
    OUTPUT
        "${OCS_TEST_CIRCUIT_PATH}"
        "${OCS_TEST_CIRCUIT_COLLISION_PATH}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${OCS_WORLD_ASSET_OUTPUT_DIR}"
    COMMAND $<TARGET_FILE:ocs_assetc>
            "${CMAKE_CURRENT_SOURCE_DIR}/world/test_circuit.glb"
            "${OCS_TEST_CIRCUIT_PATH}"
    COMMAND $<TARGET_FILE:ocs_assetc>
            "${CMAKE_CURRENT_SOURCE_DIR}/world/test_circuit_collision.glb"
            "${OCS_TEST_CIRCUIT_COLLISION_PATH}"
    DEPENDS
        ocs_assetc
        "${CMAKE_CURRENT_SOURCE_DIR}/world/test_circuit.glb"
        "${CMAKE_CURRENT_SOURCE_DIR}/world/test_circuit_collision.glb"
    COMMENT "Compiling Step 9 test circuit (visual + dedicated collision source)"
    VERBATIM
)


# Step 9.6.1 per-vehicle runtime directory. Source models and vehicle.cfg live
# together; the build mirrors that layout under build/assets/vehicle/<vehicle>.
set(OCS_VEHICLE_ASSET_OUTPUT_DIR "${CMAKE_BINARY_DIR}/assets/vehicle/ocs_lab_gt")
set(OCS_DEFAULT_VEHICLE_DIR "${OCS_VEHICLE_ASSET_OUTPUT_DIR}"
    CACHE INTERNAL "Default Step 9.6.1 vehicle profile directory")
set(OCS_VEHICLE_BODY_PATH "${OCS_VEHICLE_ASSET_OUTPUT_DIR}/body.ocsmodel"
    CACHE INTERNAL "OCS Lab GT body")
set(OCS_VEHICLE_WHEEL_PATH "${OCS_VEHICLE_ASSET_OUTPUT_DIR}/wheel.ocsmodel"
    CACHE INTERNAL "OCS Lab GT wheel")
set(OCS_VEHICLE_CONFIG_PATH "${OCS_VEHICLE_ASSET_OUTPUT_DIR}/vehicle.cfg"
    CACHE INTERNAL "OCS Lab GT runtime vehicle profile")

add_custom_command(
    OUTPUT
        "${OCS_VEHICLE_BODY_PATH}"
        "${OCS_VEHICLE_WHEEL_PATH}"
        "${OCS_VEHICLE_CONFIG_PATH}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${OCS_VEHICLE_ASSET_OUTPUT_DIR}"
    COMMAND $<TARGET_FILE:ocs_assetc>
            "${CMAKE_CURRENT_SOURCE_DIR}/vehicle/ocs_lab_gt/body.glb"
            "${OCS_VEHICLE_BODY_PATH}"
    COMMAND $<TARGET_FILE:ocs_assetc>
            "${CMAKE_CURRENT_SOURCE_DIR}/vehicle/ocs_lab_gt/wheel.glb"
            "${OCS_VEHICLE_WHEEL_PATH}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/vehicle/ocs_lab_gt/vehicle.cfg"
            "${OCS_VEHICLE_CONFIG_PATH}"
    DEPENDS
        ocs_assetc
        "${CMAKE_CURRENT_SOURCE_DIR}/vehicle/ocs_lab_gt/body.glb"
        "${CMAKE_CURRENT_SOURCE_DIR}/vehicle/ocs_lab_gt/wheel.glb"
        "${CMAKE_CURRENT_SOURCE_DIR}/vehicle/ocs_lab_gt/vehicle.cfg"
    COMMENT "Compiling Step 9.6.1 OCS Lab GT vehicle profile"
    VERBATIM
)

add_custom_target(ocs_sample_assets ALL DEPENDS
    "${OCS_SAMPLE_MODEL_PATH}"
    "${OCS_SAMPLE_TEXTURE_PATH}"
    "${OCS_DEVELOPMENT_TRACK_PATH}"
    "${OCS_DEVELOPMENT_COLLISION_PATH}"
    "${OCS_TEST_CIRCUIT_PATH}"
    "${OCS_TEST_CIRCUIT_COLLISION_PATH}"
    "${OCS_VEHICLE_BODY_PATH}"
    "${OCS_VEHICLE_WHEEL_PATH}"
    "${OCS_VEHICLE_CONFIG_PATH}"
)
