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



# Step 8.8 Blender-authored physics test circuit. The same source geometry is
# compiled into independent visual and collision runtime resources so the two
# pipelines remain decoupled even while this laboratory asset is intentionally simple.
set(OCS_TEST_CIRCUIT_PATH "${OCS_WORLD_ASSET_OUTPUT_DIR}/test_circuit.ocsmodel"
    CACHE INTERNAL "Step 8.8 visual physics test circuit")
set(OCS_TEST_CIRCUIT_COLLISION_PATH "${OCS_WORLD_ASSET_OUTPUT_DIR}/test_circuit_collision.ocsmesh"
    CACHE INTERNAL "Step 8.8 static collision mesh")

add_custom_command(
    OUTPUT
        "${OCS_TEST_CIRCUIT_PATH}"
        "${OCS_TEST_CIRCUIT_COLLISION_PATH}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${OCS_WORLD_ASSET_OUTPUT_DIR}"
    COMMAND $<TARGET_FILE:ocs_assetc>
            "${CMAKE_CURRENT_SOURCE_DIR}/world/test_circuit.glb"
            "${OCS_TEST_CIRCUIT_PATH}"
    COMMAND $<TARGET_FILE:ocs_assetc>
            "${CMAKE_CURRENT_SOURCE_DIR}/world/test_circuit.glb"
            "${OCS_TEST_CIRCUIT_COLLISION_PATH}"
    DEPENDS
        ocs_assetc
        "${CMAKE_CURRENT_SOURCE_DIR}/world/test_circuit.glb"
    COMMENT "Compiling Step 8.8 Blender physics test circuit (visual + collision)"
    VERBATIM
)

add_custom_target(ocs_sample_assets ALL DEPENDS
    "${OCS_SAMPLE_MODEL_PATH}"
    "${OCS_SAMPLE_TEXTURE_PATH}"
    "${OCS_DEVELOPMENT_TRACK_PATH}"
    "${OCS_DEVELOPMENT_COLLISION_PATH}"
    "${OCS_TEST_CIRCUIT_PATH}"
    "${OCS_TEST_CIRCUIT_COLLISION_PATH}"
)
