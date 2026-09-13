add_executable(ocs_trackgen
    main.cpp
)

target_link_libraries(ocs_trackgen
    PRIVATE
        ocs::project_options
        ocs::project_warnings
        ocs::sanitizers
        ocs::core
        ocs::math
        ocs::assets
)
