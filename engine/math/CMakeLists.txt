add_library(ocs_math STATIC
    src/vec3.cpp
)

add_library(ocs::math ALIAS ocs_math)
add_library(OpenCarreraSimulator::Math ALIAS ocs_math)

target_include_directories(ocs_math PUBLIC include)
target_link_libraries(ocs_math
    PUBLIC
        ocs::project_options
        ocs::core
    PRIVATE
        ocs::project_warnings
        ocs::sanitizers
)
