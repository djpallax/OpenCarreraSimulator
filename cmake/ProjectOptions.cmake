function(ocs_create_project_targets)
    add_library(ocs_project_options INTERFACE)
    add_library(ocs::project_options ALIAS ocs_project_options)
    target_compile_features(ocs_project_options INTERFACE cxx_std_23)

    add_library(ocs_project_warnings INTERFACE)
    add_library(ocs::project_warnings ALIAS ocs_project_warnings)

    if(MSVC)
        target_compile_options(ocs_project_warnings INTERFACE /W4 /permissive-)
        if(OCS_WARNINGS_AS_ERRORS)
            target_compile_options(ocs_project_warnings INTERFACE /WX)
        endif()
    else()
        target_compile_options(ocs_project_warnings INTERFACE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow
            -Wnon-virtual-dtor
            -Wold-style-cast
        )
        if(OCS_WARNINGS_AS_ERRORS)
            target_compile_options(ocs_project_warnings INTERFACE -Werror)
        endif()
    endif()

    add_library(ocs_sanitizers INTERFACE)
    add_library(ocs::sanitizers ALIAS ocs_sanitizers)

    if(NOT MSVC)
        if(OCS_ENABLE_TSAN AND (OCS_ENABLE_ASAN OR OCS_ENABLE_UBSAN))
            message(FATAL_ERROR "TSan cannot be combined with ASan/UBSan in this configuration")
        endif()

        if(OCS_ENABLE_ASAN)
            target_compile_options(ocs_sanitizers INTERFACE -fsanitize=address -fno-omit-frame-pointer)
            target_link_options(ocs_sanitizers INTERFACE -fsanitize=address)
        endif()

        if(OCS_ENABLE_UBSAN)
            target_compile_options(ocs_sanitizers INTERFACE -fsanitize=undefined -fno-omit-frame-pointer)
            target_link_options(ocs_sanitizers INTERFACE -fsanitize=undefined)
        endif()

        if(OCS_ENABLE_TSAN)
            target_compile_options(ocs_sanitizers INTERFACE -fsanitize=thread -fno-omit-frame-pointer)
            target_link_options(ocs_sanitizers INTERFACE -fsanitize=thread)
        endif()
    endif()
endfunction()
