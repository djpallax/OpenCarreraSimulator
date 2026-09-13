function(ocs_enable_sanitizers target)
    if(MSVC)
        return()
    endif()

    if(OCS_ENABLE_TSAN AND (OCS_ENABLE_ASAN OR OCS_ENABLE_UBSAN))
        message(FATAL_ERROR "TSan cannot be combined with ASan/UBSan in this configuration")
    endif()

    if(OCS_ENABLE_ASAN)
        target_compile_options(${target} PRIVATE -fsanitize=address -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=address)
    endif()

    if(OCS_ENABLE_UBSAN)
        target_compile_options(${target} PRIVATE -fsanitize=undefined -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=undefined)
    endif()

    if(OCS_ENABLE_TSAN)
        target_compile_options(${target} PRIVATE -fsanitize=thread -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=thread)
    endif()
endfunction()
