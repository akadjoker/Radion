add_library(radion_options INTERFACE)

if(WIN32)
    target_compile_definitions(radion_options INTERFACE
        RADION_PLATFORM_WINDOWS=1
        WIN32_LEAN_AND_MEAN
        NOMINMAX
    )
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_definitions(radion_options INTERFACE RADION_PLATFORM_LINUX=1)
else()
    message(FATAL_ERROR "Radion currently supports Windows and Linux only")
endif()

target_compile_definitions(radion_options INTERFACE
    "$<$<CONFIG:Debug>:RADION_DEBUG=1>"
)

if(MSVC)
    target_compile_options(radion_options INTERFACE
        /W4
        /permissive-
        /Zc:__cplusplus
    )
else()
    target_compile_options(radion_options INTERFACE
        -Wall
        -Wextra
        -Wpedantic
    )
endif()

if(RADION_ENABLE_SANITIZERS)
    if(MSVC)
        target_compile_options(radion_options INTERFACE
            "$<$<CONFIG:Debug>:/fsanitize=address>"
            "$<$<CONFIG:Debug>:/Zi>"
        )
        target_link_options(radion_options INTERFACE
            "$<$<CONFIG:Debug>:/INCREMENTAL:NO>"
        )
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(radion_options INTERFACE
            "$<$<CONFIG:Debug>:-fsanitize=address,undefined>"
            "$<$<CONFIG:Debug>:-fno-omit-frame-pointer>"
        )
        target_link_options(radion_options INTERFACE
            "$<$<CONFIG:Debug>:-fsanitize=address,undefined>"
        )
    else()
        message(WARNING "Sanitizers are not configured for ${CMAKE_CXX_COMPILER_ID}")
    endif()
endif()
