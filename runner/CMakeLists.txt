add_executable(radion_runner
    src/main.cpp
)

target_link_libraries(radion_runner PRIVATE radion_engine radion_scene)
target_compile_definitions(radion_runner PRIVATE
    RADION_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets"
)

if(TARGET SDL2::SDL2main)
    target_link_libraries(radion_runner PRIVATE SDL2::SDL2main)
endif()

if(WIN32)
    add_custom_command(TARGET radion_runner POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:radion_runner>
            $<TARGET_FILE_DIR:radion_runner>
        COMMAND_EXPAND_LISTS
        COMMENT "Copying runner runtime DLLs"
    )
endif()
