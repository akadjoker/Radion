add_executable(radion_physics_lab
    src/main.cpp
    src/LabScenes.cpp
)

target_precompile_headers(radion_physics_lab PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)

target_link_libraries(radion_physics_lab PRIVATE radion_engine radion_scene)
target_compile_definitions(radion_physics_lab PRIVATE
    RADION_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets"
)

if(TARGET SDL2::SDL2main)
    target_link_libraries(radion_physics_lab PRIVATE SDL2::SDL2main)
endif()

if(WIN32)
    add_custom_command(TARGET radion_physics_lab POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:radion_physics_lab>
            $<TARGET_FILE_DIR:radion_physics_lab>
        COMMAND_EXPAND_LISTS
        COMMENT "Copying physics lab runtime DLLs"
    )
endif()
