add_executable(radion_voxel_demo
    src/main.cpp
)

target_precompile_headers(radion_voxel_demo PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)

target_link_libraries(radion_voxel_demo PRIVATE radion_engine radion_scene radion_voxel)
target_compile_definitions(radion_voxel_demo PRIVATE
    RADION_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets"
)

if(TARGET SDL2::SDL2main)
    target_link_libraries(radion_voxel_demo PRIVATE SDL2::SDL2main)
endif()

if(WIN32)
    add_custom_command(TARGET radion_voxel_demo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:radion_voxel_demo>
            $<TARGET_FILE_DIR:radion_voxel_demo>
        COMMAND_EXPAND_LISTS
        COMMENT "Copying voxel demo runtime DLLs"
    )
endif()
