add_executable(radion_bistro_demo
    src/main.cpp
)

target_precompile_headers(radion_bistro_demo PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)

target_link_libraries(radion_bistro_demo PRIVATE radion_engine radion_scene)
target_compile_definitions(radion_bistro_demo PRIVATE
    RADION_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets"
)

if(TARGET SDL2::SDL2main)
    target_link_libraries(radion_bistro_demo PRIVATE SDL2::SDL2main)
endif()

if(WIN32)
    add_custom_command(TARGET radion_bistro_demo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:radion_bistro_demo>
            $<TARGET_FILE_DIR:radion_bistro_demo>
        COMMAND_EXPAND_LISTS
        COMMENT "Copying bistro demo runtime DLLs"
    )
endif()
