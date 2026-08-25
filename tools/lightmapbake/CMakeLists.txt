add_executable(radion_lightmapbake main.cpp)
target_link_libraries(radion_lightmapbake PRIVATE radion_engine)
target_compile_definitions(radion_lightmapbake PRIVATE
    RADION_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets"
)

if(TARGET SDL2::SDL2main)
    target_link_libraries(radion_lightmapbake PRIVATE SDL2::SDL2main)
endif()
