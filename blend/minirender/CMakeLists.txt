add_library(radion_minirender
    src/MiniRenderer.cpp
    include/MiniRenderer.h
    src/MiniBatch.cpp
    include/MiniBatch.h
)

target_include_directories(radion_minirender PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Independent of the render pipeline (no GPU/AssetManager), but still needs
# Engine.h and Mesh.h's MeshData/MeshSkinVertex layout, and glad for GL
# directly - see blend/doc/MINI_RENDERER.md.
target_link_libraries(radion_minirender PUBLIC radion_engine)

set_target_properties(radion_minirender PROPERTIES CXX_STANDARD 17)

if(RADION_BUILD_EXAMPLES)
    add_executable(radion_minirender_test test/main.cpp)
    target_link_libraries(radion_minirender_test PRIVATE radion_minirender)
    target_compile_definitions(radion_minirender_test PRIVATE
        RADION_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets"
    )
    set_target_properties(radion_minirender_test PROPERTIES
        CXX_STANDARD 17
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
    if(TARGET SDL2::SDL2main)
        target_link_libraries(radion_minirender_test PRIVATE SDL2::SDL2main)
    endif()
endif()
