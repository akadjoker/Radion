set(IMGUI_SOURCES
    ${PROJECT_SOURCE_DIR}/vendor/imgui/imgui.cpp
    ${PROJECT_SOURCE_DIR}/vendor/imgui/imgui_draw.cpp
    ${PROJECT_SOURCE_DIR}/vendor/imgui/imgui_tables.cpp
    ${PROJECT_SOURCE_DIR}/vendor/imgui/imgui_widgets.cpp
    ${PROJECT_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_sdl2.cpp
    ${PROJECT_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_opengl3.cpp
)

add_library(radion_render
    src/Batch.cpp
    src/DebugDraw3D.cpp
    src/ScreenDraw.cpp
    src/ScreenDrawPass.cpp
    src/DepthPass.cpp
    src/MaterialManager.cpp
    src/MeshLoader.cpp
    src/MeshClipper.cpp
    src/ObjImporter.cpp
    src/OgreMeshImporter.cpp
    src/RadionMeshImporter.cpp
    src/B3DImporter.cpp
    src/C3DSImporter.cpp
    src/MS3DImporter.cpp
    src/GltfImporter.cpp
    src/Skeleton.cpp
    src/AssetManager.cpp
    src/AssetMesh.cpp
    src/LightmapUnwrapper.cpp
    src/LightmapBakePass.cpp
    ${PROJECT_SOURCE_DIR}/vendor/xatlas/xatlas.cpp
    ${PROJECT_SOURCE_DIR}/vendor/meshoptimizer/allocator.cpp
    ${PROJECT_SOURCE_DIR}/vendor/meshoptimizer/indexgenerator.cpp
    ${PROJECT_SOURCE_DIR}/vendor/meshoptimizer/vcacheoptimizer.cpp
    ${PROJECT_SOURCE_DIR}/vendor/meshoptimizer/overdrawoptimizer.cpp
    ${PROJECT_SOURCE_DIR}/vendor/meshoptimizer/vfetchoptimizer.cpp
    ${PROJECT_SOURCE_DIR}/vendor/meshoptimizer/simplifier.cpp
    src/AssetTree.cpp
    src/AssetTexture.cpp
    src/TextureDecode.cpp
    src/AsyncTextureLoader.cpp
    src/DDSImage.cpp
    src/RenderList.cpp
    src/Shadows.cpp
    src/ShadowPass.cpp
    src/ShadowDebugView.cpp
    src/Lighting.cpp
    src/Decals.cpp
    src/VolumetricPass.cpp
    src/EnvironmentProbe.cpp
    src/LensFlarePass.cpp
    src/GrassRender.cpp
    src/HairRender.cpp
    src/OceanRender.cpp
    src/TreeRender.cpp
    src/ParticleRender.cpp
    src/ParticlePass.cpp
    src/TrailRender.cpp
    src/ForwardPass.cpp
    src/WaterPass.cpp
    src/OffscreenTarget.cpp
    src/MeshPreview.cpp
    src/PostProcess.cpp
    src/Sky.cpp
    src/Renderer.cpp
    src/ImGuiLayer.cpp
    ${IMGUI_SOURCES}
)

if(RADION_ENABLE_FBX_LOADER)
    target_sources(radion_render PRIVATE
        src/FbxImporter.cpp
        src/ofbx.cpp
    )
    target_compile_definitions(radion_render PUBLIC RADION_ENABLE_FBX_LOADER=1)
    set_source_files_properties(src/ofbx.cpp PROPERTIES
        COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-unused-variable;-Wno-sign-compare")
endif()

 
set_source_files_properties(${PROJECT_SOURCE_DIR}/vendor/xatlas/xatlas.cpp
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-but-set-variable;-Wno-implicit-const-int-float-conversion")

target_precompile_headers(radion_render PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_include_directories(radion_render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(radion_render SYSTEM PUBLIC
    ${PROJECT_SOURCE_DIR}/vendor
    ${PROJECT_SOURCE_DIR}/vendor/xatlas
    ${PROJECT_SOURCE_DIR}/vendor/imgui
    ${PROJECT_SOURCE_DIR}/vendor/imgui/backends
)
target_link_libraries(radion_render PUBLIC radion_core radion_gpu OpenGL::GL)
