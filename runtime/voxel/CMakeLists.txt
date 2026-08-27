add_library(radion_voxel
    src/VoxelBlock.cpp
    src/VoxelChunk.cpp
    src/VoxelEditHistory.cpp
    src/VoxelMesher.cpp
    src/VoxelRaycast.cpp
    src/VoxelTerrain.cpp
    src/VoxelWorld.cpp
)

target_precompile_headers(radion_voxel PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)

target_include_directories(radion_voxel PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# The voxel data model stays independent of Scene and the editor.  The mesher
# only produces CPU-side MeshData; upload remains the renderer's job.
target_link_libraries(radion_voxel PUBLIC radion_render)
