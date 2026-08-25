add_library(radion_geometry
    src/VolumeSource.cpp
    src/VolumeCSG.cpp
    src/VolumeGrid.cpp
    src/VolumeMesher.cpp
    src/VolumeMeshSource.cpp
    src/ConvexHullComputer.cpp
    src/HullMesh.cpp
    src/VoronoiShatter.cpp
)

target_precompile_headers(radion_geometry PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)

target_include_directories(radion_geometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(radion_geometry PUBLIC radion_render)
