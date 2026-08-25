# AI library: state machines, waypoint networks + A*, grid A*, flocking and
# steering behaviors. Decoupled from the engine: radion_ai depends only on
# radion_core, which provides glm and the Radion:: base types. It does not
# touch render/gpu/scene/engine.

set(RECAST_ROOT ${CMAKE_SOURCE_DIR}/vendor/recastnavigation)

# Recast (navmesh generation) + Detour (queries on it). Source list mirrors
# the one in BuGL/BuRecast; DetourCrowd/DetourTileCache are left out until
# something here actually needs crowds or streamed tiles.
add_library(radion_recast STATIC
    ${RECAST_ROOT}/Recast/Source/Recast.cpp
    ${RECAST_ROOT}/Recast/Source/RecastAlloc.cpp
    ${RECAST_ROOT}/Recast/Source/RecastArea.cpp
    ${RECAST_ROOT}/Recast/Source/RecastAssert.cpp
    ${RECAST_ROOT}/Recast/Source/RecastContour.cpp
    ${RECAST_ROOT}/Recast/Source/RecastFilter.cpp
    ${RECAST_ROOT}/Recast/Source/RecastLayers.cpp
    ${RECAST_ROOT}/Recast/Source/RecastMesh.cpp
    ${RECAST_ROOT}/Recast/Source/RecastMeshDetail.cpp
    ${RECAST_ROOT}/Recast/Source/RecastRasterization.cpp
    ${RECAST_ROOT}/Recast/Source/RecastRegion.cpp
    ${RECAST_ROOT}/Detour/Source/DetourAlloc.cpp
    ${RECAST_ROOT}/Detour/Source/DetourAssert.cpp
    ${RECAST_ROOT}/Detour/Source/DetourCommon.cpp
    ${RECAST_ROOT}/Detour/Source/DetourNavMesh.cpp
    ${RECAST_ROOT}/Detour/Source/DetourNavMeshBuilder.cpp
    ${RECAST_ROOT}/Detour/Source/DetourNavMeshQuery.cpp
    ${RECAST_ROOT}/Detour/Source/DetourNode.cpp
)
target_include_directories(radion_recast SYSTEM PUBLIC
    ${RECAST_ROOT}/Recast/Include
    ${RECAST_ROOT}/Detour/Include
)

add_library(radion_ai
    src/Behavior.cpp
    src/Entity.cpp
    src/NavMesh.cpp
    src/NavMeshBehavior.cpp
    src/FormationBehavior.cpp
    src/GridMap.cpp
    src/GridPathfinder.cpp
    src/Group.cpp
    src/Obstacle.cpp
    src/PathfindBehavior.cpp
    src/PointOfInterest.cpp
    src/SquadAI.cpp
    src/SquadEntity.cpp
    src/State.cpp
    src/StateMachine.cpp
    src/Steering.cpp
    src/VoxelMap.cpp
    src/Waypoint.cpp
    src/WaypointNetwork.cpp
    src/World.cpp
)

target_precompile_headers(radion_ai PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)

target_include_directories(radion_ai PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(radion_ai PUBLIC radion_core PRIVATE radion_recast)
