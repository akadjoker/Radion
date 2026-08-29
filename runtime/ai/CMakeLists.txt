# AI sources: state machines, waypoint networks + A*, grid A*, flocking and
# steering behaviors. Compiled into radion_scene rather than its own library:
# the flocking/steering agent is Radion::Agent, a Component, and its update
# step lives on Scene, so the two can no longer be linked apart - the same
# move runtime/physics/CMakeLists.txt made for RigidBody.

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

set(RADION_AI_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Behavior.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/BehaviorFactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/NavMesh.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/NavMeshBehavior.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FormationBehavior.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/GridMap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/GridPathfinder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Obstacle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AgentAvoidance.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PathfindBehavior.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PointOfInterest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SquadAI.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SquadEntity.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/State.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/StateMachine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Steering.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/VoxelMap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Waypoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/WaypointNetwork.cpp
    PARENT_SCOPE
)
set(RADION_AI_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
