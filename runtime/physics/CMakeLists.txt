# Lightweight physics core shared by rigid bodies, characters and soft bodies.
# Compiled into radion_scene rather than its own library: RigidBody is a
# Component and its step lives on Scene, so the two can no longer be linked
# apart.

set(RADION_PHYSICS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/RigidBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/Aerodynamics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/ContactSolver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/RaycastVehicle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/MotorcycleController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/PointJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/DistanceJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/FixedJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/HingeJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/SliderJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/PistonJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/MouseJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/UniversalJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/WheelJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/softbody/SoftBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/character/CharacterBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/character/CharacterRigidBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/character/Ragdoll.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collision/CollisionShape.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collision/Narrowphase.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collision/Broadphase.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/effects/ParticleSystem.cpp
    PARENT_SCOPE
)
set(RADION_PHYSICS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
