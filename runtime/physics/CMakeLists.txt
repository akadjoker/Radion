# Lightweight physics core shared by rigid bodies, characters and soft bodies.

add_library(radion_physics
    src/dynamics/RigidBody.cpp
    src/dynamics/Aerodynamics.cpp
    src/dynamics/ContactSolver.cpp
    src/dynamics/PhysicsWorld.cpp
    src/dynamics/RaycastVehicle.cpp
    src/dynamics/MotorcycleController.cpp
    src/dynamics/PointJoint.cpp
    src/dynamics/DistanceJoint.cpp
    src/dynamics/FixedJoint.cpp
    src/dynamics/HingeJoint.cpp
    src/dynamics/SliderJoint.cpp
    src/dynamics/PistonJoint.cpp
    src/dynamics/MouseJoint.cpp
    src/dynamics/UniversalJoint.cpp
    src/dynamics/WheelJoint.cpp
    src/softbody/SoftBody.cpp
    src/character/CharacterBody.cpp
    src/character/CharacterRigidBody.cpp
    src/character/Ragdoll.cpp
    src/collision/CollisionShape.cpp
    src/collision/Narrowphase.cpp
    src/collision/Broadphase.cpp
    src/effects/ParticleSystem.cpp
)

target_precompile_headers(radion_physics PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)

target_include_directories(radion_physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(radion_physics PUBLIC radion_render radion_geometry)
