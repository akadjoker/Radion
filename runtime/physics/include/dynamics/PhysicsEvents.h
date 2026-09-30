#ifndef RADION_PHYSICS_DYNAMICS_PHYSICSEVENTS_H
#define RADION_PHYSICS_DYNAMICS_PHYSICSEVENTS_H

#include "Types.h"

#include "Math.h"

namespace Radion::Physics
{

class RigidBody;

// What happened to a pair this step (builds onCollisionEnter/Stay/Exit); falls out of the warm-starting pair cache, which records which pairs touched last step.
enum class ContactEvent : u8
{
    Enter,
    Stay,
    Exit
};

struct ContactEventInfo
{
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    ContactEvent event = ContactEvent::Enter;
    // Empty for Exit - by then there is no contact left to describe.
    Math::vec3 normal{0.0f};
    Math::vec3 point{0.0f};
    f32 penetration = 0.0f;
};

struct WorldRayHit
{
    RigidBody* body = nullptr;
    Math::vec3 point{0.0f};
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
    f32 distance = 0.0f;
};

using ContactEventCallback = void (*)(const ContactEventInfo& info, void* userData);
// Called once per fixed step after velocities are integrated into position.
using PhysicsStepCallback = void (*)(f32 step, void* userData);

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_PHYSICSEVENTS_H
