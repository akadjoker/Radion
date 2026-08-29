#ifndef RADION_PHYSICS_DYNAMICS_PHYSICSEVENTS_H
#define RADION_PHYSICS_DYNAMICS_PHYSICSEVENTS_H

#include "Types.h"

#include <glm/glm.hpp>

namespace Radion::Physics
{

class RigidBody;

// What happened to a pair this step. The three are what a component's
// onCollisionEnter/Stay/Exit are built from, and they fall out of the same
// per-pair bookkeeping warm starting already needs - the cache that carries
// impulses from one step to the next is exactly the record of which pairs
// were touching last step.
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
    glm::vec3 normal{0.0f};
    glm::vec3 point{0.0f};
    f32 penetration = 0.0f;
};

struct WorldRayHit
{
    RigidBody* body = nullptr;
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    f32 distance = 0.0f;
};

using ContactEventCallback = void (*)(const ContactEventInfo& info, void* userData);
// Called once per fixed step, after that step's velocities have been
// integrated into position - the same point in the loop where an action
// like a vehicle updates itself against the freshly moved world.
using PhysicsStepCallback = void (*)(f32 step, void* userData);

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_PHYSICSEVENTS_H
