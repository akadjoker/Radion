#ifndef RADION_PHYSICS_LAB_SCENES_H
#define RADION_PHYSICS_LAB_SCENES_H

#include "Types.h"

namespace Radion
{
class Scene;
}

namespace Radion::Lab
{

// One scene per thing worth watching. Everything is built from primitives -
// no assets - so what is on screen is the physics and nothing else.
enum class LabScene : u8
{
    JointGallery, // every joint kind, side by side, moving
    RobotArm,     // three servo axes and a gripper
    Vehicle,      // four wheel joints, suspension under load
    AgentCrowd,   // agents steering around obstacles toward a goal
    Count
};

const char* sceneName(LabScene scene);

// Fills `scene` with the chosen setup: ground, light, camera and contents.
// Anything already in it is destroyed first, so switching scenes is one call.
void buildScene(Radion::Scene& scene, LabScene which);

// Per-frame driving that a scene needs beyond the simulation itself - the
// arm walking through its poses, the car steering. Nothing here reads input
// or touches the renderer.
void updateScene(Radion::Scene& scene, LabScene which, f32 elapsed);

} // namespace Radion::Lab

#endif // RADION_PHYSICS_LAB_SCENES_H
