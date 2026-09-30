#ifndef RADION_PHYSICS_LAB_SCENES_H
#define RADION_PHYSICS_LAB_SCENES_H

#include "Types.h"

namespace Radion
{
class Scene;
}

namespace Radion::Lab
{

enum class LabScene : u8
{
    JointGallery,
    RobotArm,
    Vehicle,
    AgentCrowd,
    Count
};

const char* sceneName(LabScene scene);

// Anything already in the scene is destroyed first.
void buildScene(Radion::Scene& scene, LabScene which);

void updateScene(Radion::Scene& scene, LabScene which, f32 elapsed);

} // namespace Radion::Lab

#endif // RADION_PHYSICS_LAB_SCENES_H
