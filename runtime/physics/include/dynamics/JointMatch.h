#ifndef RADION_PHYSICS_DYNAMICS_JOINTMATCH_H
#define RADION_PHYSICS_DYNAMICS_JOINTMATCH_H

#include "Component.h"
#include "dynamics/DistanceJoint.h"
#include "dynamics/FixedJoint.h"
#include "dynamics/HingeJoint.h"
#include "dynamics/Joint.h"
#include "dynamics/PistonJoint.h"
#include "dynamics/PointJoint.h"
#include "dynamics/SliderJoint.h"
#include "dynamics/UniversalJoint.h"
#include "dynamics/WheelJoint.h"

namespace Radion
{

#define RADION_JOINT_MATCH(Class, Kind)                                                            \
    template <> struct ComponentMatch<Class>                                                       \
    {                                                                                              \
        static bool test(const Component* component)                                               \
        {                                                                                          \
            return static_cast<const Physics::Joint*>(component)->kind() == Physics::JointKind::Kind; \
        }                                                                                          \
    };

RADION_JOINT_MATCH(Physics::DistanceJoint, Distance)
RADION_JOINT_MATCH(Physics::FixedJoint, Fixed)
RADION_JOINT_MATCH(Physics::HingeJoint, Hinge)
RADION_JOINT_MATCH(Physics::SliderJoint, Slider)
RADION_JOINT_MATCH(Physics::PistonJoint, Piston)
RADION_JOINT_MATCH(Physics::UniversalJoint, Universal)
RADION_JOINT_MATCH(Physics::PointJoint, Point)
RADION_JOINT_MATCH(Physics::WheelJoint, Wheel)

#undef RADION_JOINT_MATCH

} // namespace Radion

#endif // RADION_PHYSICS_DYNAMICS_JOINTMATCH_H
