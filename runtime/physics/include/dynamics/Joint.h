#ifndef RADION_PHYSICS_DYNAMICS_JOINT_H
#define RADION_PHYSICS_DYNAMICS_JOINT_H

#include "Types.h"

namespace Radion::Physics
{

class RigidBody;

class Joint
{
public:
    virtual ~Joint() = default;

    virtual RigidBody* bodyA() const = 0;
    virtual RigidBody* bodyB() const = 0;
    // A single-body joint reports the same body at both ends - a drag spring
    // towards a world target has no second body. The world's degenerate-pair
    // rejection must let those through.
    virtual bool singleBody() const
    {
        return false;
    }
    virtual void setup(f32 duration) = 0;
    virtual void warmStart() = 0;
    virtual void solveVelocity() = 0;
    virtual void solvePosition(f32 baumgarte) = 0;

    void setEnabled(bool enabled)
    {
        mEnabled = enabled;
    }
    bool enabled() const
    {
        return mEnabled;
    }

private:
    bool mEnabled = true;
};

}

#endif
