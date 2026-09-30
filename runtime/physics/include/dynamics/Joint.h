#ifndef RADION_PHYSICS_DYNAMICS_JOINT_H
#define RADION_PHYSICS_DYNAMICS_JOINT_H

#include "Component.h"
#include "Math.h"

namespace Radion
{
class GameObject;
class Scene;
}

namespace Radion::Physics
{

class RigidBody;

enum class JointKind : u8
{
    Distance,
    Fixed,
    Hinge,
    Slider,
    Piston,
    Universal,
    Point,
    Wheel,
    Mouse
};

class Joint : public Radion::Component
{
public:
    static constexpr ComponentType Type = ComponentType::Joint;

    JointKind kind() const
    {
        return mKind;
    }

    virtual RigidBody* bodyA() const = 0;
    virtual RigidBody* bodyB() const = 0;
    // A single-body joint reports the same body at both ends (a drag spring has no second body); degenerate-pair rejection must let these through.
    virtual bool singleBody() const
    {
        return false;
    }
    // World-space anchor pair a debug view draws, read fresh from the current pose (not cached, so correct in the editor).
    virtual Math::vec3 anchorWorldA() const = 0;
    virtual Math::vec3 anchorWorldB() const = 0;
    // Joints with a single free direction override both; others leave the default, which nothing calls unless hasAxis() said yes.
    virtual bool hasAxis() const
    {
        return false;
    }
    virtual Math::vec3 axisWorld() const
    {
        return Math::vec3(0.0f, 1.0f, 0.0f);
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

    // Component-mode only; owner()'s RigidBody sibling is always bodyA.
    void setConnectedBody(GameObject* object);
    GameObject* connectedBody() const
    {
        return mConnectedBody;
    }

    // Resolves bodyA/bodyB, calls the concrete configure() with the owner's world position as anchor (plus the rotated authored axis), then registers
    // with the scene. A missing RigidBody logs once and leaves the joint unbuilt; rebuild() retries next step.
    virtual void rebuild() = 0;
    bool built() const
    {
        return mBuilt;
    }

protected:
    explicit Joint(JointKind kind) : Component(Type), mKind(kind)
    {
    }

    // Wakes both ends. A motor or servo given a NEW target must call this (a settled body is asleep and skipped), but only on change, or nothing ever sleeps.
    void wakeBodies();
    ~Joint() override;
    void onDestroy() override;
    // A moved-from Joint is always loose; lets a subclass's move constructor carry the private mEnabled/mConnectedBody.
    // The source's Scene registration is torn down rather than carried to the new address.
    void moveJointStateFrom(Joint& other);

    bool mBuilt = false;

private:
    friend class GameObject;
    friend class Radion::Scene;

    JointKind mKind;
    bool mEnabled = true;
    GameObject* mConnectedBody = nullptr;
    // Set by Scene::addJoint(), cleared by removeJoint(): lets the destructor leave Scene::mJoints when removeJoint() was never called.
    // Unlike mBuilt, it also tracks direct addJoint() calls (Ragdoll, tests).
    Radion::Scene* mJointScene = nullptr;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_JOINT_H
