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
    // A single-body joint reports the same body at both ends - a drag spring
    // towards a world target has no second body. The world's degenerate-pair
    // rejection must let those through.
    virtual bool singleBody() const
    {
        return false;
    }
    // World-space anchor pair a debug view draws as a line between the two
    // bodies, read fresh from the current pose - not cached, so it is correct
    // in the editor too, where nothing is stepping.
    virtual glm::vec3 anchorWorldA() const = 0;
    virtual glm::vec3 anchorWorldB() const = 0;
    // Joints with a single free direction (a hinge's rotation axis, a slider's
    // or piston's travel axis, a wheel's suspension axis, ...) override both;
    // a joint with no such axis (Distance, Point, Fixed, Mouse) leaves the
    // default, which nothing calls unless hasAxis() said yes.
    virtual bool hasAxis() const
    {
        return false;
    }
    virtual glm::vec3 axisWorld() const
    {
        return glm::vec3(0.0f, 1.0f, 0.0f);
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

    // Component-mode only (a loose joint built with a body-taking constructor
    // for a test or Ragdoll never calls this). The dragged-in second object -
    // owner()'s own RigidBody sibling is always bodyA.
    void setConnectedBody(GameObject* object);
    GameObject* connectedBody() const
    {
        return mConnectedBody;
    }

    // Resolves bodyA (owner()'s RigidBody) and bodyB (connectedBody()'s
    // RigidBody), and on success calls the concrete class's own configure()
    // with the owner's current world position as anchor (and, for a kind
    // with one, the authored axis rotated by the owner's orientation), then
    // registers with the scene. A missing RigidBody on either side logs once
    // and leaves the joint unbuilt - rebuild() runs again next step.
    virtual void rebuild() = 0;
    bool built() const
    {
        return mBuilt;
    }

protected:
    explicit Joint(JointKind kind) : Component(Type), mKind(kind)
    {
    }
    ~Joint() override;
    void onDestroy() override;
    // A moved-from Joint is always loose (Component-mode ones are only ever
    // reached through a stable pointer, never relocated) - this exists so a
    // subclass's move constructor does not silently drop mEnabled/mConnectedBody
    // just because they are private here. The source's own Scene::addJoint()
    // registration, if any, is torn down here rather than carried to the new
    // address - the destination arrives unregistered, same as a fresh Joint.
    void moveJointStateFrom(Joint& other);

    bool mBuilt = false;

private:
    friend class GameObject;
    friend class Radion::Scene;

    JointKind mKind;
    bool mEnabled = true;
    GameObject* mConnectedBody = nullptr;
    // Set by Scene::addJoint()/cleared by Scene::removeJoint() - the one
    // thing that lets this destructor find its way out of Scene::mJoints
    // when nothing ever called removeJoint() first. Distinct from mBuilt:
    // that one only tracks the Component-mode rebuild() path, this one
    // tracks direct addJoint() calls too (Ragdoll, tests).
    Radion::Scene* mJointScene = nullptr;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_JOINT_H
