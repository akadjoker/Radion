#include "PCH.h"

#include "dynamics/Joint.h"

#include "GameObject.h"
#include "Scene.h"

namespace Radion::Physics
{

Joint::~Joint()
{
    // Component-mode joints already left mJoints through onDestroy() by the
    // time their destructor runs (GameObject::removeComponent() calls
    // detached() before delete). This is what a loose joint - Scene::addJoint()
    // called directly, no GameObject at all: Ragdoll's own parts, every joint
    // test - relies on instead.
    if (mJointScene)
        mJointScene->removeJoint(this);
}

void Joint::setConnectedBody(GameObject* object)
{
    mConnectedBody = object;
    mBuilt = false;
}

void Joint::onDestroy()
{
    if (mJointScene)
        mJointScene->removeJoint(this);
    mBuilt = false;
}

void Joint::moveJointStateFrom(Joint& other)
{
    mEnabled = other.mEnabled;
    mConnectedBody = other.mConnectedBody;
    mBuilt = other.mBuilt;
    other.mConnectedBody = nullptr;
    other.mBuilt = false;

    // The scene stores joints by address, so a registered one cannot follow
    // a move: its entry would still name the old object. The source leaves
    // the scene and the destination arrives unregistered, to be added by
    // whoever now owns it - same rule as RigidBody::moveFrom().
    if (other.mJointScene)
        other.mJointScene->removeJoint(&other);
}

void Joint::wakeBodies()
{
    if (RigidBody* a = bodyA())
        a->setAwake(true);
    if (RigidBody* b = bodyB())
        b->setAwake(true);
}

} // namespace Radion::Physics
