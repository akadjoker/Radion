#ifndef RADION_PHYSICS_CHARACTER_RIGID_BODY_H
#define RADION_PHYSICS_CHARACTER_RIGID_BODY_H

#include "Math.h"
#include "Types.h"
#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"
#include "dynamics/RigidBody.h"

#include <vector>

namespace Radion::Physics
{

class PhysicsWorld;

class CharacterRigidBody
{
public:
    enum class GroundState : u8
    {
        OnGround,
        OnSteepGround,
        NotSupported,
        InAir
    };

    static constexpr u32 kInvalidBodyId = 0xFFFFFFFFu;

    CharacterRigidBody();

    void setShape(f32 radius, f32 height);
    f32 radius() const
    {
        return mRadius;
    }
    f32 height() const
    {
        return mHeight;
    }

    void setMass(f32 mass);
    f32 mass() const
    {
        return mMass;
    }
    void setFriction(f32 friction);
    f32 friction() const
    {
        return mFriction;
    }
    void setMaxSlopeAngle(f32 degrees);
    f32 maxSlopeAngle() const
    {
        return mMaxSlopeAngleDegrees;
    }
    void setSupportingVolume(const Plane& plane)
    {
        mSupportingVolume = plane;
    }
    const Plane& supportingVolume() const
    {
        return mSupportingVolume;
    }
    void setUp(const glm::vec3& up)
    {
        mUp = glm::normalize(up);
    }
    const glm::vec3& up() const
    {
        return mUp;
    }
    void setLayer(u32 layer);
    u32 layer() const
    {
        return mLayer;
    }
    void setMask(u32 mask);
    u32 mask() const
    {
        return mMask;
    }

    void addToWorld(PhysicsWorld& world, const glm::vec3& position);
    void removeFromWorld();
    u32 bodyId() const
    {
        return mBodyId;
    }

    void setLinearVelocity(const glm::vec3& velocity);
    const glm::vec3& linearVelocity() const;
    void addLinearVelocity(const glm::vec3& velocity);
    void addImpulse(const glm::vec3& impulse);

    const glm::vec3& position() const;
    glm::mat4 transform() const;

    void postSimulation(f32 maxSeparationDistance);

    GroundState groundState() const
    {
        return mGroundState;
    }
    const glm::vec3& groundNormal() const
    {
        return mGroundNormal;
    }
    const glm::vec3& groundPosition() const
    {
        return mGroundPosition;
    }
    const glm::vec3& groundVelocity() const
    {
        return mGroundVelocity;
    }
    u32 groundBodyId() const
    {
        return mGroundBodyId;
    }
    bool isSupported() const
    {
        return mGroundState == GroundState::OnGround || mGroundState == GroundState::OnSteepGround;
    }

private:
    RigidBody mBody;
    CapsuleShape mShape{0.4f, 0.6f};

    PhysicsWorld* mWorld = nullptr;
    u32 mBodyId = kInvalidBodyId;

    f32 mRadius = 0.4f;
    f32 mHeight = 1.2f;
    f32 mMass = 80.0f;
    f32 mFriction = 0.2f;
    u32 mLayer = 1;
    u32 mMask = 0xFFFFFFFFu;

    glm::vec3 mUp{0.0f, 1.0f, 0.0f};
    Plane mSupportingVolume{glm::vec3(0.0f, 1.0f, 0.0f), -1.0e10f};
    f32 mMaxSlopeAngleDegrees = 50.0f;
    f32 mMaxSlopeAngleCosine = 0.64278759f;

    GroundState mGroundState = GroundState::InAir;
    u32 mGroundBodyId = kInvalidBodyId;
    glm::vec3 mGroundNormal{0.0f};
    glm::vec3 mGroundPosition{0.0f};
    glm::vec3 mGroundVelocity{0.0f};
    std::vector<u32> mCandidates;
    std::vector<ContactManifold> mManifolds;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_CHARACTER_RIGID_BODY_H
