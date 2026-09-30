#ifndef RADION_PHYSICS_SOFT_BODY_H
#define RADION_PHYSICS_SOFT_BODY_H

#include "Math.h"
#include "Types.h"
#include "collision/CollisionFilter.h"
#include "collision/Narrowphase.h"

#include <vector>

namespace Radion
{
class Scene;
}

namespace Radion::Physics
{

class RigidBody;

class SoftBody
{
public:
    struct Particle
    {
        Math::vec3 position{0.0f};

        Math::vec3 previousPosition{0.0f};
        Math::vec3 velocity{0.0f};
        // Velocity before updateVelocities() rewrote it from positions; the reference decides restitution on this.
        Math::vec3 previousVelocity{0.0f};

        f32 invMass = 0.0f;
    };

    struct DistanceConstraint
    {
        u32 a = 0;
        u32 b = 0;
        f32 restLength = 0.0f;
        f32 compliance = 0.0f;
    };

    // Two triangles sharing edge a-b (c, d opposite); holds the angle between their normals at rest. A diagonal distance only pretends to,
    // and turns near-rigid when a fold is pressed flat.
    struct DihedralBendConstraint
    {
        u32 a = 0;
        u32 b = 0;
        u32 c = 0;
        u32 d = 0;
        f32 compliance = 0.0f;
        f32 initialAngle = 0.0f;
    };

    struct LongRangeAttachment
    {
        u32 anchor = 0;
        u32 vertex = 0;
        f32 maxDistance = 0.0f;
    };

    // Read-only view of determineContactPlanes()'s per-particle result as of the last step(): resting against something, and what.
    struct Contact
    {
        bool active = false;
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
        const RigidBody* body = nullptr;
    };
    Contact contact(u32 index) const;

    void clear();

    // Particles at `positions`, sharing `totalMass` equally. Any previous
    // topology is dropped.
    void setParticles(const Math::vec3* positions, u32 count, f32 totalMass);

    // Redistributes totalMass evenly across non-pinned particles (same formula as setParticles()) without rebuilding topology or resetting the drape.
    void setTotalMass(f32 totalMass);

    // Bending mirrors the reference. Distance (default) is a distance constraint across the two vertices opposite each shared edge: approximate but
    // stable in a crumpled pile. Dihedral holds the true angle but, without self collision, traps a fully folded pile in configurations that never settle.
    enum class BendType : u8
    {
        None,
        Distance,
        Dihedral
    };

    // Structural constraints along every triangle edge, bending across each shared edge; rest lengths/angles come from current positions.
    // A negative bend compliance skips bending.
    void buildFromMesh(const u32* indices, u32 indexCount, f32 structuralCompliance,
                       f32 bendCompliance, BendType bendType = BendType::Distance);

    void addDistanceConstraint(u32 a, u32 b, f32 compliance);

    // invMass 0. Pin before building attachments: they measure from whatever
    // is pinned at that moment.
    void setPinned(u32 index, bool pinned);
    bool pinned(u32 index) const;

    // One attachment per free particle, to the pinned particle nearest it in
    // the current pose. Does nothing when nothing is pinned.
    void buildAttachments(f32 maxDistanceMultiplier = 1.0f);

    // Splits `dt` into `substeps` slices, each predicted, projected once and read back (sub-stepping, not iterating).
    void step(f32 dt, u32 substeps);

    void setGravity(const Math::vec3& gravity)
    {
        mGravity = gravity;
    }
    const Math::vec3& gravity() const
    {
        return mGravity;
    }
    // Velocity *= pow(damping, dt) per second; 1 disables.
    void setDamping(f32 damping)
    {
        mDamping = damping;
    }
    f32 damping() const
    {
        return mDamping;
    }
    // Maximum particle speed, matching Jolt's soft-body safety limit.
    void setMaxLinearVelocity(f32 velocity)
    {
        mMaxLinearVelocity = Math::max(velocity, 0.0f);
    }
    f32 maxLinearVelocity() const
    {
        return mMaxLinearVelocity;
    }
    void setWind(const Math::vec3& wind)
    {
        mWind = wind;
    }
    // Non-owning; shapes, transforms, velocities and filtering come from the engine's bodies.
    void setCollisionScene(const Radion::Scene* scene)
    {
        mCollisionScene = scene;
    }
    void setCollisionFilter(const CollisionFilter& filter)
    {
        mCollisionQuery.collision = filter;
    }
    // Excluded from every collision query: the body a splash just came out of, so it does not immediately recollide.
    void setIgnoredBody(const RigidBody* body)
    {
        mCollisionQuery.ignoredBody = body;
    }
    // Kept off the surface by this much so a sheet does not shimmer against its collider.
    void setCollisionMargin(f32 margin)
    {
        mCollisionMargin = Math::max(margin, 0.0f);
    }
    u32 particleCount() const
    {
        return static_cast<u32>(mParticles.size());
    }
    const Particle& particle(u32 index) const
    {
        return mParticles[index];
    }
    Particle& particle(u32 index)
    {
        return mParticles[index];
    }
    const std::vector<Particle>& particles() const
    {
        return mParticles;
    }
    u32 constraintCount() const
    {
        return static_cast<u32>(mConstraints.size());
    }
    u32 attachmentCount() const
    {
        return static_cast<u32>(mAttachments.size());
    }

    // Longest constraint as a fraction of rest length over the body; 1 is inextensible, 1.4 looks like rubber (what LRA holds down).
    f32 worstStretch() const;

private:
    // One plane per particle, fixed across a step's substeps (the reference determines contacts once per update, not a fresh narrowphase normal per substep).
    struct ContactPlane
    {
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
        f32 offset = 0.0f;
        const RigidBody* body = nullptr;
        f32 friction = 0.0f;
        f32 restitution = 0.0f;
        bool active = false;
    };

    void predict(f32 dt);
    void projectDistanceConstraints(f32 dt);
    void projectDihedralBendConstraints(f32 dt);
    void projectAttachments();
    void determineContactPlanes(f32 dt);
    void applyContactPlanes(f32 dt);
    void updateVelocities(f32 dt);

    std::vector<Particle> mParticles;
    std::vector<DistanceConstraint> mConstraints;
    std::vector<DihedralBendConstraint> mBendConstraints;
    std::vector<LongRangeAttachment> mAttachments;
    std::vector<ContactPlane> mContactPlanes;
    const Radion::Scene* mCollisionScene = nullptr;
    QueryFilter mCollisionQuery;
    std::vector<RigidBody*> mCollisionCandidates;
    std::vector<ContactManifold> mCollisionManifolds;
    Math::vec3 mGravity{0.0f, -9.81f, 0.0f};
    Math::vec3 mWind{0.0f};
    // Equivalent to Jolt's default linear damping of 0.1 / second:
    // exp(-0.1) retained over one second in this exponential API.
    f32 mDamping = 0.9048374f;
    f32 mMaxLinearVelocity = 500.0f;
    f32 mCollisionMargin = 0.01f;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_SOFT_BODY_H
