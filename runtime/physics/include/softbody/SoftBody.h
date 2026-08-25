#ifndef RADION_PHYSICS_SOFT_BODY_H
#define RADION_PHYSICS_SOFT_BODY_H

#include "Math.h"
#include "Types.h"
#include "collision/CollisionFilter.h"

#include <vector>

namespace Radion::Physics
{

class PhysicsWorld;
 
class SoftBody
{
public:
    struct Particle
    {
        glm::vec3 position{0.0f};

        glm::vec3 previousPosition{0.0f};
        glm::vec3 velocity{0.0f};
        // Velocity as it stood before updateVelocities() rewrote it from the
        // positions - the reference decides restitution on this one, not on
        // the freshly derived velocity.
        glm::vec3 previousVelocity{0.0f};

        f32 invMass = 0.0f;
    };

    struct DistanceConstraint
    {
        u32 a = 0;
        u32 b = 0;
        f32 restLength = 0.0f;
        f32 compliance = 0.0f;
    };

    // Two triangles sharing the edge a-b, with c and d the opposite
    // vertices; the constraint holds the angle between their normals at its
    // rest value. A distance across the diagonal only pretends to do this,
    // and turns near-rigid when a fold is pressed flat - the exact
    // configuration a crumpled sheet is full of.
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

    // A read-only view of determineContactPlanes()'s own per-particle
    // result - whether a particle is resting against something as of the
    // last step(), and against what. Lets a caller (a blood splash marking
    // where it landed, say) react to a particle's own contact without
    // re-deriving it from position and velocity itself.
    struct Contact
    {
        bool active = false;
        glm::vec3 normal{0.0f, 1.0f, 0.0f};
        u32 bodyId = 0xFFFFFFFFu;
    };
    Contact contact(u32 index) const;

    void clear();

    // Particles at `positions`, sharing `totalMass` equally. Any previous
    // topology is dropped.
    void setParticles(const glm::vec3* positions, u32 count, f32 totalMass);

    // Redistributes `totalMass` evenly across every particle that is not
    // currently pinned (pinned() is invMass 0 - see setPinned()), leaving
    // pinned particles, positions, velocities and every constraint alone.
    // Same formula setParticles() uses, but live: a caller retuning "how
    // heavy" a draped sheet is does not have to rebuild its topology, or
    // reset the drape, to do it.
    void setTotalMass(f32 totalMass);

    // How bending resistance is built, mirroring the reference's bend types.
    // Distance is the default there too: a distance constraint between the
    // two vertices opposite each shared edge - approximate, but stable in a
    // crumpled pile. Dihedral holds the true angle between the triangles and
    // folds more naturally, but without self collision a fully folded pile
    // traps it in degenerate configurations that never settle.
    enum class BendType : u8
    {
        None,
        Distance,
        Dihedral
    };

    // Structural constraints along every triangle edge, and a bending
    // constraint across each edge two triangles share. Rest lengths and rest
    // angles come from the positions as they stand, so the mesh is its own
    // rest pose. A negative bend compliance skips bending entirely.
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

    // Splits `dt` into `substeps` slices, each predicted, projected once and
    // read back. Sub-stepping, not iterating: see step()'s own note.
    void step(f32 dt, u32 substeps);

    void setGravity(const glm::vec3& gravity)
    {
        mGravity = gravity;
    }
    const glm::vec3& gravity() const
    {
        return mGravity;
    }
    // Per-second exponential velocity damping: velocity *= pow(damping, dt).
    // 1 disables it.
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
        mMaxLinearVelocity = glm::max(velocity, 0.0f);
    }
    f32 maxLinearVelocity() const
    {
        return mMaxLinearVelocity;
    }
    void setWind(const glm::vec3& wind)
    {
        mWind = wind;
    }
    // Non-owning world used for particle contacts. Shapes, transforms,
    // velocities and collision filtering come from the engine's bodies.
    void setCollisionWorld(const PhysicsWorld* world)
    {
        mCollisionWorld = world;
    }
    void setCollisionFilter(const CollisionFilter& filter)
    {
        mCollisionQuery.collision = filter;
    }
    // Excluded from every collision query this body makes - the body a
    // splash of particles just came out of, say, so it does not immediately
    // recollide with the very thing that spawned it.
    void setIgnoredBody(u32 bodyId)
    {
        mCollisionQuery.ignoredBody = bodyId;
    }
    // Kept off the surface by this much, so a sheet does not shimmer with its
    // own thickness against the collider it rests on.
    void setCollisionMargin(f32 margin)
    {
        mCollisionMargin = glm::max(margin, 0.0f);
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

    // Longest constraint as a fraction of its rest length, over the whole
    // body. 1 is inextensible; a sheet that reads 1.4 is stretched 40% and
    // looks like rubber, which is the number LRA exists to hold down.
    f32 worstStretch() const;

private:
    // One collision plane per particle, held fixed across every substep of a
    // step - the reference determines contacts once per update and lets the
    // substeps project against a stable plane, instead of re-running the
    // narrowphase with a fresh normal each substep.
    struct ContactPlane
    {
        glm::vec3 normal{0.0f, 1.0f, 0.0f};
        f32 offset = 0.0f;
        u32 bodyId = 0;
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
    const PhysicsWorld* mCollisionWorld = nullptr;
    QueryFilter mCollisionQuery;
    std::vector<u32> mCollisionCandidates;
    glm::vec3 mGravity{0.0f, -9.81f, 0.0f};
    glm::vec3 mWind{0.0f};
    // Equivalent to Jolt's default linear damping of 0.1 / second:
    // exp(-0.1) retained over one second in this exponential API.
    f32 mDamping = 0.9048374f;
    f32 mMaxLinearVelocity = 500.0f;
    f32 mCollisionMargin = 0.01f;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_SOFT_BODY_H
