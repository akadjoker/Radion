#ifndef RADION_PHYSICS_DYNAMICS_CONTACTSOLVER_H
#define RADION_PHYSICS_DYNAMICS_CONTACTSOLVER_H

#include "collision/Narrowphase.h"

#include <vector>

namespace Radion::Physics
{

class RigidBody;
class Joint;

// One contact patch and the two bodies it holds apart.
struct Contact
{
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;
    ContactManifold manifold;
    // Combined from the two materials by the caller; kept here so the solver
    // never has to know what a material is.
    f32 friction = 0.5f;
    f32 restitution = 0.0f;
};

struct ContactSolverSettings
{
    // More velocity iterations buy a stack that settles rather than sags.
    // Eight is the usual working number; the reference this follows uses ten.
    u32 velocityIterations = 8;
    // Position correction is separate and needs far fewer, because it is
    // fixing what the velocity pass could not.
    u32 positionIterations = 3;
    // Fraction of the remaining overlap removed per position iteration.
    // Pushing all of it out at once makes bodies jump apart.
    f32 baumgarte = 0.2f;
    // Overlap left alone. Without it, bodies chatter forever trying to reach
    // exactly zero, and contacts are made and lost every frame.
    f32 slop = 0.005f;
    // Approach speed below which a contact is treated as resting. A stack
    // that bounces is a stack that never sleeps.
    f32 restitutionThreshold = 1.0f;
};

 
class ContactSolver
{
public:
    void setSettings(const ContactSolverSettings& settings)
    {
        mSettings = settings;
    }
    const ContactSolverSettings& settings() const
    {
        return mSettings;
    }

 
    void solve(Contact* contacts, u32 count, Joint* const* joints, u32 jointCount, f32 duration);
    void solve(Contact* contacts, u32 count, f32 duration)
    {
        solve(contacts, count, nullptr, 0, duration);
    }

private:
    void warmStart(Contact* contacts, u32 count);
    void buildEffectiveMass(Contact* contacts, u32 count);
    void solveVelocity(Contact* contacts, u32 count);
    void solvePosition(Contact* contacts, u32 count);

    // The arm and the effective mass along each axis depend only on body
    // position and world inertia, neither of which changes while only
    // velocities are being adjusted - so this is built once per solve() call
    // and read back on every velocity iteration instead of being
    // recomputed on each one.
    struct PointMass
    {
        glm::vec3 armA{0.0f};
        glm::vec3 armB{0.0f};
        f32 tangentMass[2] = {0.0f, 0.0f};
        f32 normalMass = 0.0f;
    };
    std::vector<PointMass> mPointMass;

    ContactSolverSettings mSettings;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_CONTACTSOLVER_H
