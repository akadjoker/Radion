#ifndef RADION_PHYSICS_DYNAMICS_CONTACTSOLVER_H
#define RADION_PHYSICS_DYNAMICS_CONTACTSOLVER_H

#include "collision/Narrowphase.h"

#include <vector>

namespace Radion::Physics
{

class RigidBody;
class Joint;

struct Contact
{
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;
    ContactManifold manifold;
    // Combined from the two materials by the caller, so the solver never knows what a material is.
    f32 friction = 0.5f;
    f32 restitution = 0.0f;
};

struct ContactSolverSettings
{
    // More velocity iterations settle a stack rather than sag; eight is usual, the reference uses ten.
    u32 velocityIterations = 8;
    // Position correction needs far fewer: it fixes what the velocity pass could not.
    u32 positionIterations = 3;
    // Fraction of remaining overlap removed per position iteration; all at once makes bodies jump apart.
    f32 baumgarte = 0.2f;
    // Overlap left alone; chasing exactly zero makes contacts chatter.
    f32 slop = 0.005f;
    // Approach speed below which a contact is resting; a bouncing stack never sleeps.
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

    // Arm and effective mass per axis depend only on position and world inertia, unchanged while only velocities adjust:
    // built once per solve(), read on every velocity iteration.
    struct PointMass
    {
        Math::vec3 armA{0.0f};
        Math::vec3 armB{0.0f};
        f32 tangentMass[2] = {0.0f, 0.0f};
        f32 normalMass = 0.0f;
    };
    std::vector<PointMass> mPointMass;

    ContactSolverSettings mSettings;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_CONTACTSOLVER_H
