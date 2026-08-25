#ifndef RADION_PHYSICS_DYNAMICS_CONTACTSOLVER_H
#define RADION_PHYSICS_DYNAMICS_CONTACTSOLVER_H

#include "collision/Narrowphase.h"

namespace Radion::Physics
{

class RigidBody;
class Joint;

// One contact patch and the two bodies it holds apart.
struct Contact
{
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;
    // Which bodies these are, in the owner's own numbering. The solver never
    // reads them; they are here so a contact can be traced back to its pair
    // without keeping a second array in step with this one - which is
    // precisely what went wrong when they were separate, since only the
    // pairs the narrowphase confirms become contacts.
    u32 idA = 0;
    u32 idB = 0;
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
    void solveVelocity(Contact* contacts, u32 count);
    void solvePosition(Contact* contacts, u32 count);

    ContactSolverSettings mSettings;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_CONTACTSOLVER_H
