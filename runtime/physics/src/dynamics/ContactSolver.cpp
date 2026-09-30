#include "PCH.h"

#include "dynamics/ContactSolver.h"

#include "dynamics/Joint.h"
#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

namespace
{
// Velocity produced by a unit impulse along `direction` at these points (linear plus inertia-tensor torque part);
// dividing by it converts a wanted velocity change into an impulse.
f32 effectiveMass(const RigidBody& a, const RigidBody& b, const Math::vec3& armA,
                  const Math::vec3& armB, const Math::vec3& direction)
{
    const Math::vec3 angularA = Math::cross(armA, direction);
    const Math::vec3 angularB = Math::cross(armB, direction);
    return a.inverseMass() + b.inverseMass() +
           Math::dot(direction, Math::cross(a.inverseInertiaTensorWorld() * angularA, armA)) +
           Math::dot(direction, Math::cross(b.inverseInertiaTensorWorld() * angularB, armB));
}

Math::vec3 relativeVelocity(const RigidBody& a, const RigidBody& b, const Math::vec3& armA,
                           const Math::vec3& armB)
{
    return (b.velocity() + Math::cross(b.angularVelocity(), armB)) -
           (a.velocity() + Math::cross(a.angularVelocity(), armA));
}

// The normal points A to B, so B takes the impulse and A its opposite.
// NOT RigidBody::applyImpulseAtPoint: it wakes the body, and the solver touches every resting contact each step, so a settled stack
// never slept. Waking belongs to a NEW contact (Scene's pair cache).
void applyOne(RigidBody& body, const Math::vec3& impulse, const Math::vec3& point)
{
    if (!body.isDynamic())
        return;
    body.setVelocity(body.velocity() + impulse * body.inverseMass());
    body.setAngularVelocity(body.angularVelocity() +
                            body.inverseInertiaTensorWorld() *
                                Math::cross(point - body.position(), impulse));
}

void applyPair(RigidBody& a, RigidBody& b, const Math::vec3& impulse, const Math::vec3& point)
{
    applyOne(a, -impulse, point);
    applyOne(b, impulse, point);
}
} // namespace

void ContactSolver::warmStart(Contact* contacts, u32 count)
{
    for (u32 c = 0; c < count; ++c)
    {
        Contact& contact = contacts[c];
        if (!contact.a || !contact.b)
            continue;
        ContactManifold& manifold = contact.manifold;
        for (u32 i = 0; i < manifold.count; ++i)
        {
            ContactPoint& point = manifold.points[i];
            // Last step's answer applied before the first iteration; otherwise a tall stack sinks by what the iterations cannot recover.
            const Math::vec3 impulse = manifold.normal * point.normalImpulse +
                                      manifold.tangent[0] * point.tangentImpulse[0] +
                                      manifold.tangent[1] * point.tangentImpulse[1];
            applyPair(*contact.a, *contact.b, impulse, point.position);
        }
    }
}

void ContactSolver::buildEffectiveMass(Contact* contacts, u32 count)
{
    mPointMass.resize(static_cast<usize>(count) * ContactManifold::MaxPoints);
    for (u32 c = 0; c < count; ++c)
    {
        Contact& contact = contacts[c];
        if (!contact.a || !contact.b)
            continue;
        RigidBody& a = *contact.a;
        RigidBody& b = *contact.b;
        const ContactManifold& manifold = contact.manifold;
        PointMass* cache = &mPointMass[static_cast<usize>(c) * ContactManifold::MaxPoints];

        for (u32 i = 0; i < manifold.count; ++i)
        {
            const ContactPoint& point = manifold.points[i];
            PointMass& mass = cache[i];
            mass.armA = point.position - a.position();
            mass.armB = point.position - b.position();
            mass.tangentMass[0] = effectiveMass(a, b, mass.armA, mass.armB, manifold.tangent[0]);
            mass.tangentMass[1] = effectiveMass(a, b, mass.armA, mass.armB, manifold.tangent[1]);
            mass.normalMass = effectiveMass(a, b, mass.armA, mass.armB, manifold.normal);
        }
    }
}

void ContactSolver::solveVelocity(Contact* contacts, u32 count)
{
    for (u32 c = 0; c < count; ++c)
    {
        Contact& contact = contacts[c];
        if (!contact.a || !contact.b)
            continue;
        RigidBody& a = *contact.a;
        RigidBody& b = *contact.b;
        ContactManifold& manifold = contact.manifold;
        const PointMass* cache = &mPointMass[static_cast<usize>(c) * ContactManifold::MaxPoints];

        for (u32 i = 0; i < manifold.count; ++i)
        {
            ContactPoint& point = manifold.points[i];
            const Math::vec3& armA = cache[i].armA;
            const Math::vec3& armB = cache[i].armB;

            // Friction first, against the normal impulse the last iteration settled on; using this iteration's would let the two chase each other.
            const f32 maxFriction = contact.friction * point.normalImpulse;
            for (u32 t = 0; t < 2; ++t)
            {
                const Math::vec3& tangent = manifold.tangent[t];
                const f32 mass = cache[i].tangentMass[t];
                if (mass <= 0.0f)
                    continue;
                const f32 speed = Math::dot(relativeVelocity(a, b, armA, armB), tangent);
                const f32 wanted = -speed / mass;
                const f32 previous = point.tangentImpulse[t];
                const f32 total = Math::clamp(previous + wanted, -maxFriction, maxFriction);
                point.tangentImpulse[t] = total;
                applyPair(a, b, tangent * (total - previous), point.position);
            }

            const f32 mass = cache[i].normalMass;
            if (mass <= 0.0f)
                continue;
            const f32 separation =
                Math::dot(relativeVelocity(a, b, armA, armB), manifold.normal);
            // Bias is the separation speed a bounce should end at; zero for a resting contact.
            const f32 wanted = -(separation - point.velocityBias) / mass;

            // Clamp the ACCUMULATED impulse, not each increment: a later contact may need to take back part of an earlier iteration's.
            const f32 previous = point.normalImpulse;
            const f32 total = Math::max(previous + wanted, 0.0f);
            point.normalImpulse = total;
            applyPair(a, b, manifold.normal * (total - previous), point.position);
        }
    }
}

void ContactSolver::solvePosition(Contact* contacts, u32 count)
{
    for (u32 c = 0; c < count; ++c)
    {
        Contact& contact = contacts[c];
        if (!contact.a || !contact.b)
            continue;
        RigidBody& a = *contact.a;
        RigidBody& b = *contact.b;
        ContactManifold& manifold = contact.manifold;
        for (u32 i = 0; i < manifold.count; ++i)
        {
            ContactPoint& point = manifold.points[i];
            // Overlap up to the slop is left alone: chasing exactly zero makes resting bodies make and lose contact every frame.
            const f32 excess = point.penetration - mSettings.slop;
            if (excess <= 0.0f)
                continue;

            const Math::vec3 armA = point.position - a.position();
            const Math::vec3 armB = point.position - b.position();
            const f32 mass = effectiveMass(a, b, armA, armB, manifold.normal);
            if (mass <= 0.0f)
                continue;
            const Math::vec3 impulse =
                manifold.normal * (excess * mSettings.baumgarte / mass);

            // Moved directly, not via a bias impulse: a bias adds velocity the bodies keep, so a resting stack gains energy and drifts.
            a.applyPositionImpulseAtPoint(-impulse, point.position);
            b.applyPositionImpulseAtPoint(impulse, point.position);
            point.penetration -= excess * mSettings.baumgarte;
        }
    }
}

void ContactSolver::solve(Contact* contacts, u32 count, Joint* const* joints, u32 jointCount,
                          f32 duration)
{
    if (duration <= 0.0f || !std::isfinite(duration))
        return;

    // Tangents rebuilt from the normal: a carried-over manifold's normal may have turned.
    // Waking is not done here: a resting stack has a contact every step, so nothing would sleep; Scene wakes on a NEW contact.
    for (u32 c = 0; c < count; ++c)
    {
        Contact& contact = contacts[c];
        if (!contact.a || !contact.b)
            continue;
        ContactManifold& manifold = contact.manifold;
        manifold.buildTangents();

        // Restitution is decided HERE from the pre-impulse approach speed; after the velocity iterations the approach is zero and the bounce vanishes.
        for (u32 i = 0; i < manifold.count; ++i)
        {
            ContactPoint& point = manifold.points[i];
            point.velocityBias = 0.0f;
            if (contact.restitution <= 0.0f)
                continue;
            const Math::vec3 armA = point.position - contact.a->position();
            const Math::vec3 armB = point.position - contact.b->position();
            const f32 approach =
                Math::dot(relativeVelocity(*contact.a, *contact.b, armA, armB), manifold.normal);
            // Below the threshold it is a resting contact; bouncing those keeps a settled stack alive forever.
            if (approach < -mSettings.restitutionThreshold)
                point.velocityBias = -contact.restitution * approach;
        }
    }

    buildEffectiveMass(contacts, count);

    for (u32 i = 0; i < jointCount; ++i)
        if (joints[i] && joints[i]->enabled())
            joints[i]->setup(duration);

    warmStart(contacts, count);

    for (u32 i = 0; i < jointCount; ++i)
        if (joints[i] && joints[i]->enabled())
            joints[i]->warmStart();

    for (u32 iteration = 0; iteration < mSettings.velocityIterations; ++iteration)
    {
        solveVelocity(contacts, count);
        for (u32 i = 0; i < jointCount; ++i)
            if (joints[i] && joints[i]->enabled())
                joints[i]->solveVelocity();
    }

    for (u32 iteration = 0; iteration < mSettings.positionIterations; ++iteration)
    {
        solvePosition(contacts, count);
        for (u32 i = 0; i < jointCount; ++i)
            if (joints[i] && joints[i]->enabled())
                joints[i]->solvePosition(mSettings.baumgarte);
    }
}

} // namespace Radion::Physics
