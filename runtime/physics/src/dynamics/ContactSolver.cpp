#include "PCH.h"

#include "dynamics/ContactSolver.h"

#include "dynamics/Joint.h"
#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

namespace
{
// How much velocity a unit impulse along `direction` at these two points
// actually produces - the linear part plus what the two inertia tensors let
// the resulting torque do. Dividing by it is what turns a wanted velocity
// change into the impulse that delivers it.
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

// The normal points from A to B, so B takes the impulse and A takes its
// opposite.
//
// Deliberately NOT RigidBody::applyImpulseAtPoint: that wakes the body, and
// the solver touches every resting contact every single step. Going through
// it meant a settled stack was woken eight times a step forever, its sleep
// counter reset before it could ever cross the threshold - so nothing slept
// and the whole tower kept creeping. Waking belongs to the moment a contact
// is NEW, which PhysicsWorld does from its pair cache.
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
            // Last step's answer, applied before this step's first iteration.
            // Without it every step starts from zero and a tall stack sinks
            // by however much the iterations could not recover.
            const Math::vec3 impulse = manifold.normal * point.normalImpulse +
                                      manifold.tangent[0] * point.tangentImpulse[0] +
                                      manifold.tangent[1] * point.tangentImpulse[1];
            applyPair(*contact.a, *contact.b, impulse, point.position);
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

        for (u32 i = 0; i < manifold.count; ++i)
        {
            ContactPoint& point = manifold.points[i];
            const Math::vec3 armA = point.position - a.position();
            const Math::vec3 armB = point.position - b.position();

            // Friction first, and against the normal impulse the last
            // iteration settled on: the friction cone needs a normal force to
            // be a fraction of, and using this iteration's would let the two
            // chase each other.
            const f32 maxFriction = contact.friction * point.normalImpulse;
            for (u32 t = 0; t < 2; ++t)
            {
                const Math::vec3& tangent = manifold.tangent[t];
                const f32 mass = effectiveMass(a, b, armA, armB, tangent);
                if (mass <= 0.0f)
                    continue;
                const f32 speed = Math::dot(relativeVelocity(a, b, armA, armB), tangent);
                const f32 wanted = -speed / mass;
                const f32 previous = point.tangentImpulse[t];
                const f32 total = Math::clamp(previous + wanted, -maxFriction, maxFriction);
                point.tangentImpulse[t] = total;
                applyPair(a, b, tangent * (total - previous), point.position);
            }

            const f32 mass = effectiveMass(a, b, armA, armB, manifold.normal);
            if (mass <= 0.0f)
                continue;
            const f32 separation =
                Math::dot(relativeVelocity(a, b, armA, armB), manifold.normal);
            // The bias is the separation speed a bounce should end at. Zero
            // for a resting contact, so this is the plain "stop approaching".
            const f32 wanted = -(separation - point.velocityBias) / mass;

            // Clamping the ACCUMULATED impulse and not each increment is what
            // lets this converge: a later contact may need to take back part
            // of what an earlier iteration gave, and only a running total can
            // be reduced without going negative overall.
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
            // Overlap up to the slop is left alone. Chasing exactly zero is
            // what makes two resting bodies make and lose contact every
            // frame, and the buzz that comes with it.
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

            // Moved directly rather than through a bias impulse: a bias adds
            // velocity the bodies keep afterwards, which is how a resting
            // stack slowly gains energy and starts to drift.
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

    // Tangents are rebuilt from the normal here rather than trusted from the
    // narrowphase: a manifold can be carried over from the previous step for
    // its impulses, and its normal may have turned since.
    //
    // Waking is deliberately not done here. A contact exists every step a
    // stack is resting, so waking on one would mean nothing ever sleeps -
    // it belongs to whoever notices a contact is NEW, which is the same
    // per-pair bookkeeping that warm starting and enter/stay/exit need.
    for (u32 c = 0; c < count; ++c)
    {
        Contact& contact = contacts[c];
        if (!contact.a || !contact.b)
            continue;
        ContactManifold& manifold = contact.manifold;
        manifold.buildTangents();

        // Restitution has to be decided HERE, from the approach speed before
        // a single impulse has been applied. Measured after the velocity
        // iterations there is nothing left to be a fraction of - they have
        // already driven the approach to zero - and the bounce silently
        // vanishes.
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
            // Below the threshold it is a resting contact, not an impact.
            // Bouncing those is what keeps a settled stack alive forever.
            if (approach < -mSettings.restitutionThreshold)
                point.velocityBias = -contact.restitution * approach;
        }
    }

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
