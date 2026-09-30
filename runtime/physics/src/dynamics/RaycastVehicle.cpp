#include "PCH.h"

#include "dynamics/RaycastVehicle.h"

#include "Scene.h"
#include "dynamics/RigidBody.h"

#include <cmath>

namespace Radion::Physics
{

namespace
{

const Math::vec3 kUpAxis(0.0f, 1.0f, 0.0f);
const Math::vec3 kForwardAxis(0.0f, 0.0f, 1.0f);

f32 impulseDenominator(const RigidBody& body, const Math::vec3& pos, const Math::vec3& normal)
{
    const Math::vec3 r0 = pos - body.position();
    const Math::vec3 c0 = Math::cross(r0, normal);
    const Math::vec3 vec = Math::cross(body.inverseInertiaTensorWorld() * c0, r0);
    return body.inverseMass() + Math::dot(normal, vec);
}

// Friction is always resolved against an immovable ground point (the reference substitutes a fixed zero-velocity body for whatever was hit),
// so the second body has zero velocity and infinite mass; the two-body formulas are specialised to drop its terms.
f32 resolveSingleBilateral(RigidBody& chassis, const Math::vec3& pos, const Math::vec3& normal)
{
    const f32 normalLenSqr = Math::dot(normal, normal);
    if (normalLenSqr > 1.1f)
        return 0.0f;

    const Math::vec3 vel = chassis.velocityAtPoint(pos);
    const f32 jacDiagAB = impulseDenominator(chassis, pos, normal);
    if (jacDiagAB <= 0.0f)
        return 0.0f;
    const f32 jacDiagABInv = 1.0f / jacDiagAB;

    const f32 relVel = Math::dot(normal, vel);
    constexpr f32 contactDamping = 0.2f;
    return -contactDamping * relVel * jacDiagABInv;
}

f32 calcRollingFriction(RigidBody& chassis, const Math::vec3& contactPos, const Math::vec3& forwardDir,
                        f32 maxImpulse, u32 numWheelsOnGround)
{
    const Math::vec3 vel = chassis.velocityAtPoint(contactPos);
    const f32 vrel = Math::dot(forwardDir, vel);
    const f32 jacDiagABInv = 1.0f / impulseDenominator(chassis, contactPos, forwardDir);

    f32 j1 = -vrel * jacDiagABInv / static_cast<f32>(numWheelsOnGround);
    j1 = Math::min(j1, maxImpulse);
    j1 = Math::max(j1, -maxImpulse);
    return j1;
}

constexpr f32 kSideFrictionStiffness = 1.0f;

} // namespace

RaycastVehicle::RaycastVehicle(RigidBody& chassis, const Radion::Scene* scene)
    : mChassis(chassis), mScene(scene)
{
}

u32 RaycastVehicle::addWheel(const Math::vec3& connectionPointLocal, const Math::vec3& directionLocal,
                             const Math::vec3& axleLocal, f32 suspensionRestLength, f32 wheelRadius,
                             const Tuning& tuning, bool isFrontWheel)
{
    Wheel wheel;
    wheel.chassisConnectionLocal = connectionPointLocal;
    wheel.directionLocal = directionLocal;
    wheel.axleLocal = axleLocal;
    wheel.restLength = suspensionRestLength;
    wheel.radius = wheelRadius;
    wheel.stiffness = tuning.suspensionStiffness;
    wheel.dampingCompression = tuning.suspensionCompression;
    wheel.dampingRelaxation = tuning.suspensionDamping;
    wheel.frictionSlip = tuning.frictionSlip;
    wheel.maxSuspensionTravelCm = tuning.maxSuspensionTravelCm;
    wheel.maxSuspensionForce = tuning.maxSuspensionForce;
    wheel.isFrontWheel = isFrontWheel;

    mWheels.push_back(wheel);
    Wheel& stored = mWheels.back();
    updateWheelTransformWS(stored);
    updateWheelTransform(stored);
    return static_cast<u32>(mWheels.size() - 1);
}

void RaycastVehicle::resetSuspension()
{
    for (Wheel& wheel : mWheels)
    {
        wheel.suspensionLength = wheel.restLength;
        wheel.suspensionRelativeVelocity = 0.0f;
        wheel.contactNormal = -wheel.directionWorld;
        wheel.clippedInvContactDotSuspension = 1.0f;
    }
}

void RaycastVehicle::updateWheelTransformWS(Wheel& wheel)
{
    wheel.inContact = false;
    wheel.hardPointWorld = mChassis.pointToWorld(wheel.chassisConnectionLocal);
    wheel.directionWorld = mChassis.directionToWorld(wheel.directionLocal);
    wheel.axleWorld = mChassis.directionToWorld(wheel.axleLocal);
}

void RaycastVehicle::updateWheelTransform(Wheel& wheel)
{
    updateWheelTransformWS(wheel);

    const Math::vec3 up = -wheel.directionWorld;
    const Math::vec3& right = wheel.axleWorld;
    // A suspension direction along the wheel's own axle gives a zero cross product: normalising it is NaN, which would reach the wheel's
    // transform; fall back to a sane default for that wheel.
    const Math::vec3 forwardRaw = Math::cross(up, right);
    const f32 forwardLength = Math::length(forwardRaw);
    const Math::vec3 fwd =
        forwardLength > 1.0e-6f ? forwardRaw / forwardLength : Math::vec3(0.0f, 0.0f, 1.0f);

    const Math::quat steeringOrientation = Math::angleAxis(wheel.steering, up);
    const Math::quat rotatingOrientation = Math::angleAxis(-wheel.rotation, right);

    const Math::mat3 basis2(-right, up, fwd);
    const Math::mat3 basis =
        Math::mat3_cast(steeringOrientation) * Math::mat3_cast(rotatingOrientation) * basis2;

    wheel.worldTransform = Math::mat4(basis);
    wheel.worldTransform[3] =
        Math::vec4(wheel.hardPointWorld + wheel.directionWorld * wheel.suspensionLength, 1.0f);
}

f32 RaycastVehicle::rayCast(Wheel& wheel)
{
    updateWheelTransformWS(wheel);

    f32 depth = -1.0f;
    const f32 rayLength = wheel.restLength + wheel.radius;

    Ray ray;
    ray.origin = wheel.hardPointWorld;
    ray.direction = wheel.directionWorld;

    QueryFilter filter;
    filter.ignoredBody = &mChassis;

    WorldRayHit hit;
    const bool hasHit = mScene && mScene->raycast(ray, rayLength, filter, hit);

    wheel.groundBody = nullptr;

    if (hasHit)
    {
        depth = hit.distance;
        wheel.contactNormal = hit.normal;
        wheel.inContact = true;
        wheel.groundBody = hit.body;

        wheel.suspensionLength = hit.distance - wheel.radius;

        const f32 minSuspensionLength = wheel.restLength - wheel.maxSuspensionTravelCm * 0.01f;
        const f32 maxSuspensionLength = wheel.restLength + wheel.maxSuspensionTravelCm * 0.01f;
        wheel.suspensionLength = Math::clamp(wheel.suspensionLength, minSuspensionLength, maxSuspensionLength);

        wheel.contactPoint = hit.point;

        const f32 denominator = Math::dot(wheel.contactNormal, wheel.directionWorld);
        const Math::vec3 chassisVelocityAtContact = mChassis.velocityAtPoint(wheel.contactPoint);
        const f32 projVel = Math::dot(wheel.contactNormal, chassisVelocityAtContact);

        if (denominator >= -0.1f)
        {
            wheel.suspensionRelativeVelocity = 0.0f;
            wheel.clippedInvContactDotSuspension = 1.0f / 0.1f;
        }
        else
        {
            const f32 inv = -1.0f / denominator;
            wheel.suspensionRelativeVelocity = projVel * inv;
            wheel.clippedInvContactDotSuspension = inv;
        }
    }
    else
    {
        wheel.suspensionLength = wheel.restLength;
        wheel.suspensionRelativeVelocity = 0.0f;
        wheel.contactNormal = -wheel.directionWorld;
        wheel.clippedInvContactDotSuspension = 1.0f;
    }

    return depth;
}

void RaycastVehicle::updateSuspension()
{
    const f32 chassisMass = 1.0f / mChassis.inverseMass();

    for (Wheel& wheel : mWheels)
    {
        if (!wheel.inContact)
        {
            wheel.suspensionForce = 0.0f;
            continue;
        }

        f32 force;
        {
            const f32 lengthDiff = wheel.restLength - wheel.suspensionLength;
            force = wheel.stiffness * lengthDiff * wheel.clippedInvContactDotSuspension;
        }
        {
            const f32 projectedRelVel = wheel.suspensionRelativeVelocity;
            const f32 damping =
                projectedRelVel < 0.0f ? wheel.dampingCompression : wheel.dampingRelaxation;
            force -= damping * projectedRelVel;
        }

        wheel.suspensionForce = Math::max(force * chassisMass, 0.0f);
    }
}

void RaycastVehicle::updateFriction(f32 step)
{
    const u32 numWheels = wheelCount();
    if (numWheels == 0)
        return;

    mForwardWS.resize(numWheels);
    mAxleWS.resize(numWheels);
    mForwardImpulse.resize(numWheels);
    mSideImpulse.resize(numWheels);

    u32 numWheelsOnGround = 0;
    for (Wheel& wheel : mWheels)
    {
        if (wheel.inContact)
            ++numWheelsOnGround;
    }
    for (u32 i = 0; i < numWheels; ++i)
    {
        mSideImpulse[i] = 0.0f;
        mForwardImpulse[i] = 0.0f;
    }

    for (u32 i = 0; i < numWheels; ++i)
    {
        Wheel& wheel = mWheels[i];
        if (!wheel.inContact)
            continue;

        const Math::mat3 wheelBasis(wheel.worldTransform);
        mAxleWS[i] = -wheelBasis[0];

        const Math::vec3& surfaceNormal = wheel.contactNormal;
        const f32 proj = Math::dot(mAxleWS[i], surfaceNormal);
        mAxleWS[i] -= surfaceNormal * proj;

        // Axle square to the surface (car on its side, steep ramp): the projection cancels and normalising zero is NaN, which would reach the
        // chassis via the side impulse; take no side force this step.
        const f32 axleLength = Math::length(mAxleWS[i]);
        if (axleLength < 1.0e-6f)
        {
            mSideImpulse[i] = 0.0f;
            continue;
        }
        mAxleWS[i] /= axleLength;

        mForwardWS[i] = Math::normalize(Math::cross(surfaceNormal, mAxleWS[i]));

        mSideImpulse[i] = resolveSingleBilateral(mChassis, wheel.contactPoint, mAxleWS[i]);
        mSideImpulse[i] *= kSideFrictionStiffness;
    }

    constexpr f32 sideFactor = 1.0f;
    constexpr f32 fwdFactor = 0.5f;
    bool sliding = false;

    for (u32 i = 0; i < numWheels; ++i)
    {
        Wheel& wheel = mWheels[i];
        f32 rollingFriction = 0.0f;

        if (wheel.inContact)
        {
            if (wheel.engineForce != 0.0f)
            {
                rollingFriction = wheel.engineForce * step;
            }
            else
            {
                const f32 maxImpulse = wheel.brake != 0.0f ? wheel.brake : 0.0f;
                rollingFriction =
                    calcRollingFriction(mChassis, wheel.contactPoint, mForwardWS[i], maxImpulse,
                                       numWheelsOnGround);
            }
        }

        mForwardImpulse[i] = 0.0f;
        wheel.skidInfo = 1.0f;

        if (wheel.inContact)
        {
            wheel.skidInfo = 1.0f;

            const f32 maxImp = wheel.suspensionForce * step * wheel.frictionSlip;
            const f32 maxImpSquared = maxImp * maxImp;

            mForwardImpulse[i] = rollingFriction;

            const f32 x = mForwardImpulse[i] * fwdFactor;
            const f32 y = mSideImpulse[i] * sideFactor;
            const f32 impulseSquared = x * x + y * y;

            if (impulseSquared > maxImpSquared)
            {
                sliding = true;
                const f32 factor = maxImp / std::sqrt(impulseSquared);
                wheel.skidInfo *= factor;
            }
        }
    }

    if (sliding)
    {
        for (u32 i = 0; i < numWheels; ++i)
        {
            if (mSideImpulse[i] != 0.0f && mWheels[i].skidInfo < 1.0f)
            {
                mForwardImpulse[i] *= mWheels[i].skidInfo;
                mSideImpulse[i] *= mWheels[i].skidInfo;
            }
        }
    }

    for (u32 i = 0; i < numWheels; ++i)
    {
        Wheel& wheel = mWheels[i];
        wheel.appliedSideImpulse = 0.0f;
        if (!wheel.inContact)
            continue;

        wheel.appliedSideImpulse = mSideImpulse[i];
        wheel.lateralWorld = mAxleWS[i];
        Math::vec3 relPos = wheel.contactPoint - mChassis.position();

        if (mForwardImpulse[i] != 0.0f)
            mChassis.applyImpulseAtPoint(mForwardWS[i] * mForwardImpulse[i], wheel.contactPoint);

        if (mSideImpulse[i] != 0.0f)
        {
            const Math::vec3 sideImpulse = mAxleWS[i] * mSideImpulse[i];

            const Math::vec3 chassisWorldUp = mChassis.directionToWorld(kUpAxis);
            relPos -= chassisWorldUp * (Math::dot(chassisWorldUp, relPos) * (1.0f - wheel.rollInfluence));

            mChassis.applyImpulseAtPoint(sideImpulse, mChassis.position() + relPos);
        }
    }
}

void RaycastVehicle::update(f32 step)
{
    for (Wheel& wheel : mWheels)
        updateWheelTransform(wheel);

    mCurrentSpeedKmHour = 3.6f * Math::length(mChassis.velocity());

    const Math::vec3 forwardW = mChassis.directionToWorld(kForwardAxis);
    if (Math::dot(forwardW, mChassis.velocity()) < 0.0f)
        mCurrentSpeedKmHour *= -1.0f;

    for (Wheel& wheel : mWheels)
        rayCast(wheel);

    updateSuspension();

    for (Wheel& wheel : mWheels)
    {
        f32 suspensionForce = wheel.suspensionForce;
        if (suspensionForce > wheel.maxSuspensionForce)
            suspensionForce = wheel.maxSuspensionForce;

        wheel.appliedSuspensionImpulse = suspensionForce * step;
        const Math::vec3 impulse = wheel.contactNormal * wheel.appliedSuspensionImpulse;
        mChassis.applyImpulseAtPoint(impulse, wheel.contactPoint);
    }

    updateFriction(step);

    for (Wheel& wheel : mWheels)
    {
        const Math::vec3 vel = mChassis.velocityAtPoint(wheel.hardPointWorld);

        if (wheel.inContact)
        {
            Math::vec3 fwd = mChassis.directionToWorld(kForwardAxis);
            const f32 proj = Math::dot(fwd, wheel.contactNormal);
            fwd -= wheel.contactNormal * proj;

            const f32 proj2 = Math::dot(fwd, vel);
            wheel.deltaRotation = (proj2 * step) / wheel.radius;
            wheel.rotation += wheel.deltaRotation;
        }
        else
        {
            wheel.rotation += wheel.deltaRotation;
        }

        wheel.deltaRotation *= 0.99f;
    }
}

void RaycastVehicle::setSteering(f32 steering, u32 wheelIndex)
{
    mWheels[wheelIndex].steering = steering;
}

void RaycastVehicle::setEngineForce(f32 force, u32 wheelIndex)
{
    mWheels[wheelIndex].engineForce = force;
}

void RaycastVehicle::setBrake(f32 brake, u32 wheelIndex)
{
    mWheels[wheelIndex].brake = brake;
}

} // namespace Radion::Physics
