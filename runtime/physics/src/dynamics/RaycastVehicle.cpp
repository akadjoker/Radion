#include "PCH.h"

#include "dynamics/RaycastVehicle.h"

#include "Scene.h"
#include "dynamics/RigidBody.h"

#include <cmath>

namespace Radion::Physics
{

namespace
{

const glm::vec3 kUpAxis(0.0f, 1.0f, 0.0f);
const glm::vec3 kForwardAxis(0.0f, 0.0f, 1.0f);

f32 impulseDenominator(const RigidBody& body, const glm::vec3& pos, const glm::vec3& normal)
{
    const glm::vec3 r0 = pos - body.position();
    const glm::vec3 c0 = glm::cross(r0, normal);
    const glm::vec3 vec = glm::cross(body.inverseInertiaTensorWorld() * c0, r0);
    return body.inverseMass() + glm::dot(normal, vec);
}

// The friction pair is always resolved against an immovable ground point:
// the reference substitutes its own fixed, zero-velocity body for whichever
// object the raycast actually hit before any friction math runs, so the
// second body in every one of these formulas always has zero velocity and
// infinite mass. Specialising the two-body versions for that case is what
// removes the ground body's terms below rather than approximating them.
f32 resolveSingleBilateral(RigidBody& chassis, const glm::vec3& pos, const glm::vec3& normal)
{
    const f32 normalLenSqr = glm::dot(normal, normal);
    if (normalLenSqr > 1.1f)
        return 0.0f;

    const glm::vec3 vel = chassis.velocityAtPoint(pos);
    const f32 jacDiagAB = impulseDenominator(chassis, pos, normal);
    if (jacDiagAB <= 0.0f)
        return 0.0f;
    const f32 jacDiagABInv = 1.0f / jacDiagAB;

    const f32 relVel = glm::dot(normal, vel);
    constexpr f32 contactDamping = 0.2f;
    return -contactDamping * relVel * jacDiagABInv;
}

f32 calcRollingFriction(RigidBody& chassis, const glm::vec3& contactPos, const glm::vec3& forwardDir,
                        f32 maxImpulse, u32 numWheelsOnGround)
{
    const glm::vec3 vel = chassis.velocityAtPoint(contactPos);
    const f32 vrel = glm::dot(forwardDir, vel);
    const f32 jacDiagABInv = 1.0f / impulseDenominator(chassis, contactPos, forwardDir);

    f32 j1 = -vrel * jacDiagABInv / static_cast<f32>(numWheelsOnGround);
    j1 = glm::min(j1, maxImpulse);
    j1 = glm::max(j1, -maxImpulse);
    return j1;
}

constexpr f32 kSideFrictionStiffness = 1.0f;

} // namespace

RaycastVehicle::RaycastVehicle(RigidBody& chassis, const Radion::Scene* scene)
    : mChassis(chassis), mScene(scene)
{
}

u32 RaycastVehicle::addWheel(const glm::vec3& connectionPointLocal, const glm::vec3& directionLocal,
                             const glm::vec3& axleLocal, f32 suspensionRestLength, f32 wheelRadius,
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

    const glm::vec3 up = -wheel.directionWorld;
    const glm::vec3& right = wheel.axleWorld;
    // A wheel configured with its suspension direction along its own axle has
    // no forward: the cross product is zero and normalising it is NaN, which
    // goes straight into the wheel's world transform and from there into
    // anything drawn or queried from it. Bad configuration, but it should
    // cost that wheel a sane default, not the whole vehicle.
    const glm::vec3 forwardRaw = glm::cross(up, right);
    const f32 forwardLength = glm::length(forwardRaw);
    const glm::vec3 fwd =
        forwardLength > 1.0e-6f ? forwardRaw / forwardLength : glm::vec3(0.0f, 0.0f, 1.0f);

    const glm::quat steeringOrientation = glm::angleAxis(wheel.steering, up);
    const glm::quat rotatingOrientation = glm::angleAxis(-wheel.rotation, right);

    const glm::mat3 basis2(-right, up, fwd);
    const glm::mat3 basis =
        glm::mat3_cast(steeringOrientation) * glm::mat3_cast(rotatingOrientation) * basis2;

    wheel.worldTransform = glm::mat4(basis);
    wheel.worldTransform[3] =
        glm::vec4(wheel.hardPointWorld + wheel.directionWorld * wheel.suspensionLength, 1.0f);
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
        wheel.suspensionLength = glm::clamp(wheel.suspensionLength, minSuspensionLength, maxSuspensionLength);

        wheel.contactPoint = hit.point;

        const f32 denominator = glm::dot(wheel.contactNormal, wheel.directionWorld);
        const glm::vec3 chassisVelocityAtContact = mChassis.velocityAtPoint(wheel.contactPoint);
        const f32 projVel = glm::dot(wheel.contactNormal, chassisVelocityAtContact);

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

        wheel.suspensionForce = glm::max(force * chassisMass, 0.0f);
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

        const glm::mat3 wheelBasis(wheel.worldTransform);
        mAxleWS[i] = -wheelBasis[0];

        const glm::vec3& surfaceNormal = wheel.contactNormal;
        const f32 proj = glm::dot(mAxleWS[i], surfaceNormal);
        mAxleWS[i] -= surfaceNormal * proj;

        // With the axle square to the surface - a car on its side against a
        // wall, a wheel against a steep ramp - the projection above cancels
        // it out, and normalising zero is NaN, which then leaves through the
        // side impulse into the chassis. There is no lateral direction to
        // speak of in that pose, so the wheel simply takes no side force
        // this step instead of poisoning the vehicle.
        const f32 axleLength = glm::length(mAxleWS[i]);
        if (axleLength < 1.0e-6f)
        {
            mSideImpulse[i] = 0.0f;
            continue;
        }
        mAxleWS[i] /= axleLength;

        mForwardWS[i] = glm::normalize(glm::cross(surfaceNormal, mAxleWS[i]));

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
        glm::vec3 relPos = wheel.contactPoint - mChassis.position();

        if (mForwardImpulse[i] != 0.0f)
            mChassis.applyImpulseAtPoint(mForwardWS[i] * mForwardImpulse[i], wheel.contactPoint);

        if (mSideImpulse[i] != 0.0f)
        {
            const glm::vec3 sideImpulse = mAxleWS[i] * mSideImpulse[i];

            const glm::vec3 chassisWorldUp = mChassis.directionToWorld(kUpAxis);
            relPos -= chassisWorldUp * (glm::dot(chassisWorldUp, relPos) * (1.0f - wheel.rollInfluence));

            mChassis.applyImpulseAtPoint(sideImpulse, mChassis.position() + relPos);
        }
    }
}

void RaycastVehicle::update(f32 step)
{
    for (Wheel& wheel : mWheels)
        updateWheelTransform(wheel);

    mCurrentSpeedKmHour = 3.6f * glm::length(mChassis.velocity());

    const glm::vec3 forwardW = mChassis.directionToWorld(kForwardAxis);
    if (glm::dot(forwardW, mChassis.velocity()) < 0.0f)
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
        const glm::vec3 impulse = wheel.contactNormal * wheel.appliedSuspensionImpulse;
        mChassis.applyImpulseAtPoint(impulse, wheel.contactPoint);
    }

    updateFriction(step);

    for (Wheel& wheel : mWheels)
    {
        const glm::vec3 vel = mChassis.velocityAtPoint(wheel.hardPointWorld);

        if (wheel.inContact)
        {
            glm::vec3 fwd = mChassis.directionToWorld(kForwardAxis);
            const f32 proj = glm::dot(fwd, wheel.contactNormal);
            fwd -= wheel.contactNormal * proj;

            const f32 proj2 = glm::dot(fwd, vel);
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
