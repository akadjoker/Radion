#include "PCH.h"

#include "dynamics/RigidBody.h"

#include "Log.h"

namespace Radion::Physics
{

namespace
{
// Sleep decides on a running average of v^2 + w^2, so a body that is briefly
// slow at the top of an arc is not mistaken for one that has stopped. The
// weight is raised to the step so the average decays at the same rate in
// seconds however often it is updated.
f32 motionBias(f32 duration)
{
    return std::pow(0.5f, duration);
}

bool finiteVec(const Math::vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
} // namespace

Math::mat3 Inertia::box(f32 mass, const Math::vec3& halfExtents)
{
    const Math::vec3 size = Math::abs(halfExtents) * 2.0f;
    const f32 factor = mass / 12.0f;
    const f32 x = size.x * size.x;
    const f32 y = size.y * size.y;
    const f32 z = size.z * size.z;
    return Math::mat3(Math::vec3(factor * (y + z), 0.0f, 0.0f),
                     Math::vec3(0.0f, factor * (x + z), 0.0f),
                     Math::vec3(0.0f, 0.0f, factor * (x + y)));
}

Math::mat3 Inertia::solidSphere(f32 mass, f32 radius)
{
    const f32 value = 0.4f * mass * radius * radius;
    return Math::mat3(Math::vec3(value, 0.0f, 0.0f), Math::vec3(0.0f, value, 0.0f),
                     Math::vec3(0.0f, 0.0f, value));
}

Math::mat3 Inertia::hollowSphere(f32 mass, f32 radius)
{
    const f32 value = (2.0f / 3.0f) * mass * radius * radius;
    return Math::mat3(Math::vec3(value, 0.0f, 0.0f), Math::vec3(0.0f, value, 0.0f),
                     Math::vec3(0.0f, 0.0f, value));
}

Math::mat3 Inertia::cylinderY(f32 mass, f32 radius, f32 height)
{
    const f32 radial = 0.5f * mass * radius * radius;
    const f32 lateral = (1.0f / 12.0f) * mass * (3.0f * radius * radius + height * height);
    return Math::mat3(Math::vec3(lateral, 0.0f, 0.0f), Math::vec3(0.0f, radial, 0.0f),
                     Math::vec3(0.0f, 0.0f, lateral));
}

Math::mat3 Inertia::convexHull(f32 mass, const Math::vec3* vertices, u32 vertexCount,
                              const Radion::Geometry::ConvexHullComputer::Edge* edges,
                              u32 edgeCount, const int* faces, u32 faceCount)
{
    using Edge = Radion::Geometry::ConvexHullComputer::Edge;
    (void)edgeCount;

    // Signed tetrahedron decomposition: every face triangle plus the hull's
    // own centroid forms a tetrahedron, and both the volume and the
    // second-moment matrix are sums of those tetrahedra's own closed-form
    // contributions - the same triangle-fan walk VoronoiShatter::shatter
    // already uses to accumulate volume and centroid, extended here to also
    // accumulate the inertia terms.
    if (vertexCount < 4 || faceCount == 0 || mass <= 0.0f)
        return Math::mat3(0.0f);

    f32 volume6 = 0.0f;
    Math::vec3 centroid(0.0f);
    for (u32 faceIndex = 0; faceIndex < faceCount; ++faceIndex)
    {
        const Edge* edge = &edges[static_cast<usize>(faces[faceIndex])];
        int v0 = edge->getSourceVertex();
        int v1 = edge->getTargetVertex();
        edge = edge->getNextEdgeOfFace();
        int v2 = edge->getTargetVertex();
        while (v2 != v0)
        {
            const f32 signedVolume = Math::dot(vertices[v0], Math::cross(vertices[v1], vertices[v2]));
            volume6 += signedVolume;
            centroid += signedVolume * (vertices[v0] + vertices[v1] + vertices[v2]);
            edge = edge->getNextEdgeOfFace();
            v1 = v2;
            v2 = edge->getTargetVertex();
        }
    }
    if (std::abs(volume6) < 1.0e-12f)
        return Math::mat3(0.0f);
    centroid /= volume6 * 4.0f;
    const f32 volume = volume6 / 6.0f;

    // Second moment of the reference tetrahedron (apex at the origin, the
    // other three vertices at the unit axis points) - a fixed matrix, derived
    // once by integrating u_i*u_j over the standard simplex {u,v,w >= 0,
    // u+v+w <= 1}: a!b!c!/(a+b+c+3)! gives 1/60 on the diagonal (a=2) and
    // 1/120 off it (a=b=1). Every actual tetrahedron below maps to this one
    // through its own Jacobian, so it is only computed once.
    const Math::mat3 referenceMoment(Math::vec3(1.0f / 60.0f, 1.0f / 120.0f, 1.0f / 120.0f),
                                    Math::vec3(1.0f / 120.0f, 1.0f / 60.0f, 1.0f / 120.0f),
                                    Math::vec3(1.0f / 120.0f, 1.0f / 120.0f, 1.0f / 60.0f));

    Math::mat3 secondMoment(0.0f);
    for (u32 faceIndex = 0; faceIndex < faceCount; ++faceIndex)
    {
        const Edge* edge = &edges[static_cast<usize>(faces[faceIndex])];
        int v0 = edge->getSourceVertex();
        int v1 = edge->getTargetVertex();
        edge = edge->getNextEdgeOfFace();
        int v2 = edge->getTargetVertex();
        while (v2 != v0)
        {
            // Tetrahedron (centroid, v0, v1, v2), apex at the centroid - so
            // measuring from the centroid puts the apex at the local origin,
            // and the integral below is already the contribution to the
            // inertia about the centroid, with no parallel-axis shift needed.
            const Math::vec3 a = vertices[v0] - centroid;
            const Math::vec3 b = vertices[v1] - centroid;
            const Math::vec3 c = vertices[v2] - centroid;
            const Math::mat3 jacobian(a, b, c);
            const f32 signedDeterminant = Math::dot(a, Math::cross(b, c));
            secondMoment += signedDeterminant * (jacobian * referenceMoment *
                                                 Math::transpose(jacobian));
            edge = edge->getNextEdgeOfFace();
            v1 = v2;
            v2 = edge->getTargetVertex();
        }
    }

    const f32 density = mass / volume;
    const Math::mat3 s = secondMoment * density;
    const f32 sxx = s[0][0];
    const f32 syy = s[1][1];
    const f32 szz = s[2][2];
    const f32 sxy = 0.5f * (s[0][1] + s[1][0]);
    const f32 sxz = 0.5f * (s[0][2] + s[2][0]);
    const f32 syz = 0.5f * (s[1][2] + s[2][1]);

    return Math::mat3(Math::vec3(syy + szz, -sxy, -sxz), Math::vec3(-sxy, sxx + szz, -syz),
                     Math::vec3(-sxz, -syz, sxx + syy));
}

Math::mat3 Inertia::capsuleY(f32 mass, f32 radius, f32 cylinderHeight)
{
    // Split the mass between the cylinder and the two hemispheres by volume,
    // then move each hemisphere's own tensor out to where it actually sits
    // with the parallel axis theorem - a capsule treated as one cylinder
    // spins visibly wrong once the caps are a real share of its length.
    const f32 radiusSquared = radius * radius;
    const f32 cylinderVolume = Math::pi<f32>() * radiusSquared * cylinderHeight;
    const f32 sphereVolume = (4.0f / 3.0f) * Math::pi<f32>() * radiusSquared * radius;
    const f32 total = cylinderVolume + sphereVolume;
    if (total <= 0.0f)
        return solidSphere(mass, radius);

    const f32 cylinderMass = mass * (cylinderVolume / total);
    const f32 sphereMass = mass * (sphereVolume / total);

    const f32 radial = 0.5f * cylinderMass * radiusSquared + 0.4f * sphereMass * radiusSquared;

    f32 lateral = (1.0f / 12.0f) * cylinderMass *
                  (3.0f * radiusSquared + cylinderHeight * cylinderHeight);
    const f32 offset = cylinderHeight * 0.5f + 0.375f * radius;
    lateral += 0.4f * sphereMass * radiusSquared + sphereMass * offset * offset;

    return Math::mat3(Math::vec3(lateral, 0.0f, 0.0f), Math::vec3(0.0f, radial, 0.0f),
                     Math::vec3(0.0f, 0.0f, lateral));
}

RigidBody::RigidBody()
{
    // Awake AND seeded above the threshold, which is what setAwake(true)
    // means. Setting only the flag left the motion average at zero, and
    // since sleep is decided on that average, a body that starts at rest and
    // is then let go fell asleep on its first step - before it had moved far
    // enough for the average to catch up with it. It then hung in the air.
    setAwake(true);
    calculateDerivedData();
}

void RigidBody::setBodyType(BodyType type)
{
    if (mBodyType == type)
        return;
    mBodyType = type;
    applyBodyTypeMass();
    if (type != BodyType::Dynamic)
    {
        clearAccumulators();
        mLastFrameAcceleration = Math::vec3(0.0f);
    }
    setAwake(true);
}

void RigidBody::applyBodyTypeMass()
{
    if (mBodyType == BodyType::Dynamic)
    {
        mInverseMass = mDynamicInverseMass;
        mInverseInertiaTensor = mDynamicInverseInertiaTensor;
    }
    else
    {
        // Infinite mass and infinite inertia. An impulse divided by these
        // changes nothing, which is exactly what "a contact does not move
        // this" means to the solver - no special case anywhere else.
        mInverseMass = 0.0f;
        mInverseInertiaTensor = Math::mat3(0.0f);
    }
    calculateDerivedData();
}

void RigidBody::setMass(f32 mass)
{
    if (!(mass > 0.0f) || !std::isfinite(mass))
    {
        Log::error("RigidBody: mass must be finite and above zero - use BodyType::Static for an "
                   "immovable body");
        return;
    }
    setInverseMass(1.0f / mass);
}

f32 RigidBody::mass() const
{
    if (mInverseMass <= 0.0f)
        return std::numeric_limits<f32>::infinity();
    return 1.0f / mInverseMass;
}

void RigidBody::setInverseMass(f32 inverseMass)
{
    if (!std::isfinite(inverseMass) || inverseMass < 0.0f)
    {
        Log::error("RigidBody: inverse mass must be finite and not negative");
        return;
    }
    mDynamicInverseMass = inverseMass;
    if (mBodyType == BodyType::Dynamic)
        mInverseMass = inverseMass;
}

void RigidBody::setInertiaTensor(const Math::mat3& inertiaTensor)
{
    const f32 determinant = Math::determinant(inertiaTensor);
    if (!std::isfinite(determinant) || std::abs(determinant) < 1e-12f)
    {
        Log::error("RigidBody: inertia tensor is not invertible - a body with zero inertia about "
                   "an axis spins without limit around it");
        return;
    }
    setInverseInertiaTensor(Math::inverse(inertiaTensor));
}

Math::mat3 RigidBody::inertiaTensor() const
{
    const f32 determinant = Math::determinant(mInverseInertiaTensor);
    if (std::abs(determinant) < 1e-12f)
        return Math::mat3(0.0f);
    return Math::inverse(mInverseInertiaTensor);
}

void RigidBody::setInverseInertiaTensor(const Math::mat3& inverseInertiaTensor)
{
    mDynamicInverseInertiaTensor = inverseInertiaTensor;
    if (mBodyType == BodyType::Dynamic)
    {
        mInverseInertiaTensor = inverseInertiaTensor;
        calculateDerivedData();
    }
}

void RigidBody::setDamping(f32 linear, f32 angular)
{
    mLinearDamping = Math::clamp(linear, 0.0f, 1.0f);
    mAngularDamping = Math::clamp(angular, 0.0f, 1.0f);
}

void RigidBody::setPosition(const Math::vec3& position)
{
    if (!finiteVec(position))
        return;
    mPosition = position;
    calculateDerivedData();
}

void RigidBody::setOrientation(const Math::quat& orientation)
{
    if (!std::isfinite(orientation.w) || !finiteVec(Math::vec3(orientation.x, orientation.y,
                                                              orientation.z)))
        return;
    mOrientation = orientation;
    calculateDerivedData();
}

void RigidBody::setVelocity(const Math::vec3& velocity)
{
    if (!finiteVec(velocity))
        return;
    mVelocity = velocity;
}

void RigidBody::setAngularVelocity(const Math::vec3& angularVelocity)
{
    if (!finiteVec(angularVelocity))
        return;
    mAngularVelocity = angularVelocity;
}

void RigidBody::setAcceleration(const Math::vec3& acceleration)
{
    if (!finiteVec(acceleration))
        return;
    mAcceleration = acceleration;
}

Math::vec3 RigidBody::pointToWorld(const Math::vec3& local) const
{
    return Math::vec3(mTransform * Math::vec4(local, 1.0f));
}

Math::vec3 RigidBody::pointToLocal(const Math::vec3& world) const
{
    return Math::conjugate(mOrientation) * (world - mPosition);
}

Math::vec3 RigidBody::directionToWorld(const Math::vec3& local) const
{
    return mOrientation * local;
}

Math::vec3 RigidBody::directionToLocal(const Math::vec3& world) const
{
    return Math::conjugate(mOrientation) * world;
}

Math::vec3 RigidBody::velocityAtPoint(const Math::vec3& worldPoint) const
{
    return mVelocity + Math::cross(mAngularVelocity, worldPoint - mPosition);
}

void RigidBody::addForce(const Math::vec3& force)
{
    if (!isDynamic() || !finiteVec(force))
        return;
    mForceAccumulator += force;
    setAwake(true);
}

void RigidBody::addForceAtPoint(const Math::vec3& force, const Math::vec3& worldPoint)
{
    if (!isDynamic() || !finiteVec(force) || !finiteVec(worldPoint))
        return;
    // The torque is the offset from the centre of mass crossed with the
    // force. This is the whole reason a body rotates from a hit that is not
    // aimed at its middle.
    mForceAccumulator += force;
    mTorqueAccumulator += Math::cross(worldPoint - mPosition, force);
    setAwake(true);
}

void RigidBody::addForceAtBodyPoint(const Math::vec3& force, const Math::vec3& localPoint)
{
    addForceAtPoint(force, pointToWorld(localPoint));
}

void RigidBody::addTorque(const Math::vec3& torque)
{
    if (!isDynamic() || !finiteVec(torque))
        return;
    mTorqueAccumulator += torque;
    setAwake(true);
}

void RigidBody::clearAccumulators()
{
    mForceAccumulator = Math::vec3(0.0f);
    mTorqueAccumulator = Math::vec3(0.0f);
}

void RigidBody::applyLinearImpulse(const Math::vec3& impulse)
{
    if (!isDynamic() || !finiteVec(impulse))
        return;
    mVelocity += impulse * mInverseMass;
    setAwake(true);
}

void RigidBody::applyAngularImpulse(const Math::vec3& impulse)
{
    if (!isDynamic() || !finiteVec(impulse))
        return;
    mAngularVelocity += mInverseInertiaTensorWorld * impulse;
    setAwake(true);
}

void RigidBody::applyImpulseAtPoint(const Math::vec3& impulse, const Math::vec3& worldPoint)
{
    if (!isDynamic() || !finiteVec(impulse) || !finiteVec(worldPoint))
        return;
    mVelocity += impulse * mInverseMass;
    mAngularVelocity += mInverseInertiaTensorWorld * Math::cross(worldPoint - mPosition, impulse);
    setAwake(true);
}

void RigidBody::applyPositionImpulseAtPoint(const Math::vec3& impulse,
                                            const Math::vec3& worldPoint)
{
    if (!isDynamic() || !finiteVec(impulse) || !finiteVec(worldPoint))
        return;
    const Math::vec3 arm = worldPoint - mPosition;
    mPosition += impulse * mInverseMass;
    const Math::vec3 angularStep =
        mInverseInertiaTensorWorld * Math::cross(arm, impulse);
    const Math::quat spin(0.0f, angularStep);
    mOrientation += 0.5f * spin * mOrientation;
    calculateDerivedData();
}

void RigidBody::setAwake(bool awake)
{
    if (awake)
    {
        mAwake = true;
        // Seeded above the threshold so the very next step does not put it
        // straight back to sleep before anything has had time to happen.
        mMotion = mSleepEpsilon * 2.0f;
        return;
    }
    mAwake = false;
    mVelocity = Math::vec3(0.0f);
    mAngularVelocity = Math::vec3(0.0f);
}

void RigidBody::setCanSleep(bool canSleep)
{
    mCanSleep = canSleep;
    if (!canSleep && !mAwake)
        setAwake(true);
}

void RigidBody::setSleepEpsilon(f32 epsilon)
{
    mSleepEpsilon = Math::max(epsilon, 0.0f);
}

void RigidBody::setSleepDeferred(bool deferred)
{
    mSleepDeferred = deferred;
}

void RigidBody::calculateDerivedData()
{
    mOrientation = Math::normalize(mOrientation);

    const Math::mat3 rotation = Math::mat3_cast(mOrientation);
    mTransform = Math::mat4(rotation);
    mTransform[3] = Math::vec4(mPosition, 1.0f);

    // The tensor is stored in body space because that is where it is
    // constant. Rotating it into world space is R * I * R^T, and it has to
    // happen every step the orientation changes - a torque is applied in
    // world space and there is nothing to divide it by otherwise.
    mInverseInertiaTensorWorld = rotation * mInverseInertiaTensor * Math::transpose(rotation);
}

void RigidBody::integrateForces(f32 duration)
{
    if (duration <= 0.0f || !std::isfinite(duration))
        return;
    if (!isDynamic() || !mAwake)
        return;

    mLastFrameAcceleration = mAcceleration + mForceAccumulator * mInverseMass;
    const Math::vec3 angularAcceleration = mInverseInertiaTensorWorld * mTorqueAccumulator;
    mVelocity += mLastFrameAcceleration * duration;
    mAngularVelocity += angularAcceleration * duration;
    mVelocity *= std::pow(mLinearDamping, duration);
    mAngularVelocity *= std::pow(mAngularDamping, duration);

    clearAccumulators();
}

void RigidBody::integrateVelocity(f32 duration)
{
    if (duration <= 0.0f || !std::isfinite(duration) || mBodyType == BodyType::Static)
        return;
    if (mBodyType == BodyType::Dynamic && !mAwake)
        return;

    mPosition += mVelocity * duration;
    const Math::quat spin(0.0f, mAngularVelocity * duration);
    mOrientation += 0.5f * spin * mOrientation;
    calculateDerivedData();

    if (mBodyType == BodyType::Kinematic)
    {
        clearAccumulators();
        return;
    }

    if (!mCanSleep)
        return;

    const f32 currentMotion =
        Math::dot(mVelocity, mVelocity) + Math::dot(mAngularVelocity, mAngularVelocity);
    const f32 bias = motionBias(duration);
    mMotion = bias * mMotion + (1.0f - bias) * currentMotion;
    if (mMotion < mSleepEpsilon && !mSleepDeferred)
        setAwake(false);
    else if (mMotion > 10.0f * mSleepEpsilon)
        // Capped so a body that has been thrown hard does not need as long to
        // settle as it spent moving.
        mMotion = 10.0f * mSleepEpsilon;
}

void RigidBody::integrate(f32 duration)
{
    integrateForces(duration);
    integrateVelocity(duration);
}

} // namespace Radion::Physics
