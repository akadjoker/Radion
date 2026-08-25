#include "PCH.h"

#include "softbody/SoftBody.h"

#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"
#include "dynamics/PhysicsWorld.h"

#include <algorithm>

namespace Radion::Physics
{

namespace
{
constexpr f32 kEpsilon = 1.0e-12f;

u64 edgeKey(u32 a, u32 b)
{
    const u64 low = a < b ? a : b;
    const u64 high = a < b ? b : a;
    return (low << 32) | high;
}

// Signed angle between the normals of the two triangles sharing edge x0-x1,
// with x2 and x3 the opposite vertices. The same expression the projection
// uses, so the rest angle measured at build time cancels exactly at rest.
f32 dihedralAngle(const glm::vec3& x0, const glm::vec3& x1, const glm::vec3& x2,
                  const glm::vec3& x3)
{
    const glm::vec3 e = x1 - x0;
    const glm::vec3 n1 = glm::cross(x2 - x0, x2 - x1);
    const glm::vec3 n2 = glm::cross(x3 - x1, x3 - x0);
    const f32 lengthsSquared = glm::dot(n1, n1) * glm::dot(n2, n2);
    if (lengthsSquared < 1.0e-24f)
        return 0.0f;
    const f32 sign = glm::dot(glm::cross(n2, n1), e) < 0.0f ? -1.0f : 1.0f;
    const f32 d = glm::clamp(glm::dot(n1, n2) / std::sqrt(lengthsSquared), -1.0f, 1.0f);
    return sign * std::acos(d);
}

// How far this particle can move before determineContactPlanes() runs again:
// its own speed over the step plus what predict() will add to it inside the
// step. A search below this lets a particle cross a collider and pop back out
// in one substep; a search above it only widens the narrowphase, which for a
// level-sized trimesh means every triangle within that distance, per
// particle, per step. mMaxLinearVelocity is not the number for this - it is
// the safety clamp step() applies at the end, 500 m/s by default, which is
// metres of reach at any normal frame rate.
f32 particleTravel(const SoftBody::Particle& particle, const glm::vec3& gravity,
                   const glm::vec3& windPerParticle, f32 dt)
{
    const glm::vec3 acceleration = gravity + windPerParticle * particle.invMass;
    return (glm::length(particle.velocity) + glm::length(acceleration) * dt) * dt;
}
} // namespace

void SoftBody::clear()
{
    mParticles.clear();
    mConstraints.clear();
    mBendConstraints.clear();
    mAttachments.clear();
}

void SoftBody::setParticles(const glm::vec3* positions, u32 count, f32 totalMass)
{
    mParticles.clear();
    mConstraints.clear();
    mBendConstraints.clear();
    mAttachments.clear();
    if (!positions || count == 0)
        return;

    const f32 perParticle = totalMass > 0.0f ? totalMass / static_cast<f32>(count) : 0.0f;
    const f32 invMass = perParticle > 0.0f ? 1.0f / perParticle : 0.0f;
    mParticles.resize(count);
    for (u32 i = 0; i < count; ++i)
    {
        Particle& particle = mParticles[i];
        particle.position = positions[i];
        particle.previousPosition = positions[i];
        particle.velocity = glm::vec3(0.0f);
        particle.invMass = invMass;
    }
}

void SoftBody::setTotalMass(f32 totalMass)
{
    if (mParticles.empty())
        return;
    const f32 perParticle = totalMass > 0.0f
                                ? totalMass / static_cast<f32>(mParticles.size())
                                : 0.0f;
    const f32 invMass = perParticle > 0.0f ? 1.0f / perParticle : 0.0f;
    for (Particle& particle : mParticles)
        if (particle.invMass != 0.0f)
            particle.invMass = invMass;
}

void SoftBody::addDistanceConstraint(u32 a, u32 b, f32 compliance)
{
    if (a >= mParticles.size() || b >= mParticles.size() || a == b)
        return;
    DistanceConstraint constraint;
    constraint.a = a;
    constraint.b = b;
    constraint.restLength = glm::length(mParticles[b].position - mParticles[a].position);
    constraint.compliance = glm::max(compliance, 0.0f);
    mConstraints.push_back(constraint);
}

void SoftBody::buildFromMesh(const u32* indices, u32 indexCount, f32 structuralCompliance,
                             f32 bendCompliance, BendType bendType)
{
    mConstraints.clear();
    if (!indices || indexCount < 3 || mParticles.empty())
        return;

    const u32 triangleCount = indexCount / 3;

    struct EdgeRecord
    {
        u64 key;
        u32 triangle;
        u32 opposite;
    };
    std::vector<EdgeRecord> records;
    records.reserve(static_cast<usize>(triangleCount) * 3);

    for (u32 triangle = 0; triangle < triangleCount; ++triangle)
    {
        const u32 v[3] = {indices[triangle * 3], indices[triangle * 3 + 1],
                          indices[triangle * 3 + 2]};
        for (u32 e = 0; e < 3; ++e)
            records.push_back({edgeKey(v[e], v[(e + 1) % 3]), triangle, v[(e + 2) % 3]});
    }

    std::sort(records.begin(), records.end(),
              [](const EdgeRecord& a, const EdgeRecord& b) { return a.key < b.key; });

    for (usize i = 0; i < records.size();)
    {
        usize end = i + 1;
        while (end < records.size() && records[end].key == records[i].key)
            ++end;

        const u32 a = static_cast<u32>(records[i].key >> 32);
        const u32 b = static_cast<u32>(records[i].key & 0xFFFFFFFFull);
        addDistanceConstraint(a, b, structuralCompliance);

        if (end - i == 2 && bendCompliance >= 0.0f && bendType != BendType::None)
        {
            if (bendType == BendType::Distance)
            {
                addDistanceConstraint(records[i].opposite, records[i + 1].opposite,
                                      bendCompliance);
            }
            else
            {
                DihedralBendConstraint bend;
                bend.a = a;
                bend.b = b;
                bend.c = records[i].opposite;
                bend.d = records[i + 1].opposite;
                bend.compliance = bendCompliance;
                bend.initialAngle =
                    dihedralAngle(mParticles[bend.a].position, mParticles[bend.b].position,
                                  mParticles[bend.c].position, mParticles[bend.d].position);
                mBendConstraints.push_back(bend);
            }
        }

        i = end;
    }
}

void SoftBody::setPinned(u32 index, bool pinnedState)
{
    if (index >= mParticles.size())
        return;
    if (pinnedState)
    {
        mParticles[index].invMass = 0.0f;
        return;
    }
    if (mParticles[index].invMass == 0.0f)
        mParticles[index].invMass = 1.0f;
}

bool SoftBody::pinned(u32 index) const
{
    return index < mParticles.size() && mParticles[index].invMass == 0.0f;
}

void SoftBody::buildAttachments(f32 maxDistanceMultiplier)
{
    mAttachments.clear();

    std::vector<u32> anchors;
    for (u32 i = 0; i < mParticles.size(); ++i)
        if (mParticles[i].invMass == 0.0f)
            anchors.push_back(i);
    if (anchors.empty())
        return;

    const f32 multiplier = glm::max(maxDistanceMultiplier, 1.0f);
    for (u32 i = 0; i < mParticles.size(); ++i)
    {
        if (mParticles[i].invMass == 0.0f)
            continue;

        u32 nearest = anchors[0];
        f32 nearestSquared = std::numeric_limits<f32>::max();
        for (u32 anchor : anchors)
        {
            const glm::vec3 delta = mParticles[i].position - mParticles[anchor].position;
            const f32 squared = glm::dot(delta, delta);
            if (squared < nearestSquared)
            {
                nearestSquared = squared;
                nearest = anchor;
            }
        }

        LongRangeAttachment attachment;
        attachment.anchor = nearest;
        attachment.vertex = i;
        attachment.maxDistance = std::sqrt(nearestSquared) * multiplier;
        mAttachments.push_back(attachment);
    }
}

void SoftBody::predict(f32 dt)
{
    const f32 damping = mDamping < 1.0f ? std::pow(mDamping, dt) : 1.0f;
    // Jolt distributes an accumulated body force over all vertices before
    // applying inverse mass. Without this division, refining a cloth mesh
    // multiplies its wind acceleration by its vertex count.
    const glm::vec3 windPerParticle =
        mParticles.empty() ? glm::vec3(0.0f)
                           : mWind / static_cast<f32>(mParticles.size());
    for (Particle& particle : mParticles)
    {
        particle.previousPosition = particle.position;
        if (particle.invMass == 0.0f)
            continue;
        particle.velocity += (mGravity + windPerParticle * particle.invMass) * dt;
        particle.velocity *= damping;
        particle.position += particle.velocity * dt;
    }
}

void SoftBody::projectDistanceConstraints(f32 dt)
{
    const f32 inverseStepSquared = dt > 0.0f ? 1.0f / (dt * dt) : 0.0f;

    for (const DistanceConstraint& constraint : mConstraints)
    {
        Particle& a = mParticles[constraint.a];
        Particle& b = mParticles[constraint.b];

        const glm::vec3 delta = b.position - a.position;
        const f32 length = glm::length(delta);
        if (length < kEpsilon)
            continue;

        const f32 denominator =
            length * (a.invMass + b.invMass + constraint.compliance * inverseStepSquared);
        if (denominator < kEpsilon)
            continue;

        const glm::vec3 correction = delta * ((length - constraint.restLength) / denominator);
        a.position += correction * a.invMass;
        b.position -= correction * b.invMass;
    }
}

void SoftBody::projectDihedralBendConstraints(f32 dt)
{
    const f32 inverseStepSquared = dt > 0.0f ? 1.0f / (dt * dt) : 0.0f;

    for (const DihedralBendConstraint& bend : mBendConstraints)
    {
        Particle& p0 = mParticles[bend.a];
        Particle& p1 = mParticles[bend.b];
        Particle& p2 = mParticles[bend.c];
        Particle& p3 = mParticles[bend.d];

        const glm::vec3 x0 = p0.position;
        const glm::vec3 x1 = p1.position;
        const glm::vec3 x2 = p2.position;
        const glm::vec3 x3 = p3.position;

        const glm::vec3 e = x1 - x0;
        const f32 edgeLength = glm::length(e);
        if (edgeLength < 1.0e-6f)
            continue;

        const glm::vec3 x1x2 = x2 - x1;
        const glm::vec3 x1x3 = x3 - x1;
        glm::vec3 n1 = glm::cross(x2 - x0, x1x2);
        glm::vec3 n2 = glm::cross(x1x3, x3 - x0);
        const f32 n1LengthSquared = glm::dot(n1, n1);
        const f32 n2LengthSquared = glm::dot(n2, n2);
        const f32 lengthsSquared = n1LengthSquared * n2LengthSquared;
        if (lengthsSquared < 1.0e-24f)
            continue;

        const f32 sign = glm::dot(glm::cross(n2, n1), e) < 0.0f ? -1.0f : 1.0f;
        const f32 d = glm::clamp(glm::dot(n1, n2) / std::sqrt(lengthsSquared), -1.0f, 1.0f);
        f32 c = sign * std::acos(d) - bend.initialAngle;
        if (c > glm::pi<f32>())
            c -= glm::two_pi<f32>();
        else if (c < -glm::pi<f32>())
            c += glm::two_pi<f32>();

        n1 /= n1LengthSquared;
        n2 /= n2LengthSquared;
        const glm::vec3 d0c = (glm::dot(x1x2, e) * n1 + glm::dot(x1x3, e) * n2) / edgeLength;
        const glm::vec3 d2c = edgeLength * n1;
        const glm::vec3 d3c = edgeLength * n2;
        const glm::vec3 d1c = -d0c - d2c - d3c;

        const f32 denominator = p0.invMass * glm::dot(d0c, d0c) +
                                p1.invMass * glm::dot(d1c, d1c) +
                                p2.invMass * glm::dot(d2c, d2c) +
                                p3.invMass * glm::dot(d3c, d3c) +
                                bend.compliance * inverseStepSquared;
        if (denominator < 1.0e-12f)
            continue;
        const f32 minusLambda = c / denominator;

        p0.position = x0 - minusLambda * p0.invMass * d0c;
        p1.position = x1 - minusLambda * p1.invMass * d1c;
        p2.position = x2 - minusLambda * p2.invMass * d2c;
        p3.position = x3 - minusLambda * p3.invMass * d3c;
    }
}

void SoftBody::projectAttachments()
{
    for (const LongRangeAttachment& attachment : mAttachments)
    {
        const glm::vec3 anchor = mParticles[attachment.anchor].position;
        Particle& particle = mParticles[attachment.vertex];
        const glm::vec3 delta = particle.position - anchor;
        const f32 squared = glm::dot(delta, delta);
        if (squared > attachment.maxDistance * attachment.maxDistance && squared > kEpsilon)
            particle.position = anchor + delta * (attachment.maxDistance / std::sqrt(squared));
    }
}

// Once per step, not per substep: each particle gets the plane of the
// closest collider surface it could reach this step, and every substep
// projects against that same plane. Stable normals are what make the
// velocity handling below safe - a fresh narrowphase normal per substep
// turns a resting pile into a feedback loop between the constraints and
// the contact push-out.
void SoftBody::determineContactPlanes(f32 dt)
{
    mContactPlanes.assign(mParticles.size(), ContactPlane{});
    if (!mCollisionWorld || dt <= 0.0f || mParticles.empty())
        return;

    const glm::vec3 windPerParticle = mWind / static_cast<f32>(mParticles.size());
    AABB bounds;
    bounds.min = glm::vec3(std::numeric_limits<f32>::max());
    bounds.max = glm::vec3(-std::numeric_limits<f32>::max());
    for (const Particle& particle : mParticles)
    {
        const f32 reach =
            mCollisionMargin + particleTravel(particle, mGravity, windPerParticle, dt);
        bounds.min = glm::min(bounds.min, particle.position - glm::vec3(reach));
        bounds.max = glm::max(bounds.max, particle.position + glm::vec3(reach));
    }
    mCollisionWorld->queryAABB(bounds, mCollisionQuery, mCollisionCandidates);
    if (mCollisionCandidates.empty())
        return;

    const SphereShape particleShape(mCollisionMargin);
    std::vector<ContactManifold> manifolds;
    manifolds.reserve(4);

    for (u32 index = 0; index < mParticles.size(); ++index)
    {
        Particle& particle = mParticles[index];
        if (particle.invMass == 0.0f)
            continue;

        const f32 search = particleTravel(particle, mGravity, windPerParticle, dt);
        glm::mat4 particleTransform(1.0f);
        particleTransform[3] = glm::vec4(particle.position, 1.0f);

        f32 deepest = -std::numeric_limits<f32>::max();
        for (u32 id : mCollisionCandidates)
        {
            const BodyEntry* entry = mCollisionWorld->body(id);
            if (!entry || !entry->body || !entry->shape || !entry->enabled)
                continue;

            manifolds.clear();
            if (entry->shape->type() == ShapeType::Trimesh)
            {
                Narrowphase::convexTrimesh(
                    particleShape, particleTransform,
                    static_cast<const TrimeshShape&>(*entry->shape), entry->body->transform(),
                    manifolds, search);
            }
            else if (entry->shape->type() == ShapeType::Triangle)
            {
                ContactManifold manifold;
                if (Narrowphase::sphereTriangle(
                        particleShape, particleTransform,
                        static_cast<const TriangleShape&>(*entry->shape), entry->body->transform(),
                        manifold, search))
                    manifolds.push_back(manifold);
            }
            else
            {
                ContactManifold manifold;
                if (Narrowphase::collide(particleShape, particleTransform, *entry->shape,
                                         entry->body->transform(), manifold, search))
                    manifolds.push_back(manifold);
            }

            for (const ContactManifold& manifold : manifolds)
            {
                for (u32 i = 0; i < manifold.count; ++i)
                {
                    if (manifold.points[i].penetration <= deepest)
                        continue;
                    deepest = manifold.points[i].penetration;

                    ContactPlane& plane = mContactPlanes[index];
                    // Narrowphase normal runs from its first shape (the
                    // particle) towards the collider; the plane wants the
                    // opposite direction, out of the collider.
                    plane.normal = -manifold.normal;
                    // The particle centre sits margin - penetration from the
                    // surface along the normal; anchor the plane there so a
                    // later position measures its true clearance against it.
                    plane.offset = glm::dot(plane.normal, particle.position) -
                                   (mCollisionMargin - deepest);
                    plane.bodyId = id;
                    plane.friction = entry->friction;
                    plane.restitution = entry->restitution;
                    plane.active = true;
                }
            }
        }
    }
}

void SoftBody::applyContactPlanes(f32 dt)
{
    if (!mCollisionWorld || dt <= 0.0f || mContactPlanes.size() != mParticles.size())
        return;

    const f32 restitutionThreshold = -2.0f * glm::length(mGravity) * dt;
    for (u32 index = 0; index < mParticles.size(); ++index)
    {
        const ContactPlane& plane = mContactPlanes[index];
        Particle& particle = mParticles[index];
        if (!plane.active || particle.invMass == 0.0f)
            continue;

        const f32 projected = plane.offset - glm::dot(plane.normal, particle.position) +
                              mCollisionMargin;
        if (projected <= 0.0f)
            continue;

        // Push-out after updateVelocities() has already run, so it changes
        // pose without injecting velocity - the next substep's predict()
        // rebases previousPosition on the corrected pose.
        particle.position += plane.normal * projected;

        // Deviation from the reference, kept from the old solver: contact
        // velocity is read from the collider so a kinematic body drags and
        // carries the cloth; the reference's simple path assumes a static
        // collider and works on absolute velocity.
        glm::vec3 colliderVelocity(0.0f);
        if (const BodyEntry* entry = mCollisionWorld->body(plane.bodyId))
            if (entry->body)
                colliderVelocity = entry->body->velocityAtPoint(particle.position);

        const glm::vec3 relativeVelocity = particle.velocity - colliderVelocity;
        const f32 normalVelocity = glm::dot(relativeVelocity, plane.normal);
        const glm::vec3 tangent = relativeVelocity - plane.normal * normalVelocity;
        const f32 tangentSpeed = glm::length(tangent);
        if (tangentSpeed > kEpsilon && plane.friction > 0.0f)
            particle.velocity -=
                tangent * glm::min(plane.friction * projected / (tangentSpeed * dt), 1.0f);

        // The whole normal component goes, separating or not - while the
        // particle still reaches the plane, a resting pile's own constraint
        // corrections keep generating small outward velocities, and letting
        // those live is a sheet that jitters and glides instead of settling.
        // Bounce comes back only through restitution, decided on the
        // velocity as it stood before this substep's update.
        particle.velocity -= plane.normal * normalVelocity;
        const f32 previousNormalVelocity =
            glm::dot(particle.previousVelocity - colliderVelocity, plane.normal);
        if (previousNormalVelocity < restitutionThreshold)
            particle.velocity -= plane.restitution * previousNormalVelocity * plane.normal;
    }
}

void SoftBody::updateVelocities(f32 dt)
{
    if (dt <= 0.0f)
        return;
    const f32 inverseStep = 1.0f / dt;
    for (Particle& particle : mParticles)
    {
        particle.previousVelocity = particle.velocity;
        if (particle.invMass == 0.0f)
        {
            particle.velocity = glm::vec3(0.0f);
            continue;
        }
        particle.velocity = (particle.position - particle.previousPosition) * inverseStep;
    }
}

void SoftBody::step(f32 dt, u32 substeps)
{
    if (mParticles.empty() || !(dt > 0.0f))
        return;

    const u32 count = glm::max(substeps, 1u);
    const f32 h = dt / static_cast<f32>(count);
    determineContactPlanes(dt);
    for (u32 i = 0; i < count; ++i)
    {
        predict(h);
        projectDihedralBendConstraints(h);
        projectDistanceConstraints(h);
        projectAttachments();
        updateVelocities(h);
        applyContactPlanes(h);
    }

    // Jolt clamps vertex velocity at the end of a soft-body update. A dense
    // constraint graph can otherwise turn one large positional correction
    // into an extreme velocity for the following frame.
    const f32 maximumSquared = mMaxLinearVelocity * mMaxLinearVelocity;
    for (Particle& particle : mParticles)
    {
        const f32 speedSquared = glm::dot(particle.velocity, particle.velocity);
        if (speedSquared > maximumSquared && speedSquared > kEpsilon)
            particle.velocity *= mMaxLinearVelocity / std::sqrt(speedSquared);
    }
}

f32 SoftBody::worstStretch() const
{
    f32 worst = 1.0f;
    for (const DistanceConstraint& constraint : mConstraints)
    {
        if (constraint.restLength < kEpsilon)
            continue;
        const f32 length = glm::length(mParticles[constraint.b].position -
                                       mParticles[constraint.a].position);
        worst = glm::max(worst, length / constraint.restLength);
    }
    return worst;
}

SoftBody::Contact SoftBody::contact(u32 index) const
{
    Contact result;
    if (index >= mContactPlanes.size())
        return result;
    const ContactPlane& plane = mContactPlanes[index];
    result.active = plane.active;
    result.normal = plane.normal;
    result.bodyId = plane.bodyId;
    return result;
}

} // namespace Radion::Physics
