#include "PCH.h"

#include "CollisionWorld.h"

#include "Collider.h"
#include "GameObject.h"
#include "Octree.h"
#include "Scene.h"
#include "ZenBehaviour.h"
#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"

namespace Radion
{

namespace
{

bool buildConvexManifold(Collider& a, Collider& b, Physics::ContactManifold& out)
{
    const Math::mat4& transformA = a.owner()->globalTransform();
    const Math::mat4& transformB = b.owner()->globalTransform();

    switch (a.shape())
    {
    case ColliderShape::Sphere:
    {
        const Physics::SphereShape shapeA(a.radius());
        switch (b.shape())
        {
        case ColliderShape::Sphere:
            return Physics::Narrowphase::collide(shapeA, transformA,
                                                 Physics::SphereShape(b.radius()), transformB, out);
        case ColliderShape::Box:
            return Physics::Narrowphase::collide(
                shapeA, transformA, Physics::BoxShape(b.halfExtents()), transformB, out);
        case ColliderShape::Capsule:
            return Physics::Narrowphase::collide(
                shapeA, transformA, Physics::CapsuleShape(b.radius(), b.capsuleSegmentHalfHeight()),
                transformB, out);
        case ColliderShape::Mesh:
            return false;
        }
        return false;
    }
    case ColliderShape::Box:
    {
        const Physics::BoxShape shapeA(a.halfExtents());
        switch (b.shape())
        {
        case ColliderShape::Sphere:
            return Physics::Narrowphase::collide(shapeA, transformA,
                                                 Physics::SphereShape(b.radius()), transformB, out);
        case ColliderShape::Box:
            return Physics::Narrowphase::collide(
                shapeA, transformA, Physics::BoxShape(b.halfExtents()), transformB, out);
        case ColliderShape::Capsule:
            return Physics::Narrowphase::collide(
                shapeA, transformA, Physics::CapsuleShape(b.radius(), b.capsuleSegmentHalfHeight()),
                transformB, out);
        case ColliderShape::Mesh:
            return false;
        }
        return false;
    }
    case ColliderShape::Capsule:
    {
        const Physics::CapsuleShape shapeA(a.radius(), a.capsuleSegmentHalfHeight());
        switch (b.shape())
        {
        case ColliderShape::Sphere:
            return Physics::Narrowphase::collide(shapeA, transformA,
                                                 Physics::SphereShape(b.radius()), transformB, out);
        case ColliderShape::Box:
            return Physics::Narrowphase::collide(
                shapeA, transformA, Physics::BoxShape(b.halfExtents()), transformB, out);
        case ColliderShape::Capsule:
            return Physics::Narrowphase::collide(
                shapeA, transformA, Physics::CapsuleShape(b.radius(), b.capsuleSegmentHalfHeight()),
                transformB, out);
        case ColliderShape::Mesh:
            return false;
        }
        return false;
    }
    case ColliderShape::Mesh:
        return false;
    }
    return false;
}

struct SweptHit
{
    bool hit = false;
    f32 t = 1.0f;
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
};

bool solveQuadratic(f32 a, f32 b, f32 c, f32 maxT, f32& t)
{
    if (Math::abs(a) <= 1e-12f)
        return false;

    const f32 discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f)
        return false;

    const f32 sqrtDiscriminant = Math::sqrt(discriminant);
    f32 t0 = (-b - sqrtDiscriminant) / (2.0f * a);
    f32 t1 = (-b + sqrtDiscriminant) / (2.0f * a);
    if (t0 > t1)
        std::swap(t0, t1);

    if (t0 >= 0.0f && t0 <= maxT)
    {
        t = t0;
        return true;
    }
    if (t1 >= 0.0f && t1 <= maxT)
    {
        t = t1;
        return true;
    }
    return false;
}

bool sweepSphereSphere(const Math::vec3& sv, const Math::vec3& dv, f32 srcRadius,
                       const Math::vec3& dstCenter, f32 dstRadius, SweptHit& out)
{
    const Math::vec3 delta = dv - sv;
    const f32 r = srcRadius + dstRadius;

    const Math::vec3 o = sv - dstCenter;
    const f32 a = Math::dot(delta, delta);
    if (a <= 1e-12f)
        return false;

    const f32 b = 2.0f * Math::dot(o, delta);
    const f32 c = Math::dot(o, o) - r * r;
    const f32 d = b * b - 4.0f * a * c;
    if (d < 0.0f)
        return false;

    const f32 sd = Math::sqrt(d);
    const f32 t1 = (-b - sd) / (2.0f * a);
    const f32 t2 = (-b + sd) / (2.0f * a);

    f32 t = t1;
    if (t < 0.0f || t > 1.0f)
        t = t2;
    if (t < 0.0f || t > 1.0f)
        return false;

    const Math::vec3 centerAtHit = sv + delta * t;
    Math::vec3 n = centerAtHit - dstCenter;
    if (Math::dot(n, n) <= 1e-12f)
        n = Math::vec3(0.0f, 1.0f, 0.0f);
    else
        n = Math::normalize(n);

    out.hit = true;
    out.t = t;
    out.normal = n;
    return true;
}

bool sweepSphereAABB(const Math::vec3& sv, const Math::vec3& dv, f32 radius, const AABB& box,
                     SweptHit& out)
{
    if (box.empty())
        return false;

    AABB expanded = box;
    expanded.min -= Math::vec3(radius);
    expanded.max += Math::vec3(radius);

    const Math::vec3 delta = dv - sv;
    const f32 len = Math::length(delta);
    if (len <= 1e-12f)
        return false;

    const Math::vec3 dir = delta / len;
    const Math::vec3 inv(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
    const Math::vec3 t0 = (expanded.min - sv) * inv;
    const Math::vec3 t1 = (expanded.max - sv) * inv;
    const Math::vec3 tmin = Math::min(t0, t1);
    const Math::vec3 tmax = Math::max(t0, t1);
    const f32 enter = Math::max(Math::max(tmin.x, tmin.y), tmin.z);
    const f32 exit = Math::min(Math::min(tmax.x, tmax.y), tmax.z);
    const f32 tDist = (exit >= enter && exit >= 0.0f) ? (enter < 0.0f ? 0.0f : enter) : -1.0f;
    if (tDist < 0.0f)
        return false;

    const f32 t = tDist / len;
    if (t < 0.0f || t > 1.0f)
        return false;

    const Math::vec3 hitPos = sv + delta * t;
    const Math::vec3 c = expanded.center();
    const Math::vec3 local = hitPos - c;

    Math::vec3 n(0.0f);
    const Math::vec3 ext = expanded.extents();
    const f32 ax = Math::abs(local.x / Math::max(ext.x, 1e-6f));
    const f32 ay = Math::abs(local.y / Math::max(ext.y, 1e-6f));
    const f32 az = Math::abs(local.z / Math::max(ext.z, 1e-6f));
    if (ax >= ay && ax >= az)
        n.x = local.x >= 0.0f ? 1.0f : -1.0f;
    else if (ay >= ax && ay >= az)
        n.y = local.y >= 0.0f ? 1.0f : -1.0f;
    else
        n.z = local.z >= 0.0f ? 1.0f : -1.0f;

    out.hit = true;
    out.t = t;
    out.normal = n;
    return true;
}

bool sweepSphereCapsule(const Math::vec3& sv, const Math::vec3& dv, f32 srcRadius,
                        const Math::vec3& capA, const Math::vec3& capB, f32 capRadius, SweptHit& out)
{
    const Math::vec3 delta = dv - sv;
    if (Math::dot(delta, delta) <= 1e-12f)
        return false;

    const Math::vec3 seg = capB - capA;
    const f32 segLen2 = Math::dot(seg, seg);
    const f32 radius = srcRadius + capRadius;

    if (segLen2 <= 1e-12f)
        return sweepSphereSphere(sv, dv, srcRadius, capA, capRadius, out);

    const Math::vec3 axis = seg * Math::inversesqrt(segLen2);
    const Math::vec3 rel = sv - capA;
    const f32 relProj = Math::dot(rel, axis);
    const f32 deltaProj = Math::dot(delta, axis);
    const Math::vec3 relPerp = rel - axis * relProj;
    const Math::vec3 deltaPerp = delta - axis * deltaProj;

    f32 bestT = 1.0f;
    Math::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    bool hit = false;

    f32 tCyl = bestT;
    if (solveQuadratic(Math::dot(deltaPerp, deltaPerp), 2.0f * Math::dot(relPerp, deltaPerp),
                       Math::dot(relPerp, relPerp) - radius * radius, bestT, tCyl))
    {
        const f32 proj = relProj + deltaProj * tCyl;
        const f32 segLen = Math::sqrt(segLen2);
        if (proj >= 0.0f && proj <= segLen)
        {
            bestT = tCyl;
            const Math::vec3 center = sv + delta * bestT;
            const Math::vec3 closest = capA + axis * proj;
            const Math::vec3 n = center - closest;
            bestNormal = Math::dot(n, n) > 1e-12f ? Math::normalize(n) : Math::vec3(0.0f, 1.0f, 0.0f);
            hit = true;
        }
    }

    SweptHit capHit;
    if (sweepSphereSphere(sv, dv, srcRadius, capA, capRadius, capHit) && capHit.t <= bestT)
    {
        bestT = capHit.t;
        bestNormal = capHit.normal;
        hit = true;
    }
    if (sweepSphereSphere(sv, dv, srcRadius, capB, capRadius, capHit) && capHit.t <= bestT)
    {
        bestT = capHit.t;
        bestNormal = capHit.normal;
        hit = true;
    }

    if (!hit)
        return false;

    out.hit = true;
    out.t = bestT;
    out.normal = bestNormal;
    return true;
}

bool sweepAgainstCollider(const Math::vec3& sv, const Math::vec3& dv, f32 radius,
                         const Collider& target, SweptHit& hit)
{
    switch (target.shape())
    {
    case ColliderShape::Sphere:
    {
        const Math::vec3 center(target.owner()->globalTransform()[3]);
        return sweepSphereSphere(sv, dv, radius, center, target.radius(), hit);
    }
    case ColliderShape::Box:
        return sweepSphereAABB(sv, dv, radius, target.worldBounds(), hit);
    case ColliderShape::Capsule:
    {
        Math::vec3 lower, upper;
        Physics::CapsuleShape(target.radius(), target.capsuleSegmentHalfHeight())
            .segment(target.owner()->globalTransform(), lower, upper);
        return sweepSphereCapsule(sv, dv, radius, lower, upper, target.radius(), hit);
    }
    case ColliderShape::Mesh:
    {
        if (!target.mesh())
            return false;
        TriangleOctree::SweepHit meshHit;
        if (!target.mesh()->sweepSphere(sv, radius, dv - sv, meshHit))
            return false;
        hit.hit = true;
        hit.t = meshHit.t;
        hit.normal = meshHit.normal;
        return true;
    }
    }
    return false;
}

} // namespace

// Sphere vs Mesh only - TriangleOctree has no box/capsule sweep to reuse.
void CollisionWorld::collideMeshPair(Collider& mesh, Collider& other)
{
    if (other.shape() != ColliderShape::Sphere || !mesh.mesh() || !other.owner())
        return;

    TriangleOctree::SweepHit hit;
    const Math::vec3 center(other.owner()->globalTransform()[3]);
    if (!mesh.mesh()->sweepSphere(center, other.radius(), Math::vec3(0.0f), hit))
        return;

    other.addContact(&mesh, hit.normal, hit.point);
    mesh.addContact(&other, -hit.normal, hit.point);
    notifyCollision(other, mesh);
    notifyCollision(mesh, other);
}

void CollisionWorld::notifyCollision(Collider& self, Collider& other)
{
    GameObject* owner = self.owner();
    if (!owner)
        return;
    if (ZenBehaviour* behaviour = owner->findComponent<ZenBehaviour>())
        behaviour->onCollision(other.owner());
}

void CollisionWorld::collidePair(Collider& a, Collider& b)
{
    if (a.shape() == ColliderShape::Mesh || b.shape() == ColliderShape::Mesh)
    {
        Collider& mesh = a.shape() == ColliderShape::Mesh ? a : b;
        Collider& other = a.shape() == ColliderShape::Mesh ? b : a;
        collideMeshPair(mesh, other);
        return;
    }

    if (!a.owner() || !b.owner())
        return;

    Physics::ContactManifold manifold;
    if (!buildConvexManifold(a, b, manifold) || manifold.count == 0)
        return;

    const Math::vec3 point = manifold.points[0].position; // normal: A towards B (Narrowphase.h)
    a.addContact(&b, -manifold.normal, point);
    b.addContact(&a, manifold.normal, point);
    notifyCollision(a, b);
    notifyCollision(b, a);
}

CollisionWorld::CollisionWorld()
{
}

void CollisionWorld::initialize(Scene& scene)
{
    mScene = &scene;
}

s32 CollisionWorld::findPair(u32 typeA, u32 typeB) const
{
    for (usize i = 0; i < mPairs.size(); ++i)
    {
        const Pair& pair = mPairs[i];
        if ((pair.typeA == typeA && pair.typeB == typeB) ||
            (pair.typeA == typeB && pair.typeB == typeA))
            return static_cast<s32>(i);
    }
    return -1;
}

void CollisionWorld::enable(u32 typeA, u32 typeB, CollisionResponse response)
{
    const s32 index = findPair(typeA, typeB);
    if (index >= 0)
    {
        mPairs[static_cast<usize>(index)].response = response;
        return;
    }
    Pair pair;
    pair.typeA = typeA;
    pair.typeB = typeB;
    pair.response = response;
    mPairs.push_back(pair);
}

void CollisionWorld::disable(u32 typeA, u32 typeB)
{
    const s32 index = findPair(typeA, typeB);
    if (index < 0)
        return;
    mPairs.erase(mPairs.begin() + index);
}

bool CollisionWorld::enabled(u32 typeA, u32 typeB) const
{
    return findPair(typeA, typeB) >= 0;
}

CollisionResponse CollisionWorld::response(u32 typeA, u32 typeB) const
{
    const s32 index = findPair(typeA, typeB);
    return index >= 0 ? mPairs[static_cast<usize>(index)].response : CollisionResponse::None;
}

void CollisionWorld::step()
{
    if (!mScene)
        return;

    const std::vector<Collider*>& colliders = mScene->colliders();
    mStepColliders.clear();
    mStepBounds.clear();
    mStepColliders.reserve(colliders.size());
    mStepBounds.reserve(colliders.size());
    for (Collider* collider : colliders)
    {
        collider->clearContacts();
        GameObject* owner = collider->owner();
        if (!collider->active() || !owner || !owner->isActiveInHierarchy() || owner->disposed())
            continue;
        mStepColliders.push_back(collider);
        mStepBounds.push_back(collider->worldBounds());
    }

    for (usize i = 0; i < mStepColliders.size(); ++i)
    {
        Collider* colliderA = mStepColliders[i];

        for (usize j = i + 1; j < mStepColliders.size(); ++j)
        {
            Collider* colliderB = mStepColliders[j];

            if (findPair(colliderA->type(), colliderB->type()) < 0)
                continue;
            if (!mStepBounds[i].intersects(mStepBounds[j]))
                continue;

            collidePair(*colliderA, *colliderB);
        }
    }
}

CollisionWorld::MoveConfig& CollisionWorld::moveConfig()
{
    return mMoveConfig;
}

const CollisionWorld::MoveConfig& CollisionWorld::moveConfig() const
{
    return mMoveConfig;
}

CollisionWorld::MoveResult CollisionWorld::moveSphere(const Math::vec3& from, const Math::vec3& to,
                                                       f32 radius, u32 movingType,
                                                       u32 maxHits) const
{
    MoveResult out;
    out.position = to;

    if (!mScene)
        return out;

    const f32 epsilon = mMoveConfig.epsilon;
    const f32 zeroEpsilon = mMoveConfig.zeroEpsilon;
    const std::vector<Collider*>& colliders = mScene->colliders();

    Math::vec3 sv = from;
    Math::vec3 dv = to;
    Math::vec3 safe = sv;

    f32 td = Math::length(dv - sv);
    f32 td_xz = Math::length(Math::vec2(dv.x - sv.x, dv.z - sv.z));

    u32 n_hit = 0;
    Plane planes[2];

    for (;;)
    {
        if (out.hitCount >= maxHits)
            break;

        bool hasHit = false;
        SweptHit best;
        best.t = 1.0f;
        CollisionResponse bestResponse = CollisionResponse::None;

        for (Collider* target : colliders)
        {
            GameObject* targetOwner = target->owner();
            if (!target->active() || !targetOwner || !targetOwner->isActiveInHierarchy() ||
                targetOwner->disposed())
                continue;

            const s32 pairIndex = findPair(movingType, target->type());
            if (pairIndex < 0)
                continue;

            SweptHit hit;
            if (!sweepAgainstCollider(sv, dv, radius, *target, hit))
                continue;

            if (!hasHit || hit.t < best.t)
            {
                hasHit = true;
                best = hit;
                bestResponse = mPairs[static_cast<usize>(pairIndex)].response;
            }
        }

        if (!hasHit)
            break;

        ++out.hitCount;
        out.collided = true;
        out.lastNormal = best.normal;

        const Math::vec3 delta = dv - sv;
        const f32 t = Clamp(best.t, 0.0f, 1.0f);
        const Math::vec3 hitPos = sv + delta * t;

        Plane collPlane;
        collPlane.normal = Math::normalize(best.normal);
        collPlane.d = -Math::dot(collPlane.normal, hitPos) - epsilon;

        if (bestResponse == CollisionResponse::Stop || bestResponse == CollisionResponse::None)
        {
            dv = sv;
            break;
        }

        for (u32 i = 0; i < n_hit; ++i)
        {
            if (Math::dot(planes[i].normal, collPlane.normal) > 0.98f ||
                (planes[i].normal.y > 0.45f && collPlane.normal.y > 0.45f))
            {
                n_hit = 0;
                break;
            }
        }

        Math::vec3 nv = dv - best.normal * Math::dot(dv - hitPos, best.normal);

        if (n_hit == 0)
        {
            dv = nv;
        }
        else if (n_hit == 1)
        {
            if (planes[0].distance(nv) >= 0.0f)
            {
                dv = nv;
                n_hit = 0;
            }
            else
            {
                const f32 ndot = Math::dot(planes[0].normal, collPlane.normal);
                if (Math::abs(ndot) < 1.0f - zeroEpsilon)
                {
                    const Math::vec3 crease = Math::cross(planes[0].normal, collPlane.normal);
                    if (Math::dot(crease, crease) > 1e-12f)
                    {
                        const Math::vec3 dir = Math::normalize(crease);
                        dv = hitPos + dir * Math::dot(dv - hitPos, dir);
                    }
                    else
                    {
                        dv = sv;
                        break;
                    }
                }
                else
                {
                    dv = sv;
                    break;
                }
            }
        }
        else
        {
            if (planes[0].distance(nv) >= 0.0f && planes[1].distance(nv) >= 0.0f)
            {
                dv = nv;
                n_hit = 0;
            }
            else
            {
                dv = sv;
                break;
            }
        }

        Math::vec3 dd = dv - sv;
        if (Math::dot(dd, delta) <= 0.0f)
        {
            dv = sv;
            break;
        }

        if (bestResponse == CollisionResponse::Slide)
        {
            const f32 d = Math::length(dd);
            if (d <= zeroEpsilon)
            {
                dv = sv;
                break;
            }
            if (d > td)
                dd *= td / d;
        }
        else if (bestResponse == CollisionResponse::SlideXZ)
        {
            const f32 d = Math::length(Math::vec2(dd.x, dd.z));
            if (d <= zeroEpsilon)
            {
                dv = sv;
                break;
            }
            if (d > td_xz)
                dd *= td_xz / d;
        }

        sv += best.normal * epsilon;
        safe = sv;
        dv = sv + dd;

        if (n_hit < 2)
            planes[n_hit++] = collPlane;

        td = Math::length(dv - sv);
        td_xz = Math::length(Math::vec2(dv.x - sv.x, dv.z - sv.z));
    }

    out.position = out.hitCount >= maxHits ? safe : dv;
    return out;
}

} // namespace Radion
