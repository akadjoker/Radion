#include "PCH.h"

#include "dynamics/PhysicsWorld.h"

#include "DebugDraw3D.h"
#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"
#include "dynamics/Joint.h"

#include <cassert>

namespace Radion::Physics
{

namespace
{
// How close a new contact point has to be to a cached one to count as the
// same point. Too tight and the impulse is thrown away every step, which is
// warm starting not happening at all; too loose and a point that slid across
// a face inherits an impulse meant for somewhere else.
constexpr f32 kMatchDistance = 0.02f;

bool finiteVector(const Math::vec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

f32 radialWeight(const Math::vec3& centre, const Math::vec3& position, f32 radius,
                 Math::vec3* direction = nullptr)
{
    // Same simple linear attenuation used by b2World_Explode. Box2D scales
    // by exposed perimeter; this lightweight 3D version deliberately applies
    // a body-level impulse/force until projected area is a real requirement.
    const Math::vec3 offset = position - centre;
    const f32 distanceSquared = Math::dot(offset, offset);
    if (distanceSquared >= radius * radius)
        return 0.0f;

    const f32 distance = std::sqrt(distanceSquared);
    if (direction)
        *direction = distance > 1.0e-6f ? offset / distance : Math::vec3(0.0f, 1.0f, 0.0f);
    return 1.0f - distance / radius;
}
} // namespace

PhysicsWorld::PhysicsWorld()
{
}

void PhysicsWorld::addJoint(Joint* joint)
{
    if (!joint || !joint->bodyA() || !joint->bodyB())
        return;
    if (joint->bodyA() == joint->bodyB() && !joint->singleBody())
        return;
    if (std::find(mJoints.begin(), mJoints.end(), joint) != mJoints.end())
        return;
    mJoints.push_back(joint);
    joint->bodyA()->setAwake(true);
    joint->bodyB()->setAwake(true);
}

void PhysicsWorld::removeJoint(Joint* joint)
{
    const auto found = std::find(mJoints.begin(), mJoints.end(), joint);
    if (found == mJoints.end())
        return;
    *found = mJoints.back();
    mJoints.pop_back();
}

u64 PhysicsWorld::pairKey(u32 a, u32 b)
{
    const u32 low = a < b ? a : b;
    const u32 high = a < b ? b : a;
    return (static_cast<u64>(low) << 32) | high;
}

u32 PhysicsWorld::addBody(const BodyEntry& entry)
{
    if (!entry.body || !entry.shape)
        return InvalidIndex;
    if (mClearPending)
    {
        Log::error("PhysicsWorld: cannot add a body after clear() was requested during a step");
        return InvalidIndex;
    }
    if (entry.body->isDynamic() &&
        (entry.shape->type() == ShapeType::Trimesh || entry.shape->type() == ShapeType::Plane))
    {
        Log::error("PhysicsWorld: a dynamic body cannot use a static collision shape");
        return InvalidIndex;
    }
    // Sleep becomes this world's decision the moment a body joins it - see
    // propagateSleep() for why it cannot be the body's own.
    entry.body->setSleepDeferred(true);
    const u32 id = allocateId();
    if (mStepping)
    {
        mPendingAdds.push_back({id, entry});
        return id;
    }
    insertBody(id, entry);
    return id;
}

u32 PhysicsWorld::allocateId()
{
    if (!mFreeIds.empty())
    {
        const u32 id = mFreeIds.back();
        mFreeIds.pop_back();
        mBodyGenerations[id] = mNextBodyGeneration++;
        return id;
    }

    const u32 id = static_cast<u32>(mIdToSlot.size());
    mIdToSlot.push_back(InvalidIndex);
    mBodyGenerations.push_back(mNextBodyGeneration++);
    return id;
}

u32 PhysicsWorld::slotForId(u32 id) const
{
    return id < mIdToSlot.size() ? mIdToSlot[id] : InvalidIndex;
}

void PhysicsWorld::insertBody(u32 id, const BodyEntry& entry)
{
    assert(id < mIdToSlot.size());
    assert(mIdToSlot[id] == InvalidIndex);
    mIdToSlot[id] = static_cast<u32>(mBodies.size());
    mBodies.push_back(entry);
    mBodyIds.push_back(id);
    if (entry.body->bodyType() == BodyType::Static)
        mStaticBroadphaseDirty = true;
}

void PhysicsWorld::removeBody(u32 id)
{
    if (id >= mBodyGenerations.size() || mBodyGenerations[id] == 0)
        return;

    if (mStepping)
    {
        // An add requested earlier in this callback has no slot yet and can
        // be cancelled without ever exposing it to the simulation.
        for (auto it = mPendingAdds.begin(); it != mPendingAdds.end(); ++it)
            if (it->id == id)
            {
                it->entry.body->setSleepDeferred(false);
                mPendingAdds.erase(it);
                mBodyGenerations[id] = 0;
                mFreeIds.push_back(id);
                return;
            }
        if (std::find(mPendingRemovals.begin(), mPendingRemovals.end(), id) ==
            mPendingRemovals.end())
            mPendingRemovals.push_back(id);
        return;
    }
    removeBodyImmediate(id);
}

void PhysicsWorld::removeBodyImmediate(u32 id)
{
    const u32 slot = slotForId(id);
    if (slot == InvalidIndex)
        return;

    RigidBody* removedBody = mBodies[slot].body;
    if (removedBody->bodyType() == BodyType::Static)
        mStaticBroadphaseDirty = true;
    mJoints.erase(std::remove_if(mJoints.begin(), mJoints.end(),
                                 [removedBody](Joint* joint)
                                 {
                                     return !joint || joint->bodyA() == removedBody ||
                                            joint->bodyB() == removedBody;
                                 }),
                  mJoints.end());
    mBodies[slot].body->setSleepDeferred(false);
    if (mDispatchingEvents)
        mRetiredIds.push_back(id);
    else
        mFreeIds.push_back(id);
    mBodyGenerations[id] = 0;
    mIdToSlot[id] = InvalidIndex;

    const u32 lastSlot = static_cast<u32>(mBodies.size() - 1);
    if (slot != lastSlot)
    {
        mBodies[slot] = mBodies[lastSlot];
        mBodyIds[slot] = mBodyIds[lastSlot];
        mIdToSlot[mBodyIds[slot]] = slot;
    }
    mBodies.pop_back();
    mBodyIds.pop_back();

    // Every cached pair naming it goes too, or the next body handed this id
    // inherits impulses from a body that no longer exists.
    for (auto it = mCache.begin(); it != mCache.end();)
    {
        const u32 a = static_cast<u32>(it->first >> 32);
        const u32 b = static_cast<u32>(it->first & 0xFFFFFFFFull);
        if (a == id || b == id)
            it = mCache.erase(it);
        else
            ++it;
    }
    mContacts.erase(std::remove_if(mContacts.begin(), mContacts.end(),
                                   [id](const Contact& contact)
                                   { return contact.idA == id || contact.idB == id; }),
                    mContacts.end());
}

void PhysicsWorld::clear()
{
    if (mStepping)
    {
        mClearPending = true;
        return;
    }
    clearImmediate();
}

void PhysicsWorld::clearImmediate()
{
    for (BodyEntry& entry : mBodies)
        if (entry.body)
            entry.body->setSleepDeferred(false);
    for (PendingAdd& pending : mPendingAdds)
        if (pending.entry.body)
            pending.entry.body->setSleepDeferred(false);
    mBodies.clear();
    mBodyIds.clear();
    mBodyGenerations.clear();
    mIdToSlot.clear();
    mFreeIds.clear();
    mRetiredIds.clear();
    mPendingAdds.clear();
    mPendingRemovals.clear();
    mCache.clear();
    mContacts.clear();
    mJoints.clear();
    mStaticBroadphase.clear();
    mStaticBounds.clear();
    mStaticIds.clear();
    mStaticBroadphaseDirty = true;
    mEventQueue.clear();
    mPairs.clear();
    mAccumulator = 0.0f;
    mStepIndex = 0;
    mClearPending = false;
}

void PhysicsWorld::flushPendingMutations()
{
    if (mClearPending)
    {
        clearImmediate();
        return;
    }
    for (const PendingAdd& pending : mPendingAdds)
        insertBody(pending.id, pending.entry);
    mPendingAdds.clear();
    for (u32 id : mPendingRemovals)
        removeBodyImmediate(id);
    mPendingRemovals.clear();
}

BodyEntry* PhysicsWorld::body(u32 id)
{
    const u32 slot = slotForId(id);
    return slot != InvalidIndex ? &mBodies[slot] : nullptr;
}

const BodyEntry* PhysicsWorld::body(u32 id) const
{
    const u32 slot = slotForId(id);
    return slot != InvalidIndex ? &mBodies[slot] : nullptr;
}

BodyHandle PhysicsWorld::bodyHandle(u32 id) const
{
    if (id >= mBodyGenerations.size() || mBodyGenerations[id] == 0)
        return {};
    return {id, mBodyGenerations[id]};
}

BodyEntry* PhysicsWorld::body(BodyHandle handle)
{
    if (!handle.valid() || handle.id >= mBodyGenerations.size() ||
        mBodyGenerations[handle.id] != handle.generation)
        return nullptr;
    return body(handle.id);
}

const BodyEntry* PhysicsWorld::body(BodyHandle handle) const
{
    if (!handle.valid() || handle.id >= mBodyGenerations.size() ||
        mBodyGenerations[handle.id] != handle.generation)
        return nullptr;
    return body(handle.id);
}

void PhysicsWorld::setGravity(const Math::vec3& gravity)
{
    mGravity = gravity;
}

void PhysicsWorld::setFixedStep(f32 seconds)
{
    if (seconds > 0.0f && std::isfinite(seconds))
        mFixedStep = seconds;
}

void PhysicsWorld::setEventCallback(EventCallback callback, void* userData)
{
    mEventCallback = callback;
    mEventUserData = userData;
}

void PhysicsWorld::setStepCallback(StepCallback callback, void* userData)
{
    mStepCallback = callback;
    mStepUserData = userData;
}

void PhysicsWorld::setContactPersistence(u32 steps)
{
    mContactPersistence = Math::max(steps, 1u);
}

void PhysicsWorld::setContactMargin(f32 margin)
{
    mContactMargin = Math::max(margin, 0.0f);
    mStaticBroadphaseDirty = true;
}

void PhysicsWorld::markStaticBroadphaseDirty()
{
    mStaticBroadphaseDirty = true;
}

void PhysicsWorld::rebuildStaticBroadphase()
{
    mStaticBounds.clear();
    mStaticIds.clear();
    mStaticBounds.reserve(mBodies.size());
    mStaticIds.reserve(mBodies.size());
    for (u32 index = 0; index < mBodies.size(); ++index)
    {
        const BodyEntry& entry = mBodies[index];
        if (!entry.body || !entry.shape || !entry.enabled ||
            entry.body->bodyType() != BodyType::Static)
            continue;
        AABB bounds = entry.shape->bounds(entry.body->transform());
        bounds.min -= Math::vec3(mContactMargin);
        bounds.max += Math::vec3(mContactMargin);
        mStaticBounds.push_back(bounds);
        mStaticIds.push_back(mBodyIds[index]);
    }
    mStaticBroadphase.build(mStaticBounds.data(), static_cast<u32>(mStaticBounds.size()));
    mStaticBroadphaseDirty = false;
}

void PhysicsWorld::warmStartFromCache(u32 a, u32 b, ContactManifold& manifold)
{
    const auto found = mCache.find(pairKey(a, b));
    if (found == mCache.end())
        return;
    const CachedPair& cached = found->second;
    for (u32 i = 0; i < manifold.count; ++i)
    {
        ContactPoint& point = manifold.points[i];
        // Matched by position, not by an index: the narrowphase can emit the
        // same patch in a different order from one step to the next, and
        // carrying impulses by slot would then swap them around the face.
        f32 best = kMatchDistance;
        const CachedPoint* match = nullptr;
        for (u32 j = 0; j < cached.count; ++j)
        {
            const f32 distance = Math::length(point.position - cached.points[j].position);
            if (distance < best)
            {
                best = distance;
                match = &cached.points[j];
            }
        }
        if (!match)
            continue;
        point.normalImpulse = match->normalImpulse;
        point.tangentImpulse[0] = match->tangentImpulse[0];
        point.tangentImpulse[1] = match->tangentImpulse[1];
    }
}

void PhysicsWorld::storeInCache(u32 a, u32 b, const ContactManifold& manifold)
{
    CachedPair& cached = mCache[pairKey(a, b)];
    // The first manifold of this step starts the list; the rest of the pair's
    // manifolds append to it rather than replacing what came before, or only
    // the last triangle a body rests on keeps its impulses.
    if (cached.lastStep != mStepIndex)
        cached.count = 0;
    for (u32 i = 0; i < manifold.count && cached.count < CachedPair::MaxPoints; ++i)
    {
        CachedPoint& point = cached.points[cached.count++];
        point.position = manifold.points[i].position;
        point.normalImpulse = manifold.points[i].normalImpulse;
        point.tangentImpulse[0] = manifold.points[i].tangentImpulse[0];
        point.tangentImpulse[1] = manifold.points[i].tangentImpulse[1];
    }
    cached.lastStep = mStepIndex;
}

void PhysicsWorld::emitExits()
{
    for (auto it = mCache.begin(); it != mCache.end();)
    {
        if (mStepIndex - it->second.lastStep < mContactPersistence)
        {
            ++it;
            continue;
        }
        // Not touched this step, so the pair stopped touching. Reported once
        // and then dropped, which is also what stops the cache growing with
        // every pair that ever met.
        if (it->second.reported)
        {
            ContactEventInfo info;
            info.bodyA = static_cast<u32>(it->first >> 32);
            info.bodyB = static_cast<u32>(it->first & 0xFFFFFFFFull);
            info.event = ContactEvent::Exit;
            mEventQueue.push_back(info);
        }
        it = mCache.erase(it);
    }
}

u32 PhysicsWorld::islandRoot(u32 index)
{
    while (mIslandParent[index] != index)
    {
        // Path halving: every lookup shortens the chain it walked, so a long
        // stack does not pay for its own depth on every query.
        mIslandParent[index] = mIslandParent[mIslandParent[index]];
        index = mIslandParent[index];
    }
    return index;
}

void PhysicsWorld::propagateSleep()
{
    const usize count = mBodies.size();
    mIslandParent.resize(count);
    mIslandAwake.assign(count, 0);
    for (usize i = 0; i < count; ++i)
        mIslandParent[i] = static_cast<u32>(i);

    // Static and kinematic bodies are deliberately left out. They never sleep
    // and never wake, and joining them would put every dynamic body resting
    // on the same floor into one island - which would mean a box dropped at
    // one end of the level keeping a stack awake at the other.
    for (const Contact& contact : mContacts)
    {
        if (!contact.a->isDynamic() || !contact.b->isDynamic())
            continue;
        const u32 slotA = slotForId(contact.idA);
        const u32 slotB = slotForId(contact.idB);
        if (slotA == InvalidIndex || slotB == InvalidIndex)
            continue;
        const u32 rootA = islandRoot(slotA);
        const u32 rootB = islandRoot(slotB);
        if (rootA != rootB)
            mIslandParent[rootB] = rootA;
    }

    // An island sleeps only when EVERY body in it has gone quiet. One still
    // moving keeps the whole island awake, which is what stops a box
    // dropping out of a stack that has not finished settling.
    for (usize i = 0; i < count; ++i)
    {
        const BodyEntry& entry = mBodies[i];
        if (!entry.body || !entry.enabled || !entry.body->isDynamic())
            continue;
        if (!entry.body->canSleep() || entry.body->motion() >= entry.body->sleepEpsilon())
            mIslandAwake[islandRoot(static_cast<u32>(i))] = 1;
    }

    for (usize i = 0; i < count; ++i)
    {
        BodyEntry& entry = mBodies[i];
        if (!entry.body || !entry.enabled || !entry.body->isDynamic())
            continue;
        const bool shouldBeAwake = mIslandAwake[islandRoot(static_cast<u32>(i))] != 0;
        if (shouldBeAwake != entry.body->awake())
            entry.body->setAwake(shouldBeAwake);
    }
}

void PhysicsWorld::step(f32 duration)
{
    if (duration <= 0.0f || !std::isfinite(duration))
        return;
    if (mStepping || mDispatchingEvents)
    {
        Log::error("PhysicsWorld: recursive step() is not supported");
        return;
    }
    mStepping = true;
    ++mStepIndex;

    // Gravity is set on every dynamic body rather than added as a force, so
    // it reaches them whatever their mass - and so a body the caller gave its
    // own acceleration keeps it only until the world overwrites it, which is
    // the honest behaviour for a world that owns gravity.
    for (BodyEntry& entry : mBodies)
        if (entry.body && entry.enabled && entry.body->isDynamic())
        {
            entry.body->setAcceleration(mGravity);
            entry.body->integrateForces(duration);
        }

    if (mStaticBroadphaseDirty)
        rebuildStaticBroadphase();
    mDynamicBroadphase.clear();
    mDynamicBroadphase.reserve(mBodies.size());
    mDynamicProxies.clear();
    mDynamicProxies.reserve(mBodies.size());
    for (u32 i = 0; i < mBodies.size(); ++i)
    {
        const BodyEntry& entry = mBodies[i];
        if (!entry.body || !entry.shape || !entry.enabled ||
            entry.body->bodyType() == BodyType::Static)
            continue;
        BroadphaseProxy proxy;
        proxy.id = mBodyIds[i];
        proxy.filter = entry.filter;
        proxy.movable = true;
        proxy.bounds = entry.shape->bounds(entry.body->transform());
        // Grown by the contact margin, or the broadphase throws away exactly
        // the pairs the margin exists to keep: a body resting on a surface
        // has its AABB ending where the other one starts, they do not
        // overlap, and the narrowphase is never even asked.
        proxy.bounds.min -= Math::vec3(mContactMargin);
        proxy.bounds.max += Math::vec3(mContactMargin);
        mDynamicBroadphase.add(proxy);
        mDynamicProxies.push_back(proxy);
    }
    mPairs.clear();
    for (const BroadphaseProxy& dynamic : mDynamicProxies)
    {
        mStaticBroadphase.queryCandidates(dynamic.bounds, mStaticCandidates);
        for (u32 candidate : mStaticCandidates)
        {
            const AABB& staticBounds = mStaticBroadphase.itemBounds(candidate);
            const u32 staticId = mStaticIds[candidate];
            const u32 staticSlot = slotForId(staticId);
            if (staticSlot == InvalidIndex)
                continue;
            const BodyEntry& staticEntry = mBodies[staticSlot];
            if (!shouldCollide(dynamic.filter, staticEntry.filter) ||
                !Broadphase::overlaps(dynamic.bounds, staticBounds))
                continue;
            mPairs.push_back({Math::min(dynamic.id, staticId), Math::max(dynamic.id, staticId)});
        }
    }
    mDynamicBroadphase.findPairs(mDynamicPairs);
    mPairs.insert(mPairs.end(), mDynamicPairs.begin(), mDynamicPairs.end());

    mContacts.clear();
    mContacts.reserve(mPairs.size());
    for (const BroadphasePair& pair : mPairs)
    {
        const u32 slotA = slotForId(pair.a);
        const u32 slotB = slotForId(pair.b);
        if (slotA == InvalidIndex || slotB == InvalidIndex)
            continue;
        BodyEntry& a = mBodies[slotA];
        BodyEntry& b = mBodies[slotB];

        // A trimesh answers with one manifold per triangle touched, so the
        // single-manifold path cannot serve it. Both go through the same
        // vector below and become one Contact each.
        mManifolds.clear();
        const bool aIsMesh = a.shape->type() == ShapeType::Trimesh;
        const bool bIsMesh = b.shape->type() == ShapeType::Trimesh;
        if (aIsMesh && bIsMesh)
            continue; // two static meshes never move; nothing to solve
        if (aIsMesh || bIsMesh)
        {
            const BodyEntry& convex = aIsMesh ? b : a;
            const BodyEntry& mesh = aIsMesh ? a : b;
            if (!Narrowphase::convexTrimesh(*convex.shape, convex.body->transform(),
                                            static_cast<const TrimeshShape&>(*mesh.shape),
                                            mesh.body->transform(), mManifolds, mContactMargin))
                continue;
            // convexTrimesh() reports convex-to-mesh. When the mesh is body A
            // the contact's normal has to run A to B like every other pair.
            if (aIsMesh)
                for (ContactManifold& flipped : mManifolds)
                {
                    flipped.normal = -flipped.normal;
                    flipped.buildTangents();
                }
        }
        else
        {
            ContactManifold manifold;
            if (!Narrowphase::collide(*a.shape, a.body->transform(), *b.shape, b.body->transform(),
                                      manifold, mContactMargin))
                continue;
            mManifolds.push_back(manifold);
        }

        const auto existing = mCache.find(pairKey(pair.a, pair.b));
        const bool isNew = existing == mCache.end();

        ContactManifold& manifold = mManifolds[0];
        for (ContactManifold& current : mManifolds)
        {
            warmStartFromCache(pair.a, pair.b, current);

            Contact contact;
            contact.a = a.body;
            contact.b = b.body;
            contact.idA = pair.a;
            contact.idB = pair.b;
            contact.manifold = current;
            // Combined the usual way: the geometric mean for friction, the
            // larger for restitution, so one bouncy body is enough to bounce.
            contact.friction = std::sqrt(a.friction * b.friction);
            contact.restitution = Math::max(a.restitution, b.restitution);
            mContacts.push_back(contact);
        }

        if (isNew)
        {
            // A new contact is the one moment a sleeping body has to wake -
            // the solver deliberately does not, because a resting stack has
            // contacts every single step.
            a.body->setAwake(true);
            b.body->setAwake(true);
        }
        ContactEventInfo info;
        info.bodyA = pair.a;
        info.bodyB = pair.b;
        info.event = isNew ? ContactEvent::Enter : ContactEvent::Stay;
        info.normal = manifold.normal;
        info.point = manifold.points[0].position;
        info.penetration = manifold.points[0].penetration;
        mEventQueue.push_back(info);
        CachedPair& cached = mCache[pairKey(pair.a, pair.b)];
        cached.lastStep = mStepIndex;
        cached.reported = true;
    }

    mSolver.solve(mContacts.data(), static_cast<u32>(mContacts.size()), mJoints.data(),
                  static_cast<u32>(mJoints.size()), duration);

    // Stored after solving, so what carries into the next step is the impulse
    // the solver settled on rather than the one it started from.
    for (const Contact& contact : mContacts)
        storeInCache(contact.idA, contact.idB, contact.manifold);

    emitExits();

    for (BodyEntry& entry : mBodies)
        if (entry.body && entry.enabled)
            entry.body->integrateVelocity(duration);

    // Actions - a vehicle, anything else that reaches into bodies each step -
    // run here, once positions for this step are final and before sleep
    // state is decided from them.
    if (mStepCallback)
        mStepCallback(duration, mStepUserData);

    // After integrating, because the motion average this reads is what
    // integrate() has just updated. Bodies added to this world have their own
    // sleep deferred, so nothing has dropped out on its own in the meantime.
    propagateSleep();
    mStepping = false;
    flushPendingMutations();
    dispatchEvents();
}

void PhysicsWorld::dispatchEvents()
{
    if (!mEventCallback || mEventQueue.empty())
    {
        mEventQueue.clear();
        return;
    }

    // Callbacks are deliberately outside the simulation lock. The world is
    // non-owning, so removeBody() must return only after the solver has
    // released every pointer to a body the owner may destroy immediately.
    std::vector<ContactEventInfo> events;
    events.swap(mEventQueue);
    mDispatchingEvents = true;
    for (const ContactEventInfo& info : events)
        if (mEventCallback)
            mEventCallback(info, mEventUserData);
    mDispatchingEvents = false;
    mFreeIds.insert(mFreeIds.end(), mRetiredIds.begin(), mRetiredIds.end());
    mRetiredIds.clear();
}

void PhysicsWorld::update(f32 deltaTime)
{
    if (deltaTime <= 0.0f || !std::isfinite(deltaTime))
        return;
    mAccumulator += deltaTime;
    const f32 budget = mFixedStep * static_cast<f32>(mMaxStepsPerUpdate);
    if (mAccumulator > budget)
        mAccumulator = budget;
    while (mAccumulator >= mFixedStep)
    {
        step(mFixedStep);
        mAccumulator -= mFixedStep;
    }
}

void PhysicsWorld::debugDraw() const
{
    for (const BodyEntry& entry : mBodies)
    {
        if (!entry.body || !entry.shape || !entry.enabled)
            continue;
        // Hue says which shape it is, brightness says what the simulation is
        // doing with it. Both matter and they do not compete: telling a
        // capsule from a box at a glance is how a wrong collider is spotted,
        // and a body that never dims is one that never settled.
        Color color = Color::Gray;
        if (entry.body->bodyType() == BodyType::Kinematic)
            color = Color(230, 200, 60, 255);
        else if (entry.body->isDynamic())
        {
            switch (entry.shape->type())
            {
            case ShapeType::Sphere:  color = Color(80, 220, 200, 255); break;
            case ShapeType::Box:     color = Color(110, 220, 90, 255); break;
            case ShapeType::Capsule: color = Color(220, 110, 220, 255); break;
            case ShapeType::ConvexHull: color = Color(230, 150, 60, 255); break;
            default:                 color = Color::White; break;
            }
            // Asleep is the same colour at a third of the brightness, so a
            // stack settling reads as the whole tower dimming together.
            if (!entry.body->awake())
                color = Color(static_cast<u8>(color.r() / 3), static_cast<u8>(color.g() / 3),
                              static_cast<u8>(color.b() / 3), 255);
        }
        entry.shape->debugDraw(entry.body->transform(), color);
    }

    for (const Contact& contact : mContacts)
        for (u32 i = 0; i < contact.manifold.count; ++i)
        {
            const Math::vec3& point = contact.manifold.points[i].position;
            // The normal is drawn scaled by the impulse it is carrying, so a
            // stack shows where the weight actually goes.
            const f32 scale = 0.05f + contact.manifold.points[i].normalImpulse * 0.02f;
            DebugDraw().line(point, point + contact.manifold.normal * scale, Color::Red);
        }
}

bool PhysicsWorld::raycast(const Ray& ray, f32 maxDistance, const QueryFilter& filter,
                           WorldRayHit& hit) const
{
    bool found = false;
    f32 nearest = maxDistance;
    for (u32 slot = 0; slot < mBodies.size(); ++slot)
    {
        const BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.body || !entry.shape || !entry.enabled)
            continue;
        if (!filter.accepts(id, entry.filter))
            continue;

        ShapeRayHit shapeHit;
        if (!Narrowphase::raycast(*entry.shape, entry.body->transform(), ray, nearest, shapeHit))
            continue;

        nearest = shapeHit.distance;
        hit.body = id;
        hit.point = shapeHit.point;
        hit.normal = shapeHit.normal;
        hit.distance = shapeHit.distance;
        found = true;
    }
    return found;
}

void PhysicsWorld::overlapSphere(const Math::vec3& centre, f32 radius, const QueryFilter& filter,
                                 std::vector<u32>& out) const
{
    out.clear();
    for (u32 slot = 0; slot < mBodies.size(); ++slot)
    {
        const BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.body || !entry.shape || !entry.enabled)
            continue;
        if (!filter.accepts(id, entry.filter))
            continue;
        if (Narrowphase::overlapSphere(*entry.shape, entry.body->transform(), centre, radius))
            out.push_back(id);
    }
}

void PhysicsWorld::queryAABB(const AABB& bounds, const QueryFilter& filter,
                             std::vector<u32>& out) const
{
    out.clear();
    for (u32 slot = 0; slot < mBodies.size(); ++slot)
    {
        const BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.body || !entry.shape || !entry.enabled || !filter.accepts(id, entry.filter))
            continue;
        if (Broadphase::overlaps(bounds, entry.shape->bounds(entry.body->transform())))
            out.push_back(id);
    }
}

u32 PhysicsWorld::applyRadialImpulse(const Math::vec3& centre, f32 radius, f32 strength,
                                     const QueryFilter& filter)
{
    if (!finiteVector(centre) || !(radius > 0.0f) || !std::isfinite(radius) ||
        !std::isfinite(strength) || strength == 0.0f)
        return 0;

    u32 affected = 0;
    for (u32 slot = 0; slot < mBodies.size(); ++slot)
    {
        BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.enabled || !entry.body || !entry.body->isDynamic() ||
            !filter.accepts(id, entry.filter))
            continue;

        Math::vec3 direction;
        const f32 weight = radialWeight(centre, entry.body->position(), radius, &direction);
        if (weight <= 0.0f)
            continue;
        entry.body->applyLinearImpulse(direction * (strength * weight));
        ++affected;
    }
    return affected;
}

u32 PhysicsWorld::addRadialForce(const Math::vec3& centre, f32 radius, f32 strength,
                                 const QueryFilter& filter)
{
    if (!finiteVector(centre) || !(radius > 0.0f) || !std::isfinite(radius) ||
        !std::isfinite(strength) || strength == 0.0f)
        return 0;

    u32 affected = 0;
    for (u32 slot = 0; slot < mBodies.size(); ++slot)
    {
        BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.enabled || !entry.body || !entry.body->isDynamic() ||
            !filter.accepts(id, entry.filter))
            continue;

        Math::vec3 direction;
        const f32 weight = radialWeight(centre, entry.body->position(), radius, &direction);
        if (weight <= 0.0f)
            continue;
        entry.body->addForce(direction * (strength * weight));
        ++affected;
    }
    return affected;
}

u32 PhysicsWorld::addDirectionalForce(const Math::vec3& centre, f32 radius, const Math::vec3& force,
                                      const QueryFilter& filter)
{
    if (!finiteVector(centre) || !finiteVector(force) || !(radius > 0.0f) ||
        !std::isfinite(radius) || Math::dot(force, force) == 0.0f)
        return 0;

    u32 affected = 0;
    for (u32 slot = 0; slot < mBodies.size(); ++slot)
    {
        BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.enabled || !entry.body || !entry.body->isDynamic() ||
            !filter.accepts(id, entry.filter))
            continue;

        const f32 weight = radialWeight(centre, entry.body->position(), radius);
        if (weight <= 0.0f)
            continue;
        entry.body->addForce(force * weight);
        ++affected;
    }
    return affected;
}

} // namespace Radion::Physics
