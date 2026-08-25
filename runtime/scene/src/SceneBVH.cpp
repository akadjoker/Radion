#include "PCH.h"

#include "SceneBVH.h"

#include "AssetManager.h"
#include "DebugDraw3D.h"
#include "GameObject.h"
#include "MeshRenderer.h"

namespace Radion
{

namespace
{
Containment classifyPlanes(const AABB& box, const std::vector<Plane>* planes)
{
    if (!planes || planes->empty())
        return Containment::Inside;
    const glm::vec3 center = box.center();
    const glm::vec3 extents = box.extents();
    bool intersects = false;
    for (const Plane& plane : *planes)
    {
        const f32 distance = glm::dot(plane.normal, center) + plane.d;
        const f32 radius = glm::dot(glm::abs(plane.normal), extents);
        if (distance + radius < 0.0f)
            return Containment::Outside;
        if (distance - radius < 0.0f)
            intersects = true;
    }
    return intersects ? Containment::Intersects : Containment::Inside;
}
} // namespace

void SceneBVH::clear()
{
    // Also reached from the destructor, potentially after the GPU device
    // that owns these queries is already gone (program shutdown order is
    // not guaranteed) - GPU::ready() is the check every other GPU-owning
    // destructor in this codebase uses for exactly that case.
    if (GPU::ready())
    {
        GPU& gpu = GPU::getSingleton();
        for (Entry& entry : mEntries)
            if (entry.query.valid())
                gpu.destroy(entry.query);
    }
    mEntries.clear();
    mBounds.clear();
    mTree.clear();
    mStats.nodeCount = 0;
    mStats.entryCount = 0;
}

void SceneBVH::build(const std::vector<MeshRenderer*>& renderers)
{
    clear();

    AssetManager& assets = Assets();
    for (MeshRenderer* renderer : renderers)
    {
        GameObject* object = renderer->owner();
        if (!object || !object->isStatic() || !renderer->mesh().valid())
            continue;
        Mesh* mesh = assets.getMesh(renderer->mesh());
        // A skinned mesh's bounds move with the animation even when the
        // object's own transform never does - isStatic() only promises the
        // transform, not the pose. Indexing it here would freeze the box at
        // whatever the bind pose happened to be at build() time.
        if (!mesh || mesh->isSkinned())
            continue;

        const glm::mat4& model = object->globalTransform();
        for (u32 s = 0; s < mesh->submeshes.size(); ++s)
            // Deliberately short of the occlusion fields: their own default
            // initialisers are what "never measured" means, and the `true`
            // that used to sit here for the old bool now lands in a 32-bit
            // history as a single set bit - which reads as "visible once, 31
            // frames ago" rather than "unmeasured".
            mEntries.push_back(
                {renderer, s, transformAABB(mesh->submeshes[s].bounds, model), QueryHandle()});
    }

    if (mEntries.empty())
        return;

    // One query per entry, created here rather than lazily on first use:
    // the occlusion pass has to look one up for every visible hit, and a
    // conditional create-on-demand there would mean the first frame an
    // entry is seen always reports "not available yet" for a reason that
    // has nothing to do with the GPU. GLQuery objects are cheap to hold.
    GPU& gpu = GPU::getSingleton();
    for (Entry& entry : mEntries)
        entry.query = gpu.createQuery();

    mBounds.reserve(mEntries.size());
    for (const Entry& entry : mEntries)
        mBounds.push_back(entry.bounds);
    // Eight per leaf, which is what this class's own tree used - dropping to
    // the BoundsTree default of two would triple the node count for no gain
    // on a tree that is built once and only ever queried.
    mTree.setLeafCapacity(8);
    mTree.build(mBounds.data(), static_cast<u32>(mBounds.size()));

    mStats.nodeCount = mTree.nodeCount();
    mStats.entryCount = static_cast<u32>(mEntries.size());
}

void SceneBVH::query(const Frustum& frustum, std::vector<Hit>& out, const Sphere* cullSphere,
                     const std::vector<Plane>* casterPlanes)
{
    mStats.nodesVisited = 0;
    mStats.entriesAccepted = 0;
    if (!mTree.valid())
        return;

    // All three tests in one functor, so the tree still prunes a whole
    // subtree on any of them - and so a node the light's sphere cannot reach
    // costs one test instead of one per entry under it. That pruning is what
    // keeps a large scene from walking most of the tree per shadow cascade,
    // and it is the reason the traversal is templated on a test rather than
    // fixed to a frustum.
    struct NodeTest
    {
        const Frustum& frustum;
        const Sphere* cullSphere;
        const std::vector<Plane>* casterPlanes;

        Containment operator()(const AABB& bounds) const
        {
            if (cullSphere && !cullSphere->intersects(bounds))
                return Containment::Outside;
            const Containment inFrustum = frustum.classify(bounds);
            if (inFrustum == Containment::Outside)
                return Containment::Outside;
            const Containment inCasters = classifyPlanes(bounds, casterPlanes);
            if (inCasters == Containment::Outside)
                return Containment::Outside;
            // Only fully inside when EVERY test says so. A node can sit
            // wholly inside a cascade's wedge and still reach past the
            // light's own range, so one Inside does not license skipping the
            // others.
            const bool sphereWhollyInside = !cullSphere;
            if (inFrustum == Containment::Inside && inCasters == Containment::Inside &&
                sphereWhollyInside)
                return Containment::Inside;
            return Containment::Intersects;
        }
    };

    const NodeTest test{frustum, cullSphere, casterPlanes};
    mTree.queryCandidatesIf(test, mCandidates);
    mStats.nodesVisited = mTree.lastQueryStats().nodesVisited;

    // The tree hands back everything sharing a leaf with a hit, so the exact
    // test is done here - which is also where the entry is turned into a Hit.
    for (u32 item : mCandidates)
    {
        const Entry& entry = mEntries[item];
        if (!frustum.intersects(entry.bounds))
            continue;
        if (classifyPlanes(entry.bounds, casterPlanes) == Containment::Outside)
            continue;
        if (cullSphere && !cullSphere->intersects(entry.bounds))
            continue;
        out.push_back({entry.renderer, entry.submeshIndex, item});
        ++mStats.entriesAccepted;
    }
}

void SceneBVH::debugDraw() const
{
    for (u32 i = 0; i < mTree.nodeCount(); ++i)
    {
        const BoundsTree::Node& node = mTree.node(i);
        DebugDraw().box(node.bounds, node.isLeaf() ? Color::Green : Color::Yellow);
    }
}

} // namespace Radion
