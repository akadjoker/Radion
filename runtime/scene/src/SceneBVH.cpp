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
    const Math::vec3 center = box.center();
    const Math::vec3 extents = box.extents();
    bool intersects = false;
    for (const Plane& plane : *planes)
    {
        const f32 distance = Math::dot(plane.normal, center) + plane.d;
        const f32 radius = Math::dot(Math::abs(plane.normal), extents);
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
    // Also reached from the destructor, possibly after the GPU device is gone: GPU::ready() is the check for that.
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
        // Skinned bounds follow the animation, but isStatic() only promises the transform; indexing it would freeze the bind-pose box.
        if (!mesh || mesh->isSkinned())
            continue;

        const Math::mat4& model = object->globalTransform();
        for (u32 s = 0; s < mesh->submeshes.size(); ++s)
            // Deliberately short of the occlusion fields: their default initialisers mean "never measured".
            mEntries.push_back(
                {renderer, s, transformAABB(mesh->submeshes[s].bounds, model), QueryHandle()});
    }

    if (mEntries.empty())
        return;

    // Created here, not lazily: the occlusion pass looks one up for every visible hit.
    GPU& gpu = GPU::getSingleton();
    for (Entry& entry : mEntries)
        entry.query = gpu.createQuery();

    mBounds.reserve(mEntries.size());
    for (const Entry& entry : mEntries)
        mBounds.push_back(entry.bounds);
    // Eight per leaf (this tree's own); the BoundsTree default of two would triple the node count for a build-once tree.
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

    // All three tests in one functor so the tree prunes whole subtrees on any of them (keeps shadow cascades from walking most of the tree).
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
            // Fully inside only when every test says so: a node inside a cascade's wedge can still reach past the light's range.
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

    // The tree returns everything sharing a leaf with a hit; the exact test is here.
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
