#ifndef RADION_SCENE_H
#define RADION_SCENE_H

#include "Animation.h"
#include "BoneAttachment.h"
#include "BoundsTree.h"
#include "Camera.h"
#include "Collider.h"
#include "CollisionWorld.h"
#include "Containers.h"
#include "Thread.h"
#include "Forest.h"
#include "GPU.h"
#include "GameObject.h"
#include "Grass.h"
#include "Hair.h"
#include "Landscape.h"
#include "Light.h"
#include "MeshRenderer.h"
#include "Ocean.h"
#include "ParticleEffect.h"
#include "ReflectionProbe.h"
#include "RenderList.h"
#include "Road.h"
#include "SceneBVH.h"
#include "Terrain.h"

#include <vector>

namespace Radion
{

class Engine;

// Implements ShadowCasterSource (RenderList.h) so a render/ technique -
// ShadowPass, Lighting - can ask for a shadow view's caster list without
// render/ ever depending on scene/. See docs/ENGINE.md's layering.
class Scene : public ShadowCasterSource
{
public:
    Scene();
    ~Scene();

    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;

    GameObject& root();
    const GameObject& root() const;
    GameObject* createGameObject(const std::string& name = std::string(),
                                 GameObject* parent = nullptr);

    // Same, but with the id an earlier session (or a snapshot) recorded,
    // instead of the next one out of the counter. For loading a saved scene
    // and for undoing a delete, where the references pointing at this object
    // - a parent link, a component's target, the active camera - were all
    // written against that exact number and would otherwise dangle. Fails if
    // `id` is 0 or already taken in this Scene, rather than silently handing
    // out a different one: to whoever is restoring, a collision is a broken
    // file, not something to paper over. The counter is pushed past `id` so
    // later objects never land on it.
    GameObject* createGameObject(u64 id, const std::string& name, GameObject* parent = nullptr);

    // Hands out an id from the same counter createGameObject() itself draws
    // from, without creating anything - what a caller building several
    // ParsedObject-shaped entries up front (SceneSerializer::cloneObject())
    // uses to mint ids that are guaranteed free before any of them exist yet.
    u64 reserveId()
    {
        return mNextId++;
    }

    // The object carrying `id` in this Scene, or null - including for 0,
    // which no object ever has (the root answers to root(), not to an id).
    // What resolves a reference stored as an id back into a pointer.
    GameObject* findGameObject(u64 id) const;

    // The first object called `name`, or null. Names are not unique and not
    // indexed - this walks the scene's object list - so it is meant for a
    // handful of known objects looked up once at load ("player", "spawn"),
    // never per frame. Keep the pointer it returns.
    GameObject* findGameObject(const std::string& name) const;

    usize countByTag(const std::string& tag) const;

    bool add(GameObject* object, GameObject* parent = nullptr);
    bool remove(GameObject* object);
    bool destroy(GameObject* object);
    bool reparent(GameObject* object, GameObject* parent = nullptr);

    void setActiveCamera(Camera* camera);
    Camera* activeCamera() const;

    // Position + rotation of the active camera's GameObject, as their own
    // small JSON file - the first piece of what will grow into a full scene
    // serializer, kept to just this for now so a demo can drop the camera
    // back at a known spot (comparing FPS before/after a change, e.g.)
    // without hand-typing coordinates. Missing file/no active camera is not
    // an error, same convention as EngineSettings::load().
    bool saveCamera(const std::string& filename) const;
    bool loadCamera(const std::string& filename);
    void update(f32 deltaTime);
    f32 deltaTime() const;
    usize gameObjectCount() const;
    usize renderableCount() const;
    usize animatedCount() const;
    usize cameraCount() const;
    usize lightCount() const;

    // Cameras currently attached to scene objects. Editor Game view uses
    // this to choose which one setActiveCamera() designates as the real game
    // output; ownership remains entirely in the Scene/GameObjects.
    const std::vector<Camera*>& cameras() const
    {
        return mCameras;
    }

    // The DirectionalLight whose own orientation should drive the active
    // Sky's sun direction every frame (Engine::render() does the sync),
    // instead of the sky's own azimuth/elevation - one light stays the
    // single source of truth for both shading and the skybox. Same
    // explicit-registration shape as setActiveCamera(): a stored pointer,
    // not a per-frame search over mLights.
    void setSunLight(DirectionalLight* light);
    DirectionalLight* sunLight() const;
    DirectionalLight* electedSunLight() const;

    // Every Light currently attached in the scene (all four kinds mixed
    // together - lightType() tells them apart) - a mass "turn off every
    // point/spot light's shadow" toggle has no other way to reach them all,
    // sunLight() being the one kind with its own dedicated single-pointer
    // slot.
    const std::vector<Light*>& lights() const
    {
        return mLights;
    }

    // Every ReflectionProbe currently attached in the scene, for Engine::
    // render() to drive each one's own capture - not just Engine's single
    // built-in probe. buildRenderList() below already resolves the nearest
    // one per object on its own; this is only for capturing them.
    const std::vector<ReflectionProbe*>& reflectionProbes() const
    {
        return mReflectionProbes;
    }

    const std::vector<Collider*>& colliders() const
    {
        return mColliders;
    }

    CollisionWorld& collisions()
    {
        return mCollisionWorld;
    }
    const CollisionWorld& collisions() const
    {
        return mCollisionWorld;
    }

    // Collects every visible object into `list` from the active camera, the
    // same call Engine::render() makes internally. Public so a demo driving
    // its own multi-pass frame (a planar reflection sub-pass, e.g.) can still
    // use Scene for object management without Engine's own render() owning
    // the whole frame. `filter` is applied the same way RenderList::submit()
    // always has: only objects whose material carries every one of these
    // flags are kept, so a reflection sub-pass can ask for MaterialReflection
    // and skip the rest of the scene without a second, hand-submitted list.
    bool buildRenderList(RenderList& list, u32 filter = 0);

    // Same, but from an explicit view instead of the active Camera - an
    // editor's own free-look observer wants to cull/submit against wherever
    // it currently is without that view ever becoming the scene's one
    // Camera (owning an entity just to hold a view/projection pair is the
    // patch; this is what removes the need for one). cameraPosition alone,
    // not a full transform, because nothing downstream (terrain LOD,
    // landscape/forest distance culling, the frustum built from
    // viewProjection) reads anything else about the viewer.
    // occlusionView false = a bystander (the editor's observer): frustum
    // culling only, no occlusion verdicts or bookkeeping touched.
    // previewOcclusion lets a bystander apply the game camera's verdicts
    // read-only.
    bool buildRenderList(RenderList& list, const glm::mat4& viewProjection,
                         const glm::vec3& cameraPosition, u32 filter = 0,
                         bool occlusionView = true, bool previewOcclusion = false);

    // Rebuilds the static spatial index (SceneBVH) from every current
    // MeshRenderer whose owner GameObject::isStatic(). Membership changes
    // (add/remove renderer, setStatic(), setMesh()) also mark it dirty and
    // rebuild lazily before the next camera/shadow query. Static transforms
    // are still a caller promise: moving one requires an explicit rebuild.
    void rebuildStaticIndex();
    const SceneBVH& staticIndex() const
    {
        return mStaticIndex;
    }
    // Straight from the last buildRenderList()/updateOcclusionQueries() call
    // - for a debug panel to show directly, since stdout from a background
    // run does not reliably reach whoever is reading it.
    usize lastStaticHitCount() const
    {
        return mStaticHits.size();
    }
    usize lastOcclusionCandidateCount() const
    {
        return mOcclusionCandidates.size();
    }
    usize lastOcclusionHiddenCount() const
    {
        usize hidden = 0;
        for (const SceneBVH::Hit& hit : mStaticHits)
            if (!mStaticIndex.lastVisible(hit.entryIndex))
                ++hidden;
        return hidden;
    }

    // A/B switch for comparison, not something a shipped scene toggles:
    // false makes buildRenderList() fall the static set back through the
    // same per-renderer loop the dynamic objects already use, as if the BVH
    // did not exist. The tree itself is left alone, just unused.
    void setStaticCullingEnabled(bool enabled)
    {
        mStaticCullingEnabled = enabled;
    }
    bool staticCullingEnabled() const
    {
        return mStaticCullingEnabled;
    }

    // The dynamic counterpart to the static BVH: a BVH over every
    // MeshRenderer whose owner is not static, so moving objects are spatially
    // culled instead of the per-renderer scan. Unlike the static index it is
    // refreshed automatically - buildRenderList() rebuilds it when the
    // renderer set changes, refits it every frame, and rebuilds it in the
    // background, so there is no manual rebuild to keep in sync. Off by
    // default: turning it on is the opt-in
    // that trades the linear dynamic loop for the octree query; turning it
    // back off restores the old loop exactly.
    void rebuildDynamicIndex();
    // Applies the dirty queue to mDynamicBounds, then refits and fires the
    // background rebuild. Called once per buildRenderList().
    void refreshDynamicBounds();
    void updateDynamicTree();
    static void buildDynamicTreeJob(void* userData);
    void setDynamicLeafCapacity(u32 capacity);
    void setDynamicCullingEnabled(bool enabled);
    bool dynamicCullingEnabled() const
    {
        return mDynamicCullingEnabled;
    }
    const BoundsTree& dynamicIndex() const
    {
        return mDynamicTree;
    }

    // What a debug panel wants to show, gathered in one place so the panels
    // do not have to know which structure is behind it - the last time that
    // leaked out, swapping the octree for a BVH broke four of them.
    struct DynamicIndexStats
    {
        u32 entryCount = 0;
        u32 nodeCount = 0;
        u32 depth = 0;
        u32 nodesVisited = 0;    // last query
        u32 entriesAccepted = 0; // last query, after the exact test
        // Total node surface area over the root's. Climbs as refits let node
        // boxes overlap and drops when the background rebuild is swapped in.
        f32 quality = 0.0f;
        bool rebuildPending = false;
    };
    DynamicIndexStats dynamicIndexStats() const;
    // Straight from the last buildRenderList() call - for a debug panel.
    usize lastDynamicHitCount() const
    {
        return mDynamicHits.size();
    }
    // Debug: box() for every dynamic octree node + entry, like
    // debugDrawOcclusion() is for the static BVH. Must run before
    // Engine::render() like any other DebugDraw call.
    void debugDrawDynamicIndex(bool leavesOnly = false, bool drawEntries = true) const;

    // Off by default until verified on a scene big enough to matter (the
    // static BVH already handles Sponza/Bistro-sized frustum culling;
    // occlusion is the next win only once something is actually hidden
    // behind something else). Toggling never rebuilds the BVH - the queries
    // are created once in rebuildStaticIndex() and just go unread while this
    // is off, exactly like setStaticCullingEnabled() leaves the tree built
    // but unused.
    void setOcclusionQueryEnabled(bool enabled)
    {
        mOcclusionQueryEnabled = enabled;
    }
    bool occlusionQueryEnabled() const
    {
        return mOcclusionQueryEnabled;
    }

    // Reads last frame's occlusion result for every entry buildRenderList()
    // saw this frame (mStaticHits, still holding this frame's static hits -
    // nothing else repopulates it in between) and launches this frame's
    // query for each, testing depth-only against `depthTarget`. Call once,
    // right after the depth prepass has written real geometry into that
    // target and before the Forward pass reads buildRenderList()'s result -
    // next frame's buildRenderList() is what actually acts on what gets
    // measured here. A no-op when setOcclusionQueryEnabled() is off.
    void updateOcclusionQueries(TargetHandle depthTarget, const glm::mat4& viewProjection,
                                const glm::vec3& cameraPosition);

    // One box per entry buildRenderList() saw this frame (mStaticHits, same
    // scratch updateOcclusionQueries() reads) - green if visible (including
    // one never tested at all, too small/close to be worth it - see
    // updateOcclusionQueries() - since it defaults to visible and is drawn
    // unconditionally either way), red if measured occluded last frame.
    // Emits into DebugDraw3D, so it must run before Engine::render() the
    // same way any other DebugDraw call does - that pass clears the list at
    // the start of the frame.
    void debugDrawOcclusion() const;

    // Picks a world position + normal under a screen point by reading the
    // depth buffer the frame already wrote - no CPU raycast, so it hits
    // whatever actually drew (grass built in the vertex shader, dynamic
    // meshes, ...). Lives here rather than on DecalSystem: picking is not a
    // decal concern. mouseX/mouseY are window pixels (origin top-left);
    // depth is the frame's depth texture at its own size. Returns false when
    // the pixel has no geometry (sky or background).
    static bool pickSurface(TextureHandle depth, u32 depthWidth, u32 depthHeight, f32 mouseX,
                            f32 mouseY, u32 windowWidth, u32 windowHeight,
                            const glm::mat4& inverseProjection, const glm::mat4& inverseView,
                            glm::vec3& outPosition, glm::vec3& outNormal);

    // Nearest GameObject whose MeshRenderer's world AABB the ray crosses, or
    // nullptr - inactive objects and hidden ones (isVisibleInHierarchy()
    // false) are skipped, same as what actually draws. A plain recursive
    // walk with a per-object AABB test, not a tree descent: editor click
    // picking is once per click, not the per-frame cost mStaticIndex/
    // mDynamicIndex's own frustum queries exist to spare.
    GameObject* pickObject(const Ray& ray, f32* outDistance = nullptr) const;

    GameObject* pickDynamicObject(const Ray& ray, f32* outDistance = nullptr);

    // GameObject whose world AABB contains a world point - meant to follow up
    // a pickSurface() hit with the object that actually owns that surface
    // point, which is exact even for concave meshes a ray-vs-AABB pickObject()
    // would get wrong (the box is hit, but from an angle no triangle there
    // actually faces). Ties (nested/overlapping boxes) go to the smallest
    // volume, the tighter fit around the point.
    GameObject* pickObjectAtPoint(const glm::vec3& point) const;

    // Same idea as pickObjectAtPoint(), one level down - which of `object`'s
    // own MeshRenderer submeshes (by SubMesh::bounds, its own coarse AABB,
    // never actual triangles - same precision level pickObjectAtPoint()
    // itself works at) contains a world point already known to be on this
    // object. -1 if `object` has no MeshRenderer, no mesh, or the point
    // falls outside every submesh's box. materialSlot IS the index
    // InspectorPanel's per-slot material list already uses, so a caller
    // wanting "which material did the user just click" needs nothing more
    // than this. `outSubmesh`, when given, additionally receives the index of
    // the winning submesh within Mesh::submeshes (-1 on a miss): several
    // submeshes can share one materialSlot, so the slot alone cannot say
    // which box was actually hit - what a caller wanting to outline exactly
    // the piece under the cursor needs.
    static s32 pickSubmeshAtPoint(const GameObject& object, const glm::vec3& point,
                                  s32* outSubmesh = nullptr);

    // Set by the editor host once, right after it creates this Scene - never
    // by runtime/ itself. Gameplay code checks this to skip logic that only
    // makes sense standalone (input capture, cursor locking, ...) while the
    // editor's own camera and gizmos are what the user is actually driving.
    bool runningInEditor() const
    {
        return mRunningInEditor;
    }
    void setRunningInEditor(bool value)
    {
        mRunningInEditor = value;
    }

    // Load file-backed meshes on a worker instead of blocking the load call.
    // The editor gets this from runningInEditor() so its window keeps
    // answering; a standalone caller that wants to draw a loading screen
    // rather than freeze needs to ask for it, and must then pump frames
    // until Assets().pendingAsyncMeshLoads() reaches zero - until it does,
    // the meshes exist as valid but empty handles that draw nothing.
    bool asyncMeshLoad() const
    {
        return mAsyncMeshLoad;
    }
    void setAsyncMeshLoad(bool value)
    {
        mAsyncMeshLoad = value;
    }

private:
    friend class Engine;
    friend class GameObject;

    struct PendingAdd
    {
        GameObject* object = nullptr;
        GameObject* parent = nullptr;
    };

    // ShadowCasterSource override. Private on purpose, unlike
    // buildRenderList above: callers reach it through the ShadowCasterSource&
    // interface, where it is public, never through a concrete Scene&. No
    // position is taken for setCamera(): the sort key it would feed only
    // orders a depth-only pass for early-Z, which does not affect
    // correctness, and a directional light has no position to give it.
    bool buildShadowList(RenderList& list, const glm::mat4& viewProjection, u32 filter,
                         const Sphere* cullSphere = nullptr, MeshHandle exclude = MeshHandle(),
                         u64 excludeObjectId = 0, bool reflectionCapture = false,
                         const std::vector<Plane>* casterPlanes = nullptr,
                         f32 minCasterExtent = 0.0f) override;
    void registerBranch(GameObject* object);
    void unregisterBranch(GameObject* object);
    void componentAdded(Component* component);
    void componentRemoved(Component* component);
    void compactComponentLists();
    void debugFlagsChanged(GameObject* object, u32 previousFlags);
    void flushChanges();
    void queueDynamicBoundsUpdate(GameObject* object);
    void invalidateSpatialIndexes();
    void reapplyHiddenSubmeshes();
    void clearDynamicBoundsQueue();

    // Lazily created on the first updateOcclusionQueries() call - a scene
    // that never turns the toggle on never pays for these.
    bool setupOcclusionQueryResources();
    // Grows mOcclusionBlock to fit at least `count` entries, each at its own
    // mOcclusionBlockStride offset. Never shrinks - a scene's worst-case
    // frame stays the buffer's size for the rest of the run, the same
    // trade-off DepthPass's own instance buffer already makes.
public:
    // Frames to spread the occlusion queries over. 1 measures every entry
    // every frame, which is what the reference does; higher divides the
    // per-frame cost and multiplies how stale a verdict can get. A debug
    // panel wants this as a slider - the trade only shows up under a real
    // scene, and the number that suits one does not suit another.
    void setOcclusionStagger(u32 frames)
    {
        mOcclusionStagger = glm::max(frames, 1u);
    }
    u32 occlusionStagger() const
    {
        return mOcclusionStagger;
    }
    // Queries actually issued last frame - what the stagger divides.
    u32 lastOcclusionQueryCount() const
    {
        return mLastOcclusionQueryCount;
    }
    u32 lastOcclusionResultCount() const
    {
        return mLastOcclusionResultCount;
    }
    u32 lastOcclusionPendingCount() const
    {
        return mLastOcclusionPendingCount;
    }

private:
    u32 mOcclusionStagger = 2;
    u32 mLastOcclusionQueryCount = 0;
    u32 mLastOcclusionResultCount = 0;
    u32 mLastOcclusionPendingCount = 0;

    bool ensureOcclusionBlockCapacity(u32 count);
    bool ensureOcclusionResultCapacity(u32 count);
    // Applies the verdicts sitting in one result buffer's mapping.
    void readOcclusionResults(u32 bufferIndex);

    // What survives the per-hit poll+filter pass in updateOcclusionQueries(),
    // waiting for its turn in the batched upload - the query is launched
    // against a model matrix that by then lives at a stable offset instead
    // of being overwritten out from under a draw still in flight.
    struct OcclusionCandidate
    {
        QueryHandle query;
        u32 entryIndex = 0;
        AABB worldBounds;
    };

    // Stamps ids and hands out pointers for them. Both cover every object
    // this Scene owns, registered or merely detached by remove(), because a
    // detached object keeps its id and can be put back - a delete/undo pair
    // is exactly that. An entry only goes when the object is deleted.
    void stampId(GameObject* object);
    void forgetIdBranch(GameObject* object);

    GameObject mRoot;
    Camera* mActiveCamera = nullptr;
    DirectionalLight* mSunLight = nullptr;
    u64 mNextId = 1;
    HashMap<u64, GameObject*> mObjectsById;
    std::vector<GameObject*> mObjects;
    std::vector<MeshRenderer*> mRenderers;
    SceneBVH mStaticIndex;

    // Dynamic geometry, double buffered the way the reference does it
    // (wiScene.cpp: collider_bvh / collider_bvh_next). Every frame the live
    // tree is REFITTED - shape kept, boxes recomputed, one linear pass - and
    // a full rebuild is fired on the pool to be swapped in next frame. The
    // tree is therefore never more than one frame from a fresh build, the
    // rebuild never touches the frame's critical path, and there is no
    // threshold to tune.
    //
    // Measured before replacing the octree that used to be here: at 26000
    // objects with everything moving, refit 0.401 ms against the octree's
    // 3.253 ms update, and the frustum query 0.402 ms against 0.660 ms. The
    // refit does not care how much moved - the octree pays per dirty object,
    // this pays the same either way.
    BoundsTree mDynamicTree;
    BoundsTree mDynamicTreeNext;
    // Payload, parallel to mDynamicBounds: the tree hands back indices.
    std::vector<MeshRenderer*> mDynamicRenderers;
    HashMap<const MeshRenderer*, u32> mDynamicRendererIndex;
    // Live boxes, kept current by the dirty queue and handed to refit().
    std::vector<AABB> mDynamicBounds;
    // The background build reads its OWN copy. Handing it the live array
    // would be a data race for the sake of saving a memcpy - and the copy is
    // one, over a few hundred kilobytes.
    std::vector<AABB> mDynamicBuildBounds;
    std::vector<u32> mDynamicCandidates;
    JobGroup mDynamicBuildJob;
    bool mDynamicBuildPending = false;
    // Ticks once per buildRenderList() call - SceneBVH::justEnteredView()'s
    // "how long since this entry was last in the camera's frustum" clock.
    u32 mFrameNumber = 0;
    std::vector<SceneBVH::Hit> mStaticHits;       // scratch, reused every buildRenderList() call
    std::vector<SceneBVH::Hit> mBystanderHits;
    std::vector<SceneBVH::Hit> mShadowStaticHits; // scratch, reused every buildShadowList() call
    std::vector<MeshRenderer*> mDynamicHits;   // scratch, reused every buildRenderList() call
    std::vector<MeshRenderer*> mBystanderDynamicHits;
    // Dynamic renderers deliberately excluded from the tree (currently
    // skinned meshes). Rebuilt with the index, never rediscovered per frame.
    std::vector<MeshRenderer*> mDynamicLinearFallback;
    // Object ids rather than pointers: remove/destroy can happen before the
    // next render and stale ids can then be ignored safely.
    std::vector<u64> mDynamicBoundsDirty;
    bool mRunningInEditor = false;
    bool mAsyncMeshLoad = false;
    bool mStaticCullingEnabled = true;
    bool mOcclusionQueryEnabled = false;
    bool mDynamicCullingEnabled = false;
    // A static renderer was added, removed, or changed membership/mesh.
    // Rebuild before either the camera or a shadow pass can query the BVH;
    // otherwise its entries may still contain a freed MeshRenderer pointer.
    bool mStaticIndexDirty = true;
    // Renderer set changed (or the toggle was turned on) - buildRenderList()
    // rebuilds the octree when this is set.
    bool mDynamicIndexDirty = true;

    // A single shared unit cube (positions only, [-1, 1]^3) stands in for
    // every entry's real geometry in the occlusion query itself - the query
    // only needs to know whether *something* at that box would be visible,
    // and testing the box instead of however many thousand vertices the
    // real submesh has is strictly cheaper and never less correct (a box
    // conservatively bounds the mesh it replaces).
    PipelineHandle mOcclusionPipeline;
    // One buffer, all of this frame's entries written into it once before
    // the draw loop (each at its own aligned offset) instead of rewriting
    // the same bytes before every draw - the GPU may still be reading last
    // entry's data when the next write lands, and a driver has to stall the
    // CPU to keep that safe. A distinct offset per entry never overlaps a
    // read in flight, so nothing has to wait.
    BufferHandle mOcclusionBlock;
    u32 mOcclusionBlockCapacity = 0; // entries, not bytes
    u32 mOcclusionBlockStride = 0;   // bytes between entries, alignment-padded
    BufferHandle mOcclusionCubeVertices;
    BufferHandle mOcclusionCubeIndices;
    // Scratch, reused every updateOcclusionQueries() call - cleared, not
    // freed, so a scene's worst-case frame is the only frame that ever pays
    // for growing them.
    std::vector<OcclusionCandidate> mOcclusionCandidates;

    // Query results, resolved into a buffer on the GPU and read from its
    // mapping a frame later. Two of them: one is being written by this
    // frame's resolve while the other holds last frame's, which is what
    // makes the read a plain memory access with nothing to wait for.
    //
    // This replaces asking the driver per query whether a result was ready.
    // That question is not free even in its non-blocking form - it has to
    // flush the command buffer the query's own end packet is still sitting
    // in, dragging the frame's submission forward into a wait. The reference
    // sidesteps it the same way (wiRenderer's OcclusionCulling_Resolve into
    // queryResultBuffer[], read from mapped_data next frame).
    static constexpr u32 kOcclusionResultBuffers = 2;
    BufferHandle mOcclusionResults[kOcclusionResultBuffers];
    // Which entry each slot in the result buffer belongs to, recorded when
    // the query is launched - the buffer is a flat array and has no idea.
    std::vector<u32> mOcclusionResultOwners[kOcclusionResultBuffers];
    u32 mOcclusionResultCapacity = 0;
    u32 mOcclusionResultIndex = 0;
    std::vector<u8> mOcclusionBlockScratch;
    std::vector<Terrain*> mTerrains;
    std::vector<Landscape*> mLandscapes;
    std::vector<Road*> mRoads;
    std::vector<Forest*> mForests;
    std::vector<Grass*> mGrass;
    std::vector<Hair*> mHair;
    std::vector<Ocean*> mOceans;
    std::vector<ParticleEffect*> mParticleEffects;
    // Flat event lists avoid scanning ComponentType::Count slots for every
    // object. In script-heavy scenes most objects only carry one component,
    // so this keeps the frame cost proportional to live components instead
    // of objects times possible component types.
    std::vector<Component*> mUpdateComponents;
    std::vector<Component*> mLateUpdateComponents;
    std::vector<Animator*> mAnimators;
    std::vector<BoneAttachment*> mBoneAttachments;
    std::vector<Camera*> mCameras;
    std::vector<Light*> mLights;
    std::vector<ReflectionProbe*> mReflectionProbes;
    std::vector<Collider*> mColliders;
    CollisionWorld mCollisionWorld;
    std::vector<GameObject*> mDebugObjects;
    std::vector<PendingAdd> mPendingAdd;
    std::vector<GameObject*> mPendingRemove;
    std::vector<GameObject*> mPendingDestroy;
    std::vector<GameObject*> mDetached;
    f32 mDeltaTime = 0.0f;
};

} // namespace Radion

#endif // RADION_SCENE_H
