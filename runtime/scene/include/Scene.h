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
#include "collision/Broadphase.h"
#include "dynamics/ContactSolver.h"
#include "dynamics/JointMatch.h"
#include "dynamics/PhysicsEvents.h"
#include "dynamics/RigidBody.h"

#include <vector>

namespace Radion::Physics
{
class Joint;
}

namespace Radion::AI
{
class Obstacle;
}

namespace Radion
{

class Agent;
class Color;
class Engine;
class Obstacle;

// Implements ShadowCasterSource so render/ need not depend on scene/.
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

    // Restores a recorded id (load, undo). Fails if id is 0 or taken; the counter is pushed past id.
    GameObject* createGameObject(u64 id, const std::string& name, GameObject* parent = nullptr);

    // Mints an id from the same counter without creating anything.
    u64 reserveId()
    {
        return mNextId++;
    }

    // Null for 0 or unknown; the root answers to root().
    GameObject* findGameObject(u64 id) const;

    // Linear walk over non-unique names; not for per-frame use.
    GameObject* findGameObject(const std::string& name) const;

    usize countByTag(const std::string& tag) const;

    bool add(GameObject* object, GameObject* parent = nullptr);
    bool remove(GameObject* object);
    bool destroy(GameObject* object);
    bool reparent(GameObject* object, GameObject* parent = nullptr);

    void setActiveCamera(Camera* camera);
    Camera* activeCamera() const;

    // Missing file/no active camera is not an error, same as EngineSettings::load().
    bool saveCamera(const std::string& filename) const;
    bool loadCamera(const std::string& filename);
    void update(f32 deltaTime);
    f32 deltaTime() const;
    usize gameObjectCount() const;
    usize renderableCount() const;
    usize animatedCount() const;
    usize cameraCount() const;
    usize lightCount() const;

    const std::vector<Camera*>& cameras() const
    {
        return mCameras;
    }

    // One light drives both shading and the skybox sun (Engine::render() syncs it).
    void setSunLight(DirectionalLight* light);
    DirectionalLight* sunLight() const;
    DirectionalLight* electedSunLight() const;

    // All four light kinds mixed; lightType() tells them apart.
    const std::vector<Light*>& lights() const
    {
        return mLights;
    }

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

    void addBody(Physics::RigidBody& body);
    void removeBody(Physics::RigidBody& body);
    const std::vector<Physics::RigidBody*>& rigidBodies() const
    {
        return mRigidBodies;
    }
    usize bodyCount() const
    {
        return mRigidBodies.size();
    }
    void clearPhysics();

    void addJoint(Physics::Joint* joint);
    void removeJoint(Physics::Joint* joint);
    usize jointCount() const
    {
        return mJoints.size();
    }

    void addAgent(Agent& agent);
    void removeAgent(Agent& agent);
    // Can contain null entries while updateAgents() runs (agent destroyed by a callback); skip them.
    const std::vector<Agent*>& agents() const
    {
        return mAgents;
    }
    usize agentCount() const
    {
        return mAgents.size();
    }
    void updateAgents(f32 deltaTime);
    void clearAI();

    void addObstacle(Obstacle& obstacle);
    void removeObstacle(Obstacle& obstacle);
    // Inactive obstacles or ones on disabled objects block nobody. Called each frame, on add/remove, and after rebuildOwnedShape().
    void rebuildObstacleGroup();
    const std::vector<Obstacle*>& obstacles() const
    {
        return mObstacleComponents;
    }
    const std::vector<AI::Obstacle*>& obstacleGroup() const
    {
        return mObstacleGroup;
    }
    usize obstacleCount() const
    {
        return mObstacleComponents.size();
    }
    void debugDrawObstacles() const;
    static void debugDrawObstacleShape(const Obstacle& obstacle, Color color);

    void setGravity(const Math::vec3& gravity);
    const Math::vec3& gravity() const
    {
        return mGravity;
    }
    void setFixedStep(f32 seconds);
    f32 fixedStep() const
    {
        return mFixedStep;
    }
    void setSolverSettings(const Physics::ContactSolverSettings& settings);
    void setContactEventCallback(Physics::ContactEventCallback callback, void* userData);
    void setPhysicsStepCallback(Physics::PhysicsStepCallback callback, void* userData);
    void setContactPersistence(u32 steps);
    u32 contactPersistence() const
    {
        return mContactPersistence;
    }
    void setContactMargin(f32 margin);
    f32 contactMargin() const
    {
        return mContactMargin;
    }
    void markStaticBroadphaseDirty();

    // Consumes deltaTime in fixed steps, keeping the remainder.
    void updatePhysics(f32 deltaTime);
    void stepPhysics(f32 duration);
    void debugDrawPhysicsShapes() const;
    void debugDrawPhysicsContacts() const;
    void debugDrawPhysicsJoints() const;

    bool raycast(const Ray& ray, f32 maxDistance, const Physics::QueryFilter& filter,
                Physics::WorldRayHit& hit) const;
    void overlapSphere(const Math::vec3& centre, f32 radius, const Physics::QueryFilter& filter,
                       std::vector<Physics::RigidBody*>& out) const;
    void queryAABB(const AABB& bounds, const Physics::QueryFilter& filter,
                  std::vector<Physics::RigidBody*>& out) const;

    // Radial strength is signed: positive pushes away, negative pulls. Effects decay linearly to zero at radius.
    // Forces must be added before stepPhysics(); impulses are instantaneous.
    u32 applyRadialImpulse(const Math::vec3& centre, f32 radius, f32 strength,
                           const Physics::QueryFilter& filter = Physics::QueryFilter());
    u32 addRadialForce(const Math::vec3& centre, f32 radius, f32 strength,
                       const Physics::QueryFilter& filter = Physics::QueryFilter());
    u32 addDirectionalForce(const Math::vec3& centre, f32 radius, const Math::vec3& force,
                            const Physics::QueryFilter& filter = Physics::QueryFilter());

    usize contactCount() const
    {
        return mContacts.size();
    }
    const std::vector<Physics::Contact>& contacts() const
    {
        return mContacts;
    }

    // `filter`: only objects whose material carries every one of these flags are kept.
    bool buildRenderList(RenderList& list, u32 filter = 0);

    // cameraPosition alone suffices (LOD and culling read nothing else). occlusionView false = bystander: frustum culling only.
    // previewOcclusion applies the game camera's verdicts read-only.
    bool buildRenderList(RenderList& list, const Math::mat4& viewProjection,
                         const Math::vec3& cameraPosition, u32 filter = 0,
                         bool occlusionView = true, bool previewOcclusion = false);

    // Membership changes mark it dirty and rebuild lazily; moving a static transform requires an explicit rebuild.
    void rebuildStaticIndex();
    const SceneBVH& staticIndex() const
    {
        return mStaticIndex;
    }
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

    // A/B switch: false sends the static set through the per-renderer loop as if the BVH did not exist.
    void setStaticCullingEnabled(bool enabled)
    {
        mStaticCullingEnabled = enabled;
    }
    bool staticCullingEnabled() const
    {
        return mStaticCullingEnabled;
    }

    // Refreshed automatically (rebuilt on set change, refit every frame, rebuilt in the background). Off by default.
    void rebuildDynamicIndex();
    // Called once per buildRenderList().
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

    struct DynamicIndexStats
    {
        u32 entryCount = 0;
        u32 nodeCount = 0;
        u32 depth = 0;
        u32 nodesVisited = 0;
        u32 entriesAccepted = 0;
        // Total node surface area over the root's; climbs as refits overlap node boxes, drops on rebuild swap-in.
        f32 quality = 0.0f;
        bool rebuildPending = false;
    };
    DynamicIndexStats dynamicIndexStats() const;
    usize lastDynamicHitCount() const
    {
        return mDynamicHits.size();
    }
    // Must run before Engine::render() like any DebugDraw call.
    void debugDrawDynamicIndex(bool leavesOnly = false, bool drawEntries = true) const;

    // Off by default; toggling never rebuilds the BVH, the queries just go unread.
    void setOcclusionQueryEnabled(bool enabled)
    {
        mOcclusionQueryEnabled = enabled;
    }
    bool occlusionQueryEnabled() const
    {
        return mOcclusionQueryEnabled;
    }

    // Call once, after the depth prepass and before the Forward pass reads buildRenderList()'s result.
    // No-op when setOcclusionQueryEnabled() is off.
    void updateOcclusionQueries(TargetHandle depthTarget, const Math::mat4& viewProjection,
                                const Math::vec3& cameraPosition);

    // Green = visible (or never tested), red = occluded last frame. Must run before Engine::render().
    void debugDrawOcclusion() const;

    // Reads the depth buffer, so it hits whatever drew (grass, dynamic meshes). mouseX/mouseY are window pixels, origin top-left.
    static bool pickSurface(TextureHandle depth, u32 depthWidth, u32 depthHeight, f32 mouseX,
                            f32 mouseY, u32 windowWidth, u32 windowHeight,
                            const Math::mat4& inverseProjection, const Math::mat4& inverseView,
                            Math::vec3& outPosition, Math::vec3& outNormal);

    // Plain recursive AABB walk; inactive and hidden objects are skipped.
    GameObject* pickObject(const Ray& ray, f32* outDistance = nullptr) const;

    GameObject* pickDynamicObject(const Ray& ray, f32* outDistance = nullptr);

    // Ties (nested boxes) go to the smallest volume.
    GameObject* pickObjectAtPoint(const Math::vec3& point) const;

    // Coarse: tests SubMesh::bounds, not triangles. -1 on a miss. outSubmesh receives the submesh index within Mesh::submeshes;
    // several submeshes can share one materialSlot.
    static s32 pickSubmeshAtPoint(const GameObject& object, const Math::vec3& point,
                                  s32* outSubmesh = nullptr);

    // Set once by the editor host; gameplay skips standalone-only logic (input capture, cursor locking) when true.
    bool runningInEditor() const
    {
        return mRunningInEditor;
    }
    void setRunningInEditor(bool value)
    {
        mRunningInEditor = value;
    }

    // Async: meshes are valid but empty handles until Assets().pendingAsyncMeshLoads() reaches zero.
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

    // Private on purpose: reached through ShadowCasterSource&. No position for setCamera(): the sort key only orders an early-Z pass.
    bool buildShadowList(RenderList& list, const Math::mat4& viewProjection, u32 filter,
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

    // Created lazily on the first updateOcclusionQueries() call.
    bool setupOcclusionQueryResources();
    // Never shrinks: a scene's worst-case frame stays the buffer size.
public:
    // 1 measures every entry every frame; higher divides per-frame cost and makes verdicts staler.
    void setOcclusionStagger(u32 frames)
    {
        mOcclusionStagger = Math::max(frames, 1u);
    }
    u32 occlusionStagger() const
    {
        return mOcclusionStagger;
    }
    // What the stagger divides.
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
    void readOcclusionResults(u32 bufferIndex);

    // Waits for the batched upload so the model matrix lives at a stable offset and is not overwritten under an in-flight draw.
    struct OcclusionCandidate
    {
        QueryHandle query;
        u32 entryIndex = 0;
        AABB worldBounds;
    };

    // Covers registered and detached objects alike: a detached object keeps its id and can be put back (delete/undo).
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

    // Double buffered like the reference (wiScene.cpp collider_bvh / collider_bvh_next): the live tree is refitted each frame
    // and a full rebuild is swapped in next frame, so it is never more than one frame from a fresh build.
    BoundsTree mDynamicTree;
    BoundsTree mDynamicTreeNext;
    // Payload, parallel to mDynamicBounds.
    std::vector<MeshRenderer*> mDynamicRenderers;
    HashMap<const MeshRenderer*, u32> mDynamicRendererIndex;
    std::vector<AABB> mDynamicBounds;
    // The background build reads its own copy; sharing the live array would be a data race.
    std::vector<AABB> mDynamicBuildBounds;
    std::vector<u32> mDynamicCandidates;
    JobGroup mDynamicBuildJob;
    bool mDynamicBuildPending = false;
    // Ticks once per buildRenderList(); SceneBVH::justEnteredView()'s clock.
    u32 mFrameNumber = 0;
    std::vector<SceneBVH::Hit> mStaticHits;       // scratch, reused every buildRenderList() call
    std::vector<SceneBVH::Hit> mBystanderHits;
    std::vector<SceneBVH::Hit> mShadowStaticHits; // scratch, reused every buildShadowList() call
    std::vector<MeshRenderer*> mDynamicHits;   // scratch, reused every buildRenderList() call
    std::vector<MeshRenderer*> mBystanderDynamicHits;
    // Deliberately excluded from the tree (currently skinned meshes).
    std::vector<MeshRenderer*> mDynamicLinearFallback;
    // Ids, not pointers: stale ids are ignored safely after remove/destroy.
    std::vector<u64> mDynamicBoundsDirty;
    bool mRunningInEditor = false;
    bool mAsyncMeshLoad = false;
    bool mStaticCullingEnabled = true;
    bool mOcclusionQueryEnabled = false;
    bool mDynamicCullingEnabled = false;
    // Rebuild before any camera or shadow query, or entries may hold a freed MeshRenderer pointer.
    bool mStaticIndexDirty = true;
    bool mDynamicIndexDirty = true;

    // One shared unit cube [-1, 1]^3 stands in for each entry's geometry; a box conservatively bounds the mesh.
    PipelineHandle mOcclusionPipeline;
    // One buffer written once per frame at distinct aligned offsets, so no write overlaps an in-flight read and the CPU never stalls.
    BufferHandle mOcclusionBlock;
    u32 mOcclusionBlockCapacity = 0; // entries, not bytes
    u32 mOcclusionBlockStride = 0;   // bytes between entries, alignment-padded
    BufferHandle mOcclusionCubeVertices;
    BufferHandle mOcclusionCubeIndices;
    // Scratch: cleared, not freed.
    std::vector<OcclusionCandidate> mOcclusionCandidates;

    // Resolved on the GPU into a mapped buffer and read a frame later; two buffers, one written while the other holds last frame's.
    // Avoids per-query driver polling, which flushes the command buffer (same as the reference's OcclusionCulling_Resolve).
    static constexpr u32 kOcclusionResultBuffers = 2;
    BufferHandle mOcclusionResults[kOcclusionResultBuffers];
    // Owner entry of each result slot, recorded at launch; the buffer is a flat array.
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
    // Flat event lists avoid scanning ComponentType::Count slots per object.
    std::vector<Component*> mUpdateComponents;
    std::vector<Component*> mLateUpdateComponents;
    bool mComponentListsDirty = false;
    std::vector<Animator*> mAnimators;
    std::vector<BoneAttachment*> mBoneAttachments;
    std::vector<Camera*> mCameras;
    std::vector<Light*> mLights;
    std::vector<ReflectionProbe*> mReflectionProbes;
    std::vector<Collider*> mColliders;
    CollisionWorld mCollisionWorld;

    struct CachedContactPoint
    {
        Math::vec3 position{0.0f};
        f32 normalImpulse = 0.0f;
        f32 tangentImpulse[2] = {0.0f, 0.0f};
    };
    struct CachedContactPair
    {
        static constexpr u32 MaxPoints = Physics::ContactManifold::MaxPoints * 4;
        CachedContactPoint points[MaxPoints];
        u32 count = 0;
        u32 lastStep = 0;
        bool reported = false;
        Physics::RigidBody* bodyA = nullptr;
        Physics::RigidBody* bodyB = nullptr;
    };
    struct BulletSweep
    {
        Physics::RigidBody* body = nullptr;
        Math::vec3 previousPosition{0.0f};
    };
    static u64 pairKey(const Physics::RigidBody& a, const Physics::RigidBody& b);
    void rebuildStaticBroadphase();
    void warmStartFromCache(const CachedContactPair* cached, Physics::ContactManifold& manifold);
    void storeInCache(const Physics::RigidBody& a, const Physics::RigidBody& b,
                      const Physics::ContactManifold& manifold);
    void emitContactExits();
    u32 islandRoot(u32 index);
    void propagateSleep();
    void solveBulletSweeps();
    void dispatchContactEvents();

    std::vector<Physics::RigidBody*> mRigidBodies;
    bool mPhysicsStepping = false;
    bool mDispatchingContactEvents = false;
    u32 mNextBodyKey = 1;
    Physics::Broadphase mDynamicBroadphase;
    BoundsTree mStaticBroadphase;
    std::vector<AABB> mStaticBounds;
    std::vector<Physics::RigidBody*> mStaticBodies;
    std::vector<Physics::BroadphaseProxy> mDynamicProxies;
    std::vector<Physics::BroadphasePair> mDynamicPairs, mPairs;
    std::vector<u32> mStaticCandidates;
    bool mStaticBroadphaseDirty = true;
    std::vector<Physics::ContactManifold> mManifolds;
    std::vector<Physics::Contact> mContacts;
    std::vector<Physics::Joint*> mJoints;
    std::vector<Physics::Joint*> mJointComponents;
    std::vector<Physics::ContactEventInfo> mContactEventQueue, mContactEventsDispatching;
    HashMap<u64, CachedContactPair> mContactCache;
    std::vector<u32> mIslandParent;
    std::vector<u8> mIslandAwake;
    std::vector<BulletSweep> mBulletSweeps;
    Physics::ContactSolver mContactSolver;
    Math::vec3 mGravity{0.0f, -9.81f, 0.0f};
    f32 mFixedStep = 1.0f / 120.0f;
    f32 mPhysicsAccumulator = 0.0f;
    u32 mMaxPhysicsStepsPerUpdate = 8;
    u32 mPhysicsStepIndex = 0;
    u32 mContactPersistence = 2;
    f32 mContactMargin = 0.04f;
    Physics::ContactEventCallback mContactEventCallback = nullptr;
    void* mContactEventUserData = nullptr;
    Physics::PhysicsStepCallback mPhysicsStepCallback = nullptr;
    void* mPhysicsStepUserData = nullptr;

    // Holes appear in mAgents while updateAgents() runs (agent destroyed by its own callback) and are swept afterwards.
    std::vector<Agent*> mAgents;
    bool mAgentsUpdating = false;
    bool mAgentsDirty = false;

    // The live-subset shapes are derived from the components by rebuildObstacleGroup().
    std::vector<Obstacle*> mObstacleComponents;
    std::vector<AI::Obstacle*> mObstacleGroup;

    std::vector<GameObject*> mDebugObjects;
    std::vector<PendingAdd> mPendingAdd;
    std::vector<GameObject*> mPendingRemove;
    std::vector<GameObject*> mPendingDestroy;
    std::vector<GameObject*> mDetached;
    f32 mDeltaTime = 0.0f;
};

} // namespace Radion

#endif // RADION_SCENE_H
