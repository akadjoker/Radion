#include "PCH.h"

#include "Scene.h"
#include "Thread.h"

#include "Agent.h"
#include "AssetManager.h"
#include "AudioEngine.h"
#include "DebugDraw3D.h"
#include "FileSystem.h"
#include "GPUCaps.h"
#include "Log.h"
#include "MaterialManager.h"
#include "ObstacleComponent.h"
#include "ParticleEffectPool.h"
#include "GPUProfiler.h"
#include "Profiler.h"
#include "ScriptCache.h"
#include "RenderList.h"
#include "UiControls.h"
#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"
#include "dynamics/Joint.h"

#include <cmath>
#include <cstring>
#include <glm/gtc/matrix_transform.hpp>
#include <limits>
#include <nlohmann/json.hpp>

namespace Radion
{

using namespace Physics;

namespace
{
template <typename T> void removePointer(std::vector<T*>& values, T* value)
{
    for (usize i = 0; i < values.size(); ++i)
    {
        if (values[i] != value)
            continue;
        values[i] = values.back();
        values.pop_back();
        return;
    }
}

bool queued(const std::vector<GameObject*>& queue, const GameObject* object)
{
    return std::find(queue.begin(), queue.end(), object) != queue.end();
}

// The listener rides the active camera. Orientation goes with the position:
// a listener that only moves pans every spatial voice wrongly the moment
// the camera turns on the spot.
void syncAudioListener(const Camera* camera)
{
    AudioEngine& audio = Audio();
    if (!audio.ready())
        return;
    const GameObject* owner = camera ? camera->owner() : nullptr;
    if (!owner)
        return;
    audio.setListenerPosition(owner->globalPosition());
    audio.setListenerOrientation(owner->forward(), owner->up());
}

// Nearest live, captured probe to `position` - render/'s RenderInstance
// carries a resolved RenderProbe rather than a ReflectionProbe* because
// render/ cannot depend on scene/'s component types. A default-constructed
// (invalid cubemap) result when no probe qualifies is exactly what tells
// ForwardPass to fall back to the frame's single default probe.
RenderProbe resolveNearestProbe(const std::vector<ReflectionProbe*>& probes,
                                const glm::vec3& position)
{
    RenderProbe result;
    const ReflectionProbe* nearest = nullptr;
    f32 nearestDistSq = 0.0f;
    for (const ReflectionProbe* candidate : probes)
    {
        if (!candidate->active() || !candidate->probe().ready())
            continue;
        const EnvironmentProbe& env = candidate->probe();
        // A probe only influences objects near it. Without this gate, one
        // small local probe placed anywhere would win "nearest" for every
        // object in the scene, however far away, and silently steal
        // reflections from Engine's own single global probe
        // (frame.environmentCube), which ForwardPass only falls back to when
        // nothing here qualifies.
        //
        // influenceRadius answers that on its own, which is what lets a probe
        // serve its object while leaving extents at zero - and zero extents
        // is not "covers nowhere", it is the shader's no-parallax path
        // (EnvironmentProbe::extents' own doc): a plain mirror of what was
        // captured, sampled straight. A probe that sets only extents keeps
        // being selected by its box exactly as before.
        const glm::vec3 offset = position - env.position;
        if (env.influenceRadius > 0.0f)
        {
            if (glm::dot(offset, offset) > env.influenceRadius * env.influenceRadius)
                continue;
        }
        else if (env.extents.x <= 0.0f || env.extents.y <= 0.0f || env.extents.z <= 0.0f)
            continue;
        else if (glm::abs(offset.x) > env.extents.x || glm::abs(offset.y) > env.extents.y ||
                 glm::abs(offset.z) > env.extents.z)
            continue;
        const f32 distSq = glm::dot(offset, offset);
        if (!nearest || distSq < nearestDistSq)
        {
            nearest = candidate;
            nearestDistSq = distSq;
        }
    }
    if (nearest)
    {
        const EnvironmentProbe& probe = nearest->probe();
        result.cubemap = probe.cubemap();
        result.sampler = probe.sampler();
        result.position = probe.position;
        result.extents = probe.extents;
        result.mipCount = probe.mipCount();
        result.intensity = probe.intensity;
    }
    return result;
}

// One dynamic renderer into the camera's list, whole mesh at a time - the
// octree indexes a renderer, not a submesh, so there is nothing to narrow
// down the way the BVH path does. Reached from four places in
// buildRenderList() (octree hits, the skinned fallback, statics with static
// culling off, and the no-octree scan).
void submitDynamicRenderer(MeshRenderer* renderer, RenderList& list, AssetManager& assets,
                           MaterialManager& materials,
                           const std::vector<ReflectionProbe*>& probes)
{
    GameObject* object = renderer->owner();
    if (!renderer->active() || !object->isActiveAndVisibleInHierarchy() ||
        !renderer->mesh().valid())
        return;
    Mesh* mesh = assets.getMesh(renderer->mesh());
    if (!mesh)
        return;
    Material* overrides = const_cast<Material*>(renderer->materialOverrides());
    const u32 overrideCount = renderer->materialOverrideCount();
    if (overrides)
        for (u32 i = 0; i < overrideCount; ++i)
        {
            materials.resolvePipeline(overrides[i], mesh->colorLayout);
            materials.sync(overrides[i]);
        }
    else
        for (Material& material : mesh->materials)
        {
            materials.resolvePipeline(material, mesh->colorLayout);
            materials.sync(material);
        }
    Animator* animator = object->getComponent<Animator>();
    const std::vector<glm::mat4>* palette =
        animator && animator->active() ? &animator->palette() : nullptr;
    const std::vector<glm::mat4>* prevPalette =
        animator && animator->active() ? &animator->prevPalette() : nullptr;
    const RenderProbe probe = resolveNearestProbe(probes, object->globalPosition());
    list.submit(renderer->mesh(), *mesh, object->globalTransform(), overrides, overrideCount,
                palette, &probe, &object->previousGlobalTransform(), prevPalette);
}

// True when this caster's box is entirely behind one of the cascade's own
// cull planes - the slice cannot see it, whatever the frustum test said.
bool outsideCasterVolume(const std::vector<Plane>* casterPlanes, const AABB& bounds)
{
    if (!casterPlanes)
        return false;
    const glm::vec3 center = bounds.center();
    const glm::vec3 extents = bounds.extents();
    for (const Plane& plane : *casterPlanes)
    {
        const f32 distance = glm::dot(plane.normal, center) + plane.d;
        const f32 radius = glm::dot(glm::abs(plane.normal), extents);
        if (distance + radius < 0.0f)
            return true;
    }
    return false;
}

// Depth-buffer pixel -> view-space position, for pickSurface()'s centre pixel
// and its 3x3 neighbours.
glm::vec3 viewPositionFromDepth(s32 x, s32 y, f32 depth, u32 depthWidth, u32 depthHeight,
                                const glm::mat4& inverseProjection)
{
    const glm::vec2 uv((static_cast<f32>(x) + 0.5f) / static_cast<f32>(depthWidth),
                       (static_cast<f32>(y) + 0.5f) / static_cast<f32>(depthHeight));
    const glm::vec4 clip(uv * 2.0f - 1.0f, depth * 2.0f - 1.0f, 1.0f);
    const glm::vec4 view = inverseProjection * clip;
    return glm::vec3(view) / view.w;
}

struct OcclusionBlock
{
    glm::mat4 viewProjection;
    glm::mat4 model;
};

// One shared cube, [-1, 1]^3 - every entry's own model matrix scales it to
// that entry's world AABB (extents already half-size, matching this).
const glm::vec3 kOcclusionCubeVertices[8] = {
    {-1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, -1.0f}, {-1.0f, 1.0f, -1.0f},
    {-1.0f, -1.0f, 1.0f},  {1.0f, -1.0f, 1.0f},  {1.0f, 1.0f, 1.0f},  {-1.0f, 1.0f, 1.0f},
};
constexpr u16 kOcclusionCubeIndices[36] = {
    0, 1, 2, 0, 2, 3, // back
    4, 6, 5, 4, 7, 6, // front
    0, 4, 5, 0, 5, 1, // bottom
    3, 2, 6, 3, 6, 7, // top
    0, 3, 7, 0, 7, 4, // left
    1, 5, 6, 1, 6, 2, // right
};
} // namespace

Scene::Scene() : mRoot("Scene")
{
    mRoot.mScene = this;
    ParticleEffectPool::getSingleton().initialize(*this);
    mCollisionWorld.initialize(*this);
}

Scene::~Scene()
{
    if (mDynamicBuildPending)
    {
        Jobs().wait(mDynamicBuildJob);
        mDynamicBuildPending = false;
    }

    for (const PendingAdd& pending : mPendingAdd)
        delete pending.object;
    for (GameObject* object : mDetached)
        delete object;
    mRoot.deleteChildrenRaw();
    mRoot.mScene = nullptr;
    // Whatever is left is loose bodies (tests, ragdolls, characters) never
    // attached to a GameObject - deleteChildrenRaw() above already unregistered
    // every attached one through componentRemoved(). Detach them rather than
    // leave them pointing at a Scene about to go away.
    clearPhysics();
    clearAI();

    // mStaticIndex's own destructor releases the per-entry queries; only the
    // shared occlusion-pass resources (created lazily, see
    // setupOcclusionQueryResources()) are this class's own to free.
    if (GPU::ready())
    {
        GPU& gpu = GPU::getSingleton();
        gpu.destroy(mOcclusionPipeline);
        gpu.destroy(mOcclusionBlock);
        gpu.destroy(mOcclusionCubeVertices);
        gpu.destroy(mOcclusionCubeIndices);
    }
}

GameObject& Scene::root()
{
    return mRoot;
}
const GameObject& Scene::root() const
{
    return mRoot;
}

GameObject* Scene::createGameObject(const std::string& name, GameObject* parent)
{
    GameObject* object = new GameObject(name);
    object->mId = mNextId++;
    if (add(object, parent))
    {
        stampId(object);
        return object;
    }
    delete object;
    return nullptr;
}

GameObject* Scene::createGameObject(u64 id, const std::string& name, GameObject* parent)
{
    if (id == 0 || mObjectsById.find(id) != mObjectsById.end())
    {
        Log::error("Scene: cannot restore GameObject '%s' with id %llu: %s", name.c_str(),
                   static_cast<unsigned long long>(id),
                   id == 0 ? "id 0 is reserved" : "id already in use");
        return nullptr;
    }
    GameObject* object = new GameObject(name);
    object->mId = id;
    if (add(object, parent))
    {
        stampId(object);
        if (id >= mNextId)
            mNextId = id + 1;
        return object;
    }
    delete object;
    return nullptr;
}

GameObject* Scene::findGameObject(u64 id) const
{
    const auto entry = mObjectsById.find(id);
    return entry != mObjectsById.end() ? entry->second : nullptr;
}

GameObject* Scene::findGameObject(const std::string& name) const
{
    for (GameObject* object : mObjects)
        if (object && object->name() == name)
            return object;
    return nullptr;
}

usize Scene::countByTag(const std::string& tag) const
{
    usize count = 0;
    for (GameObject* object : mObjects)
        if (object && object->tag() == tag)
            ++count;
    return count;
}

void Scene::stampId(GameObject* object)
{
    // Anything born here already has an id nothing else can hold. Only a
    // branch that came from another Scene through add() can arrive holding
    // one of ours, or none at all, and it is renumbered rather than refused:
    // add() has already accepted it by this point.
    const auto clash = mObjectsById.find(object->mId);
    if (object->mId == 0 || (clash != mObjectsById.end() && clash->second != object))
    {
        const u64 previous = object->mId;
        object->mId = mNextId++;
        if (previous != 0)
            Log::warning("Scene: GameObject '%s' arrived with id %llu, already taken here; "
                         "renumbered to %llu",
                         object->name().c_str(), static_cast<unsigned long long>(previous),
                         static_cast<unsigned long long>(object->mId));
    }
    mObjectsById[object->mId] = object;
}

// Deleting an object takes its children with it (~GameObject), so the whole
// branch has to leave the map, not just the root of it.
void Scene::forgetIdBranch(GameObject* object)
{
    for (usize i = 0; i < object->childCount(); ++i)
        forgetIdBranch(object->child(i));
    const auto entry = mObjectsById.find(object->mId);
    if (entry != mObjectsById.end() && entry->second == object)
        mObjectsById.erase(entry);
}

bool Scene::add(GameObject* object, GameObject* parent)
{
    if (!object || object == &mRoot || object->parent())
        return false;
    for (const PendingAdd& pending : mPendingAdd)
        if (pending.object == object)
            return false;
    GameObject* destination = parent ? parent : &mRoot;
    if (destination != &mRoot && destination->root() != &mRoot)
    {
        bool parentPending = false;
        for (const PendingAdd& pending : mPendingAdd)
            parentPending |=
                pending.object == destination || pending.object->isAncestorOf(destination);
        if (!parentPending)
            return false;
        return destination->addChildRaw(object);
    }
    removePointer(mDetached, object);
    mPendingAdd.push_back({object, destination});
    return true;
}

bool Scene::remove(GameObject* object)
{
    if (!object || object == &mRoot || !object->parent() || queued(mPendingRemove, object) ||
        object->mPendingDestroyQueued)
        return false;
    mPendingRemove.push_back(object);
    return true;
}

bool Scene::destroy(GameObject* object)
{
    if (!object || object == &mRoot || object->mPendingDestroyQueued)
        return false;
    if (!object->parent() &&
        std::find(mDetached.begin(), mDetached.end(), object) == mDetached.end())
        return false;
    object->mPendingDestroyQueued = true;
    mPendingDestroy.push_back(object);
    return true;
}

bool Scene::reparent(GameObject* object, GameObject* parent)
{
    GameObject* destination = parent ? parent : &mRoot;
    if (!object || object == &mRoot || destination == object || object->mScene != this ||
        destination->mScene != this || !object->parent() || object->isAncestorOf(destination) ||
        queued(mPendingRemove, object) || object->mPendingDestroyQueued)
        return false;

    if (object->parent() == destination)
        return true;

    object->parent()->removeChildRaw(object);
    return destination->addChildRaw(object);
}

void Scene::setActiveCamera(Camera* camera)
{
    bool pending = false;
    for (const PendingAdd& entry : mPendingAdd)
        pending |= camera &&
                   (entry.object == camera->owner() || entry.object->isAncestorOf(camera->owner()));
    if (camera && !pending && std::find(mCameras.begin(), mCameras.end(), camera) == mCameras.end())
    {
        Log::warning("Scene: active camera is not registered");
        return;
    }
    mActiveCamera = camera;
}

Camera* Scene::activeCamera() const
{
    return mActiveCamera;
}

bool Scene::saveCamera(const std::string& filename) const
{
    if (!mActiveCamera || !mActiveCamera->owner())
        return false;

    GameObject* object = mActiveCamera->owner();
    const glm::vec3 position = object->globalPosition();
    const glm::quat rotation = object->globalRotation();

    nlohmann::json root;
    root["position"] = {position.x, position.y, position.z};
    root["rotation"] = {rotation.x, rotation.y, rotation.z, rotation.w};

    if (!FileSystem::getSingleton().writeText(filename, root.dump(4) + '\n'))
    {
        Log::error("Scene: could not write '%s'", filename.c_str());
        return false;
    }
    Log::info("Scene: saved camera to '%s'", filename.c_str());
    return true;
}

bool Scene::loadCamera(const std::string& filename)
{
    if (!mActiveCamera || !mActiveCamera->owner())
        return false;

    const std::string text = FileSystem::getSingleton().readText(filename);
    if (text.empty())
        return false;

    nlohmann::json root;
    try
    {
        root = nlohmann::json::parse(text);
    }
    catch (const std::exception& error)
    {
        Log::error("Scene: '%s' is not valid JSON: %s", filename.c_str(), error.what());
        return false;
    }

    const auto position = root.find("position");
    const auto rotation = root.find("rotation");
    if (position == root.end() || !position->is_array() || position->size() != 3 ||
        rotation == root.end() || !rotation->is_array() || rotation->size() != 4)
        return false;

    GameObject* object = mActiveCamera->owner();
    object->setPosition(
        glm::vec3((*position)[0].get<f32>(), (*position)[1].get<f32>(), (*position)[2].get<f32>()));
    object->setRotation(glm::quat((*rotation)[3].get<f32>(), (*rotation)[0].get<f32>(),
                                  (*rotation)[1].get<f32>(), (*rotation)[2].get<f32>()));

    Log::info("Scene: loaded camera from '%s'", filename.c_str());
    return true;
}

void Scene::update(f32 deltaTime)
{
    RADION_PROFILE_SCOPE("Scene update");
    mDeltaTime = std::isfinite(deltaTime) && deltaTime >= 0 ? deltaTime : 0;
    flushChanges();
    {
        for (GameObject* object : mObjects)
        {
            if (!object->disposed())
            {
                object->mPreviousGlobalTransform = object->globalTransform();
                object->mPreviousGlobalTransformValid = true;
            }
        }
    }
    // Layout and hit-test every UI control once, before the component update
    // below draws them: a control renders the rectangle this pass just gave
    // it, not the one from last frame.
    {
        RADION_PROFILE_SCOPE("UI update");
        UiSystems().refresh();
    }
    // Capture the count: a component attached from on_start/on_update joins
    // the list immediately, but must not run until the next frame. Removal
    // writes a tombstone, so callbacks can safely remove themselves or one
    // another without invalidating this iteration.
    {
        RADION_PROFILE_SCOPE("Component update");
        const usize updateCount = mUpdateComponents.size();
        for (usize i = 0; i < updateCount; ++i)
        {
            Component* component = mUpdateComponents[i];
            GameObject* object = component ? component->owner() : nullptr;
            if (object && object->isActiveInHierarchy() && !object->disposed())
                object->updateComponent(component, mDeltaTime);
        }
    }
    {
        RADION_PROFILE_SCOPE("Animation update");
        for (Animator* animator : mAnimators)
            if (animator->active() && animator->owner()->isActiveInHierarchy())
                animator->update(mDeltaTime);
        for (BoneAttachment* attachment : mBoneAttachments)
            if (attachment->active() && attachment->owner()->isActiveInHierarchy())
                attachment->update();
    }
    // Obstacle transforms follow their owner unconditionally, in and out of
    // Play - the shape has to track the gizmo while the scene is being
    // edited, not just while running (2b.2). Runs before AI update so this
    // same frame's ObstacleAvoidanceBehavior reads the current position.
    {
        RADION_PROFILE_SCOPE("Obstacle transform sync");
        for (Obstacle* obstacle : mObstacleComponents)
            obstacle->pushOwnerTransform();
        // Nothing announces a component being switched off, so the live set
        // is refilled here rather than tracked. Clearing a vector and
        // refilling it costs no allocation after the first frame.
        rebuildObstacleGroup();
    }
    // AI runs after Animation update and before Physics step, same reasoning
    // as the physics block below: an order given by script in onUpdate reaches
    // the agent the same frame, and the velocity AI produces is integrated by
    // physics the same frame instead of lagging by one. In the editor agents
    // only track their owner's pose, so nothing steers while it is being
    // placed and Play starts from where it was left.
    {
        RADION_PROFILE_SCOPE("AI update");
        for (Agent* agent : mAgents)
            if (agent->simulating() && agent->ownerMoved())
                agent->pushOwnerPose();
        if (!mRunningInEditor)
        {
            updateAgents(mDeltaTime);
            for (Agent* agent : mAgents)
                if (agent->simulating() && agent->owner())
                    agent->pullAgentPose();
        }
    }
    // Physics runs after Component update, so this frame's scripted forces
    // and impulses (applied from onUpdate) are already on the bodies before
    // they are integrated, and before Collision contacts and Late component
    // update, so a trigger, a character sweep, or a follow camera all read
    // the pose the simulation just produced instead of lagging it by a
    // frame. In the editor the bodies only track their objects, so nothing
    // falls while it is being placed and Play starts from where it was left.
    {
        RADION_PROFILE_SCOPE("Physics step");
        for (Physics::RigidBody* body : mRigidBodies)
            if (body->simulating() && body->ownerMoved())
                body->pushOwnerPose();
        if (!mRunningInEditor)
        {
            updatePhysics(mDeltaTime);
            for (Physics::RigidBody* body : mRigidBodies)
                if (body->simulating() && body->isDynamic() && body->owner())
                    body->pullBodyPose();
        }
    }
    {
        RADION_PROFILE_SCOPE("Collision contacts");
        mCollisionWorld.step();
    }
    {
        RADION_PROFILE_SCOPE("Late component update");
        const usize lateUpdateCount = mLateUpdateComponents.size();
        for (usize i = 0; i < lateUpdateCount; ++i)
        {
            Component* component = mLateUpdateComponents[i];
            GameObject* object = component ? component->owner() : nullptr;
            if (object && object->isActiveInHierarchy() && !object->disposed())
                object->lateUpdateComponent(component, mDeltaTime);
        }
    }

    {
        for (GameObject* object : mObjects)
            if (object->disposed() && (!object->parent() || !object->parent()->disposed()) &&
                !object->mPendingDestroyQueued)
            {
                object->mPendingDestroyQueued = true;
                mPendingDestroy.push_back(object);
            }
    }

    ParticleEffectPool::getSingleton().reclaim();

    // After late update, so a camera controller that moved this frame is
    // already where the listener should hear from.
    {
        RADION_PROFILE_SCOPE("Audio update");
        syncAudioListener(mActiveCamera);
        Audio().update();
    }

    flushChanges();
    compactComponentLists();
}

f32 Scene::deltaTime() const
{
    return mDeltaTime;
}
usize Scene::gameObjectCount() const
{
    return mObjects.size();
}
usize Scene::renderableCount() const
{
    return mRenderers.size() + mTerrains.size() + mRoads.size() + mForests.size();
}
usize Scene::animatedCount() const
{
    return mAnimators.size();
}
usize Scene::cameraCount() const
{
    return mCameras.size();
}
usize Scene::lightCount() const
{
    return mLights.size();
}

void Scene::setSunLight(DirectionalLight* light)
{
    bool pending = false;
    for (const PendingAdd& entry : mPendingAdd)
        pending |=
            light && (entry.object == light->owner() || entry.object->isAncestorOf(light->owner()));
    if (light && !pending &&
        std::find(mLights.begin(), mLights.end(), static_cast<Light*>(light)) == mLights.end())
    {
        Log::warning("Scene: sun light is not registered");
        return;
    }
    mSunLight = light;
}

DirectionalLight* Scene::sunLight() const
{
    return mSunLight;
}

DirectionalLight* Scene::electedSunLight() const
{
    if (mSunLight)
        return mSunLight;
    for (Light* light : mLights)
    {
        if (light->lightType() == LightType::Directional)
            return static_cast<DirectionalLight*>(light);
    }
    return nullptr;
}

void Scene::rebuildStaticIndex()
{
    // add()/addComponent() only queue - a MeshRenderer created right before
    // this call has not reached mRenderers yet, only mPendingAdd. Without
    // this the BVH silently builds from whatever was already flushed, which
    // right after loading a level is nothing.
    flushChanges();
    mStaticIndex.build(mRenderers);
    mStaticIndexDirty = false;

    // Every static material's pipeline, resolved once here rather than left
    // to whichever list happens to submit it first. buildRenderList() and
    // buildShadowList() both narrow the static set by their own frustum (the
    // camera's, a cascade's) before touching a single material - a submesh
    // neither has looked at yet still needs a valid material.pipeline the
    // first time either one finally does, or emitSubmesh() silently drops it.
    AssetManager& assets = Assets();
    MaterialManager& materials = MaterialManager::getSingleton();
    for (MeshRenderer* renderer : mRenderers)
    {
        GameObject* object = renderer->owner();
        if (!object || !object->isStatic() || !renderer->mesh().valid())
            continue;
        Mesh* mesh = assets.getMesh(renderer->mesh());
        if (!mesh)
            continue;
        for (Material& material : mesh->materials)
        {
            materials.resolvePipeline(material, mesh->colorLayout);
            materials.sync(material);
        }
        const u32 overrideCount = renderer->materialOverrideCount();
        if (Material* overrides = const_cast<Material*>(renderer->materialOverrides()))
            for (u32 i = 0; i < overrideCount; ++i)
            {
                materials.resolvePipeline(overrides[i], mesh->colorLayout);
                materials.sync(overrides[i]);
            }
    }
}

void Scene::rebuildDynamicIndex()
{
    // Same idea as rebuildStaticIndex(): add()/addComponent() only queue, so
    // flush first or the octree builds from a half-populated mRenderers.
    flushChanges();
    clearDynamicBoundsQueue();

    // A rebuild may already be in flight against the OLD set. Waiting for it
    // here before touching the arrays is the whole of the thread safety:
    // nothing else in this class ever touches them while a job is running.
    if (mDynamicBuildPending)
    {
        Jobs().wait(mDynamicBuildJob);
        mDynamicBuildPending = false;
    }

    mDynamicRenderers.clear();
    mDynamicRendererIndex.clear();
    mDynamicBounds.clear();
    mDynamicLinearFallback.clear();

    AssetManager& assets = Assets();
    for (MeshRenderer* renderer : mRenderers)
    {
        GameObject* object = renderer ? renderer->owner() : nullptr;
        if (!object || object->isStatic() || !renderer->mesh().valid())
            continue;
        Mesh* mesh = assets.getMesh(renderer->mesh());
        if (!mesh)
            continue;
        // A skinned mesh's box follows the pose, not the transform, so
        // nothing the dirty queue reports would keep it current. It stays on
        // the linear path, cached here rather than rediscovered per frame.
        if (mesh->isSkinned())
        {
            mDynamicLinearFallback.push_back(renderer);
            continue;
        }
        mDynamicRendererIndex[renderer] = static_cast<u32>(mDynamicRenderers.size());
        mDynamicRenderers.push_back(renderer);
        mDynamicBounds.push_back(transformAABB(mesh->bounds, object->globalTransform()));
    }

    mDynamicTree.build(mDynamicBounds.data(), static_cast<u32>(mDynamicBounds.size()));
    mDynamicIndexDirty = false;
}

void Scene::refreshDynamicBounds()
{
    AssetManager& assets = Assets();
    for (u64 id : mDynamicBoundsDirty)
    {
        GameObject* object = findGameObject(id);
        if (object)
            object->mDynamicBoundsQueued = false;
        MeshRenderer* renderer = object ? object->findComponent<MeshRenderer>() : nullptr;
        if (!renderer || !object || object->mScene != this || object->isStatic() ||
            !renderer->mesh().valid())
            continue;
        const auto found = mDynamicRendererIndex.find(renderer);
        if (found == mDynamicRendererIndex.end())
            continue;
        Mesh* mesh = assets.getMesh(renderer->mesh());
        if (!mesh)
            continue;
        mDynamicBounds[found->second] = transformAABB(mesh->bounds, object->globalTransform());
    }
    mDynamicBoundsDirty.clear();
}

void Scene::updateDynamicTree()
{
    // Last frame's rebuild, if it has finished. Checked rather than waited
    // on: the point of firing it in the background is that this frame never
    // blocks for it. A build that is still going simply gets picked up next
    // frame instead.
    if (mDynamicBuildPending && Jobs().finished(mDynamicBuildJob))
    {
        std::swap(mDynamicTree, mDynamicTreeNext);
        mDynamicBuildPending = false;
    }

    // The swapped-in tree was built from a snapshot taken a frame ago, so its
    // SHAPE is a frame stale - but the boxes it answers with are this
    // frame's, because refit runs after the swap and before any query. A
    // stale shape costs a little traversal, never a wrong answer.
    mDynamicTree.refit(mDynamicBounds.data(), static_cast<u32>(mDynamicBounds.size()));

    if (mDynamicBuildPending || mDynamicBounds.empty())
        return;

    // Its own copy, so the main thread is free to keep updating the live
    // boxes while this runs.
    mDynamicBuildBounds = mDynamicBounds;
    mDynamicTreeNext.setLeafCapacity(mDynamicTree.leafCapacity());
    Jobs().enqueue(mDynamicBuildJob, &Scene::buildDynamicTreeJob, this);
    mDynamicBuildPending = true;
}

void Scene::buildDynamicTreeJob(void* userData)
{
    Scene& scene = *static_cast<Scene*>(userData);
    scene.mDynamicTreeNext.build(scene.mDynamicBuildBounds.data(),
                                 static_cast<u32>(scene.mDynamicBuildBounds.size()));
}

void Scene::setDynamicLeafCapacity(u32 capacity)
{
    if (mDynamicTree.leafCapacity() == capacity)
        return;
    mDynamicTree.setLeafCapacity(capacity);
    mDynamicIndexDirty = true;
}

void Scene::setDynamicCullingEnabled(bool enabled)
{
    if (mDynamicCullingEnabled == enabled)
        return;
    mDynamicCullingEnabled = enabled;
    if (enabled)
        mDynamicIndexDirty = true; // build it on the next buildRenderList()
    else
        clearDynamicBoundsQueue();
}

void Scene::queueDynamicBoundsUpdate(GameObject* object)
{
    if (!object || !mDynamicCullingEnabled || mDynamicIndexDirty ||
        mDynamicRenderers.empty() || object->mDynamicBoundsQueued || object->id() == 0 ||
        object->isStatic() || !object->findComponent<MeshRenderer>())
        return;
    object->mDynamicBoundsQueued = true;
    mDynamicBoundsDirty.push_back(object->id());
}

void Scene::clearDynamicBoundsQueue()
{
    for (u64 id : mDynamicBoundsDirty)
        if (GameObject* object = findGameObject(id))
            object->mDynamicBoundsQueued = false;
    mDynamicBoundsDirty.clear();
}

void Scene::reapplyHiddenSubmeshes()
{
    for (MeshRenderer* renderer : mRenderers)
        if (renderer && !renderer->hiddenSubmeshes().empty())
            renderer->applyHiddenSubmeshes();
}

void Scene::invalidateSpatialIndexes()
{
    mStaticIndexDirty = true;
    mDynamicIndexDirty = true;
    clearDynamicBoundsQueue();
}

bool Scene::setupOcclusionQueryResources()
{
    if (mOcclusionPipeline.valid())
        return true;

    GPU& gpu = GPU::getSingleton();

    // Stride between entries in mOcclusionBlock has to respect the device's
    // own uniform-buffer offset alignment - binding at an arbitrary byte
    // offset is not portable even though this particular driver tolerates
    // it (see GPUCaps).
    const u32 alignment = glm::max(gpu.caps().uniformOffsetAlignment, 4u);
    mOcclusionBlockStride = ((sizeof(OcclusionBlock) + alignment - 1) / alignment) * alignment;
    if (!ensureOcclusionBlockCapacity(64))
        return false;

    BufferDesc verticesDesc;
    verticesDesc.size = sizeof(kOcclusionCubeVertices);
    verticesDesc.usage = BufferVertex;
    verticesDesc.residency = Residency::Static;
    verticesDesc.stride = sizeof(glm::vec3);
    verticesDesc.data = kOcclusionCubeVertices;
    verticesDesc.debugName = "occlusion.cube.vertices";
    mOcclusionCubeVertices = gpu.createBuffer(verticesDesc);

    BufferDesc indicesDesc;
    indicesDesc.size = sizeof(kOcclusionCubeIndices);
    indicesDesc.usage = BufferIndex;
    indicesDesc.residency = Residency::Static;
    indicesDesc.data = kOcclusionCubeIndices;
    indicesDesc.debugName = "occlusion.cube.indices";
    mOcclusionCubeIndices = gpu.createBuffer(indicesDesc);

    const std::string& vertexSource = Assets().loadShader("occlusion_query.vert");
    const std::string& fragmentSource = Assets().loadShader("occlusion_query.frag");
    if (vertexSource.empty() || fragmentSource.empty())
        return false;

    VertexLayout layout;
    layout.streamCount = 1;
    layout.streams[StreamPosition].stride = sizeof(glm::vec3);
    layout.attribCount = 1;
    layout.attribs[0] = {0, StreamPosition, 0, AttribFormat::Float3};

    PipelineDesc desc;
    desc.vs = {vertexSource.c_str(), 0, "occlusion_query.vert"};
    desc.fs = {fragmentSource.c_str(), 0, "occlusion_query.frag"};
    desc.layout = layout;
    // Testing coverage, not the box's own visibility, so which way its
    // triangles wind does not matter.
    desc.raster.cull = CullMode::None;
    desc.depth.test = true;
    // Must never perturb the values the depth prepass just wrote - only
    // ever compares against them.
    desc.depth.write = false;
    desc.depth.func = Compare::LessEqual;
    desc.blend.writeRGB = false;
    desc.blend.writeA = false;
    desc.debugName = "occlusion.query";
    mOcclusionPipeline = gpu.createPipeline(desc);

    return mOcclusionPipeline.valid() && mOcclusionBlock.valid() &&
           mOcclusionCubeVertices.valid() && mOcclusionCubeIndices.valid();
}

bool Scene::ensureOcclusionResultCapacity(u32 count)
{
    if (count <= mOcclusionResultCapacity && mOcclusionResults[0].valid())
        return true;

    GPU& gpu = GPU::getSingleton();
    // Anything already resolved into the old buffers is dropped: their
    // mappings are about to go away, and a verdict is only ever allowed to
    // make something MORE visible, so losing one costs a draw and nothing
    // more.
    for (u32 i = 0; i < kOcclusionResultBuffers; ++i)
    {
        gpu.destroy(mOcclusionResults[i]);
        mOcclusionResultOwners[i].clear();
    }

    // Grown in steps rather than to exactly what was asked for, so a scene
    // gaining a few entries a frame does not reallocate and remap every time.
    mOcclusionResultCapacity = glm::max(count + count / 2u, 256u);
    for (u32 i = 0; i < kOcclusionResultBuffers; ++i)
    {
        BufferDesc desc;
        desc.size = static_cast<u64>(mOcclusionResultCapacity) * sizeof(u32);
        desc.usage = BufferReadback;
        desc.debugName = "occlusion.results";
        mOcclusionResults[i] = gpu.createBuffer(desc);
        if (!mOcclusionResults[i].valid())
        {
            mOcclusionResultCapacity = 0;
            return false;
        }
    }
    return true;
}

bool Scene::ensureOcclusionBlockCapacity(u32 count)
{
    if (count <= mOcclusionBlockCapacity && mOcclusionBlock.valid())
        return true;

    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mOcclusionBlock);

    BufferDesc desc;
    desc.size = static_cast<u64>(mOcclusionBlockStride) * count;
    desc.usage = BufferUniform;
    desc.residency = Residency::Dynamic;
    desc.debugName = "occlusion.block";
    mOcclusionBlock = gpu.createBuffer(desc);
    mOcclusionBlockCapacity = mOcclusionBlock.valid() ? count : 0;
    return mOcclusionBlock.valid();
}

void Scene::updateOcclusionQueries(TargetHandle depthTarget, const glm::mat4& viewProjection,
                                   const glm::vec3& cameraPosition)
{
    if (!mOcclusionQueryEnabled || mStaticHits.empty())
        return;
    if (!setupOcclusionQueryResources())
        return;


    // Results are no longer polled at all. A query is resolved into a GPU
    // buffer right after it is issued, and that buffer is read from its own
    // mapping two frames later - by which time the GPU is long past it, so
    // the read is a plain memory access with nothing to synchronise.
    //
    // What this replaced: asking the driver per query whether the result was
    // ready. Even the non-blocking form is not free, because it has to flush
    // the command buffer the query's own end packet is still sitting in, and
    // that flush drags the frame's submission forward into a wait. The fix
    // then was a latency heuristic - wait a frame before asking - which
    // traded freshness for the flush without removing it. The reference does
    // not have the problem at all (wiRenderer's OcclusionCulling_Resolve),
    // and neither does this now.

    // The camera standing inside an entry's own box is the other case
    // apparent size alone does not catch - radius/distance grows without
    // bound as distance goes to zero, so that ratio alone would call this
    // entry huge right when the box-vs-real-geometry test is least
    // meaningful (the query would mostly measure the near/far faces of a
    // box wrapped around the camera itself).

    GPU& gpu = GPU::getSingleton();

    // Pass 1: read every entry's previous verdict and decide which ones are
    // worth a new query this frame. No GPU state touched yet - this is the
    // part that has to run for every hit regardless, so it stays separate
    // from the part that only runs for the ones that qualify.
    std::vector<OcclusionCandidate>& candidates = mOcclusionCandidates;
    candidates.clear();
    candidates.reserve(mStaticHits.size());
    {
        RADION_PROFILE_SCOPE("Occlusion poll+filter");
        u32 pending = 0;
        for (const SceneBVH::Hit& hit : mStaticHits)
        {
            const QueryHandle query = mStaticIndex.queryHandle(hit.entryIndex);
            if (!query.valid())
                continue;

            // Still in flight: not relaunched, because that would reset the
            // clock on the result this entry is already waiting for. Nothing
            // is POLLED here any more - the result arrives on its own when
            // readOcclusionResults() walks the buffer the GPU finished
            // writing two frames ago.
            if (mStaticIndex.queryPending(hit.entryIndex))
            {
                ++pending;
                continue;
            }

            // Already computed once in SceneBVH::build() from this same
            // object's (static, so unchanging) transform - no reason to
            // redo that matrix multiply for every hit, every frame.
            const AABB& worldBounds = mStaticIndex.entryBounds(hit.entryIndex);

            // The camera literally inside this entry's own box is the one
            // case a query cannot answer - it would measure the near and far
            // faces of a box wrapped around the viewer. The reference makes
            // the same single exclusion and no other.
            //
            // There used to be an apparent-size threshold here too, skipping
            // anything small on screen because a query cost real driver time
            // whatever the box behind it. That cost was the result POLLING,
            // and it is gone: a query is now a uniform bind, 36 indices, and
            // a copy the GPU makes on its own. The threshold outlived its
            // reason and had turned into a bug - far enough away, radius over
            // distance fell under it, the entry was never queried again, and
            // it kept whatever verdict it had from when it was close. Which
            // was "visible", so distant geometry could never become occluded.
            if (worldBounds.contains(cameraPosition))
                continue;

            // Spread over N frames so the query pass costs the same every
            // frame instead of everything at once. It buys that by making a
            // verdict older: an entry measured every N frames, plus the two
            // frames the result takes to come back, can stay wrongly hidden
            // for N+2 frames after whatever covered it has moved.
            //
            // Ours, not the reference's - it queries everything every frame.
            // One is the reference's behaviour.
            if (mOcclusionStagger > 1 &&
                (hit.entryIndex + mFrameNumber) % mOcclusionStagger != 0u)
                continue;

            candidates.push_back({query, hit.entryIndex, worldBounds});
        }
        mLastOcclusionPendingCount = pending;
    }
    mLastOcclusionQueryCount = static_cast<u32>(candidates.size());
    if (candidates.empty())
        return;

    // Pass 2: one write covering every candidate, each at its own offset -
    // see mOcclusionBlockStride's own comment for why not one write per draw.
    if (!ensureOcclusionBlockCapacity(static_cast<u32>(candidates.size())) ||
        !ensureOcclusionResultCapacity(static_cast<u32>(candidates.size())))
        return;
    mOcclusionBlockScratch.resize(static_cast<usize>(mOcclusionBlockStride) * candidates.size());
    for (usize i = 0; i < candidates.size(); ++i)
    {
        // Grown a hair before it is drawn. The pipeline's Compare::LessEqual
        // already lets a box face sitting at exactly the object's own depth
        // pass, but "exactly" is the problem: the box corner comes from
        // centre +/- extents and the mesh vertex from its own transform, so
        // the two never land on the same float. A face that ends up a fraction
        // BEHIND the surface fails the test and the object reports itself
        // occluded - which reads as a solid thing blinking out of existence
        // at certain angles. The reference extrudes by a flat 0.001; relative
        // to the box instead, because an absolute number means different
        // things at different scene scales, with a floor for a box that is
        // nearly flat on one axis.
        constexpr f32 kOcclusionBoxPadding = 0.002f;
        const glm::vec3 extents = candidates[i].worldBounds.extents();
        const glm::vec3 padded =
            extents + glm::max(extents * kOcclusionBoxPadding, glm::vec3(0.0005f));

        OcclusionBlock block;
        block.viewProjection = viewProjection;
        block.model = glm::translate(glm::mat4(1.0f), candidates[i].worldBounds.center()) *
                      glm::scale(glm::mat4(1.0f), padded);
        std::memcpy(mOcclusionBlockScratch.data() + i * mOcclusionBlockStride, &block,
                    sizeof(block));
    }
    gpu.updateBuffer(mOcclusionBlock, 0, mOcclusionBlockScratch.size(),
                     mOcclusionBlockScratch.data());

    // bits == 0: no clear. The depth prepass already wrote this frame's real
    // values here - the whole point is to test against them, not erase them.
    gpu.setTarget(depthTarget, {});
    gpu.setPipeline(mOcclusionPipeline);

    DrawDesc draw;
    draw.vertexBuffers[0] = mOcclusionCubeVertices;
    draw.vertexBufferCount = 1;
    draw.indexBuffer = mOcclusionCubeIndices;
    draw.indexType = IndexType::U16;
    draw.first = 0;
    draw.count = 36;

    std::vector<u32>& owners = mOcclusionResultOwners[mOcclusionResultIndex];
    owners.clear();
    const BufferHandle results = mOcclusionResults[mOcclusionResultIndex];

    {
        RADION_PROFILE_SCOPE("Occlusion draw");
        RADION_GPU_PROFILE_SCOPE("Occlusion draw");
        for (usize i = 0; i < candidates.size(); ++i)
        {
            gpu.bindUniform(0, mOcclusionBlock, i * mOcclusionBlockStride, sizeof(OcclusionBlock));
            gpu.beginOcclusionQuery(candidates[i].query);
            gpu.draw(draw);
            gpu.endOcclusionQuery();
            mStaticIndex.markQueryLaunched(candidates[i].entryIndex, mFrameNumber);
        }
    }

    if (results.valid())
    {
        RADION_PROFILE_SCOPE("Occlusion resolve");
        RADION_GPU_PROFILE_SCOPE("Occlusion resolve");
        const usize limit = glm::min(candidates.size(), usize(mOcclusionResultCapacity));
        for (usize i = 0; i < limit; ++i)
        {
            gpu.resolveQuery(candidates[i].query, results,
                             static_cast<u64>(i) * sizeof(u32));
            owners.push_back(candidates[i].entryIndex);
        }
    }

    // The buffer written kOcclusionResultBuffers frames ago, which the GPU
    // has long finished with. Read through the mapping - no driver call, no
    // flush, nothing to wait for.
    mOcclusionResultIndex = (mOcclusionResultIndex + 1) % kOcclusionResultBuffers;
    readOcclusionResults(mOcclusionResultIndex);
}

void Scene::readOcclusionResults(u32 bufferIndex)
{
    mLastOcclusionResultCount = 0;
    const std::vector<u32>& owners = mOcclusionResultOwners[bufferIndex];
    if (owners.empty())
        return;
    GPU& gpu = GPU::getSingleton();
    const u32* values = static_cast<const u32*>(gpu.mappedData(mOcclusionResults[bufferIndex]));
    if (!values)
    {
        Log::error("Scene: occlusion results unreadable, %zu verdicts dropped", owners.size());
        for (usize i = 0; i < owners.size(); ++i)
            mStaticIndex.markQueryResolved(owners[i]);
        mOcclusionResultOwners[bufferIndex].clear();
        return;
    }
    mLastOcclusionResultCount = static_cast<u32>(owners.size());
    for (usize i = 0; i < owners.size(); ++i)
    {
        mStaticIndex.setLastVisible(owners[i], values[i] > 0, mFrameNumber);
        mStaticIndex.markQueryResolved(owners[i]);
    }
    mOcclusionResultOwners[bufferIndex].clear();
}

void Scene::debugDrawOcclusion() const
{
    for (const SceneBVH::Hit& hit : mStaticHits)
    {
        const bool visible = mStaticIndex.lastVisible(hit.entryIndex);
        AABB bounds = mStaticIndex.entryBounds(hit.entryIndex);
        const glm::vec3 margin = (bounds.max - bounds.min) * 0.01f + glm::vec3(0.01f);
        bounds.min -= margin;
        bounds.max += margin;
        DebugDraw().box(bounds, visible ? Color::Green : Color::Red);
    }
}

Scene::DynamicIndexStats Scene::dynamicIndexStats() const
{
    DynamicIndexStats stats;
    stats.entryCount = static_cast<u32>(mDynamicRenderers.size());
    stats.nodeCount = mDynamicTree.nodeCount();
    stats.depth = mDynamicTree.depth();
    stats.nodesVisited = mDynamicTree.lastQueryStats().nodesVisited;
    stats.entriesAccepted = static_cast<u32>(mDynamicHits.size());
    stats.quality = mDynamicTree.quality();
    stats.rebuildPending = mDynamicBuildPending;
    return stats;
}

void Scene::debugDrawDynamicIndex(bool leavesOnly, bool drawEntries) const
{
    for (u32 i = 0; i < mDynamicTree.nodeCount(); ++i)
    {
        const BoundsTree::Node& node = mDynamicTree.node(i);
        if (leavesOnly && !node.isLeaf())
            continue;
        DebugDraw().box(node.bounds, node.isLeaf() ? Color::Green : Color::Cyan);
    }
    if (!drawEntries)
        return;
    for (const AABB& bounds : mDynamicBounds)
        DebugDraw().box(bounds, Color::Yellow);
}

bool Scene::buildRenderList(RenderList& list, u32 filter)
{
    if (!mActiveCamera || !mActiveCamera->owner())
    {
        Log::warning("Scene: cannot render without an active camera");
        return false;
    }
    return buildRenderList(list, mActiveCamera->viewProjectionMatrix(),
                           mActiveCamera->owner()->globalPosition(), filter);
}

bool Scene::buildRenderList(RenderList& list, const glm::mat4& viewProjection,
                            const glm::vec3& cameraPosition, u32 filter, bool occlusionView,
                            bool previewOcclusion)
{
    flushChanges();
    if (mStaticIndexDirty)
        rebuildStaticIndex();
    list.clear();
    list.setFilter(filter);
    list.setCamera(viewProjection, cameraPosition);

    DirectionalLight* elected = electedSunLight();
    for (Light* light : mLights)
    {
        GameObject* object = light->owner();
        if (!light->active() || !object->isActiveAndVisibleInHierarchy())
            continue;
        RenderLight output;
        output.position = object->globalPosition();
        output.direction = object->forward();
        output.color = light->color() * light->intensity();
        output.type = static_cast<RenderLightType>(light->lightType());
        output.flags = (light->castsShadows() ? static_cast<u32>(RenderLightCastShadow) : 0u) |
                       (light->volumetric() ? static_cast<u32>(RenderLightVolumetric) : 0u);
        switch (light->lightType())
        {
        case LightType::Point:
            output.range = static_cast<PointLight*>(light)->range();
            break;
        case LightType::Spot:
        {
            const SpotLight* spot = static_cast<SpotLight*>(light);
            output.range = spot->range();
            const f32 innerCos = glm::cos(glm::radians(spot->innerAngle()));
            const f32 outerCos = glm::cos(glm::radians(spot->outerAngle()));
            output.coneAngleCos = outerCos;
            output.coneAngleScale = 1.0f / glm::max(innerCos - outerCos, 0.0001f);
            break;
        }
        case LightType::Rectangle:
        {
            const RectangleLight* rectangle = static_cast<RectangleLight*>(light);
            output.range = rectangle->range();
            output.rectangleRight = object->right();
            output.rectangleUp = object->up();
            output.rectangleWidth = rectangle->width();
            output.rectangleHeight = rectangle->height();
            break;
        }
        case LightType::Directional:
            break;
        }
        if (list.addLight(output) && light == elected)
            list.setSunIndex(static_cast<s32>(list.lights().size()) - 1);
    }

    AssetManager& assets = Assets();
    MaterialManager& materials = MaterialManager::getSingleton();

    // Static geometry: SceneBVH already narrowed "everything" down to the
    // submeshes the frustum could not reject, so only their own material
    // gets resolved/synced - not the whole mesh's, which for one big merged
    // static mesh (an exported level) can be hundreds of materials nothing
    // this frame draws.
    if (occlusionView)
        ++mFrameNumber;
    if (mStaticCullingEnabled && mStaticIndex.entryCount() > 0)
    {
        std::vector<SceneBVH::Hit>& staticHits = occlusionView ? mStaticHits : mBystanderHits;
        staticHits.clear();
        mStaticIndex.query(list.frustum(), staticHits);
        list.addCulled(mStaticIndex.stats().entryCount - mStaticIndex.stats().entriesAccepted);
        // The BVH answers one hit per SUBMESH, so a merged level mesh brings
        // the same GameObject back hundreds of times in a row, and the probe
        // search is a linear scan over every probe in the scene. Nothing
        // moves inside this loop, so the same object gives the same answer:
        // remembering the last one turns that scan from per-submesh back into
        // per-object. The hits arrive grouped by entry, so a single slot is
        // all it takes.
        const GameObject* probeOwner = nullptr;
        RenderProbe probeCache;
        for (const SceneBVH::Hit& hit : staticHits)
        {
            MeshRenderer* renderer = hit.renderer;
            GameObject* object = renderer->owner();
            if (!renderer->active() || !object->isActiveAndVisibleInHierarchy())
                continue;
            // Last frame's occlusion verdict - see updateOcclusionQueries().
            // The entry is still measured every frame regardless (the query
            // pass runs off this same mStaticHits list further down the
            // frame), so a wrong skip here corrects itself next frame
            // instead of getting stuck invisible. justEnteredView() covers
            // the other direction: an entry that was off-screen (not just
            // occluded) for a while is carrying a verdict from whenever it
            // was last actually tested, which says nothing about what is in
            // front of it now that it is back in frustum - trusting a stale
            // "occluded" here is exactly the 1-frame pop-in a fast turn or a
            // newly opened doorway would otherwise show.
            if (occlusionView)
            {
            const bool justEntered = mStaticIndex.justEnteredView(hit.entryIndex, mFrameNumber);
            mStaticIndex.markSeenThisFrame(hit.entryIndex, mFrameNumber);
            // A verdict is only worth acting on while it still describes what
            // is in front of this entry. Nothing is re-measured every frame -
            // a query in flight is left alone until it answers - so an entry
            // that stayed in view the whole time can still be carrying an
            // "occluded" from several frames back, which is what a moving
            // camera turns into geometry blinking in and out. Past this many
            // frames the entry is simply drawn until a fresh measurement says
            // otherwise: drawing something that turns out to be hidden costs
            // one draw call, skipping something that turns out to be visible
            // costs a hole in the image.
            constexpr u32 kVerdictLifetimeFrames = 1;
            // A verdict also stays valid while this entry's next query is in
            // flight: the round-trip is two frames against a one-frame
            // lifetime, so without this every entry alternated between
            // stale-drawn and fresh-culled at frame rate.
            const bool verdictFresh =
                mStaticIndex.verdictAge(hit.entryIndex, mFrameNumber) <= kVerdictLifetimeFrames ||
                mStaticIndex.queryPending(hit.entryIndex);
            if (mOcclusionQueryEnabled && !justEntered && verdictFresh &&
                !mStaticIndex.lastVisible(hit.entryIndex))
                continue;
            }
            else if (previewOcclusion && mOcclusionQueryEnabled &&
                     !mStaticIndex.lastVisible(hit.entryIndex))
                continue;
            Mesh* mesh = assets.getMesh(renderer->mesh());
            if (!mesh || hit.submeshIndex >= mesh->submeshes.size())
                continue;

            Material* overrides = const_cast<Material*>(renderer->materialOverrides());
            const u32 overrideCount = renderer->materialOverrideCount();
            const u32 slot = mesh->submeshes[hit.submeshIndex].materialSlot;
            Material* material = (overrides && slot < overrideCount) ? &overrides[slot]
                                 : slot < mesh->materials.size()     ? &mesh->materials[slot]
                                                                     : nullptr;
            if (material)
            {
                materials.resolvePipeline(*material, mesh->colorLayout);
                materials.sync(*material);
            }

            Animator* animator = object->getComponent<Animator>();
            const std::vector<glm::mat4>* palette =
                animator && animator->active() ? &animator->palette() : nullptr;
            const std::vector<glm::mat4>* prevPalette =
                animator && animator->active() ? &animator->prevPalette() : nullptr;
            if (object != probeOwner)
            {
                probeCache = resolveNearestProbe(mReflectionProbes, object->globalPosition());
                probeOwner = object;
            }
            list.submitSubmesh(renderer->mesh(), *mesh, hit.submeshIndex, object->globalTransform(),
                               overrides, overrideCount, palette, &probeCache,
                               &object->previousGlobalTransform(), prevPalette);
        }
    }

    // Dynamic geometry. When the octree is on, query it instead of the scan;
    // when off, the exact per-renderer loop below runs as it always has.
    if (mDynamicCullingEnabled && mDynamicIndexDirty)
        rebuildDynamicIndex();

    if (mDynamicCullingEnabled && !mDynamicRenderers.empty())
    {
        {
            // Same name as the query below, so the two halves of the tree's
            // work add up into one row instead of spending two slots.
            RADION_PROFILE_SCOPE("Dynamic tree");
            refreshDynamicBounds();
            updateDynamicTree();
        }

        std::vector<MeshRenderer*>& dynamicHits =
            occlusionView ? mDynamicHits : mBystanderDynamicHits;
        dynamicHits.clear();
        {
            RADION_PROFILE_SCOPE("Dynamic tree");
            const Frustum& frustum = list.frustum();
            mDynamicTree.queryCandidates(frustum, mDynamicCandidates);
            // Candidates, not the answer: the tree returns everything sharing
            // a leaf with something visible, and the exact test is this.
            for (u32 item : mDynamicCandidates)
                if (frustum.intersects(mDynamicBounds[item]))
                    dynamicHits.push_back(mDynamicRenderers[item]);
        }
        list.addCulled(static_cast<u32>(mDynamicRenderers.size() - dynamicHits.size()));
        for (MeshRenderer* renderer : dynamicHits)
            submitDynamicRenderer(renderer, list, assets, materials, mReflectionProbes);

        // Renderers the tree cannot index (skinned - the box follows the
        // pose, not the transform) are cached when the index is rebuilt.
        for (MeshRenderer* renderer : mDynamicLinearFallback)
            submitDynamicRenderer(renderer, list, assets, materials, mReflectionProbes);

        // With static culling disabled there is intentionally no spatial
        // index for statics, so their linear submission is unavoidable.
        if (!mStaticCullingEnabled || mStaticIndex.entryCount() == 0)
        {
            for (MeshRenderer* renderer : mRenderers)
                if (GameObject* object = renderer ? renderer->owner() : nullptr;
                    object && object->isStatic())
                    submitDynamicRenderer(renderer, list, assets, materials, mReflectionProbes);
        }
    }
    else
    {
        for (MeshRenderer* renderer : mRenderers)
        {
            GameObject* object = renderer->owner();
            if (object && object->isStatic() && mStaticCullingEnabled &&
                mStaticIndex.entryCount() > 0)
                continue; // already handled by the BVH above
            submitDynamicRenderer(renderer, list, assets, materials, mReflectionProbes);
        }
    }

    for (Terrain* terrain : mTerrains)
    {
        GameObject* object = terrain->owner();
        if (!terrain->active() || !object->isActiveAndVisibleInHierarchy() || !terrain->valid())
            continue;
        terrain->prepare(list.frustum(), cameraPosition);
        terrain->submitCamera(list, object->globalTransform());
    }

    for (Landscape* landscape : mLandscapes)
    {
        GameObject* object = landscape->owner();
        if (!landscape->active() || !object->isActiveAndVisibleInHierarchy())
            continue;
        landscape->update(cameraPosition);
        landscape->cull(list.frustum());
        landscape->submitCamera(list, object->globalTransform(), cameraPosition);
    }

    for (Road* road : mRoads)
    {
        GameObject* object = road->owner();
        if (!road->active() || !object->isActiveAndVisibleInHierarchy() ||
            !road->valid())
            continue;
        Mesh* mesh = assets.getMesh(road->mesh());
        if (!mesh)
            continue;
        Material& material = road->material();
        materials.resolvePipeline(material, mesh->colorLayout);
        materials.sync(material);
        list.submit(road->mesh(), *mesh, object->globalTransform(), &material, 1);
    }

    for (Forest* forest : mForests)
    {
        GameObject* object = forest->owner();
        if (!forest->active() || !object->isActiveAndVisibleInHierarchy())
            continue;
        forest->submit(list, object->globalTransform(), cameraPosition);
    }

    for (Grass* grass : mGrass)
    {
        GameObject* object = grass->owner();
        if (!grass->active() || !object->isActiveAndVisibleInHierarchy())
            continue;
        grass->submit(object->globalTransform(), mDeltaTime);
    }

    for (Hair* hair : mHair)
    {
        GameObject* object = hair->owner();
        if (!hair->active() || !object->isActiveAndVisibleInHierarchy())
            continue;
        hair->submit(mDeltaTime);
    }

    for (Ocean* ocean : mOceans)
    {
        GameObject* object = ocean->owner();
        if (!ocean->active() || !object->isActiveAndVisibleInHierarchy())
            continue;
        ocean->submit(object->globalTransform());
    }

    DebugDraw3D& debug = DebugDraw();
    for (GameObject* object : mDebugObjects)
    {
        if (!object->isActiveAndVisibleInHierarchy())
            continue;
        if (object->hasDebugFlag(DebugObjectAxis))
            debug.axis(object->globalTransform());
        MeshRenderer* renderer = object->getComponent<MeshRenderer>();
        if (!renderer)
            continue;
        const Mesh* mesh = assets.getMesh(renderer->mesh());
        if (!mesh)
            continue;
        if (object->hasDebugFlag(DebugMeshBounds))
            debug.box(transformAABB(mesh->bounds, object->globalTransform()), Color::Green);
        if (object->hasDebugFlag(DebugSubMeshBounds))
            for (const SubMesh& submesh : mesh->submeshes)
                debug.box(transformAABB(submesh.bounds, object->globalTransform()), Color::Yellow);
        u8 vectorFlags = 0;
        if (object->hasDebugFlag(DebugVertexNormals))
            vectorFlags |= DebugVectorsNormal;
        if (object->hasDebugFlag(DebugVertexTangents))
            vectorFlags |= DebugVectorsTangent;
        if (vectorFlags)
            debug.meshVectors(renderer->mesh(), object->globalTransform(), 0.15f, vectorFlags);
        Animator* animator = object->getComponent<Animator>();
        if (object->hasDebugFlag(DebugSkeleton) && animator)
        {
            const Skeleton* skeleton = animator->skeleton();
            const std::vector<glm::mat4>& pose = animator->globalPose();
            if (skeleton && pose.size() == skeleton->boneCount())
                for (u32 bone = 0; bone < skeleton->boneCount(); ++bone)
                {
                    const s32 parent = skeleton->bone(bone).parent;
                    if (parent < 0)
                        continue;
                    const glm::vec3 joint =
                        glm::vec3(object->globalTransform() * pose[bone] * glm::vec4(0, 0, 0, 1));
                    const glm::vec3 parentJoint =
                        glm::vec3(object->globalTransform() * pose[parent] * glm::vec4(0, 0, 0, 1));
                    debug.line(parentJoint, joint, Color::Cyan, false);
                }
        }
    }
    list.sort();
    return true;
}

bool Scene::buildShadowList(RenderList& list, const glm::mat4& viewProjection, u32 filter,
                            const Sphere* cullSphere, MeshHandle exclude, u64 excludeObjectId,
                            bool reflectionCapture, const std::vector<Plane>* casterPlanes,
                            f32 minCasterExtent)
{
    // Shadow lists can also be built without a preceding camera list. Flush
    // and refresh here as well so a queued deletion can never leave the BVH
    // handing out a freed MeshRenderer pointer to the shadow pass.
    flushChanges();
    if (mStaticIndexDirty)
        rebuildStaticIndex();
    list.clear();
    list.setFilter(filter);
    list.setCamera(viewProjection, glm::vec3(0.0f));
    if (cullSphere)
        list.setCullSphere(*cullSphere);

    // Static geometry: same BVH buildRenderList() uses for the camera, now
    // queried against the cascade's own frustum instead - a caster the
    // camera cannot see may still throw a shadow the camera can, and the
    // camera's own frustum-narrowed BVH query never had it to begin with.
    // Pipelines for the *static* set are resolved once, up front, in
    // rebuildStaticIndex() - not here, and not by buildRenderList() either,
    // because with the camera's own BVH query narrowing what it resolves to
    // only what it can currently see, a wall behind the camera casting a
    // shadow onto ground the camera can see would otherwise have an
    // unresolved pipeline the first time anything ever asks for its shadow,
    // and emitSubmesh() drops anything whose pipeline is not valid.
    AssetManager& assets = Assets();
    if (mStaticIndex.entryCount() > 0)
    {
        mShadowStaticHits.clear();
        // cullSphere here prunes whole subtrees during the descent - a
        // point/spot shadow face's frustum is often wide, but nothing
        // outside the light's own range can ever be lit regardless.
        mStaticIndex.query(list.frustum(), mShadowStaticHits, cullSphere, casterPlanes);
        list.addCulled(mStaticIndex.stats().entryCount - mStaticIndex.stats().entriesAccepted);
        for (const SceneBVH::Hit& hit : mShadowStaticHits)
        {
            MeshRenderer* renderer = hit.renderer;
            GameObject* object = renderer->owner();
            if (!renderer->active() || !object->isActiveAndVisibleInHierarchy())
                continue;
            if (exclude.valid() && renderer->mesh() == exclude)
                continue;
            if (excludeObjectId && object->id() == excludeObjectId)
                continue;
            if (reflectionCapture && !renderer->visibleInReflections())
                continue;
            if (minCasterExtent > 0.0f)
            {
                const glm::vec3 size = mStaticIndex.entryBounds(hit.entryIndex).extents() * 2.0f;
                if (glm::max(size.x, glm::max(size.y, size.z)) < minCasterExtent)
                    continue;
            }
            Mesh* mesh = assets.getMesh(renderer->mesh());
            if (!mesh || hit.submeshIndex >= mesh->submeshes.size())
                continue;
            const Material* overrides = renderer->materialOverrides();
            const u32 overrideCount = renderer->materialOverrideCount();
            list.submitSubmesh(renderer->mesh(), *mesh, hit.submeshIndex, object->globalTransform(),
                               overrides, overrideCount, nullptr, nullptr,
                               &object->previousGlobalTransform());
        }
    }

    // Dynamic objects only - counted in dozens/hundreds, not worth a spatial
    // index (see docs/TAREFA_SCENE_CULLING.md). isVisibleInHierarchy() is a
    // manual show/hide flag, not frustum visibility, so every caster
    // buildRenderList touched is prepared here too, whether or not the
    // camera itself could see it - their own pipelines are still resolved by
    // buildRenderList()'s dynamic loop every frame, unconditionally.
    for (MeshRenderer* renderer : mRenderers)
    {
        GameObject* object = renderer->owner();
        if (object && object->isStatic() && mStaticIndex.entryCount() > 0)
            continue; // already handled by the BVH above
        if (!renderer->active() || !object->isActiveAndVisibleInHierarchy() ||
            !renderer->mesh().valid())
            continue;
        if (exclude.valid() && renderer->mesh() == exclude)
            continue;
        if (excludeObjectId && object->id() == excludeObjectId)
            continue;
        if (reflectionCapture && !renderer->visibleInReflections())
            continue;
        Mesh* mesh = assets.getMesh(renderer->mesh());
        if (!mesh)
            continue;
        const AABB worldBounds = transformAABB(mesh->bounds, object->globalTransform());
        if (outsideCasterVolume(casterPlanes, worldBounds))
            continue;
        if (minCasterExtent > 0.0f)
        {
            const glm::vec3 size = worldBounds.extents() * 2.0f;
            if (glm::max(size.x, glm::max(size.y, size.z)) < minCasterExtent)
                continue;
        }
        const Material* overrides = renderer->materialOverrides();
        const u32 overrideCount = renderer->materialOverrideCount();
        Animator* animator = object->getComponent<Animator>();
        const std::vector<glm::mat4>* palette =
            animator && animator->active() ? &animator->palette() : nullptr;
        const std::vector<glm::mat4>* prevPalette =
            animator && animator->active() ? &animator->prevPalette() : nullptr;
        list.submit(renderer->mesh(), *mesh, object->globalTransform(), overrides, overrideCount,
                    palette, nullptr, &object->previousGlobalTransform(), prevPalette);
    }

    // Terrain never entered this list before chunking: buildShadowList() only
    // ever walked mRenderers, so it never cast its own shadow onto anything -
    // it could only ever receive one. submitShadow() already skips chunks
    // with no slope, so this stays cheap on the flat majority of a terrain.
    for (Terrain* terrain : mTerrains)
    {
        GameObject* object = terrain->owner();
        if (!terrain->active() || !object->isActiveAndVisibleInHierarchy() || !terrain->valid())
            continue;
        terrain->submitShadow(list, object->globalTransform());
    }

    // Forest::submit() already tests each species' materials against
    // list.filter() through list.submit() - MaterialCastShadow here, the same
    // way buildRenderList() lets it through unfiltered. Without this loop a
    // tree just never appeared in any shadow view: buildShadowList() only
    // ever walked mRenderers.
    const glm::vec3 cameraPosition =
        mActiveCamera ? mActiveCamera->owner()->globalPosition() : glm::vec3(0.0f);
    for (Forest* forest : mForests)
    {
        GameObject* object = forest->owner();
        if (!forest->active() || !object->isActiveAndVisibleInHierarchy())
            continue;
        forest->submit(list, object->globalTransform(), cameraPosition);
    }

    // Only chunks with slope enter here (Landscape::submitShadow) - flat
    // ground never shadows itself, and most of an open world is flat ground.
    for (Landscape* landscape : mLandscapes)
    {
        GameObject* object = landscape->owner();
        if (!landscape->active() || !object->isActiveAndVisibleInHierarchy())
            continue;
        landscape->submitShadow(list, object->globalTransform(), cameraPosition);
    }

    list.sort();
    return true;
}

void Scene::componentAdded(Component* component)
{
    if (!component)
        return;
    component->mSceneUpdateIndex = mUpdateComponents.size();
    mUpdateComponents.push_back(component);
    if ((component->mEvents & ComponentEventLateUpdate) != 0)
    {
        component->mSceneLateUpdateIndex = mLateUpdateComponents.size();
        mLateUpdateComponents.push_back(component);
    }

    switch (component->type())
    {
    case ComponentType::Camera:
        mCameras.push_back(static_cast<Camera*>(component));
        break;
    case ComponentType::MeshRenderer:
        mRenderers.push_back(static_cast<MeshRenderer*>(component));
        mStaticIndexDirty = true;
        mDynamicIndexDirty = true;
        break;
    case ComponentType::Animator:
        mAnimators.push_back(static_cast<Animator*>(component));
        break;
    case ComponentType::BoneAttachment:
        mBoneAttachments.push_back(static_cast<BoneAttachment*>(component));
        break;
    case ComponentType::Terrain:
        mTerrains.push_back(static_cast<Terrain*>(component));
        break;
    case ComponentType::Landscape:
        mLandscapes.push_back(static_cast<Landscape*>(component));
        break;
    case ComponentType::Road:
        mRoads.push_back(static_cast<Road*>(component));
        break;
    case ComponentType::Forest:
        mForests.push_back(static_cast<Forest*>(component));
        break;
    case ComponentType::Grass:
        mGrass.push_back(static_cast<Grass*>(component));
        break;
    case ComponentType::Hair:
        mHair.push_back(static_cast<Hair*>(component));
        break;
    case ComponentType::Ocean:
        mOceans.push_back(static_cast<Ocean*>(component));
        break;
    case ComponentType::ParticleEffect:
        mParticleEffects.push_back(static_cast<ParticleEffect*>(component));
        break;
    case ComponentType::Light:
        mLights.push_back(static_cast<Light*>(component));
        break;
    case ComponentType::ReflectionProbe:
        mReflectionProbes.push_back(static_cast<ReflectionProbe*>(component));
        break;
    case ComponentType::Collider:
        mColliders.push_back(static_cast<Collider*>(component));
        break;
    case ComponentType::RigidBody:
        addBody(*static_cast<Physics::RigidBody*>(component));
        break;
    case ComponentType::Joint:
        mJointComponents.push_back(static_cast<Physics::Joint*>(component));
        break;
    case ComponentType::Agent:
        addAgent(*static_cast<Agent*>(component));
        break;
    case ComponentType::Obstacle:
        addObstacle(*static_cast<Obstacle*>(component));
        break;
    default:
        break;
    }
}

void Scene::componentRemoved(Component* component)
{
    if (!component)
        return;
    const auto removeFromEventList = [component](std::vector<Component*>& components,
                                                 usize& index) {
        if (index < components.size() && components[index] == component)
        {
            components[index] = nullptr;
            return true;
        }
        index = Component::InvalidSceneListIndex;
        return false;
    };
    const bool removedUpdate =
        removeFromEventList(mUpdateComponents, component->mSceneUpdateIndex);
    const bool removedLate =
        removeFromEventList(mLateUpdateComponents, component->mSceneLateUpdateIndex);
    mComponentListsDirty = mComponentListsDirty || removedUpdate || removedLate;

    // A script that fetched this component holds a handle wrapping its
    // address, and those handles are cached by pointer and never collected -
    // the class is persistent. Left in the cache, the next component to be
    // allocated at the same address inherits the handle, and a script calling
    // through the old one reaches freed memory.
    if (ScriptCache::alive())
        ScriptCache::getSingleton().forgetInstance(component);

    switch (component->type())
    {
    case ComponentType::Camera:
        removePointer(mCameras, static_cast<Camera*>(component));
        if (mActiveCamera == component)
            mActiveCamera = nullptr;
        break;
    case ComponentType::MeshRenderer:
        removePointer(mRenderers, static_cast<MeshRenderer*>(component));
        mStaticIndexDirty = true;
        mDynamicIndexDirty = true;
        break;
    case ComponentType::Animator:
        removePointer(mAnimators, static_cast<Animator*>(component));
        break;
    case ComponentType::BoneAttachment:
        removePointer(mBoneAttachments, static_cast<BoneAttachment*>(component));
        break;
    case ComponentType::Terrain:
        removePointer(mTerrains, static_cast<Terrain*>(component));
        break;
    case ComponentType::Landscape:
        removePointer(mLandscapes, static_cast<Landscape*>(component));
        break;
    case ComponentType::Road:
        removePointer(mRoads, static_cast<Road*>(component));
        break;
    case ComponentType::Forest:
        removePointer(mForests, static_cast<Forest*>(component));
        break;
    case ComponentType::Grass:
        removePointer(mGrass, static_cast<Grass*>(component));
        break;
    case ComponentType::Hair:
        removePointer(mHair, static_cast<Hair*>(component));
        break;
    case ComponentType::Ocean:
        removePointer(mOceans, static_cast<Ocean*>(component));
        break;
    case ComponentType::ParticleEffect:
        removePointer(mParticleEffects, static_cast<ParticleEffect*>(component));
        break;
    case ComponentType::Light:
        removePointer(mLights, static_cast<Light*>(component));
        if (mSunLight == component)
            mSunLight = nullptr;
        break;
    case ComponentType::ReflectionProbe:
        removePointer(mReflectionProbes, static_cast<ReflectionProbe*>(component));
        break;
    case ComponentType::Collider:
        removePointer(mColliders, static_cast<Collider*>(component));
        break;
    case ComponentType::RigidBody:
        removeBody(*static_cast<Physics::RigidBody*>(component));
        break;
    case ComponentType::Joint:
        removePointer(mJointComponents, static_cast<Physics::Joint*>(component));
        break;
    case ComponentType::Agent:
        removeAgent(*static_cast<Agent*>(component));
        break;
    case ComponentType::Obstacle:
        removeObstacle(*static_cast<Obstacle*>(component));
        break;
    default:
        break;
    }
}

void Scene::compactComponentLists()
{
    if (!mComponentListsDirty)
        return;

    const auto compact = [](std::vector<Component*>& components, bool lateUpdate) {
        usize write = 0;
        for (usize read = 0; read < components.size(); ++read)
        {
            Component* component = components[read];
            if (!component)
                continue;
            components[write] = component;
            if (lateUpdate)
                component->mSceneLateUpdateIndex = write;
            else
                component->mSceneUpdateIndex = write;
            ++write;
        }
        components.resize(write);
    };
    compact(mUpdateComponents, false);
    compact(mLateUpdateComponents, true);
    mComponentListsDirty = false;
}

void Scene::registerBranch(GameObject* object)
{
    object->mScene = this;
    stampId(object);
    mObjects.push_back(object);
    for (Component* component : object->mComponents)
        if (component)
            componentAdded(component);
    if (object->debugFlags() != DebugNone)
        mDebugObjects.push_back(object);
    for (usize i = 0; i < object->childCount(); ++i)
        registerBranch(object->child(i));
}

void Scene::unregisterBranch(GameObject* object)
{
    for (usize i = 0; i < object->childCount(); ++i)
        unregisterBranch(object->child(i));
    for (Component* component : object->mComponents)
        if (component)
            componentRemoved(component);
    removePointer(mDebugObjects, object);
    removePointer(mObjects, object);
    object->mScene = nullptr;
}

void Scene::debugFlagsChanged(GameObject* object, u32 previous)
{
    if (previous == DebugNone && object->debugFlags() != DebugNone)
        mDebugObjects.push_back(object);
    else if (previous != DebugNone && object->debugFlags() == DebugNone)
        removePointer(mDebugObjects, object);
}

void Scene::flushChanges()
{
    for (GameObject* object : mPendingRemove)
    {
        unregisterBranch(object);
        if (object->parent())
            object->parent()->removeChildRaw(object);
        mDetached.push_back(object);
    }
    mPendingRemove.clear();
    for (GameObject* object : mPendingDestroy)
    {
        if (object->mScene)
            unregisterBranch(object);
        if (object->parent())
            object->parent()->removeChildRaw(object);
        removePointer(mDetached, object);
        forgetIdBranch(object);
        delete object;
    }
    mPendingDestroy.clear();
    for (const PendingAdd& pending : mPendingAdd)
    {
        if (pending.parent->addChildRaw(pending.object))
            registerBranch(pending.object);
        else
        {
            forgetIdBranch(pending.object);
            delete pending.object;
        }
    }
    mPendingAdd.clear();
}

bool Scene::pickSurface(TextureHandle depth, u32 depthWidth, u32 depthHeight, f32 mouseX,
                        f32 mouseY, u32 windowWidth, u32 windowHeight,
                        const glm::mat4& inverseProjection, const glm::mat4& inverseView,
                        glm::vec3& outPosition, glm::vec3& outNormal)
{
    if (!depth.valid() || depthWidth <= 2 || depthHeight <= 2 || windowWidth == 0 ||
        windowHeight == 0)
        return false;

    // Window -> render target. They are different resolutions: the scene draws
    // at a fixed internal size independent of the window. And Y flips, because
    // the window counts from the top and GL counts from the bottom.
    const s32 px =
        static_cast<s32>(mouseX / static_cast<f32>(windowWidth) * static_cast<f32>(depthWidth));
    const s32 py = static_cast<s32>((1.0f - mouseY / static_cast<f32>(windowHeight)) *
                                    static_cast<f32>(depthHeight));
    if (px < 1 || py < 1 || px >= static_cast<s32>(depthWidth) - 1 ||
        py >= static_cast<s32>(depthHeight) - 1)
        return false;

    // 3x3 around the pixel: the neighbours are what the normal comes from.
    f32 d[9] = {};
    if (!GPU::getSingleton().readDepthPixels(depth, static_cast<u32>(px - 1),
                                             static_cast<u32>(py - 1), 3, 3, d, 9))
        return false;

    const f32 centre = d[4];
    if (centre >= 1.0f || centre <= 0.0f)
        return false; // sky or background: nothing to stick to

    const glm::vec3 position =
        viewPositionFromDepth(px, py, centre, depthWidth, depthHeight, inverseProjection);

    // The normal comes from differences, but taking the NEAREST neighbour on
    // each axis instead of always the right and the top one.
    //
    // Why: on a silhouette, one of the neighbours lands on the object behind.
    // The difference then crosses the gap between the two and the normal comes
    // out nearly perpendicular to the view - garbage. The decal was laid flat,
    // the slope fade killed it, and the click looked like it did nothing.
    // Comparing |dz| and keeping the side that does not jump fixes the common
    // case. Indices of the 3x3 block:  0 1 2   (y-1)
    //                                  3 4 5   (y)
    //                                  6 7 8   (y+1)
    const bool useRight = std::fabs(d[5] - centre) <= std::fabs(d[3] - centre);
    const bool useUp = std::fabs(d[7] - centre) <= std::fabs(d[1] - centre);

    const glm::vec3 alongX =
        useRight
            ? viewPositionFromDepth(px + 1, py, d[5], depthWidth, depthHeight, inverseProjection)
            : viewPositionFromDepth(px - 1, py, d[3], depthWidth, depthHeight, inverseProjection);
    const glm::vec3 alongY =
        useUp ? viewPositionFromDepth(px, py + 1, d[7], depthWidth, depthHeight, inverseProjection)
              : viewPositionFromDepth(px, py - 1, d[1], depthWidth, depthHeight, inverseProjection);

    // Keeps the cross product's sense when the far-side neighbour is used.
    const glm::vec3 dX = useRight ? (alongX - position) : (position - alongX);
    const glm::vec3 dY = useUp ? (alongY - position) : (position - alongY);

    glm::vec3 normal = glm::cross(dX, dY);
    const f32 length = glm::length(normal);
    if (length < 1e-8f)
        return false; // degenerate neighbours
    normal /= length;

    // In view space the camera looks down -Z, so a visible surface must have a
    // positive Z normal. Flip it if it came out the other way.
    if (normal.z < 0.0f)
        normal = -normal;

    outPosition = glm::vec3(inverseView * glm::vec4(position, 1.0f));
    outNormal = glm::normalize(glm::mat3(inverseView) * normal);
    return true;
}

namespace
{
void pickObjectRecursive(const GameObject& object, const Ray& ray, GameObject*& best, f32& bestT)
{
    if (object.active() && object.isVisibleInHierarchy())
    {
        if (MeshRenderer* renderer = object.getComponent<MeshRenderer>())
        {
            if (const Mesh* mesh = Assets().getMesh(renderer->mesh()))
            {
                const AABB worldBounds = transformAABB(mesh->bounds, object.globalTransform());
                f32 t;
                if (ray.intersects(worldBounds, t) && t < bestT)
                {
                    bestT = t;
                    best = const_cast<GameObject*>(&object);
                }
            }
        }
    }
    for (usize i = 0; i < object.childCount(); ++i)
        pickObjectRecursive(*object.child(i), ray, best, bestT);
}
void pickObjectAtPointRecursive(const GameObject& object, const glm::vec3& point, GameObject*& best,
                                f32& bestVolume)
{
    if (object.active() && object.isVisibleInHierarchy())
    {
        if (MeshRenderer* renderer = object.getComponent<MeshRenderer>())
        {
            if (const Mesh* mesh = Assets().getMesh(renderer->mesh()))
            {
                const AABB worldBounds = transformAABB(mesh->bounds, object.globalTransform());
                if (worldBounds.contains(point))
                {
                    const glm::vec3 extents = worldBounds.extents();
                    const f32 volume = extents.x * extents.y * extents.z;
                    if (volume < bestVolume)
                    {
                        bestVolume = volume;
                        best = const_cast<GameObject*>(&object);
                    }
                }
            }
        }
    }
    for (usize i = 0; i < object.childCount(); ++i)
        pickObjectAtPointRecursive(*object.child(i), point, best, bestVolume);
}
} // namespace

GameObject* Scene::pickObject(const Ray& ray, f32* outDistance) const
{
    GameObject* best = nullptr;
    f32 bestT = std::numeric_limits<f32>::max();
    pickObjectRecursive(root(), ray, best, bestT);
    if (best && outDistance)
        *outDistance = bestT;
    return best;
}

GameObject* Scene::pickDynamicObject(const Ray& ray, f32* outDistance)
{
    if (mDynamicIndexDirty)
        rebuildDynamicIndex();

    // Candidates, then the nearest by exact box distance. The old octree
    // returned the nearest NODE's entry, which is not the same thing: a big
    // box far away can start closer than a small one in front of it.
    mDynamicTree.queryCandidates(ray, std::numeric_limits<f32>::max(), mDynamicCandidates);
    GameObject* best = nullptr;
    f32 bestT = std::numeric_limits<f32>::max();
    for (u32 item : mDynamicCandidates)
    {
        f32 t = 0.0f;
        if (!ray.intersects(mDynamicBounds[item], t) || t >= bestT)
            continue;
        GameObject* object = mDynamicRenderers[item] ? mDynamicRenderers[item]->owner() : nullptr;
        if (!object || !object->isActiveAndVisibleInHierarchy())
            continue;
        bestT = t;
        best = object;
    }
    if (best && outDistance)
        *outDistance = bestT;
    return best;
}

GameObject* Scene::pickObjectAtPoint(const glm::vec3& point) const
{
    GameObject* best = nullptr;
    f32 bestVolume = std::numeric_limits<f32>::max();
    pickObjectAtPointRecursive(root(), point, best, bestVolume);
    return best;
}

s32 Scene::pickSubmeshAtPoint(const GameObject& object, const glm::vec3& point, s32* outSubmesh)
{
    if (outSubmesh)
        *outSubmesh = -1;

    MeshRenderer* renderer = object.getComponent<MeshRenderer>();
    const Mesh* mesh = renderer ? Assets().getMesh(renderer->mesh()) : nullptr;
    if (!mesh)
        return -1;

    const glm::mat4& transform = object.globalTransform();
    s32 best = -1;
    f32 bestVolume = std::numeric_limits<f32>::max();
    for (usize i = 0; i < mesh->submeshes.size(); ++i)
    {
        const SubMesh& submesh = mesh->submeshes[i];
        if (!submesh.visible)
            continue;
        const AABB worldBounds = transformAABB(submesh.bounds, transform);
        if (!worldBounds.contains(point))
            continue;
        const glm::vec3 extents = worldBounds.extents();
        const f32 volume = extents.x * extents.y * extents.z;
        if (volume < bestVolume)
        {
            bestVolume = volume;
            best = static_cast<s32>(submesh.materialSlot);
            if (outSubmesh)
                *outSubmesh = static_cast<s32>(i);
        }
    }
    return best;
}

// ----------------------------------------------------------------- physics

namespace
{
// How close a new contact point has to be to a cached one to count as the
// same point. Too tight and the impulse is thrown away every step, which is
// warm starting not happening at all; too loose and a point that slid across
// a face inherits an impulse meant for somewhere else.
constexpr f32 kMatchDistance = 0.02f;

bool finiteVector(const glm::vec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

f32 radialWeight(const glm::vec3& centre, const glm::vec3& position, f32 radius,
                 glm::vec3* direction = nullptr)
{
    const glm::vec3 offset = position - centre;
    const f32 distanceSquared = glm::dot(offset, offset);
    if (distanceSquared >= radius * radius)
        return 0.0f;

    const f32 distance = std::sqrt(distanceSquared);
    if (direction)
        *direction = distance > 1.0e-6f ? offset / distance : glm::vec3(0.0f, 1.0f, 0.0f);
    return 1.0f - distance / radius;
}

bool bodyCollides(const RigidBody* body)
{
    return body && body->shape() && body->enabled();
}
} // namespace

// ----------------------------------------------------------------- bodies

void Scene::addBody(RigidBody& body)
{
    if (mPhysicsStepping)
    {
        Log::error("Scene: cannot add/remove a body during a physics step");
        return;
    }
    if (body.mScene == this)
        return;
    if (body.mScene)
        body.mScene->removeBody(body);
    if (body.shape() && body.isDynamic() &&
        (body.shape()->type() == ShapeType::Trimesh || body.shape()->type() == ShapeType::Plane))
    {
        Log::error("Scene: a dynamic body cannot use a static collision shape");
        return;
    }
    body.mScene = this;
    body.mBodyKey = mNextBodyKey++;
    // Sleep becomes this scene's decision the moment a body joins it - see
    // propagateSleep() for why it cannot be the body's own.
    body.setSleepDeferred(true);
    body.pushOwnerPose();
    mRigidBodies.push_back(&body);
    if (body.bodyType() == BodyType::Static)
        mStaticBroadphaseDirty = true;
}

void Scene::removeBody(RigidBody& body)
{
    if (mPhysicsStepping)
    {
        Log::error("Scene: cannot add/remove a body during a physics step");
        return;
    }
    if (body.mScene != this)
        return;
    const auto found = std::find(mRigidBodies.begin(), mRigidBodies.end(), &body);
    if (found == mRigidBodies.end())
    {
        body.mScene = nullptr;
        return;
    }
    if (body.bodyType() == BodyType::Static)
        mStaticBroadphaseDirty = true;
    for (usize i = 0; i < mJoints.size();)
    {
        Joint* joint = mJoints[i];
        if (!joint || joint->bodyA() == &body || joint->bodyB() == &body)
        {
            mJoints[i] = mJoints.back();
            mJoints.pop_back();
        }
        else
            ++i;
    }
    body.setSleepDeferred(false);
    *found = mRigidBodies.back();
    mRigidBodies.pop_back();

    // Every cached pair naming it goes too, or a body handed this key later
    // would inherit impulses from one that no longer exists; a queued event
    // naming it must not reach a callback that could dereference it.
    const u32 key = body.mBodyKey;
    for (auto it = mContactCache.begin(); it != mContactCache.end();)
    {
        const u32 a = static_cast<u32>(it->first >> 32);
        const u32 b = static_cast<u32>(it->first & 0xFFFFFFFFull);
        if (a == key || b == key)
            it = mContactCache.erase(it);
        else
            ++it;
    }
    for (usize i = 0; i < mContacts.size();)
    {
        if (mContacts[i].a == &body || mContacts[i].b == &body)
        {
            mContacts[i] = mContacts.back();
            mContacts.pop_back();
        }
        else
            ++i;
    }
    for (ContactEventInfo& info : mContactEventQueue)
        if (info.bodyA == &body || info.bodyB == &body)
            info.bodyA = info.bodyB = nullptr;
    for (ContactEventInfo& info : mContactEventsDispatching)
        if (info.bodyA == &body || info.bodyB == &body)
            info.bodyA = info.bodyB = nullptr;
    body.mScene = nullptr;
    body.mBodyKey = 0;
}

void Scene::clearPhysics()
{
    if (mPhysicsStepping)
    {
        Log::error("Scene: cannot add/remove a body during a physics step");
        return;
    }
    for (RigidBody* body : mRigidBodies)
    {
        body->setSleepDeferred(false);
        body->mScene = nullptr;
        body->mBodyKey = 0;
    }
    mRigidBodies.clear();
    mContactCache.clear();
    mContacts.clear();
    mJoints.clear();
    mStaticBroadphase.clear();
    mStaticBounds.clear();
    mStaticBodies.clear();
    mStaticBroadphaseDirty = true;
    mContactEventQueue.clear();
    mPairs.clear();
    mPhysicsAccumulator = 0.0f;
    mPhysicsStepIndex = 0;
}

// ----------------------------------------------------------------- joints

void Scene::addJoint(Joint* joint)
{
    if (!joint || !joint->bodyA() || !joint->bodyB())
        return;
    if (joint->bodyA() == joint->bodyB() && !joint->singleBody())
        return;
    if (std::find(mJoints.begin(), mJoints.end(), joint) != mJoints.end())
        return;
    mJoints.push_back(joint);
    joint->mJointScene = this;
    joint->bodyA()->setAwake(true);
    joint->bodyB()->setAwake(true);
}

void Scene::removeJoint(Joint* joint)
{
    const auto found = std::find(mJoints.begin(), mJoints.end(), joint);
    if (found == mJoints.end())
        return;
    joint->mJointScene = nullptr;
    *found = mJoints.back();
    mJoints.pop_back();
}

// ------------------------------------------------------------------- AI

void Scene::addAgent(Agent& agent)
{
    if (agent.mScene == this)
        return;
    if (agent.mScene)
        agent.mScene->removeAgent(agent);
    agent.mScene = this;
    // Same as addBody(): the agent starts where its object already stands,
    // instead of at the origin until the first frame that notices the owner
    // moved. A caller stepping updateAgents() directly never gets that frame.
    agent.pushOwnerPose();
    mAgents.push_back(&agent);
}

void Scene::removeAgent(Agent& agent)
{
    if (agent.mScene != this)
        return;
    if (mAgentsUpdating)
    {
        // Mid-update, the entry is left as a hole and swept up afterwards -
        // the same trick the component event lists use. Compacting here
        // would move an agent the loop has not reached yet down into a slot
        // it has already passed, silently skipping its update; erasing would
        // invalidate the loop outright. Refusing the removal is not an
        // option: the agent is being destroyed either way, and a stale
        // pointer left in the list is exactly what this avoids.
        for (Agent*& entry : mAgents)
            if (entry == &agent)
                entry = nullptr;
        mAgentsDirty = true;
    }
    else
        removePointer(mAgents, &agent);
    agent.mScene = nullptr;
}

void Scene::updateAgents(f32 deltaTime)
{
    // By index and re-reading size(): an agent's behaviors and state machine
    // run user callbacks, and those can destroy an agent (leaving a hole
    // above) or create one, which reallocates. A range-for over this would
    // be walking a vector that moved.
    mAgentsUpdating = true;
    for (usize i = 0; i < mAgents.size(); ++i)
    {
        Agent* agent = mAgents[i];
        if (agent && agent->simulating())
            agent->update(deltaTime);
    }
    mAgentsUpdating = false;

    if (mAgentsDirty)
    {
        usize write = 0;
        for (usize read = 0; read < mAgents.size(); ++read)
            if (mAgents[read])
                mAgents[write++] = mAgents[read];
        mAgents.resize(write);
        mAgentsDirty = false;
    }
}

void Scene::clearAI()
{
    for (Agent* agent : mAgents)
        if (agent)
            agent->mScene = nullptr;
    mAgents.clear();
    mAgentsDirty = false;
}

void Scene::addObstacle(Obstacle& obstacle)
{
    if (obstacle.mScene == this)
        return;
    if (obstacle.mScene)
        obstacle.mScene->removeObstacle(obstacle);
    obstacle.mScene = this;
    obstacle.pushOwnerTransform();
    mObstacleComponents.push_back(&obstacle);
    rebuildObstacleGroup();
}

void Scene::removeObstacle(Obstacle& obstacle)
{
    if (obstacle.mScene != this)
        return;
    removePointer(mObstacleComponents, &obstacle);
    obstacle.mScene = nullptr;
    rebuildObstacleGroup();
}

void Scene::rebuildObstacleGroup()
{
    mObstacleGroup.clear();
    mObstacleGroup.reserve(mObstacleComponents.size());
    for (Obstacle* obstacle : mObstacleComponents)
    {
        const GameObject* object = obstacle->owner();
        if (!obstacle->active() || (object && (!object->isActiveInHierarchy() || object->disposed())))
            continue;
        mObstacleGroup.push_back(obstacle->obstacle());
    }
}

// --------------------------------------------------------------- settings

void Scene::setGravity(const glm::vec3& gravity)
{
    mGravity = gravity;
}

void Scene::setFixedStep(f32 seconds)
{
    if (seconds > 0.0f && std::isfinite(seconds))
        mFixedStep = seconds;
}

void Scene::setSolverSettings(const ContactSolverSettings& settings)
{
    mContactSolver.setSettings(settings);
}

void Scene::setContactEventCallback(ContactEventCallback callback, void* userData)
{
    mContactEventCallback = callback;
    mContactEventUserData = userData;
}

void Scene::setPhysicsStepCallback(PhysicsStepCallback callback, void* userData)
{
    mPhysicsStepCallback = callback;
    mPhysicsStepUserData = userData;
}

void Scene::setContactPersistence(u32 steps)
{
    mContactPersistence = glm::max(steps, 1u);
}

void Scene::setContactMargin(f32 margin)
{
    mContactMargin = glm::max(margin, 0.0f);
    mStaticBroadphaseDirty = true;
}

void Scene::markStaticBroadphaseDirty()
{
    mStaticBroadphaseDirty = true;
}

// ------------------------------------------------------------- broadphase

void Scene::rebuildStaticBroadphase()
{
    mStaticBounds.clear();
    mStaticBodies.clear();
    mStaticBounds.reserve(mRigidBodies.size());
    mStaticBodies.reserve(mRigidBodies.size());
    for (RigidBody* body : mRigidBodies)
    {
        if (!bodyCollides(body) || body->bodyType() != BodyType::Static)
            continue;
        AABB bounds = body->shape()->bounds(body->transform());
        bounds.min -= glm::vec3(mContactMargin);
        bounds.max += glm::vec3(mContactMargin);
        mStaticBounds.push_back(bounds);
        mStaticBodies.push_back(body);
    }
    mStaticBroadphase.build(mStaticBounds.data(), static_cast<u32>(mStaticBounds.size()));
    mStaticBroadphaseDirty = false;
}

// ---------------------------------------------------------- contact cache

u64 Scene::pairKey(const RigidBody& a, const RigidBody& b)
{
    const u32 low = a.mBodyKey < b.mBodyKey ? a.mBodyKey : b.mBodyKey;
    const u32 high = a.mBodyKey < b.mBodyKey ? b.mBodyKey : a.mBodyKey;
    return (static_cast<u64>(low) << 32) | high;
}

void Scene::warmStartFromCache(const CachedContactPair* cached, ContactManifold& manifold)
{
    if (!cached)
        return;
    for (u32 i = 0; i < manifold.count; ++i)
    {
        ContactPoint& point = manifold.points[i];
        // Matched by position, not by an index: the narrowphase can emit the
        // same patch in a different order from one step to the next, and
        // carrying impulses by slot would then swap them around the face.
        f32 best = kMatchDistance;
        const CachedContactPoint* match = nullptr;
        for (u32 j = 0; j < cached->count; ++j)
        {
            const f32 distance = glm::length(point.position - cached->points[j].position);
            if (distance < best)
            {
                best = distance;
                match = &cached->points[j];
            }
        }
        if (!match)
            continue;
        point.normalImpulse = match->normalImpulse;
        point.tangentImpulse[0] = match->tangentImpulse[0];
        point.tangentImpulse[1] = match->tangentImpulse[1];
    }
}

void Scene::storeInCache(const RigidBody& a, const RigidBody& b, const ContactManifold& manifold)
{
    CachedContactPair& cached = mContactCache[pairKey(a, b)];
    // The first manifold of this step starts the list; the rest of the pair's
    // manifolds append to it rather than replacing what came before, or only
    // the last triangle a body rests on keeps its impulses.
    if (cached.lastStep != mPhysicsStepIndex)
        cached.count = 0;
    for (u32 i = 0; i < manifold.count && cached.count < CachedContactPair::MaxPoints; ++i)
    {
        CachedContactPoint& point = cached.points[cached.count++];
        point.position = manifold.points[i].position;
        point.normalImpulse = manifold.points[i].normalImpulse;
        point.tangentImpulse[0] = manifold.points[i].tangentImpulse[0];
        point.tangentImpulse[1] = manifold.points[i].tangentImpulse[1];
    }
    cached.lastStep = mPhysicsStepIndex;
}

void Scene::emitContactExits()
{
    for (auto it = mContactCache.begin(); it != mContactCache.end();)
    {
        if (mPhysicsStepIndex - it->second.lastStep < mContactPersistence)
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
            info.bodyA = it->second.bodyA;
            info.bodyB = it->second.bodyB;
            info.event = ContactEvent::Exit;
            mContactEventQueue.push_back(info);
        }
        it = mContactCache.erase(it);
    }
}

// ------------------------------------------------------------------ sleep

u32 Scene::islandRoot(u32 index)
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

void Scene::propagateSleep()
{
    const usize count = mRigidBodies.size();
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
        const u32 rootA = islandRoot(contact.a->mStepSlot);
        const u32 rootB = islandRoot(contact.b->mStepSlot);
        if (rootA != rootB)
            mIslandParent[rootB] = rootA;
    }

    // An island sleeps only when EVERY body in it has gone quiet. One still
    // moving keeps the whole island awake, which is what stops a box
    // dropping out of a stack that has not finished settling.
    for (usize i = 0; i < count; ++i)
    {
        const RigidBody* body = mRigidBodies[i];
        if (!body->enabled() || !body->isDynamic())
            continue;
        if (!body->canSleep() || body->motion() >= body->sleepEpsilon())
            mIslandAwake[islandRoot(static_cast<u32>(i))] = 1;
    }

    for (usize i = 0; i < count; ++i)
    {
        RigidBody* body = mRigidBodies[i];
        if (!body->enabled() || !body->isDynamic())
            continue;
        const bool shouldBeAwake = mIslandAwake[islandRoot(static_cast<u32>(i))] != 0;
        if (shouldBeAwake != body->awake())
            body->setAwake(shouldBeAwake);
    }
}

// ---------------------------------------------------------------- bullets

void Scene::solveBulletSweeps()
{
    const f32 slop = mContactSolver.settings().slop;
    for (const BulletSweep& sweep : mBulletSweeps)
    {
        RigidBody* sweptBody = sweep.body;
        const glm::vec3 newPosition = sweptBody->position();
        const glm::vec3 delta = newPosition - sweep.previousPosition;

        const f32 distSq = glm::dot(delta, delta);
        if (distSq < slop * slop)
            continue;

        const f32 dist = std::sqrt(distSq);
        Ray ray;
        ray.origin = sweep.previousPosition;
        ray.direction = delta / dist;

        QueryFilter filter;
        filter.ignoredBody = sweptBody;
        WorldRayHit hit;
        if (!raycast(ray, dist, filter, hit))
            continue;
        if (!hit.body || hit.body->isDynamic())
            continue;

        const f32 endSide = glm::dot(newPosition - hit.point, hit.normal);
        if (endSide >= -slop)
            continue;

        f32 safeFraction = hit.distance / dist - slop / dist;
        if (safeFraction < 0.0f)
            safeFraction = 0.0f;
        sweptBody->setPosition(sweep.previousPosition + safeFraction * delta);
    }
}

// ------------------------------------------------------------------- step

void Scene::stepPhysics(f32 duration)
{
    if (duration <= 0.0f || !std::isfinite(duration))
        return;
    if (mPhysicsStepping || mDispatchingContactEvents)
    {
        Log::error("Scene: recursive stepPhysics() is not supported");
        return;
    }
    mPhysicsStepping = true;
    ++mPhysicsStepIndex;

    for (Joint* joint : mJointComponents)
        if (!joint->built() && joint->active() && joint->owner() &&
            joint->owner()->isActiveInHierarchy())
            joint->rebuild();

    if (mStaticBroadphaseDirty)
        rebuildStaticBroadphase();
    mDynamicBroadphase.clear();
    mDynamicBroadphase.reserve(mRigidBodies.size());
    mDynamicProxies.clear();
    mDynamicProxies.reserve(mRigidBodies.size());
    // Gravity/mStepSlot and dynamic-proxy building are one pass over
    // mRigidBodies: both only ever read or write the body at the current
    // index, so there is no ordering dependency between them and no reason to
    // walk the list twice.
    for (u32 i = 0; i < mRigidBodies.size(); ++i)
    {
        RigidBody* body = mRigidBodies[i];
        body->mStepSlot = i;
        // Gravity is set on every dynamic body rather than added as a force,
        // so it reaches them whatever their mass - and so a body the caller
        // gave its own acceleration keeps it only until the scene overwrites
        // it, which is the honest behaviour for a scene that owns gravity.
        if (body->enabled() && body->isDynamic())
        {
            body->setAcceleration(mGravity);
            body->integrateForces(duration);
        }
        if (!bodyCollides(body) || body->bodyType() == BodyType::Static)
            continue;
        BroadphaseProxy proxy;
        proxy.id = i;
        proxy.filter = body->filter();
        proxy.movable = true;
        proxy.bounds = body->shape()->bounds(body->transform());
        // Grown by the contact margin, or the broadphase throws away exactly
        // the pairs the margin exists to keep: a body resting on a surface
        // has its AABB ending where the other one starts, they do not
        // overlap, and the narrowphase is never even asked.
        proxy.bounds.min -= glm::vec3(mContactMargin);
        proxy.bounds.max += glm::vec3(mContactMargin);
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
            const RigidBody* staticBody = mStaticBodies[candidate];
            if (staticBody->mScene != this)
                continue;
            if (!shouldCollide(dynamic.filter, staticBody->filter()) ||
                !Broadphase::overlaps(dynamic.bounds, staticBounds))
                continue;
            const u32 staticSlot = staticBody->mStepSlot;
            mPairs.push_back({glm::min(dynamic.id, staticSlot), glm::max(dynamic.id, staticSlot)});
        }
    }
    mDynamicBroadphase.findPairs(mDynamicPairs);
    mPairs.insert(mPairs.end(), mDynamicPairs.begin(), mDynamicPairs.end());

    mContacts.clear();
    mContacts.reserve(mPairs.size());
    for (const BroadphasePair& pair : mPairs)
    {
        RigidBody& a = *mRigidBodies[pair.a];
        RigidBody& b = *mRigidBodies[pair.b];

        // A trimesh answers with one manifold per triangle touched, so the
        // single-manifold path cannot serve it. Both go through the same
        // vector below and become one Contact each.
        mManifolds.clear();
        const bool aIsMesh = a.shape()->type() == ShapeType::Trimesh;
        const bool bIsMesh = b.shape()->type() == ShapeType::Trimesh;
        if (aIsMesh && bIsMesh)
            continue; // two static meshes never move; nothing to solve
        if (aIsMesh || bIsMesh)
        {
            const RigidBody& convex = aIsMesh ? b : a;
            const RigidBody& mesh = aIsMesh ? a : b;
            if (!Narrowphase::convexTrimesh(*convex.shape(), convex.transform(),
                                            static_cast<const TrimeshShape&>(*mesh.shape()),
                                            mesh.transform(), mManifolds, mContactMargin))
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
            if (!Narrowphase::collide(*a.shape(), a.transform(), *b.shape(), b.transform(),
                                      manifold, mContactMargin))
                continue;
            mManifolds.push_back(manifold);
        }

        const u64 key = pairKey(a, b);
        const auto existing = mContactCache.find(key);
        const bool isNew = existing == mContactCache.end();
        const CachedContactPair* cachedForWarmStart = isNew ? nullptr : &existing->second;

        ContactManifold& manifold = mManifolds[0];
        for (ContactManifold& current : mManifolds)
        {
            warmStartFromCache(cachedForWarmStart, current);

            Contact contact;
            contact.a = &a;
            contact.b = &b;
            contact.manifold = current;
            // Combined the usual way: the geometric mean for friction, the
            // larger for restitution, so one bouncy body is enough to bounce.
            contact.friction = std::sqrt(a.friction() * b.friction());
            contact.restitution = glm::max(a.restitution(), b.restitution());
            mContacts.push_back(contact);
        }

        if (isNew)
        {
            // A new contact is the one moment a sleeping body has to wake -
            // the solver deliberately does not, because a resting stack has
            // contacts every single step.
            a.setAwake(true);
            b.setAwake(true);
        }
        ContactEventInfo info;
        info.bodyA = &a;
        info.bodyB = &b;
        info.event = isNew ? ContactEvent::Enter : ContactEvent::Stay;
        info.normal = manifold.normal;
        info.point = manifold.points[0].position;
        info.penetration = manifold.points[0].penetration;
        mContactEventQueue.push_back(info);
        // Reuses the iterator found above for a pair that was already in the
        // cache, rather than hashing and probing the same key a second time.
        CachedContactPair& cached = isNew ? mContactCache[key] : existing->second;
        cached.bodyA = &a;
        cached.bodyB = &b;
        cached.lastStep = mPhysicsStepIndex;
        cached.reported = true;
    }

    mContactSolver.solve(mContacts.data(), static_cast<u32>(mContacts.size()), mJoints.data(),
                         static_cast<u32>(mJoints.size()), duration);

    // Stored after solving, so what carries into the next step is the impulse
    // the solver settled on rather than the one it started from.
    for (const Contact& contact : mContacts)
        storeInCache(*contact.a, *contact.b, contact.manifold);

    emitContactExits();

    mBulletSweeps.clear();
    for (RigidBody* body : mRigidBodies)
        if (body->enabled() && body->isDynamic() && body->isBullet() && body->awake())
            mBulletSweeps.push_back({body, body->position()});

    for (RigidBody* body : mRigidBodies)
        if (body->enabled())
            body->integrateVelocity(duration);

    solveBulletSweeps();

    // Actions - a vehicle, anything else that reaches into bodies each step -
    // run here, once positions for this step are final and before sleep
    // state is decided from them.
    if (mPhysicsStepCallback)
        mPhysicsStepCallback(duration, mPhysicsStepUserData);

    // After integrating, because the motion average this reads is what
    // integrate() has just updated. Bodies added to this scene have their own
    // sleep deferred, so nothing has dropped out on its own in the meantime.
    propagateSleep();
    mPhysicsStepping = false;
    dispatchContactEvents();
}

void Scene::dispatchContactEvents()
{
    if (!mContactEventCallback || mContactEventQueue.empty())
    {
        mContactEventQueue.clear();
        return;
    }

    // Callbacks are deliberately outside the simulation lock. A body removed
    // by one callback is scrubbed from the events still queued behind it, so
    // a later event never hands the callback a body that is already gone.
    mContactEventsDispatching.swap(mContactEventQueue);
    mDispatchingContactEvents = true;
    for (usize i = 0; i < mContactEventsDispatching.size(); ++i)
    {
        const ContactEventInfo& info = mContactEventsDispatching[i];
        if (info.bodyA && info.bodyB && mContactEventCallback)
            mContactEventCallback(info, mContactEventUserData);
    }
    mDispatchingContactEvents = false;
    mContactEventsDispatching.clear();
}

void Scene::updatePhysics(f32 deltaTime)
{
    if (deltaTime <= 0.0f || !std::isfinite(deltaTime))
        return;
    mPhysicsAccumulator += deltaTime;
    const f32 budget = mFixedStep * static_cast<f32>(mMaxPhysicsStepsPerUpdate);
    if (mPhysicsAccumulator > budget)
        mPhysicsAccumulator = budget;
    while (mPhysicsAccumulator >= mFixedStep)
    {
        stepPhysics(mFixedStep);
        mPhysicsAccumulator -= mFixedStep;
    }
}

// -------------------------------------------------------------- debug draw

void Scene::debugDrawPhysicsShapes() const
{
    for (const RigidBody* body : mRigidBodies)
    {
        if (!bodyCollides(body))
            continue;
        // Hue says which shape it is, brightness says what the simulation is
        // doing with it. Both matter and they do not compete: telling a
        // capsule from a box at a glance is how a wrong collider is spotted,
        // and a body that never dims is one that never settled.
        Color color = Color::Gray;
        if (body->bodyType() == BodyType::Kinematic)
            color = Color(230, 200, 60, 255);
        else if (body->isDynamic())
        {
            switch (body->shape()->type())
            {
            case ShapeType::Sphere:  color = Color(80, 220, 200, 255); break;
            case ShapeType::Box:     color = Color(110, 220, 90, 255); break;
            case ShapeType::Capsule: color = Color(220, 110, 220, 255); break;
            case ShapeType::ConvexHull: color = Color(230, 150, 60, 255); break;
            default:                 color = Color::White; break;
            }
            // Asleep is the same colour at a third of the brightness, so a
            // stack settling reads as the whole tower dimming together.
            if (!body->awake())
                color = Color(static_cast<u8>(color.r() / 3), static_cast<u8>(color.g() / 3),
                              static_cast<u8>(color.b() / 3), 255);
        }
        body->shape()->debugDraw(body->transform(), color);
    }
}

void Scene::debugDrawPhysicsContacts() const
{
    for (const Contact& contact : mContacts)
        for (u32 i = 0; i < contact.manifold.count; ++i)
        {
            const glm::vec3& point = contact.manifold.points[i].position;
            // The normal is drawn scaled by the impulse it is carrying, so a
            // stack shows where the weight actually goes.
            const f32 scale = 0.05f + contact.manifold.points[i].normalImpulse * 0.02f;
            DebugDraw().line(point, point + contact.manifold.normal * scale, Color::Red);
        }
}

void Scene::debugDrawPhysicsJoints() const
{
    constexpr f32 axisLength = 0.35f;
    for (const Joint* joint : mJoints)
    {
        if (!joint || !joint->enabled())
            continue;
        const glm::vec3 anchorA = joint->anchorWorldA();
        const glm::vec3 anchorB = joint->anchorWorldB();
        DebugDraw().line(anchorA, anchorB, Color::Yellow);
        if (joint->hasAxis())
            DebugDraw().line(anchorA, anchorA + joint->axisWorld() * axisLength, Color::Cyan);
    }
}

void Scene::debugDrawObstacles() const
{
    // The live set only: an obstacle switched off steers nobody, so drawing
    // it here would show a wall that is not there. Selecting it in the
    // editor still draws it, switched off or not.
    const Color obstacleColor(200, 80, 220, 255);
    for (const Obstacle* obstacle : mObstacleComponents)
    {
        const GameObject* object = obstacle->owner();
        if (!obstacle->active() || (object && !object->isActiveInHierarchy()))
            continue;
        debugDrawObstacleShape(*obstacle, obstacleColor);
    }
}

void Scene::debugDrawObstacleShape(const Obstacle& obstacle, Color color)
{
    const GameObject* object = obstacle.owner();
    if (!object)
        return;

    constexpr f32 kNormalArrowLength = 0.75f;
    constexpr f32 kNormalArrowHead = 0.15f;
    constexpr f32 kPlaneHalfExtent = 1.0f; // finite 2m patch standing in for the infinite plane

    const glm::vec3 position = object->globalPosition();
    const glm::vec3 rightAxis = object->right();
    const glm::vec3 upAxis = object->up();
    const glm::vec3 forwardAxis = object->forward();
    const glm::mat4 transform =
        glm::translate(glm::mat4(1.0f), position) * glm::mat4_cast(object->globalRotation());

    // Where the normal arrow(s) start from - the surface, not the centre, so
    // it reads as pointing away from the shape rather than through it.
    glm::vec3 arrowOrigin = position;

    switch (obstacle.shape())
    {
    case ObstacleShape::Sphere:
        SphereShape(obstacle.radius()).debugDraw(transform, color);
        // A sphere has no single face normal; the forward axis stands in as
        // one sampled meridian, just to show which way seenFrom() points.
        arrowOrigin = position + forwardAxis * obstacle.radius();
        break;
    case ObstacleShape::Box:
        BoxShape(glm::vec3(obstacle.width(), obstacle.height(), obstacle.depth()) * 0.5f)
            .debugDraw(transform, color);
        break;
    case ObstacleShape::Rectangle:
    case ObstacleShape::Plane:
    {
        const f32 halfWidth =
            obstacle.shape() == ObstacleShape::Plane ? kPlaneHalfExtent : obstacle.width() * 0.5f;
        const f32 halfHeight =
            obstacle.shape() == ObstacleShape::Plane ? kPlaneHalfExtent : obstacle.height() * 0.5f;
        const glm::vec3 halfSide = rightAxis * halfWidth;
        const glm::vec3 halfUp = upAxis * halfHeight;
        const glm::vec3 corners[4] = {position - halfSide - halfUp, position + halfSide - halfUp,
                                      position + halfSide + halfUp, position - halfSide + halfUp};
        for (u32 i = 0; i < 4; ++i)
            DebugDraw().line(corners[i], corners[(i + 1) % 4], color);
        break;
    }
    }

    // seenFrom() decides which side steers agents away and is invisible in
    // the geometry above - the arrow(s) are the only place it shows.
    const AI::ObstacleSeenFrom seenFrom = obstacle.seenFrom();
    if (seenFrom != AI::ObstacleSeenFrom::Inside)
        DebugDraw().arrow(arrowOrigin, arrowOrigin + forwardAxis * kNormalArrowLength, 0.0f,
                          kNormalArrowHead, color);
    if (seenFrom != AI::ObstacleSeenFrom::Outside)
        DebugDraw().arrow(arrowOrigin, arrowOrigin - forwardAxis * kNormalArrowLength, 0.0f,
                          kNormalArrowHead, color);
}

// ---------------------------------------------------------------- queries

bool Scene::raycast(const Ray& ray, f32 maxDistance, const QueryFilter& filter,
                    WorldRayHit& hit) const
{
    bool found = false;
    f32 nearest = maxDistance;
    for (RigidBody* body : mRigidBodies)
    {
        if (!bodyCollides(body) || !filter.accepts(body, body->filter()))
            continue;

        ShapeRayHit shapeHit;
        if (!Narrowphase::raycast(*body->shape(), body->transform(), ray, nearest, shapeHit))
            continue;

        nearest = shapeHit.distance;
        hit.body = body;
        hit.point = shapeHit.point;
        hit.normal = shapeHit.normal;
        hit.distance = shapeHit.distance;
        found = true;
    }
    return found;
}

void Scene::overlapSphere(const glm::vec3& centre, f32 radius, const QueryFilter& filter,
                          std::vector<RigidBody*>& out) const
{
    out.clear();
    for (RigidBody* body : mRigidBodies)
    {
        if (!bodyCollides(body) || !filter.accepts(body, body->filter()))
            continue;
        if (Narrowphase::overlapSphere(*body->shape(), body->transform(), centre, radius))
            out.push_back(body);
    }
}

void Scene::queryAABB(const AABB& bounds, const QueryFilter& filter,
                      std::vector<RigidBody*>& out) const
{
    out.clear();
    for (RigidBody* body : mRigidBodies)
    {
        if (!bodyCollides(body) || !filter.accepts(body, body->filter()))
            continue;
        if (Broadphase::overlaps(bounds, body->shape()->bounds(body->transform())))
            out.push_back(body);
    }
}

// ---------------------------------------------------------- area effects

u32 Scene::applyRadialImpulse(const glm::vec3& centre, f32 radius, f32 strength,
                              const QueryFilter& filter)
{
    if (!finiteVector(centre) || !(radius > 0.0f) || !std::isfinite(radius) ||
        !std::isfinite(strength) || strength == 0.0f)
        return 0;

    u32 affected = 0;
    for (RigidBody* body : mRigidBodies)
    {
        if (!body->enabled() || !body->isDynamic() || !filter.accepts(body, body->filter()))
            continue;

        glm::vec3 direction;
        const f32 weight = radialWeight(centre, body->position(), radius, &direction);
        if (weight <= 0.0f)
            continue;
        body->applyLinearImpulse(direction * (strength * weight));
        ++affected;
    }
    return affected;
}

u32 Scene::addRadialForce(const glm::vec3& centre, f32 radius, f32 strength,
                          const QueryFilter& filter)
{
    if (!finiteVector(centre) || !(radius > 0.0f) || !std::isfinite(radius) ||
        !std::isfinite(strength) || strength == 0.0f)
        return 0;

    u32 affected = 0;
    for (RigidBody* body : mRigidBodies)
    {
        if (!body->enabled() || !body->isDynamic() || !filter.accepts(body, body->filter()))
            continue;

        glm::vec3 direction;
        const f32 weight = radialWeight(centre, body->position(), radius, &direction);
        if (weight <= 0.0f)
            continue;
        body->addForce(direction * (strength * weight));
        ++affected;
    }
    return affected;
}

u32 Scene::addDirectionalForce(const glm::vec3& centre, f32 radius, const glm::vec3& force,
                               const QueryFilter& filter)
{
    if (!finiteVector(centre) || !finiteVector(force) || !(radius > 0.0f) ||
        !std::isfinite(radius) || glm::dot(force, force) == 0.0f)
        return 0;

    u32 affected = 0;
    for (RigidBody* body : mRigidBodies)
    {
        if (!body->enabled() || !body->isDynamic() || !filter.accepts(body, body->filter()))
            continue;

        const f32 weight = radialWeight(centre, body->position(), radius);
        if (weight <= 0.0f)
            continue;
        body->addForce(force * weight);
        ++affected;
    }
    return affected;
}

} // namespace Radion
