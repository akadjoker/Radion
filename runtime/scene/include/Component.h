#ifndef RADION_COMPONENT_H
#define RADION_COMPONENT_H

#include "Types.h"

namespace Radion
{

class GameObject;

enum class ComponentType : u8
{
    Camera,
    MeshRenderer,
    ManualMesh,
    Animator,
    BoneAttachment,
    FreeFly,
    FPS,
    Orbit,
    Maya,
    ActionRunner,
    PathAnimator,
    RibbonTrail,
    Terrain,
    Landscape,
    Road,
    Forest,
    Grass,
    Hair,
    Ocean,
    ParticleEffect,
    Light,
    Script,
    CharacterController,
    Billboard,
    Text3D,
    ParticleEmitter,
    Beam,
    ReflectionProbe,
    ThirdPerson,
    SelfDestroy,
    Waypoints,
    NavMeshSurface,
    VoxelWorld,
    Collider,
    AudioPlayer,
    TiledTerrain,
    UiCanvas,
    UiPanel,
    UiLabel,
    UiButton,
    UiCheckBox,
    UiSlider,
    RigidBody,
    Joint,
    Agent,
    Obstacle,
    Count
};

enum ComponentEventFlags : u8
{
    ComponentEventNone = 0,
    ComponentEventUpdate = 1 << 0,
    ComponentEventLateUpdate = 1 << 1
};

// Types modelling one authoritative state (physics body, terrain, skeleton) stay exclusive.
bool componentTypeAllowsMultiple(ComponentType type);

class Component;

// Where several classes share one ComponentType (the light classes), specialise this to consult a runtime discriminator.
template <class T> struct ComponentMatch
{
    static bool test(const Component*)
    {
        return true;
    }
};

class Component
{
public:
    static constexpr usize InvalidSceneListIndex = static_cast<usize>(-1);

    virtual ~Component() = default;

    Component(const Component&) = delete;
    Component& operator=(const Component&) = delete;

    GameObject* owner() const;
    ComponentType type() const;
    u32 id() const;
    bool active() const;
    void setActive(bool active);

protected:
    explicit Component(ComponentType type, u8 events = ComponentEventNone);
    virtual void onAwake();
    virtual void onStart();
    virtual void onEnable();
    virtual void onDisable();
    virtual void onUpdate(f32 deltaTime);
    virtual void onLateUpdate(f32 deltaTime);
    virtual void onDestroy();

private:
    friend class GameObject;
    friend class Scene;

    void attached();
    void detached();

    GameObject* mOwner = nullptr;
    Component* mPreviousSibling = nullptr;
    Component* mNextSibling = nullptr;
    ComponentType mType;
    u32 mLocalId = 0;
    u8 mEvents;
    bool mActive = true;
    bool mStarted = false;
    // Tombstone positions while a callback is in flight; Scene compacts the lists after the frame.
    usize mSceneUpdateIndex = InvalidSceneListIndex;
    usize mSceneLateUpdateIndex = InvalidSceneListIndex;
};

} // namespace Radion

#endif // RADION_COMPONENT_H
