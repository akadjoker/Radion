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
    Collider,
    Count
};

enum ComponentEventFlags : u8
{
    ComponentEventNone = 0,
    ComponentEventUpdate = 1 << 0,
    ComponentEventLateUpdate = 1 << 1
};

class Component;

// A component's ComponentType normally identifies its class outright, so
// finding the slot occupied is proof enough and this default says so. Where
// several classes share one type - the four light classes all register as
// ComponentType::Light - it is specialised next to them to consult whatever
// runtime discriminator they carry. GameObject::contains<T>() and
// findComponent<T>() go through here; getComponent<T>() does not, and casts
// on the slot alone.
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
    ComponentType mType;
    u8 mEvents;
    bool mActive = true;
    bool mStarted = false;
    // Scene owns flat event lists so its frame loop only touches components
    // that actually exist. These are tombstone positions while a callback is
    // in flight; Scene compacts the lists once the frame is complete.
    usize mSceneUpdateIndex = InvalidSceneListIndex;
    usize mSceneLateUpdateIndex = InvalidSceneListIndex;
};

} // namespace Radion

#endif // RADION_COMPONENT_H
