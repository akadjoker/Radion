#ifndef RADION_GAME_OBJECT_H
#define RADION_GAME_OBJECT_H

#include "Component.h"
#include "Types.h"

#include "Math.h"
#include "Math.h"
#include <string>
#include <utility>
#include <vector>

namespace Radion
{

class Scene;

enum class TransformSpace : u8
{
    Local,
    Parent,
    World
};

enum GameObjectFlags : u32
{
    GameObjectActive = 1 << 0,
    GameObjectVisible = 1 << 1,
    GameObjectDispose = 1 << 2,
    GameObjectStatic = 1 << 3
};

enum GameObjectDebugFlags : u32
{
    DebugNone = 0,
    DebugObjectAxis = 1 << 0,
    DebugMeshBounds = 1 << 1,
    DebugSubMeshBounds = 1 << 2,
    DebugVertexNormals = 1 << 3,
    DebugVertexTangents = 1 << 4,
    DebugSkeleton = 1 << 5
};

class GameObject final
{
public:
    ~GameObject();

    GameObject(const GameObject&) = delete;
    GameObject& operator=(const GameObject&) = delete;

    // Unique within the owning Scene, and only there: the counter lives in
    // Scene, not in a global, so the same id can name a different object in
    // another Scene. Stable for the object's whole life, and the key a saved
    // scene uses for every reference it stores (parent, target object,
    // active camera), which is why Scene::createGameObject() can be asked to
    // restore a specific one on load. 0 means the object is not in a Scene
    // yet - the root's id, and never a valid reference.
    u64 id() const;
    Scene* scene() const;
    const std::string& name() const;
    void setName(const std::string& name);
    const std::string& tag() const;
    void setTag(const std::string& tag);

    bool active() const;
    void setActive(bool active);
    bool visible() const;
    void setVisible(bool visible);
    bool isActiveInHierarchy() const;
    bool isVisibleInHierarchy() const;
    // Both of the above in one walk up the parents. The two flags share a
    // word, so a level costs one load and one mask test instead of two
    // separate chains - and every submission loop in Scene wants them
    // together anyway.
    bool isActiveAndVisibleInHierarchy() const;
    bool isStatic() const;
    void setStatic(bool isStatic);
    void dispose();
    bool disposed() const;

    u32 debugFlags() const;
    void setDebugFlags(u32 flags);
    void addDebugFlags(u32 flags);
    void removeDebugFlags(u32 flags);
    bool hasDebugFlag(u32 flag) const;

    GameObject* parent() const;
    GameObject* root();
    const GameObject* root() const;
    usize childCount() const;
    GameObject* child(usize index) const;
    usize childIndex(const GameObject* child) const;
    GameObject* findChild(const std::string& name, bool recursive = true) const;

    // Only safe on objects neither side has registered with a Scene yet -
    // building a detached hierarchy before Scene::add() puts its root in.
    // Once either object is registered, these refuse and warn instead of
    // silently moving/deleting a branch Scene's own mObjects/mRenderers/
    // mLights/mCameras never hear about: use Scene::add()/remove()/destroy()
    // on a registered object instead. See docs/review.md finding 9.
    bool addChild(GameObject* child);
    GameObject* removeChild(GameObject* child);
    bool deleteChild(GameObject* child);
    void deleteChildren();
    bool moveChild(GameObject* child, usize index);
    bool moveChildUp(GameObject* child);
    bool moveChildDown(GameObject* child);

    template <class T, class... Args> T* addComponent(Args&&... args)
    {
        if (getComponent<T>())
            return nullptr;
        T* component = new T(std::forward<Args>(args)...);
        if (!attachComponent(component))
        {
            delete component;
            return nullptr;
        }
        return component;
    }

    // Casts on the slot alone: with several classes sharing a ComponentType
    // this hands back the wrong one rather than nothing. Ask for the base
    // class here, or use as<T>() when the exact class matters.
    template <class T> T* getComponent() const
    {
        return static_cast<T*>(mComponents[static_cast<u8>(T::Type)]);
    }

    // Whether ANY component slot is occupied - what tells a genuine "empty"
    // marker node (a spawn point, a rope anchor) apart from an object that
    // just happens not to have the one component a particular caller asked
    // about.
    bool hasAnyComponent() const
    {
        for (const Component* component : mComponents)
            if (component)
                return true;
        return false;
    }

    // Whether the object carries exactly a T, consulting the class's own
    // discriminator through ComponentMatch - which is what tells a spot light
    // from a point light, since the ComponentType cannot.
    template <class T> bool contains() const
    {
        const Component* component = mComponents[static_cast<u8>(T::Type)];
        return component != nullptr && ComponentMatch<T>::test(component);
    }

    // getComponent<T>() with that same check: null when the slot holds a
    // different class, instead of a pointer to something T is not. Named to
    // find rather than get because, unlike getComponent<T>(), it is expected
    // to come back empty.
    template <class T> T* findComponent() const
    {
        Component* component = mComponents[static_cast<u8>(T::Type)];
        return (component && ComponentMatch<T>::test(component)) ? static_cast<T*>(component)
                                                                 : nullptr;
    }

    template <class T> bool removeComponent()
    {
        return removeComponent(T::Type);
    }

    const Math::vec3& position() const;
    const Math::quat& rotation() const;
    const Math::vec3& scale() const;
    void setPosition(const Math::vec3& position);
    void setRotation(const Math::quat& rotation);
    void setRotationDegrees(const Math::vec3& rotation);
    void setScale(const Math::vec3& scale);
    void setGlobalPosition(const Math::vec3& position);
    void setGlobalRotation(const Math::quat& rotation);
    void translate(const Math::vec3& offset, TransformSpace space = TransformSpace::Local);
    void moveForward(f32 distance);
    void moveRight(f32 distance);
    void moveUp(f32 distance);
    void rotate(const Math::quat& rotation, TransformSpace space = TransformSpace::Local);
    void rotate(const Math::vec3& axis, f32 degrees, TransformSpace space = TransformSpace::Local);
    void yaw(f32 degrees, TransformSpace space = TransformSpace::Local);
    void pitch(f32 degrees, TransformSpace space = TransformSpace::Local);
    void roll(f32 degrees, TransformSpace space = TransformSpace::Local);
    void lookAt(const Math::vec3& target, const Math::vec3& up = Math::vec3(0, 1, 0));
    bool rotateTowards(const Math::vec3& target, f32 maxDegreesDelta,
                       const Math::vec3& up = Math::vec3(0, 1, 0));
    const Math::mat4& localTransform() const;
    const Math::mat4& globalTransform() const;
    const Math::mat4& previousGlobalTransform() const;
    Math::vec3 globalPosition() const;
    Math::quat globalRotation() const;
    Math::vec3 globalScale() const;
    Math::vec3 right() const;
    Math::vec3 up() const;
    Math::vec3 forward() const;
    f32 distanceTo(const GameObject& other) const;
    Math::vec3 directionTo(const GameObject& other) const;

private:
    friend class Scene;
    friend class MeshRenderer;

    explicit GameObject(const std::string& name = std::string());

    // The unguarded mechanics addChild()/removeChild()/deleteChildren() wrap.
    // Scene calls these directly - it is the one caller that is allowed to
    // move/delete a registered branch, because it is the one caller that
    // also updates mObjects/mRenderers/mLights/mCameras and mScene to match.
    // The destructor uses removeChildRaw() too: detaching from a parent that
    // is still registered must always succeed, or the parent is left holding
    // a pointer into freed memory.
    bool addChildRaw(GameObject* child);
    GameObject* removeChildRaw(GameObject* child);
    void deleteChildrenRaw();

    bool attachComponent(Component* component);
    bool removeComponent(ComponentType type);
    void deleteComponents();
    void updateComponents(f32 deltaTime);
    void lateUpdateComponents(f32 deltaTime);
    void updateComponent(Component* component, f32 deltaTime);
    void lateUpdateComponent(Component* component, f32 deltaTime);
    void flushPendingComponentDeletes();
    bool isAncestorOf(const GameObject* object) const;
    void invalidateTransform();
    void invalidateChildren();
    void invalidateSpatialMembership();
    void updateLocalTransform() const;
    void updateGlobalTransform() const;

    std::string mName;
    std::string mTag;
    u64 mId = 0;
    u32 mFlags = GameObjectActive | GameObjectVisible;
    u32 mDebugFlags = DebugNone;
    Scene* mScene = nullptr;
    GameObject* mParent = nullptr;
    std::vector<GameObject*> mChildren;
    Component* mComponents[static_cast<u8>(ComponentType::Count)]{};
    // A component whose onStart()/onUpdate()/onLateUpdate() calls
    // removeComponent<Self>() on its own owner used to be an implicit
    // `delete this`: the slot cleared immediately, but so did the component
    // object, out from under the very callback still running on it. The
    // slot still clears immediately here - getComponent<T>() must stop
    // seeing it right away - only the delete is deferred until the callback
    // loop that might still be on this component's stack has finished.
    std::vector<Component*> mPendingComponentDeletes;
    u32 mComponentCallbackDepth = 0;
    Math::vec3 mPosition = Math::vec3(0);
    Math::quat mRotation = Math::quat(1, 0, 0, 0);
    Math::vec3 mScale = Math::vec3(1);
    mutable Math::mat4 mLocalTransform = Math::mat4(1);
    mutable Math::mat4 mGlobalTransform = Math::mat4(1);
    Math::mat4 mPreviousGlobalTransform = Math::mat4(1);
    mutable Math::vec3 mGlobalPosition = Math::vec3(0);
    mutable Math::quat mGlobalRotation = Math::quat(1, 0, 0, 0);
    mutable Math::vec3 mGlobalScale = Math::vec3(1);
    mutable bool mLocalDirty = false;
    mutable bool mGlobalDirty = false;
    bool mPreviousGlobalTransformValid = false;
    bool mDynamicBoundsQueued = false;
    bool mPendingDestroyQueued = false;
};

} // namespace Radion

#endif // RADION_GAME_OBJECT_H
