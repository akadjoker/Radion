#include "PCH.h"

#include "GameObject.h"

#include "Scene.h"

#include <glm/gtc/matrix_transform.hpp>

namespace Radion
{

namespace
{

bool finite(const glm::vec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool valid(const glm::quat& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z) &&
           std::isfinite(value.w) && glm::dot(value, value) > 0.000001f;
}

} // namespace

// mId stays 0 here: the Scene that creates the object is what stamps it,
// out of its own counter. See Scene::createGameObject().
GameObject::GameObject(const std::string& name) : mName(name)
{
}

GameObject::~GameObject()
{
    deleteComponents();
    deleteChildrenRaw();
    if (mParent)
        mParent->removeChildRaw(this);
}

u64 GameObject::id() const
{
    return mId;
}
Scene* GameObject::scene() const
{
    return mScene;
}
const std::string& GameObject::name() const
{
    return mName;
}
void GameObject::setName(const std::string& name)
{
    mName = name;
}
const std::string& GameObject::tag() const
{
    return mTag;
}
void GameObject::setTag(const std::string& tag)
{
    mTag = tag;
}
bool GameObject::active() const
{
    return (mFlags & GameObjectActive) != 0;
}
bool GameObject::visible() const
{
    return (mFlags & GameObjectVisible) != 0;
}

void GameObject::setActive(bool active)
{
    active ? mFlags |= GameObjectActive : mFlags &= ~GameObjectActive;
}

void GameObject::setVisible(bool visible)
{
    visible ? mFlags |= GameObjectVisible : mFlags &= ~GameObjectVisible;
}

bool GameObject::isStatic() const
{
    return (mFlags & GameObjectStatic) != 0;
}

void GameObject::setStatic(bool isStatic)
{
    if (isStatic == this->isStatic())
        return;
    isStatic ? mFlags |= GameObjectStatic : mFlags &= ~GameObjectStatic;
    invalidateSpatialMembership();
}

bool GameObject::isActiveInHierarchy() const
{
    for (const GameObject* object = this; object; object = object->mParent)
        if (!object->active())
            return false;
    return true;
}

bool GameObject::isVisibleInHierarchy() const
{
    for (const GameObject* object = this; object; object = object->mParent)
        if (!object->visible())
            return false;
    return true;
}

bool GameObject::isActiveAndVisibleInHierarchy() const
{
    constexpr u32 kBoth = GameObjectActive | GameObjectVisible;
    for (const GameObject* object = this; object; object = object->mParent)
        if ((object->mFlags & kBoth) != kBoth)
            return false;
    return true;
}

void GameObject::dispose()
{
    if (this != root())
        mFlags |= GameObjectDispose;
}

bool GameObject::disposed() const
{
    return (mFlags & GameObjectDispose) != 0;
}

u32 GameObject::debugFlags() const
{
    return mDebugFlags;
}

void GameObject::setDebugFlags(u32 flags)
{
    if (mDebugFlags == flags)
        return;
    const u32 previous = mDebugFlags;
    mDebugFlags = flags;
    if (mScene)
        mScene->debugFlagsChanged(this, previous);
}

void GameObject::addDebugFlags(u32 flags)
{
    setDebugFlags(mDebugFlags | flags);
}
void GameObject::removeDebugFlags(u32 flags)
{
    setDebugFlags(mDebugFlags & ~flags);
}
bool GameObject::hasDebugFlag(u32 flag) const
{
    return (mDebugFlags & flag) != 0;
}
GameObject* GameObject::parent() const
{
    return mParent;
}

GameObject* GameObject::root()
{
    GameObject* object = this;
    while (object->mParent)
        object = object->mParent;
    return object;
}

const GameObject* GameObject::root() const
{
    return const_cast<GameObject*>(this)->root();
}
usize GameObject::childCount() const
{
    return mChildren.size();
}
GameObject* GameObject::child(usize index) const
{
    return index < mChildren.size() ? mChildren[index] : nullptr;
}

usize GameObject::childIndex(const GameObject* object) const
{
    for (usize i = 0; i < mChildren.size(); ++i)
        if (mChildren[i] == object)
            return i;
    return mChildren.size();
}

GameObject* GameObject::findChild(const std::string& name, bool recursive) const
{
    for (GameObject* object : mChildren)
    {
        if (object->name() == name)
            return object;
        if (recursive)
            if (GameObject* found = object->findChild(name, true))
                return found;
    }
    return nullptr;
}

bool GameObject::addChildRaw(GameObject* object)
{
    if (!object || object == this || object->mParent || object->isAncestorOf(this))
        return false;
    object->mParent = this;
    mChildren.push_back(object);
    object->mGlobalDirty = true;
    if (object->mScene)
        object->mScene->queueDynamicBoundsUpdate(object);
    object->invalidateChildren();
    return true;
}

GameObject* GameObject::removeChildRaw(GameObject* object)
{
    const usize index = childIndex(object);
    if (!object || index == mChildren.size())
        return nullptr;
    mChildren.erase(mChildren.begin() + index);
    object->mParent = nullptr;
    object->mGlobalDirty = true;
    if (object->mScene)
        object->mScene->queueDynamicBoundsUpdate(object);
    object->invalidateChildren();
    return object;
}

void GameObject::deleteChildrenRaw()
{
    while (!mChildren.empty())
    {
        GameObject* object = mChildren.back();
        mChildren.pop_back();
        object->mParent = nullptr;
        delete object;
    }
}

bool GameObject::addChild(GameObject* object)
{
    // mRoot is always registered (Scene sets its mScene at construction), so
    // this also catches the common case of adding straight under the root.
    if (object && (mScene || object->mScene))
    {
        Log::warning("GameObject: addChild() on a registered object ('%s' into '%s') - use "
                     "Scene::add() instead, or the branch never reaches mObjects/mRenderers/"
                     "mLights/mCameras",
                     object->name().c_str(), mName.c_str());
        return false;
    }
    return addChildRaw(object);
}

GameObject* GameObject::removeChild(GameObject* object)
{
    if (object && (mScene || object->mScene))
    {
        Log::warning("GameObject: removeChild() on a registered object ('%s' from '%s') - use "
                     "Scene::remove()/destroy() instead, or Scene's own lists keep pointing at "
                     "a detached branch",
                     object->name().c_str(), mName.c_str());
        return nullptr;
    }
    return removeChildRaw(object);
}

bool GameObject::deleteChild(GameObject* object)
{
    object = removeChild(object);
    if (!object)
        return false;
    delete object;
    return true;
}

void GameObject::deleteChildren()
{
    if (mScene)
    {
        Log::warning("GameObject: deleteChildren() on a registered object ('%s') - use "
                     "Scene::destroy() on each child instead, or Scene's own lists keep pointing "
                     "at deleted objects",
                     mName.c_str());
        return;
    }
    deleteChildrenRaw();
}

bool GameObject::moveChild(GameObject* object, usize index)
{
    const usize current = childIndex(object);
    if (current == mChildren.size() || index >= mChildren.size())
        return false;
    if (current == index)
        return true;
    mChildren.erase(mChildren.begin() + current);
    mChildren.insert(mChildren.begin() + index, object);
    return true;
}

bool GameObject::moveChildUp(GameObject* object)
{
    const usize index = childIndex(object);
    return index > 0 && index < mChildren.size() && moveChild(object, index - 1);
}

bool GameObject::moveChildDown(GameObject* object)
{
    const usize index = childIndex(object);
    return index + 1 < mChildren.size() && moveChild(object, index + 1);
}

bool GameObject::attachComponent(Component* component)
{
    if (!component || component->mOwner)
        return false;
    const u8 index = static_cast<u8>(component->type());
    if (index >= static_cast<u8>(ComponentType::Count))
        return false;
    component->mOwner = this;
    component->mLocalId = mNextComponentId++;
    component->mPreviousSibling = mComponentTails[index];
    component->mNextSibling = nullptr;
    Component*& head = mComponents[index];
    if (!head)
        head = component;
    else
        mComponentTails[index]->mNextSibling = component;
    mComponentTails[index] = component;
    component->attached();
    if (mScene)
        mScene->componentAdded(component);
    return true;
}

bool GameObject::removeComponent(Component* component)
{
    if (!component || component->mOwner != this)
        return false;
    if (mScene)
        mScene->componentRemoved(component);
    component->detached();
    component->mOwner = nullptr;
    unlinkComponent(component);
    // Only the actual free waits: a callback still running on this component
    // - reached through the loop in updateComponents()/lateUpdateComponents()
    // - must not have it deleted out from under itself.
    if (mComponentCallbackDepth > 0)
        mPendingComponentDeletes.push_back(component);
    else
        delete component;
    return true;
}

void GameObject::unlinkComponent(Component* component)
{
    const u8 index = static_cast<u8>(component->type());
    if (component->mPreviousSibling)
        component->mPreviousSibling->mNextSibling = component->mNextSibling;
    else
        mComponents[index] = component->mNextSibling;
    if (component->mNextSibling)
        component->mNextSibling->mPreviousSibling = component->mPreviousSibling;
    else
        mComponentTails[index] = component->mPreviousSibling;
}

void GameObject::flushPendingComponentDeletes()
{
    for (Component* component : mPendingComponentDeletes)
        delete component;
    mPendingComponentDeletes.clear();
}

void GameObject::deleteComponents()
{
    for (u8 i = 0; i < static_cast<u8>(ComponentType::Count); ++i)
        while (mComponents[i])
            removeComponent(mComponents[i]);
}

void GameObject::updateComponents(f32 deltaTime)
{
    for (Component* head : mComponents)
        for (Component* component = head; component; component = component->mNextSibling)
            updateComponent(component, deltaTime);
}

void GameObject::lateUpdateComponents(f32 deltaTime)
{
    for (Component* head : mComponents)
        for (Component* component = head; component; component = component->mNextSibling)
            lateUpdateComponent(component, deltaTime);
}

void GameObject::updateComponent(Component* component, f32 deltaTime)
{
    if (!component || !component->active())
        return;
    ++mComponentCallbackDepth;
    if (!component->mStarted)
    {
        component->mStarted = true;
        component->onStart();
    }
    if ((component->mEvents & ComponentEventUpdate) != 0)
        component->onUpdate(deltaTime);
    if (--mComponentCallbackDepth == 0)
        flushPendingComponentDeletes();
}

void GameObject::lateUpdateComponent(Component* component, f32 deltaTime)
{
    if (!component || !component->active() ||
        (component->mEvents & ComponentEventLateUpdate) == 0)
        return;
    ++mComponentCallbackDepth;
    component->onLateUpdate(deltaTime);
    if (--mComponentCallbackDepth == 0)
        flushPendingComponentDeletes();
}

bool GameObject::isAncestorOf(const GameObject* object) const
{
    while (object)
    {
        if (object == this)
            return true;
        object = object->mParent;
    }
    return false;
}

const glm::vec3& GameObject::position() const
{
    return mPosition;
}

const glm::quat& GameObject::rotation() const
{
    return mRotation;
}

const glm::vec3& GameObject::scale() const
{
    return mScale;
}

void GameObject::setPosition(const glm::vec3& position)
{
    if (!finite(position))
    {
        Log::warning("GameObject '%s': rejected non-finite position", name().c_str());
        return;
    }
    mPosition = position;
    invalidateTransform();
}

void GameObject::setRotation(const glm::quat& rotation)
{
    if (!valid(rotation))
    {
        Log::warning("GameObject '%s': rejected invalid rotation", name().c_str());
        return;
    }
    mRotation = glm::normalize(rotation);
    invalidateTransform();
}

void GameObject::setRotationDegrees(const glm::vec3& rotation)
{
    if (!finite(rotation))
    {
        Log::warning("GameObject '%s': rejected non-finite Euler rotation", name().c_str());
        return;
    }
    setRotation(glm::quat(glm::radians(rotation)));
}

void GameObject::setScale(const glm::vec3& scale)
{
    if (!finite(scale))
    {
        Log::warning("GameObject '%s': rejected non-finite scale", name().c_str());
        return;
    }
    // Terrain/Road/VegetationGrid invert the global transform, and mesh
    // upload computes an inverse-transpose of its 3x3 - a zero component
    // makes both singular and propagates NaN into normals, bounds and
    // culling. Hiding an object is what enabled/visible are for; a scale
    // this small is a caller bug, not a legitimate "make it disappear".
    constexpr f32 kEpsilon = 0.000001f;
    if (glm::abs(scale.x) < kEpsilon || glm::abs(scale.y) < kEpsilon ||
        glm::abs(scale.z) < kEpsilon)
    {
        Log::warning("GameObject '%s': rejected near-zero scale component - use "
                     "setActive(false)/setVisible(false) to hide an object instead",
                     name().c_str());
        return;
    }
    mScale = scale;
    invalidateTransform();
}

void GameObject::setGlobalPosition(const glm::vec3& position)
{
    if (!finite(position))
    {
        Log::warning("GameObject '%s': rejected non-finite global position", name().c_str());
        return;
    }

    if (!mParent)
        setPosition(position);
    else
    {
        const glm::vec3 parentScale = mParent->globalScale();
        if (std::abs(parentScale.x) <= 0.000001f || std::abs(parentScale.y) <= 0.000001f ||
            std::abs(parentScale.z) <= 0.000001f)
        {
            Log::warning("GameObject '%s': cannot set global position through zero parent scale",
                         name().c_str());
            return;
        }
        setPosition(
            glm::vec3(glm::inverse(mParent->globalTransform()) * glm::vec4(position, 1.0f)));
    }
}

void GameObject::setGlobalRotation(const glm::quat& rotation)
{
    if (!valid(rotation))
    {
        Log::warning("GameObject '%s': rejected invalid global rotation", name().c_str());
        return;
    }

    setRotation(mParent ? glm::inverse(mParent->globalRotation()) * glm::normalize(rotation)
                        : rotation);
}

void GameObject::translate(const glm::vec3& offset, TransformSpace space)
{
    if (!finite(offset))
        return;
    if (space == TransformSpace::Local)
        setPosition(mPosition + mRotation * offset);
    else if (space == TransformSpace::Parent)
        setPosition(mPosition + offset);
    else
        setGlobalPosition(globalPosition() + offset);
}

void GameObject::moveForward(f32 distance)
{
    translate(glm::vec3(0.0f, 0.0f, -distance), TransformSpace::Local);
}

void GameObject::moveRight(f32 distance)
{
    translate(glm::vec3(distance, 0.0f, 0.0f), TransformSpace::Local);
}

void GameObject::moveUp(f32 distance)
{
    translate(glm::vec3(0.0f, distance, 0.0f), TransformSpace::Local);
}

void GameObject::rotate(const glm::quat& rotation, TransformSpace space)
{
    if (!valid(rotation))
        return;
    const glm::quat normalized = glm::normalize(rotation);
    if (space == TransformSpace::Local)
        setRotation(mRotation * normalized);
    else if (space == TransformSpace::Parent)
        setRotation(normalized * mRotation);
    else
        setGlobalRotation(normalized * globalRotation());
}

void GameObject::rotate(const glm::vec3& axis, f32 degrees, TransformSpace space)
{
    if (!finite(axis) || !std::isfinite(degrees) || glm::dot(axis, axis) <= 0.000001f)
        return;
    rotate(glm::angleAxis(glm::radians(degrees), glm::normalize(axis)), space);
}

void GameObject::yaw(f32 degrees, TransformSpace space)
{
    rotate(glm::vec3(0.0f, 1.0f, 0.0f), degrees, space);
}

void GameObject::pitch(f32 degrees, TransformSpace space)
{
    rotate(glm::vec3(1.0f, 0.0f, 0.0f), degrees, space);
}

void GameObject::roll(f32 degrees, TransformSpace space)
{
    rotate(glm::vec3(0.0f, 0.0f, -1.0f), degrees, space);
}

void GameObject::lookAt(const glm::vec3& target, const glm::vec3& up)
{
    const glm::vec3 origin = globalPosition();
    const glm::vec3 direction = target - origin;
    if (glm::dot(direction, direction) <= 0.0f || glm::dot(up, up) <= 0.0f ||
        glm::dot(glm::cross(direction, up), glm::cross(direction, up)) <= 0.000001f)
        return;

    const glm::mat4 view = glm::lookAt(origin, target, glm::normalize(up));
    setGlobalRotation(glm::normalize(glm::quat_cast(glm::inverse(view))));
}

bool GameObject::rotateTowards(const glm::vec3& target, f32 maxDegreesDelta, const glm::vec3& up)
{
    const glm::vec3 origin = globalPosition();
    const glm::vec3 direction = target - origin;
    if (glm::dot(direction, direction) <= 0.0f || glm::dot(up, up) <= 0.0f ||
        glm::dot(glm::cross(direction, up), glm::cross(direction, up)) <= 0.000001f ||
        maxDegreesDelta < 0.0f)
        return false;

    const glm::quat desired = glm::normalize(
        glm::quat_cast(glm::inverse(glm::lookAt(origin, target, glm::normalize(up)))));
    glm::quat current = globalRotation();
    glm::quat adjusted = desired;
    f32 cosine = glm::dot(current, adjusted);
    if (cosine < 0.0f)
    {
        adjusted = -adjusted;
        cosine = -cosine;
    }

    const f32 angle = glm::degrees(2.0f * std::acos(glm::clamp(cosine, 0.0f, 1.0f)));
    if (angle <= maxDegreesDelta || angle <= 0.0001f)
    {
        setGlobalRotation(adjusted);
        return true;
    }

    setGlobalRotation(glm::slerp(current, adjusted, maxDegreesDelta / angle));
    return false;
}

const glm::mat4& GameObject::localTransform() const
{
    updateLocalTransform();
    return mLocalTransform;
}

const glm::mat4& GameObject::globalTransform() const
{
    updateGlobalTransform();
    return mGlobalTransform;
}

const glm::mat4& GameObject::previousGlobalTransform() const
{
    return mPreviousGlobalTransformValid ? mPreviousGlobalTransform : globalTransform();
}

glm::vec3 GameObject::globalPosition() const
{
    updateGlobalTransform();
    return mGlobalPosition;
}

glm::quat GameObject::globalRotation() const
{
    updateGlobalTransform();
    return mGlobalRotation;
}

glm::vec3 GameObject::globalScale() const
{
    updateGlobalTransform();
    return mGlobalScale;
}

glm::vec3 GameObject::right() const
{
    return globalRotation() * glm::vec3(1.0f, 0.0f, 0.0f);
}

glm::vec3 GameObject::up() const
{
    return globalRotation() * glm::vec3(0.0f, 1.0f, 0.0f);
}

glm::vec3 GameObject::forward() const
{
    return globalRotation() * glm::vec3(0.0f, 0.0f, -1.0f);
}

f32 GameObject::distanceTo(const GameObject& other) const
{
    return glm::length(other.globalPosition() - globalPosition());
}

glm::vec3 GameObject::directionTo(const GameObject& other) const
{
    const glm::vec3 direction = other.globalPosition() - globalPosition();
    const f32 lengthSquared = glm::dot(direction, direction);
    return lengthSquared > 0.0f ? direction / std::sqrt(lengthSquared) : glm::vec3(0.0f);
}

void GameObject::invalidateTransform()
{
    mLocalDirty = true;
    mGlobalDirty = true;
    if (mScene)
        mScene->queueDynamicBoundsUpdate(this);
    invalidateChildren();
}

void GameObject::invalidateChildren()
{
    for (GameObject* child : mChildren)
    {
        child->mGlobalDirty = true;
        if (child->mScene)
            child->mScene->queueDynamicBoundsUpdate(child);
        child->invalidateChildren();
    }
}

void GameObject::invalidateSpatialMembership()
{
    if (mScene)
        mScene->invalidateSpatialIndexes();
}

void GameObject::updateLocalTransform() const
{
    if (!mLocalDirty)
        return;
    mLocalTransform = glm::translate(glm::mat4(1.0f), mPosition) * glm::mat4_cast(mRotation) *
                      glm::scale(glm::mat4(1.0f), mScale);
    mLocalDirty = false;
}

void GameObject::updateGlobalTransform() const
{
    if (!mGlobalDirty)
        return;

    if (mParent)
    {
        mParent->updateGlobalTransform();
        mGlobalRotation = glm::normalize(mParent->mGlobalRotation * mRotation);
        mGlobalScale = mParent->mGlobalScale * mScale;
        mGlobalPosition = mParent->mGlobalPosition +
                          mParent->mGlobalRotation * (mParent->mGlobalScale * mPosition);
    }
    else
    {
        mGlobalPosition = mPosition;
        mGlobalRotation = mRotation;
        mGlobalScale = mScale;
    }

    mGlobalTransform = glm::translate(glm::mat4(1.0f), mGlobalPosition) *
                       glm::mat4_cast(mGlobalRotation) * glm::scale(glm::mat4(1.0f), mGlobalScale);
    mGlobalDirty = false;
}

} // namespace Radion
