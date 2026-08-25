#include "PCH.h"

#include "Collider.h"

#include "CollisionWorld.h"
#include "GameObject.h"
#include "Octree.h"
#include "collision/CollisionShape.h"

namespace Radion
{

Collider::Collider() : Component(Type)
{
}

void Collider::setSphere(f32 radius)
{
    mShape = ColliderShape::Sphere;
    mRadius = radius;
}
void Collider::setBox(const glm::vec3& halfExtents)
{
    mShape = ColliderShape::Box;
    mHalfExtents = halfExtents;
}
void Collider::setCapsule(f32 radius, f32 height)
{
    mShape = ColliderShape::Capsule;
    mRadius = radius;
    mHeight = height;
}
void Collider::setMesh(const TriangleOctree* octree)
{
    mShape = ColliderShape::Mesh;
    mMesh = octree;
}

ColliderShape Collider::shape() const
{
    return mShape;
}
f32 Collider::radius() const
{
    return mRadius;
}
const glm::vec3& Collider::halfExtents() const
{
    return mHalfExtents;
}
f32 Collider::height() const
{
    return mHeight;
}
const TriangleOctree* Collider::mesh() const
{
    return mMesh;
}

void Collider::setType(u32 type)
{
    mType = type;
}
u32 Collider::type() const
{
    return mType;
}

void Collider::setResponse(CollisionResponse response)
{
    mResponse = response;
}
CollisionResponse Collider::response() const
{
    return mResponse;
}

f32 Collider::capsuleSegmentHalfHeight() const
{
    return glm::max(mHeight * 0.5f - mRadius, 0.0f);
}

AABB Collider::worldBounds() const
{
    const GameObject* object = owner();
    if (!object)
        return AABB();

    switch (mShape)
    {
    case ColliderShape::Sphere:
        return Physics::SphereShape(mRadius).bounds(object->globalTransform());
    case ColliderShape::Box:
        return Physics::BoxShape(mHalfExtents).bounds(object->globalTransform());
    case ColliderShape::Capsule:
        return Physics::CapsuleShape(mRadius, capsuleSegmentHalfHeight())
            .bounds(object->globalTransform());
    case ColliderShape::Mesh:
        return mMesh ? mMesh->bounds() : AABB(); // already baked to world space
    }
    return AABB();
}

usize Collider::contactCount() const
{
    return mContacts.size();
}
const Collider::Contact& Collider::contactAt(usize index) const
{
    return mContacts[index];
}

void Collider::clearContacts()
{
    mContacts.clear();
}
void Collider::addContact(Collider* other, const glm::vec3& normal, const glm::vec3& point)
{
    Contact contact;
    contact.other = other;
    contact.normal = normal;
    contact.point = point;
    mContacts.push_back(contact);
}

} // namespace Radion
