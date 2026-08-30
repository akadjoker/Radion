#include "PCH.h"

#include "Collider.h"

#include "AssetManager.h"
#include "CollisionWorld.h"
#include "GameObject.h"
#include "MeshRenderer.h"
#include "Octree.h"
#include "collision/CollisionShape.h"

namespace Radion
{

Collider::Collider() : Component(Type)
{
}

Collider::~Collider()
{
    releaseOwnedMesh();
}

void Collider::releaseOwnedMesh()
{
    delete mOwnedMesh;
    mOwnedMesh = nullptr;
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
    releaseOwnedMesh();
    mShape = ColliderShape::Mesh;
    mMesh = octree;
}

bool Collider::rebuildMeshFromRenderer()
{
    // Mesh is the shape either way - a renderer added later, or a mesh
    // re-assigned on the sibling MeshRenderer, is what a rebuild call is
    // for. Only whether there is geometry to bake right now differs.
    releaseOwnedMesh();
    mShape = ColliderShape::Mesh;
    mMesh = nullptr;

    GameObject* object = owner();
    if (!object)
        return false;

    // The octree is baked once, in world space, at whatever transform the
    // object has right now, and never re-baked on its own - the same
    // promise GameObject::isStatic() already makes for the renderer's own
    // static BVH (SceneBVH::build()). Refusing here is what keeps a moved
    // object from colliding against where its mesh used to be.
    if (!object->isStatic())
    {
        Log::warning("Collider: '%s' needs GameObject::setStatic(true) before a Mesh shape can "
                     "be baked",
                     object->name().c_str());
        return false;
    }

    MeshRenderer* renderer = object->getComponent<MeshRenderer>();
    if (!renderer)
        return false;

    MeshData meshData;
    if (!Assets().buildMeshData(Assets().meshDesc(renderer->mesh()), meshData) ||
        meshData.indices.size() < 3)
        return false;

    CollisionMesh collisionMesh;
    collisionMesh.positions = std::move(meshData.positions);
    collisionMesh.indices = std::move(meshData.indices);

    mOwnedMesh = new TriangleOctree();
    mOwnedMesh->addCollisionMesh(collisionMesh, object->globalTransform());
    mOwnedMesh->build();
    mMesh = mOwnedMesh;
    return true;
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
