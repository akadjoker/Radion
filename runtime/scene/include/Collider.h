#ifndef RADION_COLLIDER_H
#define RADION_COLLIDER_H

#include "Component.h"
#include "Math.h"
#include "Types.h"

#include <vector>

namespace Radion
{

class TriangleOctree;
enum class CollisionResponse : u8;

enum class ColliderShape : u8
{
    Sphere,
    Box,
    Capsule,
    Mesh
};

class Collider final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Collider;

    struct Contact
    {
        Collider* other = nullptr;
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
        Math::vec3 point{0.0f};
    };

    ~Collider() override;

    void setSphere(f32 radius);
    void setBox(const Math::vec3& halfExtents);
    void setCapsule(f32 radius, f32 height); // height is total, cap to cap
    void setMesh(const TriangleOctree* octree); // borrowed, not owned
    // One-time bake, so the object must be GameObject::isStatic().
    bool rebuildMeshFromRenderer();

    ColliderShape shape() const;
    f32 radius() const;
    const Math::vec3& halfExtents() const;
    f32 height() const;
    const TriangleOctree* mesh() const;

    void setType(u32 type);
    u32 type() const;

    void setResponse(CollisionResponse response);
    CollisionResponse response() const;

    f32 capsuleSegmentHalfHeight() const;

    AABB worldBounds() const;

    usize contactCount() const;
    const Contact& contactAt(usize index) const;

private:
    friend class GameObject;
    friend class CollisionWorld;

    Collider();

    void clearContacts();
    void addContact(Collider* other, const Math::vec3& normal, const Math::vec3& point);
    void releaseOwnedMesh();

    ColliderShape mShape = ColliderShape::Sphere;
    f32 mRadius = 0.5f;
    Math::vec3 mHalfExtents{0.5f, 0.5f, 0.5f};
    f32 mHeight = 1.0f;
    const TriangleOctree* mMesh = nullptr;
    TriangleOctree* mOwnedMesh = nullptr; // built by rebuildMeshFromRenderer(), freed here
    u32 mType = 0;
    CollisionResponse mResponse{};
    std::vector<Contact> mContacts;
};

} // namespace Radion

#endif // RADION_COLLIDER_H
