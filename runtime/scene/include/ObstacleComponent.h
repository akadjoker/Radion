#ifndef RADION_OBSTACLE_COMPONENT_H
#define RADION_OBSTACLE_COMPONENT_H

// ObstacleComponent.h - Radion::Obstacle, the scene-side wrapper around one
// AI::Obstacle shape (Obstacle.h, runtime/ai). Named apart from that header
// on purpose: radion_scene's include path lists runtime/scene/include before
// runtime/ai/include, so a second file also called "Obstacle.h" living there
// would shadow the AI one for every #include "Obstacle.h" in the whole
// target - including AI's own Obstacle.cpp. Every existing #include
// "Obstacle.h" keeps reaching AI::Obstacle unchanged.
//
// One component, not one class per shape: Collider and RigidBody already
// hold their shape kind in an enum rather than a class hierarchy, and the
// four ObstacleShape cases differ in exactly the same way - a handful of
// floats, not behavior.

#include "Component.h"
#include "Obstacle.h" // AI::Obstacle, AI::ObstacleSeenFrom

namespace Radion
{

class Scene;

enum class ObstacleShape : u8
{
    Sphere,
    Plane,
    Rectangle,
    Box
};

class Obstacle final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Obstacle;

    void setSphere(f32 radius);
    void setPlane();
    void setRectangle(f32 width, f32 height);
    void setBox(f32 width, f32 height, f32 depth);

    ObstacleShape shape() const
    {
        return mShape;
    }
    f32 radius() const
    {
        return mRadius;
    }
    f32 width() const
    {
        return mWidth;
    }
    f32 height() const
    {
        return mHeight;
    }
    f32 depth() const
    {
        return mDepth;
    }

    void setSeenFrom(AI::ObstacleSeenFrom seenFrom);
    AI::ObstacleSeenFrom seenFrom() const
    {
        return mSeenFrom;
    }

    // Owned; rebuilt whenever the shape or its dimensions change, so a
    // pointer taken before that call must not be kept across it.
    AI::Obstacle* obstacle() const
    {
        return mObstacle;
    }

private:
    friend class GameObject;
    friend class Scene;

    Obstacle();
    ~Obstacle() override;

    void rebuildOwnedShape();
    // Pushes the owner's world transform into the owned AI::Obstacle - runs
    // every Scene::update(), in and out of Play, so the shape follows the
    // gizmo with the game paused too (Scene::debugDrawObstacles() is what
    // makes that visible).
    void pushOwnerTransform();

    Scene* mScene = nullptr;
    ObstacleShape mShape = ObstacleShape::Sphere;
    f32 mRadius = 1.0f;
    f32 mWidth = 1.0f;
    f32 mHeight = 1.0f;
    f32 mDepth = 1.0f;
    AI::ObstacleSeenFrom mSeenFrom = AI::ObstacleSeenFrom::Outside;
    AI::Obstacle* mObstacle = nullptr; // owned
};

} // namespace Radion

#endif // RADION_OBSTACLE_COMPONENT_H
