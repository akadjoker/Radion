#ifndef RADION_OBSTACLE_COMPONENT_H
#define RADION_OBSTACLE_COMPONENT_H

// Named apart from AI's Obstacle.h: include path order would otherwise shadow it for every #include "Obstacle.h".

#include "Component.h"
#include "Obstacle.h"

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

    // Rebuilt on shape/dimension change; do not keep the pointer across it.
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
    // Runs every Scene::update(), in and out of Play, so the shape follows the gizmo while paused.
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
