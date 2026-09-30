#include "PCH.h"

#include "ObstacleComponent.h"

#include "GameObject.h"
#include "Scene.h"

namespace Radion
{

Obstacle::Obstacle() : Component(Type)
{
    rebuildOwnedShape();
}

Obstacle::~Obstacle()
{
    // A loose obstacle would leave the Scene's group holding a dangling pointer (as Agent/RigidBody).
    if (mScene)
        mScene->removeObstacle(*this);
    delete mObstacle;
}

void Obstacle::setSphere(f32 radius)
{
    mShape = ObstacleShape::Sphere;
    mRadius = radius;
    rebuildOwnedShape();
}

void Obstacle::setPlane()
{
    mShape = ObstacleShape::Plane;
    rebuildOwnedShape();
}

void Obstacle::setRectangle(f32 width, f32 height)
{
    mShape = ObstacleShape::Rectangle;
    mWidth = width;
    mHeight = height;
    rebuildOwnedShape();
}

void Obstacle::setBox(f32 width, f32 height, f32 depth)
{
    mShape = ObstacleShape::Box;
    mWidth = width;
    mHeight = height;
    mDepth = depth;
    rebuildOwnedShape();
}

void Obstacle::setSeenFrom(AI::ObstacleSeenFrom seenFrom)
{
    mSeenFrom = seenFrom;
    if (mObstacle)
        mObstacle->setSeenFrom(mSeenFrom);
}

void Obstacle::rebuildOwnedShape()
{
    delete mObstacle;
    switch (mShape)
    {
    case ObstacleShape::Sphere:
        mObstacle = new AI::SphereObstacle(mRadius, Math::vec3(0.0f));
        break;
    case ObstacleShape::Plane:
        mObstacle = new AI::PlaneObstacle();
        break;
    case ObstacleShape::Rectangle:
        mObstacle = new AI::RectangleObstacle(mWidth, mHeight);
        break;
    case ObstacleShape::Box:
        mObstacle = new AI::BoxObstacle(mWidth, mHeight, mDepth, Math::vec3(1.0f, 0.0f, 0.0f),
                                        Math::vec3(0.0f, 1.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f),
                                        Math::vec3(0.0f));
        break;
    }
    mObstacle->setSeenFrom(mSeenFrom);

    // The Scene's ObstacleGroup holds the deleted AI::Obstacle* by address; refill now or an avoidance step reads freed memory.
    if (mScene)
        mScene->rebuildObstacleGroup();
    pushOwnerTransform();
}

void Obstacle::pushOwnerTransform()
{
    const GameObject* object = owner();
    if (!object)
        return;

    if (mShape == ObstacleShape::Sphere)
    {
        static_cast<AI::SphereObstacle*>(mObstacle)->center = object->globalPosition();
        return;
    }

    AI::PlaneObstacle* plane = static_cast<AI::PlaneObstacle*>(mObstacle);
    plane->setSide(object->right());
    plane->setUp(object->up());
    plane->setForward(object->forward());
    plane->setPosition(object->globalPosition());
}

} // namespace Radion
