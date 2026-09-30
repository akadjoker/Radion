#ifndef RADION_AI_OBSTACLE_H
#define RADION_AI_OBSTACLE_H

#include "Math.h"
#include <vector>

namespace Radion
{
class Agent;
}

namespace Radion::AI
{

class Obstacle;

using ObstacleGroup = std::vector<Obstacle*>;

// Solid side ("seenFrom"): outside = solid chunk, inside = enclosed clear space, both = hollow shell.
enum class ObstacleSeenFrom
{
    Outside,
    Inside,
    Both
};

struct PathIntersection
{
    bool intersect = false;
    float distance = 0.0f;
    Math::vec3 surfacePoint = Math::vec3(0.0f);
    Math::vec3 surfaceNormal = Math::vec3(0.0f);
    Math::vec3 steerHint = Math::vec3(0.0f);
    bool vehicleOutside = true;
    const Obstacle* obstacle = nullptr;

    // Lateral component of steerHint scaled to maxForce, if the intersection is within minTimeToCollision.
    Math::vec3 steerToAvoidIfNeeded(const Radion::Agent& vehicle, float minTimeToCollision) const;
};

class Obstacle
{
public:
    Obstacle() = default;
    virtual ~Obstacle() = default;

    Math::vec3 steerToAvoid(const Radion::Agent& vehicle, float minTimeToCollision) const;

    static Math::vec3 steerToAvoidObstacles(const Radion::Agent& vehicle, float minTimeToCollision,
                                           const ObstacleGroup& obstacles);

    static void firstPathIntersectionWithObstacleGroup(const Radion::Agent& vehicle,
                                                       const ObstacleGroup& obstacles,
                                                       PathIntersection& nearest,
                                                       PathIntersection& next);

    virtual void findIntersectionWithVehiclePath(const Radion::Agent& vehicle,
                                                 PathIntersection& pi) const = 0;

    ObstacleSeenFrom seenFrom() const
    {
        return mSeenFrom;
    }
    void setSeenFrom(ObstacleSeenFrom sf)
    {
        mSeenFrom = sf;
    }

private:
    ObstacleSeenFrom mSeenFrom = ObstacleSeenFrom::Outside;
};

class SphereObstacle final : public Obstacle
{
public:
    float radius = 1.0f;
    Math::vec3 center = Math::vec3(0.0f);

    SphereObstacle() = default;
    SphereObstacle(float r, const Math::vec3& c) : radius(r), center(c)
    {
    }

    void findIntersectionWithVehiclePath(const Radion::Agent& vehicle,
                                         PathIntersection& pi) const override;
};

// Planar obstacle in the local XY plane; the +Z half-space is outside.
class PlaneObstacle : public Obstacle
{
public:
    PlaneObstacle() = default;
    PlaneObstacle(const Math::vec3& s, const Math::vec3& u, const Math::vec3& f, const Math::vec3& p)
        : mSide(s), mUp(u), mForward(f), mPosition(p)
    {
    }

    void findIntersectionWithVehiclePath(const Radion::Agent& vehicle,
                                         PathIntersection& pi) const override;

    virtual bool xyPointInsideShape(const Math::vec3& point, float radius) const
    {
        (void)point;
        (void)radius;
        return true;
    }

    Math::vec3 side() const
    {
        return mSide;
    }
    Math::vec3 up() const
    {
        return mUp;
    }
    Math::vec3 forward() const
    {
        return mForward;
    }
    Math::vec3 position() const
    {
        return mPosition;
    }
    void setSide(const Math::vec3& s)
    {
        mSide = s;
    }
    void setUp(const Math::vec3& u)
    {
        mUp = u;
    }
    void setForward(const Math::vec3& f)
    {
        mForward = f;
    }
    void setPosition(const Math::vec3& p)
    {
        mPosition = p;
    }

private:
    Math::vec3 localizeDirection(const Math::vec3& gd) const;
    Math::vec3 localizePosition(const Math::vec3& gp) const;
    Math::vec3 globalizePosition(const Math::vec3& lp) const;
    Math::vec3 globalizeDirection(const Math::vec3& ld) const;

    Math::vec3 mSide = Math::vec3(1.0f, 0.0f, 0.0f);
    Math::vec3 mUp = Math::vec3(0.0f, 1.0f, 0.0f);
    Math::vec3 mForward = Math::vec3(0.0f, 0.0f, 1.0f);
    Math::vec3 mPosition = Math::vec3(0.0f);
};

class RectangleObstacle final : public PlaneObstacle
{
public:
    float width = 1.0f;
    float height = 1.0f;

    RectangleObstacle() = default;
    RectangleObstacle(float w, float h) : width(w), height(h)
    {
    }
    RectangleObstacle(float w, float h, const Math::vec3& s, const Math::vec3& u, const Math::vec3& f,
                      const Math::vec3& p, ObstacleSeenFrom sf)
        : PlaneObstacle(s, u, f, p), width(w), height(h)
    {
        setSeenFrom(sf);
    }

    bool xyPointInsideShape(const Math::vec3& point, float radius) const override;
};

class BoxObstacle final : public PlaneObstacle
{
public:
    float width = 1.0f;
    float height = 1.0f;
    float depth = 1.0f;

    BoxObstacle() = default;
    BoxObstacle(float w, float h, float d, const Math::vec3& s, const Math::vec3& u,
                const Math::vec3& f, const Math::vec3& p)
        : PlaneObstacle(s, u, f, p), width(w), height(h), depth(d)
    {
    }

    void findIntersectionWithVehiclePath(const Radion::Agent& vehicle,
                                         PathIntersection& pi) const override;
};

} // namespace Radion::AI

#endif // RADION_AI_OBSTACLE_H
