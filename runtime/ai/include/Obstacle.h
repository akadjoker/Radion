#ifndef RADION_AI_OBSTACLE_H
#define RADION_AI_OBSTACLE_H

#include "Entity.h"

#include <vector>

namespace Radion::AI
{

class Obstacle;

using ObstacleGroup = std::vector<Obstacle*>;

// Which side of an obstacle is solid ("seenFrom"): outside = a solid chunk in
// clear space, inside = clear space enclosed by solid, both = hollow shell.
enum class ObstacleSeenFrom
{
    Outside,
    Inside,
    Both
};

// Information about an intersection of a vehicle's path with an obstacle.
struct PathIntersection
{
    bool intersect = false;                    // was an intersection found?
    float distance = 0.0f;                     // how far the intersection is from the vehicle
    Math::vec3 surfacePoint = Math::vec3(0.0f);  // position of intersection
    Math::vec3 surfaceNormal = Math::vec3(0.0f); // unit normal at intersection
    Math::vec3 steerHint = Math::vec3(0.0f);     // where to steer away from it
    bool vehicleOutside = true;                // is the vehicle outside the obstacle?
    const Obstacle* obstacle = nullptr;        // obstacle the path intersects

    // Determine steering once path intersections have been found: lateral
    // component of steerHint, scaled to the vehicle's maxForce, if the
    // intersection is inside minTimeToCollision seconds of travel.
    Math::vec3 steerToAvoidIfNeeded(const Entity& vehicle, float minTimeToCollision) const;
};

class Obstacle
{
public:
    Obstacle() = default;
    virtual ~Obstacle() = default;

    // Compute steering for a vehicle to avoid this obstacle, if needed.
    Math::vec3 steerToAvoid(const Entity& vehicle, float minTimeToCollision) const;

    // Apply steerToAvoid to the nearest obstacle in a group.
    static Math::vec3 steerToAvoidObstacles(const Entity& vehicle, float minTimeToCollision,
                                           const ObstacleGroup& obstacles);

    // Find the first vehicle-path intersection in a group, storing the nearest
    // in `nearest`.
    static void firstPathIntersectionWithObstacleGroup(const Entity& vehicle,
                                                       const ObstacleGroup& obstacles,
                                                       PathIntersection& nearest,
                                                       PathIntersection& next);

    // Find the first intersection of the vehicle's path with this obstacle.
    virtual void findIntersectionWithVehiclePath(const Entity& vehicle,
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

// A simple ball-shaped obstacle.
class SphereObstacle final : public Obstacle
{
public:
    float radius = 1.0f;
    Math::vec3 center = Math::vec3(0.0f);

    SphereObstacle() = default;
    SphereObstacle(float r, const Math::vec3& c) : radius(r), center(c)
    {
    }

    void findIntersectionWithVehiclePath(const Entity& vehicle,
                                         PathIntersection& pi) const override;
};

// A planar obstacle (the XY / side-up plane of a local space). The +Z
// (forward) half-space is "outside". Base class for rectangle/box faces.
class PlaneObstacle : public Obstacle
{
public:
    PlaneObstacle() = default;
    PlaneObstacle(const Math::vec3& s, const Math::vec3& u, const Math::vec3& f, const Math::vec3& p)
        : mSide(s), mUp(u), mForward(f), mPosition(p)
    {
    }

    void findIntersectionWithVehiclePath(const Entity& vehicle,
                                         PathIntersection& pi) const override;

    // Is a point on the local XY plane inside this obstacle's 2D shape?
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

// A rectangular obstacle, centered on the XY (side/up) plane of a local space.
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

// A box-shaped (cuboid) obstacle, centered on and aligned with a local space.
class BoxObstacle final : public PlaneObstacle
{
public:
    float width = 1.0f;  // extent along local X (side)
    float height = 1.0f; // extent along local Y (up)
    float depth = 1.0f;  // extent along local Z (forward)

    BoxObstacle() = default;
    BoxObstacle(float w, float h, float d, const Math::vec3& s, const Math::vec3& u,
                const Math::vec3& f, const Math::vec3& p)
        : PlaneObstacle(s, u, f, p), width(w), height(h), depth(d)
    {
    }

    void findIntersectionWithVehiclePath(const Entity& vehicle,
                                         PathIntersection& pi) const override;
};

} // namespace Radion::AI

#endif // RADION_AI_OBSTACLE_H
