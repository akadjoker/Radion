#ifndef RADION_AI_OBSTACLE_H
#define RADION_AI_OBSTACLE_H

#include <glm/glm.hpp>
#include <vector>

namespace Radion
{
class Agent;
}

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
    glm::vec3 surfacePoint = glm::vec3(0.0f);  // position of intersection
    glm::vec3 surfaceNormal = glm::vec3(0.0f); // unit normal at intersection
    glm::vec3 steerHint = glm::vec3(0.0f);     // where to steer away from it
    bool vehicleOutside = true;                // is the vehicle outside the obstacle?
    const Obstacle* obstacle = nullptr;        // obstacle the path intersects

    // Determine steering once path intersections have been found: lateral
    // component of steerHint, scaled to the vehicle's maxForce, if the
    // intersection is inside minTimeToCollision seconds of travel.
    glm::vec3 steerToAvoidIfNeeded(const Radion::Agent& vehicle, float minTimeToCollision) const;
};

class Obstacle
{
public:
    Obstacle() = default;
    virtual ~Obstacle() = default;

    // Compute steering for a vehicle to avoid this obstacle, if needed.
    glm::vec3 steerToAvoid(const Radion::Agent& vehicle, float minTimeToCollision) const;

    // Apply steerToAvoid to the nearest obstacle in a group.
    static glm::vec3 steerToAvoidObstacles(const Radion::Agent& vehicle, float minTimeToCollision,
                                           const ObstacleGroup& obstacles);

    // Find the first vehicle-path intersection in a group, storing the nearest
    // in `nearest`.
    static void firstPathIntersectionWithObstacleGroup(const Radion::Agent& vehicle,
                                                       const ObstacleGroup& obstacles,
                                                       PathIntersection& nearest,
                                                       PathIntersection& next);

    // Find the first intersection of the vehicle's path with this obstacle.
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

// A simple ball-shaped obstacle.
class SphereObstacle final : public Obstacle
{
public:
    float radius = 1.0f;
    glm::vec3 center = glm::vec3(0.0f);

    SphereObstacle() = default;
    SphereObstacle(float r, const glm::vec3& c) : radius(r), center(c)
    {
    }

    void findIntersectionWithVehiclePath(const Radion::Agent& vehicle,
                                         PathIntersection& pi) const override;
};

// A planar obstacle (the XY / side-up plane of a local space). The +Z
// (forward) half-space is "outside". Base class for rectangle/box faces.
class PlaneObstacle : public Obstacle
{
public:
    PlaneObstacle() = default;
    PlaneObstacle(const glm::vec3& s, const glm::vec3& u, const glm::vec3& f, const glm::vec3& p)
        : mSide(s), mUp(u), mForward(f), mPosition(p)
    {
    }

    void findIntersectionWithVehiclePath(const Radion::Agent& vehicle,
                                         PathIntersection& pi) const override;

    // Is a point on the local XY plane inside this obstacle's 2D shape?
    virtual bool xyPointInsideShape(const glm::vec3& point, float radius) const
    {
        (void)point;
        (void)radius;
        return true;
    }

    glm::vec3 side() const
    {
        return mSide;
    }
    glm::vec3 up() const
    {
        return mUp;
    }
    glm::vec3 forward() const
    {
        return mForward;
    }
    glm::vec3 position() const
    {
        return mPosition;
    }
    void setSide(const glm::vec3& s)
    {
        mSide = s;
    }
    void setUp(const glm::vec3& u)
    {
        mUp = u;
    }
    void setForward(const glm::vec3& f)
    {
        mForward = f;
    }
    void setPosition(const glm::vec3& p)
    {
        mPosition = p;
    }

private:
    glm::vec3 localizeDirection(const glm::vec3& gd) const;
    glm::vec3 localizePosition(const glm::vec3& gp) const;
    glm::vec3 globalizePosition(const glm::vec3& lp) const;
    glm::vec3 globalizeDirection(const glm::vec3& ld) const;

    glm::vec3 mSide = glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 mUp = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 mForward = glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 mPosition = glm::vec3(0.0f);
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
    RectangleObstacle(float w, float h, const glm::vec3& s, const glm::vec3& u, const glm::vec3& f,
                      const glm::vec3& p, ObstacleSeenFrom sf)
        : PlaneObstacle(s, u, f, p), width(w), height(h)
    {
        setSeenFrom(sf);
    }

    bool xyPointInsideShape(const glm::vec3& point, float radius) const override;
};

// A box-shaped (cuboid) obstacle, centered on and aligned with a local space.
class BoxObstacle final : public PlaneObstacle
{
public:
    float width = 1.0f;  // extent along local X (side)
    float height = 1.0f; // extent along local Y (up)
    float depth = 1.0f;  // extent along local Z (forward)

    BoxObstacle() = default;
    BoxObstacle(float w, float h, float d, const glm::vec3& s, const glm::vec3& u,
                const glm::vec3& f, const glm::vec3& p)
        : PlaneObstacle(s, u, f, p), width(w), height(h), depth(d)
    {
    }

    void findIntersectionWithVehiclePath(const Radion::Agent& vehicle,
                                         PathIntersection& pi) const override;
};

} // namespace Radion::AI

#endif // RADION_AI_OBSTACLE_H
