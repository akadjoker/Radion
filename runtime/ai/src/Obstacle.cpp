// Obstacle.cpp - obstacle avoidance math.
//
// The vehicles are the Radion Entity; all math uses glm.

#include "PCH.h"

#include "Obstacle.h"

#include "AIInternal.h"

namespace Radion::AI
{

using detail::perpendicularComponent;
using detail::safeNormalize;

// --- Obstacle ---------------------------------------------------------------

Math::Vec3 Obstacle::steerToAvoid(const Entity& vehicle, float minTimeToCollision) const
{
    // Find the nearest intersection with this obstacle along the vehicle's path.
    PathIntersection pi;
    findIntersectionWithVehiclePath(vehicle, pi);

    // Return steering to avoid the intersection, or zero if none found.
    return pi.steerToAvoidIfNeeded(vehicle, minTimeToCollision);
}

Math::Vec3 Obstacle::steerToAvoidObstacles(const Entity& vehicle, float minTimeToCollision,
                                          const ObstacleGroup& obstacles)
{
    PathIntersection nearest, next;

    // Test all obstacles for an intersection with the vehicle's future path,
    // selecting the one whose intersection point is nearest.
    firstPathIntersectionWithObstacleGroup(vehicle, obstacles, nearest, next);

    // If a nearby intersection was found, steer away from it.
    return nearest.steerToAvoidIfNeeded(vehicle, minTimeToCollision);
}

void Obstacle::firstPathIntersectionWithObstacleGroup(const Entity& vehicle,
                                                      const ObstacleGroup& obstacles,
                                                      PathIntersection& nearest,
                                                      PathIntersection& next)
{
    next.intersect = false;
    nearest.intersect = false;
    for (Obstacle* obstacle : obstacles)
    {
        // Find the nearest point (if any) where the vehicle path intersects
        // this obstacle, storing the result in `next`.
        obstacle->findIntersectionWithVehiclePath(vehicle, next);

        // If this is the first intersection found, or it is the nearest found
        // so far, store it in `nearest`.
        const bool firstFound = !nearest.intersect;
        const bool nearestFound = next.intersect && (next.distance < nearest.distance);
        if (firstFound || nearestFound)
            nearest = next;
    }
}

Math::Vec3 PathIntersection::steerToAvoidIfNeeded(const Entity& vehicle,
                                                 float minTimeToCollision) const
{
    // If a nearby intersection was found, steer away from it, otherwise no
    // steering.
    const float minDistanceToCollision = minTimeToCollision * vehicle.speed();
    if (intersect && (distance < minDistanceToCollision))
    {
        // Compute the avoidance force: take the component of steerHint that is
        // lateral (perpendicular to the vehicle's forward), set its length to
        // the vehicle's maxForce.
        Math::Vec3 lateral = perpendicularComponent(steerHint, vehicle.forward());
        return safeNormalize(lateral) * vehicle.maxForce();
    }
    else
    {
        return Math::Vec3(0.0f);
    }
}

// --- SphereObstacle ---------------------------------------------------------

void SphereObstacle::findIntersectionWithVehiclePath(const Entity& vehicle,
                                                     PathIntersection& pi) const
{
    // This routine is based on Paul Bourke's "Intersection of a Line and a
    // Sphere", but computed in the vehicle's local space, where the line in
    // question is the Z (forward) axis - which simplifies the math.
    float b, c, d, p, q, s;
    Math::Vec3 lc;

    // Initialize to "no intersection found".
    pi.intersect = false;

    // Find the sphere's "local center" (lc) in the vehicle's coordinate space.
    lc = vehicle.localizePosition(center);
    pi.vehicleOutside = glm::length(lc) > radius;

    // If the obstacle is seen from inside, but the vehicle is outside, it must
    // be avoided (a vehicle outside would otherwise ignore it).
    if (pi.vehicleOutside && (seenFrom() == ObstacleSeenFrom::Inside))
    {
        pi.intersect = true;
        pi.distance = 0.0f;
        pi.steerHint = safeNormalize(center - vehicle.position());
        return;
    }

    // Compute line-sphere intersection parameters.
    const float r = radius + vehicle.radius();
    b = -2.0f * lc.z;
    c = (lc.x * lc.x) + (lc.y * lc.y) + (lc.z * lc.z) - (r * r);
    d = (b * b) - (4.0f * c);

    // When the path does not intersect the sphere.
    if (d < 0.0f)
        return;

    // Otherwise the path intersects the sphere in two points with parametric
    // coordinates p and q (if d == 0 the path is tangent).
    s = std::sqrt(d);
    p = (-b + s) / 2.0f;
    q = (-b - s) / 2.0f;

    // Both intersections are behind us, so no potential collision.
    if ((p < 0.0f) && (q < 0.0f))
        return;

    // At least one intersection is in front, so the path intersects.
    pi.intersect = true;
    pi.obstacle = this;
    pi.distance = ((p > 0.0f) && (q > 0.0f))
                      ? ((p < q) ? p : q) // both in front, take the nearer one
                      : (seenFrom() == ObstacleSeenFrom::Outside
                             ? 0.0f                   // inside a solid obstacle, distance is zero
                             : ((p > 0.0f) ? p : q)); // hollow obstacle, take the front point
    pi.surfacePoint = vehicle.position() + (vehicle.forward() * pi.distance);
    pi.surfaceNormal = safeNormalize(pi.surfacePoint - center);
    switch (seenFrom())
    {
    case ObstacleSeenFrom::Outside:
        pi.steerHint = pi.surfaceNormal;
        break;
    case ObstacleSeenFrom::Inside:
        pi.steerHint = -pi.surfaceNormal;
        break;
    case ObstacleSeenFrom::Both:
        pi.steerHint = pi.surfaceNormal * (pi.vehicleOutside ? 1.0f : -1.0f);
        break;
    }
}

// --- PlaneObstacle ----------------------------------------------------------

void PlaneObstacle::findIntersectionWithVehiclePath(const Entity& vehicle,
                                                    PathIntersection& pi) const
{
    // Initialize to "no intersection found".
    pi.intersect = false;

    const Math::Vec3 lp = localizePosition(vehicle.position());
    const Math::Vec3 ld = localizeDirection(vehicle.forward());

    // No obstacle intersection if the path is parallel to the XY (side/up) plane.
    if (ld.z == 0.0f)
        return;

    // No obstacle intersection if the vehicle is heading away from the XY plane.
    if ((lp.z > 0.0f) && (ld.z > 0.0f))
        return;
    if ((lp.z < 0.0f) && (ld.z < 0.0f))
        return;

    // No obstacle intersection if the obstacle is "not seen" from this side.
    if ((seenFrom() == ObstacleSeenFrom::Outside) && (lp.z < 0.0f))
        return;
    if ((seenFrom() == ObstacleSeenFrom::Inside) && (lp.z > 0.0f))
        return;

    // Find the intersection of the path with the rectangle's plane (XY plane).
    const float ix = lp.x - (ld.x * lp.z / ld.z);
    const float iy = lp.y - (ld.y * lp.z / ld.z);
    const Math::Vec3 planeIntersection(ix, iy, 0.0f);

    // No obstacle intersection if the plane intersection is outside the 2D shape.
    if (!xyPointInsideShape(planeIntersection, vehicle.radius()))
        return;

    // Otherwise the vehicle path DOES intersect this plane.
    const Math::Vec3 localXYradial = safeNormalize(planeIntersection);
    const Math::Vec3 radial = globalizeDirection(localXYradial);
    const float sideSign = (lp.z > 0.0f) ? +1.0f : -1.0f;
    const Math::Vec3 opposingNormal = forward() * sideSign;
    pi.intersect = true;
    pi.obstacle = this;
    pi.distance = glm::length(lp - planeIntersection);
    pi.steerHint = opposingNormal + radial; // should have "toward edge" term?
    pi.surfacePoint = globalizePosition(planeIntersection);
    pi.surfaceNormal = opposingNormal;
    pi.vehicleOutside = lp.z > 0.0f;
}

// --- RectangleObstacle ------------------------------------------------------

bool RectangleObstacle::xyPointInsideShape(const Math::Vec3& point, float radius) const
{
    const float w = radius + (width * 0.5f);
    const float h = radius + (height * 0.5f);
    return !((point.x > w) || (point.x < -w) || (point.y > h) || (point.y < -h));
}

// --- BoxObstacle ------------------------------------------------------------

void BoxObstacle::findIntersectionWithVehiclePath(const Entity& vehicle, PathIntersection& pi) const
{
    // Abbreviations.
    const Math::Vec3 s = side(); // local axes
    const Math::Vec3 u = up();
    const Math::Vec3 f = forward();
    const Math::Vec3 p = position();
    const Math::Vec3 hw = s * (0.5f * width); // offsets for face centers
    const Math::Vec3 hh = u * (0.5f * height);
    const Math::Vec3 hd = f * (0.5f * depth);
    const ObstacleSeenFrom sf = seenFrom();

    // The box's six rectangular faces.
    RectangleObstacle r1(width, height, s, u, f, p + hd, sf);   // front
    RectangleObstacle r2(width, height, -s, u, -f, p - hd, sf); // back
    RectangleObstacle r3(depth, height, -f, u, s, p + hw, sf);  // side
    RectangleObstacle r4(depth, height, f, u, -s, p - hw, sf);  // other side
    RectangleObstacle r5(width, depth, s, -f, u, p + hh, sf);   // top
    RectangleObstacle r6(width, depth, -s, -f, -u, p - hh, sf); // bottom

    // Group the six faces together.
    ObstacleGroup faces;
    faces.push_back(&r1);
    faces.push_back(&r2);
    faces.push_back(&r3);
    faces.push_back(&r4);
    faces.push_back(&r5);
    faces.push_back(&r6);

    // Find the first intersection of the vehicle path with the group of faces.
    PathIntersection next;
    firstPathIntersectionWithObstacleGroup(vehicle, faces, pi, next);

    // When an intersection was found, adjust it for the box case.
    if (pi.intersect)
    {
        pi.obstacle = this;
        pi.steerHint =
            safeNormalize(pi.surfacePoint - position()) * (pi.vehicleOutside ? 1.0f : -1.0f);
    }
}

// --- PlaneObstacle local-space transforms -----------------------------------

Math::Vec3 PlaneObstacle::localizeDirection(const Math::Vec3& gd) const
{
    const Math::Mat3 basis(mSide, mUp, mForward);
    return glm::transpose(basis) * gd;
}

Math::Vec3 PlaneObstacle::localizePosition(const Math::Vec3& gp) const
{
    const Math::Mat3 basis(mSide, mUp, mForward);
    return glm::transpose(basis) * (gp - mPosition);
}

Math::Vec3 PlaneObstacle::globalizePosition(const Math::Vec3& lp) const
{
    const Math::Mat3 basis(mSide, mUp, mForward);
    return mPosition + (basis * lp);
}

Math::Vec3 PlaneObstacle::globalizeDirection(const Math::Vec3& ld) const
{
    const Math::Mat3 basis(mSide, mUp, mForward);
    return basis * ld;
}

} // namespace Radion::AI
