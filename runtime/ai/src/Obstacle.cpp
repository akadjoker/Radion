#include "PCH.h"

#include "Obstacle.h"

#include "AIInternal.h"
#include "Agent.h"

namespace Radion::AI
{

using detail::perpendicularComponent;
using detail::safeNormalize;
using Radion::Agent;

Math::vec3 Obstacle::steerToAvoid(const Agent& vehicle, float minTimeToCollision) const
{
    PathIntersection pi;
    findIntersectionWithVehiclePath(vehicle, pi);

    return pi.steerToAvoidIfNeeded(vehicle, minTimeToCollision);
}

Math::vec3 Obstacle::steerToAvoidObstacles(const Agent& vehicle, float minTimeToCollision,
                                          const ObstacleGroup& obstacles)
{
    PathIntersection nearest, next;

    firstPathIntersectionWithObstacleGroup(vehicle, obstacles, nearest, next);

    return nearest.steerToAvoidIfNeeded(vehicle, minTimeToCollision);
}

void Obstacle::firstPathIntersectionWithObstacleGroup(const Agent& vehicle,
                                                      const ObstacleGroup& obstacles,
                                                      PathIntersection& nearest,
                                                      PathIntersection& next)
{
    next.intersect = false;
    nearest.intersect = false;
    for (Obstacle* obstacle : obstacles)
    {
        obstacle->findIntersectionWithVehiclePath(vehicle, next);

        const bool firstFound = !nearest.intersect;
        const bool nearestFound = next.intersect && (next.distance < nearest.distance);
        if (firstFound || nearestFound)
            nearest = next;
    }
}

Math::vec3 PathIntersection::steerToAvoidIfNeeded(const Agent& vehicle,
                                                 float minTimeToCollision) const
{
    const float minDistanceToCollision = minTimeToCollision * vehicle.speed();
    if (intersect && (distance < minDistanceToCollision))
    {
        Math::vec3 lateral = perpendicularComponent(steerHint, vehicle.forward());

        // Dead ahead the hint has no lateral component (only the normal, back along the heading), giving zero force;
        // pick the vehicle's own side, deterministic so it does not dither.
        if (Math::dot(lateral, lateral) < 1.0e-8f)
            lateral = vehicle.side();
        return safeNormalize(lateral) * vehicle.maxForce();
    }
    else
    {
        return Math::vec3(0.0f);
    }
}

void SphereObstacle::findIntersectionWithVehiclePath(const Agent& vehicle,
                                                     PathIntersection& pi) const
{
    // Based on Paul Bourke's "Intersection of a Line and a Sphere", in the vehicle's local space where the line is the Z axis.
    float b, c, d, p, q, s;
    Math::vec3 lc;

    pi.intersect = false;

    lc = vehicle.localizePosition(center);
    pi.vehicleOutside = Math::length(lc) > radius;

    // Seen from inside but the vehicle is outside: it must still be avoided.
    if (pi.vehicleOutside && (seenFrom() == ObstacleSeenFrom::Inside))
    {
        pi.intersect = true;
        pi.distance = 0.0f;
        pi.steerHint = safeNormalize(center - vehicle.position());
        return;
    }

    const float r = radius + vehicle.radius();
    b = -2.0f * lc.z;
    c = (lc.x * lc.x) + (lc.y * lc.y) + (lc.z * lc.z) - (r * r);
    d = (b * b) - (4.0f * c);

    if (d < 0.0f)
        return;

    // Two intersections at parameters p and q (d == 0: tangent).
    s = std::sqrt(d);
    p = (-b + s) / 2.0f;
    q = (-b - s) / 2.0f;

    if ((p < 0.0f) && (q < 0.0f))
        return;

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

void PlaneObstacle::findIntersectionWithVehiclePath(const Agent& vehicle,
                                                    PathIntersection& pi) const
{
    pi.intersect = false;

    const Math::vec3 lp = localizePosition(vehicle.position());
    const Math::vec3 ld = localizeDirection(vehicle.forward());

    if (ld.z == 0.0f)
        return;

    if ((lp.z > 0.0f) && (ld.z > 0.0f))
        return;
    if ((lp.z < 0.0f) && (ld.z < 0.0f))
        return;

    if ((seenFrom() == ObstacleSeenFrom::Outside) && (lp.z < 0.0f))
        return;
    if ((seenFrom() == ObstacleSeenFrom::Inside) && (lp.z > 0.0f))
        return;

    const float ix = lp.x - (ld.x * lp.z / ld.z);
    const float iy = lp.y - (ld.y * lp.z / ld.z);
    const Math::vec3 planeIntersection(ix, iy, 0.0f);

    if (!xyPointInsideShape(planeIntersection, vehicle.radius()))
        return;

    const Math::vec3 localXYradial = safeNormalize(planeIntersection);
    const Math::vec3 radial = globalizeDirection(localXYradial);
    const float sideSign = (lp.z > 0.0f) ? +1.0f : -1.0f;
    const Math::vec3 opposingNormal = forward() * sideSign;
    pi.intersect = true;
    pi.obstacle = this;
    pi.distance = Math::length(lp - planeIntersection);
    pi.steerHint = opposingNormal + radial;
    pi.surfacePoint = globalizePosition(planeIntersection);
    pi.surfaceNormal = opposingNormal;
    pi.vehicleOutside = lp.z > 0.0f;
}

bool RectangleObstacle::xyPointInsideShape(const Math::vec3& point, float radius) const
{
    const float w = radius + (width * 0.5f);
    const float h = radius + (height * 0.5f);
    return !((point.x > w) || (point.x < -w) || (point.y > h) || (point.y < -h));
}

void BoxObstacle::findIntersectionWithVehiclePath(const Agent& vehicle, PathIntersection& pi) const
{
    const Math::vec3 s = side();
    const Math::vec3 u = up();
    const Math::vec3 f = forward();
    const Math::vec3 p = position();
    const Math::vec3 hw = s * (0.5f * width);
    const Math::vec3 hh = u * (0.5f * height);
    const Math::vec3 hd = f * (0.5f * depth);
    const ObstacleSeenFrom sf = seenFrom();

    RectangleObstacle r1(width, height, s, u, f, p + hd, sf);
    RectangleObstacle r2(width, height, -s, u, -f, p - hd, sf);
    RectangleObstacle r3(depth, height, -f, u, s, p + hw, sf);
    RectangleObstacle r4(depth, height, f, u, -s, p - hw, sf);
    RectangleObstacle r5(width, depth, s, -f, u, p + hh, sf);
    RectangleObstacle r6(width, depth, -s, -f, -u, p - hh, sf);

    ObstacleGroup faces;
    faces.push_back(&r1);
    faces.push_back(&r2);
    faces.push_back(&r3);
    faces.push_back(&r4);
    faces.push_back(&r5);
    faces.push_back(&r6);

    PathIntersection next;
    firstPathIntersectionWithObstacleGroup(vehicle, faces, pi, next);

    if (pi.intersect)
    {
        pi.obstacle = this;
        pi.steerHint =
            safeNormalize(pi.surfacePoint - position()) * (pi.vehicleOutside ? 1.0f : -1.0f);
    }
}

Math::vec3 PlaneObstacle::localizeDirection(const Math::vec3& gd) const
{
    const Math::mat3 basis(mSide, mUp, mForward);
    return Math::transpose(basis) * gd;
}

Math::vec3 PlaneObstacle::localizePosition(const Math::vec3& gp) const
{
    const Math::mat3 basis(mSide, mUp, mForward);
    return Math::transpose(basis) * (gp - mPosition);
}

Math::vec3 PlaneObstacle::globalizePosition(const Math::vec3& lp) const
{
    const Math::mat3 basis(mSide, mUp, mForward);
    return mPosition + (basis * lp);
}

Math::vec3 PlaneObstacle::globalizeDirection(const Math::vec3& ld) const
{
    const Math::mat3 basis(mSide, mUp, mForward);
    return basis * ld;
}

} // namespace Radion::AI
