#include "PCH.h"

#include "Math.h"

namespace Radion
{

bool AABB::empty() const
{
    return min.x > max.x || min.y > max.y || min.z > max.z;
}

glm::vec3 AABB::center() const
{
    return (min + max) * 0.5f;
}

glm::vec3 AABB::extents() const
{
    return (max - min) * 0.5f;
}

float AABB::radius() const
{
    return glm::length(extents());
}

void AABB::expand(const glm::vec3& point)
{
    min = glm::min(min, point);
    max = glm::max(max, point);
}

void AABB::merge(const AABB& other)
{
    if (other.empty())
        return;
    min = glm::min(min, other.min);
    max = glm::max(max, other.max);
}

bool AABB::contains(const glm::vec3& point) const
{
    return point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y &&
           point.z >= min.z && point.z <= max.z;
}

bool AABB::intersects(const AABB& other) const
{
    return min.x <= other.max.x && max.x >= other.min.x && min.y <= other.max.y &&
           max.y >= other.min.y && min.z <= other.max.z && max.z >= other.min.z;
}

AABB transformAABB(const AABB& box, const glm::mat4& matrix)
{
    if (box.empty())
        return box;

    const glm::vec3 center = box.center();
    const glm::vec3 extents = box.extents();

    const glm::vec3 newCenter = glm::vec3(matrix * glm::vec4(center, 1.0f));
    glm::vec3 newExtents;
    for (int axis = 0; axis < 3; ++axis)
    {
        newExtents[axis] = glm::abs(matrix[0][axis]) * extents.x +
                           glm::abs(matrix[1][axis]) * extents.y +
                           glm::abs(matrix[2][axis]) * extents.z;
    }

    AABB result;
    result.min = newCenter - newExtents;
    result.max = newCenter + newExtents;
    return result;
}

// ------------------------------------------------------------------- sphere

bool Sphere::contains(const glm::vec3& point) const
{
    const glm::vec3 delta = point - center;
    return glm::dot(delta, delta) <= radius * radius;
}

bool Sphere::intersects(const Sphere& other) const
{
    const glm::vec3 delta = other.center - center;
    const float reach = radius + other.radius;
    return glm::dot(delta, delta) <= reach * reach;
}

bool Sphere::intersects(const AABB& box) const
{
    if (box.empty())
        return false;

    const glm::vec3 closest = glm::clamp(center, box.min, box.max);
    const glm::vec3 delta = center - closest;
    return glm::dot(delta, delta) <= radius * radius;
}

Sphere sphereOfAABB(const AABB& box)
{
    Sphere sphere;
    if (box.empty())
        return sphere;

    sphere.center = box.center();
    sphere.radius = box.radius();
    return sphere;
}

// -------------------------------------------------------------------- plane

float Plane::distance(const glm::vec3& point) const
{
    return glm::dot(normal, point) + d;
}

void Plane::normalize()
{
    const float length = glm::length(normal);
    if (length <= 0.0f)
        return;

    const float inverse = 1.0f / length;
    normal *= inverse;
    d *= inverse;
}

// ---------------------------------------------------------------------- ray

glm::vec3 Ray::at(float t) const
{
    return origin + direction * t;
}

bool Ray::intersects(const AABB& box, float& t) const
{
    if (box.empty())
        return false;

    // Slab method. A zero component yields an infinity that the min/max still
    // orders correctly, so axes parallel to the ray need no special case.
    float near = -3.402823466e+38F;
    float far = 3.402823466e+38F;

    for (int axis = 0; axis < 3; ++axis)
    {
        const float inverse = 1.0f / direction[axis];
        float t0 = (box.min[axis] - origin[axis]) * inverse;
        float t1 = (box.max[axis] - origin[axis]) * inverse;
        if (t0 > t1)
        {
            const float swap = t0;
            t0 = t1;
            t1 = swap;
        }

        near = t0 > near ? t0 : near;
        far = t1 < far ? t1 : far;
        if (near > far)
            return false;
    }

    if (far < 0.0f)
        return false;

    t = near >= 0.0f ? near : far;
    return true;
}

bool Ray::intersects(const Sphere& sphere, float& t) const
{
    const glm::vec3 delta = origin - sphere.center;
    const float b = glm::dot(delta, direction);
    const float c = glm::dot(delta, delta) - sphere.radius * sphere.radius;

    const float discriminant = b * b - c;
    if (discriminant < 0.0f)
        return false;

    const float root = std::sqrt(discriminant);
    float hit = -b - root;
    if (hit < 0.0f)
        hit = -b + root;
    if (hit < 0.0f)
        return false;

    t = hit;
    return true;
}

bool Ray::intersects(const Plane& plane, float& t) const
{
    const float denominator = glm::dot(plane.normal, direction);
    if (glm::abs(denominator) < 1e-6f)
        return false;

    const float hit = -(glm::dot(plane.normal, origin) + plane.d) / denominator;
    if (hit < 0.0f)
        return false;

    t = hit;
    return true;
}

bool Ray::intersects(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, float& t) const
{
    const glm::vec3 edge1 = v1 - v0;
    const glm::vec3 edge2 = v2 - v0;

    const glm::vec3 h = glm::cross(direction, edge2);
    const float a = glm::dot(edge1, h);
    if (glm::abs(a) < 1e-8f)
        return false;

    const float f = 1.0f / a;
    const glm::vec3 s = origin - v0;
    const float u = f * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f)
        return false;

    const glm::vec3 q = glm::cross(s, edge1);
    const float v = f * glm::dot(direction, q);
    if (v < 0.0f || u + v > 1.0f)
        return false;

    const float hit = f * glm::dot(edge2, q);
    if (hit < 0.0f)
        return false;

    t = hit;
    return true;
}

Ray rayFromScreen(float screenX, float screenY, float viewportWidth, float viewportHeight,
                  const glm::mat4& inverseViewProjection)
{
    Ray ray;
    if (viewportWidth <= 0.0f || viewportHeight <= 0.0f)
        return ray;

    const float ndcX = (screenX / viewportWidth) * 2.0f - 1.0f;
    const float ndcY = 1.0f - (screenY / viewportHeight) * 2.0f;

    glm::vec4 nearPoint = inverseViewProjection * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
    glm::vec4 farPoint = inverseViewProjection * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);

    if (nearPoint.w == 0.0f || farPoint.w == 0.0f)
        return ray;

    nearPoint /= nearPoint.w;
    farPoint /= farPoint.w;

    ray.origin = glm::vec3(nearPoint);
    ray.direction = glm::normalize(glm::vec3(farPoint - nearPoint));
    return ray;
}

// ------------------------------------------------------------------ frustum

Frustum::Frustum()
{
}

void Frustum::update(const glm::mat4& m)
{
    // Row i of the matrix, remembering glm indexes as m[column][row].
    const glm::vec4 row0(m[0][0], m[1][0], m[2][0], m[3][0]);
    const glm::vec4 row1(m[0][1], m[1][1], m[2][1], m[3][1]);
    const glm::vec4 row2(m[0][2], m[1][2], m[2][2], m[3][2]);
    const glm::vec4 row3(m[0][3], m[1][3], m[2][3], m[3][3]);

    const glm::vec4 sides[SideCount] = {row3 + row0, row3 - row0, row3 + row1,
                                        row3 - row1, row3 + row2, row3 - row2};

    for (int i = 0; i < SideCount; ++i)
    {
        mPlanes[i].normal = glm::vec3(sides[i]);
        mPlanes[i].d = sides[i].w;
        mPlanes[i].normalize();
    }
}

bool Frustum::contains(const glm::vec3& point) const
{
    for (int i = 0; i < SideCount; ++i)
    {
        if (mPlanes[i].distance(point) < 0.0f)
            return false;
    }
    return true;
}

bool Frustum::intersects(const Sphere& sphere) const
{
    for (int i = 0; i < SideCount; ++i)
    {
        if (mPlanes[i].distance(sphere.center) < -sphere.radius)
            return false;
    }
    return true;
}

bool Frustum::intersects(const AABB& box) const
{
    return classify(box) != Containment::Outside;
}

bool Frustum::intersects(const glm::vec3& min, const glm::vec3& max) const
{
    for (int i = 0; i < SideCount; ++i)
    {
        const Plane& plane = mPlanes[i];

        // Projected radius of the box onto the plane normal.
        const float reach = (max.x - min.x) * 0.5f * glm::abs(plane.normal.x) +
                            (max.y - min.y) * 0.5f * glm::abs(plane.normal.y) +
                            (max.z - min.z) * 0.5f * glm::abs(plane.normal.z);
        const float distance = plane.distance((min + max) * 0.5f);

        if (distance < -reach)
            return false;
    }
    return true;
}

Containment Frustum::classify(const AABB& box) const
{
    if (box.empty())
        return Containment::Outside;

    const glm::vec3 center = box.center();
    const glm::vec3 extents = box.extents();
    bool intersecting = false;

    for (int i = 0; i < SideCount; ++i)
    {
        const Plane& plane = mPlanes[i];

        // Projected radius of the box onto the plane normal.
        const float reach = extents.x * glm::abs(plane.normal.x) +
                            extents.y * glm::abs(plane.normal.y) +
                            extents.z * glm::abs(plane.normal.z);
        const float distance = plane.distance(center);

        if (distance < -reach)
            return Containment::Outside;
        if (distance < reach)
            intersecting = true;
    }

    return intersecting ? Containment::Intersects : Containment::Inside;
}

const Plane& Frustum::plane(Side side) const
{
    return mPlanes[side];
}

} // namespace Radion
