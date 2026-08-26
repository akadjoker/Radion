#include "PCH.h"

#include "Math.h"

namespace Radion
{

bool AABB::empty() const
{
    return min.x > max.x || min.y > max.y || min.z > max.z;
}

Math::Vec3 AABB::center() const
{
    return (min + max) * 0.5f;
}

Math::Vec3 AABB::extents() const
{
    return (max - min) * 0.5f;
}

float AABB::radius() const
{
    return extents().Length();
}

void AABB::expand(const Math::Vec3& point)
{
    min = Math::Vec3::Min(min, point);
    max = Math::Vec3::Max(max, point);
}

void AABB::merge(const AABB& other)
{
    if (other.empty())
        return;
    min = Math::Vec3::Min(min, other.min);
    max = Math::Vec3::Max(max, other.max);
}

bool AABB::contains(const Math::Vec3& point) const
{
    return point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y &&
           point.z >= min.z && point.z <= max.z;
}

bool AABB::intersects(const AABB& other) const
{
    return min.x <= other.max.x && max.x >= other.min.x && min.y <= other.max.y &&
           max.y >= other.min.y && min.z <= other.max.z && max.z >= other.min.z;
}

AABB transformAABB(const AABB& box, const Math::Mat4& matrix)
{
    if (box.empty())
        return box;

    const Math::Vec3 center = box.center();
    const Math::Vec3 extents = box.extents();

    const Math::Vec3 newCenter = (matrix * Math::Vec4(center, 1.0f)).xyz();
    Math::Vec3 newExtents;
    for (int axis = 0; axis < 3; ++axis)
    {
        newExtents[axis] = std::abs(matrix[0][axis]) * extents.x +
                           std::abs(matrix[1][axis]) * extents.y +
                           std::abs(matrix[2][axis]) * extents.z;
    }

    AABB result;
    result.min = newCenter - newExtents;
    result.max = newCenter + newExtents;
    return result;
}

// ------------------------------------------------------------------- sphere

bool Sphere::contains(const Math::Vec3& point) const
{
    const Math::Vec3 delta = point - center;
    return Math::Vec3::Dot(delta, delta) <= radius * radius;
}

bool Sphere::intersects(const Sphere& other) const
{
    const Math::Vec3 delta = other.center - center;
    const float reach = radius + other.radius;
    return Math::Vec3::Dot(delta, delta) <= reach * reach;
}

bool Sphere::intersects(const AABB& box) const
{
    if (box.empty())
        return false;

    const Math::Vec3 closest = Math::Vec3::Clamp(center, box.min, box.max);
    const Math::Vec3 delta = center - closest;
    return Math::Vec3::Dot(delta, delta) <= radius * radius;
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

float Plane::distance(const Math::Vec3& point) const
{
    return Math::Vec3::Dot(normal, point) + d;
}

void Plane::normalize()
{
    const float length = normal.Length();
    if (length <= 0.0f)
        return;

    const float inverse = 1.0f / length;
    normal *= inverse;
    d *= inverse;
}

// ---------------------------------------------------------------------- ray

Math::Vec3 Ray::at(float t) const
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
    const Math::Vec3 delta = origin - sphere.center;
    const float b = Math::Vec3::Dot(delta, direction);
    const float c = Math::Vec3::Dot(delta, delta) - sphere.radius * sphere.radius;

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
    const float denominator = Math::Vec3::Dot(plane.normal, direction);
    if (std::abs(denominator) < 1e-6f)
        return false;

    const float hit = -(Math::Vec3::Dot(plane.normal, origin) + plane.d) / denominator;
    if (hit < 0.0f)
        return false;

    t = hit;
    return true;
}

bool Ray::intersects(const Math::Vec3& v0, const Math::Vec3& v1, const Math::Vec3& v2, float& t) const
{
    const Math::Vec3 edge1 = v1 - v0;
    const Math::Vec3 edge2 = v2 - v0;

    const Math::Vec3 h = Math::Vec3::Cross(direction, edge2);
    const float a = Math::Vec3::Dot(edge1, h);
    if (std::abs(a) < 1e-8f)
        return false;

    const float f = 1.0f / a;
    const Math::Vec3 s = origin - v0;
    const float u = f * Math::Vec3::Dot(s, h);
    if (u < 0.0f || u > 1.0f)
        return false;

    const Math::Vec3 q = Math::Vec3::Cross(s, edge1);
    const float v = f * Math::Vec3::Dot(direction, q);
    if (v < 0.0f || u + v > 1.0f)
        return false;

    const float hit = f * Math::Vec3::Dot(edge2, q);
    if (hit < 0.0f)
        return false;

    t = hit;
    return true;
}

Ray rayFromScreen(float screenX, float screenY, float viewportWidth, float viewportHeight,
                  const Math::Mat4& inverseViewProjection)
{
    Ray ray;
    if (viewportWidth <= 0.0f || viewportHeight <= 0.0f)
        return ray;

    const float ndcX = (screenX / viewportWidth) * 2.0f - 1.0f;
    const float ndcY = 1.0f - (screenY / viewportHeight) * 2.0f;

    Math::Vec4 nearPoint = inverseViewProjection * Math::Vec4(ndcX, ndcY, -1.0f, 1.0f);
    Math::Vec4 farPoint = inverseViewProjection * Math::Vec4(ndcX, ndcY, 1.0f, 1.0f);

    if (nearPoint.w == 0.0f || farPoint.w == 0.0f)
        return ray;

    nearPoint /= nearPoint.w;
    farPoint /= farPoint.w;

    ray.origin = nearPoint.xyz();
    ray.direction = (farPoint - nearPoint).xyz().Normalized();
    return ray;
}

// ------------------------------------------------------------------ frustum

Frustum::Frustum()
{
}

void Frustum::update(const Math::Mat4& m)
{
    // Row i of the matrix, remembering glm indexes as m[column][row].
    const Math::Vec4 row0(m[0][0], m[1][0], m[2][0], m[3][0]);
    const Math::Vec4 row1(m[0][1], m[1][1], m[2][1], m[3][1]);
    const Math::Vec4 row2(m[0][2], m[1][2], m[2][2], m[3][2]);
    const Math::Vec4 row3(m[0][3], m[1][3], m[2][3], m[3][3]);

    const Math::Vec4 sides[SideCount] = {row3 + row0, row3 - row0, row3 + row1,
                                        row3 - row1, row3 + row2, row3 - row2};

    for (int i = 0; i < SideCount; ++i)
    {
        mPlanes[i].normal = sides[i].xyz();
        mPlanes[i].d = sides[i].w;
        mPlanes[i].normalize();
    }
}

bool Frustum::contains(const Math::Vec3& point) const
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

bool Frustum::intersects(const Math::Vec3& min, const Math::Vec3& max) const
{
    for (int i = 0; i < SideCount; ++i)
    {
        const Plane& plane = mPlanes[i];

        // Projected radius of the box onto the plane normal.
        const float reach = (max.x - min.x) * 0.5f * std::abs(plane.normal.x) +
                            (max.y - min.y) * 0.5f * std::abs(plane.normal.y) +
                            (max.z - min.z) * 0.5f * std::abs(plane.normal.z);
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

    const Math::Vec3 center = box.center();
    const Math::Vec3 extents = box.extents();
    bool intersecting = false;

    for (int i = 0; i < SideCount; ++i)
    {
        const Plane& plane = mPlanes[i];

        // Projected radius of the box onto the plane normal.
        const float reach = extents.x * std::abs(plane.normal.x) +
                            extents.y * std::abs(plane.normal.y) +
                            extents.z * std::abs(plane.normal.z);
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
