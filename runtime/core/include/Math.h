#ifndef RADION_MATH_H
#define RADION_MATH_H

#include <cmath>
#include "MathCompat.h"

namespace Radion
{

const unsigned int MaxUInt32 = 0xFFFFFFFF;
const int MinInt32 = 0x80000000;
const int MaxInt32 = 0x7FFFFFFF;
const float MaxFloat = 3.402823466e+38F;
const float MinPosFloat = 1.175494351e-38F;

const float Pi = 3.141592654f;
const float TwoPi = 6.283185307f;
const float PiHalf = 1.570796327f;

const float Epsilon = 0.000001f;
const float ZeroEpsilon = 32.0f * MinPosFloat; // Very small epsilon for checking against 0.0f

const float M_INFINITY = 1.0e30f;

#define powi(base, exp) (int)powf((float)(base), (float)(exp))

#define ToRadians(x) (float)(((x) * Pi / 180.0f))
#define ToDegrees(x) (float)(((x) * 180.0f / Pi))

inline float Sin(float a)
{
    return sin(a * Pi / 180);
}
inline float Cos(float a)
{
    return cos(a * Pi / 180);
}
inline float Tan(float a)
{
    return tan(a * Pi / 180);
}
inline float SinRad(float a)
{
    return sin(a);
}
inline float CosRad(float a)
{
    return cos(a);
}
inline float TanRad(float a)
{
    return tan(a);
}
inline float ASin(float a)
{
    return asin(a) * 180 / Pi;
}
inline float ACos(float a)
{
    return acos(a) * 180 / Pi;
}
inline float ATan(float a)
{
    return atan(a) * 180 / Pi;
}
inline float ATan2(float y, float x)
{
    return atan2(y, x) * 180 / Pi;
}
inline float ASinRad(float a)
{
    return asin(a);
}
inline float ACosRad(float a)
{
    return acos(a);
}
inline float ATanRad(float a)
{
    return atan(a);
}
inline float ATan2Rad(float y, float x)
{
    return atan2(y, x);
}
inline int Floor(float a)
{
    return (int)(floor(a));
}
inline int Ceil(float a)
{
    return (int)(ceil(a));
}
inline int Trunc(float a)
{
    if (a > 0)
        return Floor(a);
    else
        return Ceil(a);
}
inline int Round(float a)
{
    if (a < 0)
        return (int)(ceil(a - 0.5f));
    else
        return (int)(floor(a + 0.5f));
}
inline float Sqrt(float a)
{
    if (a > 0)
        return sqrt(a);
    else
        return 0;
}
inline float Abs(float a)
{
    if (a < 0)
        a = -a;
    return a;
}
inline int Mod(int a, int b)
{
    if (b == 0)
        return 0;
    return a % b;
}
inline float FMod(float a, float b)
{
    if (b == 0)
        return 0;
    return fmod(a, b);
}
inline float Pow(float a, float b)
{
    return pow(a, b);
}
inline int Sign(float a)
{
    if (a < 0)
        return -1;
    else if (a > 0)
        return 1;
    else
        return 0;
}
inline float Min(float a, float b)
{
    return a < b ? a : b;
}
inline float Max(float a, float b)
{
    return a > b ? a : b;
}
inline int Min(int a, int b)
{
    return a < b ? a : b;
}
inline int Max(int a, int b)
{
    return a > b ? a : b;
}
inline float Clamp(float a, float min, float max)
{
    if (a < min)
        a = min;
    else if (a > max)
        a = max;
    return a;
}
inline int Clamp(int a, int min, int max)
{
    if (a < min)
        a = min;
    else if (a > max)
        a = max;
    return a;
}
static inline float Clamp1(float x)
{
    return x < -1.f ? -1.f : (x > 1.f ? 1.f : x);
}

inline float DegToRad(float f)
{
    return f * 0.017453293f;
}

inline float RadToDeg(float f)
{
    return f * 57.29577951f;
}

inline float Lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

inline Math::vec3 Lerp(const Math::vec3& a, const Math::vec3& b, float t)
{
    return a + (b - a) * t;
}

template <typename T> struct Rectangle
{

    T x;
    T y;
    T width;
    T height;

    Rectangle() : x(0), y(0), width(0), height(0)
    {
    }
    Rectangle(T x, T y, T width, T height) : x(x), y(y), width(width), height(height)
    {
    }
    Rectangle(const Rectangle& rect) : x(rect.x), y(rect.y), width(rect.width), height(rect.height)
    {
    }

    void set(T x, T y, T width, T height)
    {
        this->x = x;
        this->y = y;
        this->width = width;
        this->height = height;
    }

    void merge(const Rectangle& rect)
    {
        T right = x + width;
        T bottom = y + height;
        T rectRight = rect.x + rect.width;
        T rectBottom = rect.y + rect.height;
        x = Min(x, rect.x);
        y = Min(y, rect.y);
        right = Max(right, rectRight);
        bottom = Max(bottom, rectBottom);
        width = right - x;
        height = bottom - y;
    }

    template <typename Point> void merge(const Point& point)
    {
        T right = x + width;
        T bottom = y + height;
        x = Min(x, point.x);
        y = Min(y, point.y);
        right = Max(right, point.x);
        bottom = Max(bottom, point.y);
        width = right - x;
        height = bottom - y;
    }

    void clear()
    {
        x = 0;
        y = 0;
        width = 0;
        height = 0;
    }

    Rectangle& operator=(const Rectangle& rect)
    {
        if (this == &rect)
            return *this;
        x = rect.x;
        y = rect.y;
        width = rect.width;
        height = rect.height;
        return *this;
    }
};

template <typename T> struct Size
{
    T width;
    T height;

    Size() : width(0), height(0)
    {
    }
    Size(T w, T h) : width(w), height(h)
    {
    }
    Size(const Size& size) : width(size.width), height(size.height)
    {
    }

    Size& operator=(const Size& size)
    {
        if (this == &size)
            return *this;
        width = size.width;
        height = size.height;
        return *this;
    }
};

typedef Rectangle<int> IntRect;
typedef Rectangle<float> FloatRect;
typedef Size<int> IntSize;
typedef Size<float> FloatSize;

// Empty is min > max, so a fresh box absorbs the first point correctly and
// merging an empty box with anything is a no-op.
struct AABB
{
    Math::vec3 min = Math::vec3(3.402823466e+38F);
    Math::vec3 max = Math::vec3(-3.402823466e+38F);

    bool empty() const;
    Math::vec3 center() const;
    Math::vec3 extents() const;
    float radius() const;

    void expand(const Math::vec3& point);
    void merge(const AABB& other);

    bool contains(const Math::vec3& point) const;
    bool intersects(const AABB& other) const;
};

// The transformed box is the box of the transformed box, not of the transformed
// corners one by one: each axis takes the sum of the absolute contributions.
AABB transformAABB(const AABB& box, const Math::mat4& matrix);

struct Sphere
{
    Math::vec3 center = Math::vec3(0.0f);
    float radius = 0.0f;

    bool contains(const Math::vec3& point) const;
    bool intersects(const Sphere& other) const;
    bool intersects(const AABB& box) const;
};

Sphere sphereOfAABB(const AABB& box);

// Points are inside when dot(normal, point) + d >= 0.
struct Plane
{
    Math::vec3 normal = Math::vec3(0.0f, 1.0f, 0.0f);
    float d = 0.0f;

    float distance(const Math::vec3& point) const;
    void normalize();
};

struct Ray
{
    Math::vec3 origin = Math::vec3(0.0f);
    Math::vec3 direction = Math::vec3(0.0f, 0.0f, -1.0f);

    Math::vec3 at(float t) const;

    // Every hit test writes the distance along the ray into t and ignores
    // intersections behind the origin.
    bool intersects(const AABB& box, float& t) const;
    bool intersects(const Sphere& sphere, float& t) const;
    bool intersects(const Plane& plane, float& t) const;
    bool intersects(const Math::vec3& v0, const Math::vec3& v1, const Math::vec3& v2, float& t) const;
};

// Builds a ray from a pixel. Pass viewProjection already combined.
Ray rayFromScreen(float screenX, float screenY, float viewportWidth, float viewportHeight,
                  const Math::mat4& inverseViewProjection);

enum class Containment
{
    Outside,
    Intersects,
    Inside
};

class Frustum
{
public:
    enum Side
    {
        SideLeft = 0,
        SideRight,
        SideBottom,
        SideTop,
        SideNear,
        SideFar,

        SideCount
    };

    Frustum();

    // Planes come straight out of the combined view-projection matrix, so any
    // projection works without knowing how it was built.
    void update(const Math::mat4& viewProjection);

    bool contains(const Math::vec3& point) const;
    bool intersects(const Sphere& sphere) const;
    bool intersects(const AABB& box) const;
    bool intersects(const Math::vec3& min, const Math::vec3& max) const;

    // Tells fully-inside from partly-inside, so a tree node that is entirely
    // inside can stop testing its children.
    Containment classify(const AABB& box) const;

    const Plane& plane(Side side) const;

private:
    Plane mPlanes[SideCount];
};

} // namespace Radion

#endif // RADION_MATH_H
