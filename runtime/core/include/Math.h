#ifndef RADION_MATH_H
#define RADION_MATH_H

#include <mathc.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace Radion
{

// GLM-shaped surface over Mathc types, so call sites read like the math they
// express (vec3, mat4, quat, ...) regardless of the underlying library.
namespace Math
{
using vec2 = ::Mathc::Vec2;
using vec3 = ::Mathc::Vec3;
using vec4 = ::Mathc::Vec4;
using mat2 = ::Mathc::Mat2;
using mat3 = ::Mathc::Mat3;
using mat4 = ::Mathc::Mat4;
struct quat : ::Mathc::Quaternion
{
    quat() : ::Mathc::Quaternion(::Mathc::Quaternion::Identity()) {}
    // GLM orders quaternion constructor arguments as w, x, y, z; Mathc stores
    // and constructs them as x, y, z, w.
    quat(float w, float x, float y, float z) : ::Mathc::Quaternion(x, y, z, w) {}
    quat(float w, const vec3& xyz) : ::Mathc::Quaternion(xyz.x, xyz.y, xyz.z, w) {}
    quat(const vec3& eulerRadians)
        : ::Mathc::Quaternion(::Mathc::Quaternion::FromEulerAngles(eulerRadians.x, eulerRadians.y, eulerRadians.z)) {}
    quat(const ::Mathc::Quaternion& value) : ::Mathc::Quaternion(value) {}
};
using dvec3 = ::Mathc::Vec3;

struct bvec3 { bool x, y, z; };

struct ivec3
{
    int x, y, z;
    constexpr ivec3() : x(0), y(0), z(0) {}
    constexpr explicit ivec3(int v) : x(v), y(v), z(v) {}
    constexpr ivec3(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}
    ivec3(const vec3& v) : x(static_cast<int>(v.x)), y(static_cast<int>(v.y)), z(static_cast<int>(v.z)) {}
    int& operator[](int i) { return (&x)[i]; }
    const int& operator[](int i) const { return (&x)[i]; }
    constexpr ivec3 operator+(const ivec3& r) const { return {x + r.x, y + r.y, z + r.z}; }
    constexpr ivec3 operator-(const ivec3& r) const { return {x - r.x, y - r.y, z - r.z}; }
    constexpr ivec3 operator-() const { return {-x, -y, -z}; }
    constexpr bool operator==(const ivec3& r) const { return x == r.x && y == r.y && z == r.z; }
    constexpr bool operator!=(const ivec3& r) const { return !(*this == r); }
    operator vec3() const { return vec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)); }
};

struct uvec3
{
    std::uint32_t x, y, z;
    constexpr uvec3() : x(0), y(0), z(0) {}
    constexpr explicit uvec3(std::uint32_t v) : x(v), y(v), z(v) {}
    constexpr uvec3(std::uint32_t x_, std::uint32_t y_, std::uint32_t z_) : x(x_), y(y_), z(z_) {}
    uvec3(const vec3& v) : x(static_cast<std::uint32_t>(v.x)), y(static_cast<std::uint32_t>(v.y)), z(static_cast<std::uint32_t>(v.z)) {}
    uvec3(const ivec3& v) : x(static_cast<std::uint32_t>(v.x)), y(static_cast<std::uint32_t>(v.y)), z(static_cast<std::uint32_t>(v.z)) {}
    std::uint32_t& operator[](int i) { return (&x)[i]; }
    const std::uint32_t& operator[](int i) const { return (&x)[i]; }
    constexpr uvec3 operator+(const uvec3& r) const { return {x + r.x, y + r.y, z + r.z}; }
    constexpr uvec3 operator-(const uvec3& r) const { return {x - r.x, y - r.y, z - r.z}; }
    constexpr uvec3 operator/(std::uint32_t v) const { return {x / v, y / v, z / v}; }
    constexpr uvec3 operator%(std::uint32_t v) const { return {x % v, y % v, z % v}; }
    constexpr bool operator==(const uvec3& r) const { return x == r.x && y == r.y && z == r.z; }
    constexpr bool operator!=(const uvec3& r) const { return !(*this == r); }
    operator vec3() const { return vec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)); }
    operator ivec3() const { return ivec3(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)); }
};

struct ivec4
{
    int x, y, z, w;
    constexpr ivec4() : x(0), y(0), z(0), w(0) {}
    constexpr explicit ivec4(int v) : x(v), y(v), z(v), w(v) {}
    constexpr ivec4(int x_, int y_, int z_, int w_) : x(x_), y(y_), z(z_), w(w_) {}
    int& operator[](int i) { return (&x)[i]; }
    const int& operator[](int i) const { return (&x)[i]; }
    constexpr bool operator==(const ivec4& r) const { return x == r.x && y == r.y && z == r.z && w == r.w; }
};

struct uvec4
{
    std::uint32_t x, y, z, w;
    constexpr uvec4() : x(0), y(0), z(0), w(0) {}
    constexpr explicit uvec4(std::uint32_t v) : x(v), y(v), z(v), w(v) {}
    constexpr uvec4(std::uint32_t x_, std::uint32_t y_, std::uint32_t z_, std::uint32_t w_) : x(x_), y(y_), z(z_), w(w_) {}
    std::uint32_t& operator[](int i) { return (&x)[i]; }
    const std::uint32_t& operator[](int i) const { return (&x)[i]; }
    constexpr bool operator==(const uvec4& r) const { return x == r.x && y == r.y && z == r.z && w == r.w; }
};

inline vec2 operator+(const vec2& v, float s) { return v + vec2(s); }
inline vec2 operator+(float s, const vec2& v) { return vec2(s) + v; }
inline vec2 operator-(const vec2& v, float s) { return v - vec2(s); }
inline vec2 operator-(float s, const vec2& v) { return vec2(s) - v; }
inline vec3 operator+(const vec3& v, float s) { return v + vec3(s); }
inline vec3 operator+(float s, const vec3& v) { return vec3(s) + v; }
inline vec3 operator-(const vec3& v, float s) { return v - vec3(s); }
inline vec3 operator-(float s, const vec3& v) { return vec3(s) - v; }
inline vec4 operator+(const vec4& v, float s) { return v + vec4(s); }
inline vec4 operator+(float s, const vec4& v) { return vec4(s) + v; }
inline vec4 operator-(const vec4& v, float s) { return v - vec4(s); }
inline vec4 operator-(float s, const vec4& v) { return vec4(s) - v; }

inline float dot(const vec2& a, const vec2& b) { return vec2::Dot(a, b); }
inline float dot(const vec3& a, const vec3& b) { return vec3::Dot(a, b); }
inline float dot(const vec4& a, const vec4& b) { return vec4::Dot(a, b); }
inline float dot(const quat& a, const quat& b) { return quat::Dot(a, b); }
inline quat operator+(const quat& a, const quat& b) { return quat(static_cast<const ::Mathc::Quaternion&>(a) + static_cast<const ::Mathc::Quaternion&>(b)); }
inline quat operator-(const quat& a, const quat& b) { return quat(static_cast<const ::Mathc::Quaternion&>(a) - static_cast<const ::Mathc::Quaternion&>(b)); }
inline quat operator*(const quat& a, const quat& b) { return quat(static_cast<const ::Mathc::Quaternion&>(a) * static_cast<const ::Mathc::Quaternion&>(b)); }
inline vec3 operator*(const quat& q, const vec3& v) { return static_cast<const ::Mathc::Quaternion&>(q) * v; }
inline quat operator*(const quat& q, float s) { return quat(static_cast<const ::Mathc::Quaternion&>(q) * s); }
inline quat operator*(float s, const quat& q) { return quat(s * static_cast<const ::Mathc::Quaternion&>(q)); }
inline quat operator/(const quat& q, float s) { return quat(static_cast<const ::Mathc::Quaternion&>(q) * (1.0f / s)); }
inline vec3 cross(const vec3& a, const vec3& b) { return vec3::Cross(a, b); }
inline float length(const vec2& v) { return v.Length(); }
inline float length(const vec3& v) { return v.Length(); }
inline float length(const vec4& v) { return v.Length(); }
inline float length(const quat& v) { return v.Length(); }
inline vec2 normalize(const vec2& v) { return v.Normalized(); }
inline vec3 normalize(const vec3& v) { return v.Normalized(); }
inline vec4 normalize(const vec4& v) { return v.Normalized(); }
inline quat normalize(const quat& v) { return v.Normalized(); }
inline vec2 min(const vec2& a, const vec2& b) { return vec2::Min(a, b); }
inline vec3 min(const vec3& a, const vec3& b) { return vec3::Min(a, b); }
inline vec4 min(const vec4& a, const vec4& b) { return vec4::Min(a, b); }
inline vec2 max(const vec2& a, const vec2& b) { return vec2::Max(a, b); }
inline vec3 max(const vec3& a, const vec3& b) { return vec3::Max(a, b); }
inline vec4 max(const vec4& a, const vec4& b) { return vec4::Max(a, b); }
template <typename T> inline T min(T a, T b) { return std::min(a, b); }
template <typename T> inline T max(T a, T b) { return std::max(a, b); }
inline vec2 clamp(const vec2& v, const vec2& lo, const vec2& hi) { return vec2::Clamp(v, lo, hi); }
inline vec3 clamp(const vec3& v, const vec3& lo, const vec3& hi) { return vec3::Clamp(v, lo, hi); }
inline vec4 clamp(const vec4& v, const vec4& lo, const vec4& hi) { return vec4::Clamp(v, lo, hi); }
inline ivec3 clamp(const ivec3& v, const ivec3& lo, const ivec3& hi) { return ivec3(std::clamp(v.x, lo.x, hi.x), std::clamp(v.y, lo.y, hi.y), std::clamp(v.z, lo.z, hi.z)); }
template <typename T> inline T clamp(T v, T lo, T hi) { return std::clamp(v, lo, hi); }
inline vec2 mix(const vec2& a, const vec2& b, float t) { return vec2::Lerp(a, b, t); }
inline vec3 mix(const vec3& a, const vec3& b, float t) { return vec3::Lerp(a, b, t); }
inline vec4 mix(const vec4& a, const vec4& b, float t) { return vec4::Lerp(a, b, t); }
inline quat mix(const quat& a, const quat& b, float t) { return quat::Lerp(a, b, t); }
inline float mix(float a, float b, float t) { return a + (b - a) * t; }
inline quat slerp(const quat& a, const quat& b, float t) { return quat::Slerp(a, b, t); }
inline float radians(float v) { return v * ::Mathc::DEG2RAD; }
inline vec3 radians(const vec3& v) { return v * ::Mathc::DEG2RAD; }
inline float degrees(float v) { return v * ::Mathc::RAD2DEG; }
inline vec3 degrees(const vec3& v) { return v * ::Mathc::RAD2DEG; }
template <typename T = float> constexpr T pi() { return static_cast<T>(::Mathc::PI); }
template <typename T = float> constexpr T two_pi() { return static_cast<T>(::Mathc::PI * 2.0f); }
template <typename T = float> constexpr T half_pi() { return static_cast<T>(::Mathc::PI * 0.5f); }
template <typename T = float> constexpr T quarter_pi() { return static_cast<T>(::Mathc::PI * 0.25f); }
inline float abs(float v) { return std::fabs(v); }
inline vec2 abs(const vec2& v) { return vec2(std::fabs(v.x), std::fabs(v.y)); }
inline vec3 abs(const vec3& v) { return vec3(std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)); }
inline vec4 abs(const vec4& v) { return vec4(std::fabs(v.x), std::fabs(v.y), std::fabs(v.z), std::fabs(v.w)); }
inline float floor(float v) { return std::floor(v); }
inline vec3 floor(const vec3& v) { return vec3(std::floor(v.x), std::floor(v.y), std::floor(v.z)); }
inline float ceil(float v) { return std::ceil(v); }
inline vec3 ceil(const vec3& v) { return vec3(std::ceil(v.x), std::ceil(v.y), std::ceil(v.z)); }
inline vec3 round(const vec3& v) { return vec3(std::round(v.x), std::round(v.y), std::round(v.z)); }
inline float round(float v) { return std::round(v); }
inline float distance(const vec3& a, const vec3& b) { return vec3::Distance(a, b); }
inline float distance(const vec2& a, const vec2& b) { return vec2::Distance(a, b); }
inline mat2 transpose(const mat2& m) { return m.Transposed(); }
inline mat3 transpose(const mat3& m) { return m.Transposed(); }
inline mat4 transpose(const mat4& m) { return m.Transposed(); }
inline mat2 inverse(const mat2& m) { return m.Inverse(); }
inline mat3 inverse(const mat3& m) { return m.Inverse(); }
inline mat4 inverse(const mat4& m) { return m.Inverse(); }
inline quat inverse(const quat& q) { return q.Inverse(); }
inline float determinant(const mat2& m) { return m.Determinant(); }
inline float determinant(const mat3& m) { return m.Determinant(); }
inline float determinant(const mat4& m) { return m.Determinant(); }
inline quat conjugate(const quat& q) { return q.Conjugate(); }
inline quat angleAxis(float angle, const vec3& axis) { return quat::FromAxisAngle(axis, angle); }
inline mat3 mat3_cast(const quat& q) { return q.ToMat3(); }
inline mat4 mat4_cast(const quat& q) { return q.ToMat4(); }
inline quat quat_cast(const mat3& m) { return quat::FromMat3(m); }
inline quat quat_cast(const mat4& m) { return quat::FromMat3(m.UpperLeft3x3()); }
inline mat4 translate(const mat4& m, const vec3& v) { return m * mat4::Translation(v); }
inline mat4 scale(const mat4& m, const vec3& v) { return m * mat4::Scale(v); }
inline mat4 rotate(const mat4& m, float angle, const vec3& axis) { return m * mat4_cast(angleAxis(angle, normalize(axis))); }
inline mat4 lookAt(const vec3& eye, const vec3& target, const vec3& up) { return mat4::LookAt(eye, target, up); }
inline mat4 perspective(float fov, float aspect, float nearPlane, float farPlane) { return mat4::Perspective(fov, aspect, nearPlane, farPlane); }
inline mat4 ortho(float left, float right, float bottom, float top, float nearPlane, float farPlane) { return mat4::Ortho(left, right, bottom, top, nearPlane, farPlane); }
inline mat4 frustum(float left, float right, float bottom, float top, float nearPlane, float farPlane)
{
    return mat4(2.0f * nearPlane / (right - left), 0.0f, 0.0f, 0.0f,
                0.0f, 2.0f * nearPlane / (top - bottom), 0.0f, 0.0f,
                (right + left) / (right - left), (top + bottom) / (top - bottom), -(farPlane + nearPlane) / (farPlane - nearPlane), -1.0f,
                0.0f, 0.0f, -(2.0f * farPlane * nearPlane) / (farPlane - nearPlane), 0.0f);
}
inline const float* value_ptr(const vec2& v) { return &v.x; }
inline float* value_ptr(vec2& v) { return &v.x; }
inline const float* value_ptr(const vec3& v) { return &v.x; }
inline float* value_ptr(vec3& v) { return &v.x; }
inline const float* value_ptr(const vec4& v) { return &v.x; }
inline float* value_ptr(vec4& v) { return &v.x; }
inline const float* value_ptr(const mat2& m) { return m.Data(); }
inline float* value_ptr(mat2& m) { return m.Data(); }
inline const float* value_ptr(const mat3& m) { return m.Data(); }
inline float* value_ptr(mat3& m) { return m.Data(); }
inline const float* value_ptr(const mat4& m) { return m.Data(); }
inline float* value_ptr(mat4& m) { return m.Data(); }
inline mat4 make_mat4(const float* values) { return mat4(values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9], values[10], values[11], values[12], values[13], values[14], values[15]); }
inline bool decompose(const mat4& matrix, vec3& scaleOut, quat& rotationOut, vec3& translationOut,
                      vec3& skewOut, vec4& perspectiveOut)
{
    translationOut = matrix.GetTranslation();
    vec3 x = matrix[0].xyz();
    vec3 y = matrix[1].xyz();
    vec3 z = matrix[2].xyz();
    scaleOut = vec3(x.Length(), y.Length(), z.Length());
    if (scaleOut.x <= ::Mathc::EPSILON || scaleOut.y <= ::Mathc::EPSILON || scaleOut.z <= ::Mathc::EPSILON)
        return false;
    x /= scaleOut.x;
    y /= scaleOut.y;
    z /= scaleOut.z;
    if (dot(cross(x, y), z) < 0.0f)
    {
        scaleOut.x = -scaleOut.x;
        x = -x;
    }
    rotationOut = quat_cast(mat3(x, y, z));
    skewOut = vec3(0.0f);
    perspectiveOut = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    return true;
}
inline bvec3 lessThan(const vec3& a, const vec3& b) { return {a.x < b.x, a.y < b.y, a.z < b.z}; }
inline bvec3 greaterThan(const vec3& a, const vec3& b) { return {a.x > b.x, a.y > b.y, a.z > b.z}; }
inline bool any(const bvec3& v) { return v.x || v.y || v.z; }
inline bool all(const bvec3& v) { return v.x && v.y && v.z; }
inline quat rotation(const vec3& from, const vec3& to) { return quat::FromTo(from, to); }
inline float sin(float v) { return std::sin(v); }
inline float cos(float v) { return std::cos(v); }
inline float tan(float v) { return std::tan(v); }
inline float asin(float v) { return std::asin(v); }
inline float atan(float v) { return std::atan(v); }
inline float atan(float y, float x) { return std::atan2(y, x); }
inline float sqrt(float v) { return std::sqrt(v); }
inline float exp(float v) { return std::exp(v); }
inline vec3 exp(const vec3& v) { return vec3(std::exp(v.x), std::exp(v.y), std::exp(v.z)); }
inline float mod(float x, float y) { return std::fmod(x, y); }
inline float inversesqrt(float v) { return 1.0f / std::sqrt(v); }
inline float smoothstep(float edge0, float edge1, float x)
{
    const float t = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}
inline vec3 eulerAngles(const quat& q)
{
    const float sinX = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosX = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    const float sinY = 2.0f * (q.w * q.y - q.z * q.x);
    const float sinZ = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosZ = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    return vec3(std::atan2(sinX, cosX), std::asin(std::clamp(sinY, -1.0f, 1.0f)), std::atan2(sinZ, cosZ));
}
} // namespace Radion::Math

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
