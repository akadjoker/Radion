#ifndef RADION_UTILS_MATH_UTILS_H
#define RADION_UTILS_MATH_UTILS_H

#include "mathc.h"

#include <cmath>

namespace Radion
{
namespace Utils
{
// Splits an affine, column-major matrix into the same TRS convention used by
// Math::Mat4::Translation() * rotation.ToMat4() * Math::Mat4::Scale().
// Perspective matrices are rejected because engine transforms never use them.
inline bool decomposeAffineTRS(const Math::Mat4& matrix, Math::Vec3& scale,
                               Math::Quaternion& rotation, Math::Vec3& translation,
                               Math::Vec3* skew = nullptr, float epsilon = 1.0e-6f)
{
    if (std::fabs(matrix[0].w) > epsilon || std::fabs(matrix[1].w) > epsilon ||
        std::fabs(matrix[2].w) > epsilon || std::fabs(matrix[3].w - 1.0f) > epsilon)
        return false;

    translation = matrix.GetTranslation();
    Math::Vec3 axisX = matrix[0].xyz();
    Math::Vec3 axisY = matrix[1].xyz();
    Math::Vec3 axisZ = matrix[2].xyz();

    scale.x = axisX.Length();
    if (scale.x <= epsilon)
        return false;
    axisX /= scale.x;

    Math::Vec3 localSkew;
    localSkew.z = axisX.Dot(axisY);
    axisY -= axisX * localSkew.z;
    scale.y = axisY.Length();
    if (scale.y <= epsilon)
        return false;
    axisY /= scale.y;
    localSkew.z /= scale.y;

    localSkew.y = axisX.Dot(axisZ);
    axisZ -= axisX * localSkew.y;
    localSkew.x = axisY.Dot(axisZ);
    axisZ -= axisY * localSkew.x;
    scale.z = axisZ.Length();
    if (scale.z <= epsilon)
        return false;
    axisZ /= scale.z;
    localSkew.y /= scale.z;
    localSkew.x /= scale.z;

    // A reflected basis has a negative determinant. Keep that information in
    // scale so composing the result reconstructs the original transform.
    if (axisX.Dot(axisY.Cross(axisZ)) < 0.0f)
    {
        scale = -scale;
        axisX = -axisX;
        axisY = -axisY;
        axisZ = -axisZ;
    }

    rotation = Math::Quaternion::FromMat3(Math::Mat3(axisX, axisY, axisZ)).Normalized();
    if (skew)
        *skew = localSkew;
    return true;
}
} // namespace Utils
} // namespace Radion

#endif // RADION_UTILS_MATH_UTILS_H
