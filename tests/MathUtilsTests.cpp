#include "PCH.h"

#include "Utils/MathUtils.h"

#include <cstdio>

using namespace Radion;

namespace
{
bool near(f32 a, f32 b, f32 epsilon = 1.0e-4f)
{
    return std::fabs(a - b) <= epsilon;
}

bool near(const Math::Mat4& a, const Math::Mat4& b, f32 epsilon = 1.0e-4f)
{
    for (s32 column = 0; column < 4; ++column)
        for (s32 row = 0; row < 4; ++row)
            if (!near(a[column][row], b[column][row], epsilon))
                return false;
    return true;
}

bool testDecomposeAffineTRS()
{
    const Math::Vec3 translation(4.0f, -3.0f, 9.0f);
    const Math::Quaternion rotation =
        Math::Quaternion::FromAxisAngle(Math::Vec3(1.0f, 2.0f, -1.0f).Normalized(), 0.75f);
    const Math::Vec3 scale(2.0f, 3.0f, 4.0f);
    const Math::Mat4 transform = Math::Mat4::Translation(translation) * rotation.ToMat4() *
                                 Math::Mat4::Scale(scale);

    Math::Vec3 actualTranslation;
    Math::Vec3 actualScale;
    Math::Quaternion actualRotation;
    if (!Utils::decomposeAffineTRS(transform, actualScale, actualRotation, actualTranslation))
        return false;

    const Math::Mat4 recomposed = Math::Mat4::Translation(actualTranslation) *
                                  actualRotation.ToMat4() * Math::Mat4::Scale(actualScale);
    return near(recomposed, transform);
}

bool testRejectsPerspective()
{
    Math::Vec3 scale;
    Math::Vec3 translation;
    Math::Quaternion rotation;
    return !Utils::decomposeAffineTRS(Math::Mat4::Perspective(1.0f, 1.0f, 0.1f, 100.0f), scale,
                                      rotation, translation);
}
} // namespace

int main()
{
    if (testDecomposeAffineTRS() && testRejectsPerspective())
        return 0;
    std::fprintf(stderr, "MathUtilsTests: failed\n");
    return 1;
}
