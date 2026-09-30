#ifndef RADION_BLENDER_PROCEDURAL_SHAPES_H
#define RADION_BLENDER_PROCEDURAL_SHAPES_H

#include "Mesh.h"
#include "Types.h"

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <string>
#include <vector>

namespace Radion
{

// Shapes that a short list of numbers describes but no primitive covers - a
// nose cone, a tapering tail boom, a fuselage. Pure geometry: no engine, no GL,
// so they are tested on their own. Each builds one submesh with smooth
// normals and UVs, wound counter-clockwise seen from outside.

// Surface of revolution around the Y axis. `profile` is (radius, y) from one
// end to the other; a radius of 0 pinches the surface to a point. `capStart` /
// `capEnd` close an open end with a flat disc.
struct LatheParams
{
    std::vector<glm::vec2> profile;
    u32 slices = 24;
    bool capStart = true;
    bool capEnd = true;
};

// Cross-sections strung along an axis. Each section is a superellipse in the
// plane perpendicular to the axis, `width` by `height` across (along the two
// remaining axes, in x-y-z order: for axis Z they are X then Y), centred on
// `offset` and sitting at `at` along the axis. Sections are listed in
// increasing `at`. A width or height of 0 pinches the loft to a point.
struct LoftSection
{
    f32 at = 0.0f;
    f32 width = 1.0f;
    f32 height = 1.0f;
    glm::vec2 offset = glm::vec2(0.0f);
    // 2 is an ellipse; larger values square it off toward a rounded box.
    f32 exponent = 2.0f;
};

struct LoftParams
{
    // 0 = X, 1 = Y, 2 = Z.
    s32 axis = 2;
    std::vector<LoftSection> sections;
    u32 segments = 24;
    bool capStart = true;
    bool capEnd = true;
};

// False with `error` set when the parameters cannot make a surface (too few
// points, non-finite numbers, sections out of order, too many vertices).
bool buildLathe(const LatheParams& params, MeshData& out, std::string* error = nullptr);
bool buildLoft(const LoftParams& params, MeshData& out, std::string* error = nullptr);

} // namespace Radion

#endif // RADION_BLENDER_PROCEDURAL_SHAPES_H
