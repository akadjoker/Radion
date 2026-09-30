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

// A flat outline pushed straight through: `profile` (x, y, at least 3 points, no
// repeated last point, simple - it must not cross itself, and it has no holes) is
// extruded along Z from -depth/2 to +depth/2. Concave outlines are fine. Either
// winding is accepted. Flat-shaded.
struct ExtrusionParams
{
    std::vector<glm::vec2> profile;
    f32 depth = 1.0f;
};

// A few solids that are an outline extruded (or a lathe) but come up often enough
// to name. All are centred on the origin, flat-shaded, and stand with their
// height along Y.
struct DiscParams
{
    f32 radius = 0.5f;
    u32 slices = 24;
};
struct TubeParams
{
    f32 outerRadius = 0.5f;
    f32 innerRadius = 0.35f;
    f32 height = 1.0f;
    u32 slices = 24;
};
struct PrismParams
{
    u32 sides = 6;
    f32 radius = 0.5f;
    f32 height = 1.0f;
};
// A staircase: the run goes along +Z, the rise along +Y, the width along X. Its
// bounding box is centred on the origin.
struct StairsParams
{
    u32 steps = 5;
    f32 width = 1.0f;
    f32 stepHeight = 0.2f;
    f32 stepDepth = 0.3f;
};
// An archway: two piers and a semicircular head, `thickness` thick, with a
// matching opening. Width along X, height along Y (feet to crown), depth along Z.
struct ArchParams
{
    f32 width = 2.0f;
    f32 height = 2.5f;
    f32 depth = 0.5f;
    f32 thickness = 0.4f;
    u32 segments = 16;
};

// False with `error` set when the parameters cannot make a surface (too few
// points, non-finite numbers, sections out of order, too many vertices).
bool buildLathe(const LatheParams& params, MeshData& out, std::string* error = nullptr);
bool buildLoft(const LoftParams& params, MeshData& out, std::string* error = nullptr);
bool buildExtrusion(const ExtrusionParams& params, MeshData& out, std::string* error = nullptr);
// A flat disc facing +Y (one-sided: it is a surface, not a solid).
bool buildDisc(const DiscParams& params, MeshData& out, std::string* error = nullptr);
bool buildTube(const TubeParams& params, MeshData& out, std::string* error = nullptr);
bool buildPrism(const PrismParams& params, MeshData& out, std::string* error = nullptr);
bool buildStairs(const StairsParams& params, MeshData& out, std::string* error = nullptr);
bool buildArch(const ArchParams& params, MeshData& out, std::string* error = nullptr);

} // namespace Radion

#endif // RADION_BLENDER_PROCEDURAL_SHAPES_H
