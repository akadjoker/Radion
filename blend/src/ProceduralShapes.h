#ifndef RADION_BLENDER_PROCEDURAL_SHAPES_H
#define RADION_BLENDER_PROCEDURAL_SHAPES_H

#include "Mesh.h"
#include "Types.h"

#include "Math.h"
#include <string>
#include <vector>

namespace Radion
{

// Pure geometry (no engine, no GL); each builds one submesh with smooth normals and UVs, wound counter-clockwise from
// outside.

// Surface of revolution around Y; `profile` is (radius, y) and radius 0 pinches to a point; caps close open ends.
struct LatheParams
{
    std::vector<Math::vec2> profile;
    u32 slices = 24;
    bool capStart = true;
    bool capEnd = true;
};

// Superellipse sections perpendicular to the axis, `width` by `height` along the two remaining axes in x-y-z order,
// at `at` along the axis (increasing); a width or height of 0 pinches to a point.
struct LoftSection
{
    f32 at = 0.0f;
    f32 width = 1.0f;
    f32 height = 1.0f;
    Math::vec2 offset = Math::vec2(0.0f);
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

// Outline (x, y; 3+ points, no repeated last point, simple, no holes; either winding) extruded along Z from -depth/2 to
// +depth/2. Flat-shaded.
struct ExtrusionParams
{
    std::vector<Math::vec2> profile;
    f32 depth = 1.0f;
};

// Named solids: centred on the origin, flat-shaded, height along Y.
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
// Run along +Z, rise along +Y, width along X; bounding box centred on the origin.
struct StairsParams
{
    u32 steps = 5;
    f32 width = 1.0f;
    f32 stepHeight = 0.2f;
    f32 stepDepth = 0.3f;
};
// Width along X, height along Y (feet to crown), depth along Z.
struct ArchParams
{
    f32 width = 2.0f;
    f32 height = 2.5f;
    f32 depth = 0.5f;
    f32 thickness = 0.4f;
    u32 segments = 16;
};

// False with `error` set when the parameters cannot make a surface.
bool buildLathe(const LatheParams& params, MeshData& out, std::string* error = nullptr);
bool buildLoft(const LoftParams& params, MeshData& out, std::string* error = nullptr);
bool buildExtrusion(const ExtrusionParams& params, MeshData& out, std::string* error = nullptr);
// A flat disc facing +Y (one-sided).
bool buildDisc(const DiscParams& params, MeshData& out, std::string* error = nullptr);
bool buildTube(const TubeParams& params, MeshData& out, std::string* error = nullptr);
bool buildPrism(const PrismParams& params, MeshData& out, std::string* error = nullptr);
bool buildStairs(const StairsParams& params, MeshData& out, std::string* error = nullptr);
bool buildArch(const ArchParams& params, MeshData& out, std::string* error = nullptr);

} // namespace Radion

#endif // RADION_BLENDER_PROCEDURAL_SHAPES_H
