#ifndef RADION_PROC_TREE_H
#define RADION_PROC_TREE_H

#include "Types.h"

namespace Radion
{

// Parameters for AssetManager::createTree(), one to one with what an editor
// panel shows. Degenerate combinations - clumpMin over clumpMax, a radius
// wider than the branch, a falloff at or above 1 - are legal here and produce
// what they describe; AssetManager::randomTreeParams() is the one that keeps
// inside the bands that look like trees.
struct TreeParams
{
    f32 clumpMax = 0.8f;
    f32 clumpMin = 0.5f;
    f32 lengthFalloffFactor = 0.85f;
    f32 lengthFalloffPower = 1.0f;
    f32 branchFactor = 2.0f;
    f32 radiusFalloffRate = 0.6f;
    f32 taperRate = 0.95f;
    f32 maxRadius = 0.25f;
    f32 climbRate = 1.5f;
    f32 trunkKink = 0.0f;
    f32 twistRate = 13.0f;
    f32 trunkLength = 2.5f;
    f32 initialBranchLength = 0.85f;
    f32 dropAmount = 0.0f;
    f32 growAmount = 0.0f;
    f32 sweepAmount = 0.0f;
    f32 vMultiplier = 0.2f;
    f32 twigScale = 2.0f;

    // Forced even and at least 4: the fork builder walks segments/2, and an
    // odd count leaves holes in the mesh.
    u32 segments = 6;

    // Depth of the branch tree, and how far the trunk climbs before it starts
    // splitting. Levels is exponential - each one doubles the branch count.
    u32 levels = 3;
    u32 trunkSteps = 2;

    s32 seed = 10;
};

struct TreePreset
{
    const char* name;
    TreeParams params;
    u32 twigTexture;
};

} // namespace Radion

#endif // RADION_PROC_TREE_H
