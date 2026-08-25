// Layout shared by the cull compute, the simulation and the draw. Every offset
// here is agreed with GrassRender.h - a field added on one side and not the
// other reads as garbage rather than as a compile error.

const int GrassMaxInfluencers = 8;

struct GrassClump
{
    vec4 positionScale;  // xyz world position, w scale
    vec4 normalRotation; // xyz surface normal, w rotation about it
    vec4 rectPad;        // x atlas rect index
};

struct GrassAtlasRect
{
    vec4 texMulAdd;  // xy scale, zw offset
    vec4 sizeAspect; // x height multiplier, y width/height of the region
};

// Per tuft, the tip in world space now and one step ago. The velocity is not
// stored: it is the difference between the two, which is what makes this
// Verlet and not a spring with bookkeeping.
struct GrassSim
{
    vec4 currentTail;
    vec4 previousTail;
};

layout(std140, binding = 0) uniform GrassBlock
{
    mat4 uViewProjection;
    vec4 uCameraPosition; // xyz, w time
    vec4 uCameraUp;       // xyz, w draw distance
    vec4 uFrustum[6];     // xyz normal, w distance
    vec4 uShape;          // height, width, wind, alpha cut
    vec4 uCounts;         // camera bend, clump count, influencer count, reset
    vec4 uStep;           // delta time, stiffness, drag, unused

    // xyz centre, w radius. The forces come packed four to a vec4 rather than
    // one per slot - std140 would round each float up to sixteen bytes.
    vec4 uInfluencers[GrassMaxInfluencers];
    vec4 uInfluencerForces[GrassMaxInfluencers / 4];
};

layout(std430, binding = 0) readonly buffer Clumps
{
    GrassClump uClumps[];
};

layout(std430, binding = 1) readonly buffer AtlasRects
{
    GrassAtlasRect uRects[];
};

// Length of the stem, agreed between the simulation that constrains the tip to
// it and the draw that lays the quads along it.
float grassStemLength(GrassClump clump)
{
    const GrassAtlasRect rect = uRects[uint(clump.rectPad.x)];
    return max(0.001, uShape.x * rect.sizeAspect.x * clump.positionScale.w);
}
