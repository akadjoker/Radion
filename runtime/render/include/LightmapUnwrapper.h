#ifndef RADION_LIGHTMAP_UNWRAPPER_H
#define RADION_LIGHTMAP_UNWRAPPER_H

#include "Mesh.h"

namespace Radion
{

struct LightmapUnwrapSettings
{
    // Returning false aborts the unwrap (unwrap() then fails with `output` cleared); the only way out of a long atlas pack.
    using ProgressCallback = bool (*)(const char* stage, u32 percent, void* userData);

    // Page size in texels. Zero is the only setting that guarantees a SINGLE page (sized by texelsPerUnit); non-zero pins the size and may spill into multiple pages.
    u32 resolution = 1024;
    u32 padding = 4;

    // Texels per world unit; atlas size scales with its square. Zero lets xatlas pick one that roughly fills `resolution` (1024 if also 0).
    f32 texelsPerUnit = 0.0f;
    ProgressCallback progress = nullptr;
    void* progressUserData = nullptr;
};

// What the unwrap produced. UV2 is normalized against width/height, so baking into a different-size texture rescales charts and bleeds padding.
struct LightmapUnwrapResult
{
    u32 width = 0;
    u32 height = 0;
    u32 chartCount = 0;
};

// Builds a CPU mesh with a non-overlapping second UV set. Input is unmodified; xatlas may split vertices at seams.
class LightmapUnwrapper
{
public:
    bool unwrap(const MeshData& input, MeshData& output,
                const LightmapUnwrapSettings& settings = LightmapUnwrapSettings(),
                LightmapUnwrapResult* result = nullptr) const;
};

} // namespace Radion

#endif // RADION_LIGHTMAP_UNWRAPPER_H
