#ifndef RADION_LIGHTMAP_UNWRAPPER_H
#define RADION_LIGHTMAP_UNWRAPPER_H

#include "Mesh.h"

namespace Radion
{

struct LightmapUnwrapSettings
{
    // Returning false aborts the unwrap. It is the only way out of one:
    // packing a large atlas is a single call that can run for many minutes,
    // and nothing else gets a turn while it does - a caller that wants to
    // answer a Ctrl+C, or close a window, has to say so from in here.
    // unwrap() then fails like any other error, leaving `output` cleared.
    using ProgressCallback = bool (*)(const char* stage, u32 percent, void* userData);

    // Page size in texels. Zero is not "no preference" - it is the only
    // setting that guarantees the unwrap comes back as a SINGLE page, sized
    // to whatever texelsPerUnit ends up needing. Any non-zero value pins the
    // page size and lets the atlas spill into as many pages as the charts
    // require, which every consumer that assumes one texture then has to
    // cope with. Set this to 0 unless multi-page output is actually handled.
    u32 resolution = 1024;
    u32 padding = 4;

    // Texels per world unit - what actually decides how big the atlas has to
    // be, since the total scales with the square of it. Zero lets xatlas pick
    // a value that approximately fills `resolution` (or a 1024 page when that
    // is 0 too). Raising it makes every chart bigger and the atlas larger,
    // which is the opposite of what is wanted when an unwrap comes back with
    // more pages than the consumer can take.
    f32 texelsPerUnit = 0.0f;
    ProgressCallback progress = nullptr;
    void* progressUserData = nullptr;
};

// What the unwrap actually produced, as opposed to what was asked for. The
// UV2 it writes is normalized against `width`/`height`, so baking into a
// texture of a different size rescales every chart and shrinks the padding
// with it - at which point neighbouring charts bleed into each other. The
// caller has no way to notice that without these.
struct LightmapUnwrapResult
{
    u32 width = 0;
    u32 height = 0;
    u32 chartCount = 0;
};

// Builds a renderable CPU mesh with a non-overlapping second UV set. The
// input is never modified; xatlas may split vertices at chart seams, so the
// output can have a different vertex and index count.
class LightmapUnwrapper
{
public:
    bool unwrap(const MeshData& input, MeshData& output,
                const LightmapUnwrapSettings& settings = LightmapUnwrapSettings(),
                LightmapUnwrapResult* result = nullptr) const;
};

} // namespace Radion

#endif // RADION_LIGHTMAP_UNWRAPPER_H
