#ifndef RADION_OFFSCREEN_TARGET_H
#define RADION_OFFSCREEN_TARGET_H

#include "GPU.h"

namespace Radion
{

// A colour + depth render target pair, the shape every render-to-texture
// technique needs (a planar reflection, a scene colour copy for refraction).
// Owns its own textures - unlike AssetManager's, these are never shared or
// looked up by name, only published under one once built.
struct OffscreenTarget
{
    TargetHandle target;
    TextureHandle color;
    TextureHandle velocity;
    TextureHandle reactive;
    TextureHandle depth;
    u32 width = 0;
    u32 height = 0;

    bool valid() const
    {
        return target.valid();
    }

    // Pass Format::Unknown for depth when a colour-only ping-pong target is
    // enough (post-processing, for example). `mips` allocates the full chain
    // on the colour texture instead of just level 0 - for a target something
    // reads back with textureLod (a mirror's roughness-driven blur), not for
    // one only ever sampled at its native resolution; the caller still has
    // to generate them itself after rendering (GPU::generateMips()), this
    // only reserves the room.
    bool create(u32 w, u32 h, Format colorFormat, Format depthFormat, const char* debugName,
               bool mips = false, Format velocityFormat = Format::Unknown,
               Format reactiveFormat = Format::Unknown, bool storage = false);
    void destroy();
};

} // namespace Radion

#endif // RADION_OFFSCREEN_TARGET_H
