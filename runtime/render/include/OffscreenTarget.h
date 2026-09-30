#ifndef RADION_OFFSCREEN_TARGET_H
#define RADION_OFFSCREEN_TARGET_H

#include "GPU.h"

namespace Radion
{

// Colour + depth render target pair for render-to-texture techniques. Owns its textures; never shared or looked up by name.
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

    // Format::Unknown depth gives a colour-only ping-pong target. `mips` reserves the full chain on colour (for textureLod readers); the caller still calls GPU::generateMips().
    bool create(u32 w, u32 h, Format colorFormat, Format depthFormat, const char* debugName,
               bool mips = false, Format velocityFormat = Format::Unknown,
               Format reactiveFormat = Format::Unknown, bool storage = false);
    void destroy();
};

} // namespace Radion

#endif // RADION_OFFSCREEN_TARGET_H
