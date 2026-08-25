#include "PCH.h"

#include "OffscreenTarget.h"

namespace Radion
{

bool OffscreenTarget::create(u32 w, u32 h, Format colorFormat, Format depthFormat,
                             const char* debugName, bool mips, Format velocityFormat,
                             Format reactiveFormat, bool storage)
{
    // Built into a temporary set first, and only swapped in once every piece
    // of it has succeeded: destroying the old set up front - as this used to
    // do - meant a resize that failed partway (an OOM on the depth texture,
    // an incomplete FBO on some driver) left the caller with nothing valid
    // at all, when the old, still-good set was sitting right there.
    OffscreenTarget next;
    GPU& gpu = GPU::getSingleton();

    TextureDesc colorDesc;
    colorDesc.type = TextureType::Tex2D;
    colorDesc.format = colorFormat;
    colorDesc.width = w;
    colorDesc.height = h;
    // 0 asks for the full chain, per TextureDesc's own convention.
    colorDesc.mips = mips ? 0 : 1;
    colorDesc.usage = TextureSampled | TextureTarget | (storage ? TextureStorage : 0);
    colorDesc.debugName = debugName;
    next.color = gpu.createTexture(colorDesc);

    if (velocityFormat != Format::Unknown)
    {
        TextureDesc velocityDesc = colorDesc;
        velocityDesc.format = velocityFormat;
        next.velocity = gpu.createTexture(velocityDesc);
    }
    if (reactiveFormat != Format::Unknown)
    {
        TextureDesc reactiveDesc = colorDesc;
        reactiveDesc.format = reactiveFormat;
        next.reactive = gpu.createTexture(reactiveDesc);
    }

    if (depthFormat != Format::Unknown)
    {
        TextureDesc depthDesc = colorDesc;
        depthDesc.format = depthFormat;
        next.depth = gpu.createTexture(depthDesc);
    }

    if (!next.color.valid() || (velocityFormat != Format::Unknown && !next.velocity.valid()) ||
        (reactiveFormat != Format::Unknown && !next.reactive.valid()) ||
        (depthFormat != Format::Unknown && !next.depth.valid()))
    {
        next.destroy();
        return false;
    }

    TargetDesc targetDesc;
    targetDesc.colors[0].texture = next.color;
    targetDesc.colorCount = 1;
    if (next.velocity.valid())
        targetDesc.colors[targetDesc.colorCount++].texture = next.velocity;
    if (next.reactive.valid())
        targetDesc.colors[targetDesc.colorCount++].texture = next.reactive;
    if (next.depth.valid())
        targetDesc.depth.texture = next.depth;
    targetDesc.debugName = debugName;
    next.target = gpu.createTarget(targetDesc);

    if (!next.target.valid())
    {
        next.destroy();
        return false;
    }

    next.width = w;
    next.height = h;

    // Only now does the set this call replaces go away.
    destroy();
    *this = next;
    return true;
}

void OffscreenTarget::destroy()
{
    GPU& gpu = GPU::getSingleton();
    if (target.valid())
        gpu.destroy(target);
    if (color.valid())
        gpu.destroy(color);
    if (velocity.valid())
        gpu.destroy(velocity);
    if (reactive.valid())
        gpu.destroy(reactive);
    if (depth.valid())
        gpu.destroy(depth);

    target = TargetHandle();
    color = TextureHandle();
    velocity = TextureHandle();
    reactive = TextureHandle();
    depth = TextureHandle();
    width = 0;
    height = 0;
}

} // namespace Radion
