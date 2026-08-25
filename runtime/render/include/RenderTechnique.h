#ifndef RADION_RENDER_TECHNIQUE_H
#define RADION_RENDER_TECHNIQUE_H

#include "FrameContext.h"

namespace Radion
{

// A self-contained stage of the frame: owns its shaders and resources and
// draws when asked. Terrain, grass, trees, particles, ocean and post-process
// are all this shape, which is what lets the frame be a list of stages
// assembled at startup instead of fixed code.
class RenderTechnique
{
public:
    virtual ~RenderTechnique() = default;

    virtual const char* name() const = 0;

    // Called once before the first frame. False means the technique failed to
    // build and Renderer drops it rather than running it broken.
    virtual bool setup() = 0;

    virtual void execute(const FrameContext& frame) = 0;

    // Must run while the GPU context is alive; Renderer::shutdown() calls it.
    virtual void shutdown()
    {
    }

    bool isEnabled() const
    {
        return mEnabled;
    }
    void setEnabled(bool enabled)
    {
        mEnabled = enabled;
    }

private:
    bool mEnabled = true;
};

} // namespace Radion

#endif // RADION_RENDER_TECHNIQUE_H
