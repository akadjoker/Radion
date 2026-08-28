#ifndef RADION_SCREEN_DRAW_PASS_H
#define RADION_SCREEN_DRAW_PASS_H

#include "Batch.h"

namespace Radion
{

// Draws the ScreenDraw queue on top of whatever is already in the current
// target - no clear, no depth test/write, no cull, alpha blended. Not a
// RenderTechnique: those all run inside Renderer::execute(), before
// tonemapping and TAA, so a menu drawn there would pick up bloom, tonemap
// and TAA blur meant for the 3D scene. This runs after
// PostProcessStack::resolve() instead - see Engine.cpp.
class ScreenDrawPass
{
public:
    ScreenDrawPass();
    ~ScreenDrawPass();

    ScreenDrawPass(const ScreenDrawPass&) = delete;
    ScreenDrawPass& operator=(const ScreenDrawPass&) = delete;

    // drawableWidth/drawableHeight are the full drawable in pixels, not the
    // present rect - fade() has to cover a letterboxed game's black bars
    // too, not just the rendered area inside them. No-op (and the internal
    // BatchRenderer is never touched) when the queue is empty.
    void execute(u32 drawableWidth, u32 drawableHeight);
    void shutdown();

private:
    bool ensureBatch();

    BatchRenderer mBatch;
    bool mBatchInitialized = false;
};

} // namespace Radion

#endif // RADION_SCREEN_DRAW_PASS_H
