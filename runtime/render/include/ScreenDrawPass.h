#ifndef RADION_SCREEN_DRAW_PASS_H
#define RADION_SCREEN_DRAW_PASS_H

#include "Batch.h"

namespace Radion
{

// Draws the ScreenDraw queue over the current target: no clear, depth or cull, alpha blended.
// Not a RenderTechnique: those run before tonemapping and TAA, which would bloom/blur a menu. Runs after PostProcessStack::resolve() (see Engine.cpp).
class ScreenDrawPass
{
public:
    ScreenDrawPass();
    ~ScreenDrawPass();

    ScreenDrawPass(const ScreenDrawPass&) = delete;
    ScreenDrawPass& operator=(const ScreenDrawPass&) = delete;

    // drawableWidth/Height are the full drawable, not the present rect, so fade() covers letterbox bars. No-op when the queue is empty.
    void execute(u32 drawableWidth, u32 drawableHeight);
    void shutdown();

private:
    bool ensureBatch();

    BatchRenderer mBatch;
    bool mBatchInitialized = false;
};

} // namespace Radion

#endif // RADION_SCREEN_DRAW_PASS_H
