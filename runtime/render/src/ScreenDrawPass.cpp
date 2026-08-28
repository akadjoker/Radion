#include "PCH.h"

#include "ScreenDrawPass.h"

#include "ScreenDraw.h"

namespace Radion
{

ScreenDrawPass::ScreenDrawPass() = default;

ScreenDrawPass::~ScreenDrawPass()
{
    shutdown();
}

bool ScreenDrawPass::ensureBatch()
{
    if (mBatchInitialized)
        return true;

    BatchRenderer::Config config;
    config.maxVertices = 16384;
    config.maxDrawCalls = 64;
    config.enableProfiling = false;
    if (!mBatch.init(config))
        return false;

    mBatchInitialized = true;
    return true;
}

void ScreenDrawPass::execute(u32 drawableWidth, u32 drawableHeight)
{
    ScreenDraw& queue = ScreenDraws();
    if (queue.empty())
        return;
    if (!ensureBatch())
        return;

    GPU& gpu = GPU::getSingleton();
    gpu.setTarget(TargetHandle());
    gpu.setViewport(Viewport{0.0f, 0.0f, static_cast<f32>(drawableWidth),
                             static_cast<f32>(drawableHeight)});

    mBatch.resize(static_cast<int>(drawableWidth), static_cast<int>(drawableHeight));
    mBatch.update();
    mBatch.loadIdentity();
    mBatch.setDepthTest(false);
    mBatch.setDepthWrite(false);
    mBatch.setCullFace(false);
    mBatch.setBlend(true);
    mBatch.setBlendMode(BatchRenderer::BlendMode::Alpha);

    const f32 screenWidth = static_cast<f32>(drawableWidth);
    const f32 screenHeight = static_cast<f32>(drawableHeight);

    for (const ScreenDrawCommand& command : queue.commands())
    {
        mBatch.setColor(command.color.r(), command.color.g(), command.color.b(), command.color.a());
        switch (command.type)
        {
        case ScreenDrawCommandType::Line:
            if (command.thickness > 1.0f)
                mBatch.drawThickLine(command.x0, command.y0, command.x1, command.y1,
                                     command.thickness);
            else
                mBatch.drawLine(command.x0, command.y0, command.x1, command.y1);
            break;
        case ScreenDrawCommandType::Rect:
        {
            const FloatRect rect = ScreenDraw::resolvedRect(command, screenWidth, screenHeight);
            mBatch.drawRect(rect.x, rect.y, rect.width, rect.height, command.filled);
            break;
        }
        case ScreenDrawCommandType::Sprite:
            mBatch.drawTexture(command.texture, command.x, command.y, command.w, command.h,
                               command.srcX, command.srcY, command.srcW, command.srcH,
                               command.pivotX, command.pivotY, command.rotationDeg);
            break;
        case ScreenDrawCommandType::Text:
            mBatch.drawText(command.x, command.y, command.textSize,
                            queue.textAt(command.textOffset));
            break;
        }
    }

    mBatch.drawRenderBatch();
    mBatch.flip();
}

void ScreenDrawPass::shutdown()
{
    if (mBatchInitialized)
    {
        mBatch.shutdown();
        mBatchInitialized = false;
    }
}

} // namespace Radion
