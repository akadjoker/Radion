#ifndef RADION_SCREEN_DRAW_H
#define RADION_SCREEN_DRAW_H

#include "Color.h"
#include "GPU.h"
#include "Math.h"

#include <vector>

namespace Radion
{

enum class ScreenDrawCommandType : u8
{
    Line,
    Rect,
    Sprite,
    Text
};

// One queued 2D command, in window pixels with the origin at the top-left.
// A single flat struct rather than one vector per command type: layering has
// to stay stable across types too (a label drawn after a panel in the same
// menu must not swap places with it between frames), so the draw side needs
// one submission-ordered list it can stable_sort by layer - splitting by
// type would only add a sequence number per element and a merge at draw
// time, for no benefit.
struct ScreenDrawCommand
{
    ScreenDrawCommandType type = ScreenDrawCommandType::Line;
    s32 layer = 0;
    Color color;

    // Line endpoints.
    f32 x0 = 0.0f;
    f32 y0 = 0.0f;
    f32 x1 = 0.0f;
    f32 y1 = 0.0f;
    f32 thickness = 1.0f;

    // Rect/Sprite top-left corner and size. Text's (x, y) is the top-left of
    // its first glyph cell too, not a baseline - BatchRenderer::drawText
    // emits each glyph from the pen down and to the right.
    f32 x = 0.0f;
    f32 y = 0.0f;
    f32 w = 0.0f;
    f32 h = 0.0f;

    bool filled = true;
    // Set by fade(): the queue never learns the drawable size, so the draw
    // side substitutes it at resolvedRect() time instead.
    bool fullscreen = false;

    // Sprite.
    TextureHandle texture;
    f32 srcX = 0.0f;
    f32 srcY = 0.0f;
    f32 srcW = 0.0f;
    f32 srcH = 0.0f;
    f32 pivotX = 0.0f;
    f32 pivotY = 0.0f;
    f32 rotationDeg = 0.0f;

    // Text: offset/length into ScreenDraw's own character buffer, not a
    // std::string per command - this is per-frame queueing, and a HUD label
    // is not worth an allocation every frame.
    usize textOffset = 0;
    usize textLength = 0;
    f32 textSize = 0.0f;
};

// Frame-scoped queue of 2D commands to draw over the resolved backbuffer.
// Pure data - no GL, no BatchRenderer - so it can be filled and inspected
// without a graphics context. See ScreenDrawPass for the side that consumes
// it.
class ScreenDraw
{
public:
    static ScreenDraw& getSingleton();

    // Drops every queued command but keeps the vectors' capacity - called
    // once at the start of each frame, so a command submitted after the
    // frame's draw has already happened is silently lost rather than
    // leaking into the next frame.
    void clear();

    void line(f32 x0, f32 y0, f32 x1, f32 y1, Color color, f32 thickness = 1.0f, s32 layer = 0);
    void rect(f32 x, f32 y, f32 w, f32 h, Color color, bool filled = true, s32 layer = 0);
    // srcX/srcY/srcW/srcH at zero means the whole texture, same convention
    // as BatchRenderer::drawTexture.
    void sprite(TextureHandle texture, f32 x, f32 y, f32 w, f32 h, Color color, f32 srcX = 0.0f,
                f32 srcY = 0.0f, f32 srcW = 0.0f, f32 srcH = 0.0f, f32 pivotX = 0.0f,
                f32 pivotY = 0.0f, f32 rotationDeg = 0.0f, s32 layer = 0);
    void text(f32 x, f32 y, f32 size, Color color, const char* utf8, s32 layer = 0);
    // A Rect command sized to whatever the drawable turns out to be - see
    // ScreenDrawCommand::fullscreen and resolvedRect().
    void fade(Color color, s32 layer = 0);

    // Queued commands ordered by ascending layer; commands sharing a layer
    // keep their submission order. Sorting is std::stable_sort, done here
    // lazily on read rather than on every submit - submit is the actual
    // per-frame hot path, this is called at most once per frame by the draw
    // side (plus whenever a test wants to inspect the order).
    const std::vector<ScreenDrawCommand>& commands();
    // Null-terminated pointer to a Text command's characters - see
    // ScreenDrawCommand::textOffset/textLength.
    const char* textAt(usize offset) const;
    bool empty() const;

    // The rect a command actually covers once the drawable size is known -
    // resolves ScreenDrawCommand::fullscreen. Pure geometry, no GL, so a
    // test can call it with any made-up resolution.
    static FloatRect resolvedRect(const ScreenDrawCommand& command, f32 screenWidth,
                                  f32 screenHeight);

private:
    ScreenDraw();

    std::vector<ScreenDrawCommand> mCommands;
    std::vector<char> mTextBuffer;
    bool mSorted = true;
};

inline ScreenDraw& ScreenDraws()
{
    return ScreenDraw::getSingleton();
}

} // namespace Radion

#endif // RADION_SCREEN_DRAW_H
