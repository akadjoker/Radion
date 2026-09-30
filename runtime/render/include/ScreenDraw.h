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

// One queued 2D command, in window pixels, origin top-left. A single flat struct so layering stays stable across types: the draw side stable_sorts one submission-ordered list by layer.
struct ScreenDrawCommand
{
    ScreenDrawCommandType type = ScreenDrawCommandType::Line;
    s32 layer = 0;
    Color color;

    f32 x0 = 0.0f;
    f32 y0 = 0.0f;
    f32 x1 = 0.0f;
    f32 y1 = 0.0f;
    f32 thickness = 1.0f;

    // Rect/Sprite top-left corner and size. Text's (x, y) is the top-left of its first glyph cell, not a baseline.
    f32 x = 0.0f;
    f32 y = 0.0f;
    f32 w = 0.0f;
    f32 h = 0.0f;

    bool filled = true;
    // Set by fade(): the queue never learns the drawable size; the draw side substitutes it in resolvedRect().
    bool fullscreen = false;

    TextureHandle texture;
    f32 srcX = 0.0f;
    f32 srcY = 0.0f;
    f32 srcW = 0.0f;
    f32 srcH = 0.0f;
    f32 pivotX = 0.0f;
    f32 pivotY = 0.0f;
    f32 rotationDeg = 0.0f;

    // Text: offset/length into ScreenDraw's character buffer, avoiding a std::string allocation per command.
    usize textOffset = 0;
    usize textLength = 0;
    f32 textSize = 0.0f;
};

// Frame-scoped queue of 2D commands drawn over the resolved backbuffer. Pure data (no GL), so it can be filled and inspected without a graphics context. Consumed by ScreenDrawPass.
class ScreenDraw
{
public:
    static ScreenDraw& getSingleton();

    // Drops every command but keeps capacity; called at frame start, so a command submitted after the frame's draw is lost rather than leaking into the next frame.
    void clear();

    void line(f32 x0, f32 y0, f32 x1, f32 y1, Color color, f32 thickness = 1.0f, s32 layer = 0);
    void rect(f32 x, f32 y, f32 w, f32 h, Color color, bool filled = true, s32 layer = 0);
    // srcX/srcY/srcW/srcH at zero = whole texture (as BatchRenderer::drawTexture).
    void sprite(TextureHandle texture, f32 x, f32 y, f32 w, f32 h, Color color, f32 srcX = 0.0f,
                f32 srcY = 0.0f, f32 srcW = 0.0f, f32 srcH = 0.0f, f32 pivotX = 0.0f,
                f32 pivotY = 0.0f, f32 rotationDeg = 0.0f, s32 layer = 0);
    void text(f32 x, f32 y, f32 size, Color color, const char* utf8, s32 layer = 0);
    // A Rect sized to the drawable; see ScreenDrawCommand::fullscreen.
    void fade(Color color, s32 layer = 0);

    // Commands by ascending layer, submission order within a layer. Sorted lazily here (std::stable_sort) rather than per submit, which is the hot path.
    const std::vector<ScreenDrawCommand>& commands();
    const char* textAt(usize offset) const;
    bool empty() const;

    // The rect a command covers once the drawable size is known (resolves fullscreen). Pure geometry, callable with any resolution.
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
