#include "PCH.h"

#include "ScreenDraw.h"

#include <algorithm>
#include <cstring>

namespace Radion
{

namespace
{

bool commandLayerLess(const ScreenDrawCommand& a, const ScreenDrawCommand& b)
{
    return a.layer < b.layer;
}

} // namespace

ScreenDraw::ScreenDraw()
{
    mCommands.reserve(256);
    mTextBuffer.reserve(1024);
}

ScreenDraw& ScreenDraw::getSingleton()
{
    static ScreenDraw instance;
    return instance;
}

void ScreenDraw::clear()
{
    mCommands.clear();
    mTextBuffer.clear();
    mSorted = true;
}

void ScreenDraw::line(f32 x0, f32 y0, f32 x1, f32 y1, Color color, f32 thickness, s32 layer)
{
    ScreenDrawCommand command;
    command.type = ScreenDrawCommandType::Line;
    command.layer = layer;
    command.color = color;
    command.x0 = x0;
    command.y0 = y0;
    command.x1 = x1;
    command.y1 = y1;
    command.thickness = thickness;
    mCommands.push_back(command);
    mSorted = false;
}

void ScreenDraw::rect(f32 x, f32 y, f32 w, f32 h, Color color, bool filled, s32 layer)
{
    ScreenDrawCommand command;
    command.type = ScreenDrawCommandType::Rect;
    command.layer = layer;
    command.color = color;
    command.x = x;
    command.y = y;
    command.w = w;
    command.h = h;
    command.filled = filled;
    mCommands.push_back(command);
    mSorted = false;
}

void ScreenDraw::sprite(TextureHandle texture, f32 x, f32 y, f32 w, f32 h, Color color, f32 srcX,
                        f32 srcY, f32 srcW, f32 srcH, f32 pivotX, f32 pivotY, f32 rotationDeg,
                        s32 layer)
{
    ScreenDrawCommand command;
    command.type = ScreenDrawCommandType::Sprite;
    command.layer = layer;
    command.color = color;
    command.x = x;
    command.y = y;
    command.w = w;
    command.h = h;
    command.texture = texture;
    command.srcX = srcX;
    command.srcY = srcY;
    command.srcW = srcW;
    command.srcH = srcH;
    command.pivotX = pivotX;
    command.pivotY = pivotY;
    command.rotationDeg = rotationDeg;
    mCommands.push_back(command);
    mSorted = false;
}

void ScreenDraw::text(f32 x, f32 y, f32 size, Color color, const char* utf8, s32 layer)
{
    if (!utf8 || !(size > 0.0f))
        return;

    const usize length = std::strlen(utf8);
    const usize offset = mTextBuffer.size();
    mTextBuffer.insert(mTextBuffer.end(), utf8, utf8 + length);
    mTextBuffer.push_back('\0');

    ScreenDrawCommand command;
    command.type = ScreenDrawCommandType::Text;
    command.layer = layer;
    command.color = color;
    command.x = x;
    command.y = y;
    command.textSize = size;
    command.textOffset = offset;
    command.textLength = length;
    mCommands.push_back(command);
    mSorted = false;
}

void ScreenDraw::fade(Color color, s32 layer)
{
    ScreenDrawCommand command;
    command.type = ScreenDrawCommandType::Rect;
    command.layer = layer;
    command.color = color;
    command.filled = true;
    command.fullscreen = true;
    mCommands.push_back(command);
    mSorted = false;
}

const std::vector<ScreenDrawCommand>& ScreenDraw::commands()
{
    if (!mSorted)
    {
        std::stable_sort(mCommands.begin(), mCommands.end(), commandLayerLess);
        mSorted = true;
    }
    return mCommands;
}

const char* ScreenDraw::textAt(usize offset) const
{
    return mTextBuffer.data() + offset;
}

bool ScreenDraw::empty() const
{
    return mCommands.empty();
}

FloatRect ScreenDraw::resolvedRect(const ScreenDrawCommand& command, f32 screenWidth,
                                   f32 screenHeight)
{
    if (command.fullscreen)
        return FloatRect(0.0f, 0.0f, screenWidth, screenHeight);
    return FloatRect(command.x, command.y, command.w, command.h);
}

} // namespace Radion
