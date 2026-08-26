#include "PCH.h"

#include "Text3D.h"

#include "Batch.h"
#include "GameObject.h"

namespace Radion
{

Text3D::Text3D() : Component(Type, ComponentEventLateUpdate)
{
}

namespace
{

f32 alignOffset(TextAlign align, f32 width)
{
    if (align == TextAlign::Center)
        return -width * 0.5f;
    if (align == TextAlign::Right)
        return -width;
    return 0.0f;
}

} // namespace

void Text3D::setText(const std::string& text)
{
    if (text == mText)
        return;
    mText = text;
    mGlyphsDirty = true;
}
const std::string& Text3D::text() const
{
    return mText;
}
void Text3D::setCharacterSize(f32 size)
{
    const f32 clamped = glm::max(size, 0.001f);
    if (clamped == mCharacterSize)
        return;
    mCharacterSize = clamped;
    mGlyphsDirty = true;
}
f32 Text3D::characterSize() const
{
    return mCharacterSize;
}
void Text3D::setSpacing(f32 spacing)
{
    if (spacing == mSpacing)
        return;
    mSpacing = spacing;
    mGlyphsDirty = true;
}
f32 Text3D::spacing() const
{
    return mSpacing;
}
void Text3D::setColor(Color color)
{
    mColor = color;
}
Color Text3D::color() const
{
    return mColor;
}
void Text3D::setMode(BillboardMode mode)
{
    mMode = mode;
}
BillboardMode Text3D::mode() const
{
    return mMode;
}
void Text3D::setAlignment(TextAlign align)
{
    if (align == mAlign)
        return;
    mAlign = align;
    mGlyphsDirty = true;
}
TextAlign Text3D::alignment() const
{
    return mAlign;
}
void Text3D::setAdditive(bool additive)
{
    mAdditive = additive;
}
bool Text3D::additive() const
{
    return mAdditive;
}
void Text3D::setDepthTest(bool enabled)
{
    mDepthTest = enabled;
}
bool Text3D::depthTest() const
{
    return mDepthTest;
}

// Two passes over mText: the first measures each line's pen advance so
// alignment can offset it before a single glyph is placed, the second lays
// glyphs out left-to-right along the string's own right axis and steps one
// characterSize down its up axis per '\n'. Non-printable codes fall back to
// '?', same as BatchRenderer::drawText(); a space still advances the pen but
// leaves its glyph's UV rect zeroed so the render pass skips drawing it.
void Text3D::rebuildGlyphs()
{
    mGlyphsDirty = false;
    mGlyphs.clear();
    if (mText.empty())
        return;

    const f32 advance = mCharacterSize * mSpacing;

    std::vector<f32>& lineWidths = mLineWidths;
    lineWidths.clear();
    u32 lineLength = 0;
    for (usize i = 0; i <= mText.size(); ++i)
    {
        if (i == mText.size() || mText[i] == '\n')
        {
            lineWidths.push_back((f32)lineLength * advance);
            lineLength = 0;
        }
        else
        {
            ++lineLength;
        }
    }

    mGlyphs.reserve(mText.size());
    usize line = 0;
    f32 penX = alignOffset(mAlign, lineWidths[0]);
    f32 penY = 0.0f;
    for (char c : mText)
    {
        if (c == '\n')
        {
            ++line;
            penY -= mCharacterSize;
            penX = alignOffset(mAlign, lineWidths[line]);
            continue;
        }
        u8 code = (u8)c;
        if (code < 32 || code > 127)
            code = '?';
        MeshGlyph glyph;
        glyph.offset = glm::vec2(penX, penY);
        glyph.uvRect = fontGlyphUVRect(code);
        mGlyphs.push_back(glyph);
        penX += advance;
    }
}

void Text3D::onLateUpdate(f32 deltaTime)
{
    (void)deltaTime;
    if (mGlyphsDirty)
        rebuildGlyphs();
    if (mGlyphs.empty())
        return;

    MeshTextInstance instance;
    instance.position = owner()->globalPosition();
    instance.glyphSize = mCharacterSize;
    instance.color = mColor;
    instance.mode = mMode;
    instance.fixedRight = owner()->right();
    instance.fixedUp = owner()->up();
    instance.texture = TrailDraws().fontTexture();
    instance.blend =
        mAdditive ? BatchRenderer::BlendMode::Additive : BatchRenderer::BlendMode::Alpha;
    instance.depthTest = mDepthTest;
    instance.glyphs = mGlyphs.data();
    instance.glyphCount = (u32)mGlyphs.size();
    TrailDraws().submit(instance);
}

} // namespace Radion
