#ifndef RADION_TEXT3D_H
#define RADION_TEXT3D_H

#include "Color.h"
#include "Component.h"
#include "TrailRender.h" // BillboardMode, MeshGlyph

#include <string>
#include <vector>

namespace Radion
{

enum class TextAlign : u8
{
    Left,
    Center,
    Right
};

// A string of the engine's embedded 8x8 font rendered as world-space quads,
// one per glyph, camera-facing the same way Billboard is. Floats over a
// GameObject the way a name tag or a damage number would.
class Text3D final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Text3D;

    void setText(const std::string& text);
    const std::string& text() const;
    void setCharacterSize(f32 size); // world-space height of one glyph
    f32 characterSize() const;
    void setSpacing(f32 spacing); // multiplies the horizontal advance, 1.0 = glyphs touch
    f32 spacing() const;
    void setColor(Color color);
    Color color() const;
    void setMode(BillboardMode mode);
    BillboardMode mode() const;
    void setAlignment(TextAlign align);
    TextAlign alignment() const;
    void setAdditive(bool additive);
    bool additive() const;
    void setDepthTest(bool enabled);
    bool depthTest() const;

private:
    friend class GameObject;

    Text3D();
    void onLateUpdate(f32 deltaTime) override;
    void rebuildGlyphs();

    std::string mText;
    f32 mCharacterSize = 0.5f;
    f32 mSpacing = 1.0f;
    Color mColor;
    BillboardMode mMode = BillboardMode::Free;
    TextAlign mAlign = TextAlign::Left;
    bool mAdditive = false;
    bool mDepthTest = true;
    // The glyph layout depends on the text and its metrics alone, never on
    // the transform or the camera, so it survives untouched across frames.
    bool mGlyphsDirty = true;
    std::vector<MeshGlyph> mGlyphs;
    std::vector<f32> mLineWidths;
};

} // namespace Radion

#endif // RADION_TEXT3D_H
