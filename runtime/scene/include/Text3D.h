#ifndef RADION_TEXT3D_H
#define RADION_TEXT3D_H

#include "Color.h"
#include "Component.h"
#include "TrailRender.h"

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

// Embedded 8x8 font as world-space camera-facing quads, one per glyph.
class Text3D final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Text3D;

    void setText(const std::string& text);
    const std::string& text() const;
    void setCharacterSize(f32 size);
    f32 characterSize() const;
    void setSpacing(f32 spacing);
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
    // Layout depends on text and metrics alone, never on transform or camera.
    bool mGlyphsDirty = true;
    std::vector<MeshGlyph> mGlyphs;
    std::vector<f32> mLineWidths;
};

} // namespace Radion

#endif // RADION_TEXT3D_H
