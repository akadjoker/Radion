#ifndef RADION_BILLBOARD_H
#define RADION_BILLBOARD_H

#include "Color.h"
#include "Component.h"
#include "GPU.h"
#include "TrailRender.h" // BillboardMode

#include "Math.h"

#include <string>

namespace Radion
{

// A single camera-facing quad - a flame, a marker, a static poster - that
// doesn't fit ParticleEffect's emission/lifetime model or Grass/Forest's
// mass scattering. Renders through TrailDraws(), the same world-space
// triangle queue RibbonTrail and Text3D submit to.
class Billboard final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Billboard;

    void setSize(f32 width, f32 height);
    void setSize(const Math::vec2& size);
    const Math::vec2& size() const;
    void setColor(Color color);
    Color color() const;
    void setMode(BillboardMode mode);
    BillboardMode mode() const;
    void setTexture(TextureHandle texture);
    TextureHandle texture() const;
    void setTextureFile(const std::string& file);
    const std::string& textureFile() const;
    // Additive/Alpha only - kept for existing callers, expressed in terms of
    // setBlendMode() below (true picks Additive, false picks Alpha).
    void setAdditive(bool additive);
    bool additive() const;
    // The full range Batch.h's rasterizer actually supports (Alpha,
    // Additive, Multiplied, AddColors, SubtractColors) - a shadow blob wants
    // Multiplied, a glow wants Additive, neither is reachable through
    // setAdditive() alone.
    void setBlendMode(BatchRenderer::BlendMode mode);
    BatchRenderer::BlendMode blendMode() const;
    void setDepthTest(bool enabled);
    bool depthTest() const;

    // Atlas rect in normalized [0,1] UV space: (u0, v0, width, height).
    // Default (0,0,1,1) samples the whole texture. Turns off flip-book
    // animation if it was on.
    void setUVRect(f32 u0, f32 v0, f32 width, f32 height);
    const Math::vec4& uvRect() const;
    // Convenience: an NxM grid, pick a fixed cell (col,row), 0-based and
    // clamped. Turns off flip-book animation if it was on.
    void setAtlasCell(u32 cols, u32 rows, u32 col, u32 row);
    // Flip-book animation: cycles every cell of an NxM atlas at `fps`
    // frames/second, looping forever. Overrides setUVRect()/setAtlasCell()
    // until one of those is called again.
    void setAnimatedAtlas(u32 cols, u32 rows, f32 fps);
    bool animated() const;
    u32 atlasCols() const;
    u32 atlasRows() const;
    f32 atlasFps() const;

private:
    friend class GameObject;

    Billboard();
    void onLateUpdate(f32 deltaTime) override;
    Math::vec4 currentUVRect() const;

    Math::vec2 mSize{1.0f, 1.0f};
    Color mColor;
    Math::vec4 mUVRect{0.0f, 0.0f, 1.0f, 1.0f};
    BillboardMode mMode = BillboardMode::Free;
    TextureHandle mTexture;
    std::string mTextureFile;
    BatchRenderer::BlendMode mBlend = BatchRenderer::BlendMode::Additive;
    bool mDepthTest = true;
    bool mAnimated = false;
    u32 mAtlasCols = 1;
    u32 mAtlasRows = 1;
    f32 mAtlasFps = 12.0f;
    f32 mAtlasTime = 0.0f;
};

} // namespace Radion

#endif // RADION_BILLBOARD_H
