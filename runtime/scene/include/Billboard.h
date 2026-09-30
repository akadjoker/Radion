#ifndef RADION_BILLBOARD_H
#define RADION_BILLBOARD_H

#include "Color.h"
#include "Component.h"
#include "GPU.h"
#include "TrailRender.h"

#include "Math.h"

#include <string>

namespace Radion
{

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
    void setAdditive(bool additive);
    bool additive() const;
    void setBlendMode(BatchRenderer::BlendMode mode);
    BatchRenderer::BlendMode blendMode() const;
    void setDepthTest(bool enabled);
    bool depthTest() const;

    // Normalized [0,1] UV space: (u0, v0, width, height).
    void setUVRect(f32 u0, f32 v0, f32 width, f32 height);
    const Math::vec4& uvRect() const;
    void setAtlasCell(u32 cols, u32 rows, u32 col, u32 row);
    // Overrides setUVRect()/setAtlasCell() until one is called again.
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
