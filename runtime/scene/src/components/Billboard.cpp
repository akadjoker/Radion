#include "PCH.h"

#include "Billboard.h"

#include "GameObject.h"
#include "AssetManager.h"

namespace Radion
{

Billboard::Billboard() : Component(Type, ComponentEventLateUpdate)
{
}

void Billboard::setSize(f32 width, f32 height)
{
    mSize = Math::vec2(width, height);
}
void Billboard::setSize(const Math::vec2& size)
{
    mSize = size;
}
const Math::vec2& Billboard::size() const
{
    return mSize;
}
void Billboard::setColor(Color color)
{
    mColor = color;
}
Color Billboard::color() const
{
    return mColor;
}
void Billboard::setMode(BillboardMode mode)
{
    mMode = mode;
}
BillboardMode Billboard::mode() const
{
    return mMode;
}
void Billboard::setTexture(TextureHandle texture)
{
    mTexture = texture;
}
void Billboard::setTextureFile(const std::string& file)
{
    mTextureFile = file;
    mTexture = file.empty() ? TextureHandle() : Assets().loadTexture(file, ColorSpace::sRGB);
}
const std::string& Billboard::textureFile() const
{
    return mTextureFile;
}
TextureHandle Billboard::texture() const
{
    return mTexture;
}
void Billboard::setAdditive(bool additive)
{
    mBlend = additive ? BatchRenderer::BlendMode::Additive : BatchRenderer::BlendMode::Alpha;
}
bool Billboard::additive() const
{
    return mBlend == BatchRenderer::BlendMode::Additive;
}
void Billboard::setBlendMode(BatchRenderer::BlendMode mode)
{
    mBlend = mode;
}
BatchRenderer::BlendMode Billboard::blendMode() const
{
    return mBlend;
}
void Billboard::setDepthTest(bool enabled)
{
    mDepthTest = enabled;
}
bool Billboard::depthTest() const
{
    return mDepthTest;
}
void Billboard::setUVRect(f32 u0, f32 v0, f32 width, f32 height)
{
    mUVRect = Math::vec4(u0, v0, width, height);
    mAnimated = false;
}
const Math::vec4& Billboard::uvRect() const
{
    return mUVRect;
}
void Billboard::setAtlasCell(u32 cols, u32 rows, u32 col, u32 row)
{
    cols = cols > 0 ? cols : 1;
    rows = rows > 0 ? rows : 1;
    col = Math::min(col, cols - 1);
    row = Math::min(row, rows - 1);
    const f32 w = 1.0f / (f32)cols, h = 1.0f / (f32)rows;
    mUVRect = Math::vec4((f32)col * w, (f32)row * h, w, h);
    mAnimated = false;
}
void Billboard::setAnimatedAtlas(u32 cols, u32 rows, f32 fps)
{
    mAtlasCols = cols > 0 ? cols : 1;
    mAtlasRows = rows > 0 ? rows : 1;
    mAtlasFps = fps;
    mAnimated = true;
}
bool Billboard::animated() const
{
    return mAnimated;
}
u32 Billboard::atlasCols() const
{
    return mAtlasCols;
}
u32 Billboard::atlasRows() const
{
    return mAtlasRows;
}
f32 Billboard::atlasFps() const
{
    return mAtlasFps;
}

// Normalized (u0, v0, width, height) for whichever atlas cell mAtlasTime
// lands on - row-major order, looping once it runs past the last cell.
Math::vec4 Billboard::currentUVRect() const
{
    if (!mAnimated)
        return mUVRect;
    const u32 total = mAtlasCols * mAtlasRows;
    u32 frame = total > 0 ? (u32)(mAtlasTime * mAtlasFps) % total : 0;
    const u32 col = frame % mAtlasCols, row = frame / mAtlasCols;
    const f32 w = 1.0f / (f32)mAtlasCols, h = 1.0f / (f32)mAtlasRows;
    return Math::vec4((f32)col * w, (f32)row * h, w, h);
}

void Billboard::onLateUpdate(f32 deltaTime)
{
    if (mAnimated)
        mAtlasTime += deltaTime;

    BillboardInstance instance;
    instance.position = owner()->globalPosition();
    instance.size = mSize;
    instance.uvRect = currentUVRect();
    instance.color = mColor;
    instance.mode = mMode;
    instance.fixedRight = owner()->right();
    instance.fixedUp = owner()->up();
    instance.texture = mTexture;
    instance.blend = mBlend;
    instance.depthTest = mDepthTest;
    TrailDraws().submit(instance);
}

} // namespace Radion
