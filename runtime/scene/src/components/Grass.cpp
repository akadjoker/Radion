#include "PCH.h"

#include "Grass.h"

#include "AssetManager.h"
#include "GameObject.h"
#include "VegetationGrid.h"

namespace Radion
{

Grass::Grass() : Component(Type)
{
}

void Grass::onDestroy()
{
    mClumps.clear();
    mWorldClumps.clear();
    mRegions.clear();
    mWeights.clear();
    mInfluencers.clear();
    mTotalWeight = 0.0f;
}

f32 Grass::random()
{
    mRandomState = mRandomState * 1664525u + 1013904223u;
    return static_cast<f32>(mRandomState >> 8) / 16777216.0f;
}

bool Grass::loadAtlas(const std::string& filename)
{
    // Four levels take a ~400px region down to ~25px, which is already under a
    // pixel on screen at the distance the grass fades out. Past that the box
    // filter is averaging one plant into the next one along in the atlas.
    const TextureHandle atlas =
        Assets().loadTexture(filename, Material::colorSpaceFor(SlotAlbedo), true, 4);
    if (!atlas.valid())
        return false;

    TextureDesc desc;
    if (!GPU::getSingleton().textureInfo(atlas, desc) || desc.width == 0 || desc.height == 0)
        return false;

    mAtlas = atlas;
    mAtlasFile = filename;
    mAtlasWidth = desc.width;
    mAtlasHeight = desc.height;
    return true;
}

TextureHandle Grass::atlas() const
{
    return mAtlas;
}

const std::string& Grass::atlasFile() const
{
    return mAtlasFile;
}

GrassAtlasRect Grass::computeRegion(u32 x, u32 y, u32 width, u32 height, f32 size,
                                    u32 atlasWidth, u32 atlasHeight)
{
    GrassAtlasRect region;
    region.texMulAdd =
        Math::vec4(static_cast<f32>(width) / static_cast<f32>(atlasWidth),
                  static_cast<f32>(height) / static_cast<f32>(atlasHeight),
                  static_cast<f32>(x) / static_cast<f32>(atlasWidth),
                  static_cast<f32>(y) / static_cast<f32>(atlasHeight));
    region.sizeAspect =
        Math::vec4(size, static_cast<f32>(width) / static_cast<f32>(height), 0.0f, 0.0f);
    return region;
}

bool Grass::addRegion(u32 x, u32 y, u32 width, u32 height, f32 size, f32 weight)
{
    if (mAtlasWidth == 0 || mAtlasHeight == 0 || width == 0 || height == 0)
        return false;
    if (x + width > mAtlasWidth || y + height > mAtlasHeight)
        return false;

    mRegions.push_back(computeRegion(x, y, width, height, size, mAtlasWidth, mAtlasHeight));
    mWeights.push_back(weight > 0.0f ? weight : 0.0f);
    mTotalWeight += mWeights.back();
    return true;
}

bool Grass::addRegion(const GrassAtlasRect& region, f32 weight)
{
    if (region.texMulAdd.x <= 0.0f || region.texMulAdd.y <= 0.0f ||
        region.sizeAspect.x <= 0.0f || region.sizeAspect.y <= 0.0f)
        return false;
    mRegions.push_back(region);
    mWeights.push_back(weight > 0.0f ? weight : 0.0f);
    mTotalWeight += mWeights.back();
    return true;
}

bool Grass::region(u32 index, GrassAtlasRect& region, f32& weight) const
{
    if (index >= mRegions.size())
        return false;
    region = mRegions[index];
    weight = mWeights[index];
    return true;
}

bool Grass::setRegion(u32 index, const GrassAtlasRect& region, f32 weight)
{
    if (index >= mRegions.size() || region.texMulAdd.x <= 0.0f || region.texMulAdd.y <= 0.0f ||
        region.sizeAspect.x <= 0.0f || region.sizeAspect.y <= 0.0f)
        return false;
    mTotalWeight -= mWeights[index];
    mRegions[index] = region;
    mWeights[index] = Math::max(0.0f, weight);
    mTotalWeight += mWeights[index];
    mDirty = true;
    return true;
}

bool Grass::setRegion(u32 index, u32 x, u32 y, u32 width, u32 height, f32 size, f32 weight)
{
    if (index >= mRegions.size() || mAtlasWidth == 0 || mAtlasHeight == 0 ||
        width == 0 || height == 0)
        return false;
    if (x + width > mAtlasWidth || y + height > mAtlasHeight)
        return false;

    mTotalWeight -= mWeights[index];
    mRegions[index] = computeRegion(x, y, width, height, size, mAtlasWidth, mAtlasHeight);
    mWeights[index] = Math::max(0.0f, weight);
    mTotalWeight += mWeights[index];
    mDirty = true;
    return true;
}

bool Grass::regionPixels(u32 index, u32& x, u32& y, u32& width, u32& height, f32& size,
                         f32& weight) const
{
    if (index >= mRegions.size() || mAtlasWidth == 0 || mAtlasHeight == 0)
        return false;
    const GrassAtlasRect& region = mRegions[index];
    width = static_cast<u32>(std::lround(region.texMulAdd.x * static_cast<f32>(mAtlasWidth)));
    height = static_cast<u32>(std::lround(region.texMulAdd.y * static_cast<f32>(mAtlasHeight)));
    x = static_cast<u32>(std::lround(region.texMulAdd.z * static_cast<f32>(mAtlasWidth)));
    y = static_cast<u32>(std::lround(region.texMulAdd.w * static_cast<f32>(mAtlasHeight)));
    size = region.sizeAspect.x;
    weight = mWeights[index];
    return true;
}

u32 Grass::regionCount() const
{
    return static_cast<u32>(mRegions.size());
}

void Grass::clearRegions()
{
    mRegions.clear();
    mWeights.clear();
    mClumps.clear();
    mTotalWeight = 0.0f;
    mDirty = true;
}

bool Grass::plant(const Math::vec3& position, const Math::vec3& normal, f32 scale)
{
    if (mRegions.empty() || scale <= 0.0f)
        return false;

    // Weighted pick, so a lawn can be mostly dense tufts with the odd tall
    // stalk instead of an even mix of everything in the atlas.
    u32 region = 0;
    if (mTotalWeight > 0.0f)
    {
        f32 pick = random() * mTotalWeight;
        for (usize i = 0; i < mWeights.size(); ++i)
        {
            pick -= mWeights[i];
            if (pick <= 0.0f)
            {
                region = static_cast<u32>(i);
                break;
            }
        }
    }

    GrassClump clump;
    clump.positionScale = Math::vec4(position, scale);
    clump.normalRotation = Math::vec4(Math::normalize(normal), random() * 2.0f * Math::pi<f32>());
    clump.rect = Math::vec4(static_cast<f32>(region), 0.0f, 0.0f, 0.0f);
    mClumps.push_back(clump);
    mDirty = true;
    return true;
}

bool Grass::plant(const GrassClump& clump)
{
    const u32 region = static_cast<u32>(clump.rect.x);
    if (region >= mRegions.size() || clump.positionScale.w <= 0.0f ||
        Math::length(Math::vec3(clump.normalRotation)) <= 0.000001f)
        return false;
    mClumps.push_back(clump);
    mDirty = true;
    return true;
}

bool Grass::clump(u32 index, GrassClump& clump) const
{
    if (index >= mClumps.size())
        return false;
    clump = mClumps[index];
    return true;
}

u32 Grass::paint(const Math::vec3& centre, f32 radius, u32 count)
{
    if (mRegions.empty() || radius <= 0.0f)
        return 0;

    u32 planted = 0;
    for (u32 i = 0; i < count; ++i)
    {
        // Square root of the random radius: area grows with r², so a uniform
        // one would pile every tuft in the middle.
        const f32 angle = random() * 2.0f * Math::pi<f32>();
        const f32 distance = std::sqrt(random()) * radius;
        const Math::vec3 position =
            centre + Math::vec3(std::cos(angle) * distance, 0.0f, std::sin(angle) * distance);
        const Math::vec3 normal(0.0f, 1.0f, 0.0f);

        const f32 scale = 0.75f + random() * 0.5f;
        if (plant(position, normal, scale))
            ++planted;
    }
    return planted;
}

void Grass::clear()
{
    mClumps.clear();
    mDirty = true;
}

u32 Grass::count() const
{
    return static_cast<u32>(mClumps.size());
}

void Grass::setHeight(f32 metres)
{
    mHeight = metres > 0.0f ? metres : 0.0f;
}

void Grass::setWidth(f32 multiplier)
{
    mWidth = multiplier > 0.0f ? multiplier : 0.0f;
}

void Grass::setWind(f32 strength)
{
    mWind = strength;
}

void Grass::setAlphaCut(f32 cut)
{
    mAlphaCut = Math::clamp(cut, 0.0f, 1.0f);
}

void Grass::setDrawDistance(f32 metres)
{
    mDrawDistance = metres > 0.0f ? metres : 0.0f;
}

void Grass::setCameraBend(f32 amount)
{
    mCameraBend = amount;
}

void Grass::setStiffness(f32 stiffness)
{
    mStiffness = stiffness > 0.0f ? stiffness : 0.0f;
}

void Grass::setDrag(f32 drag)
{
    mDrag = Math::clamp(drag, 0.0f, 1.0f);
}

f32 Grass::height() const
{
    return mHeight;
}

f32 Grass::width() const
{
    return mWidth;
}

f32 Grass::wind() const
{
    return mWind;
}

f32 Grass::alphaCut() const
{
    return mAlphaCut;
}

f32 Grass::drawDistance() const
{
    return mDrawDistance;
}

f32 Grass::stiffness() const
{
    return mStiffness;
}

f32 Grass::drag() const
{
    return mDrag;
}

f32 Grass::cameraBend() const
{
    return mCameraBend;
}

bool Grass::softFringe() const
{
    return mSoftFringe;
}

void Grass::clearInfluencers()
{
    mInfluencers.clear();
}

bool Grass::addInfluencer(const Math::vec3& centre, f32 radius, f32 force)
{
    if (mInfluencers.size() >= kGrassMaxInfluencers || radius <= 0.0f)
        return false;
    mInfluencers.push_back({centre, radius, force});
    return true;
}

u32 Grass::influencerCount() const
{
    return static_cast<u32>(mInfluencers.size());
}

void Grass::setSoftFringe(bool enabled)
{
    mSoftFringe = enabled;
}

void Grass::setSeed(u32 seed)
{
    mRandomState = seed;
}

u32 Grass::seed() const
{
    return mRandomState;
}

void Grass::setGrid(VegetationGrid* grid)
{
    mGrid = grid;
}

const VegetationGrid* Grass::grid() const
{
    return mGrid;
}

u32 Grass::paintFromGrid()
{
    if (!mGrid || mGrid->cells().empty() || mRegions.empty())
        return 0;

    u32 planted = 0;
    for (const VegetationGrid::Cell& cell : mGrid->cells())
    {
        if (cell.occupant != VegetationGrid::Occupant::Grass)
            continue;

        if (plant(cell.position, cell.normal, cell.scale))
            ++planted;
    }
    return planted;
}

// Tufts are authored where they were painted, in the owner's space, but the
// pass culls against world-space planes and the shader never sees a model
// matrix. Baking the transform in is the one place that gap is closed, and it
// only runs when the field or the object actually moved.
void Grass::rebuildWorld(const Math::mat4& transform)
{
    mWorldClumps.resize(mClumps.size());
    const Math::mat3 rotation(transform);
    for (usize i = 0; i < mClumps.size(); ++i)
    {
        const GrassClump& clump = mClumps[i];
        mWorldClumps[i].positionScale =
            Math::vec4(Math::vec3(transform * Math::vec4(Math::vec3(clump.positionScale), 1.0f)),
                      clump.positionScale.w);
        mWorldClumps[i].normalRotation = Math::vec4(
            Math::normalize(rotation * Math::vec3(clump.normalRotation)), clump.normalRotation.w);
        mWorldClumps[i].rect = clump.rect;
    }

    mTransform = transform;
    mDirty = false;
    ++mRevision;
}

void Grass::submit(const Math::mat4& transform, f32 deltaTime)
{
    if (mClumps.empty() || mRegions.empty() || !mAtlas.valid())
        return;

    if (mDirty || transform != mTransform)
        rebuildWorld(transform);

    GrassDrawCommand command;
    command.clumps = mWorldClumps.data();
    command.clumpCount = static_cast<u32>(mWorldClumps.size());
    command.rects = mRegions.data();
    command.rectCount = static_cast<u32>(mRegions.size());
    command.revision = mRevision;
    command.atlas = mAtlas;
    command.height = mHeight;
    command.width = mWidth;
    command.wind = mWind;
    command.alphaCut = mAlphaCut;
    command.cameraBend = mCameraBend;
    command.drawDistance = mDrawDistance;
    command.stiffness = mStiffness;
    command.drag = mDrag;
    command.softFringe = mSoftFringe;
    command.deltaTime = deltaTime;
    command.influencers = mInfluencers.empty() ? nullptr : mInfluencers.data();
    command.influencerCount = static_cast<u32>(mInfluencers.size());
    GrassDraws().submit(command);
}

} // namespace Radion
