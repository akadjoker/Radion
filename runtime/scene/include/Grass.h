#ifndef RADION_GRASS_H
#define RADION_GRASS_H

#include "Component.h"
#include "GrassRender.h"

#include <string>
#include <vector>

namespace Radion
{

class VegetationGrid;

// Culling and drawing belong to the GPU grass pass.
class Grass final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Grass;

    bool loadAtlas(const std::string& filename);
    TextureHandle atlas() const;
    const std::string& atlasFile() const;

    // `size` multiplies tuft height; `weight` above 1 makes it come up more often.
    bool addRegion(u32 x, u32 y, u32 width, u32 height, f32 size = 1.0f, f32 weight = 1.0f);
    bool addRegion(const GrassAtlasRect& region, f32 weight = 1.0f);
    bool region(u32 index, GrassAtlasRect& region, f32& weight) const;
    bool setRegion(u32 index, const GrassAtlasRect& region, f32 weight);
    bool setRegion(u32 index, u32 x, u32 y, u32 width, u32 height, f32 size, f32 weight);

    bool regionPixels(u32 index, u32& x, u32& y, u32& width, u32& height, f32& size, f32& weight) const;
    u32 regionCount() const;
    void clearRegions();

    u32 paint(const Math::vec3& centre, f32 radius, u32 count);
    bool plant(const Math::vec3& position, const Math::vec3& normal, f32 scale = 1.0f);
    bool plant(const GrassClump& clump);
    bool clump(u32 index, GrassClump& clump) const;
    void clear();
    u32 count() const;

    // Width is a multiplier derived from height and region aspect.
    void setHeight(f32 metres);
    void setWidth(f32 multiplier);
    void setWind(f32 strength);
    void setAlphaCut(f32 cut);
    void setDrawDistance(f32 metres);

    f32 height() const;
    f32 width() const;
    f32 wind() const;
    f32 alphaCut() const;
    f32 drawDistance() const;

    void setStiffness(f32 stiffness);
    void setDrag(f32 drag);
    f32 stiffness() const;
    f32 drag() const;

    // Only the first eight are kept.
    void clearInfluencers();
    bool addInfluencer(const Math::vec3& centre, f32 radius, f32 force);
    u32 influencerCount() const;

    void setCameraBend(f32 amount);

    // Draws the field again: doubles fill rate.
    void setSoftFringe(bool enabled);
    void setSeed(u32 seed);
    u32 seed() const;
    f32 cameraBend() const;
    bool softFringe() const;

    void setGrid(VegetationGrid* grid);
    const VegetationGrid* grid() const;
    u32 paintFromGrid();

private:
    friend class GameObject;
    friend class Scene;

    Grass();
    void onDestroy() override;

    void submit(const Math::mat4& transform, f32 deltaTime);
    void rebuildWorld(const Math::mat4& transform);

    f32 random();
    static GrassAtlasRect computeRegion(u32 x, u32 y, u32 width, u32 height, f32 size,
                                        u32 atlasWidth, u32 atlasHeight);

    std::vector<GrassClump> mClumps;
    std::vector<GrassClump> mWorldClumps;
    Math::mat4 mTransform = Math::mat4(1.0f);
    std::vector<GrassAtlasRect> mRegions;
    std::vector<f32> mWeights;
    std::vector<GrassInfluencer> mInfluencers;
    VegetationGrid* mGrid = nullptr;
    TextureHandle mAtlas;
    std::string mAtlasFile;
    u32 mAtlasWidth = 0;
    u32 mAtlasHeight = 0;
    f32 mTotalWeight = 0.0f;
    f32 mHeight = 1.2f;
    f32 mWidth = 0.7f;
    f32 mWind = 1.0f;
    f32 mAlphaCut = 0.35f;
    f32 mCameraBend = 0.55f;
    f32 mDrawDistance = 300.0f;
    f32 mStiffness = 12.0f;
    f32 mDrag = 0.12f;
    bool mSoftFringe = true;
    u64 mRevision = 0;
    bool mDirty = true;
    u32 mRandomState = 12345u;
};

} // namespace Radion

#endif // RADION_GRASS_H
