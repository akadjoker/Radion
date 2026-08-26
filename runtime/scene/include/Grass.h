#ifndef RADION_GRASS_H
#define RADION_GRASS_H

#include "Component.h"
#include "GrassRender.h"

#include <string>
#include <vector>

namespace Radion
{

class VegetationGrid;

// A field of grass tufts. Three crossed quads per tuft, textured from an
// atlas: a single blade never reads as grass, and no amount of geometry
// replaces a photograph with a cut alpha.
//
// The component owns the tufts and nothing else. Culling and drawing belong to
// the grass pass, which does both on the GPU - the count of what survives is
// written into an indirect buffer and never comes back, so a field of two
// hundred thousand costs the same on the CPU as a field of one.
class Grass final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Grass;

    // The vegetation atlas: one image holding several plants, carved up by
    // addRegion().
    bool loadAtlas(const std::string& filename);
    TextureHandle atlas() const;
    const std::string& atlasFile() const;

    // A region in atlas pixels. `size` multiplies the tuft's height, so one
    // atlas can hold both a low bush and a tall stalk; `weight` above 1 makes
    // it come up more often under the brush.
    bool addRegion(u32 x, u32 y, u32 width, u32 height, f32 size = 1.0f, f32 weight = 1.0f);
    bool addRegion(const GrassAtlasRect& region, f32 weight = 1.0f);
    bool region(u32 index, GrassAtlasRect& region, f32& weight) const;
    bool setRegion(u32 index, const GrassAtlasRect& region, f32 weight);
    bool setRegion(u32 index, u32 x, u32 y, u32 width, u32 height, f32 size, f32 weight);

    // Same rect back out in atlas pixels, the shape it went in as - an editor
    // panel showing UV fractions to someone who is reading pixel coordinates
    // off an image is asking them to do the division themselves.
    bool regionPixels(u32 index, u32& x, u32& y, u32& width, u32& height, f32& size, f32& weight) const;
    u32 regionCount() const;
    void clearRegions();

    // Scatters `count` tufts in a disc on the centre's own y plane, upright.
    // Following terrain height and slope is the caller's job - plant() takes
    // whatever position and normal it is given.
    u32 paint(const glm::vec3& centre, f32 radius, u32 count);
    bool plant(const glm::vec3& position, const glm::vec3& normal, f32 scale = 1.0f);
    bool plant(const GrassClump& clump);
    bool clump(u32 index, GrassClump& clump) const;
    void clear();
    u32 count() const;

    // Height in metres, before the atlas region's own size multiplier. Width
    // is a multiplier, not a length: the shader takes it from the height and
    // the region's aspect, so a thin stalk and a wide bush keep the shapes
    // they have in the texture.
    void setHeight(f32 metres);
    void setWidth(f32 multiplier);
    void setWind(f32 strength);
    void setAlphaCut(f32 cut);
    void setDrawDistance(f32 metres);

    // Read back what the setters above clamped. An editor panel needs these:
    // it has to show the value that is actually in effect, not the one it last
    // asked for.
    f32 height() const;
    f32 width() const;
    f32 wind() const;
    f32 alphaCut() const;
    f32 drawDistance() const;

    // Verlet parameters. Stiffness pulls the tip back to rest, drag is how
    // much of the inertia is lost per step - together they are how heavy the
    // grass feels when something pushes through it.
    void setStiffness(f32 stiffness);
    void setDrag(f32 drag);
    f32 stiffness() const;
    f32 drag() const;

    // Spheres that push the grass aside. Cleared and re-added each frame by
    // whatever is moving through the field; only the first eight are kept.
    void clearInfluencers();
    bool addInfluencer(const glm::vec3& centre, f32 radius, f32 force);
    u32 influencerCount() const;

    // How far the tip leans towards the camera's up. Zero shows what it is
    // for: looking down from above, upright quads go edge-on and the field
    // thins away to nothing.
    void setCameraBend(f32 amount);

    // Second pass that blends the leaf outline instead of cutting it. It draws
    // the whole field again, so it doubles the fill rate - the first thing to
    // turn off when the frame is spent on overdraw.
    void setSoftFringe(bool enabled);
    void setSeed(u32 seed);
    u32 seed() const;
    f32 cameraBend() const;
    bool softFringe() const;

    // Optional occupancy grid. When set, paintFromGrid() plants one tuft per
    // cell marked as Grass. The grid keeps the positions/normals in local space,
    // just like the clumps stored here.
    void setGrid(VegetationGrid* grid);
    const VegetationGrid* grid() const;
    u32 paintFromGrid();

private:
    friend class GameObject;
    friend class Scene;

    Grass();
    void onDestroy() override;

    // Hands the field to the grass pass for this frame. Nothing is copied -
    // the pass reads these arrays and uploads on a revision change.
    void submit(const glm::mat4& transform, f32 deltaTime);
    void rebuildWorld(const glm::mat4& transform);

    f32 random();
    static GrassAtlasRect computeRegion(u32 x, u32 y, u32 width, u32 height, f32 size,
                                        u32 atlasWidth, u32 atlasHeight);

    std::vector<GrassClump> mClumps;      // as painted, in the owner's space
    std::vector<GrassClump> mWorldClumps; // what the pass reads
    glm::mat4 mTransform = glm::mat4(1.0f);
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
