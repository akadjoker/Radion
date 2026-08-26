#ifndef RADION_FOREST_H
#define RADION_FOREST_H

#include "Component.h"
#include "Material.h"
#include "Mesh.h"
#include "ProcTree.h"
#include "TreeRender.h"

#include <string>
#include <vector>

namespace Radion
{

class RenderList;
class VegetationGrid;

class Forest final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Forest;

    // Generates the mesh and scales it so the tree stands `height` metres
    // tall, whatever the parameters happened to produce. `twigTexture` picks
    // which entry set by setTwigTextures() this species' leaf cards sample -
    // out of range falls back to entry 0. Returns the species index, or -1 if
    // the generator gave nothing usable.
    s32 addSpecies(const TreeParams& params, f32 height = 14.0f, f32 weight = 1.0f,
                   u32 twigTexture = 0);
    bool rebuildSpecies(u32 species, const TreeParams& params, f32 height);
    u32 speciesCount() const;
    const TreeParams& speciesParams(u32 species) const;
    f32 speciesHeight(u32 species) const;
    f32 speciesWeight(u32 species) const;
    bool setSpeciesWeight(u32 species, f32 weight);
    u32 speciesTwigTexture(u32 species) const;
    MeshHandle speciesMesh(u32 species) const;

    bool setSpeciesTwigTexture(u32 species, u32 twigTexture);

    // Slot 0 is the bark, slot 1 the twig cards.
    Material& material(u32 species, u32 slot);
    const Material& material(u32 species, u32 slot) const;
    const std::string& barkAlbedoPath() const;
    const std::string& barkNormalPath() const;
    const std::vector<std::string>& twigTexturePaths() const;

    void setBarkTexture(const std::string& albedo, const std::string& normalMap = std::string());

    // Twig cards vary by species; each entry here is what a species'
    // `twigTexture` index into addSpecies() refers to.
    void setTwigTextures(const std::vector<std::string>& albedoPaths);
    void addTwigTexture(const std::string& albedoPath);
    bool removeTwigTexture(u32 index);

    // Trees redraw once per shadow cascade, so turning this off is the knob
    // for a scene where that cost outweighs the shadows it buys.
    void setCastShadows(bool enabled);
    bool castsShadows() const;

    // Leaf sway. Only the leaves move - the trunk is rigid and its base is in
    // the ground, so swaying it would give a rubber tree.
    void setWind(f32 strength);
    f32 wind() const;
    void setBarkBumpForce(f32 force);
    f32 barkBumpForce() const;

    // Where the leaf cards are cut. Low lets the thin tips through but also
    // the texture's halo; high eats the tips and the crown thins out.
    void setAlphaCut(f32 cut);
    f32 alphaCut() const;

    // Beyond swapDistance a tree becomes a photographed quad. The band is the
    // overlap where both draw and the impostor fades in over the mesh - zero
    // makes the handover a pop.
    void setImpostorsEnabled(bool enabled);
    bool impostorsEnabled() const;
    void setSwapDistance(f32 metres);
    f32 swapDistance() const;
    void setSwapBand(f32 metres);
    f32 swapBand() const;
    // The impostor quad's width over its height. A tree is taller than it is
    // wide, so a square quad leaves the crown floating in empty space; too
    // narrow and the crown is clipped at the handover.
    void setImpostorWidth(f32 ratio);
    f32 impostorWidth() const;
    u32 impostorsVisible() const;

    // Scatters `count` trees inside a disc, in the owner's local space, flat
    // on the centre's own y plane. Following terrain height is the caller's
    // job - plant() takes whatever position it is given.
    u32 paint(const Math::Vec3& centre, f32 radius, u32 count);
    bool plant(const Math::Vec3& position, u32 species, f32 scale = 1.0f, f32 yawDegrees = 0.0f);
    void clear();

    u32 count() const;
    u32 instanceCount() const;
    Math::Vec3 instancePosition(u32 index) const;
    f32 instanceScale(u32 index) const;
    f32 instanceYaw(u32 index) const;
    u32 instanceSpecies(u32 index) const;
    u32 visibleCount() const;

    void setDrawDistance(f32 metres);
    f32 drawDistance() const;
    void setScaleRange(f32 minimum, f32 maximum);
    f32 scaleMinimum() const;
    f32 scaleMaximum() const;
    void setSeed(u32 seed);
    u32 seed() const;

    // Optional occupancy grid. When set, paintFromGrid() plants one tree per
    // cell marked as Tree using the species/scale/yaw stored in the cell.
    void setGrid(VegetationGrid* grid);
    const VegetationGrid* grid() const;
    u32 paintFromGrid();

private:
    friend class GameObject;
    friend class Scene;

    struct Species
    {
        TreeParams params;
        MeshHandle mesh;
        Material materials[2];
        f32 weight = 1.0f;
        f32 radius = 1.0f;
        f32 height = 14.0f;
        u32 twigTexture = 0;

        // Rebuilt each frame in submitCamera(): world-space instances for the
        // tree pass, already distance-culled. Kept per species so the vector's
        // storage survives between frames instead of reallocating.
        std::vector<TreeInstanceData> batch;
        std::vector<TreeInstanceData> impostorBatch;

        // Bumped by buildSpecies(): the mesh changed, so whatever photographs
        // the tree pass holds of it are stale.
        u32 impostorRevision = 0;
    };

    struct Instance
    {
        Math::Vec3 position = Math::Vec3(0.0f);
        f32 scale = 1.0f;
        f32 yaw = 0.0f;
        u32 species = 0;
    };

    Forest();
    void onDestroy() override;

    // Drops what is beyond the draw distance and hands the rest to the list,
    // which does the frustum test itself. Grouped by species so the sort has
    // less to move.
    void submit(RenderList& list, const Math::Mat4& transform, const Math::Vec3& cameraPosition);
    void submitCamera(const Math::Mat4& transform, const Math::Vec3& cameraPosition);
    void submitShadow(RenderList& list, const Math::Mat4& transform,
                      const Math::Vec3& cameraPosition);

    f32 random();
    bool buildSpecies(Species& species, const TreeParams& params, f32 height);
    u32 pickSpecies();

    std::vector<Species> mSpecies;
    std::vector<Instance> mInstances;
    VegetationGrid* mGrid = nullptr;
    std::string mBarkAlbedoPath;
    std::string mBarkNormalPath;
    std::vector<std::string> mTwigAlbedoPaths;
    bool mCastShadows = true;
    f32 mWind = 1.0f;
    f32 mBarkBumpForce = 1.0f;
    f32 mAlphaCut = 0.5f;
    bool mImpostorsEnabled = true;
    f32 mSwapDistance = 120.0f;
    f32 mSwapBand = 12.0f;
    f32 mImpostorWidth = 0.85f;
    u32 mImpostorsVisible = 0;
    f32 mDrawDistance = 500.0f;
    f32 mScaleMinimum = 0.8f;
    f32 mScaleMaximum = 1.25f;
    f32 mTotalWeight = 0.0f;
    u32 mVisible = 0;
    u32 mRandomState = 777u;
};

} // namespace Radion

#endif // RADION_FOREST_H
