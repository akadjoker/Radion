#ifndef RADION_TERRAIN_H
#define RADION_TERRAIN_H

#include "Component.h"
#include "Material.h"
#include "Math.h"
#include "Mesh.h"
#include "Pixmap.h"

#include <vector>
#include <string>
#include <memory>

namespace Radion
{

class Scene;
class RenderList;
class Grass;
class Forest;


class Terrain final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Terrain;

    // Fixed, not a load() parameter: the shared index buffer is built once
    // for this width. "+3": index 0 and ChunkWidth-1 are the border ring,
    // always full density; 1..ChunkWidth-2 is the interior, walked with a
    // step of (1 << lod).
    static constexpr u32 ChunkWidth = 64 + 3;
    static constexpr u32 ChunkSpan = ChunkWidth - 1; // vertices one chunk covers before the next starts
    static constexpr u32 VertexCount = ChunkWidth * ChunkWidth;

    struct VegetationSettings
    {
        // One candidate per spacing-sized cell, jittered inside it. Density
        // rejects candidates without changing the stable layout/seed.
        f32 spacing = 1.5f;
        f32 density = 0.75f;
        f32 jitter = 0.85f;
        f32 minimumHeight = -1000000.0f;
        f32 maximumHeight = 1000000.0f;
        f32 maximumSlopeDegrees = 34.0f;
        f32 minimumScale = 0.75f;
        f32 maximumScale = 1.25f;
        u32 seed = 3926;
        u32 maximumInstances = 250000;
    };

    // One RGBA control map shared by all terrain vegetation. The channels are
    // deliberately semantic instead of tied to a renderer: grass and trees
    // use R/A today, while G/B are already available for flowers and bushes.
    enum class VegetationChannel : u8
    {
        Grass = 0,
        Flowers = 1,
        Bushes = 2,
        Trees = 3
    };

    enum class SurfaceLayer : u8
    {
        Base = 0,
        Rock = 1,
        Soil = 2,
        High = 3
    };

    // How the ground is textured. Layers is the four-slot blend driven by
    // height, slope and the painted splat; Classic is one image stretched
    // over the whole terrain times a tiling detail map. The two read the
    // same slots with different meanings, so each keeps its own copy of
    // params.custom0 across a switch.
    enum class SurfaceMode : u8
    {
        Layers = 0,
        Classic = 1
    };

    bool load(const Pixmap& heightmap, f32 cellSize = 1.0f, f32 heightScale = 32.0f,
              u32 maxLod = 6, f32 uvTiles = 1.0f);
    bool loadFile(const char* filename, f32 cellSize = 1.0f, f32 heightScale = 32.0f,
                  u32 maxLod = 6, f32 uvTiles = 1.0f);
    bool loadRaw(const char* filename, u32 size, f32 cellSize = 1.0f, f32 heightScale = 32.0f,
                 u32 maxLod = 6, f32 uvTiles = 1.0f);
    bool saveHeightmap(const char* filename);
    void clear();

    bool valid() const;
    // Sun shadows on the ground. Off skips the cascade lookup outright rather
    // than multiplying by one - see RECEIVES_SHADOW in lit.frag.
    void setReceiveShadows(bool enabled);
    bool receivesShadows() const;
    f32 cellSize() const;
    f32 heightScale() const;
    f32 uvTiles() const;
    void setUvTiles(f32 tiles);
    u32 maxLod() const;
    const std::string& heightmapFile() const;

    void setSurfaceMode(SurfaceMode mode);
    SurfaceMode surfaceMode() const;
    // Classic mode's two knobs, params.custom0.xy while it is the active
    // mode and remembered here while it is not. Tiles counts repeats of the
    // detail map across the whole terrain, strength how much of it shows.
    void setDetailTiling(f32 tiles);
    f32 detailTiling() const;
    void setDetailStrength(f32 strength);
    f32 detailStrength() const;

    Material& material();
    const Material& material() const;

    u32 width() const;
    u32 height() const;
    // Kept the old patch-era names: nothing outside this class needs to know
    // a chunk from a patch, only that these count and count-visible.
    u32 patchCount() const;
    u32 visiblePatchCount() const;
    u32 triangleCount() const;
    // Height/mesh revision only. Consumers such as Road use it to rebuild
    // conformed geometry; painting vegetation must not invalidate them.
    u64 revision() const;

    f32 heightAt(f32 localX, f32 localZ) const;

    // Surface normal at the same coordinates, from the height samples around
    // it. What a caller scattering props reads to reject cliffs, or to sit
    // something flat on the slope.
    glm::vec3 normalAt(f32 localX, f32 localZ) const;
    bool raycast(const Ray& worldRay, glm::vec3& worldHit) const;

    bool raise(const glm::vec3& worldCenter, f32 radius, f32 amount);
    bool lower(const glm::vec3& worldCenter, f32 radius, f32 amount);
    bool smooth(const glm::vec3& worldCenter, f32 radius, f32 strength = 0.5f);
    bool flatten(const glm::vec3& worldCenter, f32 radius, f32 targetHeight = 0.0f,
                 f32 strength = 1.0f);

    // Deterministic whole-terrain scatter. Grass/Forest keep ownership and
    // rendering of the generated instances; Terrain supplies height, normal,
    // slope/altitude rejection and stable jitter. Components may share this
    // GameObject or live under another transform.
    u32 generateGrass(Grass& grass, bool clearExisting = true) const;
    u32 generateTrees(Forest& forest, bool clearExisting = true) const;
    bool createVegetationMask(u32 width = 0, u32 height = 0);
    bool loadVegetationMask(const char* filename);
    bool saveVegetationMask(const char* filename);
    void discardVegetationMask();
    bool hasVegetationMask() const;
    const std::string& vegetationMaskFile() const;
    // Centre is in world space, matching the sculpt tools. Strength is the
    // amount added/subtracted from the selected channel, with a soft falloff.
    bool paintVegetation(const glm::vec3& worldCenter, f32 radius,
                         VegetationChannel channel, f32 strength, bool erase = false);
    void fillVegetation(VegetationChannel channel, f32 density);
    f32 vegetationDensity(f32 localX, f32 localZ, VegetationChannel channel) const;

    // RGBA visual splat map sampled by TerrainSurface() in lit.frag. Unlike
    // the vegetation mask, this is uploaded to SlotColorMap and changes the
    // four rendered ground layers directly.
    bool createSurfaceSplat(u32 width = 0, u32 height = 0);
    bool loadSurfaceSplat(const char* filename);
    bool saveSurfaceSplat(const char* filename);
    void discardSurfaceSplat();
    bool hasSurfaceSplat() const;
    const std::string& surfaceSplatFile() const;
    bool paintSurface(const glm::vec3& worldCenter, f32 radius, SurfaceLayer layer,
                      f32 strength, bool restoreAutomatic = false);
    VegetationSettings& grassGenerationSettings() { return mGrassGeneration; }
    const VegetationSettings& grassGenerationSettings() const { return mGrassGeneration; }
    VegetationSettings& treeGenerationSettings() { return mTreeGeneration; }
    const VegetationSettings& treeGenerationSettings() const { return mTreeGeneration; }

private:
    friend class GameObject;
    friend class Scene;

    struct Chunk
    {
        glm::vec3 position = glm::vec3(0.0f); // this chunk's centre, in the terrain's own local space
        MeshHandle mesh;

        glm::vec3 sphereCentre = glm::vec3(0.0f);
        f32 sphereRadius = 0.0f;
        f32 minimumY = 0.0f;
        f32 maximumY = 0.0f;

        // A flat chunk cannot shadow itself - whatever stands on it is a
        // different object with its own cast-shadow flag.
        bool castShadow = false;
        bool visible = true;
        u32 lastLod = 0;
    };

    Terrain();
    void onDestroy() override;

    void prepare(const Frustum& frustum, const glm::vec3& cameraPosition);
    void submitCamera(RenderList& list, const glm::mat4& transform);
    void submitShadow(RenderList& list, const glm::mat4& transform);

    bool edit(const glm::vec3& worldCenter, f32 radius, f32 amount, bool smoothing);
    void rebuildChunksTouching(u32 minX, u32 minZ, u32 maxX, u32 maxZ);
    bool buildChunk(u32 chunkX, u32 chunkZ);
    void releaseChunk(Chunk& chunk);
    u32 pickLod(const Chunk& chunk, const glm::vec3& cameraPosition,
                const glm::mat4& transform) const;
    u32 vertexIndex(u32 x, u32 z) const;
    f32 sampleHeight(s32 x, s32 z) const;
    glm::vec4 automaticSurfaceWeights(f32 localX, f32 localZ) const;
    // The layer thresholds wherever they currently live: params.custom0 in
    // Layers mode, the saved copy while Classic mode is holding custom0.
    const glm::vec4& layerThresholds() const;
    void uploadSurfaceSplat(bool recreate);

    std::vector<f32> mHeights;
    std::vector<Chunk> mChunks;
    Material mMaterial;
    AABB mBounds;
    u32 mWidth = 0;
    u32 mHeight = 0;
    u32 mChunkCountX = 0;
    u32 mChunkCountZ = 0;
    u32 mMaxLod = 0;
    u32 mVisibleChunks = 0;
    f32 mCellSize = 1.0f;
    f32 mHeightScale = 1.0f;
    f32 mUvTiles = 1.0f;
    SurfaceMode mSurfaceMode = SurfaceMode::Layers;
    // The inactive mode's custom0. Whichever mode is active owns the live
    // params.custom0; setSurfaceMode() copies it here before handing the
    // register over, so sliders edited directly on the material survive a
    // round trip through the other mode.
    glm::vec4 mLayerParams = glm::vec4(0.16f, 0.72f, 0.08f, 0.32f);
    glm::vec4 mClassicParams = glm::vec4(64.0f, 0.6f, 0.0f, 0.0f);
    std::string mHeightmapFile;
    u64 mRevision = 0;
    VegetationSettings mGrassGeneration;
    VegetationSettings mTreeGeneration;
    std::unique_ptr<Pixmap> mVegetationMask;
    std::string mVegetationMaskFile;
    std::unique_ptr<Pixmap> mSurfaceSplat;
    std::string mSurfaceSplatFile;
    TextureHandle mSurfaceSplatTexture;
};

} // namespace Radion

#endif // RADION_TERRAIN_H
