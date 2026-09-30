#ifndef RADION_LANDSCAPE_H
#define RADION_LANDSCAPE_H

#include "Component.h"
#include "Material.h"
#include "Mesh.h"
#include "Noise.h"

#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

namespace Radion
{

class RenderList;

 
class Landscape final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Landscape;

    // "+3" is the reference's accounting: indices 0 and CHUNK_WIDTH-1 are the always-full-density border ring.
    static constexpr u32 ChunkWidth = 64 + 3;
    static constexpr f32 ChunkHalfWidth = static_cast<f32>(ChunkWidth - 1) * 0.5f;
    static constexpr u32 VertexCount = ChunkWidth * ChunkWidth;

    // Applied in order, each moves height in [0,1]; SampleHeight() maps to [bottomLevel, topLevel] once at the end.
    struct Modifier
    {
        enum class Kind : u8
        {
            Perlin,
            Voronoi,
            Heightmap
        };
        enum class Blend : u8
        {
            Normal,
            Additive,
            Multiply
        };

        Kind kind = Kind::Perlin;
        Blend blend = Blend::Normal;
        f32 weight = 0.5f;
        f32 frequency = 0.0005f; // 1 / scale
        bool enabled = true;

        // Overwritten by Landscape::restart() to config.seed.
        u32 seed = 0;
        Noise::Perlin perlin;

        u32 octaves = 6;

        f32 fade = 2.59f;
        f32 shape = 0.7f;
        f32 falloff = 6.0f;
        f32 perturbation = 0.1f;

        std::vector<u8> heightmapData; // 8-bit greyscale, row-major
        u32 heightmapWidth = 0;
        u32 heightmapHeight = 0;
        f32 heightmapAmount = 0.1f;

 
        f32 borderFadePixels = 48.0f;

        void setScale(f32 scale)
        {
            frequency = 1.0f / scale;
        }
    };

    struct Config
    {
        f32 chunkScale = 1.0f;
        f32 bottomLevel = -60.0f;
        f32 topLevel = 380.0f;

        f32 slopeThreshold = 1.0f;
        f32 lowAltitudeThreshold = 2.0f;
        f32 highAltitudeThreshold = 8.0f;

        u32 generationRadius = 8;
        u32 seed = 3926;

        // The spiral build stops when the ms budget runs out and resumes next frame.
        f32 generationBudgetMs = 8.0f;

        s32 lodBias = 0;
        bool centreToCamera = true;
        bool removal = true;
    };

    Config config;
    std::vector<Modifier> modifiers;

    void restart();
 
    void invalidateRegion(const Math::vec2& centreXZ, f32 radius);

    // Far cheaper than restart(): keeps the GPU buffers.
    void invalidateAll();
    u32 pendingInvalidations() const;
 
    f32 heightAt(f32 worldX, f32 worldZ) const;

    // Paints into the first enabled Heightmap modifier; no-op if none. amount/strength in normalised [0,1] height.
    bool sculptRaise(const Math::vec3& worldCenter, f32 radius, f32 amount);
    bool sculptLower(const Math::vec3& worldCenter, f32 radius, f32 amount);
    bool sculptSmooth(const Math::vec3& worldCenter, f32 radius, f32 strength);

    bool sculptFlatten(const Math::vec3& worldCenter, f32 radius, f32 targetHeight, f32 strength);

    Material& material();
    const Material& material() const;

    u32 chunkCount() const;
    u32 visibleChunkCount() const;
    u32 shadowChunkCount() const;
    u32 triangleCount() const;
    u32 builtLastFrame() const;
    f32 lastBuildMilliseconds() const;
    u32 lodHistogram(u32 lod) const;
    u32 lodCount() const;

private:
    friend class GameObject;
    friend class Scene;

    struct ChunkKey
    {
        s32 x = 0;
        s32 z = 0;

        bool operator==(const ChunkKey& other) const
        {
            return x == other.x && z == other.z;
        }
    };

    struct ChunkKeyHash
    {
        usize operator()(const ChunkKey& key) const
        {
            return (static_cast<usize>(static_cast<u32>(key.x)) << 32) |
                   static_cast<usize>(static_cast<u32>(key.z));
        }
    };

    struct Chunk
    {
        Math::vec3 position = Math::vec3(0.0f); // the chunk's own origin, in world
        MeshHandle mesh;

        Math::vec3 sphereCentre = Math::vec3(0.0f); // in world
        f32 sphereRadius = 0.0f;
        f32 minimumY = 0.0f;
        f32 maximumY = 0.0f;

        // Flat ground never shadows itself, so a chunk with no slope skips the shadow view.
        bool castShadow = false;

        bool built = false;
        bool visible = true;
        u32 lastLod = 0;
        bool invalidated = false;

        std::vector<f32> heights;
    };

    Landscape();
    void onDestroy() override;

    void update(const Math::vec3& cameraPosition);
    void cull(const Frustum& frustum);
    void submitCamera(RenderList& list, const Math::mat4& transform,
                      const Math::vec3& cameraPosition);
    void submitShadow(RenderList& list, const Math::mat4& transform,
                      const Math::vec3& cameraPosition);

    f32 sampleHeight(const Math::vec2& worldPosition) const;
    bool buildChunk(const ChunkKey& key, Chunk& chunk);
    void releaseChunk(Chunk& chunk);
    bool buildChunkWithinBudget(const ChunkKey& key, class Timer& buildTimer);
    // requestChunk(0, 0) is the centre chunk.
    bool requestChunk(s32 offsetX, s32 offsetZ, class Timer& buildTimer);
    u32 pickLod(const Chunk& chunk, const Math::vec3& cameraPosition) const;
    bool sculpt(const Math::vec3& worldCenter, f32 radius, f32 amount, bool smoothing);

    std::unordered_map<ChunkKey, Chunk, ChunkKeyHash> mChunks;
    std::deque<ChunkKey> mPriorityInvalidation;
    ChunkKey mCentreChunk;
    Material mMaterial;
    bool mSeeded = false;

    u32 mVisible = 0;
    u32 mShadowChunks = 0;
    u32 mTriangles = 0;
    u32 mBuiltLastFrame = 0;
    f32 mLastBuildMilliseconds = 0.0f;
    u32 mLodHistogram[8] = {};
};

} // namespace Radion

#endif // RADION_LANDSCAPE_H
