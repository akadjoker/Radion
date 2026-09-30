#ifndef RADION_DECALS_H
#define RADION_DECALS_H

#include "GPU.h"
#include "Types.h"

#include "Math.h"
#include <string>
#include <vector>

namespace Radion
{

// Projected decals: a box and a matrix, not geometry. Each lit fragment tests whether it is inside the box and blends the decal before the light loop (ApplyDecals() in lit.frag).
// Shares the lights' entity SSBO, tile culling and budget (Lighting::submitDecals()); a decal that does not fit is dropped.
class DecalSystem
{
public:
    // Box is unit-sized in local space ([-1,1]^3); `size` gives world dimensions.
    struct Decal
    {
        Math::vec3 position = Math::vec3(0.0f);
        Math::quat rotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
        Math::vec3 size = Math::vec3(1.0f); // full box dimensions, in world units

        Math::vec3 color = Math::vec3(1.0f); // tint, multiplied by the texture
        f32 opacity = 1.0f;

        // Exponent of the slope fade (surfaces turning away from the decal's normal get less); 0 disables.
        f32 slopePower = 8.0f;

        f32 normalStrength = 1.0f; // 0 ignores the normal map
        s32 layer = 0;             // slice in the texture arrays
        bool baseColorOnlyAlpha = false;
        bool enabled = true;
    };

    f32 globalOpacity = 1.0f;
    f32 normalStrengthScale = 1.0f;
    f32 slopePowerOverride = -1.0f; // < 0 uses the decal's own

    bool create(u32 textureDim = 256, u32 maxLayers = 16);
    void shutdown();

    enum class Procedural
    {
        BulletHole, // impact: dark hole, chipped edge, crater
        ScorchMark, // burn: soft smudge, very rough
        Crack,      // branching fissure
        Blood,      // pool with satellite droplets, wet sheen, no relief
    };
    s32 addProcedural(Procedural kind, u32 seed = 1u);

    // normalPath/surfacePath may be empty.
    s32 addFromFiles(const std::string& albedoPath, const std::string& normalPath = std::string(),
                     const std::string& surfacePath = std::string());

    s32 addDecal(const Decal& decal);
    Decal& decal(u32 index)
    {
        return mDecals[index];
    }
    const Decal& decal(u32 index) const
    {
        return mDecals[index];
    }
    u32 count() const
    {
        return static_cast<u32>(mDecals.size());
    }
    u32 layerCount() const
    {
        return mLayerCount;
    }
    void clear()
    {
        mDecals.clear();
    }

    // Centres on a hit surface with the box's +Z rotated onto `normal` (the axis the slope fade uses).
    s32 placeOnSurface(const Math::vec3& position, const Math::vec3& normal, s32 layer, f32 size,
                       f32 thickness, f32 rotationRadians, const Math::vec3& color = Math::vec3(1.0f),
                       f32 opacity = 1.0f);

    // World -> decal box ([-1,1]^3), layer index in the 4th row; public for Lighting::submitDecals().
    static Math::mat4 makeProjection(const Decal& decal);

    // Bounding-sphere radius, for tile culling.
    static f32 boundingRadius(const Decal& decal)
    {
        return Math::length(decal.size) * 0.5f;
    }

    TextureHandle albedoArray() const
    {
        return mAlbedo;
    }
    TextureHandle normalArray() const
    {
        return mNormal;
    }
    TextureHandle surfaceArray() const
    {
        return mSurface;
    }

private:
    s32 reserveLayer();
    void uploadLayer(u32 layer, const std::vector<u8>& albedo, const std::vector<u8>& normal,
                     const std::vector<u8>& surface);

    TextureHandle mAlbedo;  // RGBA8: color + alpha (alpha is the decal's mask)
    TextureHandle mNormal;  // RGBA8: RG = tangent-space normal, rest unused
    TextureHandle mSurface; // RGBA8: R = roughness, G = metallic, B = reflectance

    u32 mDim = 256;
    u32 mMaxLayers = 16;
    u32 mLayerCount = 0;

    std::vector<Decal> mDecals;
};

} // namespace Radion

#endif // RADION_DECALS_H
