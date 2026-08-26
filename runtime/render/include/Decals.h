#ifndef RADION_DECALS_H
#define RADION_DECALS_H

#include "GPU.h"
#include "Types.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>

namespace Radion
{

// Projected decals, the way Wicked shades them: a decal is not geometry, it
// is a box and a matrix. Every lit fragment asks "am I inside this box?",
// and if so blends the sampled texture into albedo/normal/surface before the
// light loop runs - see ApplyDecals() in lit.frag.
//
// They ride the same entity SSBO and the same tile culling the lights
// already use: Lighting::submitDecals() is what appends them, sharing the
// lights' entity and matrix budget. A decal that does not fit is dropped
// rather than pushed past that limit.
class DecalSystem
{
public:
    // A decal placed in the scene. The box is unit-sized in local space
    // ([-1,1]^3); `size` gives it its world dimensions.
    struct Decal
    {
        glm::vec3 position = glm::vec3(0.0f);
        glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        glm::vec3 size = glm::vec3(1.0f); // full box dimensions, in world units

        glm::vec3 color = glm::vec3(1.0f); // tint, multiplied by the texture
        f32 opacity = 1.0f;

        // Exponent of the slope fade: a surface whose normal turns away from
        // the decal's own gets less of it. This is what keeps a floor decal
        // from smearing up the walls its box happens to clip. 0 disables it.
        f32 slopePower = 8.0f;

        f32 normalStrength = 1.0f; // 0 ignores the normal map
        s32 layer = 0;             // slice in the texture arrays
        bool baseColorOnlyAlpha = false;
        bool enabled = true;
    };

    // Global multipliers, so a panel can tune every decal at once without
    // overwriting the values authored per decal.
    f32 globalOpacity = 1.0f;
    f32 normalStrengthScale = 1.0f;
    f32 slopePowerOverride = -1.0f; // < 0 uses the decal's own

    bool create(u32 textureDim = 256, u32 maxLayers = 16);
    void shutdown();

    // Procedural placeholders: there is no decal art in the asset tree yet,
    // and a procedural mask still exercises all three maps with data that
    // agrees with itself - a crater has to have both relief and roughness.
    enum class Procedural
    {
        BulletHole, // impact: dark hole, chipped edge, crater
        ScorchMark, // burn: soft smudge, very rough
        Crack,      // branching fissure
        Blood,      // pool with satellite droplets, wet sheen, no relief
    };
    s32 addProcedural(Procedural kind, u32 seed = 1u);

    // Loads from file. normalPath/surfacePath may be empty.
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

    // Places a decal centred on a hit surface, its box's +Z rotated onto
    // `normal` - the same axis the slope fade compares the surface against.
    s32 placeOnSurface(const glm::vec3& position, const glm::vec3& normal, s32 layer, f32 size,
                       f32 thickness, f32 rotationRadians, const glm::vec3& color = glm::vec3(1.0f),
                       f32 opacity = 1.0f);

    // World -> decal box ([-1,1]^3), with the layer index hidden in the 4th
    // row. Public because Lighting::submitDecals() needs it too.
    static glm::mat4 makeProjection(const Decal& decal);

    // Radius of the sphere enclosing the box, for the same tile culling the
    // lights go through.
    static f32 boundingRadius(const Decal& decal)
    {
        return glm::length(decal.size) * 0.5f;
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
