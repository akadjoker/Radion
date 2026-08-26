#ifndef RADION_HAIR_H
#define RADION_HAIR_H

#include "Component.h"
#include "HairRender.h"
#include "Mesh.h"

#include <string>
#include <vector>

namespace Radion
{

class Animator;

// GPU simulated hair grown from a sibling MeshRenderer. The selected scalp
// submesh should contain only the part allowed to grow hair; vertex alpha is
// additionally used as both density and relative length (empty colours mean
// full density). Roots are generated once and remain deterministic for seed.
class Hair final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Hair;

    bool generate();
    void clear();
    u32 rootCount() const;

    bool loadTexture(const std::string& filename);
    const std::string& textureFile() const;
    TextureHandle texture() const;

    void setStrandCount(u32 count);
    void setSubmesh(u32 submesh);
    void setSeed(u32 seed);
    void setMinimumGrowthNormalY(f32 value);
    void setSegments(u32 segments);
    void setFollowers(u32 followers);
    void setLengthRange(f32 minimum, f32 maximum);
    void setWidth(f32 width);
    void setStiffness(f32 stiffness);
    void setDrag(f32 drag);
    void setGravity(f32 gravity);
    void setWind(f32 wind);
    void setDrawDistance(f32 distance);
    void setAlphaCut(f32 cut);
    void setRoughness(f32 roughness);
    void setSpecularStrength(f32 strength);
    void setSpecularTint(f32 tint);
    void setTransmission(f32 transmission);
    void setColor(const Math::Vec3& color);
    void setSoftFringe(bool enabled);

    u32 strandCount() const;
    u32 submesh() const;
    u32 seed() const;
    f32 minimumGrowthNormalY() const;
    u32 segments() const;
    u32 followers() const;
    f32 minimumLength() const;
    f32 maximumLength() const;
    f32 width() const;
    f32 stiffness() const;
    f32 drag() const;
    f32 gravity() const;
    f32 wind() const;
    f32 drawDistance() const;
    f32 alphaCut() const;
    f32 roughness() const;
    f32 specularStrength() const;
    f32 specularTint() const;
    f32 transmission() const;
    const Math::Vec3& color() const;
    bool softFringe() const;

    void clearColliders();
    bool addSphereCollider(const Math::Vec3& worldCentre, f32 radius);
    bool addCapsuleCollider(const Math::Vec3& worldA, const Math::Vec3& worldB, f32 radius);
    u32 colliderCount() const;

private:
    friend class GameObject;
    friend class Scene;

    Hair();
    void onDestroy() override;
    void submit(f32 deltaTime);
    f32 random();

    std::vector<HairRoot> mRoots;
    std::vector<HairCollider> mColliders;
    TextureHandle mTexture;
    std::string mTextureFile;
    u64 mKey = 0;
    u64 mRevision = 0;
    u32 mStrandCount = 8000;
    u32 mSubmesh = 0;
    u32 mSeed = 1337;
    f32 mMinimumGrowthNormalY = -1.0f;
    u32 mRandomState = 1337;
    u32 mSegments = 6;
    u32 mFollowers = 2;
    f32 mMinimumLength = 0.12f;
    f32 mMaximumLength = 0.32f;
    f32 mWidth = 0.008f;
    f32 mStiffness = 18.0f;
    f32 mDrag = 0.12f;
    f32 mGravity = 5.0f;
    f32 mWind = 0.35f;
    f32 mDrawDistance = 80.0f;
    f32 mAlphaCut = 0.32f;
    f32 mRoughness = 0.38f;
    f32 mSpecularStrength = 0.12f;
    f32 mSpecularTint = 0.55f;
    f32 mTransmission = 0.30f;
    Math::Vec3 mColor = Math::Vec3(0.12f, 0.055f, 0.025f);
    bool mSoftFringe = true;
    bool mReset = true;
};

} // namespace Radion

#endif // RADION_HAIR_H
