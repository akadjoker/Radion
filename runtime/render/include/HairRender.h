#ifndef RADION_HAIR_RENDER_H
#define RADION_HAIR_RENDER_H

#include "GPU.h"
#include "RenderTechnique.h"

#include <glm/glm.hpp>
#include <vector>

namespace Radion
{

// A stable root sampled from the scalp in bind pose. Four dominant bone
// influences are enough to keep it attached to the same deformation as the
// source mesh without making the compute shader decode MeshSkinVertex's
// packed/interleaved vertex format.
struct alignas(16) HairRoot
{
    Math::Vec4 positionLength = Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
    Math::Vec4 normalWidth = Math::Vec4(0.0f, 1.0f, 0.0f, 0.01f);
    glm::uvec4 joints = glm::uvec4(0u);
    Math::Vec4 weights = Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f);
    // x rotation, y stable colour variation, z clump offset, w reserved.
    Math::Vec4 params = Math::Vec4(0.0f);
};

enum class HairColliderType : u32
{
    Sphere = 0,
    Capsule = 1,
};

struct alignas(16) HairCollider
{
    // Sphere: a.xyz=center, a.w=radius. Capsule: a/b are endpoints, a.w=radius.
    Math::Vec4 a = Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
    Math::Vec4 b = Math::Vec4(0.0f);
    HairColliderType type = HairColliderType::Sphere;
    u32 padding[3] = {};
};

static_assert(sizeof(HairRoot) == 80, "HairRoot must match the shader's std430 layout");
static_assert(sizeof(HairCollider) == 48, "HairCollider must match the shader's std140 layout");

constexpr u32 kHairMaxColliders = 8;
constexpr u32 kHairMaxSegments = 10;
constexpr u32 kHairMaxFollowers = 3;

struct HairDrawCommand
{
    u64 key = 0;
    const HairRoot* roots = nullptr;
    u32 rootCount = 0;
    u64 revision = 0;

    const std::vector<Math::Mat4>* palette = nullptr;
    const std::vector<Math::Mat4>* previousPalette = nullptr;
    Math::Mat4 model = Math::Mat4(1.0f);
    Math::Mat4 previousModel = Math::Mat4(1.0f);

    const HairCollider* colliders = nullptr;
    u32 colliderCount = 0;
    TextureHandle texture;

    Math::Vec3 color = Math::Vec3(0.12f, 0.055f, 0.025f);
    f32 roughness = 0.38f;
    f32 specularStrength = 0.12f;
    f32 specularTint = 0.55f;
    f32 transmission = 0.30f;
    f32 stiffness = 18.0f;
    f32 drag = 0.12f;
    f32 gravity = 5.0f;
    f32 wind = 0.35f;
    f32 drawDistance = 80.0f;
    f32 alphaCut = 0.32f;
    f32 deltaTime = 0.0f;
    u32 segments = 6;
    u32 followers = 2;
    bool softFringe = true;
    bool reset = false;
};

class HairRenderQueue
{
public:
    static HairRenderQueue& getSingleton();
    void clear();
    void submit(const HairDrawCommand& command);
    const std::vector<HairDrawCommand>& commands() const;

private:
    std::vector<HairDrawCommand> mCommands;
};

HairRenderQueue& HairDraws();
RenderTechnique* createHairPass();

} // namespace Radion

#endif // RADION_HAIR_RENDER_H
