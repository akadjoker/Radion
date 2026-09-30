#ifndef RADION_RENDER_LIST_H
#define RADION_RENDER_LIST_H

#include "Material.h"
#include "Math.h"
#include "Mesh.h"
#include "Types.h"

#include <vector>

namespace Radion
{

// Ordinals match ENTITY_TYPE_* in lit.frag (Decal included), so RenderLight uploads to the entity SSBO untranslated.
enum class RenderLightType : u32
{
    Directional,
    Point,
    Spot,
    Rectangle,
    Decal
};

// Bit positions match ENTITY_FLAG_* in lit.frag and volumetric_spot.comp; bit 0 is deliberately unused.
enum RenderLightFlags : u32
{
    RenderLightCastShadow = 1 << 1,
    RenderLightVolumetric = 1 << 2,

    // Matches ENTITY_FLAG_DECAL_BASECOLOR_ONLY_ALPHA in lit.frag. Decal-only: tints with the decal's own color instead of multiplying the albedo sample.
    RenderDecalBaseColorOnlyAlpha = 1 << 3
};

// Light snapshot, doubling as a decal snapshot: decals ride the same entity SSBO and reuse fields (see Lighting::submitDecals): coneAngleCos = slope-fade exponent, coneAngleScale = opacity, rectangleWidth = normal strength.
// Shadow allocation fills matrixIndex/atlas/fade later in the frame.
struct alignas(16) RenderLight
{
    Math::vec3 position = Math::vec3(0.0f);
    f32 range = 0.0f;
    Math::vec3 direction = Math::vec3(0.0f, -1.0f, 0.0f);
    f32 coneAngleCos = 0.0f;
    Math::vec3 color = Math::vec3(1.0f);
    f32 coneAngleScale = 0.0f;
    RenderLightType type = RenderLightType::Point;
    u32 flags = 0;
    s32 matrixIndex = -1;
    f32 shadowFade = 0.0f;
    Math::vec4 shadowAtlasMulAdd = Math::vec4(0.0f);
    Math::vec3 rectangleRight = Math::vec3(1.0f, 0.0f, 0.0f);
    f32 rectangleWidth = 0.0f;
    Math::vec3 rectangleUp = Math::vec3(0.0f, 1.0f, 0.0f);
    f32 rectangleHeight = 0.0f;
};

static_assert(sizeof(RenderLight) % 16 == 0, "GPU light layout must stay std430 aligned");

// Local reflection probe already resolved to this instance (nearest ReflectionProbe, picked before render/, which cannot depend on scene/).
// Default-constructed (invalid cubemap) means none nearby; ForwardPass then uses the frame's environment cube.
struct RenderProbe
{
    TextureHandle cubemap;
    SamplerHandle sampler;
    Math::vec3 position = Math::vec3(0.0f);
    Math::vec3 extents = Math::vec3(0.0f);
    u32 mipCount = 1;
    f32 intensity = 1.0f;
};

// What a packet needs at draw time but the sort must never move. The model matrix lives in a parallel array so all matrices upload in one go.
struct RenderInstance
{
    const Material* material = nullptr;
    PipelineHandle pipeline;
    MeshHandle mesh;
    u32 submesh = 0;
    const std::vector<Math::mat4>* palette = nullptr;
    const std::vector<Math::mat4>* prevPalette = nullptr;
    RenderProbe probe;
};

// 16 bytes, and must stay that way: the sort moves these, so only indices and sort fields live here.
struct RenderPacket
{
    u32 instance = 0;
    u32 mesh = 0;     // MeshHandle index; in the key so same-mesh packets group
    u32 sortBits = 0; // pipeline index
    u16 distance = 0; // top 16 bits of the float pattern, monotonic
    // Albedo TextureHandle index, truncated. A merged mesh has one MeshHandle for all submeshes, so `mesh` gives the sort nothing to group by; this keeps consecutive draws from rebinding texture sets. See opaqueKey().
    u16 textureKey = 0;
};

static_assert(sizeof(RenderPacket) == 16, "the sort moves these; keep them small");

struct RenderListStats
{
    u32 submitted = 0;
    u32 culledMeshes = 0;
    u32 culledSubmeshes = 0;
    u32 packets = 0;
    u32 lights = 0;
    u32 droppedLights = 0;
};

class RenderList
{
public:
    static constexpr usize CategoryCount = 4;
    static constexpr usize MaxLights = 256;

    void setCamera(const Math::mat4& viewProjection, const Math::vec3& position);

    const Frustum& frustum() const
    {
        return mFrustum;
    }
    const Math::vec3& cameraPosition() const
    {
        return mCameraPosition;
    }

    void clear();

    // Only collect materials carrying every flag in `required` (0 = all); shadow views set MaterialCastShadow.
    void setFilter(u32 required)
    {
        mFilter = required;
    }
    u32 filter() const
    {
        return mFilter;
    }

    // Extra reject test before the frustum, cheaper than six planes: a point light's shadow face is a 90-degree frustum but nothing beyond its range is lit. Radius 0 (default, reset by clear()) disables it.
    void setCullSphere(const Sphere& sphere)
    {
        mCullSphere = sphere;
    }

    void addCulled(u32 count)
    {
        mStats.culledMeshes += count;
    }

    // Culls mesh box then per submesh, appending a packet per survivor. Materials come from `overrides` (indexed by SubMesh::materialSlot) when given. Returns packets added.
    u32 submit(MeshHandle handle, const Mesh& mesh, const Math::mat4& model,
               const Material* overrides = nullptr, u32 overrideCount = 0,
               const std::vector<Math::mat4>* palette = nullptr, const RenderProbe* probe = nullptr,
               const Math::mat4* prevModel = nullptr,
               const std::vector<Math::mat4>* prevPalette = nullptr);

    // submit() for exactly one submesh, for callers (SceneBVH) that already narrowed visibility. Returns 1 if it survived culling.
    u32 submitSubmesh(MeshHandle handle, const Mesh& mesh, u32 submeshIndex, const Math::mat4& model,
                      const Material* overrides = nullptr, u32 overrideCount = 0,
                      const std::vector<Math::mat4>* palette = nullptr,
                      const RenderProbe* probe = nullptr, const Math::mat4* prevModel = nullptr,
                      const std::vector<Math::mat4>* prevPalette = nullptr);

    bool addLight(const RenderLight& light);
    const std::vector<RenderLight>& lights() const
    {
        return mLights;
    }

    void setSunIndex(s32 index);
    s32 sunIndex() const
    {
        return mSunIndex;
    }
    const RenderLight* sun() const;

    // Opaque: pipeline, then mesh (adjacent same-mesh packets collapse into one instanced draw), then front-to-back for early-Z. Transparent: back-to-front, since blending is order-dependent.
    void sort();

    const std::vector<RenderPacket>& packets(RenderCategory category) const;

    const RenderInstance& instance(u32 index) const
    {
        return mInstances[index];
    }

    // Contiguous and in instance order, so it uploads as one block.
    const Math::mat4* models() const
    {
        return mModels.data();
    }
    const Math::mat4* prevModels() const
    {
        return mPrevModels.data();
    }
    u32 instanceCount() const
    {
        return static_cast<u32>(mModels.size());
    }

    const RenderListStats& stats() const
    {
        return mStats;
    }

    // 256 identity matrices: the palette a skinned instance falls back to when it has none, else the vertex shader skins by stale bone matrices instead of bind pose. 256 matches the importers' per-file bone cap (FbxImporter buildBoneMap).
    static const std::vector<Math::mat4>& identityPalette();

private:
    // Material lookup + packet emission shared by submit() and submitSubmesh().
    bool emitSubmesh(MeshHandle handle, const Mesh& mesh, u32 submeshIndex, const Math::mat4& model,
                     const AABB& bounds, const Material* overrides, u32 overrideCount,
                     const std::vector<Math::mat4>* palette, const RenderProbe* probe,
                     const Math::mat4* prevModel, const std::vector<Math::mat4>* prevPalette);

    std::vector<RenderPacket> mPackets[CategoryCount];
    std::vector<RenderInstance> mInstances;
    std::vector<Math::mat4> mModels;
    std::vector<Math::mat4> mPrevModels;
    std::vector<RenderLight> mLights;
    s32 mSunIndex = -1;

    Frustum mFrustum;
    Math::vec3 mCameraPosition = Math::vec3(0.0f);
    u32 mFilter = 0;
    Sphere mCullSphere;
    RenderListStats mStats;
};

// What a shadow-casting view needs from the scene owner. scene/ depends on render/, not the reverse (docs/ENGINE.md), so techniques call this and Scene implements it.
class ShadowCasterSource
{
public:
    virtual ~ShadowCasterSource() = default;

    // A shadow-casting view of the camera list's frame (same renderers and resolved pipelines), culled against `viewProjection`. `cullSphere` is an extra reject test, exact for a point light's face.
    // `exclude` drops every renderer drawing that mesh (environment probe: keeps a mirrored surface at the capture point out of the cubemap); shadow passes leave it invalid.
    // `excludeObjectId` excludes one object, since meshes are shared and `exclude` cannot distinguish identical ones; opaque id, 0 = none.
    // `reflectionCapture` marks an environment-probe list, the only kind that honours a per-object opt-out of reflections (the object must still cast shadows).
    virtual bool buildShadowList(RenderList& list, const Math::mat4& viewProjection, u32 filter,
                                 const Sphere* cullSphere = nullptr,
                                 MeshHandle exclude = MeshHandle(), u64 excludeObjectId = 0,
                                 bool reflectionCapture = false,
                                 const std::vector<Plane>* casterPlanes = nullptr,
                                 f32 minCasterExtent = 0.0f) = 0;
};

} // namespace Radion

#endif // RADION_RENDER_LIST_H
