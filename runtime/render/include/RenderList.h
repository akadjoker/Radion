#ifndef RADION_RENDER_LIST_H
#define RADION_RENDER_LIST_H

#include "Material.h"
#include "Math.h"
#include "Mesh.h"
#include "Types.h"

#include <vector>

namespace Radion
{

// Ordinals matter: they match ENTITY_TYPE_* in lit.frag exactly, Decal
// included, so RenderLight uploads to the entity SSBO with no translation.
enum class RenderLightType : u32
{
    Directional,
    Point,
    Spot,
    Rectangle,
    Decal
};

// Bit positions match ENTITY_FLAG_* in the GLSL shaders exactly (lit.frag,
// volumetric_spot.comp), which in turn match the reference's Wicked-derived
// layout - bit 0 is deliberately left unused so this stays lined up.
enum RenderLightFlags : u32
{
    RenderLightCastShadow = 1 << 1,
    RenderLightVolumetric = 1 << 2,

    // Matches ENTITY_FLAG_DECAL_BASECOLOR_ONLY_ALPHA in lit.frag. Decal-only:
    // the decal tints with its own color instead of multiplying the albedo
    // sample by it.
    RenderDecalBaseColorOnlyAlpha = 1 << 3
};

// Scene-facing light snapshot, doing double duty as a decal snapshot: a decal
// is not geometry, so it rides the same entity SSBO the lights already share
// and reuses their fields for its own purpose (see Lighting::submitDecals) -
// coneAngleCos becomes the slope-fade exponent, coneAngleScale the opacity,
// rectangleWidth the normal strength. Shadow allocation fills
// matrixIndex/atlas/fade later in the frame; collection itself never knows
// about a GPU backend.
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

// A local reflection probe already resolved to this instance - the nearest
// ReflectionProbe (scene/) to whatever submitted it, picked before render/
// ever sees it, since render/ cannot depend on scene/'s component types.
// Default-constructed (an invalid cubemap) means "no local probe nearby";
// ForwardPass falls back to the frame's own single environment cube for
// those, exactly the behavior a scene with no ReflectionProbe components had
// before this existed.
struct RenderProbe
{
    TextureHandle cubemap;
    SamplerHandle sampler;
    Math::vec3 position = Math::vec3(0.0f);
    Math::vec3 extents = Math::vec3(0.0f);
    u32 mipCount = 1;
    f32 intensity = 1.0f;
};

// What a packet needs at draw time but the sort must never move. The model
// matrix is not here: it lives in a parallel array of its own so the whole
// frame's matrices can go to the GPU in one upload.
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

// 16 bytes, and it has to stay that way: the sort moves these around, so
// everything here is an index or a sort field and the matrix stays out.
struct RenderPacket
{
    u32 instance = 0;
    u32 mesh = 0;     // MeshHandle index; in the key so same-mesh packets group
    u32 sortBits = 0; // pipeline index
    u16 distance = 0; // top 16 bits of the float pattern, monotonic
    // Albedo TextureHandle index, truncated - the base texture, always
    // paired with the same normal/specular/emissive set for a given
    // material. A single big merged mesh (an exported level) has one
    // MeshHandle for every submesh, so `mesh` above gives the sort nothing
    // to group by; this is what actually keeps consecutive draws from
    // rebinding a different texture set every time. See opaqueKey().
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

    // Only collect materials carrying every flag in `required`; 0 takes
    // everything. A shadow view sets MaterialCastShadow here.
    void setFilter(u32 required)
    {
        mFilter = required;
    }
    u32 filter() const
    {
        return mFilter;
    }

    // An extra reject test, run before the frustum, on top of it rather than
    // instead of it. A point light's shadow face is only a 90-degree frustum,
    // but nothing beyond the light's own range is ever lit regardless of
    // facing, and a sphere-vs-box test is cheaper than the six-plane one.
    // Radius 0 (the default, and what clear() resets to) disables it.
    void setCullSphere(const Sphere& sphere)
    {
        mCullSphere = sphere;
    }

    void addCulled(u32 count)
    {
        mStats.culledMeshes += count;
    }

    // Culls against the frustum - mesh box first, then per submesh - and
    // appends a packet for each submesh that survives. Materials come from
    // `overrides` when given (indexed by SubMesh::materialSlot), otherwise
    // from the mesh's own. Returns the number of packets added.
    u32 submit(MeshHandle handle, const Mesh& mesh, const Math::mat4& model,
               const Material* overrides = nullptr, u32 overrideCount = 0,
               const std::vector<Math::mat4>* palette = nullptr, const RenderProbe* probe = nullptr,
               const Math::mat4* prevModel = nullptr,
               const std::vector<Math::mat4>* prevPalette = nullptr);

    // Same culling and packet emission as submit(), for exactly one submesh
    // instead of scanning every one in `mesh` - for a caller (SceneBVH) that
    // already narrowed visibility down to specific submeshes of a mesh with
    // many, where a big static one otherwise means testing hundreds of boxes
    // that have nothing to do with what got past the spatial index. Returns
    // 1 if the submesh survived culling, 0 otherwise.
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

    // Opaque orders pipeline, then mesh (so same-mesh packets end up adjacent
    // and can collapse into one instanced draw), then front-to-back for
    // early-Z. Transparent is led by distance, back-to-front, because
    // blending is order-dependent.
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

    // 256 identity matrices - the palette a skinned RenderInstance falls
    // back to when it has none of its own (a MeshRenderer with no Animator
    // yet: Scene passes palette=nullptr). Every draw pass that appends
    // instance.palette into its own upload buffer uses this instead when
    // instance.palette is null but instance.material carries MaterialSkinned
    // - without it, the pass leaves that instance's slice of the buffer
    // untouched (whatever the previous frame's draw left there), and the
    // vertex shader skins by garbage bone matrices instead of the identity
    // (== bind pose) a still-unposed skinned mesh should read as. 256 is the
    // same per-file bone cap the importers already enforce (see FbxImporter's
    // buildBoneMap), so every joint index a skin vertex can carry resolves.
    static const std::vector<Math::mat4>& identityPalette();

private:
    // Material lookup + packet emission, the part submit()'s loop and
    // submitSubmesh() share - culling and the loop itself are the only
    // difference between them.
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

// What a shadow-casting view needs from whoever owns the scene. scene/ sits
// above render/ and depends on it, never the other way around (see
// docs/ENGINE.md), so a render/ technique that needs a shadow view's caster
// list cannot call Scene directly - it calls this instead, and Scene is the
// one thing on the other side implementing it.
class ShadowCasterSource
{
public:
    virtual ~ShadowCasterSource() = default;

    // A shadow-casting view of the same frame the camera's list was built
    // from: same renderers, same already-resolved pipelines, culled against
    // `viewProjection` instead of the camera's. `cullSphere`, when given, is
    // an extra reject test run before the frustum - cheaper, and exact for a
    // point light's shadow face, where nothing outside its range can ever be
    // lit in the first place.
    //
    // `exclude` drops every renderer drawing that mesh. It exists for the
    // environment probe: a mirrored surface standing at the capture point
    // would fill the cubemap with the inside of itself, and then reflect
    // that. Left invalid by the shadow passes, which want every caster.
    // `excludeObjectId` is the same idea addressed at one object instead of
    // one mesh, and is what an environment probe attached to an object uses:
    // meshes are shared (AssetManager caches them by description), so
    // `exclude` alone cannot say "this sphere but not that identical one".
    // An opaque id here - whoever implements this is the side that knows what
    // it identifies; 0 means exclude nothing.
    // `reflectionCapture` says this list is for an environment probe rather
    // than for a shadow view, which is the only thing that can honour a
    // per-object "keep me out of reflections" opt-out: the same object still
    // has to cast its shadow, so the flag cannot simply drop it everywhere.
    virtual bool buildShadowList(RenderList& list, const Math::mat4& viewProjection, u32 filter,
                                 const Sphere* cullSphere = nullptr,
                                 MeshHandle exclude = MeshHandle(), u64 excludeObjectId = 0,
                                 bool reflectionCapture = false,
                                 const std::vector<Plane>* casterPlanes = nullptr,
                                 f32 minCasterExtent = 0.0f) = 0;
};

} // namespace Radion

#endif // RADION_RENDER_LIST_H
