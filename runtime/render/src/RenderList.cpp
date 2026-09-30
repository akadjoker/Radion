#include "PCH.h"

#include "RenderList.h"

#include "MaterialManager.h"

namespace Radion
{

namespace
{

// Non-negative floats order the same as their bit patterns, so the top bits
// are a monotonic depth key with no range to tune.
u16 quantizeDepth(f32 distance)
{
    if (!(distance > 0.0f)) // also catches NaN
        return 0;

    u32 bits = 0;
    std::memcpy(&bits, &distance, sizeof(bits));
    return static_cast<u16>(bits >> 16);
}

// Both keys pack packet.mesh into fewer than its native 32 bits, so both
// have to mask it the same way - an unmasked mesh index high enough to reach
// into a neighbouring field corrupts whatever sits above it, silently and
// only for whichever handles happen to be large enough to hit it.
constexpr u64 kMeshKeyMask = 0xFFFFu;

// Built at compare time rather than stored, which is what keeps RenderPacket
// at 16 bytes. Pipeline leads (still the biggest state change), then the
// base texture - the actual thing that was rebinding a texture every draw
// on a big merged mesh, where every packet shares one `mesh` - then mesh,
// then front-to-back distance for early-Z within whatever is left to break
// ties.
u64 opaqueKey(const RenderPacket& packet)
{
    return (static_cast<u64>(packet.sortBits & 0xFFFFu) << 48) |
           (static_cast<u64>(packet.textureKey) << 32) |
           (static_cast<u64>(packet.mesh & kMeshKeyMask) << 16) | static_cast<u64>(packet.distance);
}

u64 transparentKey(const RenderPacket& packet)
{
    return (static_cast<u64>(packet.distance) << 48) | (static_cast<u64>(packet.sortBits) << 24) |
           static_cast<u64>(packet.mesh & kMeshKeyMask);
}

} // namespace

const std::vector<Math::mat4>& RenderList::identityPalette()
{
    static const std::vector<Math::mat4> identity(256, Math::mat4(1.0f));
    return identity;
}

void RenderList::setCamera(const Math::mat4& viewProjection, const Math::vec3& position)
{
    mFrustum.update(viewProjection);
    mCameraPosition = position;
}

void RenderList::clear()
{
    for (usize i = 0; i < CategoryCount; ++i)
        mPackets[i].clear();
    mInstances.clear();
    mModels.clear();
    mPrevModels.clear();
    mLights.clear();
    mSunIndex = -1;
    mStats = RenderListStats();
    // A reused list (a shadow view rebuilt every cascade or every atlas tile)
    // must not carry the previous pass's sphere into one that never set its
    // own - that would silently reject casters a directional cascade should
    // have kept.
    mCullSphere = Sphere();
    // Same reasoning for the flag filter: a list last used for a shadow view
    // (MaterialCastShadow) that gets reused for an ordinary camera view
    // without an explicit setFilter(0) would silently drop every material
    // that does not cast a shadow. Every builder that wants a filter sets one
    // itself; clear() must not let the previous use's filter survive it.
    mFilter = 0;
}

bool RenderList::addLight(const RenderLight& light)
{
    if (mLights.size() >= MaxLights)
    {
        ++mStats.droppedLights;
        return false;
    }
    mLights.push_back(light);
    mStats.lights = static_cast<u32>(mLights.size());
    return true;
}

void RenderList::setSunIndex(s32 index)
{
    mSunIndex = index;
}

const RenderLight* RenderList::sun() const
{
    if (mSunIndex >= 0 && static_cast<usize>(mSunIndex) < mLights.size())
        return &mLights[mSunIndex];
    return nullptr;
}

u32 RenderList::submit(MeshHandle handle, const Mesh& mesh, const Math::mat4& model,
                       const Material* overrides, u32 overrideCount,
                       const std::vector<Math::mat4>* palette, const RenderProbe* probe,
                       const Math::mat4* prevModel, const std::vector<Math::mat4>* prevPalette)
{
    ++mStats.submitted;

    const AABB meshBounds = transformAABB(mesh.bounds, model);
    if (mCullSphere.radius > 0.0f && !mCullSphere.intersects(meshBounds))
    {
        ++mStats.culledMeshes;
        return 0;
    }
    if (!mFrustum.intersects(meshBounds))
    {
        ++mStats.culledMeshes;
        return 0;
    }

    const bool manySubmeshes = mesh.submeshes.size() > 1;
    u32 added = 0;

    for (usize i = 0; i < mesh.submeshes.size(); ++i)
    {
        if (!mesh.submeshes[i].visible)
            continue;
        const AABB bounds = transformAABB(mesh.submeshes[i].bounds, model);

        // With one submesh its box is the mesh's, already tested above.
        if (manySubmeshes && (!mFrustum.intersects(bounds) ||
                              (mCullSphere.radius > 0.0f && !mCullSphere.intersects(bounds))))
        {
            ++mStats.culledSubmeshes;
            continue;
        }

        if (emitSubmesh(handle, mesh, static_cast<u32>(i), model, bounds, overrides, overrideCount,
                        palette, probe, prevModel, prevPalette))
            ++added;
    }

    mStats.packets += added;
    return added;
}

u32 RenderList::submitSubmesh(MeshHandle handle, const Mesh& mesh, u32 submeshIndex,
                              const Math::mat4& model, const Material* overrides, u32 overrideCount,
                              const std::vector<Math::mat4>* palette, const RenderProbe* probe,
                              const Math::mat4* prevModel,
                              const std::vector<Math::mat4>* prevPalette)
{
    if (submeshIndex >= mesh.submeshes.size() || !mesh.submeshes[submeshIndex].visible)
        return 0;

    // Counted here as well as in submit(), or the panel's "submitted" reads
    // as almost nothing the moment the BVH is on: everything static comes in
    // through this entry point, and only what the BVH did not already reject
    // ever reaches it.
    ++mStats.submitted;

    const AABB bounds = transformAABB(mesh.submeshes[submeshIndex].bounds, model);
    if (mCullSphere.radius > 0.0f && !mCullSphere.intersects(bounds))
    {
        ++mStats.culledSubmeshes;
        return 0;
    }
    if (!mFrustum.intersects(bounds))
    {
        ++mStats.culledSubmeshes;
        return 0;
    }

    const bool added = emitSubmesh(handle, mesh, submeshIndex, model, bounds, overrides,
                                   overrideCount, palette, probe, prevModel, prevPalette);
    mStats.packets += added ? 1 : 0;
    return added ? 1 : 0;
}

bool RenderList::emitSubmesh(MeshHandle handle, const Mesh& mesh, u32 submeshIndex,
                             const Math::mat4& model, const AABB& bounds, const Material* overrides,
                             u32 overrideCount, const std::vector<Math::mat4>* palette,
                             const RenderProbe* probe, const Math::mat4* prevModel,
                             const std::vector<Math::mat4>* prevPalette)
{
    const SubMesh& submesh = mesh.submeshes[submeshIndex];

    const Material* material = nullptr;
    if (overrides && submesh.materialSlot < overrideCount)
        material = &overrides[submesh.materialSlot];
    else if (submesh.materialSlot < mesh.materials.size())
        material = &mesh.materials[submesh.materialSlot];

    if (!material || !material->pipeline.valid() || (material->flags & mFilter) != mFilter)
        return false;

    RenderInstance instance;
    instance.material = material;
    instance.pipeline = material->pipeline;
    instance.mesh = handle;
    instance.submesh = submeshIndex;
    instance.palette = palette;
    instance.prevPalette = prevPalette ? prevPalette : palette;
    if (probe)
        instance.probe = *probe;

    RenderPacket packet;
    packet.instance = static_cast<u32>(mInstances.size());
    packet.mesh = handle.index;
    packet.sortBits = instance.pipeline.index;
    packet.distance = quantizeDepth(Math::length(bounds.center() - mCameraPosition));
    packet.textureKey = static_cast<u16>(material->textures[SlotAlbedo].texture.index);

    mInstances.push_back(instance);
    mModels.push_back(model);
    mPrevModels.push_back(prevModel ? *prevModel : model);
    const MaterialManager& materials = MaterialManager::getSingleton();
    mPackets[static_cast<usize>(materials.categoryOf(*material))].push_back(packet);
    return true;
}

void RenderList::sort()
{
    for (usize i = 0; i < CategoryCount; ++i)
    {
        std::vector<RenderPacket>& packets = mPackets[i];
        if (packets.size() < 2)
            continue;

        if (static_cast<RenderCategory>(i) == RenderCategory::Transparent)
        {
            std::sort(packets.begin(), packets.end(),
                      [](const RenderPacket& a, const RenderPacket& b)
                      {
                          return transparentKey(a) > transparentKey(b);
                      });
        }
        else
        {
            std::sort(packets.begin(), packets.end(),
                      [](const RenderPacket& a, const RenderPacket& b)
                      {
                          return opaqueKey(a) < opaqueKey(b);
                      });
        }
    }
}

const std::vector<RenderPacket>& RenderList::packets(RenderCategory category) const
{
    return mPackets[static_cast<usize>(category)];
}

} // namespace Radion
