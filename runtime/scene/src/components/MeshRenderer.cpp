#include "PCH.h"

#include "MeshRenderer.h"

#include "AssetManager.h"
#include "GameObject.h"
#include "MaterialManager.h"

namespace Radion
{

namespace
{
// Material GPU resources belong to the material instance that created them.
// An override is a value copy, so carrying these handles across aliases the
// mesh material's uniform buffer/pipeline and lets editing one object affect
// other objects (or update a destroyed buffer after a mesh replacement).
Material materialForOverride(const Material& source)
{
    Material copy = source;
    copy.paramsBuffer = BufferHandle();
    copy.pipeline = PipelineHandle();
    copy.paramsDirty = true;
    return copy;
}
} // namespace

MeshRenderer::MeshRenderer(MeshHandle mesh) : Component(Type), mMesh(mesh)
{
}
MeshRenderer::~MeshRenderer()
{
    clearMaterialOverrides();
    if (!mHiddenSubmeshes.empty())
    {
        mHiddenSubmeshes.clear();
        applyHiddenSubmeshes();
    }
}
void MeshRenderer::setMesh(MeshHandle mesh)
{
    if (mMesh == mesh)
        return;
    // Overrides are authored against the old mesh's slot layout and own
    // their parameter buffers. Neither may follow a different mesh.
    clearMaterialOverrides();
    if (!mHiddenSubmeshes.empty())
    {
        mHiddenSubmeshes.clear();
        applyHiddenSubmeshes();
    }
    mMesh = mesh;
    if (owner())
        owner()->invalidateSpatialMembership();
}
MeshHandle MeshRenderer::mesh() const
{
    return mMesh;
}

void MeshRenderer::setMaterialOverride(u32 slot, const Material& material)
{
    if (slot >= mMaterialOverrides.size())
    {
        // std::vector::resize value-initializes every new slot to a blank
        // Material() - and emitSubmesh() (RenderList.cpp) reads ANY index
        // below materialOverrideCount() as a real override, mesh's own
        // material or not. Left blank, setting slot 12 alone would silently
        // blank out every submesh on slots 0-11 too: untextured, unlit,
        // white. Filling each new slot with the mesh's own material first
        // keeps every index genuinely valid the instant it exists, so only
        // the one slot actually being set here ever changes what renders.
        const usize previousCount = mMaterialOverrides.size();
        mMaterialOverrides.resize(slot + 1);
        const Mesh* mesh = Assets().getMesh(mMesh);
        if (mesh)
            for (usize i = previousCount; i < mesh->materials.size() && i <= slot; ++i)
                mMaterialOverrides[i] = materialForOverride(mesh->materials[i]);
    }
    Material replacement = materialForOverride(material);
    // sync() may already have allocated a UBO for this slot. Replacing the
    // value without releasing it leaks one buffer on every Inspector edit.
    MaterialManager::getSingleton().release(mMaterialOverrides[slot]);
    mMaterialOverrides[slot] = std::move(replacement);
    if (const Mesh* mesh = Assets().getMesh(mMesh))
    {
        MaterialManager& manager = MaterialManager::getSingleton();
        manager.resolvePipeline(mMaterialOverrides[slot], mesh->colorLayout);
        manager.sync(mMaterialOverrides[slot]);
    }
}

void MeshRenderer::clearMaterialOverrides()
{
    MaterialManager& materials = MaterialManager::getSingleton();
    for (Material& material : mMaterialOverrides)
        materials.release(material);
    mMaterialOverrides.clear();
}
const Material* MeshRenderer::materialOverrides() const
{
    return mMaterialOverrides.empty() ? nullptr : mMaterialOverrides.data();
}
u32 MeshRenderer::materialOverrideCount() const
{
    return static_cast<u32>(mMaterialOverrides.size());
}

void MeshRenderer::setVisibleInReflections(bool visible)
{
    mVisibleInReflections = visible;
}

bool MeshRenderer::visibleInReflections() const
{
    return mVisibleInReflections;
}

bool MeshRenderer::submeshVisible(u32 submesh) const
{
    for (u32 hidden : mHiddenSubmeshes)
        if (hidden == submesh)
            return false;
    return true;
}

void MeshRenderer::setSubmeshVisible(u32 submesh, bool visible)
{
    for (usize i = 0; i < mHiddenSubmeshes.size(); ++i)
        if (mHiddenSubmeshes[i] == submesh)
        {
            if (visible)
                mHiddenSubmeshes.erase(mHiddenSubmeshes.begin() + static_cast<std::ptrdiff_t>(i));
            applyHiddenSubmeshes();
            return;
        }
    if (!visible)
        mHiddenSubmeshes.push_back(submesh);
    applyHiddenSubmeshes();
}

const std::vector<u32>& MeshRenderer::hiddenSubmeshes() const
{
    return mHiddenSubmeshes;
}

void MeshRenderer::setHiddenSubmeshes(std::vector<u32> hidden)
{
    mHiddenSubmeshes = std::move(hidden);
    applyHiddenSubmeshes();
}

void MeshRenderer::applyHiddenSubmeshes()
{
    Mesh* mesh = Assets().getMesh(mMesh);
    if (!mesh)
        return;
    for (SubMesh& submesh : mesh->submeshes)
        submesh.visible = true;
    for (u32 hidden : mHiddenSubmeshes)
        if (hidden < mesh->submeshes.size())
            mesh->submeshes[hidden].visible = false;
}

} // namespace Radion
