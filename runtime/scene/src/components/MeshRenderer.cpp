#include "PCH.h"

#include "MeshRenderer.h"

#include "AssetManager.h"
#include "GameObject.h"
#include "MaterialManager.h"

namespace Radion
{

namespace
{
// An override is a value copy; carrying GPU handles across would alias the mesh material's UBO/pipeline, so editing one object would affect others.
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
    // Overrides are authored against the old mesh's slot layout and own their parameter buffers; neither may follow a different mesh.
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

bool MeshRenderer::generated() const
{
    return mGeneratedBy != nullptr;
}

void MeshRenderer::setGeneratedBy(Component* component)
{
    mGeneratedBy = component;
}
MeshHandle MeshRenderer::mesh() const
{
    return mMesh;
}

void MeshRenderer::setMaterialOverride(u32 slot, const Material& material)
{
    if (slot >= mMaterialOverrides.size())
    {
        // resize() value-initializes blank Materials, and emitSubmesh() reads any index below materialOverrideCount() as a real override; fill new slots with the mesh's own material or setting slot 12 would blank slots 0-11.
        const usize previousCount = mMaterialOverrides.size();
        mMaterialOverrides.resize(slot + 1);
        const Mesh* mesh = Assets().getMesh(mMesh);
        if (mesh)
            for (usize i = previousCount; i < mesh->materials.size() && i <= slot; ++i)
                mMaterialOverrides[i] = materialForOverride(mesh->materials[i]);
    }
    Material replacement = materialForOverride(material);
    // sync() may have allocated a UBO for this slot; replacing without releasing leaks one buffer per Inspector edit.
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

u32 MeshRenderer::submeshCount() const
{
    const Mesh* mesh = Assets().getMesh(mMesh);
    return mesh ? static_cast<u32>(mesh->submeshes.size()) : 0;
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
