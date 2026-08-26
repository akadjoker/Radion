#include "PCH.h"

#include "ManualMesh.h"

#include "AssetManager.h"
#include "GameObject.h"
#include "MaterialManager.h"
#include "MeshRenderer.h"

namespace Radion
{

ManualMesh::ManualMesh() : Component(Type)
{
}

void ManualMesh::begin(const std::string& material)
{
    clear();
    beginSubMesh(material);
    mBuilding = true;
}

void ManualMesh::beginSubMesh(const std::string& material)
{
    mBuilding = true;
    Material out;
    if (!material.empty())
    {
        std::vector<Material> loaded;
        if (MaterialManager::getSingleton().load(material, loaded) && !loaded.empty())
            out = loaded.front();
        else
            out.name = material;
    }
    mCurrentMaterial = static_cast<u32>(mData.materials.size());
    mData.materials.push_back(out);
    mData.submeshes.push_back({static_cast<u32>(mData.indices.size()), 0,
                               mCurrentMaterial, 0, {}});
}

void ManualMesh::position(const Math::vec3& value)
{
    mData.positions.push_back(value);
    mData.normals.push_back(Math::vec3(0.0f, 1.0f, 0.0f));
    mData.tangents.push_back(Math::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    mData.uvs.push_back(Math::vec2(0.0f));
    mData.colors.push_back(0xffffffffu);
}

void ManualMesh::normal(const Math::vec3& value)
{
    if (!mData.normals.empty())
        mData.normals.back() = value;
}

void ManualMesh::uv(const Math::vec2& value)
{
    if (!mData.uvs.empty())
        mData.uvs.back() = value;
}

void ManualMesh::setColor(u32 value)
{
    if (!mData.colors.empty())
        mData.colors.back() = value;
}

void ManualMesh::index(u32 value)
{
    mData.indices.push_back(value);
    if (!mData.submeshes.empty())
        ++mData.submeshes.back().indexCount;
}

void ManualMesh::triangle(u32 a, u32 b, u32 c)
{
    index(a);
    index(b);
    index(c);
}

bool ManualMesh::end()
{
    if (!mBuilding || mData.positions.empty() || mData.indices.empty())
        return false;
    Assets().computeBounds(mData);
    Assets().computeSubMeshBounds(mData);
    mMesh = Assets().createMesh(mData);
    if (!mMesh.valid())
        return false;
    if (!mRenderer)
        mRenderer = owner()->addComponent<MeshRenderer>(mMesh);
    else
        mRenderer->setMesh(mMesh);
    mBuilding = false;
    return true;
}

void ManualMesh::clear()
{
    mData.clear();
    // end() hands mMesh a new GPU mesh every rebuild without ever freeing
    // the one it replaces - releasing it here, the one place both begin()
    // and a standalone clear() funnel through, is what stops every rebuild
    // after the first leaking the previous mesh's buffers.
    if (mMesh.valid())
        Assets().destroyMesh(mMesh);
    mMesh = MeshHandle();
    mBuilding = false;
}

u32 ManualMesh::vertexCount() const
{
    return static_cast<u32>(mData.positions.size());
}

Math::vec3& ManualMesh::position(u32 index)
{
    return mData.positions.at(index);
}

const Math::vec3& ManualMesh::position(u32 index) const
{
    return mData.positions.at(index);
}

Math::vec3& ManualMesh::normal(u32 index)
{
    return mData.normals.at(index);
}

const Math::vec3& ManualMesh::normal(u32 index) const
{
    return mData.normals.at(index);
}

Math::vec2& ManualMesh::uv(u32 index)
{
    return mData.uvs.at(index);
}

const Math::vec2& ManualMesh::uv(u32 index) const
{
    return mData.uvs.at(index);
}

u32& ManualMesh::color(u32 index)
{
    return mData.colors.at(index);
}

const u32& ManualMesh::color(u32 index) const
{
    return mData.colors.at(index);
}

MeshHandle ManualMesh::mesh() const
{
    return mMesh;
}

} // namespace Radion
