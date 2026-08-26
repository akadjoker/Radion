#include "PCH.h"

#include "Road.h"

#include "AssetManager.h"
#include "FileSystem.h"
#include "GameObject.h"
#include "Scene.h"
#include "Terrain.h"

#include <sstream>

namespace Radion
{

Road::Road() : Component(Type, ComponentEventUpdate)
{
    mMaterial.params.baseColor = Math::Vec4(1.0f);
    mMaterial.params.custom1.z = 1.0f;
}

bool Road::addPoint(GameObject* object, f32 width)
{
    return insertPoint(mPoints.size(), object, width);
}

bool Road::insertPoint(usize index, GameObject* object, f32 width)
{
    if (!object || index > mPoints.size())
        return false;
    for (const Point& entry : mPoints)
        if (entry.object == object)
            return false;
    mPoints.insert(mPoints.begin() + index,
                   {object, object->globalPosition(), glm::max(width, 0.1f)});
    mDirty = true;
    return true;
}

bool Road::removePoint(GameObject* object)
{
    for (usize i = 0; i < mPoints.size(); ++i)
        if (mPoints[i].object == object)
        {
            mPoints.erase(mPoints.begin() + i);
            mDirty = true;
            return true;
        }
    return false;
}

void Road::clearPoints()
{
    mPoints.clear();
    mDirty = true;
}
usize Road::pointCount() const
{
    return mPoints.size();
}
GameObject* Road::point(usize index) const
{
    return index < mPoints.size() ? mPoints[index].object : nullptr;
}
void Road::setPointWidth(usize index, f32 width)
{
    if (index >= mPoints.size())
        return;
    mPoints[index].width = glm::max(width, 0.1f);
    mDirty = true;
}
f32 Road::pointWidth(usize index) const
{
    return index < mPoints.size() ? mPoints[index].width : 0.0f;
}
void Road::setTerrain(Terrain* terrain)
{
    mTerrain = terrain;
    mTerrainRevision = terrain ? terrain->revision() : 0;
    mDirty = true;
}
Terrain* Road::terrain() const
{
    return mTerrain;
}
void Road::setSubdivisions(u32 subdivisions)
{
    mSubdivisions = glm::clamp(subdivisions, 1u, 64u);
    mDirty = true;
}
void Road::setTextureRepeat(f32 meters)
{
    mTextureRepeat = glm::max(meters, 0.1f);
    mDirty = true;
}
void Road::setSurfaceOffset(f32 offset)
{
    mSurfaceOffset = offset;
    mDirty = true;
}
void Road::setConformTerrain(bool enabled)
{
    mConformTerrain = enabled;
    mDirty = true;
}

bool Road::saveSpline(const std::string& path) const
{
    std::ostringstream text;
    text << "radion_road 1\n";
    text << "subdivisions " << mSubdivisions << '\n';
    text << "texture_repeat " << mTextureRepeat << '\n';
    text << "surface_offset " << mSurfaceOffset << '\n';
    text << "conform " << (mConformTerrain ? 1 : 0) << '\n';
    for (const Point& entry : mPoints)
    {
        if (!entry.object || entry.object->disposed())
            continue;
        const Math::Vec3 position = entry.object->globalPosition();
        text << "point " << position.x << ' ' << position.y << ' ' << position.z << ' '
             << entry.width << '\n';
    }
    return FileSystem::getSingleton().writeText(path, text.str());
}

bool Road::loadSpline(const std::string& path, Scene& scene)
{
    const std::string source = FileSystem::getSingleton().readText(path);
    if (source.empty() || !owner())
        return false;

    std::istringstream text(source);
    std::string token;
    u32 version = 0;
    if (!(text >> token >> version) || token != "radion_road" || version != 1)
        return false;

    struct LoadedPoint
    {
        Math::Vec3 position;
        f32 width;
    };
    std::vector<LoadedPoint> loaded;
    u32 subdivisions = mSubdivisions;
    f32 textureRepeat = mTextureRepeat;
    f32 surfaceOffset = mSurfaceOffset;
    bool conform = mConformTerrain;
    while (text >> token)
    {
        if (token == "subdivisions")
            text >> subdivisions;
        else if (token == "texture_repeat")
            text >> textureRepeat;
        else if (token == "surface_offset")
            text >> surfaceOffset;
        else if (token == "conform")
        {
            u32 value = 0;
            text >> value;
            conform = value != 0;
        }
        else if (token == "point")
        {
            LoadedPoint point;
            if (!(text >> point.position.x >> point.position.y >> point.position.z >> point.width))
                return false;
            loaded.push_back(point);
        }
        else
            return false;
    }
    if (loaded.size() < 2)
        return false;

    for (Point& point : mPoints)
        if (point.object && point.object->parent() == owner())
            point.object->dispose();
    clearPoints();
    setSubdivisions(subdivisions);
    setTextureRepeat(textureRepeat);
    setSurfaceOffset(surfaceOffset);
    setConformTerrain(conform);
    for (usize i = 0; i < loaded.size(); ++i)
    {
        GameObject* object = scene.createGameObject("Road point", owner());
        object->setGlobalPosition(loaded[i].position);
        if (!addPoint(object, loaded[i].width))
            return false;
    }
    return true;
}
bool Road::valid() const
{
    return mMesh.valid();
}
MeshHandle Road::mesh() const
{
    return mMesh;
}

u32 Road::subdivisions() const { return mSubdivisions; }
f32 Road::textureRepeat() const { return mTextureRepeat; }
f32 Road::surfaceOffset() const { return mSurfaceOffset; }
bool Road::conformTerrain() const { return mConformTerrain; }
Material& Road::material()
{
    return mMaterial;
}
const Material& Road::material() const
{
    return mMaterial;
}

void Road::onUpdate(f32)
{
    if (mTerrain && mTerrainRevision != mTerrain->revision())
    {
        mTerrainRevision = mTerrain->revision();
        mDirty = true;
    }
    for (Point& entry : mPoints)
    {
        if (!entry.object || entry.object->disposed())
            continue;
        const Math::Vec3 current = entry.object->globalPosition();
        if (glm::length(current - entry.previous) > 0.0001f)
        {
            entry.previous = current;
            mDirty = true;
        }
    }
    if (mDirty)
        rebuild();
}

void Road::onDestroy()
{
    if (mMesh.valid() && GPU::ready())
        Assets().destroyMesh(mMesh);
    mMesh = MeshHandle();
}

Road::PathSample Road::evaluate(usize segment, f32 amount) const
{
    const usize last = mPoints.size() - 1;
    const usize i0 = segment > 0 ? segment - 1 : segment;
    const usize i1 = segment;
    const usize i2 = glm::min(segment + 1, last);
    const usize i3 = glm::min(segment + 2, last);
    const Math::Vec3 p0 = mPoints[i0].object->globalPosition();
    const Math::Vec3 p1 = mPoints[i1].object->globalPosition();
    const Math::Vec3 p2 = mPoints[i2].object->globalPosition();
    const Math::Vec3 p3 = mPoints[i3].object->globalPosition();
    const f32 t2 = amount * amount;
    const f32 t3 = t2 * amount;
    PathSample result;
    result.position =
        0.5f * ((2.0f * p1) + (-p0 + p2) * amount + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
    result.tangent = 0.5f * ((-p0 + p2) + 2.0f * (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * amount +
                             3.0f * (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t2);
    result.width = glm::mix(mPoints[i1].width, mPoints[i2].width, amount);
    return result;
}

void Road::rebuild()
{
    mDirty = false;
    if (mMesh.valid())
    {
        Assets().destroyMesh(mMesh);
        mMesh = MeshHandle();
    }
    if (mPoints.size() < 2 || !owner())
        return;
    for (const Point& entry : mPoints)
        if (!entry.object || entry.object->disposed())
            return;

    const Math::Mat4 inverseOwner = glm::inverse(owner()->globalTransform());
    Math::Mat4 terrainInverse(1.0f);
    Math::Mat4 terrainTransform(1.0f);
    if (mTerrain && mTerrain->owner())
    {
        terrainTransform = mTerrain->owner()->globalTransform();
        terrainInverse = glm::inverse(terrainTransform);
    }
    std::vector<PathSample> path;
    path.reserve((mPoints.size() - 1) * mSubdivisions + 1);
    f32 distance = 0.0f;
    for (usize segment = 0; segment + 1 < mPoints.size(); ++segment)
        for (u32 division = 0; division < mSubdivisions; ++division)
        {
            PathSample value = evaluate(segment, static_cast<f32>(division) / mSubdivisions);
            if (!path.empty())
                distance += glm::length(value.position - path.back().position);
            value.distance = distance;
            path.push_back(value);
        }
    PathSample end = evaluate(mPoints.size() - 2, 1.0f);
    distance += glm::length(end.position - path.back().position);
    end.distance = distance;
    path.push_back(end);

    MeshData data;
    data.positions.reserve(path.size() * 2);
    data.normals.reserve(path.size() * 2);
    data.uvs.reserve(path.size() * 2);
    for (const PathSample& value : path)
    {
        Math::Vec3 tangent = value.tangent;
        tangent.y = 0.0f;
        if (glm::dot(tangent, tangent) < 0.0001f)
            tangent = Math::Vec3(0.0f, 0.0f, -1.0f);
        tangent = glm::normalize(tangent);
        const Math::Vec3 right = glm::normalize(glm::cross(Math::Vec3(0, 1, 0), tangent));
        Math::Vec3 leftPosition = value.position - right * (value.width * 0.5f);
        Math::Vec3 rightPosition = value.position + right * (value.width * 0.5f);
        if (mConformTerrain && mTerrain)
        {
            Math::Vec3 local = Math::Vec3(terrainInverse * Math::Vec4(leftPosition, 1.0f));
            local.y = mTerrain->heightAt(local.x, local.z);
            leftPosition = Math::Vec3(terrainTransform * Math::Vec4(local, 1.0f));
            local = Math::Vec3(terrainInverse * Math::Vec4(rightPosition, 1.0f));
            local.y = mTerrain->heightAt(local.x, local.z);
            rightPosition = Math::Vec3(terrainTransform * Math::Vec4(local, 1.0f));
            leftPosition.y += mSurfaceOffset;
            rightPosition.y += mSurfaceOffset;
        }
        data.positions.push_back(Math::Vec3(inverseOwner * Math::Vec4(leftPosition, 1.0f)));
        data.positions.push_back(Math::Vec3(inverseOwner * Math::Vec4(rightPosition, 1.0f)));
        data.normals.push_back(Math::Vec3(0, 1, 0));
        data.normals.push_back(Math::Vec3(0, 1, 0));
        const f32 v = value.distance / mTextureRepeat;
        data.uvs.push_back(Math::Vec2(0.0f, v));
        data.uvs.push_back(Math::Vec2(1.0f, v));
    }
    for (u32 i = 0; i + 1 < path.size(); ++i)
    {
        const u32 a = i * 2;
        const u32 quad[6] = {a, a + 2, a + 1, a + 1, a + 2, a + 3};
        data.indices.insert(data.indices.end(), quad, quad + 6);
    }
    // Scene::buildRenderList() always submits Road with mMaterial as an
    // override. Keep only an authored placeholder in the generated Mesh and
    // never copy mMaterial's live paramsBuffer into it: destroying a previous
    // rebuilt mesh would otherwise release the same UBO still owned by Road,
    // after which its tint/UV parameters read unrelated recycled GPU data.
    Material meshMaterial = mMaterial;
    meshMaterial.paramsBuffer = BufferHandle();
    meshMaterial.pipeline = PipelineHandle();
    meshMaterial.paramsDirty = true;
    data.materials.push_back(std::move(meshMaterial));
    Assets().computeBounds(data);
    Assets().computeSubMeshBounds(data);
    mMesh = Assets().createMesh(data);
}

} // namespace Radion
