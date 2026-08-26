#include "PCH.h"

#include "NavMeshSurface.h"

#include "AssetManager.h"
#include "GameObject.h"
#include "MeshRenderer.h"

#include <chrono>

namespace Radion
{

NavMeshSurface::NavMeshSurface() : Component(Type)
{
}

AI::NavMeshConfig& NavMeshSurface::config()
{
    return mConfig;
}

const AI::NavMeshConfig& NavMeshSurface::config() const
{
    return mConfig;
}

bool NavMeshSurface::built() const
{
    return mNavMesh.valid();
}

const AI::NavMesh& NavMeshSurface::navMesh() const
{
    return mNavMesh;
}

f32 NavMeshSurface::lastBuildMilliseconds() const
{
    return mLastBuildMilliseconds;
}

void NavMeshSurface::setGroundSeed(const Math::Vec3& worldPosition)
{
    mGroundSeed = worldPosition;
    mGroundSeedSet = true;
}

const Math::Vec3& NavMeshSurface::groundSeed() const
{
    return mGroundSeed;
}

bool NavMeshSurface::saveNavData(const std::string& filename)
{
    if (!mNavMesh.save(filename))
        return false;
    mNavDataFile = filename;
    return true;
}

bool NavMeshSurface::loadNavData(const std::string& filename)
{
    if (!mNavMesh.load(filename))
        return false;
    mNavDataFile = filename;
    return true;
}

const std::string& NavMeshSurface::navDataFile() const
{
    return mNavDataFile;
}

bool NavMeshSurface::build()
{
    const auto started = std::chrono::steady_clock::now();

    GameObject* object = owner();
    MeshRenderer* renderer = object ? object->getComponent<MeshRenderer>() : nullptr;
    if (!renderer)
    {
        Log::error("NavMeshSurface: '%s' has no MeshRenderer to bake from",
                   object ? object->name().c_str() : "?");
        return false;
    }

    MeshData meshData;
    if (!Assets().buildMeshData(Assets().meshDesc(renderer->mesh()), meshData) ||
        meshData.indices.size() < 3)
    {
        Log::error("NavMeshSurface: could not read mesh data on '%s'", object->name().c_str());
        return false;
    }

    // Recast works in one space, and the level is drawn through its owner's
    // transform - baking the vertices through it here is what keeps the
    // surface under the geometry instead of at the origin.
    const Math::Mat4 transform = object->globalTransform();
    for (Math::Vec3& position : meshData.positions)
        position = Math::Vec3(transform * Math::Vec4(position, 1.0f));

    // Recast indexes with signed ints; meshes store unsigned.
    std::vector<s32> indices(meshData.indices.begin(), meshData.indices.end());

    const Math::Vec3 seed = mGroundSeedSet ? mGroundSeed : object->globalPosition();
    const bool ok =
        mNavMesh.build(&meshData.positions[0].x, static_cast<s32>(meshData.positions.size()),
                       indices.data(), static_cast<s32>(indices.size() / 3), mConfig, &seed);

    const auto finished = std::chrono::steady_clock::now();
    mLastBuildMilliseconds =
        std::chrono::duration<f32, std::milli>(finished - started).count();

    if (ok)
        Log::info("NavMeshSurface: baked '%s' in %.1f ms (%zu surface triangles)",
                  object->name().c_str(), static_cast<f64>(mLastBuildMilliseconds),
                  mNavMesh.debugTriangles().size() / 3);
    else
        Log::error("NavMeshSurface: bake failed on '%s'", object->name().c_str());
    return ok;
}

} // namespace Radion
