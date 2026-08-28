#include "Prefab.h"

#include "ByteArray.h"
#include "FileSystem.h"
#include "GameObject.h"
#include "Log.h"
#include "Scene.h"
#include "SceneSerializer.h"

namespace Radion
{

Prefab::Prefab()
{
}

bool Prefab::load(const std::string& path)
{
    mLoaded = false;
    mPath.clear();

    const ByteArray file = FileSystem::getSingleton().readBinary(path);
    if (file.size() == 0)
    {
        Log::warning("Prefab: '%s' is missing or empty", path.c_str());
        return false;
    }

    const nlohmann::json parsed = nlohmann::json::parse(
        file.data(), file.data() + file.size(), nullptr, false);
    if (parsed.is_discarded())
    {
        Log::error("Prefab: '%s' is not valid json", path.c_str());
        return false;
    }

    mData = parsed;
    mPath = path;
    mLoaded = true;
    return true;
}

void Prefab::loadFromJson(const nlohmann::json& data)
{
    mData = data;
    mPath.clear();
    mLoaded = true;
}

void Prefab::saveFromObject(GameObject& object)
{
    SceneSerializer serializer;
    mData = serializer.subtreeToJson(object);
    mLoaded = !mData.is_null();
}

bool Prefab::saveToFile(const std::string& path, GameObject& object)
{
    saveFromObject(object);
    if (!mLoaded)
        return false;
    if (!FileSystem::getSingleton().writeText(path, mData.dump(2)))
        return false;
    mPath = path;
    return true;
}

GameObject* Prefab::instantiate(Scene& scene, GameObject* parent, SceneLoadResult& result) const
{
    if (!mLoaded)
        return nullptr;
    SceneSerializer serializer;
    return serializer.subtreeFromJson(mData, scene, parent, result);
}

bool Prefab::valid() const
{
    return mLoaded;
}

const nlohmann::json& Prefab::data() const
{
    return mData;
}

const std::string& Prefab::path() const
{
    return mPath;
}

} // namespace Radion
