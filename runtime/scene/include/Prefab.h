#ifndef RADION_PREFAB_H
#define RADION_PREFAB_H

#include "Types.h"

#include <nlohmann/json.hpp>
#include <string>

namespace Radion
{

class GameObject;
class Scene;
struct SceneLoadResult;

// Instances are independent copies, not links.
class Prefab
{
public:
    Prefab();

    bool load(const std::string& path);
    void loadFromJson(const nlohmann::json& data);

    void saveFromObject(GameObject& object);
    bool saveToFile(const std::string& path, GameObject& object);

    GameObject* instantiate(Scene& scene, GameObject* parent, SceneLoadResult& result) const;

    bool valid() const;
    const nlohmann::json& data() const;
    const std::string& path() const;

private:
    nlohmann::json mData;
    std::string mPath;
    bool mLoaded = false;
};

} // namespace Radion

#endif // RADION_PREFAB_H
