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

// One saved GameObject subtree, ready to be dropped into a scene as many
// times as wanted. The document is exactly what SceneSerializer writes for
// a subtree, so a component the serializer can read and write is one a
// prefab carries for free.
//
// Instances are independent copies, not links: editing the file does not
// reach instances already placed.
class Prefab
{
public:
    Prefab();

    bool load(const std::string& path);
    void loadFromJson(const nlohmann::json& data);

    // Captures `object` and its descendants. saveToFile() writes what
    // saveFromObject() captured, so the prefab is usable without a reload.
    void saveFromObject(GameObject& object);
    bool saveToFile(const std::string& path, GameObject& object);

    // Null on failure; check result for why.
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
