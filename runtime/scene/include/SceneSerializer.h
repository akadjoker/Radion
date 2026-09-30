#ifndef RADION_SCENE_SERIALIZER_H
#define RADION_SCENE_SERIALIZER_H

#include "Types.h"

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace Radion
{

class Scene;
class GameObject;
struct CascadeShadowSettings;
struct ShadowAtlasSettings;
class PostProcessStack;
class LensFlarePass;
class EnvironmentProbe;
class Lighting;
class VolumetricPass;
struct SkySettings;
struct RenderResolution;
class ParticleRenderQueue;

// Engine-owned settings grouped so toJson()/fromJson() take one optional pointer. A null field is not written/read.
struct SceneRenderSettings
{
    CascadeShadowSettings* shadows = nullptr;
    ShadowAtlasSettings* shadowAtlas = nullptr;
    PostProcessStack* postProcess = nullptr;
    LensFlarePass* lensFlare = nullptr;
    EnvironmentProbe* environmentProbe = nullptr;
    Lighting* lighting = nullptr;
    VolumetricPass* volumetric = nullptr;
    SkySettings* sky = nullptr;
    RenderResolution* renderResolution = nullptr;
    ParticleRenderQueue* particles = nullptr;
};

enum class SceneDiagnosticSeverity : u8
{
    Warning,
    Error
};

// Carries enough context for a UI to point at the exact spot.
struct SceneDiagnostic
{
    SceneDiagnosticSeverity severity = SceneDiagnosticSeverity::Warning;
    // Path into the document being read/written, e.g. "scene.objects[3].parent".
    std::string jsonPath;
    std::string message;
};

// Never owns the Scene: see SceneSerializer::load().
struct SceneLoadResult
{
    std::vector<SceneDiagnostic> diagnostics;

    // Warnings alone still count as success.
    bool success() const;

    void addWarning(const std::string& jsonPath, const std::string& message);
    void addError(const std::string& jsonPath, const std::string& message);
};

// One toJson/fromJson pair backs disk, undo/duplicate snapshots and tests.
// fromJson()/load() never mutate `out` until everything is produced; on a fatal error it is left as received (transactional load).
// The caller owns `out` (Scene has no copy/move), passes a temporary one, checks result.success(), then swaps it in.
class SceneSerializer
{
public:
    bool save(const Scene& scene, const std::string& filename,
             const SceneRenderSettings* settings = nullptr) const;
    bool load(const std::string& filename, Scene& out, SceneLoadResult& result,
             const SceneRenderSettings* settings = nullptr) const;

    nlohmann::json toJson(const Scene& scene, const SceneRenderSettings* settings = nullptr) const;
    nlohmann::json renderSettingsToJson(const SceneRenderSettings& settings) const;
    bool fromJson(const nlohmann::json& root, Scene& out, SceneLoadResult& result,
                  const SceneRenderSettings* settings = nullptr) const;

    // Deep-copies source and descendants into `out` with fresh ids via Scene::reserveId(), under newParent (root() if null). Null on failure.
    GameObject* cloneObject(GameObject& source, Scene& out, GameObject* newParent,
                            SceneLoadResult& result) const;

    // subtreeToJson() writes a standalone document in save()'s format (what Prefab is built on). Empty json if source is not in a Scene.
    nlohmann::json subtreeToJson(GameObject& source) const;

    // Mints fresh ids, attaches under newParent (root() if null). Only objects are read, not scene-level settings. Null on failure.
    GameObject* subtreeFromJson(const nlohmann::json& document, Scene& out,
                                GameObject* newParent, SceneLoadResult& result) const;
};

} // namespace Radion

#endif // RADION_SCENE_SERIALIZER_H
