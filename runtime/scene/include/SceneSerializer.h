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

// Render-wide settings that live on Engine, not Scene (Renderer/Lighting own
// the structs; PostProcessStack/LensFlarePass are Engine's own) - grouped
// here only so toJson()/fromJson() can pass "the stuff a scene file should
// also carry" through one optional pointer instead of four. Any field left
// null is simply not written/read; an old scene file with no
// "renderSettings" section loads exactly as before this existed.
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

// One thing that went wrong (or was ignored) while reading or writing a
// scene, with enough context for a UI to point at the exact spot without
// depending on stdout - the editor shows this list directly, it does not go
// digging through logs.
struct SceneDiagnostic
{
    SceneDiagnosticSeverity severity = SceneDiagnosticSeverity::Warning;
    // A json path like "scene.objects[3].parent" - always into the document
    // being read/written, not a C++ symbol name.
    std::string jsonPath;
    std::string message;
};

// What a load produced besides the Scene itself. Never owns the Scene: see
// SceneSerializer::load()'s comment for why.
struct SceneLoadResult
{
    std::vector<SceneDiagnostic> diagnostics;

    // False on any Error diagnostic. Warnings alone still count as success -
    // an asset missing its texture opens with a placeholder and a warning,
    // it does not refuse to open.
    bool success() const;

    void addWarning(const std::string& jsonPath, const std::string& message);
    void addError(const std::string& jsonPath, const std::string& message);
};

// Reads and writes a Radion::Scene as JSON. The same toJson/fromJson pair
// backs Save/Open on disk, in-memory snapshots for undo/duplicate, and
// tests - nothing here is disk-specific except save()/load() themselves.
//
// fromJson()/load() never mutate the Scene passed in until every object,
// every reference and every diagnostic has been produced from the document;
// on any fatal error `out` is left exactly as it was received. This is what
// makes loading transactional (see docs/PLANO_SERIALIZACAO_CENA.md,
// principle 5): the caller passes a fresh, temporary Scene, checks
// result.success(), and only then swaps it in for the one currently open.
// There is no SceneLoadResult::scene to hand back instead - Scene has no
// copy or move constructor, and the engine does not pass ownership through
// smart pointers - so the caller owns `out` from the start and this only
// ever writes into it.
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

    // Deep-copies `source` and every descendant into `out` (ordinarily the
    // same Scene `source` already lives in - Hierarchy's Duplicate), fresh
    // ids throughout via Scene::reserveId(), attached under `newParent`
    // (root() if null). The same write/read this class already does for
    // Save/Load and Undo, just scoped to one subtree - a component type this
    // class knows how to read/write, it knows how to duplicate for free.
    // Null on failure; check result for why.
    GameObject* cloneObject(GameObject& source, Scene& out, GameObject* newParent,
                            SceneLoadResult& result) const;

    // The two halves of cloneObject(), with the document in the middle
    // exposed so it can go to disk and come back - what Prefab is built on.
    //
    // subtreeToJson() writes `source` and every descendant as a standalone
    // document in the same format save() writes. Empty json if `source` is
    // not in a Scene.
    nlohmann::json subtreeToJson(GameObject& source) const;

    // Reads such a document into `out`, minting fresh ids so it never
    // collides with what is already there, attached under `newParent`
    // (root() if null). Only the objects are read: the scene-level settings
    // of wherever the subtree came from are not applied to `out`. Null on
    // failure; check result for why.
    GameObject* subtreeFromJson(const nlohmann::json& document, Scene& out,
                                GameObject* newParent, SceneLoadResult& result) const;
};

} // namespace Radion

#endif // RADION_SCENE_SERIALIZER_H
