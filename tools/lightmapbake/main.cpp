#include "PCH.h"

#include "AssetManager.h"
#include "AssetPaths.h"
#include "Engine.h"
#include "FileSystem.h"
#include "LightmapBakePass.h"
#include "LightmapUnwrapper.h"
#include "MaterialManager.h"
#include "Sky.h"

#include "Math.h"
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>

using namespace Radion;

namespace
{
// Set from the SIGINT handler, which may do nothing else - the unwrap's
// progress callback is what reads it and tells xatlas to stop. Packing a
// large atlas is one call that can run for many minutes with nothing else
// getting a turn, so without this the only way out of a bad texel density
// is killing the process.
std::atomic<bool> gInterrupted{false};

void onInterrupt(int)
{
    gInterrupted.store(true, std::memory_order_relaxed);
}

// xatlas reports the same stage repeatedly as it climbs from 0 to 100 - only
// worth a log line every time the stage changes or the percentage moves by
// a whole decile, or a multi-million-triangle mesh drowns the console.
// Returning false aborts the unwrap; see LightmapUnwrapSettings.
bool logUnwrapProgress(const char* stage, u32 percent, void* userData)
{
    static std::string lastStage;
    static u32 lastDecile = 0xFFFFFFFFu;
    if (gInterrupted.load(std::memory_order_relaxed))
        return false;
    const u32 decile = percent / 10;
    if (stage == lastStage && decile == lastDecile)
        return true;
    lastStage = stage;
    lastDecile = decile;
    (void)userData;
    Log::info("LightmapBake: unwrap %s %u%%", stage, percent);
    return true;
}

// Every field optional; anything missing keeps the LightmapBakeSettings
// default, so a preset only has to spell out what it wants to change.
void readBakeSettings(const nlohmann::json& node, LightmapBakeSettings& settings, u32& resolution)
{
    if (auto it = node.find("resolution"); it != node.end())
        resolution = it->get<u32>();
    if (auto it = node.find("bias"); it != node.end())
        settings.bias = it->get<f32>();
    // "ambient" takes either a single number (grey) or [r, g, b] - presets
    // written before the sky term became a colour keep working as the grey
    // they always meant.
    if (auto it = node.find("ambient"); it != node.end())
    {
        if (it->is_array() && it->size() == 3)
            settings.ambient = ::Radion::Math::vec3((*it)[0].get<f32>(), (*it)[1].get<f32>(),
                                         (*it)[2].get<f32>());
        else
            settings.ambient = ::Radion::Math::vec3(it->get<f32>());
    }
    if (auto it = node.find("ambientGround"); it != node.end())
        settings.ambientGround = it->get<f32>();
    if (auto it = node.find("filterRadius"); it != node.end())
        settings.filterRadius = it->get<f32>();
    if (auto it = node.find("sunAngularRadius"); it != node.end())
        settings.sunAngularRadius = it->get<f32>();
    if (auto it = node.find("sampleCount"); it != node.end())
        settings.sampleCount = it->get<u32>();
    if (auto it = node.find("shadowResolution"); it != node.end())
        settings.shadowResolution = it->get<u32>();
}

::Radion::Math::vec3 readVec3(const nlohmann::json& node, const ::Radion::Math::vec3& fallback)
{
    if (!node.is_array() || node.size() != 3)
        return fallback;
    return ::Radion::Math::vec3(node[0].get<f32>(), node[1].get<f32>(), node[2].get<f32>());
}
}

int main(int argc, char** argv)
{
    const std::string settingsFile = argc > 1 ? argv[1] : "lightmapbake_settings.json";

    std::signal(SIGINT, onInterrupt);

    Engine engine;
    EngineConfig config;
    config.title = "Radion Lightmap Bake";
    config.visible = false;
    if (!engine.initialize(config))
    {
        Log::error("LightmapBake: failed to initialize the engine");
        return 1;
    }
    Log::setMode(LogMode::Verbose);

    FileSystem& files = FileSystem::getSingleton();
    const std::string assetDirectory = resolveAssetDirectory(RADION_ASSET_DIR);
    files.addSearchPath(assetDirectory);
    files.addSearchPath(assetDirectory + "/shaders");

    // Shaders always come from the engine's own assets; the mesh being
    // baked (and whatever it needs, e.g. Bistro's textures) usually lives
    // somewhere else entirely - one file per scene, added before the mesh
    // load so material texture lookups can find it.
    const std::string text = files.readText(settingsFile);
    if (text.empty())
    {
        Log::error("LightmapBake: could not read '%s'", settingsFile.c_str());
        engine.shutdown();
        return 1;
    }

    nlohmann::json root;
    try
    {
        root = nlohmann::json::parse(text);
    }
    catch (const std::exception& error)
    {
        Log::error("LightmapBake: '%s' is not valid JSON: %s", settingsFile.c_str(), error.what());
        engine.shutdown();
        return 1;
    }

    for (const nlohmann::json& path : root.value("searchPaths", nlohmann::json::array()))
        if (path.is_string())
            files.addSearchPath(path.get<std::string>());

    const std::string meshFile = root.value("mesh", "");
    const std::string output = root.value("output", "lightmap");
    const std::string outputDir =
        root.value("outputDir", assetDirectory + "/lightmaps");
    {
        std::error_code error;
        std::filesystem::create_directories(outputDir, error);
        if (!std::filesystem::is_directory(outputDir, error))
        {
            Log::error("LightmapBake: could not create output directory '%s'", outputDir.c_str());
            engine.shutdown();
            return 1;
        }
    }
    if (meshFile.empty())
    {
        Log::error("LightmapBake: settings file has no 'mesh'");
        engine.shutdown();
        return 1;
    }
    files.addSearchPath(outputDir);

    MeshData source;
    if (!Assets().importMesh(meshFile, source))
    {
        Log::error("LightmapBake: could not load '%s'", meshFile.c_str());
        engine.shutdown();
        return 1;
    }
    if (source.materials.empty())
    {
        Material material;
        material.flags = MaterialLit | MaterialCastShadow | MaterialReceiveShadow;
        source.materials.push_back(material);
        for (SubMesh& submesh : source.submeshes)
            submesh.materialSlot = 0;
    }

    // A mesh file carries only material NAMES; every texture, colour and flag
    // lives in the companion .mat. Without loading it the bake runs on the
    // placeholder materials importMesh() synthesizes - no albedo, and the
    // default pale base colour - and then saves those as the baked material
    // set, so the scene comes back untextured with the lightmap multiplied
    // into flat grey. Load replaces the list positionally, so it has to
    // happen here: before the unwrap copies the materials into its output.
    std::string materialFile = root.value("material", std::string());
    if (materialFile.empty())
    {
        const usize dot = meshFile.find_last_of('.');
        materialFile = (dot == std::string::npos ? meshFile : meshFile.substr(0, dot)) + ".mat";
    }
    if (!MaterialManager::getSingleton().load(materialFile, source.materials))
        Log::warning("LightmapBake: no material file at '%s' - baking with placeholder materials, "
                    "the result will have no albedo",
                    materialFile.c_str());

    // The mesh file's own units, not necessarily what the scene renders it
    // at - a raw FBX export (Bistro, say) commonly needs shrinking to match
    // the rest of a scene authored in metres. The saved mesh keeps the
    // file's original coordinates untouched; only texel density and the
    // bake's own shadow frustum need to know about the mismatch.
    const f32 sceneScale = root.value("scale", 1.0f);

    LightmapUnwrapSettings unwrapSettings;
    const nlohmann::json unwrapNode = root.value("unwrap", nlohmann::json::object());
    unwrapSettings.resolution = unwrapNode.value("resolution", unwrapSettings.resolution);
    unwrapSettings.padding = unwrapNode.value("padding", unwrapSettings.padding);
    // texelsPerUnit in the settings file means texels per real-world unit;
    // xatlas measures against the mesh's own (unscaled) coordinates, so it
    // needs the density scaled up by the same factor the mesh will later be
    // scaled down by.
    unwrapSettings.texelsPerUnit =
        unwrapNode.value("texelsPerUnit", unwrapSettings.texelsPerUnit) * sceneScale;
    unwrapSettings.progress = logUnwrapProgress;

    MeshData data;
    if (!LightmapUnwrapper().unwrap(source, data, unwrapSettings))
    {
        if (gInterrupted.load(std::memory_order_relaxed))
            Log::info("LightmapBake: interrupted, nothing written");
        else
            Log::error("LightmapBake: unwrap failed for '%s'", meshFile.c_str());
        engine.shutdown();
        return 1;
    }
    u32 lightmapPages = 0;
    for (const SubMesh& submesh : data.submeshes)
        lightmapPages = ::Radion::Math::max(lightmapPages, submesh.lightmapPage + 1);
    if (lightmapPages > 1)
    {
        // bake() has no per-page filter yet - it would draw every submesh
        // into the same target using UV coordinates meant for different
        // pages, on top of each other.
        Log::error("LightmapBake: unwrap generated %u pages, this tool only bakes a single page",
                  lightmapPages);
        Log::error("LightmapBake: set unwrap.resolution to 0 - that is the only value that "
                  "guarantees one page, sized from unwrap.texelsPerUnit (currently %.1f). "
                  "Lower texelsPerUnit if the resulting atlas comes out too large; raising it "
                  "makes the atlas bigger, not smaller.",
                  static_cast<double>(unwrapSettings.texelsPerUnit));
        engine.shutdown();
        return 1;
    }
    Assets().computeBounds(data);
    Assets().computeSubMeshBounds(data);
    MeshHandle mesh = Assets().createMesh(data);
    if (!mesh.valid())
    {
        Log::error("LightmapBake: could not upload the unwrapped mesh");
        engine.shutdown();
        return 1;
    }

    const nlohmann::json sunNode = root.value("sun", nlohmann::json::object());
    engine.sky().automaticSun = false;
    engine.sky().sunAzimuth = sunNode.value("azimuth", engine.sky().sunAzimuth);
    engine.sky().sunElevation = sunNode.value("elevation", engine.sky().sunElevation);
    engine.sky().updateSun();
    const ::Radion::Math::vec3 sunColor =
        readVec3(sunNode.value("color", nlohmann::json::array()), ::Radion::Math::vec3(1.0f, 0.95f, 0.85f));
    const f32 sunIntensity = sunNode.value("intensity", 1.0f);
    const ::Radion::Math::vec3 lightColor = sunColor * sunIntensity;
    const ::Radion::Math::vec3 lightDirection = -engine.sky().sunDirection;

    const Mesh* uploadedMesh = Assets().getMesh(mesh);
    const ::Radion::Math::mat4 model = ::Radion::Math::scale(::Radion::Math::mat4(1.0f), ::Radion::Math::vec3(sceneScale));

    LightmapBakePass baker;
    std::string appliedPreset;
    for (const nlohmann::json& preset : root.value("presets", nlohmann::json::array()))
    {
        const std::string name = preset.value("name", "preset");
        u32 resolution = 512;
        LightmapBakeSettings settings;
        readBakeSettings(preset, settings, resolution);

        const auto start = std::chrono::steady_clock::now();
        const bool baked = baker.bake(mesh, model, data.bounds, lightDirection, lightColor,
                                      resolution, settings);
        const std::string pngFile = output + "_" + name + ".png";
        const bool saved = baked && baker.save(outputDir + "/" + pngFile);
        const auto elapsed = std::chrono::steady_clock::now() - start;
        const f64 seconds = std::chrono::duration<f64>(elapsed).count();

        if (!saved)
        {
            Log::error("LightmapBake: preset '%s' failed", name.c_str());
            continue;
        }
        Log::info("LightmapBake: preset '%s' -> %s (%ux%u atlas, %u shadow map, %u sample(s), "
                 "%.2fs)",
                 name.c_str(), pngFile.c_str(), resolution, resolution,
                 settings.shadowResolution ? settings.shadowResolution : resolution,
                 settings.sampleCount, seconds);

        if (preset.value("apply", false) && uploadedMesh)
        {
            const TextureHandle texture = Assets().reloadTexture(pngFile, ColorSpace::Linear, true);
            if (texture.valid())
            {
                LightmapBakePass::applyToMaterials(data.materials, uploadedMesh->colorLayout, texture,
                                                   pngFile);
                const bool meshSaved =
                    Assets().saveMesh(data, outputDir + "/" + output + "_baked.rmesh");
                const bool materialsSaved = MaterialManager::getSingleton().save(
                    outputDir + "/" + output + "_baked.mat", data.materials);
                if (meshSaved && materialsSaved)
                    appliedPreset = name;
                else
                    Log::warning("LightmapBake: preset '%s' baked but failed to persist mesh=%d material=%d",
                                name.c_str(), meshSaved, materialsSaved);
            }
        }
    }

    if (!appliedPreset.empty())
        Log::info("LightmapBake: '%s_baked.rmesh'/'.mat' written from preset '%s'", output.c_str(),
                  appliedPreset.c_str());

    engine.shutdown();
    return 0;
}
