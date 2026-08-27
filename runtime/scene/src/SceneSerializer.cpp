#include "PCH.h"

#include "SceneSerializer.h"

#include "Animation.h"
#include "AssetManager.h"
#include "Billboard.h"
#include "BoneAttachment.h"
#include "Camera.h"
#include "CameraControllers.h"
#include "CharacterController.h"
#include "Collider.h"
#include "Containers.h"
#include "FileSystem.h"
#include "Forest.h"
#include "GameObject.h"
#include "Grass.h"
#include "Hair.h"
#include "LensFlarePass.h"
#include "Light.h"
#include "Landscape.h"
#include "Lighting.h"
#include "MaterialManager.h"
#include "MaterialSlotNames.h"
#include "MeshRenderer.h"
#include "Ocean.h"
#include "ParticleEffect.h"
#include "ParticleEmitter.h"
#include "ParticlePass.h"
#include "PostProcess.h"
#include "ReflectionProbe.h"
#include "RibbonTrail.h"
#include "Road.h"
#include "Scene.h"
#include "Shadows.h"
#include "Sky.h"
#include "NavMeshSurface.h"
#include "SelfDestroy.h"
#include "ZenBehaviour.h"
#include "Waypoints.h"
#include "Text3D.h"
#include "Terrain.h"
#include "VoxelWorldComponent.h"
#include "VolumetricPass.h"

#include <cmath>
#include <type_traits>

namespace Radion
{

namespace
{

constexpr const char* kFormatName = "radion-scene";
constexpr u32 kFormatVersion = 1;
// GameObject::setScale() itself rejects a component this close to zero (see
// GameObject.cpp) - kept in step so a scale the runtime would silently
// reject is instead a load-time diagnostic with a json path.
constexpr f32 kMinScaleComponent = 0.000001f;

std::string indexPath(usize index)
{
    return "scene.objects[" + std::to_string(index) + "]";
}

// A json value coming from parse() of "5" is number_unsigned; the identical
// value built in C++ as a plain `int` (json field = 5) is number_integer -
// nlohmann tells the two apart by how the value was produced, not by
// whether it's actually negative. An id/version has no business caring
// about that distinction, only about "is this a whole number >= 0".
bool readNonNegativeInteger(const nlohmann::json& field, u64& out)
{
    if (field.is_number_unsigned())
    {
        out = field.get<u64>();
        return true;
    }
    if (field.is_number_integer())
    {
        const s64 value = field.get<s64>();
        if (value < 0)
            return false;
        out = static_cast<u64>(value);
        return true;
    }
    return false;
}

bool readVec3(const nlohmann::json& array, glm::vec3& out)
{
    if (!array.is_array() || array.size() != 3)
        return false;
    for (const nlohmann::json& component : array)
        if (!component.is_number())
            return false;
    out = glm::vec3(array[0].get<f32>(), array[1].get<f32>(), array[2].get<f32>());
    return std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.z);
}

// File order is [x, y, z, w]; glm::quat's own constructor takes (w, x, y, z)
// - swapped from the file order, never confuse the two here or on write.
bool readQuat(const nlohmann::json& array, glm::quat& out)
{
    if (!array.is_array() || array.size() != 4)
        return false;
    for (const nlohmann::json& component : array)
        if (!component.is_number())
            return false;
    const f32 x = array[0].get<f32>();
    const f32 y = array[1].get<f32>();
    const f32 z = array[2].get<f32>();
    const f32 w = array[3].get<f32>();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w))
        return false;
    out = glm::quat(w, x, y, z);
    // Matches GameObject's own valid() tolerance (GameObject.cpp): not
    // degenerate, not necessarily already unit-length - setRotation()
    // normalizes it regardless.
    return glm::dot(out, out) > 0.000001f;
}

bool readVec4(const nlohmann::json& array, glm::vec4& out)
{
    if (!array.is_array() || array.size() != 4)
        return false;
    for (const nlohmann::json& component : array)
        if (!component.is_number())
            return false;
    out = glm::vec4(array[0].get<f32>(), array[1].get<f32>(), array[2].get<f32>(),
                    array[3].get<f32>());
    return std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.z) &&
           std::isfinite(out.w);
}

nlohmann::json writeVec4(const glm::vec4& v)
{
    return nlohmann::json{v.x, v.y, v.z, v.w};
}

nlohmann::json writeVec3(const glm::vec3& v)
{
    return nlohmann::json{v.x, v.y, v.z};
}

bool readFloatField(const nlohmann::json& json, const char* key, f32& out, const std::string& path,
                    SceneLoadResult& result)
{
    const auto field = json.find(key);
    if (field == json.end() || !field->is_number())
    {
        result.addError(path + "." + key, "missing or not a number");
        return false;
    }
    const f32 value = field->get<f32>();
    if (!std::isfinite(value))
    {
        result.addError(path + "." + key, "must be finite");
        return false;
    }
    out = value;
    return true;
}

// A number field that always has a sensible fallback, for optional values a
// caller is about to pass straight into a setter anyway - not worth a
// SceneLoadResult entry when it is missing, unlike readFloatField()'s
// always-written fields.
f32 readNumberOr(const nlohmann::json& json, const char* key, f32 fallback)
{
    const auto field = json.find(key);
    if (field != json.end() && field->is_number())
    {
        const f32 value = field->get<f32>();
        if (std::isfinite(value))
            return value;
    }
    return fallback;
}

bool readBoolOr(const nlohmann::json& json, const char* key, bool fallback)
{
    const auto field = json.find(key);
    return field != json.end() && field->is_boolean() ? field->get<bool>() : fallback;
}

bool readVec4Field(const nlohmann::json& json, const char* key, glm::vec4& out,
                   const std::string& path, SceneLoadResult& result)
{
    const auto field = json.find(key);
    if (field == json.end() || !readVec4(*field, out))
    {
        result.addError(path + "." + key, "expected [x, y, z, w] of finite numbers");
        return false;
    }
    return true;
}

bool readVec3Field(const nlohmann::json& json, const char* key, glm::vec3& out,
                   const std::string& path, SceneLoadResult& result)
{
    const auto field = json.find(key);
    if (field == json.end() || !readVec3(*field, out))
    {
        result.addError(path + "." + key, "expected [x, y, z] of finite numbers");
        return false;
    }
    return true;
}

// ------------------------------------------------------------------- Camera

const char* cameraProjectionName(CameraProjection mode)
{
    return mode == CameraProjection::Perspective ? "Perspective" : "Orthographic";
}

bool cameraProjectionFromName(const std::string& name, CameraProjection& out)
{
    if (name == "Perspective")
    {
        out = CameraProjection::Perspective;
        return true;
    }
    if (name == "Orthographic")
    {
        out = CameraProjection::Orthographic;
        return true;
    }
    return false;
}

const char* environmentProbeContentName(EnvironmentProbe::Content content)
{
    switch (content)
    {
    case EnvironmentProbe::Content::FaceColors:
        return "FaceColors";
    case EnvironmentProbe::Content::Sky:
        return "Sky";
    case EnvironmentProbe::Content::SkyAndWorld:
        return "SkyAndWorld";
    }
    return "SkyAndWorld";
}

bool environmentProbeContentFromName(const std::string& name, EnvironmentProbe::Content& out)
{
    if (name == "FaceColors")
        out = EnvironmentProbe::Content::FaceColors;
    else if (name == "Sky")
        out = EnvironmentProbe::Content::Sky;
    else if (name == "SkyAndWorld")
        out = EnvironmentProbe::Content::SkyAndWorld;
    else
        return false;
    return true;
}

const char* environmentProbeRefreshName(EnvironmentProbe::Refresh refresh)
{
    switch (refresh)
    {
    case EnvironmentProbe::Refresh::Manual:
        return "Manual";
    case EnvironmentProbe::Refresh::Automatic:
        return "Automatic";
    case EnvironmentProbe::Refresh::Timed:
        return "Timed";
    }
    return "Automatic";
}

bool environmentProbeRefreshFromName(const std::string& name, EnvironmentProbe::Refresh& out)
{
    if (name == "Manual")
        out = EnvironmentProbe::Refresh::Manual;
    else if (name == "Automatic" || name == "Once")
        out = EnvironmentProbe::Refresh::Automatic;
    else if (name == "Timed")
        out = EnvironmentProbe::Refresh::Timed;
    else
        return false;
    return true;
}

nlohmann::json writeReflectionProbe(ReflectionProbe& probeComponent)
{
    const EnvironmentProbe& env = probeComponent.probe();
    nlohmann::json json;
    json["type"] = "ReflectionProbe";
    json["version"] = 1;
    json["active"] = probeComponent.active();
    json["resolution"] = env.resolution();
    json["extents"] = writeVec3(env.extents);
    json["influenceRadius"] = env.influenceRadius;
    json["intensity"] = env.intensity;
    json["content"] = environmentProbeContentName(env.content);
    json["refresh"] = environmentProbeRefreshName(env.refresh);
    json["interval"] = env.interval;
    json["nearPlane"] = env.nearPlane;
    json["farPlane"] = env.farPlane;
    return json;
}

void readReflectionProbe(GameObject& object, const nlohmann::json& json, const std::string& path,
                         SceneLoadResult& result)
{
    f32 resolution = 128.0f, influenceRadius = 0.0f, intensity = 1.0f, interval = 0.5f,
        nearPlane = 0.1f, farPlane = 1000.0f;
    glm::vec3 extents(0.0f);
    bool ok = true;
    ok &= readFloatField(json, "resolution", resolution, path, result);
    ok &= readVec3Field(json, "extents", extents, path, result);
    ok &= readFloatField(json, "influenceRadius", influenceRadius, path, result);
    ok &= readFloatField(json, "intensity", intensity, path, result);
    ok &= readFloatField(json, "interval", interval, path, result);
    ok &= readFloatField(json, "nearPlane", nearPlane, path, result);
    ok &= readFloatField(json, "farPlane", farPlane, path, result);
    if (!ok)
        return;

    EnvironmentProbe::Content content = EnvironmentProbe::Content::SkyAndWorld;
    const auto contentField = json.find("content");
    if (contentField == json.end() || !contentField->is_string() ||
        !environmentProbeContentFromName(contentField->get<std::string>(), content))
    {
        result.addError(path + ".content", "missing or not one of FaceColors/Sky/SkyAndWorld");
        return;
    }
    EnvironmentProbe::Refresh refresh = EnvironmentProbe::Refresh::Automatic;
    const auto refreshField = json.find("refresh");
    if (refreshField == json.end() || !refreshField->is_string() ||
        !environmentProbeRefreshFromName(refreshField->get<std::string>(), refresh))
    {
        result.addError(path + ".refresh", "missing or not one of Manual/Automatic/Timed");
        return;
    }

    ReflectionProbe* probeComponent = object.addComponent<ReflectionProbe>();
    if (!probeComponent)
    {
        result.addError(path, "object already has a ReflectionProbe component");
        return;
    }
    probeComponent->create(static_cast<u32>(resolution));
    EnvironmentProbe& env = probeComponent->probe();
    env.extents = extents;
    env.influenceRadius = influenceRadius;
    env.intensity = intensity;
    env.content = content;
    env.refresh = refresh;
    env.interval = interval;
    env.nearPlane = nearPlane;
    env.farPlane = farPlane;
    env.invalidate();

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean())
        probeComponent->setActive(activeField->get<bool>());
}

const char* oceanQualityName(OceanQuality quality)
{
    switch (quality)
    {
    case OceanQuality::SkyOnly: return "SkyOnly";
    case OceanQuality::Reflection: return "Reflection";
    case OceanQuality::ReflectionRefraction: return "ReflectionRefraction";
    }
    return "Reflection";
}

bool oceanQualityFromName(const std::string& name, OceanQuality& out)
{
    if (name == "SkyOnly") out = OceanQuality::SkyOnly;
    else if (name == "Reflection") out = OceanQuality::Reflection;
    else if (name == "ReflectionRefraction") out = OceanQuality::ReflectionRefraction;
    else return false;
    return true;
}

nlohmann::json writeOcean(Ocean& ocean)
{
    nlohmann::json json{{"type", "Ocean"}, {"version", 2}, {"active", ocean.active()}};
    json["size"] = ocean.size();
    json["segments"] = ocean.segments();
    json["quality"] = oceanQualityName(ocean.quality());
    json["waveCount"] = ocean.waveCount();
    json["waveScale"] = ocean.waveScale();
    json["steepness"] = ocean.steepness();
    json["timeScale"] = ocean.timeScale();
    json["level"] = ocean.level();
    json["shallowColor"] = writeVec3(ocean.shallowColor());
    json["deepColor"] = writeVec3(ocean.deepColor());
    json["absorptionDistance"] = ocean.absorptionDistance();
    json["roughness"] = ocean.roughness();
    json["specularStrength"] = ocean.specularStrength();
    json["normalMapEnabled"] = ocean.normalMapEnabled();
    json["normalMap"] = ocean.normalMapFile();
    json["normalOctaves"] = ocean.normalOctaves();
    json["normalScale"] = {ocean.normalScale1(), ocean.normalScale2()};
    json["normalStrength"] = ocean.normalStrength();
    json["normalSpeed"] = {ocean.normalSpeed1(), ocean.normalSpeed2()};
    json["foamEnabled"] = ocean.foamEnabled();
    json["foamTexture"] = ocean.foamTextureFile();
    json["foamScale"] = ocean.foamScale();
    json["foamStrength"] = ocean.foamStrength();
    json["foamDepth"] = ocean.foamDepth();
    json["foamCrest"] = ocean.foamCrest();
    json["fresnelDetail"] = ocean.fresnelDetail();
    json["fresnelMax"] = ocean.fresnelMax();
    json["fresnelBias"] = ocean.fresnelBias();
    json["fresnelScale"] = ocean.fresnelScale();
    json["fresnelPower"] = ocean.fresnelPower();
    json["minOpacity"] = ocean.minOpacity();
    json["reflectionDistortion"] = ocean.reflectionDistortion();
    json["reflectionStrength"] = ocean.reflectionStrength();
    json["refractionStrength"] = ocean.refractionStrength();
    json["colorStrength"] = ocean.colorStrength();
    json["underwaterColor"] = writeVec3(ocean.underwaterColor());
    json["debugMode"] = ocean.debugMode();
    json["waves"] = nlohmann::json::array();
    for (u32 i = 0; i < kOceanMaxWaves; ++i)
    {
        const OceanWave& wave = ocean.wave(i);
        json["waves"].push_back({{"direction", {wave.direction.x, wave.direction.y}},
                                  {"wavelength", wave.wavelength},
                                  {"amplitude", wave.amplitude}});
    }
    return json;
}

void readOcean(GameObject& object, const nlohmann::json& json, const std::string& path,
               SceneLoadResult& result)
{
    Ocean* ocean = object.addComponent<Ocean>();
    if (!ocean)
    {
        result.addError(path, "object already has an Ocean component");
        return;
    }
    const f32 size = readNumberOr(json, "size", 0.0f);
    const u32 segments = static_cast<u32>(glm::max(0.0f, readNumberOr(json, "segments", 0.0f)));
    if (size > 0.0f && segments > 0)
        ocean->build(size, segments);
    OceanQuality quality = OceanQuality::Reflection;
    const auto qualityField = json.find("quality");
    if (qualityField != json.end() && qualityField->is_string() &&
        oceanQualityFromName(qualityField->get<std::string>(), quality))
        ocean->setQuality(quality);
    ocean->setWaveCount(static_cast<u32>(glm::clamp(readNumberOr(json, "waveCount", 4.0f), 0.0f,
                                                    static_cast<f32>(kOceanMaxWaves))));
    ocean->setWaveScale(readNumberOr(json, "waveScale", ocean->waveScale()));
    ocean->setSteepness(readNumberOr(json, "steepness", ocean->steepness()));
    ocean->setTimeScale(readNumberOr(json, "timeScale", ocean->timeScale()));
    ocean->setLevel(readNumberOr(json, "level", ocean->level()));
    glm::vec3 color;
    if (json.find("shallowColor") != json.end() && readVec3(*json.find("shallowColor"), color)) ocean->setShallowColor(color);
    if (json.find("deepColor") != json.end() && readVec3(*json.find("deepColor"), color)) ocean->setDeepColor(color);
    ocean->setAbsorptionDistance(readNumberOr(json, "absorptionDistance", ocean->absorptionDistance()));
    ocean->setRoughness(readNumberOr(json, "roughness", ocean->roughness()));
    ocean->setSpecularStrength(readNumberOr(json, "specularStrength", ocean->specularStrength()));
    const auto boolOr = [&](const char* key, bool fallback) { const auto it = json.find(key); return it != json.end() && it->is_boolean() ? it->get<bool>() : fallback; };
    ocean->setNormalMapEnabled(boolOr("normalMapEnabled", ocean->normalMapEnabled()));
    const auto normalMap = json.find("normalMap");
    if (normalMap != json.end() && normalMap->is_string())
        ocean->setNormalMapFile(normalMap->get<std::string>());
    ocean->setNormalOctaves(static_cast<u32>(readNumberOr(json, "normalOctaves", ocean->normalOctaves())));
    const auto normalScale = json.find("normalScale");
    if (normalScale != json.end() && normalScale->is_array() && normalScale->size() == 2 &&
        (*normalScale)[0].is_number() && (*normalScale)[1].is_number())
        ocean->setNormalScale((*normalScale)[0].get<f32>(), (*normalScale)[1].get<f32>());
    ocean->setNormalStrength(readNumberOr(json, "normalStrength", ocean->normalStrength()));
    const auto normalSpeed = json.find("normalSpeed");
    if (normalSpeed != json.end() && normalSpeed->is_array() && normalSpeed->size() == 2 &&
        (*normalSpeed)[0].is_number() && (*normalSpeed)[1].is_number())
        ocean->setNormalSpeed((*normalSpeed)[0].get<f32>(), (*normalSpeed)[1].get<f32>());
    ocean->setFoamEnabled(boolOr("foamEnabled", ocean->foamEnabled()));
    const auto foamTexture = json.find("foamTexture");
    if (foamTexture != json.end() && foamTexture->is_string())
        ocean->setFoamTextureFile(foamTexture->get<std::string>());
    ocean->setFoamScale(readNumberOr(json, "foamScale", ocean->foamScale()));
    ocean->setFoamStrength(readNumberOr(json, "foamStrength", ocean->foamStrength()));
    ocean->setFoamDepth(readNumberOr(json, "foamDepth", ocean->foamDepth()));
    ocean->setFoamCrest(readNumberOr(json, "foamCrest", ocean->foamCrest()));
    ocean->setFresnelDetail(readNumberOr(json, "fresnelDetail", ocean->fresnelDetail()));
    ocean->setFresnelMax(readNumberOr(json, "fresnelMax", ocean->fresnelMax()));
    ocean->setFresnelBias(readNumberOr(json, "fresnelBias", ocean->fresnelBias()));
    ocean->setFresnelScale(readNumberOr(json, "fresnelScale", ocean->fresnelScale()));
    ocean->setFresnelPower(readNumberOr(json, "fresnelPower", ocean->fresnelPower()));
    ocean->setMinOpacity(readNumberOr(json, "minOpacity", ocean->minOpacity()));
    ocean->setReflectionDistortion(readNumberOr(json, "reflectionDistortion", ocean->reflectionDistortion()));
    ocean->setReflectionStrength(readNumberOr(json, "reflectionStrength", ocean->reflectionStrength()));
    ocean->setRefractionStrength(readNumberOr(json, "refractionStrength", ocean->refractionStrength()));
    ocean->setColorStrength(readNumberOr(json, "colorStrength", ocean->colorStrength()));
    if (json.find("underwaterColor") != json.end() && readVec3(*json.find("underwaterColor"), color)) ocean->setUnderwaterColor(color);
    ocean->setDebugMode(static_cast<s32>(readNumberOr(json, "debugMode", ocean->debugMode())));
    const auto waves = json.find("waves");
    if (waves != json.end() && waves->is_array())
        for (u32 i = 0; i < waves->size() && i < kOceanMaxWaves; ++i)
        {
            const auto& entry = (*waves)[i];
            if (!entry.is_object())
                continue;
            const auto direction = entry.find("direction");
            if (direction == entry.end() || !direction->is_array() || direction->size() != 2 ||
                !(*direction)[0].is_number() || !(*direction)[1].is_number())
                continue;
            const glm::vec2 dir(direction->at(0).get<f32>(), direction->at(1).get<f32>());
            ocean->setWave(i, dir, readNumberOr(entry, "wavelength", ocean->wave(i).wavelength),
                           readNumberOr(entry, "amplitude", ocean->wave(i).amplitude));
        }
    const auto active = json.find("active");
    if (active != json.end() && active->is_boolean()) ocean->setActive(active->get<bool>());
}

nlohmann::json writeMaterial(const Material& material);
bool readMaterial(const nlohmann::json& json, Material& out, const std::string& path,
                  SceneLoadResult& result);

nlohmann::json writeTerrainVegetation(const Terrain::VegetationSettings& settings)
{
    return {{"spacing", settings.spacing},
            {"density", settings.density},
            {"jitter", settings.jitter},
            {"minimumHeight", settings.minimumHeight},
            {"maximumHeight", settings.maximumHeight},
            {"maximumSlopeDegrees", settings.maximumSlopeDegrees},
            {"minimumScale", settings.minimumScale},
            {"maximumScale", settings.maximumScale},
            {"seed", settings.seed},
            {"maximumInstances", settings.maximumInstances}};
}

void readTerrainVegetation(const nlohmann::json& json, Terrain::VegetationSettings& settings)
{
    if (!json.is_object())
        return;
    settings.spacing = readNumberOr(json, "spacing", settings.spacing);
    settings.density = readNumberOr(json, "density", settings.density);
    settings.jitter = readNumberOr(json, "jitter", settings.jitter);
    settings.minimumHeight = readNumberOr(json, "minimumHeight", settings.minimumHeight);
    settings.maximumHeight = readNumberOr(json, "maximumHeight", settings.maximumHeight);
    settings.maximumSlopeDegrees =
        readNumberOr(json, "maximumSlopeDegrees", settings.maximumSlopeDegrees);
    settings.minimumScale = readNumberOr(json, "minimumScale", settings.minimumScale);
    settings.maximumScale = readNumberOr(json, "maximumScale", settings.maximumScale);
    u64 value = 0;
    const auto seed = json.find("seed");
    if (seed != json.end() && readNonNegativeInteger(*seed, value) &&
        value <= std::numeric_limits<u32>::max())
        settings.seed = static_cast<u32>(value);
    const auto maximum = json.find("maximumInstances");
    if (maximum != json.end() && readNonNegativeInteger(*maximum, value) &&
        value <= std::numeric_limits<u32>::max())
        settings.maximumInstances = static_cast<u32>(value);
}

nlohmann::json writeTerrain(Terrain& terrain)
{
    nlohmann::json json{{"type", "Terrain"}, {"version", 6}, {"active", terrain.active()}};
    json["heightmap"] = terrain.heightmapFile();
    json["vegetationMask"] = terrain.vegetationMaskFile();
    json["surfaceSplat"] = terrain.surfaceSplatFile();
    json["cellSize"] = terrain.cellSize();
    json["heightScale"] = terrain.heightScale();
    json["uvTiles"] = terrain.uvTiles();
    json["maxLod"] = terrain.maxLod();
    json["receiveShadows"] = terrain.receivesShadows();
    json["grassGeneration"] = writeTerrainVegetation(terrain.grassGenerationSettings());
    json["treeGeneration"] = writeTerrainVegetation(terrain.treeGenerationSettings());
    json["material"] = writeMaterial(terrain.material());
    return json;
}

nlohmann::json writeVoxelWorld(VoxelWorldComponent& voxelWorld)
{
    return nlohmann::json{{"type", "VoxelWorld"},
                          {"version", 1},
                          {"active", voxelWorld.active()},
                          {"seed", voxelWorld.seed()},
                          {"chunkRadius", voxelWorld.chunkRadius()},
                          {"minWorldY", voxelWorld.minWorldY()},
                          {"maxWorldY", voxelWorld.maxWorldY()},
                          {"waterLevel", voxelWorld.waterLevel()},
                          {"atlasFile", voxelWorld.atlasFile()},
                          {"originObjectId", voxelWorld.originObjectId()}};
}

nlohmann::json writeRoad(Road& road)
{
    nlohmann::json json{{"type", "Road"}, {"version", 2}, {"active", road.active()},
                        {"subdivisions", road.subdivisions()},
                        {"textureRepeat", road.textureRepeat()},
                        {"surfaceOffset", road.surfaceOffset()},
                        {"conformTerrain", road.conformTerrain()},
                        {"material", writeMaterial(road.material())}, {"points", nlohmann::json::array()}};
    if (road.terrain() && road.terrain()->owner())
        json["terrain"] = road.terrain()->owner()->id();
    for (usize i = 0; i < road.pointCount(); ++i)
    {
        GameObject* point = road.point(i);
        json["points"].push_back({{"id", point ? point->id() : 0}, {"width", road.pointWidth(i)}});
    }
    return json;
}

void readRoad(GameObject& object, const nlohmann::json& json, const std::string& path,
              Scene& scene, SceneLoadResult& result)
{
    Road* road = object.addComponent<Road>();
    if (!road)
    {
        result.addError(path, "object already has a Road component");
        return;
    }
    road->setSubdivisions(static_cast<u32>(glm::max(1.0f, readNumberOr(json, "subdivisions", 12.0f))));
    road->setTextureRepeat(readNumberOr(json, "textureRepeat", 4.0f));
    road->setSurfaceOffset(readNumberOr(json, "surfaceOffset", 0.06f));
    road->setConformTerrain(readBoolOr(json, "conformTerrain", true));
    const auto terrain = json.find("terrain");
    u64 terrainId = 0;
    if (terrain != json.end() && readNonNegativeInteger(*terrain, terrainId))
        if (GameObject* terrainObject = scene.findGameObject(terrainId))
            road->setTerrain(terrainObject->getComponent<Terrain>());
    const auto points = json.find("points");
    if (points != json.end() && points->is_array())
        for (const auto& entry : *points)
        {
            if (!entry.is_object()) continue;
            const auto id = entry.find("id");
            u64 pointId = 0;
            if (id == entry.end() || !readNonNegativeInteger(*id, pointId)) continue;
            if (GameObject* point = scene.findGameObject(pointId))
                road->addPoint(point, readNumberOr(entry, "width", 6.0f));
        }
    const auto material = json.find("material");
    if (material != json.end() && material->is_object())
        readMaterial(*material, road->material(), path + ".material", result);
    road->rebuild();
    road->setActive(readBoolOr(json, "active", true));
}

nlohmann::json writeTreeParams(const TreeParams& params)
{
    return nlohmann::json{{"clumpMax", params.clumpMax}, {"clumpMin", params.clumpMin},
                          {"lengthFalloffFactor", params.lengthFalloffFactor},
                          {"lengthFalloffPower", params.lengthFalloffPower},
                          {"branchFactor", params.branchFactor},
                          {"radiusFalloffRate", params.radiusFalloffRate},
                          {"taperRate", params.taperRate}, {"maxRadius", params.maxRadius},
                          {"climbRate", params.climbRate}, {"trunkKink", params.trunkKink},
                          {"twistRate", params.twistRate}, {"trunkLength", params.trunkLength},
                          {"initialBranchLength", params.initialBranchLength},
                          {"dropAmount", params.dropAmount}, {"growAmount", params.growAmount},
                          {"sweepAmount", params.sweepAmount}, {"vMultiplier", params.vMultiplier},
                          {"twigScale", params.twigScale}, {"segments", params.segments},
                          {"levels", params.levels}, {"trunkSteps", params.trunkSteps},
                          {"seed", params.seed}};
}

TreeParams readTreeParams(const nlohmann::json& json)
{
    TreeParams params;
    params.clumpMax = readNumberOr(json, "clumpMax", params.clumpMax);
    params.clumpMin = readNumberOr(json, "clumpMin", params.clumpMin);
    params.lengthFalloffFactor = readNumberOr(json, "lengthFalloffFactor", params.lengthFalloffFactor);
    params.lengthFalloffPower = readNumberOr(json, "lengthFalloffPower", params.lengthFalloffPower);
    params.branchFactor = readNumberOr(json, "branchFactor", params.branchFactor);
    params.radiusFalloffRate = readNumberOr(json, "radiusFalloffRate", params.radiusFalloffRate);
    params.taperRate = readNumberOr(json, "taperRate", params.taperRate);
    params.maxRadius = readNumberOr(json, "maxRadius", params.maxRadius);
    params.climbRate = readNumberOr(json, "climbRate", params.climbRate);
    params.trunkKink = readNumberOr(json, "trunkKink", params.trunkKink);
    params.twistRate = readNumberOr(json, "twistRate", params.twistRate);
    params.trunkLength = readNumberOr(json, "trunkLength", params.trunkLength);
    params.initialBranchLength = readNumberOr(json, "initialBranchLength", params.initialBranchLength);
    params.dropAmount = readNumberOr(json, "dropAmount", params.dropAmount);
    params.growAmount = readNumberOr(json, "growAmount", params.growAmount);
    params.sweepAmount = readNumberOr(json, "sweepAmount", params.sweepAmount);
    params.vMultiplier = readNumberOr(json, "vMultiplier", params.vMultiplier);
    params.twigScale = readNumberOr(json, "twigScale", params.twigScale);
    params.segments = static_cast<u32>(glm::max(4.0f, readNumberOr(json, "segments", params.segments)));
    params.levels = static_cast<u32>(glm::max(1.0f, readNumberOr(json, "levels", params.levels)));
    params.trunkSteps = static_cast<u32>(glm::max(1.0f, readNumberOr(json, "trunkSteps", params.trunkSteps)));
    params.seed = static_cast<s32>(readNumberOr(json, "seed", params.seed));
    return params;
}

nlohmann::json writeForest(Forest& forest)
{
    nlohmann::json json{{"type", "Forest"}, {"version", 2}, {"active", forest.active()}};
    json["barkAlbedo"] = forest.barkAlbedoPath();
    json["barkNormal"] = forest.barkNormalPath();
    json["twigTextures"] = forest.twigTexturePaths();
    json["castShadows"] = forest.castsShadows();
    json["wind"] = forest.wind();
    json["barkBumpForce"] = forest.barkBumpForce();
    json["alphaCut"] = forest.alphaCut();
    json["impostors"] = forest.impostorsEnabled();
    json["swapDistance"] = forest.swapDistance();
    json["swapBand"] = forest.swapBand();
    json["impostorWidth"] = forest.impostorWidth();
    json["drawDistance"] = forest.drawDistance();
    json["scaleMinimum"] = forest.scaleMinimum();
    json["scaleMaximum"] = forest.scaleMaximum();
    json["seed"] = forest.seed();
    json["species"] = nlohmann::json::array();
    for (u32 i = 0; i < forest.speciesCount(); ++i)
        json["species"].push_back({{"params", writeTreeParams(forest.speciesParams(i))},
                                    {"height", forest.speciesHeight(i)},
                                    {"weight", forest.speciesWeight(i)},
                                    {"twigTexture", forest.speciesTwigTexture(i)},
                                    {"barkMaterial", writeMaterial(forest.material(i, 0))},
                                    {"twigMaterial", writeMaterial(forest.material(i, 1))}});
    json["instances"] = nlohmann::json::array();
    for (u32 i = 0; i < forest.instanceCount(); ++i)
        json["instances"].push_back({{"position", writeVec3(forest.instancePosition(i))},
                                      {"scale", forest.instanceScale(i)},
                                      {"yaw", glm::degrees(forest.instanceYaw(i))},
                                      {"species", forest.instanceSpecies(i)}});
    return json;
}

void readForest(GameObject& object, const nlohmann::json& json, const std::string& path,
                SceneLoadResult& result)
{
    Forest* forest = object.addComponent<Forest>();
    if (!forest)
    {
        result.addError(path, "object already has a Forest component");
        return;
    }
    const auto barkAlbedo = json.find("barkAlbedo");
    const auto barkNormal = json.find("barkNormal");
    if (barkAlbedo != json.end() && barkAlbedo->is_string())
        forest->setBarkTexture(barkAlbedo->get<std::string>(),
                               barkNormal != json.end() && barkNormal->is_string() ? barkNormal->get<std::string>() : std::string());
    const auto twigs = json.find("twigTextures");
    if (twigs != json.end() && twigs->is_array())
    {
        std::vector<std::string> paths;
        for (const auto& twig : *twigs)
            if (twig.is_string())
                paths.push_back(twig.get<std::string>());
        forest->setTwigTextures(paths);
    }
    const auto species = json.find("species");
    if (species != json.end() && species->is_array())
        for (const auto& entry : *species)
        {
            if (!entry.is_object()) continue;
            const auto params = entry.find("params");
            const s32 index = forest->addSpecies(params != entry.end() && params->is_object() ? readTreeParams(*params) : TreeParams(),
                                                 readNumberOr(entry, "height", 14.0f),
                                                 readNumberOr(entry, "weight", 1.0f),
                                                 static_cast<u32>(readNumberOr(entry, "twigTexture", 0.0f)));
            if (index >= 0)
            {
                const auto bark = entry.find("barkMaterial");
                const auto twig = entry.find("twigMaterial");
                if (bark != entry.end() && bark->is_object()) readMaterial(*bark, forest->material(index, 0), path + ".barkMaterial", result);
                if (twig != entry.end() && twig->is_object()) readMaterial(*twig, forest->material(index, 1), path + ".twigMaterial", result);
            }
        }
    const auto instances = json.find("instances");
    if (instances != json.end() && instances->is_array())
        for (const auto& entry : *instances)
        {
            glm::vec3 position;
            const auto p = entry.find("position");
            if (p == entry.end() || !readVec3(*p, position)) continue;
            forest->plant(position, static_cast<u32>(readNumberOr(entry, "species", 0.0f)),
                          readNumberOr(entry, "scale", 1.0f), readNumberOr(entry, "yaw", 0.0f));
        }
    forest->setCastShadows(readBoolOr(json, "castShadows", true));
    forest->setWind(readNumberOr(json, "wind", forest->wind()));
    forest->setBarkBumpForce(readNumberOr(json, "barkBumpForce", forest->barkBumpForce()));
    forest->setAlphaCut(readNumberOr(json, "alphaCut", forest->alphaCut()));
    forest->setImpostorsEnabled(readBoolOr(json, "impostors", forest->impostorsEnabled()));
    forest->setSwapDistance(readNumberOr(json, "swapDistance", forest->swapDistance()));
    forest->setSwapBand(readNumberOr(json, "swapBand", forest->swapBand()));
    forest->setImpostorWidth(readNumberOr(json, "impostorWidth", forest->impostorWidth()));
    forest->setDrawDistance(readNumberOr(json, "drawDistance", forest->drawDistance()));
    forest->setScaleRange(readNumberOr(json, "scaleMinimum", forest->scaleMinimum()),
                          readNumberOr(json, "scaleMaximum", forest->scaleMaximum()));
    const auto seed = json.find("seed");
    u64 seedValue = 0;
    if (seed != json.end() && readNonNegativeInteger(*seed, seedValue) &&
        seedValue <= std::numeric_limits<u32>::max())
        forest->setSeed(static_cast<u32>(seedValue));
    forest->setActive(readBoolOr(json, "active", true));
}

nlohmann::json writeGrass(Grass& grass)
{
    nlohmann::json json{{"type", "Grass"}, {"version", 3}, {"active", grass.active()},
                        {"atlas", grass.atlasFile()}, {"height", grass.height()},
                        {"width", grass.width()}, {"wind", grass.wind()},
                        {"alphaCut", grass.alphaCut()}, {"cameraBend", grass.cameraBend()},
                        {"drawDistance", grass.drawDistance()}, {"stiffness", grass.stiffness()},
                        {"drag", grass.drag()}, {"softFringe", grass.softFringe()},
                        {"seed", grass.seed()}, {"regions", nlohmann::json::array()},
                        {"clumps", nlohmann::json::array()}};
    for (u32 i = 0; i < grass.regionCount(); ++i)
    {
        GrassAtlasRect region;
        f32 weight = 0.0f;
        if (grass.region(i, region, weight))
            json["regions"].push_back({{"texMulAdd", writeVec4(region.texMulAdd)},
                                       {"sizeAspect", writeVec4(region.sizeAspect)},
                                       {"weight", weight}});
    }
    for (u32 i = 0; i < grass.count(); ++i)
    {
        GrassClump clump;
        if (grass.clump(i, clump))
            json["clumps"].push_back({{"positionScale", writeVec4(clump.positionScale)},
                                      {"normalRotation", writeVec4(clump.normalRotation)},
                                      {"rect", writeVec4(clump.rect)}});
    }
    return json;
}

void readGrass(GameObject& object, const nlohmann::json& json, const std::string& path,
               SceneLoadResult& result)
{
    Grass* grass = object.addComponent<Grass>();
    if (!grass)
    {
        result.addError(path, "object already has a Grass component");
        return;
    }
    const auto atlas = json.find("atlas");
    if (atlas != json.end() && atlas->is_string() && !atlas->get<std::string>().empty())
        grass->loadAtlas(atlas->get<std::string>());
    const auto regions = json.find("regions");
    if (regions != json.end() && regions->is_array())
        for (const auto& value : *regions)
        {
            if (!value.is_object()) continue;
            GrassAtlasRect region;
            const auto texMulAdd = value.find("texMulAdd");
            const auto sizeAspect = value.find("sizeAspect");
            if (texMulAdd == value.end() || sizeAspect == value.end() ||
                !readVec4(*texMulAdd, region.texMulAdd) || !readVec4(*sizeAspect, region.sizeAspect))
                continue;
            grass->addRegion(region, readNumberOr(value, "weight", 1.0f));
        }
    const auto clumps = json.find("clumps");
    if (clumps != json.end() && clumps->is_array())
        for (const auto& value : *clumps)
        {
            if (!value.is_object()) continue;
            GrassClump clump;
            const auto positionScale = value.find("positionScale");
            const auto normalRotation = value.find("normalRotation");
            const auto rect = value.find("rect");
            if (positionScale == value.end() || normalRotation == value.end() || rect == value.end() ||
                !readVec4(*positionScale, clump.positionScale) ||
                !readVec4(*normalRotation, clump.normalRotation) || !readVec4(*rect, clump.rect))
                continue;
            grass->plant(clump);
        }
    grass->setHeight(readNumberOr(json, "height", grass->height()));
    grass->setWidth(readNumberOr(json, "width", grass->width()));
    grass->setWind(readNumberOr(json, "wind", grass->wind()));
    grass->setAlphaCut(readNumberOr(json, "alphaCut", grass->alphaCut()));
    grass->setCameraBend(readNumberOr(json, "cameraBend", grass->cameraBend()));
    grass->setDrawDistance(readNumberOr(json, "drawDistance", grass->drawDistance()));
    grass->setStiffness(readNumberOr(json, "stiffness", grass->stiffness()));
    grass->setDrag(readNumberOr(json, "drag", grass->drag()));
    const auto seed = json.find("seed");
    u64 seedValue = 0;
    if (seed != json.end() && readNonNegativeInteger(*seed, seedValue) && seedValue <= std::numeric_limits<u32>::max())
        grass->setSeed(static_cast<u32>(seedValue));
    const auto soft = json.find("softFringe");
    if (soft != json.end() && soft->is_boolean())
        grass->setSoftFringe(soft->get<bool>());
    const auto active = json.find("active");
    if (active != json.end() && active->is_boolean())
        grass->setActive(active->get<bool>());
}

nlohmann::json writeHair(Hair& hair)
{
    return nlohmann::json{{"type", "Hair"}, {"version", 1}, {"active", hair.active()},
                          {"texture", hair.textureFile()}, {"strandCount", hair.strandCount()},
                          {"submesh", hair.submesh()}, {"seed", hair.seed()},
                          {"minimumGrowthNormalY", hair.minimumGrowthNormalY()},
                          {"segments", hair.segments()}, {"followers", hair.followers()},
                          {"minimumLength", hair.minimumLength()},
                          {"maximumLength", hair.maximumLength()}, {"width", hair.width()},
                          {"stiffness", hair.stiffness()}, {"drag", hair.drag()},
                          {"gravity", hair.gravity()}, {"wind", hair.wind()},
                          {"drawDistance", hair.drawDistance()}, {"alphaCut", hair.alphaCut()},
                          {"roughness", hair.roughness()},
                          {"specularStrength", hair.specularStrength()},
                          {"specularTint", hair.specularTint()},
                          {"transmission", hair.transmission()},
                          {"color", writeVec3(hair.color())},
                          {"softFringe", hair.softFringe()}};
}

void readHair(GameObject& object, const nlohmann::json& json, const std::string& path,
              SceneLoadResult& result)
{
    Hair* hair = object.addComponent<Hair>();
    if (!hair)
    {
        result.addError(path, "object already has a Hair component");
        return;
    }
    const auto readU32 = [&](const char* name, u32 fallback)
    {
        const auto field = json.find(name);
        u64 value = fallback;
        if (field != json.end() && readNonNegativeInteger(*field, value) &&
            value <= std::numeric_limits<u32>::max())
            return static_cast<u32>(value);
        return fallback;
    };
    hair->setStrandCount(readU32("strandCount", hair->strandCount()));
    hair->setSubmesh(readU32("submesh", hair->submesh()));
    hair->setSeed(readU32("seed", hair->seed()));
    hair->setMinimumGrowthNormalY(
        readNumberOr(json, "minimumGrowthNormalY", hair->minimumGrowthNormalY()));
    hair->setSegments(readU32("segments", hair->segments()));
    hair->setFollowers(readU32("followers", hair->followers()));
    hair->setLengthRange(readNumberOr(json, "minimumLength", hair->minimumLength()),
                         readNumberOr(json, "maximumLength", hair->maximumLength()));
    hair->setWidth(readNumberOr(json, "width", hair->width()));
    hair->setStiffness(readNumberOr(json, "stiffness", hair->stiffness()));
    hair->setDrag(readNumberOr(json, "drag", hair->drag()));
    hair->setGravity(readNumberOr(json, "gravity", hair->gravity()));
    hair->setWind(readNumberOr(json, "wind", hair->wind()));
    hair->setDrawDistance(readNumberOr(json, "drawDistance", hair->drawDistance()));
    hair->setAlphaCut(readNumberOr(json, "alphaCut", hair->alphaCut()));
    hair->setRoughness(readNumberOr(json, "roughness", hair->roughness()));
    hair->setSpecularStrength(
        readNumberOr(json, "specularStrength", hair->specularStrength()));
    hair->setSpecularTint(readNumberOr(json, "specularTint", hair->specularTint()));
    hair->setTransmission(readNumberOr(json, "transmission", hair->transmission()));
    glm::vec3 color;
    const auto colorField = json.find("color");
    if (colorField != json.end() && readVec3(*colorField, color))
        hair->setColor(color);
    hair->setSoftFringe(readBoolOr(json, "softFringe", hair->softFringe()));
    const auto texture = json.find("texture");
    if (texture != json.end() && texture->is_string() && !texture->get<std::string>().empty())
        hair->loadTexture(texture->get<std::string>());
    hair->setActive(readBoolOr(json, "active", true));
    // Roots are deterministic derived data, so scene files stay compact.
    // MeshRenderer precedes Hair in writeComponents(), making regeneration
    // immediately possible for files produced by this serializer.
    if (!hair->generate())
        result.addWarning(path, "hair roots could not be regenerated; assign a rebuildable scalp mesh");
}

void readTerrain(GameObject& object, const nlohmann::json& json, const std::string& path,
                 SceneLoadResult& result)
{
    Terrain* terrain = object.addComponent<Terrain>();
    if (!terrain)
    {
        result.addError(path, "object already has a Terrain component");
        return;
    }
    const auto heightmap = json.find("heightmap");
    const f32 uvTiles = readNumberOr(json, "uvTiles", 1.0f);
    if (heightmap != json.end() && heightmap->is_string() && !heightmap->get<std::string>().empty())
    {
        terrain->loadFile(heightmap->get<std::string>().c_str(), readNumberOr(json, "cellSize", 1.0f),
                          readNumberOr(json, "heightScale", 32.0f),
                          static_cast<u32>(glm::max(1.0f, readNumberOr(json, "maxLod", 6.0f))),
                          uvTiles);
    }
    const auto vegetationMask = json.find("vegetationMask");
    if (vegetationMask != json.end() && vegetationMask->is_string() &&
        !vegetationMask->get<std::string>().empty() &&
        !terrain->loadVegetationMask(vegetationMask->get<std::string>().c_str()))
        result.addWarning(path, "terrain vegetation mask could not be loaded");
    terrain->setUvTiles(uvTiles);
    const auto receive = json.find("receiveShadows");
    if (receive != json.end() && receive->is_boolean())
        terrain->setReceiveShadows(receive->get<bool>());
    const auto material = json.find("material");
    if (material != json.end() && material->is_object())
        readMaterial(*material, terrain->material(), path + ".material", result);
    terrain->material().flags |= MaterialLit | MaterialTerrain;
    const auto surfaceSplat = json.find("surfaceSplat");
    if (surfaceSplat != json.end() && surfaceSplat->is_string() &&
        !surfaceSplat->get<std::string>().empty() &&
        !terrain->loadSurfaceSplat(surfaceSplat->get<std::string>().c_str()))
        result.addWarning(path, "terrain surface splat map could not be loaded");
    const auto grassGeneration = json.find("grassGeneration");
    if (grassGeneration != json.end())
        readTerrainVegetation(*grassGeneration, terrain->grassGenerationSettings());
    const auto treeGeneration = json.find("treeGeneration");
    if (treeGeneration != json.end())
        readTerrainVegetation(*treeGeneration, terrain->treeGenerationSettings());
    const auto active = json.find("active");
    if (active != json.end() && active->is_boolean())
        terrain->setActive(active->get<bool>());
}

void readVoxelWorld(GameObject& object, const nlohmann::json& json, const std::string& path,
                    SceneLoadResult& result)
{
    VoxelWorldComponent* voxelWorld = object.addComponent<VoxelWorldComponent>();
    if (!voxelWorld)
    {
        result.addError(path, "object already has a VoxelWorld component");
        return;
    }
    u64 seed = 0;
    const auto seedField = json.find("seed");
    if (seedField != json.end() && readNonNegativeInteger(*seedField, seed) &&
        seed <= std::numeric_limits<u32>::max())
        voxelWorld->setSeed(static_cast<u32>(seed));
    voxelWorld->setChunkRadius(static_cast<s32>(readNumberOr(json, "chunkRadius", 2.0f)));
    voxelWorld->setMinWorldY(static_cast<s32>(readNumberOr(json, "minWorldY", -64.0f)));
    voxelWorld->setMaxWorldY(static_cast<s32>(readNumberOr(json, "maxWorldY", 127.0f)));
    voxelWorld->setWaterLevel(static_cast<s32>(readNumberOr(json, "waterLevel", 20.0f)));
    const auto atlas = json.find("atlasFile");
    if (atlas != json.end() && atlas->is_string())
        voxelWorld->setAtlasFile(atlas->get<std::string>());
    u64 originObjectId = 0;
    const auto origin = json.find("originObjectId");
    if (origin != json.end() && readNonNegativeInteger(*origin, originObjectId))
        voxelWorld->setOriginObjectId(originObjectId);
    const auto active = json.find("active");
    if (active != json.end() && active->is_boolean())
        voxelWorld->setActive(active->get<bool>());
}

nlohmann::json writeCamera(Camera& camera)
{
    nlohmann::json json;
    json["type"] = "Camera";
    json["version"] = 1;
    json["active"] = camera.active();
    json["projection"] = cameraProjectionName(camera.projectionMode());
    // Both written unconditionally rather than only the one the current mode
    // uses - a flat, always-present schema is simpler to read back than a
    // shape that changes with "projection".
    json["fieldOfView"] = camera.fieldOfView();
    json["orthographicSize"] = camera.orthographicSize();
    json["aspect"] = camera.aspect();
    json["near"] = camera.nearPlane();
    json["far"] = camera.farPlane();
    return json;
}

void readCamera(GameObject& object, const nlohmann::json& json, const std::string& path,
                SceneLoadResult& result)
{
    CameraProjection projection = CameraProjection::Perspective;
    const auto projectionField = json.find("projection");
    if (projectionField == json.end() || !projectionField->is_string() ||
        !cameraProjectionFromName(projectionField->get<std::string>(), projection))
    {
        result.addError(path + ".projection", "missing or not one of Perspective/Orthographic");
        return;
    }

    f32 fieldOfView = 0.0f, orthographicSize = 0.0f, aspect = 0.0f, nearPlane = 0.0f,
        farPlane = 0.0f;
    bool ok = true;
    ok &= readFloatField(json, "fieldOfView", fieldOfView, path, result);
    ok &= readFloatField(json, "orthographicSize", orthographicSize, path, result);
    ok &= readFloatField(json, "aspect", aspect, path, result);
    ok &= readFloatField(json, "near", nearPlane, path, result);
    ok &= readFloatField(json, "far", farPlane, path, result);
    if (!ok)
        return;

    Camera* camera = object.addComponent<Camera>();
    if (!camera)
    {
        result.addError(path, "object already has a Camera component");
        return;
    }
    if (projection == CameraProjection::Perspective)
        camera->setPerspective(fieldOfView, aspect, nearPlane, farPlane);
    else
        camera->setOrthographic(orthographicSize, aspect, nearPlane, farPlane);

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        camera->setActive(false);
}

// -------------------------------------------------------------------- Light

const char* lightTypeName(LightType type)
{
    switch (type)
    {
    case LightType::Directional:
        return "Directional";
    case LightType::Point:
        return "Point";
    case LightType::Spot:
        return "Spot";
    case LightType::Rectangle:
        return "Rectangle";
    }
    return "Point";
}

bool lightTypeFromName(const std::string& name, LightType& out)
{
    if (name == "Directional")
    {
        out = LightType::Directional;
        return true;
    }
    if (name == "Point")
    {
        out = LightType::Point;
        return true;
    }
    if (name == "Spot")
    {
        out = LightType::Spot;
        return true;
    }
    if (name == "Rectangle")
    {
        out = LightType::Rectangle;
        return true;
    }
    return false;
}

const char* billboardModeName(BillboardMode mode)
{
    switch (mode)
    {
    case BillboardMode::Free:
        return "Free";
    case BillboardMode::Upright:
        return "Upright";
    case BillboardMode::Fixed:
        return "Fixed";
    }
    return "Free";
}

bool billboardModeFromName(const std::string& name, BillboardMode& out)
{
    if (name == "Free")
    {
        out = BillboardMode::Free;
        return true;
    }
    if (name == "Upright")
    {
        out = BillboardMode::Upright;
        return true;
    }
    if (name == "Fixed")
    {
        out = BillboardMode::Fixed;
        return true;
    }
    return false;
}

const char* billboardBlendName(BatchRenderer::BlendMode mode)
{
    switch (mode)
    {
    case BatchRenderer::BlendMode::Alpha:
        return "Alpha";
    case BatchRenderer::BlendMode::Additive:
        return "Additive";
    case BatchRenderer::BlendMode::Multiplied:
        return "Multiplied";
    case BatchRenderer::BlendMode::AddColors:
        return "AddColors";
    case BatchRenderer::BlendMode::SubtractColors:
        return "SubtractColors";
    }
    return "Alpha";
}

bool billboardBlendFromName(const std::string& name, BatchRenderer::BlendMode& out)
{
    if (name == "Alpha")
    {
        out = BatchRenderer::BlendMode::Alpha;
        return true;
    }
    if (name == "Additive")
    {
        out = BatchRenderer::BlendMode::Additive;
        return true;
    }
    if (name == "Multiplied")
    {
        out = BatchRenderer::BlendMode::Multiplied;
        return true;
    }
    if (name == "AddColors")
    {
        out = BatchRenderer::BlendMode::AddColors;
        return true;
    }
    if (name == "SubtractColors")
    {
        out = BatchRenderer::BlendMode::SubtractColors;
        return true;
    }
    return false;
}

const char* textAlignName(TextAlign align)
{
    switch (align)
    {
    case TextAlign::Left:
        return "Left";
    case TextAlign::Center:
        return "Center";
    case TextAlign::Right:
        return "Right";
    }
    return "Left";
}

bool textAlignFromName(const std::string& name, TextAlign& out)
{
    if (name == "Left")
    {
        out = TextAlign::Left;
        return true;
    }
    if (name == "Center")
    {
        out = TextAlign::Center;
        return true;
    }
    if (name == "Right")
    {
        out = TextAlign::Right;
        return true;
    }
    return false;
}

const char* particleEffectModeName(ParticleEffectMode mode)
{
    switch (mode)
    {
    case ParticleEffectMode::OneShot:
        return "OneShot";
    case ParticleEffectMode::Continuous:
        return "Continuous";
    }
    return "OneShot";
}

bool particleEffectModeFromName(const std::string& name, ParticleEffectMode& out)
{
    if (name == "OneShot")
    {
        out = ParticleEffectMode::OneShot;
        return true;
    }
    if (name == "Continuous")
    {
        out = ParticleEffectMode::Continuous;
        return true;
    }
    return false;
}

nlohmann::json writeLight(Light& light)
{
    nlohmann::json json;
    json["type"] = "Light";
    json["version"] = 1;
    json["active"] = light.active();
    json["lightType"] = lightTypeName(light.lightType());
    json["color"] = {light.color().x, light.color().y, light.color().z};
    json["intensity"] = light.intensity();
    json["castShadows"] = light.castsShadows();
    json["volumetric"] = light.volumetric();

    // Every subtype field is written unconditionally, same convention as
    // Camera's two projections above - unused ones for the current
    // lightType just carry their subtype's default.
    f32 range = 0.0f, innerAngle = 0.0f, outerAngle = 0.0f, width = 0.0f, height = 0.0f;
    switch (light.lightType())
    {
    case LightType::Point:
        range = static_cast<PointLight&>(light).range();
        break;
    case LightType::Spot:
    {
        SpotLight& spot = static_cast<SpotLight&>(light);
        range = spot.range();
        innerAngle = spot.innerAngle();
        outerAngle = spot.outerAngle();
        break;
    }
    case LightType::Rectangle:
    {
        RectangleLight& rectangle = static_cast<RectangleLight&>(light);
        range = rectangle.range();
        width = rectangle.width();
        height = rectangle.height();
        break;
    }
    case LightType::Directional:
        break;
    }
    json["range"] = range;
    json["innerAngle"] = innerAngle;
    json["outerAngle"] = outerAngle;
    json["width"] = width;
    json["height"] = height;
    return json;
}

void readLight(GameObject& object, const nlohmann::json& json, const std::string& path,
               SceneLoadResult& result)
{
    LightType lightType = LightType::Point;
    const auto lightTypeField = json.find("lightType");
    if (lightTypeField == json.end() || !lightTypeField->is_string() ||
        !lightTypeFromName(lightTypeField->get<std::string>(), lightType))
    {
        result.addError(path + ".lightType",
                        "missing or not one of Directional/Point/Spot/Rectangle");
        return;
    }

    glm::vec3 color(1.0f);
    const auto colorField = json.find("color");
    if (colorField == json.end() || !readVec3(*colorField, color))
    {
        result.addError(path + ".color", "expected [r, g, b] of finite numbers");
        return;
    }
    f32 intensity = 0.0f;
    if (!readFloatField(json, "intensity", intensity, path, result))
        return;

    bool castShadows = false;
    const auto castField = json.find("castShadows");
    if (castField != json.end() && castField->is_boolean())
        castShadows = castField->get<bool>();
    bool volumetric = false;
    const auto volumetricField = json.find("volumetric");
    if (volumetricField != json.end() && volumetricField->is_boolean())
        volumetric = volumetricField->get<bool>();

    // Only the fields the resolved subtype actually consumes are required.
    f32 range = 0.0f, innerAngle = 0.0f, outerAngle = 0.0f, width = 0.0f, height = 0.0f;
    const bool needsRange = lightType == LightType::Point || lightType == LightType::Spot ||
                            lightType == LightType::Rectangle;
    if (needsRange && !readFloatField(json, "range", range, path, result))
        return;
    if (lightType == LightType::Spot &&
        (!readFloatField(json, "innerAngle", innerAngle, path, result) ||
         !readFloatField(json, "outerAngle", outerAngle, path, result)))
        return;
    if (lightType == LightType::Rectangle &&
        (!readFloatField(json, "width", width, path, result) ||
         !readFloatField(json, "height", height, path, result)))
        return;

    Light* light = nullptr;
    switch (lightType)
    {
    case LightType::Directional:
        light = object.addComponent<DirectionalLight>();
        break;
    case LightType::Point:
        if (PointLight* point = object.addComponent<PointLight>())
        {
            point->setRange(range);
            light = point;
        }
        break;
    case LightType::Spot:
        if (SpotLight* spot = object.addComponent<SpotLight>())
        {
            spot->setRange(range);
            spot->setAngles(innerAngle, outerAngle);
            light = spot;
        }
        break;
    case LightType::Rectangle:
        if (RectangleLight* rectangle = object.addComponent<RectangleLight>())
        {
            rectangle->setRange(range);
            rectangle->setSize(width, height);
            light = rectangle;
        }
        break;
    }
    if (!light)
    {
        result.addError(path, "object already has a Light component");
        return;
    }
    light->setColor(color);
    light->setIntensity(intensity);
    light->setCastShadows(castShadows);
    light->setVolumetric(volumetric);

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        light->setActive(false);
}

// ------------------------------------------------------------------ Material

// A material's flags are a bitmask, not one enum value - written as a list of
// stable names so a bit's numeric position can move without breaking a saved
// file, the same reasoning MaterialSlotNames.h already applies to slots.
nlohmann::json writeMaterialFlags(u32 flags)
{
    nlohmann::json array = nlohmann::json::array();
    u32 count = 0;
    const MaterialFlagName* entries = MaterialManager::flagNames(count);
    for (u32 i = 0; i < count; ++i)
        if (flags & entries[i].bit)
            array.push_back(entries[i].name);
    return array;
}

u32 readMaterialFlags(const nlohmann::json& array, const std::string& path, SceneLoadResult& result)
{
    u32 flags = 0;
    if (!array.is_array())
    {
        result.addError(path, "expected an array of flag names");
        return flags;
    }
    for (const nlohmann::json& entry : array)
    {
        if (!entry.is_string())
        {
            result.addError(path, "flag entries must be strings");
            continue;
        }
        const std::string name = entry.get<std::string>();
        const u32 bit = MaterialManager::flagBit(name.c_str());
        if (bit != 0)
            flags |= bit;
        else
            result.addWarning(path, "unknown material flag '" + name + "', ignored");
    }
    return flags;
}

const char* blendModeName(BlendMode mode)
{
    switch (mode)
    {
    case BlendMode::Opaque:
        return "Opaque";
    case BlendMode::Alpha:
        return "Alpha";
    case BlendMode::Additive:
        return "Additive";
    case BlendMode::Multiply:
        return "Multiply";
    case BlendMode::PremultipliedAlpha:
        return "PremultipliedAlpha";
    case BlendMode::AddColors:
        return "AddColors";
    case BlendMode::SubtractColors:
        return "SubtractColors";
    }
    return "Opaque";
}

bool blendModeFromName(const std::string& name, BlendMode& out)
{
    static const std::pair<const char*, BlendMode> kModes[] = {
        {"Opaque", BlendMode::Opaque},
        {"Alpha", BlendMode::Alpha},
        {"Additive", BlendMode::Additive},
        {"Multiply", BlendMode::Multiply},
        {"PremultipliedAlpha", BlendMode::PremultipliedAlpha},
        {"AddColors", BlendMode::AddColors},
        {"SubtractColors", BlendMode::SubtractColors},
    };
    for (const auto& entry : kModes)
        if (name == entry.first)
        {
            out = entry.second;
            return true;
        }
    return false;
}

const char* cullModeName(CullMode mode)
{
    switch (mode)
    {
    case CullMode::None:
        return "None";
    case CullMode::Back:
        return "Back";
    case CullMode::Front:
        return "Front";
    }
    return "Back";
}

bool cullModeFromName(const std::string& name, CullMode& out)
{
    if (name == "None")
    {
        out = CullMode::None;
        return true;
    }
    if (name == "Back")
    {
        out = CullMode::Back;
        return true;
    }
    if (name == "Front")
    {
        out = CullMode::Front;
        return true;
    }
    return false;
}

nlohmann::json writeMaterialParams(const MaterialParams& params)
{
    nlohmann::json json;
    json["baseColor"] = writeVec4(params.baseColor);
    json["emissive"] = writeVec4(params.emissive);
    json["surface"] = writeVec4(params.surface);
    json["uvTransform"] = writeVec4(params.uvTransform);
    json["uvAnim"] = writeVec4(params.uvAnim);
    json["sequence"] = writeVec4(params.sequence);
    json["custom0"] = writeVec4(params.custom0);
    json["custom1"] = writeVec4(params.custom1);
    return json;
}

bool readMaterialParams(const nlohmann::json& json, MaterialParams& out, const std::string& path,
                        SceneLoadResult& result)
{
    bool ok = true;
    ok &= readVec4Field(json, "baseColor", out.baseColor, path, result);
    ok &= readVec4Field(json, "emissive", out.emissive, path, result);
    ok &= readVec4Field(json, "surface", out.surface, path, result);
    ok &= readVec4Field(json, "uvTransform", out.uvTransform, path, result);
    ok &= readVec4Field(json, "uvAnim", out.uvAnim, path, result);
    ok &= readVec4Field(json, "sequence", out.sequence, path, result);
    ok &= readVec4Field(json, "custom0", out.custom0, path, result);
    ok &= readVec4Field(json, "custom1", out.custom1, path, result);
    return ok;
}

// Persist every plain-file slot. Sequence and RenderTarget textures are
// skipped with a warning because they need a richer schema than one path;
// the static Detail/ColorMap slots are important for Terrain's fourth layer
// and optional RGBA splat map.
constexpr MaterialSlot kSerializedSlots[] = {
    SlotAlbedo, SlotNormal, SlotSurface, SlotEmissive,
    SlotDetail, SlotColorMap, SlotLightmap, SlotHeight};

nlohmann::json writeMaterial(const Material& material)
{
    nlohmann::json json;
    json["name"] = material.name;
    json["flags"] = writeMaterialFlags(material.flags);
    json["blend"] = blendModeName(material.blend);
    json["cull"] = cullModeName(material.cull);
    json["params"] = writeMaterialParams(material.params);

    nlohmann::json textures = nlohmann::json::array();
    for (MaterialSlot slot : kSerializedSlots)
    {
        const MaterialTexture& texture = material.textures[slot];
        if (texture.source == TextureSource::None)
            continue;
        if (texture.source != TextureSource::Static)
        {
            Log::warning("SceneSerializer: material '%s' slot %s is not a plain file texture, "
                         "not saved",
                         material.name.c_str(), kMaterialSlotNames[slot]);
            continue;
        }
        nlohmann::json entry;
        entry["slot"] = kMaterialSlotNames[slot];
        entry["file"] = texture.file;
        textures.push_back(entry);
    }
    json["textures"] = textures;
    return json;
}

bool readMaterial(const nlohmann::json& json, Material& out, const std::string& path,
                  SceneLoadResult& result)
{
    const auto nameField = json.find("name");
    out.name = (nameField != json.end() && nameField->is_string()) ? nameField->get<std::string>()
                                                                   : std::string();

    // Absent in scenes written before this field existed - the material keeps
    // its default flags rather than failing the whole load.
    const auto flagsField = json.find("flags");
    if (flagsField != json.end())
        out.flags = readMaterialFlags(*flagsField, path + ".flags", result);

    const auto blendField = json.find("blend");
    if (blendField == json.end() || !blendField->is_string() ||
        !blendModeFromName(blendField->get<std::string>(), out.blend))
    {
        result.addError(path + ".blend", "missing or unrecognized blend mode");
        return false;
    }
    const auto cullField = json.find("cull");
    if (cullField == json.end() || !cullField->is_string() ||
        !cullModeFromName(cullField->get<std::string>(), out.cull))
    {
        result.addError(path + ".cull", "missing or unrecognized cull mode");
        return false;
    }
    const auto paramsField = json.find("params");
    if (paramsField == json.end() || !paramsField->is_object() ||
        !readMaterialParams(*paramsField, out.params, path + ".params", result))
    {
        result.addError(path + ".params", "missing or invalid");
        return false;
    }

    const auto texturesField = json.find("textures");
    if (texturesField != json.end() && texturesField->is_array())
    {
        for (usize i = 0; i < texturesField->size(); ++i)
        {
            const nlohmann::json& entry = (*texturesField)[i];
            const std::string entryPath = path + ".textures[" + std::to_string(i) + "]";
            const auto slotField = entry.is_object() ? entry.find("slot") : entry.end();
            const auto fileField = entry.is_object() ? entry.find("file") : entry.end();
            if (!entry.is_object() || slotField == entry.end() || !slotField->is_string() ||
                fileField == entry.end() || !fileField->is_string())
            {
                result.addError(entryPath, "expected {slot, file} strings");
                continue;
            }
            const std::string slotName = slotField->get<std::string>();
            u32 slot = kMaterialSlotCount;
            for (u32 candidate = 0; candidate < kMaterialSlotCount; ++candidate)
                if (slotName == kMaterialSlotNames[candidate])
                    slot = candidate;
            bool supported = false;
            for (MaterialSlot serialized : kSerializedSlots)
                supported |= (slot == static_cast<u32>(serialized));
            if (!supported)
            {
                result.addWarning(entryPath, "slot '" + slotName + "' not supported yet, ignored");
                continue;
            }
            const std::string file = fileField->get<std::string>();
            // Async: a scene the size of Sponza has enough of these that
            // loading them one at a time, synchronously, is what used to
            // freeze the editor window for the whole file. This hands back a
            // usable placeholder immediately - AsyncTextureLoader fills in
            // the real image in place over the next several frames.
            const TextureHandle handle = Assets().loadTextureAsync(
                file, Material::colorSpaceFor(static_cast<MaterialSlot>(slot), out.flags), true);
            if (!handle.valid())
            {
                result.addWarning(entryPath, "could not load texture '" + file + "'");
                continue;
            }
            out.textures[slot].texture = handle;
            out.textures[slot].file = file;
            out.textures[slot].source = TextureSource::Static;
        }
    }
    return true;
}

// -------------------------------------------------------------- MeshRenderer

nlohmann::json writeMeshDesc(const MeshDesc& desc)
{
    nlohmann::json json;
    json["source"] = meshSourceName(desc.source);
    json["file"] = desc.file;
    nlohmann::json params = nlohmann::json::array();
    for (f32 value : desc.params)
        params.push_back(value);
    json["params"] = params;
    return json;
}

bool readMeshDesc(const nlohmann::json& json, MeshDesc& out, const std::string& path,
                  SceneLoadResult& result)
{
    const auto sourceField = json.find("source");
    if (sourceField == json.end() || !sourceField->is_string() ||
        !meshSourceFromName(sourceField->get<std::string>(), out.source))
    {
        result.addError(path + ".source", "missing or unrecognized mesh source");
        return false;
    }
    const auto fileField = json.find("file");
    out.file = (fileField != json.end() && fileField->is_string()) ? fileField->get<std::string>()
                                                                   : std::string();
    const auto paramsField = json.find("params");
    if (paramsField == json.end() || !paramsField->is_array() || paramsField->size() != 8)
    {
        result.addError(path + ".params", "expected an array of 8 numbers");
        return false;
    }
    for (usize i = 0; i < 8; ++i)
    {
        if (!(*paramsField)[i].is_number())
        {
            result.addError(path + ".params", "all 8 entries must be numbers");
            return false;
        }
        out.params[i] = (*paramsField)[i].get<f32>();
    }
    return true;
}

nlohmann::json writeMeshRenderer(MeshRenderer& renderer)
{
    nlohmann::json json;
    json["type"] = "MeshRenderer";
    json["version"] = 1;
    json["active"] = renderer.active();
    // Written unconditionally rather than only when false: a reader that
    // defaults it to true either way costs nothing, and a field that appears
    // only sometimes is the kind a later reader forgets exists.
    json["visibleInReflections"] = renderer.visibleInReflections();

    const MeshDesc& desc = Assets().meshDesc(renderer.mesh());
    if (desc.source == MeshSource::None)
    {
        // An invalid handle is the editor's intentional blank Mesh Instance:
        // it remains empty until the primitive creator or asset picker fills
        // it. A valid handle with no recipe is genuinely anonymous and still
        // cannot survive a save.
        if (renderer.mesh().valid())
            Log::error("SceneSerializer: MeshRenderer on '%s' has an anonymous mesh, not saved",
                       renderer.owner() ? renderer.owner()->name().c_str() : "?");
        json["mesh"] = nullptr;
    }
    else
    {
        json["mesh"] = writeMeshDesc(desc);
    }

    // Material overrides are asset data, not scene data. The mesh loader
    // restores the material from its .mat/.material sidecar; serializing
    // copied materials here makes Save Scene embed stale/incomplete copies
    // and can hide submeshes when the scene is reopened. The exception is a
    // mesh with no file to own a sidecar (editor primitives): the scene is
    // the only place its material can live.
    json["materialOverrides"] = nlohmann::json::array();
    if (desc.source != MeshSource::File && renderer.materialOverrideCount() > 0)
        for (u32 i = 0; i < renderer.materialOverrideCount(); ++i)
            json["materialOverrides"].push_back(writeMaterial(renderer.materialOverrides()[i]));

    if (!renderer.hiddenSubmeshes().empty())
        json["hiddenSubmeshes"] = renderer.hiddenSubmeshes();

    return json;
}

void readMeshRenderer(GameObject& object, const nlohmann::json& json, const std::string& path,
                      SceneLoadResult& result, bool asyncFileLoad)
{
    const auto meshField = json.find("mesh");
    if (meshField == json.end())
    {
        result.addError(path + ".mesh", "missing mesh reference");
        return;
    }

    MeshHandle mesh;
    MeshDesc desc;
    if (!meshField->is_null())
    {
        if (!readMeshDesc(*meshField, desc, path + ".mesh", result))
            return;
        mesh = asyncFileLoad ? Assets().createMeshAsync(desc) : Assets().createMesh(desc);
    }
    // Attached even when the mesh failed to resolve, same "keep the object,
    // warn" policy as a missing texture below - a missing mesh should not
    // take the whole load down.
    //
    // Known gap: MeshRenderer has no field of its own to remember `desc`
    // when `mesh` comes back invalid, so a Save right after a failed
    // resolve writes an anonymous mesh instead of preserving this
    // reference - the plan's "preserve the logical name" policy needs an
    // API addition on MeshRenderer to hold fully, not yet done.
    MeshRenderer* renderer = object.addComponent<MeshRenderer>(mesh);
    if (!renderer)
    {
        result.addError(path, "object already has a MeshRenderer component");
        return;
    }
    if (!meshField->is_null() && !mesh.valid())
        result.addWarning(path + ".mesh",
                          "could not resolve mesh '" + desc.key() + "', renderer left without one");

    // Absent in scenes written before this field existed, and true is what
    // they all meant - every object was in every capture back then.
    const auto reflectionsField = json.find("visibleInReflections");
    if (reflectionsField != json.end() && reflectionsField->is_boolean())
        renderer->setVisibleInReflections(reflectionsField->get<bool>());

    // Legacy materialOverrides on file meshes are intentionally ignored.
    // Materials belong to the mesh sidecar and must be loaded by AssetManager
    // from its .mat or .material file, otherwise an old scene can restore
    // stale partial overrides and make submeshes disappear. File-less meshes
    // (editor primitives) have no sidecar, so the scene copy is authoritative
    // for them.
    const auto overridesField = json.find("materialOverrides");
    if (overridesField != json.end() && overridesField->is_array() &&
        !overridesField->empty() && desc.source != MeshSource::File)
    {
        u32 slot = 0;
        for (const auto& entry : *overridesField)
        {
            // Older scenes wrapped each override as {"material": {...}}.
            const auto wrapped = entry.is_object() ? entry.find("material") : entry.end();
            const nlohmann::json& materialJson =
                wrapped != entry.end() && wrapped->is_object() ? *wrapped : entry;
            Material material;
            if (materialJson.is_object() &&
                readMaterial(materialJson, material, path + ".materialOverrides", result))
                renderer->setMaterialOverride(slot, material);
            ++slot;
        }
    }

    const auto hiddenField = json.find("hiddenSubmeshes");
    if (hiddenField != json.end() && hiddenField->is_array())
    {
        std::vector<u32> hidden;
        for (const auto& entry : *hiddenField)
            if (entry.is_number_unsigned())
                hidden.push_back(entry.get<u32>());
        renderer->setHiddenSubmeshes(std::move(hidden));
    }

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        renderer->setActive(false);
}

// ----------------------------------------------------------- CharacterController

nlohmann::json writeCharacterController(CharacterController& controller)
{
    nlohmann::json json;
    json["type"] = "CharacterController";
    json["version"] = 1;
    json["active"] = controller.active();
    json["radius"] = controller.radius();
    json["height"] = controller.height();
    json["stepOffset"] = controller.stepOffset();
    json["slopeLimit"] = controller.slopeLimit();
    json["skinWidth"] = controller.skinWidth();
    json["gravity"] = controller.gravity();
    json["maxFallSpeed"] = controller.maxFallSpeed();
    json["maxIterations"] = controller.maxIterations();
    // Not written: octree() (RuntimeOnly - built from a level's static
    // geometry, not authored per-object; the caller re-links it after load,
    // same as it already does today) and every simulation field
    // (moveInput/isGrounded/velocity/...).
    return json;
}

void readCharacterController(GameObject& object, const nlohmann::json& json,
                             const std::string& path, SceneLoadResult& result)
{
    f32 radius = 0.0f, height = 0.0f, stepOffset = 0.0f, slopeLimit = 0.0f, skinWidth = 0.0f,
        gravity = 0.0f, maxFallSpeed = 0.0f;
    f32 maxIterationsField = 0.0f;
    bool ok = true;
    ok &= readFloatField(json, "radius", radius, path, result);
    ok &= readFloatField(json, "height", height, path, result);
    ok &= readFloatField(json, "stepOffset", stepOffset, path, result);
    ok &= readFloatField(json, "slopeLimit", slopeLimit, path, result);
    ok &= readFloatField(json, "skinWidth", skinWidth, path, result);
    ok &= readFloatField(json, "gravity", gravity, path, result);
    ok &= readFloatField(json, "maxFallSpeed", maxFallSpeed, path, result);
    ok &= readFloatField(json, "maxIterations", maxIterationsField, path, result);
    if (!ok)
        return;

    CharacterController* controller = object.addComponent<CharacterController>();
    if (!controller)
    {
        result.addError(path, "object already has a CharacterController component");
        return;
    }
    controller->setRadius(radius);
    controller->setHeight(height);
    controller->setStepOffset(stepOffset);
    controller->setSlopeLimit(slopeLimit);
    controller->setSkinWidth(skinWidth);
    controller->setGravity(gravity);
    controller->setMaxFallSpeed(maxFallSpeed);
    controller->setMaxIterations(static_cast<u32>(glm::max(0.0f, maxIterationsField)));

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        controller->setActive(false);
}

// ---------------------------------------------------------- camera controllers

const char* mouseButtonName(MouseButton button)
{
    switch (button)
    {
    case LEFT:
        return "Left";
    case RIGHT:
        return "Right";
    case MIDDLE:
        return "Middle";
    }
    return "Right";
}

bool mouseButtonFromName(const std::string& name, MouseButton& out)
{
    if (name == "Left")
    {
        out = LEFT;
        return true;
    }
    if (name == "Right")
    {
        out = RIGHT;
        return true;
    }
    if (name == "Middle")
    {
        out = MIDDLE;
        return true;
    }
    return false;
}

// KeyCode's values are not Radion's own enum ordinals - each one is pinned by
// the header to an explicit external convention (KEY_A = 65, KEY_ESCAPE =
// 256, matching GLFW's key codes), the same kind of stable external number an
// asset path or a hash already is elsewhere in this file. Writing the raw
// integer here does not carry the "reordering the enum breaks the file" risk
// the plan's "stable strings, never ordinals" rule is about.
template <class ControllerType>
void writeFreeLookKeys(nlohmann::json& json, ControllerType& controller)
{
    nlohmann::json keys;
    for (u8 i = 0; i < static_cast<u8>(ControllerType::Action::Count); ++i)
    {
        const auto action = static_cast<typename ControllerType::Action>(i);
        keys[std::to_string(i)] = static_cast<int>(controller.key(action));
    }
    json["keys"] = keys;
}

template <class ControllerType>
bool readFreeLookKeys(const nlohmann::json& json, ControllerType& controller,
                      const std::string& path, SceneLoadResult& result)
{
    const auto keysField = json.find("keys");
    if (keysField == json.end() || !keysField->is_object())
    {
        result.addError(path + ".keys", "missing keybinding object");
        return false;
    }
    for (u8 i = 0; i < static_cast<u8>(ControllerType::Action::Count); ++i)
    {
        const auto entry = keysField->find(std::to_string(i));
        if (entry == keysField->end() || !entry->is_number_integer())
        {
            // A warning, not addError()+return false: a scene saved before
            // this action existed (Sprint, added after Forward..Down) simply
            // never wrote one, and the constructor's own default for it is a
            // perfectly good fallback - failing the whole controller over
            // one missing binding would be a worse outcome than that.
            result.addWarning(path + ".keys", "missing entry for action " + std::to_string(i) +
                                                  ", keeping its default binding");
            continue;
        }
        controller.setKey(static_cast<typename ControllerType::Action>(i),
                          static_cast<KeyCode>(entry->get<int>()));
    }
    return true;
}

template <class ControllerType>
nlohmann::json writeFreeLookController(const char* typeName, ControllerType& controller)
{
    nlohmann::json json;
    json["type"] = typeName;
    json["version"] = 1;
    json["active"] = controller.active();
    json["moveSpeed"] = controller.moveSpeed();
    json["sprintMultiplier"] = controller.sprintMultiplier();
    json["lookSpeed"] = controller.lookSpeed();
    json["pitchLimit"] = controller.pitchLimit();
    json["lookButton"] = mouseButtonName(controller.lookButton());
    json["requireLookButton"] = controller.requiresLookButton();
    json["invertY"] = controller.invertY();
    writeFreeLookKeys(json, controller);
    return json;
}

template <class ControllerType>
void readFreeLookController(GameObject& object, const nlohmann::json& json, const std::string& path,
                            SceneLoadResult& result)
{
    f32 moveSpeed = 0.0f, lookSpeed = 0.0f, pitchLimit = 0.0f;
    bool ok = true;
    ok &= readFloatField(json, "moveSpeed", moveSpeed, path, result);
    ok &= readFloatField(json, "lookSpeed", lookSpeed, path, result);
    ok &= readFloatField(json, "pitchLimit", pitchLimit, path, result);

    MouseButton lookButton = RIGHT;
    const auto lookButtonField = json.find("lookButton");
    if (lookButtonField == json.end() || !lookButtonField->is_string() ||
        !mouseButtonFromName(lookButtonField->get<std::string>(), lookButton))
    {
        result.addError(path + ".lookButton", "missing or unrecognized mouse button");
        ok = false;
    }
    if (!ok)
        return;

    ControllerType* controller = object.addComponent<ControllerType>();
    if (!controller)
    {
        result.addError(path, "object already has this component");
        return;
    }
    if (!readFreeLookKeys(json, *controller, path, result))
        return;
    controller->setMoveSpeed(moveSpeed);
    // Optional, not required like moveSpeed/lookSpeed/pitchLimit above - a
    // scene saved before Sprint existed has no field for it at all, and
    // should still load instead of failing this whole component.
    controller->setSprintMultiplier(json.value("sprintMultiplier", controller->sprintMultiplier()));
    controller->setLookSpeed(lookSpeed);
    controller->setPitchLimit(pitchLimit);
    controller->setLookButton(lookButton);

    const auto requireField = json.find("requireLookButton");
    if (requireField != json.end() && requireField->is_boolean())
        controller->setRequireLookButton(requireField->get<bool>());
    const auto invertField = json.find("invertY");
    if (invertField != json.end() && invertField->is_boolean())
        controller->setInvertY(invertField->get<bool>());

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        controller->setActive(false);
}

// ---------------------------------------------------------------- Orbit/Maya

nlohmann::json writeOrbitLike(const char* typeName, f32 yaw, f32 pitch, f32 pitchLimit,
                              f32 distance, GameObject* target, const glm::vec3& targetPoint)
{
    nlohmann::json json;
    json["type"] = typeName;
    json["version"] = 1;
    json["yaw"] = yaw;
    json["pitch"] = pitch;
    json["pitchLimit"] = pitchLimit;
    json["distance"] = distance;
    // targetPoint only matters when target is null, but is written
    // unconditionally - same flat-schema convention as Camera/Light above.
    json["targetPoint"] = {targetPoint.x, targetPoint.y, targetPoint.z};
    json["target"] = target ? nlohmann::json(target->id()) : nlohmann::json(nullptr);
    return json;
}

// Resolved immediately, not deferred like BoneAttachment: a target is a
// GameObject, which createObjects() already guarantees exists for every id
// in the file by the time any component is read - no component of the
// target's has to exist yet, unlike BoneAttachment's Animator.
GameObject* readTargetField(const nlohmann::json& json, Scene& out, const std::string& path,
                            SceneLoadResult& result)
{
    const auto targetField = json.find("target");
    if (targetField == json.end() || targetField->is_null())
        return nullptr;
    u64 id = 0;
    if (!readNonNegativeInteger(*targetField, id) || id == 0)
    {
        result.addWarning(path + ".target", "not a valid id, ignored");
        return nullptr;
    }
    GameObject* target = out.findGameObject(id);
    if (!target)
        result.addWarning(path + ".target", "object " + std::to_string(id) + " does not exist");
    return target;
}

nlohmann::json writeOrbit(Orbit& orbit)
{
    nlohmann::json json = writeOrbitLike("Orbit", orbit.yaw(), orbit.pitch(), 0.0f,
                                         orbit.distance(), orbit.target(), orbit.targetPoint());
    // Orbit has no pitchLimit()/orbitSpeed()/zoomSpeed()/orbitButton()/
    // requireOrbitButton() getter - only setters. Written as the class's own
    // defaults until an accessor exists; setYawPitch()'s pitch clamps against
    // whatever limit is live in memory regardless, so this is a real
    // round-trip gap, not a cosmetic one. Flagged rather than guessed at.
    json["active"] = orbit.active();
    return json;
}

void readOrbit(GameObject& object, const nlohmann::json& json, const std::string& path, Scene& out,
               SceneLoadResult& result)
{
    f32 yaw = 0.0f, pitch = 0.0f, distance = 0.0f;
    glm::vec3 targetPoint(0.0f);
    bool ok = true;
    ok &= readFloatField(json, "yaw", yaw, path, result);
    ok &= readFloatField(json, "pitch", pitch, path, result);
    ok &= readFloatField(json, "distance", distance, path, result);
    const auto targetPointField = json.find("targetPoint");
    if (targetPointField == json.end() || !readVec3(*targetPointField, targetPoint))
    {
        result.addError(path + ".targetPoint", "expected [x, y, z] of finite numbers");
        ok = false;
    }
    if (!ok)
        return;

    GameObject* target = readTargetField(json, out, path, result);

    Orbit* orbit = object.addComponent<Orbit>();
    if (!orbit)
    {
        result.addError(path, "object already has an Orbit component");
        return;
    }
    orbit->setTargetPoint(targetPoint);
    if (target)
        orbit->setTarget(target);
    orbit->setDistance(distance);
    orbit->setYawPitch(yaw, pitch);

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        orbit->setActive(false);
}

nlohmann::json writeThirdPerson(ThirdPerson& camera)
{
    nlohmann::json json;
    json["type"] = "ThirdPerson";
    json["version"] = 1;
    json["active"] = camera.active();
    json["yaw"] = camera.yaw();
    json["pitch"] = camera.pitch();
    json["minPitch"] = camera.minPitch();
    json["maxPitch"] = camera.maxPitch();
    json["distance"] = camera.distance();
    json["heightOffset"] = camera.heightOffset();
    json["shoulderOffset"] = camera.shoulderOffset();
    json["smoothTime"] = camera.smoothTime();
    json["lookSpeed"] = camera.lookSpeed();
    json["invertY"] = camera.invertY();
    GameObject* target = camera.target();
    json["target"] = target ? nlohmann::json(target->id()) : nlohmann::json(nullptr);
    return json;
}

void readThirdPerson(GameObject& object, const nlohmann::json& json, const std::string& path,
                     Scene& out, SceneLoadResult& result)
{
    f32 yaw = 0.0f, pitch = 0.0f, minPitch = 0.0f, maxPitch = 0.0f, distance = 0.0f;
    f32 heightOffset = 0.0f, shoulderOffset = 0.0f, smoothTime = 0.0f, lookSpeed = 0.0f;
    bool ok = true;
    ok &= readFloatField(json, "yaw", yaw, path, result);
    ok &= readFloatField(json, "pitch", pitch, path, result);
    ok &= readFloatField(json, "minPitch", minPitch, path, result);
    ok &= readFloatField(json, "maxPitch", maxPitch, path, result);
    ok &= readFloatField(json, "distance", distance, path, result);
    ok &= readFloatField(json, "heightOffset", heightOffset, path, result);
    ok &= readFloatField(json, "shoulderOffset", shoulderOffset, path, result);
    ok &= readFloatField(json, "smoothTime", smoothTime, path, result);
    ok &= readFloatField(json, "lookSpeed", lookSpeed, path, result);
    if (!ok)
        return;

    GameObject* target = readTargetField(json, out, path, result);

    ThirdPerson* camera = object.addComponent<ThirdPerson>();
    if (!camera)
    {
        result.addError(path, "object already has a ThirdPerson component");
        return;
    }
    // Limits before the angles: setYawPitch() clamps the pitch against
    // whatever limits are live, so reading them in the other order would
    // clamp the saved pitch against the class defaults.
    camera->setPitchLimits(minPitch, maxPitch);
    camera->setYawPitch(yaw, pitch);
    camera->setDistance(distance);
    camera->setHeightOffset(heightOffset);
    camera->setShoulderOffset(shoulderOffset);
    camera->setSmoothTime(smoothTime);
    camera->setLookSpeed(lookSpeed);
    if (target)
        camera->setTarget(target);

    const auto invertField = json.find("invertY");
    if (invertField != json.end() && invertField->is_boolean())
        camera->setInvertY(invertField->get<bool>());
    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        camera->setActive(false);
}

nlohmann::json writeMaya(Maya& maya)
{
    nlohmann::json json;
    json["type"] = "Maya";
    json["version"] = 1;
    json["active"] = maya.active();
    json["distance"] = maya.distance();
    const glm::vec3& targetPoint = maya.targetPoint();
    json["targetPoint"] = {targetPoint.x, targetPoint.y, targetPoint.z};
    json["target"] = maya.target() ? nlohmann::json(maya.target()->id()) : nlohmann::json(nullptr);
    json["yaw"] = maya.yaw();
    json["pitch"] = maya.pitch();
    return json;
}

void readMaya(GameObject& object, const nlohmann::json& json, const std::string& path, Scene& out,
              SceneLoadResult& result)
{
    f32 distance = 0.0f;
    glm::vec3 targetPoint(0.0f);
    bool ok = true;
    ok &= readFloatField(json, "distance", distance, path, result);
    const auto targetPointField = json.find("targetPoint");
    if (targetPointField == json.end() || !readVec3(*targetPointField, targetPoint))
    {
        result.addError(path + ".targetPoint", "expected [x, y, z] of finite numbers");
        ok = false;
    }
    if (!ok)
        return;

    GameObject* target = readTargetField(json, out, path, result);

    Maya* maya = object.addComponent<Maya>();
    if (!maya)
    {
        result.addError(path, "object already has a Maya component");
        return;
    }
    maya->setTargetPoint(targetPoint);
    if (target)
        maya->setTarget(target);
    maya->setDistance(distance);
    // Optional, not required like distance/targetPoint above - a scene saved
    // before yaw()/pitch() existed on Maya has neither field, and should
    // still load with the construction default (yaw 0, pitch 17) rather than
    // failing the whole component.
    maya->setYawPitch(json.value("yaw", maya->yaw()), json.value("pitch", maya->pitch()));

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        maya->setActive(false);
}

// -------------------------------------------------------------- RibbonTrail

nlohmann::json writeRibbonTrail(RibbonTrail& trail)
{
    nlohmann::json json;
    json["type"] = "RibbonTrail";
    json["version"] = 1;
    json["active"] = trail.active();
    json["emitting"] = trail.emitting();
    // The blade's base under "target", matching readTargetField on the way in.
    json["target"] = trail.base() ? nlohmann::json(trail.base()->id()) : nlohmann::json(nullptr);
    json["tip"] = trail.tip() ? nlohmann::json(trail.tip()->id()) : nlohmann::json(nullptr);
    json["lifetime"] = trail.lifetime();
    json["minDistance"] = trail.minDistance();
    json["smoothness"] = trail.smoothness();
    json["startColor"] = trail.startColor().value();
    json["endColor"] = trail.endColor().value();
    json["additive"] = trail.additive();
    json["depthTest"] = trail.depthTest();
    // The texture is a live handle with no source path on the component, so
    // it still cannot be saved - reattach it from code after load.
    return json;
}

void readRibbonTrail(GameObject& object, const nlohmann::json& json, const std::string& path,
                     Scene& out, SceneLoadResult& result)
{
    RibbonTrail* trail = object.addComponent<RibbonTrail>();
    if (!trail)
    {
        result.addError(path, "object already has a RibbonTrail component");
        return;
    }

    GameObject* base = readTargetField(json, out, path, result);
    const auto tipField = json.find("tip");
    GameObject* tip = nullptr;
    if (tipField != json.end() && !tipField->is_null())
    {
        u64 id = 0;
        if (readNonNegativeInteger(*tipField, id) && id != 0)
            tip = out.findGameObject(id);
    }
    if (base && tip)
        trail->setBlade(base, tip);

    // All optional with the component's construction defaults - scenes saved
    // before these fields were written still load.
    trail->setLifetime(readNumberOr(json, "lifetime", trail->lifetime()));
    trail->setMinDistance(readNumberOr(json, "minDistance", trail->minDistance()));
    const auto smoothnessField = json.find("smoothness");
    if (smoothnessField != json.end() && smoothnessField->is_number_unsigned())
        trail->setSmoothness(smoothnessField->get<u32>());
    const auto startColorField = json.find("startColor");
    const auto endColorField = json.find("endColor");
    if (startColorField != json.end() && startColorField->is_number_unsigned())
        trail->setColor(Color(startColorField->get<u32>()),
                        endColorField != json.end() && endColorField->is_number_unsigned()
                            ? Color(endColorField->get<u32>())
                            : trail->endColor());
    const auto additiveField = json.find("additive");
    if (additiveField != json.end() && additiveField->is_boolean())
        trail->setAdditive(additiveField->get<bool>());
    const auto depthTestField = json.find("depthTest");
    if (depthTestField != json.end() && depthTestField->is_boolean())
        trail->setDepthTest(depthTestField->get<bool>());

    const auto emittingField = json.find("emitting");
    if (emittingField != json.end() && emittingField->is_boolean())
        trail->setEmitting(emittingField->get<bool>());

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        trail->setActive(false);
}

// -------------------------------------------------------------- Billboard

nlohmann::json writeBillboard(Billboard& billboard)
{
    nlohmann::json json;
    json["type"] = "Billboard";
    json["version"] = 1;
    json["active"] = billboard.active();
    json["size"] = {billboard.size().x, billboard.size().y};
    json["color"] = billboard.color().value();
    json["mode"] = billboardModeName(billboard.mode());
    json["texture"] = billboard.textureFile();
    // "additive" stays alongside "blend" - an older editor build reading this
    // same file back still gets a sensible Additive/Alpha choice out of the
    // bool, even though it cannot see Multiplied/AddColors/SubtractColors.
    json["additive"] = billboard.additive();
    json["blend"] = billboardBlendName(billboard.blendMode());
    json["depthTest"] = billboard.depthTest();
    if (billboard.animated())
    {
        json["animated"] = true;
        json["atlasCols"] = billboard.atlasCols();
        json["atlasRows"] = billboard.atlasRows();
        json["atlasFps"] = billboard.atlasFps();
    }
    else
    {
        json["animated"] = false;
        json["uvRect"] = writeVec4(billboard.uvRect());
    }
    return json;
}

void readBillboard(GameObject& object, const nlohmann::json& json, const std::string& path,
                   SceneLoadResult& result)
{
    Billboard* billboard = object.addComponent<Billboard>();
    if (!billboard)
    {
        result.addError(path, "object already has a Billboard component");
        return;
    }

    const auto sizeField = json.find("size");
    if (sizeField != json.end() && sizeField->is_array() && sizeField->size() == 2 &&
        (*sizeField)[0].is_number() && (*sizeField)[1].is_number())
        billboard->setSize((*sizeField)[0].get<f32>(), (*sizeField)[1].get<f32>());

    const auto colorField = json.find("color");
    if (colorField != json.end() && colorField->is_number_unsigned())
        billboard->setColor(Color(colorField->get<u32>()));

    const auto modeField = json.find("mode");
    BillboardMode mode = BillboardMode::Free;
    if (modeField != json.end() && modeField->is_string() &&
        billboardModeFromName(modeField->get<std::string>(), mode))
        billboard->setMode(mode);

    const auto textureField = json.find("texture");
    if (textureField != json.end() && textureField->is_string())
        billboard->setTextureFile(textureField->get<std::string>());

    // "blend" first - a scene saved before it existed only has "additive",
    // which still resolves to a valid (if narrower) choice on its own.
    const auto blendField = json.find("blend");
    BatchRenderer::BlendMode blend = BatchRenderer::BlendMode::Additive;
    if (blendField != json.end() && blendField->is_string() &&
        billboardBlendFromName(blendField->get<std::string>(), blend))
        billboard->setBlendMode(blend);
    else
    {
        const auto additiveField = json.find("additive");
        if (additiveField != json.end() && additiveField->is_boolean())
            billboard->setAdditive(additiveField->get<bool>());
    }

    const auto depthTestField = json.find("depthTest");
    if (depthTestField != json.end() && depthTestField->is_boolean())
        billboard->setDepthTest(depthTestField->get<bool>());

    const auto animatedField = json.find("animated");
    if (animatedField != json.end() && animatedField->is_boolean() && animatedField->get<bool>())
    {
        const auto colsField = json.find("atlasCols");
        const auto rowsField = json.find("atlasRows");
        const auto fpsField = json.find("atlasFps");
        if (colsField != json.end() && colsField->is_number_unsigned() && rowsField != json.end() &&
            rowsField->is_number_unsigned() && fpsField != json.end() && fpsField->is_number())
            billboard->setAnimatedAtlas(colsField->get<u32>(), rowsField->get<u32>(),
                                        fpsField->get<f32>());
    }
    else
    {
        glm::vec4 uvRect(0.0f, 0.0f, 1.0f, 1.0f);
        const auto uvField = json.find("uvRect");
        if (uvField != json.end() && readVec4(*uvField, uvRect))
            billboard->setUVRect(uvRect.x, uvRect.y, uvRect.z, uvRect.w);
    }

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        billboard->setActive(false);
}

// ---------------------------------------------------------------- Text3D

nlohmann::json writeText3D(Text3D& text)
{
    nlohmann::json json;
    json["type"] = "Text3D";
    json["version"] = 1;
    json["active"] = text.active();
    json["text"] = text.text();
    json["characterSize"] = text.characterSize();
    json["spacing"] = text.spacing();
    json["color"] = text.color().value();
    json["mode"] = billboardModeName(text.mode());
    json["align"] = textAlignName(text.alignment());
    json["additive"] = text.additive();
    json["depthTest"] = text.depthTest();
    return json;
}

void readText3D(GameObject& object, const nlohmann::json& json, const std::string& path,
                SceneLoadResult& result)
{
    Text3D* text = object.addComponent<Text3D>();
    if (!text)
    {
        result.addError(path, "object already has a Text3D component");
        return;
    }

    const auto textField = json.find("text");
    if (textField != json.end() && textField->is_string())
        text->setText(textField->get<std::string>());

    f32 characterSize = 0.0f;
    if (readFloatField(json, "characterSize", characterSize, path, result))
        text->setCharacterSize(characterSize);

    const auto spacingField = json.find("spacing");
    if (spacingField != json.end() && spacingField->is_number())
        text->setSpacing(spacingField->get<f32>());

    const auto colorField = json.find("color");
    if (colorField != json.end() && colorField->is_number_unsigned())
        text->setColor(Color(colorField->get<u32>()));

    const auto modeField = json.find("mode");
    BillboardMode mode = BillboardMode::Free;
    if (modeField != json.end() && modeField->is_string() &&
        billboardModeFromName(modeField->get<std::string>(), mode))
        text->setMode(mode);

    const auto alignField = json.find("align");
    TextAlign align = TextAlign::Left;
    if (alignField != json.end() && alignField->is_string() &&
        textAlignFromName(alignField->get<std::string>(), align))
        text->setAlignment(align);

    const auto additiveField = json.find("additive");
    if (additiveField != json.end() && additiveField->is_boolean())
        text->setAdditive(additiveField->get<bool>());

    const auto depthTestField = json.find("depthTest");
    if (depthTestField != json.end() && depthTestField->is_boolean())
        text->setDepthTest(depthTestField->get<bool>());

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        text->setActive(false);
}

// ------------------------------------------------------ Waypoints

nlohmann::json writeWaypoints(Waypoints& waypoints)
{
    nlohmann::json json;
    json["type"] = "Waypoints";
    json["version"] = 1;
    json["active"] = waypoints.active();

    nlohmann::json points = nlohmann::json::array();
    for (u32 i = 0; i < static_cast<u32>(waypoints.pointCount()); ++i)
    {
        const WaypointNode& node = waypoints.point(i);
        nlohmann::json entry;
        entry["position"] = {node.position.x, node.position.y, node.position.z};
        entry["radius"] = node.radius;
        // Written as stored: once per pair, on the lower index.
        entry["links"] = node.links;
        points.push_back(entry);
    }
    json["points"] = points;
    return json;
}

void readWaypoints(GameObject& object, const nlohmann::json& json, const std::string& path,
                   SceneLoadResult& result)
{
    Waypoints* waypoints = object.addComponent<Waypoints>();
    if (!waypoints)
    {
        result.addError(path, "object already has a Waypoints component");
        return;
    }

    const auto points = json.find("points");
    if (points == json.end() || !points->is_array())
        return;

    // Two passes: every node has to exist before any link can name one.
    for (const nlohmann::json& entry : *points)
    {
        glm::vec3 position(0.0f);
        const auto positionField = entry.find("position");
        if (positionField != entry.end() && positionField->is_array() && positionField->size() == 3)
            position = glm::vec3((*positionField)[0].get<f32>(), (*positionField)[1].get<f32>(),
                                 (*positionField)[2].get<f32>());
        f32 radius = 1.5f;
        const auto radiusField = entry.find("radius");
        if (radiusField != entry.end() && radiusField->is_number())
            radius = radiusField->get<f32>();
        waypoints->addPoint(position, radius);
    }

    u32 index = 0;
    for (const nlohmann::json& entry : *points)
    {
        const auto links = entry.find("links");
        if (links != entry.end() && links->is_array())
            for (const nlohmann::json& link : *links)
                if (link.is_number_unsigned())
                    waypoints->link(index, link.get<u32>());
        ++index;
    }

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        waypoints->setActive(false);
}

// ------------------------------------------------------ NavMeshSurface

nlohmann::json writeNavMeshSurface(NavMeshSurface& surface)
{
    const AI::NavMeshConfig& config = surface.config();
    nlohmann::json json;
    json["type"] = "NavMeshSurface";
    json["version"] = 1;
    json["active"] = surface.active();
    // Only the recipe is stored, never the baked Detour data: it would be
    // megabytes in the scene file and stale the moment the level mesh
    // changed. `baked` records that a surface HAD been built, so loading
    // rebuilds it instead of coming back up saying it never was.
    json["baked"] = surface.built();
    // A baked surface saved to its own file is loaded straight back instead
    // of rebuilt - the recipe below is then only what a future re-bake would
    // use, not what this scene pays for on every open.
    json["navData"] = surface.navDataFile();
    const glm::vec3& seed = surface.groundSeed();
    json["groundSeed"] = {seed.x, seed.y, seed.z};
    json["cellSize"] = config.cellSize;
    json["cellHeight"] = config.cellHeight;
    json["agentHeight"] = config.agentHeight;
    json["agentRadius"] = config.agentRadius;
    json["agentMaxClimb"] = config.agentMaxClimb;
    json["agentMaxSlope"] = config.agentMaxSlope;
    json["regionMinSize"] = config.regionMinSize;
    json["regionMergeSize"] = config.regionMergeSize;
    json["edgeMaxLen"] = config.edgeMaxLen;
    json["edgeMaxError"] = config.edgeMaxError;
    json["vertsPerPoly"] = config.vertsPerPoly;
    json["detailSampleDist"] = config.detailSampleDist;
    json["detailSampleMaxError"] = config.detailSampleMaxError;
    return json;
}

void readNavMeshSurface(GameObject& object, const nlohmann::json& json, const std::string& path,
                        SceneLoadResult& result)
{
    NavMeshSurface* surface = object.addComponent<NavMeshSurface>();
    if (!surface)
    {
        result.addError(path, "object already has a NavMeshSurface component");
        return;
    }

    AI::NavMeshConfig& config = surface->config();
    readFloatField(json, "cellSize", config.cellSize, path, result);
    readFloatField(json, "cellHeight", config.cellHeight, path, result);
    readFloatField(json, "agentHeight", config.agentHeight, path, result);
    readFloatField(json, "agentRadius", config.agentRadius, path, result);
    readFloatField(json, "agentMaxClimb", config.agentMaxClimb, path, result);
    readFloatField(json, "agentMaxSlope", config.agentMaxSlope, path, result);
    readFloatField(json, "edgeMaxLen", config.edgeMaxLen, path, result);
    readFloatField(json, "edgeMaxError", config.edgeMaxError, path, result);
    readFloatField(json, "detailSampleDist", config.detailSampleDist, path, result);
    readFloatField(json, "detailSampleMaxError", config.detailSampleMaxError, path, result);

    const auto minSize = json.find("regionMinSize");
    if (minSize != json.end() && minSize->is_number())
        config.regionMinSize = minSize->get<s32>();
    const auto mergeSize = json.find("regionMergeSize");
    if (mergeSize != json.end() && mergeSize->is_number())
        config.regionMergeSize = mergeSize->get<s32>();
    const auto verts = json.find("vertsPerPoly");
    if (verts != json.end() && verts->is_number())
        config.vertsPerPoly = verts->get<s32>();

    const auto seedField = json.find("groundSeed");
    if (seedField != json.end() && seedField->is_array() && seedField->size() == 3)
        surface->setGroundSeed(glm::vec3((*seedField)[0].get<f32>(), (*seedField)[1].get<f32>(),
                                         (*seedField)[2].get<f32>()));

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        surface->setActive(false);

    // A surface that was baked when the scene was saved is restored here.
    // Reading the baked file back is preferred over rebuilding: same result,
    // none of the Recast pipeline's cost. Rebuilding is the fallback for a
    // scene saved before any file existed, or one whose file has since gone.
    const auto navDataField = json.find("navData");
    const std::string navDataFile =
        navDataField != json.end() && navDataField->is_string() ? navDataField->get<std::string>()
                                                                : std::string();
    const auto bakedField = json.find("baked");
    const bool wasBaked =
        bakedField != json.end() && bakedField->is_boolean() && bakedField->get<bool>();
    if (!navDataFile.empty() && surface->loadNavData(navDataFile))
        return;
    if (wasBaked)
        surface->build();
}

// ------------------------------------------------------ SelfDestroy

nlohmann::json writeSelfDestroy(SelfDestroy& selfDestroy)
{
    nlohmann::json json;
    json["type"] = "SelfDestroy";
    json["version"] = 1;
    json["active"] = selfDestroy.active();
    json["lifetime"] = selfDestroy.lifetime();
    return json;
}

void readSelfDestroy(GameObject& object, const nlohmann::json& json, const std::string& path,
                     SceneLoadResult& result)
{
    SelfDestroy* selfDestroy = object.addComponent<SelfDestroy>();
    if (!selfDestroy)
    {
        result.addError(path, "object already has a SelfDestroy component");
        return;
    }

    f32 lifetime = 1.0f;
    if (readFloatField(json, "lifetime", lifetime, path, result))
        selfDestroy->setLifetime(lifetime);

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        selfDestroy->setActive(false);
}

// --------------------------------------------------------- Collider

const char* colliderShapeName(ColliderShape shape)
{
    switch (shape)
    {
    case ColliderShape::Sphere:
        return "Sphere";
    case ColliderShape::Box:
        return "Box";
    case ColliderShape::Capsule:
        return "Capsule";
    case ColliderShape::Mesh:
        return "Mesh";
    }
    return "Sphere";
}

bool colliderShapeFromName(const std::string& name, ColliderShape& out)
{
    if (name == "Sphere")
        out = ColliderShape::Sphere;
    else if (name == "Box")
        out = ColliderShape::Box;
    else if (name == "Capsule")
        out = ColliderShape::Capsule;
    else if (name == "Mesh")
        out = ColliderShape::Mesh;
    else
        return false;
    return true;
}

const char* collisionResponseName(CollisionResponse response)
{
    switch (response)
    {
    case CollisionResponse::None:
        return "None";
    case CollisionResponse::Stop:
        return "Stop";
    case CollisionResponse::Slide:
        return "Slide";
    case CollisionResponse::SlideXZ:
        return "SlideXZ";
    }
    return "None";
}

bool collisionResponseFromName(const std::string& name, CollisionResponse& out)
{
    if (name == "None")
        out = CollisionResponse::None;
    else if (name == "Stop")
        out = CollisionResponse::Stop;
    else if (name == "Slide")
        out = CollisionResponse::Slide;
    else if (name == "SlideXZ")
        out = CollisionResponse::SlideXZ;
    else
        return false;
    return true;
}

nlohmann::json writeCollider(Collider& collider)
{
    nlohmann::json json;
    json["type"] = "Collider";
    json["version"] = 1;
    json["active"] = collider.active();
    json["shape"] = colliderShapeName(collider.shape());
    json["radius"] = collider.radius();
    json["halfExtents"] = writeVec3(collider.halfExtents());
    json["height"] = collider.height();
    json["collisionType"] = collider.type();
    json["response"] = collisionResponseName(collider.response());
    return json;
}

// The shape kind round-trips; the octree itself does not (not an asset path).
void readCollider(GameObject& object, const nlohmann::json& json, const std::string& path,
                  SceneLoadResult& result)
{
    ColliderShape shape = ColliderShape::Sphere;
    const auto shapeField = json.find("shape");
    if (shapeField == json.end() || !shapeField->is_string() ||
        !colliderShapeFromName(shapeField->get<std::string>(), shape))
    {
        result.addError(path + ".shape", "missing or not one of Sphere/Box/Capsule/Mesh");
        return;
    }

    f32 radius = 0.5f, height = 1.0f;
    glm::vec3 halfExtents(0.5f);
    bool ok = true;
    ok &= readFloatField(json, "radius", radius, path, result);
    ok &= readVec3Field(json, "halfExtents", halfExtents, path, result);
    ok &= readFloatField(json, "height", height, path, result);
    if (!ok)
        return;

    CollisionResponse response = CollisionResponse::None;
    const auto responseField = json.find("response");
    if (responseField == json.end() || !responseField->is_string() ||
        !collisionResponseFromName(responseField->get<std::string>(), response))
    {
        result.addError(path + ".response", "missing or not one of None/Stop/Slide/SlideXZ");
        return;
    }

    Collider* collider = object.addComponent<Collider>();
    if (!collider)
    {
        result.addError(path, "object already has a Collider component");
        return;
    }

    switch (shape)
    {
    case ColliderShape::Sphere:
        collider->setSphere(radius);
        break;
    case ColliderShape::Box:
        collider->setBox(halfExtents);
        break;
    case ColliderShape::Capsule:
        collider->setCapsule(radius, height);
        break;
    case ColliderShape::Mesh:
        collider->setMesh(nullptr);
        break;
    }

    collider->setType(static_cast<u32>(readNumberOr(json, "collisionType", 0.0f) + 0.5f));
    collider->setResponse(response);

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        collider->setActive(false);
}

// ----------------------------------------------------- ZenBehaviour

nlohmann::json writeZenBehaviour(ZenBehaviour& behaviour)
{
    nlohmann::json json;
    json["type"] = "ZenBehaviour";
    json["version"] = 1;
    json["active"] = behaviour.active();
    json["script"] = behaviour.scriptPath();

    // Only the overrides. The defaults belong to the script's __init__, and
    // writing them here would freeze a copy that goes stale the moment the
    // script is edited - the scene would keep resurrecting the old value.
    nlohmann::json properties = nlohmann::json::array();
    for (usize i = 0; i < behaviour.overrideCount(); ++i)
    {
        const ScriptProperty* property = behaviour.overrideAt(i);
        nlohmann::json entry;
        entry["name"] = property->name;
        switch (property->kind)
        {
        case ScriptProperty::Kind::Number:
            if (property->integer)
                entry["int"] = (s64)property->number;
            else
                entry["number"] = property->number;
            break;
        case ScriptProperty::Kind::String:
            entry["string"] = property->text;
            break;
        case ScriptProperty::Kind::Bool:
            entry["bool"] = property->flag;
            break;
        }
        properties.push_back(entry);
    }
    if (!properties.empty())
        json["properties"] = properties;
    return json;
}

// The value field's own name carries the type, so a number that was written
// as an int comes back as an int - a script testing "self.lives == 3" would
// break against a 3.0 restored from the file.
void readZenBehaviourProperty(ZenBehaviour& behaviour, const nlohmann::json& json,
                              const std::string& path, SceneLoadResult& result)
{
    if (!json.is_object())
    {
        result.addWarning(path, "expected an object, ignored");
        return;
    }
    const auto nameField = json.find("name");
    if (nameField == json.end() || !nameField->is_string())
    {
        result.addWarning(path + ".name", "missing or not a string, ignored");
        return;
    }
    const std::string name = nameField->get<std::string>();

    const auto intField = json.find("int");
    if (intField != json.end() && intField->is_number())
    {
        behaviour.setNumberOverride(name, (f64)intField->get<s64>(), true);
        return;
    }
    const auto numberField = json.find("number");
    if (numberField != json.end() && numberField->is_number())
    {
        behaviour.setNumberOverride(name, numberField->get<f64>(), false);
        return;
    }
    const auto stringField = json.find("string");
    if (stringField != json.end() && stringField->is_string())
    {
        behaviour.setStringOverride(name, stringField->get<std::string>());
        return;
    }
    const auto boolField = json.find("bool");
    if (boolField != json.end() && boolField->is_boolean())
    {
        behaviour.setBoolOverride(name, boolField->get<bool>());
        return;
    }
    result.addWarning(path, "no int/number/string/bool value, ignored");
}

void readZenBehaviour(GameObject& object, const nlohmann::json& json, const std::string& path,
                      SceneLoadResult& result)
{
    ZenBehaviour* behaviour = object.addComponent<ZenBehaviour>();
    if (!behaviour)
    {
        result.addError(path, "object already has a component in the Script slot");
        return;
    }

    const auto scriptField = json.find("script");
    const std::string script = scriptField != json.end() && scriptField->is_string()
                                   ? scriptField->get<std::string>()
                                   : std::string();
    // A script file that is gone or no longer compiles is a warning, not a
    // load error: the component and its path come back either way, so the
    // file can be fixed and reloaded from the inspector instead of the whole
    // scene refusing to open.
    if (!script.empty() && !behaviour->loadFile(script))
        result.addWarning(path + ".script", behaviour->lastError());

    // After the load, so the script's own declarations are known. The values
    // only reach the VM when the instance is built, on the first update.
    const auto propertiesField = json.find("properties");
    if (propertiesField != json.end() && propertiesField->is_array())
    {
        for (usize i = 0; i < propertiesField->size(); ++i)
            readZenBehaviourProperty(*behaviour, (*propertiesField)[i],
                                     path + ".properties[" + std::to_string(i) + "]", result);
    }

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        behaviour->setActive(false);
}

// -------------------------------------------------------- ParticleEffect

nlohmann::json writeParticleEffect(ParticleEffect& effect)
{
    const ParticleSystem::Emitter& emitter = effect.emitter();
    nlohmann::json json;
    json["type"] = "ParticleEffect";
    json["version"] = 1;
    json["active"] = effect.active();
    json["mode"] = particleEffectModeName(effect.mode());
    json["burstCount"] = effect.burstCount();
    json["autoDestroy"] = effect.autoDestroy();
    json["useOwnerDirection"] = effect.useOwnerDirection();
    json["playing"] = effect.isPlaying();
    // emitter.position is set from the owner every frame while playing
    // (see ParticleEffect::onUpdate()) - not worth round-tripping.
    json["rate"] = emitter.rate;
    json["direction"] = {emitter.direction.x, emitter.direction.y, emitter.direction.z};
    json["spread"] = emitter.spread;
    json["speedMin"] = emitter.speedMin;
    json["speedMax"] = emitter.speedMax;
    json["lifeMin"] = emitter.lifeMin;
    json["lifeMax"] = emitter.lifeMax;
    json["sizeBegin"] = emitter.sizeBegin;
    json["sizeEnd"] = emitter.sizeEnd;
    json["colorBegin"] = writeVec4(emitter.colorBegin);
    json["colorEnd"] = writeVec4(emitter.colorEnd);
    json["mass"] = emitter.mass;
    json["rotationVelocity"] = emitter.rotationVelocity;
    json["startRadius"] = emitter.startRadius;
    return json;
}

void readParticleEffect(GameObject& object, const nlohmann::json& json, const std::string& path,
                        SceneLoadResult& result)
{
    ParticleEffect* effect = object.addComponent<ParticleEffect>();
    if (!effect)
    {
        result.addError(path, "object already has a ParticleEffect component");
        return;
    }

    ParticleSystem::Emitter emitter;
    bool ok = true;
    ok &= readFloatField(json, "rate", emitter.rate, path, result);
    glm::vec3 direction = emitter.direction;
    const auto directionField = json.find("direction");
    ok &= directionField != json.end() && readVec3(*directionField, direction);
    emitter.direction = direction;
    ok &= readFloatField(json, "spread", emitter.spread, path, result);
    ok &= readFloatField(json, "speedMin", emitter.speedMin, path, result);
    ok &= readFloatField(json, "speedMax", emitter.speedMax, path, result);
    ok &= readFloatField(json, "lifeMin", emitter.lifeMin, path, result);
    ok &= readFloatField(json, "lifeMax", emitter.lifeMax, path, result);
    ok &= readFloatField(json, "sizeBegin", emitter.sizeBegin, path, result);
    ok &= readFloatField(json, "sizeEnd", emitter.sizeEnd, path, result);
    const auto colorBeginField = json.find("colorBegin");
    ok &= colorBeginField != json.end() && readVec4(*colorBeginField, emitter.colorBegin);
    const auto colorEndField = json.find("colorEnd");
    ok &= colorEndField != json.end() && readVec4(*colorEndField, emitter.colorEnd);
    ok &= readFloatField(json, "mass", emitter.mass, path, result);
    ok &= readFloatField(json, "rotationVelocity", emitter.rotationVelocity, path, result);
    ok &= readFloatField(json, "startRadius", emitter.startRadius, path, result);
    if (ok)
        effect->setEmitter(emitter);

    const auto modeField = json.find("mode");
    ParticleEffectMode mode = ParticleEffectMode::OneShot;
    if (modeField != json.end() && modeField->is_string() &&
        particleEffectModeFromName(modeField->get<std::string>(), mode))
        effect->setMode(mode);

    const auto burstCountField = json.find("burstCount");
    if (burstCountField != json.end() && burstCountField->is_number_unsigned())
        effect->setBurstCount(burstCountField->get<u32>());

    const auto autoDestroyField = json.find("autoDestroy");
    if (autoDestroyField != json.end() && autoDestroyField->is_boolean())
        effect->setAutoDestroy(autoDestroyField->get<bool>());

    const auto useOwnerField = json.find("useOwnerDirection");
    if (useOwnerField != json.end() && useOwnerField->is_boolean())
        effect->setUseOwnerDirection(useOwnerField->get<bool>());

    const auto playingField = json.find("playing");
    if (playingField != json.end() && playingField->is_boolean() && playingField->get<bool>())
        effect->play();

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        effect->setActive(false);
}

// ------------------------------------------------------- ParticleEmitter

const char* particleEmitterShapeName(ParticleEmitterShape shape)
{
    switch (shape)
    {
    case ParticleEmitterShape::Point:
        return "Point";
    case ParticleEmitterShape::Sphere:
        return "Sphere";
    case ParticleEmitterShape::Box:
        return "Box";
    case ParticleEmitterShape::Cone:
        return "Cone";
    case ParticleEmitterShape::Circle:
        return "Circle";
    case ParticleEmitterShape::Ring:
        return "Ring";
    }
    return "Point";
}

bool particleEmitterShapeFromName(const std::string& name, ParticleEmitterShape& out)
{
    if (name == "Point")
    {
        out = ParticleEmitterShape::Point;
        return true;
    }
    if (name == "Sphere")
    {
        out = ParticleEmitterShape::Sphere;
        return true;
    }
    if (name == "Box")
    {
        out = ParticleEmitterShape::Box;
        return true;
    }
    if (name == "Cone")
    {
        out = ParticleEmitterShape::Cone;
        return true;
    }
    if (name == "Circle")
    {
        out = ParticleEmitterShape::Circle;
        return true;
    }
    if (name == "Ring")
    {
        out = ParticleEmitterShape::Ring;
        return true;
    }
    return false;
}

const char* particleEmissionModeName(ParticleEmissionMode mode)
{
    switch (mode)
    {
    case ParticleEmissionMode::Continuous:
        return "Continuous";
    case ParticleEmissionMode::Burst:
        return "Burst";
    case ParticleEmissionMode::OneShot:
        return "OneShot";
    case ParticleEmissionMode::Pulse:
        return "Pulse";
    }
    return "Continuous";
}

bool particleEmissionModeFromName(const std::string& name, ParticleEmissionMode& out)
{
    if (name == "Continuous")
    {
        out = ParticleEmissionMode::Continuous;
        return true;
    }
    if (name == "Burst")
    {
        out = ParticleEmissionMode::Burst;
        return true;
    }
    if (name == "OneShot")
    {
        out = ParticleEmissionMode::OneShot;
        return true;
    }
    if (name == "Pulse")
    {
        out = ParticleEmissionMode::Pulse;
        return true;
    }
    return false;
}

nlohmann::json writeParticleEmitter(ParticleEmitter& emitter)
{
    nlohmann::json json;
    json["type"] = "ParticleEmitter";
    json["version"] = 1;
    json["active"] = emitter.active();
    json["maxParticles"] = emitter.maxParticles();
    json["emissionMode"] = particleEmissionModeName(emitter.emissionMode());
    json["emissionRate"] = emitter.emissionRate();
    json["burstCount"] = emitter.burstCount();
    json["burstInterval"] = emitter.burstInterval();
    json["oneShotCount"] = emitter.oneShotCount();
    json["pulseRate"] = emitter.pulseRate();
    json["particlesPerPulse"] = emitter.particlesPerPulse();
    json["shape"] = particleEmitterShapeName(emitter.shape());
    json["shapeRadius"] = emitter.shapeRadius();
    json["shapeInnerRadius"] = emitter.shapeInnerRadius();
    json["shapeConeAngle"] = emitter.shapeConeAngle();
    json["shapeBoxSize"] = {emitter.shapeBoxSize().x, emitter.shapeBoxSize().y,
                            emitter.shapeBoxSize().z};
    json["emissionOffset"] = {emitter.emissionOffset().x, emitter.emissionOffset().y,
                              emitter.emissionOffset().z};
    json["emissionDirection"] = {emitter.emissionDirection().x, emitter.emissionDirection().y,
                                 emitter.emissionDirection().z};
    json["spreadAngle"] = emitter.spreadAngle();
    json["lifetimeMin"] = emitter.lifetimeMin();
    json["lifetimeMax"] = emitter.lifetimeMax();
    json["speedMin"] = emitter.speedMin();
    json["speedMax"] = emitter.speedMax();
    json["sizeStart"] = {emitter.sizeStart().x, emitter.sizeStart().y};
    json["sizeEnd"] = {emitter.sizeEnd().x, emitter.sizeEnd().y};
    json["colorStart"] = emitter.colorStart().value();
    json["colorEnd"] = emitter.colorEnd().value();
    json["rotationSpeedMin"] = emitter.rotationSpeedMin();
    json["rotationSpeedMax"] = emitter.rotationSpeedMax();
    json["gravity"] = {emitter.gravity().x, emitter.gravity().y, emitter.gravity().z};
    json["drag"] = emitter.drag();
    json["duration"] = emitter.duration();
    json["loop"] = emitter.loop();
    json["usesAtlas"] = emitter.usesAtlas();
    json["atlasCols"] = emitter.atlasCols();
    json["atlasRows"] = emitter.atlasRows();
    json["additive"] = emitter.additive();
    json["depthTest"] = emitter.depthTest();
    json["billboardMode"] = billboardModeName(emitter.billboardMode());
    json["texture"] = emitter.textureFile();
    json["affectors"] = nlohmann::json::array();
    for (ParticleAffector* affector : emitter.affectors())
    {
        nlohmann::json value{{"enabled", affector->enabled}};
        if (const auto* gravity = dynamic_cast<const GravityAffector*>(affector))
            value["type"] = "Gravity", value["gravity"] = writeVec3(gravity->gravity);
        else if (const auto* drag = dynamic_cast<const DragAffector*>(affector))
            value["type"] = "Drag", value["drag"] = drag->drag;
        else if (const auto* vortex = dynamic_cast<const VortexAffector*>(affector))
            value["type"] = "Vortex", value["center"] = writeVec3(vortex->center), value["strength"] = vortex->strength, value["radius"] = vortex->radius;
        else if (const auto* attractor = dynamic_cast<const AttractorAffector*>(affector))
            value["type"] = "Attractor", value["position"] = writeVec3(attractor->position), value["strength"] = attractor->strength, value["radius"] = attractor->radius, value["repulse"] = attractor->repulse;
        else if (const auto* turbulence = dynamic_cast<const TurbulenceAffector*>(affector))
            value["type"] = "Turbulence", value["strength"] = turbulence->strength, value["frequency"] = turbulence->frequency;
        else if (const auto* color = dynamic_cast<const ColorOverLifetimeAffector*>(affector))
            value["type"] = "ColorOverLifetime", value["start"] = color->startColor.value(), value["end"] = color->endColor.value();
        else if (const auto* size = dynamic_cast<const SizeOverLifetimeAffector*>(affector))
            value["type"] = "SizeOverLifetime", value["start"] = {size->startSize.x, size->startSize.y}, value["end"] = {size->endSize.x, size->endSize.y};
        else
            continue;
        json["affectors"].push_back(value);
    }
    json["playing"] = emitter.isPlaying();
    return json;
}

void readParticleEmitter(GameObject& object, const nlohmann::json& json, const std::string& path,
                         SceneLoadResult& result)
{
    ParticleEmitter* emitter = object.addComponent<ParticleEmitter>();
    if (!emitter)
    {
        result.addError(path, "object already has a ParticleEmitter component");
        return;
    }

    const auto texture = json.find("texture");
    if (texture != json.end() && texture->is_string())
        emitter->setTextureFile(texture->get<std::string>());
    const auto affectors = json.find("affectors");
    if (affectors != json.end() && affectors->is_array())
        for (const auto& value : *affectors)
        {
            const auto typeField = value.find("type");
            if (!value.is_object() || typeField == value.end() || !typeField->is_string()) continue;
            const std::string type = typeField->get<std::string>();
            const auto readAffectorVec3 = [&](const char* key, glm::vec3& out)
            {
                const auto field = value.find(key);
                return field != value.end() && readVec3(*field, out);
            };
            const auto readAffectorBool = [&](const char* key, bool fallback)
            {
                const auto field = value.find(key);
                return field != value.end() && field->is_boolean() ? field->get<bool>() : fallback;
            };
            const auto readAffectorColor = [&](const char* key, Color& out)
            {
                const auto field = value.find(key);
                u64 packed = 0;
                if (field == value.end() || !readNonNegativeInteger(*field, packed) ||
                    packed > std::numeric_limits<u32>::max())
                    return false;
                out = Color(static_cast<u32>(packed));
                return true;
            };
            ParticleAffector* affector = nullptr;
            if (type == "Gravity") { glm::vec3 v; if (readAffectorVec3("gravity", v)) affector = new GravityAffector(v); }
            else if (type == "Drag") affector = new DragAffector(readNumberOr(value, "drag", 0.0f));
            else if (type == "Vortex") { glm::vec3 v; if (readAffectorVec3("center", v)) affector = new VortexAffector(v, readNumberOr(value, "strength", 0.0f), readNumberOr(value, "radius", 1.0f)); }
            else if (type == "Attractor") { glm::vec3 v; if (readAffectorVec3("position", v)) affector = new AttractorAffector(v, readNumberOr(value, "strength", 0.0f), readNumberOr(value, "radius", 1.0f), readAffectorBool("repulse", false)); }
            else if (type == "Turbulence") affector = new TurbulenceAffector(readNumberOr(value, "strength", 0.0f), readNumberOr(value, "frequency", 1.0f));
            else if (type == "ColorOverLifetime")
            {
                Color start;
                Color end;
                if (readAffectorColor("start", start) && readAffectorColor("end", end))
                    affector = new ColorOverLifetimeAffector(start, end);
            }
            else if (type == "SizeOverLifetime")
            {
                glm::vec2 a(0.0f), b(0.0f);
                const auto start = value.find("start");
                const auto end = value.find("end");
                if (start == value.end() || !start->is_array() || start->size() != 2 ||
                    !(*start)[0].is_number() || !(*start)[1].is_number() ||
                    end == value.end() || !end->is_array() || end->size() != 2 ||
                    !(*end)[0].is_number() || !(*end)[1].is_number())
                    continue;
                a = {(*start)[0].get<f32>(), (*start)[1].get<f32>()};
                b = {(*end)[0].get<f32>(), (*end)[1].get<f32>()};
                affector = new SizeOverLifetimeAffector(a, b);
            }
            if (affector) { affector->enabled = readAffectorBool("enabled", true); emitter->addAffector(affector); }
        }

    const auto maxParticlesField = json.find("maxParticles");
    if (maxParticlesField != json.end() && maxParticlesField->is_number_unsigned())
        emitter->setMaxParticles(maxParticlesField->get<u32>());

    const auto shapeField = json.find("shape");
    ParticleEmitterShape shape = ParticleEmitterShape::Point;
    if (shapeField != json.end() && shapeField->is_string() &&
        particleEmitterShapeFromName(shapeField->get<std::string>(), shape))
    {
        const f32 radius = readNumberOr(json, "shapeRadius", 1.0f);
        const f32 innerRadius = readNumberOr(json, "shapeInnerRadius", 0.5f);
        const f32 coneAngle = readNumberOr(json, "shapeConeAngle", 45.0f);
        glm::vec3 boxSize(1.0f);
        const auto boxSizeField = json.find("shapeBoxSize");
        if (boxSizeField != json.end())
            readVec3(*boxSizeField, boxSize);

        switch (shape)
        {
        case ParticleEmitterShape::Point:
            emitter->setShapePoint();
            break;
        case ParticleEmitterShape::Sphere:
            emitter->setShapeSphere(radius);
            break;
        case ParticleEmitterShape::Box:
            emitter->setShapeBox(boxSize);
            break;
        case ParticleEmitterShape::Cone:
            emitter->setShapeCone(coneAngle, radius);
            break;
        case ParticleEmitterShape::Circle:
            emitter->setShapeCircle(radius);
            break;
        case ParticleEmitterShape::Ring:
            emitter->setShapeRing(radius, innerRadius);
            break;
        }
    }

    const auto emissionOffsetField = json.find("emissionOffset");
    glm::vec3 emissionOffset(0.0f);
    if (emissionOffsetField != json.end() && readVec3(*emissionOffsetField, emissionOffset))
        emitter->setEmissionOffset(emissionOffset);

    const auto emissionDirectionField = json.find("emissionDirection");
    glm::vec3 emissionDirection(0.0f, 1.0f, 0.0f);
    if (emissionDirectionField != json.end() &&
        readVec3(*emissionDirectionField, emissionDirection))
        emitter->setEmissionDirection(emissionDirection);

    const auto spreadAngleField = json.find("spreadAngle");
    if (spreadAngleField != json.end() && spreadAngleField->is_number())
        emitter->setSpreadAngle(spreadAngleField->get<f32>());

    f32 lifetimeMin = 0.0f, lifetimeMax = 0.0f;
    const bool hasLifetimeMin = readFloatField(json, "lifetimeMin", lifetimeMin, path, result);
    const bool hasLifetimeMax = readFloatField(json, "lifetimeMax", lifetimeMax, path, result);
    if (hasLifetimeMin || hasLifetimeMax)
        emitter->setLifetime(hasLifetimeMin ? lifetimeMin : emitter->lifetimeMin(),
                             hasLifetimeMax ? lifetimeMax : emitter->lifetimeMax());

    f32 speedMin = 0.0f, speedMax = 0.0f;
    const bool hasSpeedMin = readFloatField(json, "speedMin", speedMin, path, result);
    const bool hasSpeedMax = readFloatField(json, "speedMax", speedMax, path, result);
    if (hasSpeedMin || hasSpeedMax)
        emitter->setSpeed(hasSpeedMin ? speedMin : emitter->speedMin(),
                          hasSpeedMax ? speedMax : emitter->speedMax());

    glm::vec3 sizeStart3(0.0f), sizeEnd3(0.0f);
    const auto sizeStartField = json.find("sizeStart");
    const bool hasSizeStart = sizeStartField != json.end() && sizeStartField->is_array() &&
                              sizeStartField->size() == 2 && (*sizeStartField)[0].is_number() &&
                              (*sizeStartField)[1].is_number();
    const auto sizeEndField = json.find("sizeEnd");
    const bool hasSizeEnd = sizeEndField != json.end() && sizeEndField->is_array() &&
                            sizeEndField->size() == 2 && (*sizeEndField)[0].is_number() &&
                            (*sizeEndField)[1].is_number();
    if (hasSizeStart || hasSizeEnd)
    {
        const glm::vec2 sizeStart = hasSizeStart ? glm::vec2((*sizeStartField)[0].get<f32>(),
                                                             (*sizeStartField)[1].get<f32>())
                                                 : emitter->sizeStart();
        const glm::vec2 sizeEnd =
            hasSizeEnd ? glm::vec2((*sizeEndField)[0].get<f32>(), (*sizeEndField)[1].get<f32>())
                       : emitter->sizeEnd();
        emitter->setSize(sizeStart, sizeEnd);
    }

    const auto colorStartField = json.find("colorStart");
    const auto colorEndField = json.find("colorEnd");
    if (colorStartField != json.end() && colorStartField->is_number_unsigned() &&
        colorEndField != json.end() && colorEndField->is_number_unsigned())
        emitter->setColor(Color(colorStartField->get<u32>()), Color(colorEndField->get<u32>()));

    f32 rotationMin = 0.0f, rotationMax = 0.0f;
    const bool hasRotationMin = readFloatField(json, "rotationSpeedMin", rotationMin, path, result);
    const bool hasRotationMax = readFloatField(json, "rotationSpeedMax", rotationMax, path, result);
    if (hasRotationMin || hasRotationMax)
        emitter->setRotationSpeed(hasRotationMin ? rotationMin : emitter->rotationSpeedMin(),
                                  hasRotationMax ? rotationMax : emitter->rotationSpeedMax());

    const auto gravityField = json.find("gravity");
    glm::vec3 gravity(0.0f);
    if (gravityField != json.end() && readVec3(*gravityField, gravity))
        emitter->setGravity(gravity);

    f32 drag = 0.0f;
    if (readFloatField(json, "drag", drag, path, result))
        emitter->setDrag(drag);

    f32 duration = -1.0f;
    if (readFloatField(json, "duration", duration, path, result))
        emitter->setDuration(duration);

    const auto loopField = json.find("loop");
    if (loopField != json.end() && loopField->is_boolean())
        emitter->setLoop(loopField->get<bool>());

    const auto usesAtlasField = json.find("usesAtlas");
    const auto atlasColsField = json.find("atlasCols");
    const auto atlasRowsField = json.find("atlasRows");
    if (usesAtlasField != json.end() && usesAtlasField->is_boolean() &&
        usesAtlasField->get<bool>() && atlasColsField != json.end() &&
        atlasColsField->is_number_unsigned() && atlasRowsField != json.end() &&
        atlasRowsField->is_number_unsigned())
        emitter->setAtlasGrid(atlasColsField->get<u32>(), atlasRowsField->get<u32>());

    const auto additiveField = json.find("additive");
    if (additiveField != json.end() && additiveField->is_boolean())
        emitter->setAdditive(additiveField->get<bool>());

    const auto depthTestField = json.find("depthTest");
    if (depthTestField != json.end() && depthTestField->is_boolean())
        emitter->setDepthTest(depthTestField->get<bool>());

    const auto billboardModeField = json.find("billboardMode");
    BillboardMode billboardMode = BillboardMode::Free;
    if (billboardModeField != json.end() && billboardModeField->is_string() &&
        billboardModeFromName(billboardModeField->get<std::string>(), billboardMode))
        emitter->setBillboardMode(billboardMode);

    // Emission mode is set last: setContinuous()/setBurst()/setOneShot()/
    // setPulse() each also assign the mode's own rate/count fields, which
    // the calls above already read from more specific keys - this only
    // fixes up which mode those values apply under.
    const auto emissionModeField = json.find("emissionMode");
    ParticleEmissionMode emissionMode = ParticleEmissionMode::Continuous;
    if (emissionModeField != json.end() && emissionModeField->is_string() &&
        particleEmissionModeFromName(emissionModeField->get<std::string>(), emissionMode))
    {
        switch (emissionMode)
        {
        case ParticleEmissionMode::Continuous:
            emitter->setContinuous(readNumberOr(json, "emissionRate", emitter->emissionRate()));
            break;
        case ParticleEmissionMode::Burst:
            emitter->setBurst((u32)readNumberOr(json, "burstCount", (f32)emitter->burstCount()),
                              readNumberOr(json, "burstInterval", emitter->burstInterval()));
            break;
        case ParticleEmissionMode::OneShot:
            emitter->setOneShot(
                (u32)readNumberOr(json, "oneShotCount", (f32)emitter->oneShotCount()));
            break;
        case ParticleEmissionMode::Pulse:
            emitter->setPulse(
                readNumberOr(json, "pulseRate", emitter->pulseRate()),
                (u32)readNumberOr(json, "particlesPerPulse", (f32)emitter->particlesPerPulse()));
            break;
        }
    }

    const auto playingField = json.find("playing");
    if (playingField != json.end() && playingField->is_boolean() && playingField->get<bool>())
        emitter->play();

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        emitter->setActive(false);
}

// -------------------------------------------------------------- Animator

nlohmann::json writeAnimator(Animator& animator)
{
    nlohmann::json json;
    json["type"] = "Animator";
    json["version"] = 1;
    json["active"] = animator.active();

    const AnimationSetHandle handle = animator.animationSet();
    const std::string& skeletonFile = Animations().skeletonSourceFile(handle);
    if (skeletonFile.empty() && animator.bound())
        Log::warning("SceneSerializer: Animator on '%s' is bound to an animation set not loaded "
                     "through AnimationManager::loadFromFiles(), not saved",
                     animator.owner() ? animator.owner()->name().c_str() : "?");
    json["skeleton"] = skeletonFile;
    nlohmann::json clips = nlohmann::json::array();
    for (const std::string& file : Animations().animationSourceFiles(handle))
        clips.push_back(file);
    json["clips"] = clips;

    std::string clip;
    f32 time = 0.0f;
    if (animator.layerCount() > 0)
    {
        AnimationLayer& layer = animator.layer(0);
        clip = layer.current();
        time = layer.time();
    }
    json["clip"] = clip;
    json["time"] = time;
    // Known gap: AnimationLayer has no mode()/speed() getter (play()/
    // setSpeed() are setter-only), so PlayMode and playback speed cannot be
    // written back - a reload always restores Loop at speed 1. Only layer 0
    // is covered; multi-layer blending and IK chains are not v1 scope.
    return json;
}

void readAnimator(GameObject& object, const nlohmann::json& json, const std::string& path,
                  SceneLoadResult& result)
{
    const auto skeletonField = json.find("skeleton");
    const std::string skeletonFile = (skeletonField != json.end() && skeletonField->is_string())
                                         ? skeletonField->get<std::string>()
                                         : std::string();
    std::vector<std::string> clipFiles;
    const auto clipsField = json.find("clips");
    if (clipsField != json.end() && clipsField->is_array())
        for (const nlohmann::json& entry : *clipsField)
            if (entry.is_string())
                clipFiles.push_back(entry.get<std::string>());
    const auto clipField = json.find("clip");
    const std::string clip = (clipField != json.end() && clipField->is_string())
                                 ? clipField->get<std::string>()
                                 : std::string();
    f32 time = 0.0f;
    const auto timeField = json.find("time");
    if (timeField != json.end() && timeField->is_number())
    {
        const f32 value = timeField->get<f32>();
        if (std::isfinite(value))
            time = value;
    }

    Animator* animator = object.addComponent<Animator>();
    if (!animator)
    {
        result.addError(path, "object already has an Animator component");
        return;
    }

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        animator->setActive(false);

    if (skeletonFile.empty())
        return; // valid: nothing bound yet

    // AnimationManager::loadFromFiles() caches by (skeleton, clips) itself,
    // so several Animators naming the same character within one load still
    // only decode the files once - no cache needed here on top of it.
    const AnimationSetHandle handle = Animations().loadFromFiles(skeletonFile, clipFiles);
    if (!handle.valid())
    {
        result.addWarning(path + ".skeleton",
                          "could not load skeleton/animations from '" + skeletonFile + "'");
        return;
    }
    animator->bind(handle);
    if (!clip.empty())
    {
        animator->play(clip, PlayMode::Loop, 0.0f);
        if (animator->layerCount() > 0)
            animator->layer(0).seek(time);
    }
}

// -------------------------------------------------------------- BoneAttachment

// A bone is named, not indexed, in the file: an index is only stable within
// one build of one skeleton, and BoneAttachment::bind() itself is content to
// take either - see Skeleton::bone(index).name for how the name is read back
// out for writing.
struct PendingBoneAttachment
{
    u64 ownerId = 0;
    u64 targetId = 0;
    std::string boneName;
    std::string path;
};

nlohmann::json writeBoneAttachment(BoneAttachment& attachment)
{
    nlohmann::json json;
    json["type"] = "BoneAttachment";
    json["version"] = 1;
    json["active"] = attachment.active();

    Animator* animator = attachment.animator();
    const Skeleton* skeleton = animator ? animator->skeleton() : nullptr;
    const s32 boneIndex = attachment.boneIndex();
    if (animator && animator->owner() && skeleton && boneIndex >= 0 &&
        static_cast<u32>(boneIndex) < skeleton->boneCount())
    {
        json["target"] = animator->owner()->id();
        json["bone"] = skeleton->bone(static_cast<u32>(boneIndex)).name;
    }
    else
    {
        json["target"] = nullptr;
        json["bone"] = "";
    }
    return json;
}

// Only creates the component and records what it should end up bound to -
// the bind() call itself waits for resolveBoneAttachments(), run once every
// object's every component has been read, because the target's Animator may
// not exist yet at this point (it can appear later in the array).
void readBoneAttachment(GameObject& object, const nlohmann::json& json, const std::string& path,
                        std::vector<PendingBoneAttachment>& pending, SceneLoadResult& result)
{
    BoneAttachment* attachment = object.addComponent<BoneAttachment>();
    if (!attachment)
    {
        result.addError(path, "object already has a BoneAttachment component");
        return;
    }

    const auto activeField = json.find("active");
    if (activeField != json.end() && activeField->is_boolean() && !activeField->get<bool>())
        attachment->setActive(false);

    const auto targetField = json.find("target");
    if (targetField == json.end() || targetField->is_null())
        return; // valid: unbound

    u64 targetId = 0;
    if (!readNonNegativeInteger(*targetField, targetId) || targetId == 0)
    {
        result.addWarning(path + ".target", "not a valid id, ignored");
        return;
    }
    const auto boneField = json.find("bone");
    if (boneField == json.end() || !boneField->is_string() || boneField->get<std::string>().empty())
    {
        result.addWarning(path + ".bone", "missing bone name, attachment left unbound");
        return;
    }
    pending.push_back({object.id(), targetId, boneField->get<std::string>(), path});
}

void resolveBoneAttachments(Scene& out, const std::vector<PendingBoneAttachment>& pending,
                            SceneLoadResult& result)
{
    for (const PendingBoneAttachment& entry : pending)
    {
        GameObject* owner = out.findGameObject(entry.ownerId);
        BoneAttachment* attachment = owner ? owner->getComponent<BoneAttachment>() : nullptr;
        if (!attachment)
            continue; // created moments ago by readBoneAttachment(); should always exist

        GameObject* target = out.findGameObject(entry.targetId);
        Animator* animator = target ? target->getComponent<Animator>() : nullptr;
        if (!animator)
        {
            result.addWarning(entry.path, "target object " + std::to_string(entry.targetId) +
                                              " has no Animator component");
            continue;
        }
        if (!attachment->bind(animator, entry.boneName))
            result.addWarning(entry.path,
                              "bone '" + entry.boneName + "' not found on target's skeleton");
    }
}

// -------------------------------------------------------------- dispatch

// Bundles what a handful of component readers need beyond their own
// GameObject& and json - resolving a cross-object reference (Orbit/Maya's
// target, RibbonTrail's base/tip) needs `out` to look objects up in;
// BoneAttachment defers its bind() until every object's components exist
// (see resolveBoneAttachments()). A struct instead of more parameters
// growing on every read function down the chain as more components need
// cross-cutting state.
struct ComponentReadContext
{
    Scene& out;
    std::vector<PendingBoneAttachment>& pendingBoneAttachments;
};

nlohmann::json writeComponents(GameObject& object)
{
    nlohmann::json array = nlohmann::json::array();
    // A switch over ComponentType would have to skip every type Grupo C
    // still owns no writer for; checking each covered type directly is
    // simpler while the list is this short. Revisit as a real dispatch table
    // once more component types are covered (see PLANO_SERIALIZACAO_CENA.md).
    if (Camera* camera = object.getComponent<Camera>())
        array.push_back(writeCamera(*camera));
    if (Light* light = object.getComponent<Light>())
        array.push_back(writeLight(*light));
    if (MeshRenderer* renderer = object.getComponent<MeshRenderer>())
        array.push_back(writeMeshRenderer(*renderer));
    if (CharacterController* controller = object.getComponent<CharacterController>())
        array.push_back(writeCharacterController(*controller));
    if (FreeFly* freeFly = object.getComponent<FreeFly>())
        array.push_back(writeFreeLookController("FreeFly", *freeFly));
    if (FPS* fps = object.getComponent<FPS>())
        array.push_back(writeFreeLookController("FPS", *fps));
    if (Orbit* orbit = object.getComponent<Orbit>())
        array.push_back(writeOrbit(*orbit));
    if (Maya* maya = object.getComponent<Maya>())
        array.push_back(writeMaya(*maya));
    if (ThirdPerson* thirdPerson = object.getComponent<ThirdPerson>())
        array.push_back(writeThirdPerson(*thirdPerson));
    if (RibbonTrail* trail = object.getComponent<RibbonTrail>())
        array.push_back(writeRibbonTrail(*trail));
    if (Billboard* billboard = object.getComponent<Billboard>())
        array.push_back(writeBillboard(*billboard));
    if (Text3D* text = object.getComponent<Text3D>())
        array.push_back(writeText3D(*text));
    if (SelfDestroy* selfDestroy = object.getComponent<SelfDestroy>())
        array.push_back(writeSelfDestroy(*selfDestroy));
    if (Collider* collider = object.getComponent<Collider>())
        array.push_back(writeCollider(*collider));
    if (Waypoints* waypoints = object.getComponent<Waypoints>())
        array.push_back(writeWaypoints(*waypoints));
    if (NavMeshSurface* surface = object.getComponent<NavMeshSurface>())
        array.push_back(writeNavMeshSurface(*surface));
    if (ParticleEffect* effect = object.getComponent<ParticleEffect>())
        array.push_back(writeParticleEffect(*effect));
    if (ParticleEmitter* emitter = object.getComponent<ParticleEmitter>())
        array.push_back(writeParticleEmitter(*emitter));
    if (Animator* animator = object.getComponent<Animator>())
        array.push_back(writeAnimator(*animator));
    if (BoneAttachment* attachment = object.getComponent<BoneAttachment>())
        array.push_back(writeBoneAttachment(*attachment));
    if (ReflectionProbe* probeComponent = object.getComponent<ReflectionProbe>())
        array.push_back(writeReflectionProbe(*probeComponent));
    const auto writeMarker = [](const char* type, const Component& component)
    {
        return nlohmann::json{{"type", type}, {"version", 1}, {"active", component.active()}};
    };
    // These components currently expose creation but no authoring controls
    // in the editor. Persisting the component and active state still matters:
    // an object created from Hierarchy must not reopen as an empty node.
    if (Terrain* component = object.getComponent<Terrain>())
        array.push_back(writeTerrain(*component));
    if (Landscape* component = object.getComponent<Landscape>())
        array.push_back(writeMarker("Landscape", *component));
    if (Road* component = object.getComponent<Road>())
        array.push_back(writeRoad(*component));
    if (Grass* component = object.getComponent<Grass>())
        array.push_back(writeGrass(*component));
    if (Hair* component = object.getComponent<Hair>())
        array.push_back(writeHair(*component));
    if (Forest* component = object.getComponent<Forest>())
        array.push_back(writeForest(*component));
    if (Ocean* component = object.getComponent<Ocean>())
        array.push_back(writeOcean(*component));
    if (VoxelWorldComponent* component = object.getComponent<VoxelWorldComponent>())
        array.push_back(writeVoxelWorld(*component));
    if (ZenBehaviour* component = object.findComponent<ZenBehaviour>())
        array.push_back(writeZenBehaviour(*component));
    // Not written: ActionRunner (no getters over its command queue - cannot
    // be read back at all, see PLANO_SERIALIZACAO_CENA.md) and a C++
    // ScriptComponent subclass other than ZenBehaviour (user extension slot,
    // no fixed shape to write).
    return array;
}

void readComponent(GameObject& object, const nlohmann::json& json, const std::string& path,
                   ComponentReadContext& context, SceneLoadResult& result)
{
    if (!json.is_object())
    {
        result.addError(path, "expected an object");
        return;
    }
    const auto typeField = json.find("type");
    if (typeField == json.end() || !typeField->is_string())
    {
        result.addError(path + ".type", "missing or not a string");
        return;
    }
    const std::string type = typeField->get<std::string>();
    const auto readMarker = [&](auto* marker)
    {
        using ComponentType = std::remove_pointer_t<decltype(marker)>;
        ComponentType* component = object.addComponent<ComponentType>();
        if (!component)
        {
            result.addError(path, "object already has a " + type + " component");
            return;
        }
        const auto active = json.find("active");
        if (active != json.end() && active->is_boolean())
            component->setActive(active->get<bool>());
    };
    if (type == "Camera")
        readCamera(object, json, path, result);
    else if (type == "Light")
        readLight(object, json, path, result);
    else if (type == "MeshRenderer")
        readMeshRenderer(object, json, path, result,
                         context.out.runningInEditor() || context.out.asyncMeshLoad());
    else if (type == "ReflectionProbe")
        readReflectionProbe(object, json, path, result);
    else if (type == "Terrain")
        readTerrain(object, json, path, result);
    else if (type == "Landscape")
        readMarker(static_cast<Landscape*>(nullptr));
    else if (type == "Road")
        readRoad(object, json, path, context.out, result);
    else if (type == "Grass")
        readGrass(object, json, path, result);
    else if (type == "Hair")
        readHair(object, json, path, result);
    else if (type == "Forest")
        readForest(object, json, path, result);
    else if (type == "Ocean")
        readOcean(object, json, path, result);
    else if (type == "VoxelWorld")
        readVoxelWorld(object, json, path, result);
    else if (type == "CharacterController")
        readCharacterController(object, json, path, result);
    else if (type == "FreeFly")
        readFreeLookController<FreeFly>(object, json, path, result);
    else if (type == "FPS")
        readFreeLookController<FPS>(object, json, path, result);
    else if (type == "Orbit")
        readOrbit(object, json, path, context.out, result);
    else if (type == "Maya")
        readMaya(object, json, path, context.out, result);
    else if (type == "ThirdPerson")
        readThirdPerson(object, json, path, context.out, result);
    else if (type == "RibbonTrail")
        readRibbonTrail(object, json, path, context.out, result);
    else if (type == "Billboard")
        readBillboard(object, json, path, result);
    else if (type == "Text3D")
        readText3D(object, json, path, result);
    else if (type == "SelfDestroy")
        readSelfDestroy(object, json, path, result);
    else if (type == "Collider")
        readCollider(object, json, path, result);
    else if (type == "ZenBehaviour")
        readZenBehaviour(object, json, path, result);
    else if (type == "Waypoints")
        readWaypoints(object, json, path, result);
    else if (type == "NavMeshSurface")
        readNavMeshSurface(object, json, path, result);
    else if (type == "ParticleEffect")
        readParticleEffect(object, json, path, result);
    else if (type == "ParticleEmitter")
        readParticleEmitter(object, json, path, result);
    else if (type == "Animator")
        readAnimator(object, json, path, result);
    else if (type == "BoneAttachment")
        readBoneAttachment(object, json, path, context.pendingBoneAttachments, result);
    else if (type == "ActionRunner")
        result.addWarning(path + ".type",
                          "ActionRunner is not serializable yet (no read-back API), ignored");
    else
        result.addWarning(path + ".type", "unknown component type '" + type + "', ignored");
}

// One GameObject entry read out of the "objects" array, before any Scene
// call is made. Parsing and hierarchy validation happen entirely over this
// - not over GameObject/Scene - so a malformed file never has a chance to
// mutate `out` (see SceneSerializer.h's transactional contract).
struct ParsedObject
{
    u64 id = 0; // 0 means the id field itself failed to parse
    bool hasParent = false;
    u64 parentId = 0;
    std::string name;
    std::string tag;
    bool active = true;
    bool visible = true;
    bool isStatic = false;
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};
};

void parseObjects(const nlohmann::json& objectsJson, std::vector<ParsedObject>& parsed,
                  SceneLoadResult& result)
{
    parsed.reserve(objectsJson.size());
    for (usize i = 0; i < objectsJson.size(); ++i)
    {
        const std::string base = indexPath(i);
        const nlohmann::json& entry = objectsJson[i];
        if (!entry.is_object())
        {
            result.addError(base, "expected an object");
            parsed.emplace_back();
            continue;
        }

        ParsedObject object;

        const auto idField = entry.find("id");
        u64 idValue = 0;
        if (idField == entry.end() || !readNonNegativeInteger(*idField, idValue) || idValue == 0)
            result.addError(base + ".id", "missing or invalid (must be a positive integer)");
        else
            object.id = idValue;

        const auto parentField = entry.find("parent");
        u64 parentValue = 0;
        if (parentField == entry.end())
        {
            result.addError(base + ".parent", "missing field");
        }
        else if (parentField->is_null())
        {
            object.hasParent = false;
        }
        else if (readNonNegativeInteger(*parentField, parentValue) && parentValue != 0)
        {
            object.hasParent = true;
            object.parentId = parentValue;
        }
        else
        {
            result.addError(base + ".parent", "must be null or a positive integer id");
        }

        if (object.id != 0 && object.hasParent && object.parentId == object.id)
            result.addError(base + ".parent", "an object cannot be its own parent");

        const auto nameField = entry.find("name");
        if (nameField != entry.end() && nameField->is_string())
            object.name = nameField->get<std::string>();
        else
            result.addWarning(base + ".name", "missing or not a string, defaulting to empty");

        const auto tagField = entry.find("tag");
        if (tagField != entry.end() && tagField->is_string())
            object.tag = tagField->get<std::string>();

        const auto flagsField = entry.find("flags");
        if (flagsField != entry.end() && flagsField->is_object())
        {
            const auto activeField = flagsField->find("active");
            if (activeField != flagsField->end() && activeField->is_boolean())
                object.active = activeField->get<bool>();
            const auto visibleField = flagsField->find("visible");
            if (visibleField != flagsField->end() && visibleField->is_boolean())
                object.visible = visibleField->get<bool>();
            const auto staticField = flagsField->find("static");
            if (staticField != flagsField->end() && staticField->is_boolean())
                object.isStatic = staticField->get<bool>();
        }
        else
        {
            result.addWarning(base + ".flags", "missing, defaulting to active/visible/non-static");
        }

        const auto transformField = entry.find("transform");
        if (transformField == entry.end() || !transformField->is_object())
        {
            result.addError(base + ".transform", "missing transform block");
        }
        else
        {
            const auto positionField = transformField->find("position");
            if (positionField == transformField->end() ||
                !readVec3(*positionField, object.position))
                result.addError(base + ".transform.position",
                                "expected [x, y, z] of finite numbers");

            const auto rotationField = transformField->find("rotation");
            if (rotationField == transformField->end() ||
                !readQuat(*rotationField, object.rotation))
                result.addError(base + ".transform.rotation",
                                "expected [x, y, z, w], a non-degenerate quaternion");

            const auto scaleField = transformField->find("scale");
            if (scaleField == transformField->end() || !readVec3(*scaleField, object.scale) ||
                glm::abs(object.scale.x) < kMinScaleComponent ||
                glm::abs(object.scale.y) < kMinScaleComponent ||
                glm::abs(object.scale.z) < kMinScaleComponent)
                result.addError(base + ".transform.scale",
                                "expected [x, y, z] of finite, non-zero numbers");
        }

        // Components are Fase 2's dispatch, not read here - the field is
        // reserved by the format but simply skipped for now.

        parsed.push_back(object);
    }
}

// Validates the id/parent graph and returns a creation order where every
// parent lands before its children - Scene::createGameObject() needs the
// parent GameObject* to already exist. Repeated array sweeps (Kahn's
// algorithm) rather than an adjacency list: scenes here are hundreds to
// thousands of objects, not the size where the O(depth) extra sweeps show up.
bool buildCreationOrder(const std::vector<ParsedObject>& parsed, std::vector<usize>& order,
                        SceneLoadResult& result)
{
    HashMap<u64, usize> idToIndex;
    for (usize i = 0; i < parsed.size(); ++i)
    {
        if (parsed[i].id == 0)
            continue; // already an error from parseObjects
        const auto existing = idToIndex.find(parsed[i].id);
        if (existing != idToIndex.end())
        {
            result.addError(indexPath(i) + ".id", "duplicate id " + std::to_string(parsed[i].id) +
                                                      ", first used by " +
                                                      indexPath(existing->second));
            continue;
        }
        idToIndex[parsed[i].id] = i;
    }

    for (usize i = 0; i < parsed.size(); ++i)
    {
        const ParsedObject& object = parsed[i];
        if (object.id == 0 || !object.hasParent || object.parentId == object.id)
            continue; // self-parent already reported by parseObjects
        if (idToIndex.find(object.parentId) == idToIndex.end())
            result.addError(indexPath(i) + ".parent",
                            "parent id " + std::to_string(object.parentId) + " does not exist");
    }

    if (!result.success())
        return false;

    std::vector<bool> placed(parsed.size(), false);
    order.clear();
    order.reserve(parsed.size());
    usize remaining = parsed.size();
    while (remaining > 0)
    {
        usize placedThisSweep = 0;
        for (usize i = 0; i < parsed.size(); ++i)
        {
            if (placed[i])
                continue;
            const ParsedObject& object = parsed[i];
            const bool ready = !object.hasParent || placed[idToIndex[object.parentId]];
            if (!ready)
                continue;
            placed[i] = true;
            order.push_back(i);
            ++placedThisSweep;
            --remaining;
        }
        if (placedThisSweep == 0)
        {
            // Every still-unplaced object's parent chain loops back on
            // itself - direct self-parent is caught above, this is the
            // longer A -> B -> A case.
            for (usize i = 0; i < parsed.size(); ++i)
                if (!placed[i])
                    result.addError(indexPath(i) + ".parent", "parent cycle");
            return false;
        }
    }
    return true;
}

bool createObjects(Scene& out, const std::vector<ParsedObject>& parsed,
                   const std::vector<usize>& order, SceneLoadResult& result)
{
    HashMap<u64, GameObject*> idToObject;
    for (usize index : order)
    {
        const ParsedObject& object = parsed[index];
        GameObject* parent = object.hasParent ? idToObject[object.parentId] : nullptr;
        GameObject* created = out.createGameObject(object.id, object.name, parent);
        if (!created)
        {
            result.addError(indexPath(index) + ".id",
                            "Scene refused id " + std::to_string(object.id));
            return false;
        }
        created->setTag(object.tag);
        created->setActive(object.active);
        created->setVisible(object.visible);
        created->setStatic(object.isStatic);
        created->setPosition(object.position);
        created->setRotation(object.rotation);
        created->setScale(object.scale);
        idToObject[object.id] = created;
    }
    return true;
}

// Every object from createObjects() already exists (this only runs once that
// pass succeeded in full), so components attach straight onto them - no id
// resolution needed here, `objectsJson` and `parsed` still line up 1:1 by
// array index the way parseObjects() built them.
void readComponentsPass(Scene& out, const nlohmann::json& objectsJson,
                        const std::vector<ParsedObject>& parsed, ComponentReadContext& context,
                        SceneLoadResult& result)
{
    for (usize i = 0; i < parsed.size(); ++i)
    {
        if (parsed[i].id == 0)
            continue; // never created - already an error from parseObjects
        GameObject* object = out.findGameObject(parsed[i].id);
        if (!object)
            continue;
        const auto componentsField = objectsJson[i].find("components");
        if (componentsField == objectsJson[i].end())
            continue;
        if (!componentsField->is_array())
        {
            result.addError(indexPath(i) + ".components", "must be an array");
            continue;
        }
        for (usize c = 0; c < componentsField->size(); ++c)
            readComponent(*object, (*componentsField)[c],
                          indexPath(i) + ".components[" + std::to_string(c) + "]", context, result);
    }
}

void collectPreOrder(GameObject& node, std::vector<GameObject*>& out)
{
    for (usize i = 0; i < node.childCount(); ++i)
    {
        GameObject* child = node.child(i);
        out.push_back(child);
        collectPreOrder(*child, out);
    }
}

nlohmann::json writeObject(GameObject& object, GameObject& root)
{
    nlohmann::json json;
    json["id"] = object.id();
    json["parent"] =
        object.parent() == &root ? nlohmann::json(nullptr) : nlohmann::json(object.parent()->id());
    json["name"] = object.name();
    json["tag"] = object.tag();

    nlohmann::json flags;
    flags["active"] = object.active();
    flags["visible"] = object.visible();
    flags["static"] = object.isStatic();
    json["flags"] = flags;

    nlohmann::json transform;
    const glm::vec3& position = object.position();
    const glm::quat& rotation = object.rotation();
    const glm::vec3& scale = object.scale();
    transform["position"] = {position.x, position.y, position.z};
    // [x, y, z, w] on disk - see readQuat() for the constructor-order note.
    transform["rotation"] = {rotation.x, rotation.y, rotation.z, rotation.w};
    transform["scale"] = {scale.x, scale.y, scale.z};
    json["transform"] = transform;

    json["components"] = writeComponents(object);

    return json;
}

// --------------------------------------------------------- render settings

nlohmann::json writeCascadeShadowSettings(const CascadeShadowSettings& settings)
{
    nlohmann::json json;
    json["count"] = settings.count;
    json["resolution"] = settings.resolution;
    json["lambda"] = settings.lambda;
    json["distance"] = settings.distance;
    json["casterExtrusion"] = settings.casterExtrusion;
    json["depthBiasSlope"] = settings.depthBiasSlope;
    json["depthBiasConstant"] = settings.depthBiasConstant;
    json["quality"] = settings.quality;
    json["splitOffsets"] = {settings.splitOffset[0], settings.splitOffset[1],
                            settings.splitOffset[2]};
    json["bias"] = settings.bias;
    json["normalBias"] = settings.normalBias;
    json["pancakeSize"] = settings.pancakeSize;
    json["blur"] = settings.blur;
    json["fadeStart"] = settings.fadeStart;
    json["opacity"] = settings.opacity;
    json["angularDiameter"] = settings.angularDiameter;
    json["stabilize"] = settings.stabilize;
    json["blend"] = settings.blend;
    json["cullFront"] = settings.cullFront;
    json["enabled"] = settings.enabled;
    return json;
}

void readCascadeShadowSettings(const nlohmann::json& json, CascadeShadowSettings& out)
{
    out.count = json.value("count", out.count);
    out.resolution = json.value("resolution", out.resolution);
    out.lambda = json.value("lambda", out.lambda);
    out.distance = json.value("distance", out.distance);
    out.casterExtrusion = json.value("casterExtrusion", out.casterExtrusion);
    out.depthBiasSlope = json.value("depthBiasSlope", out.depthBiasSlope);
    out.depthBiasConstant = json.value("depthBiasConstant", out.depthBiasConstant);
    out.quality = json.value("quality", out.quality);
    const auto splitField = json.find("splitOffsets");
    if (splitField != json.end() && splitField->is_array() && splitField->size() == 3)
        for (usize i = 0; i < 3; ++i)
            if ((*splitField)[i].is_number())
                out.splitOffset[i] = (*splitField)[i].get<f32>();
    out.bias = json.value("bias", out.bias);
    out.normalBias = json.value("normalBias", out.normalBias);
    out.pancakeSize = json.value("pancakeSize", out.pancakeSize);
    out.blur = json.value("blur", out.blur);
    out.fadeStart = json.value("fadeStart", out.fadeStart);
    out.opacity = json.value("opacity", out.opacity);
    out.angularDiameter = json.value("angularDiameter", out.angularDiameter);
    out.stabilize = json.value("stabilize", out.stabilize);
    out.blend = json.value("blend", out.blend);
    out.cullFront = json.value("cullFront", out.cullFront);
    out.enabled = json.value("enabled", out.enabled);
}

nlohmann::json writeShadowAtlasSettings(const ShadowAtlasSettings& settings)
{
    nlohmann::json json;
    json["size"] = settings.size;
    json["maximumTileSize"] = settings.maximumTileSize;
    json["minimumTileSize"] = settings.minimumTileSize;
    json["volumetricPriority"] = settings.volumetricPriority;
    json["pointPriority"] = settings.pointPriority;
    json["pointBias"] = settings.pointBias;
    json["biasSlope"] = settings.biasSlope;
    json["biasConstant"] = settings.biasConstant;
    json["point"] = settings.point;
    json["spot"] = settings.spot;
    return json;
}

void readShadowAtlasSettings(const nlohmann::json& json, ShadowAtlasSettings& out)
{
    out.size = json.value("size", out.size);
    out.maximumTileSize = json.value("maximumTileSize", out.maximumTileSize);
    out.minimumTileSize = json.value("minimumTileSize", out.minimumTileSize);
    out.volumetricPriority = json.value("volumetricPriority", out.volumetricPriority);
    out.pointPriority = json.value("pointPriority", out.pointPriority);
    out.pointBias = json.value("pointBias", out.pointBias);
    out.biasSlope = json.value("biasSlope", out.biasSlope);
    out.biasConstant = json.value("biasConstant", out.biasConstant);
    out.point = json.value("point", out.point);
    out.spot = json.value("spot", out.spot);
}

const char* toneMapModeName(ToneMapMode mode)
{
    switch (mode)
    {
    case ToneMapMode::None:
        return "None";
    case ToneMapMode::Reinhard:
        return "Reinhard";
    case ToneMapMode::ACES:
        return "ACES";
    }
    return "ACES";
}

bool toneMapModeFromName(const std::string& name, ToneMapMode& out)
{
    if (name == "None")
        out = ToneMapMode::None;
    else if (name == "Reinhard")
        out = ToneMapMode::Reinhard;
    else if (name == "ACES")
        out = ToneMapMode::ACES;
    else
        return false;
    return true;
}

const char* postEffectName(PostEffect effect)
{
    switch (effect)
    {
    case PostEffect::ToneMap:
        return "ToneMap";
    case PostEffect::Bloom:
        return "Bloom";
    case PostEffect::FXAA:
        return "FXAA";
    }
    return "ToneMap";
}

bool postEffectFromName(const std::string& name, PostEffect& out)
{
    if (name == "ToneMap")
        out = PostEffect::ToneMap;
    else if (name == "Bloom")
        out = PostEffect::Bloom;
    else if (name == "FXAA")
        out = PostEffect::FXAA;
    else
        return false;
    return true;
}

nlohmann::json writePostProcessSettings(const PostProcessStack& stack)
{
    nlohmann::json json;
    json["exposure"] = stack.exposure;
    json["toneMap"] = toneMapModeName(stack.toneMap);
    json["bloomThreshold"] = stack.bloomThreshold;
    json["bloomSoftKnee"] = stack.bloomSoftKnee;
    json["bloomStrength"] = stack.bloomStrength;
    json["fxaaSubpixel"] = stack.fxaaSubpixel;
    json["fxaaEdgeThreshold"] = stack.fxaaEdgeThreshold;
    json["fxaaEdgeThresholdMin"] = stack.fxaaEdgeThresholdMin;
    json["ssaoEnabled"] = stack.ssaoEnabled;
    json["ssaoRadius"] = stack.ssaoRadius;
    json["ssaoBias"] = stack.ssaoBias;
    json["ssaoIntensity"] = stack.ssaoIntensity;
    json["ssaoSamples"] = stack.ssaoSamples;
    json["ssaoDepthSigma"] = stack.ssaoDepthSigma;
    json["ssaoBlur"] = stack.ssaoBlur;
    json["ssaoDebug"] = stack.ssaoDebug;
    json["taaEnabled"] = stack.taaEnabled;
    json["taaFeedback"] = stack.taaFeedback;
    json["taaMotionFeedback"] = stack.taaMotionFeedback;
    json["taaClipWidth"] = stack.taaClipWidth;
    json["taaSharpness"] = stack.taaSharpness;
    json["enabled"] = stack.enabled;

    // Order matters (each layer draws into the next) - an array, not a set
    // of per-effect enabled flags, so move()'d ordering round-trips too.
    nlohmann::json layers = nlohmann::json::array();
    for (const PostLayer& layer : stack.layers())
    {
        nlohmann::json entry;
        entry["effect"] = postEffectName(layer.effect);
        entry["enabled"] = layer.enabled;
        layers.push_back(entry);
    }
    json["layers"] = layers;
    return json;
}

void readPostProcessSettings(const nlohmann::json& json, PostProcessStack& out)
{
    out.exposure = json.value("exposure", out.exposure);
    ToneMapMode toneMap;
    if (toneMapModeFromName(json.value("toneMap", std::string(toneMapModeName(out.toneMap))),
                            toneMap))
        out.toneMap = toneMap;
    out.bloomThreshold = json.value("bloomThreshold", out.bloomThreshold);
    out.bloomSoftKnee = json.value("bloomSoftKnee", out.bloomSoftKnee);
    out.bloomStrength = json.value("bloomStrength", out.bloomStrength);
    out.fxaaSubpixel = json.value("fxaaSubpixel", out.fxaaSubpixel);
    out.fxaaEdgeThreshold = json.value("fxaaEdgeThreshold", out.fxaaEdgeThreshold);
    out.fxaaEdgeThresholdMin = json.value("fxaaEdgeThresholdMin", out.fxaaEdgeThresholdMin);
    out.ssaoEnabled = json.value("ssaoEnabled", out.ssaoEnabled);
    out.ssaoRadius = json.value("ssaoRadius", out.ssaoRadius);
    out.ssaoBias = json.value("ssaoBias", out.ssaoBias);
    out.ssaoIntensity = json.value("ssaoIntensity", out.ssaoIntensity);
    out.ssaoSamples = json.value("ssaoSamples", out.ssaoSamples);
    out.ssaoDepthSigma = json.value("ssaoDepthSigma", out.ssaoDepthSigma);
    out.ssaoBlur = json.value("ssaoBlur", out.ssaoBlur);
    out.ssaoDebug = json.value("ssaoDebug", out.ssaoDebug);
    out.taaEnabled = json.value("taaEnabled", out.taaEnabled);
    out.taaFeedback = json.value("taaFeedback", out.taaFeedback);
    out.taaMotionFeedback = json.value("taaMotionFeedback", out.taaMotionFeedback);
    out.taaClipWidth = json.value("taaClipWidth", out.taaClipWidth);
    out.taaSharpness = json.value("taaSharpness", out.taaSharpness);
    out.enabled = json.value("enabled", out.enabled);

    const auto layersField = json.find("layers");
    if (layersField != json.end() && layersField->is_array())
    {
        // Replaces the whole chain rather than patching it in place - a
        // saved scene's layer list (which effects, what order) is the
        // authority once one exists, the same way loading a scene's object
        // list replaces whatever was there before, not merges with it.
        out.clear();
        for (const nlohmann::json& entry : *layersField)
        {
            if (!entry.is_object())
                continue;
            PostEffect effect;
            if (!postEffectFromName(entry.value("effect", std::string()), effect))
                continue;
            PostLayer& layer = out.add(effect);
            layer.enabled = entry.value("enabled", true);
        }
    }
}

nlohmann::json writeLensFlareSettings(const LensFlarePass& pass)
{
    nlohmann::json json;
    json["enabled"] = pass.enabled;
    json["occlusionRadius"] = pass.occlusionRadius;
    json["sunDistance"] = pass.sunDistance;
    json["debugOcclusion"] = pass.debugOcclusion;
    return json;
}

void readLensFlareSettings(const nlohmann::json& json, LensFlarePass& out)
{
    out.enabled = json.value("enabled", out.enabled);
    out.occlusionRadius = json.value("occlusionRadius", out.occlusionRadius);
    out.sunDistance = json.value("sunDistance", out.sunDistance);
    out.debugOcclusion = json.value("debugOcclusion", out.debugOcclusion);
}

nlohmann::json writeEnvironmentProbeSettings(const EnvironmentProbe& probe)
{
    return {{"enabled", probe.enabled},
            {"resolution", probe.resolution()},
            {"refresh", environmentProbeRefreshName(probe.refresh)},
            {"interval", probe.interval},
            {"content", environmentProbeContentName(probe.content)},
            {"position", writeVec3(probe.position)},
            {"extents", writeVec3(probe.extents)},
            {"influenceRadius", probe.influenceRadius},
            {"intensity", probe.intensity},
            {"nearPlane", probe.nearPlane},
            {"farPlane", probe.farPlane}};
}

void readEnvironmentProbeSettings(const nlohmann::json& json, EnvironmentProbe& probe)
{
    const u32 resolution = json.value("resolution", probe.resolution());
    if (resolution > 0 && resolution != probe.resolution())
        probe.create(resolution);
    probe.enabled = json.value("enabled", probe.enabled);
    EnvironmentProbe::Refresh refresh;
    if (environmentProbeRefreshFromName(json.value("refresh", std::string()), refresh))
        probe.refresh = refresh;
    EnvironmentProbe::Content content;
    if (environmentProbeContentFromName(json.value("content", std::string()), content))
        probe.content = content;
    probe.interval = json.value("interval", probe.interval);
    const auto position = json.find("position");
    if (position != json.end())
        readVec3(*position, probe.position);
    const auto extents = json.find("extents");
    if (extents != json.end())
        readVec3(*extents, probe.extents);
    probe.influenceRadius = json.value("influenceRadius", probe.influenceRadius);
    probe.intensity = json.value("intensity", probe.intensity);
    probe.nearPlane = json.value("nearPlane", probe.nearPlane);
    probe.farPlane = json.value("farPlane", probe.farPlane);
    probe.invalidate();
}

nlohmann::json writeLightingSettings(const Lighting& lighting)
{
    return {{"tiled", lighting.tiled},
            {"use25D", lighting.use25D},
            {"debugTiles", lighting.debugTiles},
            {"decalsEnabled", lighting.decalsEnabled}};
}

void readLightingSettings(const nlohmann::json& json, Lighting& lighting)
{
    lighting.tiled = json.value("tiled", lighting.tiled);
    lighting.use25D = json.value("use25D", lighting.use25D);
    lighting.debugTiles = json.value("debugTiles", lighting.debugTiles);
    lighting.decalsEnabled = json.value("decalsEnabled", lighting.decalsEnabled);
}

nlohmann::json writeVolumetricSettings(const VolumetricPass& pass)
{
    return {{"sunEnabled", pass.sunEnabled}, {"spotEnabled", pass.spotEnabled},
            {"pointEnabled", pass.pointEnabled}, {"rectEnabled", pass.rectEnabled},
            {"blurEnabled", pass.blurEnabled}, {"samples", pass.samples},
            {"scattering", pass.scattering}, {"maxDistance", pass.maxDistance},
            {"strength", pass.strength}, {"debugFallback", pass.debugFallback},
            {"sunDensity", pass.sunDensity}, {"spotDensity", pass.spotDensity},
            {"spotStrength", pass.spotStrength}, {"pointDensity", pass.pointDensity},
            {"pointStrength", pass.pointStrength}, {"rectDensity", pass.rectDensity},
            {"rectStrength", pass.rectStrength}, {"pointProxyIsCube", pass.pointProxyIsCube}};
}

void readVolumetricSettings(const nlohmann::json& json, VolumetricPass& pass)
{
    pass.sunEnabled = json.value("sunEnabled", pass.sunEnabled);
    pass.spotEnabled = json.value("spotEnabled", pass.spotEnabled);
    pass.pointEnabled = json.value("pointEnabled", pass.pointEnabled);
    pass.rectEnabled = json.value("rectEnabled", pass.rectEnabled);
    pass.blurEnabled = json.value("blurEnabled", pass.blurEnabled);
    pass.samples = json.value("samples", pass.samples);
    pass.scattering = json.value("scattering", pass.scattering);
    pass.maxDistance = json.value("maxDistance", pass.maxDistance);
    pass.strength = json.value("strength", pass.strength);
    pass.debugFallback = json.value("debugFallback", pass.debugFallback);
    pass.sunDensity = json.value("sunDensity", pass.sunDensity);
    pass.spotDensity = json.value("spotDensity", pass.spotDensity);
    pass.spotStrength = json.value("spotStrength", pass.spotStrength);
    pass.pointDensity = json.value("pointDensity", pass.pointDensity);
    pass.pointStrength = json.value("pointStrength", pass.pointStrength);
    pass.rectDensity = json.value("rectDensity", pass.rectDensity);
    pass.rectStrength = json.value("rectStrength", pass.rectStrength);
    pass.pointProxyIsCube = json.value("pointProxyIsCube", pass.pointProxyIsCube);
}

nlohmann::json writeSkySettings(const SkySettings& sky)
{
    return {{"enabled", sky.enabled}, {"mode", static_cast<u8>(sky.mode)},
            {"automaticSun", sky.automaticSun}, {"sunFromSky", sky.sunFromSky},
            {"timeOfDay", sky.timeOfDay},
            {"sunAzimuth", sky.sunAzimuth}, {"sunElevation", sky.sunElevation},
            {"northOffset", sky.northOffset}, {"maximumElevation", sky.maximumElevation},
            {"ambient", writeVec3(sky.ambient)}, {"ambientStrength", sky.ambientStrength},
            {"intensity", sky.intensity}, {"lightmapIntensity", sky.lightmapIntensity},
            {"lightmapShadowLift", sky.lightmapShadowLift}, {"sunIntensity", sky.sunIntensity},
            {"rayleigh", sky.rayleigh}, {"mie", sky.mie}, {"mieG", sky.mieG},
            {"atmosphereExposure", sky.atmosphereExposure}, {"viewSteps", sky.viewSteps},
            {"lightSteps", sky.lightSteps}, {"cubemapName", sky.cubemapName},
            {"cloudsEnabled", sky.cloudsEnabled}, {"cloudHeight", sky.cloudHeight},
            {"cloudScale", sky.cloudScale}, {"cloudCoverage", sky.cloudCoverage},
            {"cloudDensity", sky.cloudDensity}, {"cloudSpeed", sky.cloudSpeed},
            {"cloudDirection", {sky.cloudDirection.x, sky.cloudDirection.y}},
            {"cloudColor", writeVec3(sky.cloudColor)}};
}

void readSkySettings(const nlohmann::json& json, SkySettings& sky)
{
    sky.enabled = json.value("enabled", sky.enabled);
    sky.mode = static_cast<SkyMode>(json.value("mode", static_cast<u8>(sky.mode)));
    sky.automaticSun = json.value("automaticSun", sky.automaticSun);
    sky.sunFromSky = json.value("sunFromSky", sky.sunFromSky);
    sky.timeOfDay = json.value("timeOfDay", sky.timeOfDay);
    sky.sunAzimuth = json.value("sunAzimuth", sky.sunAzimuth);
    sky.sunElevation = json.value("sunElevation", sky.sunElevation);
    sky.northOffset = json.value("northOffset", sky.northOffset);
    sky.maximumElevation = json.value("maximumElevation", sky.maximumElevation);
    const auto ambient = json.find("ambient");
    if (ambient != json.end()) readVec3(*ambient, sky.ambient);
    sky.ambientStrength = json.value("ambientStrength", sky.ambientStrength);
    sky.intensity = json.value("intensity", sky.intensity);
    sky.lightmapIntensity = json.value("lightmapIntensity", sky.lightmapIntensity);
    sky.lightmapShadowLift = json.value("lightmapShadowLift", sky.lightmapShadowLift);
    sky.sunIntensity = json.value("sunIntensity", sky.sunIntensity);
    sky.rayleigh = json.value("rayleigh", sky.rayleigh);
    sky.mie = json.value("mie", sky.mie);
    sky.mieG = json.value("mieG", sky.mieG);
    sky.atmosphereExposure = json.value("atmosphereExposure", sky.atmosphereExposure);
    sky.viewSteps = json.value("viewSteps", sky.viewSteps);
    sky.lightSteps = json.value("lightSteps", sky.lightSteps);
    const std::string cubemapName = json.value("cubemapName", sky.cubemapName);
    if (!cubemapName.empty() && cubemapName != sky.cubemapName)
        loadSkyCubemap(sky, cubemapName);
    sky.cloudsEnabled = json.value("cloudsEnabled", sky.cloudsEnabled);
    sky.cloudHeight = json.value("cloudHeight", sky.cloudHeight);
    sky.cloudScale = json.value("cloudScale", sky.cloudScale);
    sky.cloudCoverage = json.value("cloudCoverage", sky.cloudCoverage);
    sky.cloudDensity = json.value("cloudDensity", sky.cloudDensity);
    sky.cloudSpeed = json.value("cloudSpeed", sky.cloudSpeed);
    const auto direction = json.find("cloudDirection");
    if (direction != json.end() && direction->is_array() && direction->size() == 2)
        sky.cloudDirection = glm::vec2((*direction)[0].get<f32>(), (*direction)[1].get<f32>());
    const auto color = json.find("cloudColor");
    if (color != json.end()) readVec3(*color, sky.cloudColor);
}

} // namespace

bool SceneLoadResult::success() const
{
    for (const SceneDiagnostic& diagnostic : diagnostics)
        if (diagnostic.severity == SceneDiagnosticSeverity::Error)
            return false;
    return true;
}

void SceneLoadResult::addWarning(const std::string& jsonPath, const std::string& message)
{
    diagnostics.push_back({SceneDiagnosticSeverity::Warning, jsonPath, message});
}

void SceneLoadResult::addError(const std::string& jsonPath, const std::string& message)
{
    diagnostics.push_back({SceneDiagnosticSeverity::Error, jsonPath, message});
}

nlohmann::json SceneSerializer::toJson(const Scene& scene,
                                       const SceneRenderSettings* settings) const
{
    // root()/child()/parent() stay non-const even through a const Scene& -
    // an existing pattern on GameObject (see GameObject.h), not something
    // introduced here. Nothing below mutates the hierarchy.
    GameObject& root = const_cast<Scene&>(scene).root();

    std::vector<GameObject*> objects;
    collectPreOrder(root, objects);

    nlohmann::json objectsJson = nlohmann::json::array();
    for (GameObject* object : objects)
        objectsJson.push_back(writeObject(*object, root));

    nlohmann::json sceneJson;
    sceneJson["name"] = root.name();
    Camera* activeCamera = scene.activeCamera();
    sceneJson["activeCamera"] = (activeCamera && activeCamera->owner())
                                    ? nlohmann::json(activeCamera->owner()->id())
                                    : nlohmann::json(nullptr);
    sceneJson["objects"] = objectsJson;
    sceneJson["culling"] = {{"static", scene.staticCullingEnabled()},
                             {"dynamic", scene.dynamicCullingEnabled()},
                             {"occlusion", scene.occlusionQueryEnabled()}};

    nlohmann::json document;
    document["format"] = kFormatName;
    document["version"] = kFormatVersion;
    document["scene"] = sceneJson;

    // Absent entirely (not even an empty object) when the caller passed no
    // settings, or owns none of the three - an old reader opening a file a
    // new one wrote sees nothing different for a section it never asked for.
    if (settings)
    {
        nlohmann::json renderSettingsJson = renderSettingsToJson(*settings);
        if (!renderSettingsJson.empty())
            document["renderSettings"] = renderSettingsJson;
    }
    return document;
}

nlohmann::json SceneSerializer::renderSettingsToJson(const SceneRenderSettings& settings) const
{
    nlohmann::json json;
    if (settings.shadows)
        json["cascadeShadows"] = writeCascadeShadowSettings(*settings.shadows);
    if (settings.shadowAtlas)
        json["shadowAtlas"] = writeShadowAtlasSettings(*settings.shadowAtlas);
    if (settings.postProcess)
        json["postProcess"] = writePostProcessSettings(*settings.postProcess);
    if (settings.lensFlare)
        json["lensFlare"] = writeLensFlareSettings(*settings.lensFlare);
    if (settings.environmentProbe)
        json["environmentProbe"] = writeEnvironmentProbeSettings(*settings.environmentProbe);
    if (settings.lighting)
        json["lighting"] = writeLightingSettings(*settings.lighting);
    if (settings.volumetric)
        json["volumetric"] = writeVolumetricSettings(*settings.volumetric);
    if (settings.sky)
        json["sky"] = writeSkySettings(*settings.sky);
    if (settings.renderResolution)
        json["renderResolution"] = {{"width", settings.renderResolution->width},
                                    {"height", settings.renderResolution->height},
                                    {"scale", settings.renderResolution->scale}};
    if (settings.particles)
        json["particles"] = {{"texture", settings.particles->textureFile()}};
    return json;
}

bool SceneSerializer::fromJson(const nlohmann::json& root, Scene& out, SceneLoadResult& result,
                               const SceneRenderSettings* settings) const
{
    if (!root.is_object())
    {
        result.addError("", "root is not a JSON object");
        return false;
    }

    const auto formatField = root.find("format");
    if (formatField == root.end() || !formatField->is_string() ||
        formatField->get<std::string>() != kFormatName)
    {
        result.addError("format", std::string("missing or not '") + kFormatName + "'");
        return false;
    }

    const auto versionField = root.find("version");
    u64 versionValue = 0;
    if (versionField == root.end() || !readNonNegativeInteger(*versionField, versionValue) ||
        versionValue == 0)
    {
        result.addError("version", "missing or not a positive integer");
        return false;
    }
    const u32 version = static_cast<u32>(versionValue);
    if (version > kFormatVersion)
    {
        result.addError("version", "file is version " + std::to_string(version) +
                                       ", newer than this build's " +
                                       std::to_string(kFormatVersion));
        return false;
    }
    if (version < kFormatVersion)
    {
        // No migrations exist yet - every version below current is
        // unreadable until SceneSerializer grows a migrateVNtoVN+1() for it.
        result.addError("version", "file is version " + std::to_string(version) +
                                       " and no migration to " + std::to_string(kFormatVersion) +
                                       " exists yet");
        return false;
    }

    const auto sceneField = root.find("scene");
    if (sceneField == root.end() || !sceneField->is_object())
    {
        result.addError("scene", "missing scene object");
        return false;
    }

    nlohmann::json emptyArray = nlohmann::json::array();
    const auto objectsField = sceneField->find("objects");
    if (objectsField != sceneField->end() && !objectsField->is_array())
        result.addError("scene.objects", "must be an array");
    const nlohmann::json& objectsJson =
        (objectsField != sceneField->end() && objectsField->is_array()) ? *objectsField
                                                                        : emptyArray;

    std::vector<ParsedObject> parsed;
    parseObjects(objectsJson, parsed, result);

    std::vector<usize> order;
    buildCreationOrder(parsed, order, result);

    if (!result.success())
        return false;

    if (!createObjects(out, parsed, order, result))
        return false;

    // Components attach directly onto mComponents, independent of whether
    // the owning object has been flushed into Scene's own lists yet (see the
    // update(0.0f) comment right below) - so this can run before that flush
    // and registerBranch() still picks every one of them up correctly.
    std::vector<PendingBoneAttachment> pendingBoneAttachments;
    ComponentReadContext context{out, pendingBoneAttachments};
    readComponentsPass(out, objectsJson, parsed, context, result);

    // Fourth pass: cross-object references that needed every component to
    // exist first - a BoneAttachment's target Animator may have been read
    // after the BoneAttachment itself.
    resolveBoneAttachments(out, pendingBoneAttachments, result);

    // Scene::createGameObject() with no parent queues the object rather than
    // linking it under root() right away (see Scene::add()'s mPendingAdd
    // path) - by design, so code mid-frame never mutates mObjects out from
    // under an iteration in progress. A load is not mid-frame: the caller
    // expects `out` fully usable the moment this returns, so the queue is
    // flushed here. deltaTime 0 means no component actually ticks - Fase 2
    // is the first phase with components to create, so this is inert today.
    // It also touches ParticleEffectPool (rebinds its singleton to `out`),
    // consistent with `out` being expected fresh per this class's contract.
    out.update(0.0f);

    // Scene::mSunLight is never itself part of a DirectionalLight's own
    // serialized data (writeLight() has no field for it) - nothing else
    // ever set it either (setSunLight() has exactly one caller in the whole
    // tree, a demo's own main.cpp), so a scene reload always drops the link
    // even when the object it pointed at is right there in the file. The
    // first DirectionalLight found standing in for "whichever one the
    // scene had" is the same convention SettingsPanel's own Shadows section
    // already assumes when it says "Sun shadows" for a scene with exactly
    // one directional light, the overwhelming majority of scenes.
    if (!out.sunLight())
        for (Light* light : out.lights())
            if (light->lightType() == LightType::Directional)
            {
                out.setSunLight(static_cast<DirectionalLight*>(light));
                break;
            }

    const auto nameField = sceneField->find("name");
    if (nameField != sceneField->end() && nameField->is_string())
        out.root().setName(nameField->get<std::string>());
    const auto cullingField = sceneField->find("culling");
    if (cullingField != sceneField->end() && cullingField->is_object())
    {
        out.setStaticCullingEnabled(cullingField->value("static", out.staticCullingEnabled()));
        out.setDynamicCullingEnabled(cullingField->value("dynamic", out.dynamicCullingEnabled()));
        out.setOcclusionQueryEnabled(cullingField->value("occlusion", out.occlusionQueryEnabled()));
    }

    // Active camera is resolved last, after readComponentsPass() above has
    // had a chance to attach a Camera - and never fatal: a scene with no
    // camera set is valid.
    const auto activeCameraField = sceneField->find("activeCamera");
    if (activeCameraField != sceneField->end() && !activeCameraField->is_null())
    {
        u64 id = 0;
        if (!readNonNegativeInteger(*activeCameraField, id) || id == 0)
        {
            result.addWarning("scene.activeCamera", "not an integer id, ignored");
        }
        else
        {
            GameObject* object = out.findGameObject(id);
            Camera* camera = object ? object->findComponent<Camera>() : nullptr;
            if (camera)
                out.setActiveCamera(camera);
            else
                result.addWarning("scene.activeCamera",
                                  "object " + std::to_string(id) + " has no Camera component");
        }
    }

    // Best-effort, never a load error: a missing/malformed field just keeps
    // whatever Engine already had (its own constructor defaults, most of the
    // time), the same way a Play/Stop snapshot or an older scene file with no
    // "renderSettings" section at all works today.
    if (settings)
    {
        const auto renderSettingsField = root.find("renderSettings");
        if (renderSettingsField != root.end() && renderSettingsField->is_object())
        {
            if (settings->shadows)
            {
                const auto field = renderSettingsField->find("cascadeShadows");
                if (field != renderSettingsField->end() && field->is_object())
                    readCascadeShadowSettings(*field, *settings->shadows);
            }
            if (settings->shadowAtlas)
            {
                const auto field = renderSettingsField->find("shadowAtlas");
                if (field != renderSettingsField->end() && field->is_object())
                    readShadowAtlasSettings(*field, *settings->shadowAtlas);
            }
            if (settings->postProcess)
            {
                const auto field = renderSettingsField->find("postProcess");
                if (field != renderSettingsField->end() && field->is_object())
                    readPostProcessSettings(*field, *settings->postProcess);
            }
            if (settings->renderResolution)
            {
                const auto field = renderSettingsField->find("renderResolution");
                if (field != renderSettingsField->end() && field->is_object())
                {
                    settings->renderResolution->width =
                        field->value("width", settings->renderResolution->width);
                    settings->renderResolution->height =
                        field->value("height", settings->renderResolution->height);
                    settings->renderResolution->scale =
                        field->value("scale", settings->renderResolution->scale);
                }
            }
            if (settings->lensFlare)
            {
                const auto field = renderSettingsField->find("lensFlare");
                if (field != renderSettingsField->end() && field->is_object())
                    readLensFlareSettings(*field, *settings->lensFlare);
            }
            if (settings->environmentProbe)
            {
                const auto field = renderSettingsField->find("environmentProbe");
                if (field != renderSettingsField->end() && field->is_object())
                    readEnvironmentProbeSettings(*field, *settings->environmentProbe);
            }
            if (settings->lighting)
            {
                const auto field = renderSettingsField->find("lighting");
                if (field != renderSettingsField->end() && field->is_object())
                    readLightingSettings(*field, *settings->lighting);
            }
            if (settings->volumetric)
            {
                const auto field = renderSettingsField->find("volumetric");
                if (field != renderSettingsField->end() && field->is_object())
                    readVolumetricSettings(*field, *settings->volumetric);
            }
            if (settings->sky)
            {
                const auto field = renderSettingsField->find("sky");
                if (field != renderSettingsField->end() && field->is_object())
                    readSkySettings(*field, *settings->sky);
            }
            if (settings->particles)
            {
                const auto field = renderSettingsField->find("particles");
                if (field != renderSettingsField->end() && field->is_object())
                {
                    const auto texture = field->find("texture");
                    if (texture != field->end() && texture->is_string())
                        settings->particles->setTextureFile(texture->get<std::string>());
                }
            }
        }
    }

    return result.success();
}

GameObject* SceneSerializer::cloneObject(GameObject& source, Scene& out, GameObject* newParent,
                                         SceneLoadResult& result) const
{
    std::vector<GameObject*> objects;
    objects.push_back(&source);
    collectPreOrder(source, objects);

    // Written exactly like toJson() writes any other object - each entry's
    // "parent" is whatever GameObject::parent() really is, which for every
    // one of these except `source` itself already lands inside this same
    // array (a descendant's parent is either `source` or another
    // descendant). `source`'s own real parent lies outside the subtree (or
    // is Scene's root, which nothing here was ever going to reference
    // anyway) - forced to null right after so buildCreationOrder() never
    // has to resolve an id it was never given. reparent() below is what
    // actually places the clone; this only has to parse successfully.
    nlohmann::json objectsJson = nlohmann::json::array();
    for (GameObject* object : objects)
        objectsJson.push_back(writeObject(*object, out.root()));
    objectsJson[0]["parent"] = nullptr;

    // Fresh ids throughout, minted up front from the same counter
    // createGameObject() itself draws from - creating these into `out` (very
    // often the exact Scene `source` already lives in) must never collide
    // with the originals sitting right next to them. One remap pass, applied
    // to both "id" and every "parent" that points within this same set.
    HashMap<u64, u64> remap;
    for (nlohmann::json& entry : objectsJson)
    {
        const u64 oldId = entry.value("id", u64(0));
        const u64 newId = out.reserveId();
        remap[oldId] = newId;
        entry["id"] = newId;
    }
    for (nlohmann::json& entry : objectsJson)
        if (!entry["parent"].is_null())
        {
            const auto found = remap.find(entry["parent"].get<u64>());
            entry["parent"] =
                found != remap.end() ? nlohmann::json(found->second) : nlohmann::json(nullptr);
        }

    nlohmann::json sceneJson;
    sceneJson["name"] = out.root().name();
    sceneJson["activeCamera"] = nullptr;
    sceneJson["objects"] = objectsJson;

    nlohmann::json document;
    document["format"] = kFormatName;
    document["version"] = kFormatVersion;
    document["scene"] = sceneJson;

    if (!fromJson(document, out, result))
        return nullptr;

    GameObject* clone = out.findGameObject(remap.at(source.id()));
    if (clone && newParent)
        out.reparent(clone, newParent);
    return clone;
}

bool SceneSerializer::save(const Scene& scene, const std::string& filename,
                           const SceneRenderSettings* settings) const
{
    const nlohmann::json document = toJson(scene, settings);
    // Plain write for now, same convention as Scene::saveCamera(); Fase 3
    // upgrades this to a tmp-file + atomic rename.
    if (!FileSystem::getSingleton().writeText(filename, document.dump(4) + '\n'))
    {
        Log::error("SceneSerializer: could not write '%s'", filename.c_str());
        return false;
    }
    return true;
}

bool SceneSerializer::load(const std::string& filename, Scene& out, SceneLoadResult& result,
                           const SceneRenderSettings* settings) const
{
    const std::string text = FileSystem::getSingleton().readText(filename);
    if (text.empty())
    {
        result.addError("", "could not read '" + filename + "'");
        return false;
    }

    nlohmann::json document;
    try
    {
        document = nlohmann::json::parse(text);
    }
    catch (const std::exception& error)
    {
        result.addError("", "'" + filename + "' is not valid JSON: " + error.what());
        return false;
    }

    return fromJson(document, out, result, settings);
}

} // namespace Radion
