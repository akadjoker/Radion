#include "PCH.h"

#include "EngineSettings.h"

#include "Engine.h"
#include "FileSystem.h"
#include "Lighting.h"
#include "Log.h"
#include "PostProcess.h"
#include "Shadows.h"
#include "Sky.h"

#include <nlohmann/json.hpp>

namespace Radion
{

namespace
{

using Json = nlohmann::json;

// Reads `key` into `value` when the file has it, leaves it alone when it does
// not. Every field goes through one of these, which is what makes a file
// written by an older build still load.
template <typename T>
void read(const Json& node, const char* key, T& value)
{
    const auto found = node.find(key);
    if (found != node.end() && !found->is_null())
        value = found->get<T>();
}

void readVec3(const Json& node, const char* key, glm::vec3& value)
{
    const auto found = node.find(key);
    if (found == node.end() || !found->is_array() || found->size() != 3)
        return;
    value = glm::vec3((*found)[0].get<f32>(), (*found)[1].get<f32>(), (*found)[2].get<f32>());
}

Json writeVec3(const glm::vec3& value)
{
    return Json::array({value.x, value.y, value.z});
}

void readCascades(const Json& node, CascadeShadowSettings& settings)
{
    read(node, "count", settings.count);
    read(node, "resolution", settings.resolution);
    read(node, "distance", settings.distance);
    read(node, "quality", settings.quality);
    read(node, "bias", settings.bias);
    read(node, "normalBias", settings.normalBias);
    read(node, "pancakeSize", settings.pancakeSize);
    read(node, "blur", settings.blur);
    read(node, "fadeStart", settings.fadeStart);
    read(node, "opacity", settings.opacity);
    read(node, "angularDiameter", settings.angularDiameter);
    read(node, "blend", settings.blend);
    read(node, "enabled", settings.enabled);
}

Json writeCascades(const CascadeShadowSettings& settings)
{
    Json node;
    node["count"] = settings.count;
    node["resolution"] = settings.resolution;
    node["distance"] = settings.distance;
    node["quality"] = settings.quality;
    node["bias"] = settings.bias;
    node["normalBias"] = settings.normalBias;
    node["pancakeSize"] = settings.pancakeSize;
    node["blur"] = settings.blur;
    node["fadeStart"] = settings.fadeStart;
    node["opacity"] = settings.opacity;
    node["angularDiameter"] = settings.angularDiameter;
    node["blend"] = settings.blend;
    node["enabled"] = settings.enabled;
    return node;
}

void readAtlas(const Json& node, ShadowAtlasSettings& settings)
{
    read(node, "size", settings.size);
    read(node, "maximumTileSize", settings.maximumTileSize);
    read(node, "minimumTileSize", settings.minimumTileSize);
    read(node, "volumetricPriority", settings.volumetricPriority);
    read(node, "pointPriority", settings.pointPriority);
    read(node, "pointBias", settings.pointBias);
    read(node, "biasSlope", settings.biasSlope);
    read(node, "biasConstant", settings.biasConstant);
    read(node, "point", settings.point);
    read(node, "spot", settings.spot);
}

Json writeAtlas(const ShadowAtlasSettings& settings)
{
    Json node;
    node["size"] = settings.size;
    node["maximumTileSize"] = settings.maximumTileSize;
    node["minimumTileSize"] = settings.minimumTileSize;
    node["volumetricPriority"] = settings.volumetricPriority;
    node["pointPriority"] = settings.pointPriority;
    node["pointBias"] = settings.pointBias;
    node["biasSlope"] = settings.biasSlope;
    node["biasConstant"] = settings.biasConstant;
    node["point"] = settings.point;
    node["spot"] = settings.spot;
    return node;
}

void readPost(const Json& node, PostProcessStack& post)
{
    read(node, "enabled", post.enabled);
    read(node, "exposure", post.exposure);
    u8 toneMap = static_cast<u8>(post.toneMap);
    read(node, "toneMap", toneMap);
    post.toneMap = static_cast<ToneMapMode>(toneMap);
    read(node, "bloomThreshold", post.bloomThreshold);
    read(node, "bloomSoftKnee", post.bloomSoftKnee);
    read(node, "bloomStrength", post.bloomStrength);
    read(node, "fxaaSubpixel", post.fxaaSubpixel);
    read(node, "fxaaEdgeThreshold", post.fxaaEdgeThreshold);
    read(node, "fxaaEdgeThresholdMin", post.fxaaEdgeThresholdMin);
    read(node, "ssaoEnabled", post.ssaoEnabled);
    read(node, "ssaoBlur", post.ssaoBlur);
    read(node, "ssaoRadius", post.ssaoRadius);
    read(node, "ssaoBias", post.ssaoBias);
    read(node, "ssaoIntensity", post.ssaoIntensity);
    read(node, "ssaoSamples", post.ssaoSamples);
    read(node, "ssaoDepthSigma", post.ssaoDepthSigma);
    read(node, "ssaoDebug", post.ssaoDebug);
}

Json writePost(const PostProcessStack& post)
{
    Json node;
    node["enabled"] = post.enabled;
    node["exposure"] = post.exposure;
    node["toneMap"] = static_cast<u8>(post.toneMap);
    node["bloomThreshold"] = post.bloomThreshold;
    node["bloomSoftKnee"] = post.bloomSoftKnee;
    node["bloomStrength"] = post.bloomStrength;
    node["fxaaSubpixel"] = post.fxaaSubpixel;
    node["fxaaEdgeThreshold"] = post.fxaaEdgeThreshold;
    node["fxaaEdgeThresholdMin"] = post.fxaaEdgeThresholdMin;
    node["ssaoEnabled"] = post.ssaoEnabled;
    node["ssaoBlur"] = post.ssaoBlur;
    node["ssaoRadius"] = post.ssaoRadius;
    node["ssaoBias"] = post.ssaoBias;
    node["ssaoIntensity"] = post.ssaoIntensity;
    node["ssaoSamples"] = post.ssaoSamples;
    node["ssaoDepthSigma"] = post.ssaoDepthSigma;
    node["ssaoDebug"] = post.ssaoDebug;
    return node;
}

void readSky(const Json& node, SkySettings& sky)
{
    read(node, "enabled", sky.enabled);
    u8 mode = static_cast<u8>(sky.mode);
    read(node, "mode", mode);
    sky.mode = static_cast<SkyMode>(mode);
    read(node, "automaticSun", sky.automaticSun);
    read(node, "sunFromSky", sky.sunFromSky);
    read(node, "timeOfDay", sky.timeOfDay);
    read(node, "sunAzimuth", sky.sunAzimuth);
    read(node, "sunElevation", sky.sunElevation);
    read(node, "northOffset", sky.northOffset);
    read(node, "maximumElevation", sky.maximumElevation);
    readVec3(node, "ambient", sky.ambient);
    read(node, "ambientStrength", sky.ambientStrength);
    read(node, "lightmapIntensity", sky.lightmapIntensity);
    read(node, "lightmapShadowLift", sky.lightmapShadowLift);
    read(node, "intensity", sky.intensity);
    read(node, "sunIntensity", sky.sunIntensity);
    read(node, "rayleigh", sky.rayleigh);
    read(node, "mie", sky.mie);
    read(node, "mieG", sky.mieG);
    read(node, "atmosphereExposure", sky.atmosphereExposure);
    read(node, "viewSteps", sky.viewSteps);
    read(node, "lightSteps", sky.lightSteps);
    read(node, "cloudsEnabled", sky.cloudsEnabled);
    read(node, "cloudHeight", sky.cloudHeight);
    read(node, "cloudScale", sky.cloudScale);
    read(node, "cloudCoverage", sky.cloudCoverage);
    read(node, "cloudDensity", sky.cloudDensity);
    read(node, "cloudSpeed", sky.cloudSpeed);
    read(node, "cloudDirectionX", sky.cloudDirection.x);
    read(node, "cloudDirectionY", sky.cloudDirection.y);
    readVec3(node, "cloudColor", sky.cloudColor);
}

Json writeSky(const SkySettings& sky)
{
    Json node;
    node["enabled"] = sky.enabled;
    node["mode"] = static_cast<u8>(sky.mode);
    node["automaticSun"] = sky.automaticSun;
    node["sunFromSky"] = sky.sunFromSky;
    node["timeOfDay"] = sky.timeOfDay;
    node["sunAzimuth"] = sky.sunAzimuth;
    node["sunElevation"] = sky.sunElevation;
    node["northOffset"] = sky.northOffset;
    node["maximumElevation"] = sky.maximumElevation;
    node["ambient"] = writeVec3(sky.ambient);
    node["ambientStrength"] = sky.ambientStrength;
    node["lightmapIntensity"] = sky.lightmapIntensity;
    node["lightmapShadowLift"] = sky.lightmapShadowLift;
    node["intensity"] = sky.intensity;
    node["sunIntensity"] = sky.sunIntensity;
    node["rayleigh"] = sky.rayleigh;
    node["mie"] = sky.mie;
    node["mieG"] = sky.mieG;
    node["atmosphereExposure"] = sky.atmosphereExposure;
    node["viewSteps"] = sky.viewSteps;
    node["lightSteps"] = sky.lightSteps;
    node["cubemapName"] = sky.cubemapName;
    node["cloudsEnabled"] = sky.cloudsEnabled;
    node["cloudHeight"] = sky.cloudHeight;
    node["cloudScale"] = sky.cloudScale;
    node["cloudCoverage"] = sky.cloudCoverage;
    node["cloudDensity"] = sky.cloudDensity;
    node["cloudSpeed"] = sky.cloudSpeed;
    node["cloudDirectionX"] = sky.cloudDirection.x;
    node["cloudDirectionY"] = sky.cloudDirection.y;
    node["cloudColor"] = writeVec3(sky.cloudColor);
    return node;
}

} // namespace

bool EngineSettings::load(Engine& engine, const std::string& filename)
{
    // Registered even when the file is missing or malformed: a first run has
    // nothing to load yet, but still gets its window position/size saved on
    // the way out, see Engine::shutdown().
    engine.setSettingsFile(filename);

    const std::string text = FileSystem::getSingleton().readText(filename);
    if (text.empty())
        return false;

    Json root;
    try
    {
        root = Json::parse(text);
    }
    catch (const std::exception& error)
    {
        Log::error("EngineSettings: '%s' is not valid JSON: %s", filename.c_str(), error.what());
        return false;
    }

    if (const auto node = root.find("window"); node != root.end())
    {
        Platform::Window& window = engine.getWindow();
        int x = 0, y = 0;
        window.getPosition(x, y);
        int width = window.getWidth();
        int height = window.getHeight();
        read(*node, "x", x);
        read(*node, "y", y);
        read(*node, "width", width);
        read(*node, "height", height);
        // Restore the monitor from the saved top-left position before
        // applying the size. A window created on monitor 0 can otherwise be
        // clamped to that monitor's work area even when the saved layout
        // belongs to a larger secondary display.
        const int displayCount = SDL_GetNumVideoDisplays();
        for (int display = 0; display < displayCount; ++display)
        {
            SDL_Rect bounds;
            if (SDL_GetDisplayBounds(display, &bounds) != 0)
                continue;
            if (x >= bounds.x && x < bounds.x + bounds.w && y >= bounds.y &&
                y < bounds.y + bounds.h)
            {
                window.setMonitor(display);
                break;
            }
        }
        // Move first, then resize. If the saved layout belongs to a larger
        // monitor, resizing while the newly-created window is still on the
        // default monitor makes the window manager clamp it permanently to
        // that monitor's work area.
        window.setPosition(x, y);
        window.setSize(width, height);
    }
    if (const auto node = root.find("render"); node != root.end())
    {
        RenderResolution resolution = engine.renderResolution();
        read(*node, "width", resolution.width);
        read(*node, "height", resolution.height);
        read(*node, "scale", resolution.scale);
        engine.setRenderResolution(resolution);
    }
    if (const auto node = root.find("cascades"); node != root.end())
        if (CascadeShadowSettings* settings = engine.cascadeSettings())
            readCascades(*node, *settings);
    if (Lighting* lighting = engine.lighting())
    {
        if (const auto node = root.find("atlas"); node != root.end())
            readAtlas(*node, lighting->atlasSettings());
        if (const auto node = root.find("lighting"); node != root.end())
        {
            read(*node, "tiled", lighting->tiled);
            read(*node, "use25D", lighting->use25D);
        }
    }
    if (const auto node = root.find("debug"); node != root.end())
    {
        read(*node, "showShadowCascades", engine.debugShowShadowCascades);
        read(*node, "showShadowAtlas", engine.debugShowShadowAtlas);
        read(*node, "showPhysicsShapes", engine.debugShowPhysicsShapes);
        read(*node, "showPhysicsContacts", engine.debugShowPhysicsContacts);
        read(*node, "showPhysicsJoints", engine.debugShowPhysicsJoints);
        read(*node, "showAIObstacles", engine.debugShowAIObstacles);
        if (Lighting* lighting = engine.lighting())
            read(*node, "lightingTiles", lighting->debugTiles);
    }
    if (const auto node = root.find("post"); node != root.end())
        readPost(*node, engine.postProcess());
    if (const auto node = root.find("sky"); node != root.end())
    {
        readSky(*node, engine.sky());
        // Not part of readSky(): restoring the texture needs Engine, not just
        // SkySettings, and has to run after the plain fields above so a mode
        // saved as Cubemap does not get clobbered back to Gradient by them.
        std::string cubemapName;
        read(*node, "cubemapName", cubemapName);
        if (!cubemapName.empty())
            loadSkyCubemap(engine.sky(), cubemapName);
    }

    Log::info("EngineSettings: loaded '%s'", filename.c_str());
    return true;
}

bool EngineSettings::save(const Engine& engine, const std::string& filename)
{
    // const_cast because everything this reaches only offers non-const
    // accessors; nothing here writes through them.
    Engine& mutableEngine = const_cast<Engine&>(engine);

    Json root;
    {
        Platform::Window& window = mutableEngine.getWindow();
        int x = 0, y = 0;
        window.getPosition(x, y);
        root["window"]["x"] = x;
        root["window"]["y"] = y;
        root["window"]["width"] = window.getWidth();
        root["window"]["height"] = window.getHeight();
    }

    const RenderResolution& resolution = engine.renderResolution();
    root["render"]["width"] = resolution.width;
    root["render"]["height"] = resolution.height;
    root["render"]["scale"] = resolution.scale;

    if (const CascadeShadowSettings* settings = mutableEngine.cascadeSettings())
        root["cascades"] = writeCascades(*settings);
    if (Lighting* lighting = mutableEngine.lighting())
    {
        root["atlas"] = writeAtlas(lighting->atlasSettings());
        root["lighting"]["tiled"] = lighting->tiled;
        root["lighting"]["use25D"] = lighting->use25D;
        root["debug"]["lightingTiles"] = lighting->debugTiles;
    }
    root["debug"]["showShadowCascades"] = engine.debugShowShadowCascades;
    root["debug"]["showShadowAtlas"] = engine.debugShowShadowAtlas;
    root["debug"]["showPhysicsShapes"] = engine.debugShowPhysicsShapes;
    root["debug"]["showPhysicsContacts"] = engine.debugShowPhysicsContacts;
    root["debug"]["showPhysicsJoints"] = engine.debugShowPhysicsJoints;
    root["debug"]["showAIObstacles"] = engine.debugShowAIObstacles;
    root["post"] = writePost(mutableEngine.postProcess());
    root["sky"] = writeSky(mutableEngine.sky());

    if (!FileSystem::getSingleton().writeText(filename, root.dump(4) + '\n'))
    {
        Log::error("EngineSettings: could not write '%s'", filename.c_str());
        return false;
    }
    Log::info("EngineSettings: saved '%s'", filename.c_str());
    return true;
}

} // namespace Radion
