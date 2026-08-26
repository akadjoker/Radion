#include "PCH.h"

#include "EditorSettings.h"

#include "FileSystem.h"

#include <nlohmann/json.hpp>

namespace Radion
{

bool EditorSettings::load(const std::string& filename)
{
    const std::string text = FileSystem::getSingleton().readText(filename);
    if (text.empty())
        return false;

    nlohmann::json root;
    try
    {
        root = nlohmann::json::parse(text);
    }
    catch (const std::exception& error)
    {
        Log::error("EditorSettings: '%s' is not valid JSON: %s", filename.c_str(), error.what());
        return false;
    }

    const auto lastScene = root.find("lastScenePath");
    if (lastScene != root.end() && lastScene->is_string())
        lastScenePath = lastScene->get<std::string>();
    const auto lastOpen = root.find("lastOpenDirectory");
    if (lastOpen != root.end() && lastOpen->is_string())
        lastOpenDirectory = lastOpen->get<std::string>();
    const auto lastSave = root.find("lastSaveDirectory");
    if (lastSave != root.end() && lastSave->is_string())
        lastSaveDirectory = lastSave->get<std::string>();
    const auto lastProject = root.find("lastProjectPath");
    if (lastProject != root.end() && lastProject->is_string())
        lastProjectPath = lastProject->get<std::string>();
    const auto recentProjects = root.find("recentProjectPaths");
    if (recentProjects != root.end() && recentProjects->is_array())
        for (const auto& path : *recentProjects)
            if (path.is_string() && !path.get<std::string>().empty() &&
                std::find(recentProjectPaths.begin(), recentProjectPaths.end(),
                          path.get<std::string>()) == recentProjectPaths.end() &&
                recentProjectPaths.size() < 12)
                recentProjectPaths.push_back(path.get<std::string>());
    const auto theme = root.find("themeIndex");
    if (theme != root.end() && theme->is_number_integer())
        themeIndex = theme->get<int>();
    const auto dockLayout = root.find("dockLayoutVersion");
    if (dockLayout != root.end() && dockLayout->is_number_integer())
        dockLayoutVersion = dockLayout->get<int>();

    const auto readVec3 = [](const nlohmann::json& json, const char* key, Math::Vec3& out)
    {
        const auto field = json.find(key);
        if (field != json.end() && field->is_array() && field->size() == 3)
            out = Math::Vec3((*field)[0].get<f32>(), (*field)[1].get<f32>(), (*field)[2].get<f32>());
    };
    const auto readFloat = [](const nlohmann::json& json, const char* key, f32& out)
    {
        const auto field = json.find(key);
        if (field != json.end() && field->is_number())
            out = field->get<f32>();
    };
    readVec3(root, "cameraPosition", cameraPosition);
    readVec3(root, "cameraOrbitTarget", cameraOrbitTarget);
    readFloat(root, "cameraOrbitDistance", cameraOrbitDistance);
    readFloat(root, "cameraOrbitYaw", cameraOrbitYaw);
    readFloat(root, "cameraOrbitPitch", cameraOrbitPitch);
    readFloat(root, "cameraMoveSpeed", cameraMoveSpeed);
    readFloat(root, "cameraFastSpeed", cameraFastSpeed);
    readFloat(root, "cameraMinView", cameraMinView);
    readFloat(root, "cameraMaxView", cameraMaxView);
    readFloat(root, "cameraNearPlane", cameraNearPlane);
    readFloat(root, "cameraFarPlane", cameraFarPlane);
    cameraMinView = glm::clamp(cameraMinView, 0.001f, 10000.0f);
    cameraMaxView = glm::max(cameraMaxView, cameraMinView);
    cameraNearPlane = glm::clamp(cameraNearPlane, 0.001f, 10000.0f);
    cameraFarPlane = glm::max(cameraFarPlane, cameraNearPlane + 0.001f);
    const auto perspective = root.find("cameraPerspective");
    if (perspective != root.end() && perspective->is_boolean())
        cameraPerspective = perspective->get<bool>();
    const auto readBool = [](const nlohmann::json& json, const char* key, bool& out)
    {
        const auto field = json.find(key);
        if (field != json.end() && field->is_boolean())
            out = field->get<bool>();
    };
    const auto readInt = [](const nlohmann::json& json, const char* key, int& out)
    {
        const auto field = json.find(key);
        if (field != json.end() && field->is_number_integer())
            out = field->get<int>();
    };
    readBool(root, "viewportGrid", viewportGrid);
    readBool(root, "viewportSnap", viewportSnap);
    readInt(root, "viewportTool", viewportTool);
    readInt(root, "viewportNavigationTool", viewportNavigationTool);
    readVec3(root, "cursor3D", cursor3D);
    readInt(root, "viewMode", viewMode);
    readBool(root, "showStatsOverlay", showStatsOverlay);
    readBool(root, "showDynamicIndexDebug", showDynamicIndexDebug);
    readBool(root, "showOcclusionDebug", showOcclusionDebug);
    readBool(root, "showSubmeshBounds", showSubmeshBounds);
    readBool(root, "showShadowCascades", showShadowCascades);
    readBool(root, "showShadowAtlas", showShadowAtlas);
    readBool(root, "previewShadows", previewShadows);
    readBool(root, "previewSSAO", previewSSAO);
    readBool(root, "previewVolumetrics", previewVolumetrics);
    readBool(root, "previewLensFlares", previewLensFlares);
    readBool(root, "previewPlanarReflections", previewPlanarReflections);
    readBool(root, "previewOcclusionCulling", previewOcclusionCulling);
    readFloat(root, "previewNavigationScale", previewNavigationScale);
    previewNavigationScale = glm::clamp(previewNavigationScale, 0.25f, 1.0f);
    const auto assetsDirectoryField = root.find("assetsDirectory");
    if (assetsDirectoryField != root.end() && assetsDirectoryField->is_string())
        assetsDirectory = assetsDirectoryField->get<std::string>();
    readInt(root, "assetsViewMode", assetsViewMode);
    readFloat(root, "assetsThumbnailSize", assetsThumbnailSize);
    readInt(root, "gameResolutionPreset", gameResolutionPreset);
    readBool(root, "consoleAutoScroll", consoleAutoScroll);
    readBool(root, "consoleShowInfo", consoleShowInfo);
    readBool(root, "consoleShowWarning", consoleShowWarning);
    readBool(root, "consoleShowError", consoleShowError);
    readBool(root, "consoleShowDebug", consoleShowDebug);
    readBool(root, "debugPreviewEnabled", debugPreviewEnabled);
    readInt(root, "debugPreviewView", debugPreviewView);
    const auto panels = root.find("panelOpen");
    if (panels != root.end() && panels->is_object())
        for (auto entry = panels->begin(); entry != panels->end(); ++entry)
            if (entry.value().is_boolean())
                panelOpen[entry.key()] = entry.value().get<bool>();
    return true;
}

bool EditorSettings::save(const std::string& filename) const
{
    nlohmann::json root;
    root["lastScenePath"] = lastScenePath;
    root["lastOpenDirectory"] = lastOpenDirectory;
    root["lastSaveDirectory"] = lastSaveDirectory;
    root["lastProjectPath"] = lastProjectPath;
    root["recentProjectPaths"] = recentProjectPaths;
    root["themeIndex"] = themeIndex;
    root["dockLayoutVersion"] = dockLayoutVersion;
    root["cameraPosition"] = {cameraPosition.x, cameraPosition.y, cameraPosition.z};
    root["cameraOrbitTarget"] = {cameraOrbitTarget.x, cameraOrbitTarget.y, cameraOrbitTarget.z};
    root["cameraOrbitDistance"] = cameraOrbitDistance;
    root["cameraOrbitYaw"] = cameraOrbitYaw;
    root["cameraOrbitPitch"] = cameraOrbitPitch;
    root["cameraMoveSpeed"] = cameraMoveSpeed;
    root["cameraFastSpeed"] = cameraFastSpeed;
    root["cameraMinView"] = cameraMinView;
    root["cameraMaxView"] = cameraMaxView;
    root["cameraNearPlane"] = cameraNearPlane;
    root["cameraFarPlane"] = cameraFarPlane;
    root["cameraPerspective"] = cameraPerspective;
    root["viewportGrid"] = viewportGrid;
    root["viewportSnap"] = viewportSnap;
    root["viewportTool"] = viewportTool;
    root["viewportNavigationTool"] = viewportNavigationTool;
    root["cursor3D"] = {cursor3D.x, cursor3D.y, cursor3D.z};
    root["viewMode"] = viewMode;
    root["showStatsOverlay"] = showStatsOverlay;
    root["showDynamicIndexDebug"] = showDynamicIndexDebug;
    root["showOcclusionDebug"] = showOcclusionDebug;
    root["showSubmeshBounds"] = showSubmeshBounds;
    root["showShadowCascades"] = showShadowCascades;
    root["showShadowAtlas"] = showShadowAtlas;
    root["previewShadows"] = previewShadows;
    root["previewSSAO"] = previewSSAO;
    root["previewVolumetrics"] = previewVolumetrics;
    root["previewLensFlares"] = previewLensFlares;
    root["previewPlanarReflections"] = previewPlanarReflections;
    root["previewOcclusionCulling"] = previewOcclusionCulling;
    root["previewNavigationScale"] = previewNavigationScale;
    root["assetsDirectory"] = assetsDirectory;
    root["assetsViewMode"] = assetsViewMode;
    root["assetsThumbnailSize"] = assetsThumbnailSize;
    root["gameResolutionPreset"] = gameResolutionPreset;
    root["consoleAutoScroll"] = consoleAutoScroll;
    root["consoleShowInfo"] = consoleShowInfo;
    root["consoleShowWarning"] = consoleShowWarning;
    root["consoleShowError"] = consoleShowError;
    root["consoleShowDebug"] = consoleShowDebug;
    root["debugPreviewEnabled"] = debugPreviewEnabled;
    root["debugPreviewView"] = debugPreviewView;
    root["panelOpen"] = panelOpen;
    if (!FileSystem::getSingleton().writeText(filename, root.dump(4) + '\n'))
    {
        Log::error("EditorSettings: could not write '%s'", filename.c_str());
        return false;
    }
    return true;
}

} // namespace Radion
