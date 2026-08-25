#include "PCH.h"
#include "BlenderSettings.h"

#include "FileSystem.h"
#include "Log.h"

#include <nlohmann/json.hpp>

using namespace Radion;

namespace
{
constexpr usize kMaxRecentFiles = 10;
} // namespace

BlenderSettings::BlenderSettings()
{
}

BlenderSettings::~BlenderSettings()
{
}

void BlenderSettings::addRecentFile(const std::string& path)
{
    std::vector<std::string>& files = mGeneral.recentFiles;
    files.erase(std::remove(files.begin(), files.end(), path), files.end());
    files.insert(files.begin(), path);
    if (files.size() > kMaxRecentFiles)
        files.resize(kMaxRecentFiles);
}

void BlenderSettings::removeRecentFile(const std::string& path)
{
    std::vector<std::string>& files = mGeneral.recentFiles;
    files.erase(std::remove(files.begin(), files.end(), path), files.end());
}

void BlenderSettings::clearRecentFiles()
{
    mGeneral.recentFiles.clear();
}

namespace
{
void readVec3(const nlohmann::json& json, const char* key, glm::vec3& out)
{
    const auto field = json.find(key);
    if (field != json.end() && field->is_array() && field->size() == 3)
        out = glm::vec3((*field)[0].get<f32>(), (*field)[1].get<f32>(), (*field)[2].get<f32>());
}

void readFloat(const nlohmann::json& json, const char* key, f32& out)
{
    const auto field = json.find(key);
    if (field != json.end() && field->is_number())
        out = field->get<f32>();
}

void readBool(const nlohmann::json& json, const char* key, bool& out)
{
    const auto field = json.find(key);
    if (field != json.end() && field->is_boolean())
        out = field->get<bool>();
}

void readString(const nlohmann::json& json, const char* key, std::string& out)
{
    const auto field = json.find(key);
    if (field != json.end() && field->is_string())
        out = field->get<std::string>();
}
} // namespace

bool BlenderSettings::load(const std::string& path)
{
    const std::string text = FileSystem::getSingleton().readText(path);
    if (text.empty())
        return false;

    nlohmann::json root;
    try
    {
        root = nlohmann::json::parse(text);
    }
    catch (const std::exception& error)
    {
        Log::error("BlenderSettings: '%s' is not valid JSON: %s", path.c_str(), error.what());
        return false;
    }

    const auto viewport = root.find("viewport");
    if (viewport != root.end() && viewport->is_object())
    {
        readFloat(*viewport, "fov", mViewport.fov);
        readFloat(*viewport, "nearPlane", mViewport.nearPlane);
        readFloat(*viewport, "farPlane", mViewport.farPlane);
        readVec3(*viewport, "backgroundColor", mViewport.backgroundColor);
        readVec3(*viewport, "vertexColor", mViewport.vertexColor);
        readVec3(*viewport, "selectedVertexColor", mViewport.selectedVertexColor);
        readVec3(*viewport, "faceHighlightColor", mViewport.faceHighlightColor);
        readFloat(*viewport, "faceHighlightAlpha", mViewport.faceHighlightAlpha);
        readVec3(*viewport, "faceEdgeHighlightColor", mViewport.faceEdgeHighlightColor);
        readVec3(*viewport, "submeshHighlightColor", mViewport.submeshHighlightColor);
        readFloat(*viewport, "submeshHighlightAlpha", mViewport.submeshHighlightAlpha);
        readVec3(*viewport, "boxSelectColor", mViewport.boxSelectColor);
        readBool(*viewport, "colorBySubmesh", mViewport.colorBySubmesh);
        readFloat(*viewport, "debugVectorLength", mViewport.debugVectorLength);
        readVec3(*viewport, "normalVectorColor", mViewport.normalVectorColor);
        readVec3(*viewport, "tangentVectorColor", mViewport.tangentVectorColor);
    }

    const auto animation = root.find("animation");
    if (animation != root.end() && animation->is_object())
    {
        readFloat(*animation, "playbackSpeed", mAnimation.playbackSpeed);
        readBool(*animation, "autoLoop", mAnimation.autoLoop);
    }

    const auto general = root.find("general");
    if (general != root.end() && general->is_object())
    {
        readString(*general, "lastOpenedMesh", mGeneral.lastOpenedMesh);
        readString(*general, "lastOpenDirectory", mGeneral.lastOpenDirectory);
        readBool(*general, "showNormals", mGeneral.showNormals);
        readBool(*general, "showWireframe", mGeneral.showWireframe);
        readFloat(*general, "vertexPointSize", mGeneral.vertexPointSize);
        const auto theme = general->find("themeIndex");
        if (theme != general->end() && theme->is_number_integer())
            mGeneral.themeIndex = theme->get<int>();

        const auto recent = general->find("recentFiles");
        if (recent != general->end() && recent->is_array())
        {
            mGeneral.recentFiles.clear();
            for (const auto& entry : *recent)
            {
                if (entry.is_string())
                    mGeneral.recentFiles.push_back(entry.get<std::string>());
            }
            if (mGeneral.recentFiles.size() > kMaxRecentFiles)
                mGeneral.recentFiles.resize(kMaxRecentFiles);
        }
    }

    return true;
}

bool BlenderSettings::save(const std::string& path)
{
    nlohmann::json root;

    nlohmann::json viewport;
    viewport["fov"] = mViewport.fov;
    viewport["nearPlane"] = mViewport.nearPlane;
    viewport["farPlane"] = mViewport.farPlane;
    viewport["backgroundColor"] = {mViewport.backgroundColor.x, mViewport.backgroundColor.y,
                                   mViewport.backgroundColor.z};
    viewport["vertexColor"] = {mViewport.vertexColor.x, mViewport.vertexColor.y, mViewport.vertexColor.z};
    viewport["selectedVertexColor"] = {mViewport.selectedVertexColor.x, mViewport.selectedVertexColor.y,
                                       mViewport.selectedVertexColor.z};
    viewport["faceHighlightColor"] = {mViewport.faceHighlightColor.x, mViewport.faceHighlightColor.y,
                                      mViewport.faceHighlightColor.z};
    viewport["faceHighlightAlpha"] = mViewport.faceHighlightAlpha;
    viewport["faceEdgeHighlightColor"] = {mViewport.faceEdgeHighlightColor.x,
                                          mViewport.faceEdgeHighlightColor.y,
                                          mViewport.faceEdgeHighlightColor.z};
    viewport["submeshHighlightColor"] = {mViewport.submeshHighlightColor.x,
                                         mViewport.submeshHighlightColor.y,
                                         mViewport.submeshHighlightColor.z};
    viewport["submeshHighlightAlpha"] = mViewport.submeshHighlightAlpha;
    viewport["boxSelectColor"] = {mViewport.boxSelectColor.x, mViewport.boxSelectColor.y,
                                  mViewport.boxSelectColor.z};
    viewport["colorBySubmesh"] = mViewport.colorBySubmesh;
    viewport["debugVectorLength"] = mViewport.debugVectorLength;
    viewport["normalVectorColor"] = {mViewport.normalVectorColor.x, mViewport.normalVectorColor.y,
                                     mViewport.normalVectorColor.z};
    viewport["tangentVectorColor"] = {mViewport.tangentVectorColor.x, mViewport.tangentVectorColor.y,
                                      mViewport.tangentVectorColor.z};
    root["viewport"] = viewport;

    nlohmann::json animation;
    animation["playbackSpeed"] = mAnimation.playbackSpeed;
    animation["autoLoop"] = mAnimation.autoLoop;
    root["animation"] = animation;

    nlohmann::json general;
    general["lastOpenedMesh"] = mGeneral.lastOpenedMesh;
    general["lastOpenDirectory"] = mGeneral.lastOpenDirectory;
    general["showNormals"] = mGeneral.showNormals;
    general["showWireframe"] = mGeneral.showWireframe;
    general["vertexPointSize"] = mGeneral.vertexPointSize;
    general["themeIndex"] = mGeneral.themeIndex;
    general["recentFiles"] = mGeneral.recentFiles;
    root["general"] = general;

    if (!FileSystem::getSingleton().writeText(path, root.dump(4) + '\n'))
    {
        Log::error("BlenderSettings: could not write '%s'", path.c_str());
        return false;
    }
    return true;
}
