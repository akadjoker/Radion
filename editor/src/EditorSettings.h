#ifndef RADION_EDITOR_SETTINGS_H
#define RADION_EDITOR_SETTINGS_H

#include "Types.h"

#include <glm/vec3.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace Radion
{

class EditorSettings
{
public:
    bool load(const std::string& filename);
    bool save(const std::string& filename) const;

    std::string lastScenePath;
    std::string lastOpenDirectory;
    std::string lastSaveDirectory;
    std::string lastProjectPath;
    std::vector<std::string> recentProjectPaths;

    int themeIndex = 0;
    int dockLayoutVersion = 0;

    // The Viewport's own free-fly camera, restored on launch instead of
    // dropping back to the hardcoded starting pose every time - a large
    // scene otherwise means re-navigating to wherever the work actually is
    // after every single reopen. ViewportPanel owns the live values; this is
    // only where the last-known pose is kept between sessions.
    Math::Vec3 cameraPosition = Math::Vec3(0.0f, 2.5f, 9.5f);
    Math::Vec3 cameraOrbitTarget = Math::Vec3(0.0f);
    f32 cameraOrbitDistance = 10.0f;
    f32 cameraOrbitYaw = 0.0f;
    f32 cameraOrbitPitch = -0.25f;
    bool cameraPerspective = true;

    // Editor Viewport camera navigation, edited from the "View > Camera
    // Settings..." popup. Unlike the pose fields above, these are owned here
    // (the popup edits them) and only consumed by ViewportPanel.
    f32 cameraMoveSpeed = 0.1f;    // WASD base fly speed while looking
    f32 cameraFastSpeed = 0.3f;    // Shift-boost fly speed while looking
    f32 cameraMinView = 0.1f;      // orbit/zoom-in limit
    f32 cameraMaxView = 100000.0f; // orbit/zoom-out limit
    f32 cameraNearPlane = 0.05f;
    f32 cameraFarPlane = 1000.0f;

    bool viewportGrid = true;
    bool viewportSnap = false;
    int viewportTool = 0;
    int viewportNavigationTool = 0;
    Math::Vec3 cursor3D = Math::Vec3(0.0f);
    int viewMode = 0;
    bool showStatsOverlay = false;
    bool showDynamicIndexDebug = false;
    bool showOcclusionDebug = false;
    bool showSubmeshBounds = false;
    bool showShadowCascades = false;
    bool showShadowAtlas = false;

    // Editor Viewport quality only. These never alter the scene or the Game
    // view; they remove expensive preview passes while authoring.
    bool previewShadows = true;
    bool previewSSAO = false;
    bool previewVolumetrics = false;
    bool previewLensFlares = false;
    bool previewPlanarReflections = false;
    bool previewOcclusionCulling = false;
    f32 previewNavigationScale = 0.5f;

    std::string assetsDirectory;
    int assetsViewMode = 0;
    f32 assetsThumbnailSize = 96.0f;
    int gameResolutionPreset = 0;
    bool consoleAutoScroll = true;
    bool consoleShowInfo = true;
    bool consoleShowWarning = true;
    bool consoleShowError = true;
    bool consoleShowDebug = false;
    bool debugPreviewEnabled = false;
    int debugPreviewView = 0;
    std::unordered_map<std::string, bool> panelOpen;
};

} // namespace Radion

#endif // RADION_EDITOR_SETTINGS_H
