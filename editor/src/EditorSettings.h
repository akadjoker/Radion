#ifndef RADION_EDITOR_SETTINGS_H
#define RADION_EDITOR_SETTINGS_H

#include "Types.h"

#include "Math.h"
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

    // Viewport camera pose restored on launch; ViewportPanel owns the live values.
    Math::vec3 cameraPosition = Math::vec3(0.0f, 2.5f, 9.5f);
    Math::vec3 cameraOrbitTarget = Math::vec3(0.0f);
    f32 cameraOrbitDistance = 10.0f;
    f32 cameraOrbitYaw = 0.0f;
    f32 cameraOrbitPitch = -0.25f;
    bool cameraPerspective = true;

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
    Math::vec3 cursor3D = Math::vec3(0.0f);
    int viewMode = 0;
    bool showDynamicIndexDebug = false;
    bool showOcclusionDebug = false;
    bool showSubmeshBounds = false;
    bool showShadowCascades = false;
    bool showShadowAtlas = false;
    bool showPhysicsShapes = false;
    bool showPhysicsContacts = false;
    bool showPhysicsJoints = false;
    bool showAIObstacles = false;

    // Viewport-only; never alters the scene or the Game view.
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
