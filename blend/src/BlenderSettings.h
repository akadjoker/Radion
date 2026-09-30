#ifndef RADION_BLENDER_SETTINGS_H
#define RADION_BLENDER_SETTINGS_H

#include "Math.h"
#include <string>
#include <vector>

namespace Radion
{

class BlenderSettings
{
public:
    BlenderSettings();
    ~BlenderSettings();

    struct ViewportSettings
    {
        f32 fov = 60.0f;
        f32 nearPlane = 0.1f;
        f32 farPlane = 1000.0f;
        Math::vec3 backgroundColor = Math::vec3(0.12f, 0.12f, 0.12f);

        Math::vec3 vertexColor = Math::vec3(0.9f, 0.45f, 0.05f);
        Math::vec3 selectedVertexColor = Math::vec3(1.0f, 0.9f, 0.2f);
        Math::vec3 faceHighlightColor = Math::vec3(1.0f, 0.6f, 0.0f);
        f32 faceHighlightAlpha = 0.35f;
        Math::vec3 faceEdgeHighlightColor = Math::vec3(1.0f, 1.0f, 1.0f);
        Math::vec3 submeshHighlightColor = Math::vec3(1.0f, 0.8f, 0.1f);
        f32 submeshHighlightAlpha = 0.35f;
        Math::vec3 boxSelectColor = Math::vec3(1.0f, 0.65f, 0.0f);
        bool colorBySubmesh = false;
        // Multiply by painted vertex colours (glTF COLOR_0).
        bool showVertexColors = true;

        f32 debugVectorLength = 0.15f;
        Math::vec3 normalVectorColor = Math::vec3(0.2f, 0.9f, 1.0f);
        Math::vec3 tangentVectorColor = Math::vec3(1.0f, 0.3f, 0.7f);
    };

    struct AnimationSettings
    {
        enum class InterpolationMode : u8
        {
            Linear,
            Bezier,
            Constant
        };
        InterpolationMode interpolation = InterpolationMode::Linear;
        f32 playbackSpeed = 1.0f;
        bool autoLoop = true;
    };

    struct GeneralSettings
    {
        std::string lastOpenedMesh;
        std::string lastOpenDirectory;
        bool showNormals = false;
        bool showWireframe = false;
        f32 vertexPointSize = 5.0f;
        int themeIndex = 2;
        // Most recent first, capped at kMaxRecentFiles; use addRecentFile() etc., never push_back.
        std::vector<std::string> recentFiles;
    };

    // Gizmo snap steps, and the pixel reach for Ctrl-drag vertex snap.
    struct SnapSettings
    {
        f32 moveStep = 1.0f;
        f32 rotateStepDegrees = 15.0f;
        f32 scaleStep = 0.1f;
        f32 vertexRadiusPixels = 14.0f;
    };

    SnapSettings& snap()
    {
        return mSnap;
    }
    const SnapSettings& snap() const
    {
        return mSnap;
    }

    // The secret is not kept here: it comes from the command line or environment so it never lands in a file.
    struct ApiSettings
    {
        bool enabled = false;
        int port = 7420;
    };

    ApiSettings& api()
    {
        return mApi;
    }
    const ApiSettings& api() const
    {
        return mApi;
    }

    ViewportSettings& viewport()
    {
        return mViewport;
    }
    const ViewportSettings& viewport() const
    {
        return mViewport;
    }

    AnimationSettings& animation()
    {
        return mAnimation;
    }
    const AnimationSettings& animation() const
    {
        return mAnimation;
    }

    GeneralSettings& general()
    {
        return mGeneral;
    }
    const GeneralSettings& general() const
    {
        return mGeneral;
    }

    bool load(const std::string& path);
    bool save(const std::string& path);

    void addRecentFile(const std::string& path);
    // Used when opening a recent entry fails, so a moved/deleted file does not stay listed.
    void removeRecentFile(const std::string& path);
    void clearRecentFiles();

private:
    ViewportSettings mViewport;
    AnimationSettings mAnimation;
    GeneralSettings mGeneral;
    ApiSettings mApi;
    SnapSettings mSnap;
};

} // namespace Radion

#endif // RADION_BLENDER_SETTINGS_H
