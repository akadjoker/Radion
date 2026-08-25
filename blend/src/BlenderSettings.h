#ifndef RADION_BLENDER_SETTINGS_H
#define RADION_BLENDER_SETTINGS_H

#include <glm/vec3.hpp>
#include <string>
#include <vector>

namespace Radion
{

class BlenderSettings
{
public:
    BlenderSettings();
    ~BlenderSettings();

    // Viewport settings
    struct ViewportSettings
    {
        f32 fov = 60.0f;
        f32 nearPlane = 0.1f;
        f32 farPlane = 1000.0f;
        glm::vec3 backgroundColor = glm::vec3(0.12f, 0.12f, 0.12f);

        glm::vec3 vertexColor = glm::vec3(0.9f, 0.45f, 0.05f);
        glm::vec3 selectedVertexColor = glm::vec3(1.0f, 0.9f, 0.2f);
        glm::vec3 faceHighlightColor = glm::vec3(1.0f, 0.6f, 0.0f);
        f32 faceHighlightAlpha = 0.35f;
        glm::vec3 faceEdgeHighlightColor = glm::vec3(1.0f, 1.0f, 1.0f);
        glm::vec3 submeshHighlightColor = glm::vec3(1.0f, 0.8f, 0.1f);
        f32 submeshHighlightAlpha = 0.35f;
        glm::vec3 boxSelectColor = glm::vec3(1.0f, 0.65f, 0.0f);
        bool colorBySubmesh = false;

        // Normals/Tangents debug view: line length drawn from each vertex,
        // and the two colors - the shader's own color-coded surface is hard
        // to read a direction off of, this is the line-per-vertex on top.
        f32 debugVectorLength = 0.15f;
        glm::vec3 normalVectorColor = glm::vec3(0.2f, 0.9f, 1.0f);
        glm::vec3 tangentVectorColor = glm::vec3(1.0f, 0.3f, 0.7f);
    };

    // Animation settings
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

    // General settings
    struct GeneralSettings
    {
        std::string lastOpenedMesh;
        std::string lastOpenDirectory;
        bool showNormals = false;
        bool showWireframe = false;
        f32 vertexPointSize = 5.0f;
        int themeIndex = 2;
        // Most recent first, capped at kMaxRecentFiles - use addRecentFile()/
        // removeRecentFile()/clearRecentFiles() to change it, never push_back
        // directly, so the cap and the promote-to-top rule always hold.
        std::vector<std::string> recentFiles;
    };

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

    // Moves `path` to the front of general().recentFiles, adding it if it
    // was not already there, and drops anything past kMaxRecentFiles.
    void addRecentFile(const std::string& path);
    // Drops `path` from general().recentFiles if present - used when opening
    // a recent entry fails, so a moved/deleted file does not stay listed.
    void removeRecentFile(const std::string& path);
    void clearRecentFiles();

private:
    ViewportSettings mViewport;
    AnimationSettings mAnimation;
    GeneralSettings mGeneral;
};

} // namespace Radion

#endif // RADION_BLENDER_SETTINGS_H
