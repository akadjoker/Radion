#ifndef RADION_LIGHTMAP_PANEL_H
#define RADION_LIGHTMAP_PANEL_H

#include "EditorPanel.h"
#include "LightmapBakePass.h"
#include "LightmapUnwrapJob.h"
#include "LightmapUnwrapper.h"

#include <string>

namespace Radion
{

class GameObject;
class MeshRenderer;

class LightmapPanel final : public EditorPanel
{
public:
    explicit LightmapPanel(EditorApplication& app);

    void onImGui() override;

private:
    void drawUnwrapSection(MeshRenderer& renderer, MeshData& data);
    void drawBakeSection(GameObject& object, MeshRenderer& renderer, MeshData& data);
    void applyPreset(bool draft, const MeshData& data, const Math::mat4& transform);
    // The sun the scene is actually lit by, not the sky's: buildRenderList()
    // sends the renderer this object's own forward(), so a bake reading
    // anything else silently disagrees with the real-time lighting the moment
    // the light is moved directly.
    bool sceneSun(Math::vec3& direction, Math::vec3& color);
    void applyBakedTexture(MeshRenderer& renderer, MeshData& data, const std::string& file);

    LightmapUnwrapSettings mUnwrapSettings;
    LightmapUnwrapJob mUnwrapJob;
    // Which object the running unwrap belongs to - the selection can move
    // while it runs, and the result must not land on a different mesh.
    u64 mUnwrapObjectId = 0;
    int mFitResolution = 2048;

    LightmapBakeSettings mBakeSettings;
    LightmapBakePass mBakePass;
    u32 mBakeResolution = 1024;
    // bake() is one blocking call, so the frame that says it started has to
    // be presented before it runs - pressing the button only arms this.
    bool mBakeRequested = false;
    u32 mBakeFramesShown = 0;
};

} // namespace Radion

#endif // RADION_LIGHTMAP_PANEL_H
