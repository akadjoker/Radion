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
    // The scene's actual sun: buildRenderList() uses this object's forward(), so the bake must match.
    bool sceneSun(Math::vec3& direction, Math::vec3& color);
    void applyBakedTexture(MeshRenderer& renderer, MeshData& data, const std::string& file);

    LightmapUnwrapSettings mUnwrapSettings;
    LightmapUnwrapJob mUnwrapJob;
    // Object the running unwrap belongs to; the selection may move meanwhile.
    u64 mUnwrapObjectId = 0;
    int mFitResolution = 2048;

    LightmapBakeSettings mBakeSettings;
    LightmapBakePass mBakePass;
    u32 mBakeResolution = 1024;
    // bake() blocks, so the frame announcing it must be presented first; the button only arms this.
    bool mBakeRequested = false;
    u32 mBakeFramesShown = 0;
};

} // namespace Radion

#endif // RADION_LIGHTMAP_PANEL_H
