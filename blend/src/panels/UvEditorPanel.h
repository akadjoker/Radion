#ifndef RADION_UV_EDITOR_PANEL_H
#define RADION_UV_EDITOR_PANEL_H

#include "../BlenderPanel.h"
#include "Types.h"

#include "Math.h"
#include <vector>

struct ImVec2;

namespace Radion
{

// The 2D view of a part's UVs over its albedo map: select, drag, and one-click operations. UV (0,0) is the top left, as in glTF.
// Edits go through BlenderApplication, one undo step each (a whole drag is one).
class UvEditorPanel : public BlenderPanel
{
public:
    explicit UvEditorPanel(BlenderApplication& app);

    void onImGui() override;

private:
    enum class Drag
    {
        None,
        Pan,
        Box,
        Move
    };

    void drawToolbar(const std::vector<u32>& shown);
    void drawCanvas(const std::vector<u32>& triangles, const std::vector<u32>& shown);
    void validateSelection();
    std::vector<u32> selectedVertices() const;
    std::vector<u32> operationTarget(const std::vector<u32>& shown) const;
    void frameView(const ImVec2& canvasSize);

    std::vector<u8> mSelected;
    Math::vec2 mPan = Math::vec2(0.0f); // screen offset of UV (0,0) from the canvas corner
    f32 mZoom = 256.0f;               // pixels per UV unit
    bool mFrameRequested = true;

    Drag mDrag = Drag::None;
    Math::vec2 mDragStart = Math::vec2(0.0f);          // screen, at the press
    std::vector<std::pair<u32, Math::vec2>> mMoveOrigin;
    bool mMoveRecorded = false;

    bool mSnap = false;
    f32 mSnapStep = 1.0f / 16.0f;
    bool mShowTexture = true;
    f32 mRotateStep = 90.0f;
    f32 mScaleStep = 2.0f;
    f32 mBoxTile = 1.0f;
    f32 mFitMargin = 0.02f;
};

} // namespace Radion

#endif // RADION_UV_EDITOR_PANEL_H
