#ifndef RADION_TILE_PAINTER_PANEL_H
#define RADION_TILE_PAINTER_PANEL_H

#include "EditorPanel.h"

namespace Radion
{

class TilePainterPanel final : public EditorPanel
{
public:
    explicit TilePainterPanel(EditorApplication& app);

    void onImGui() override;

private:
    enum class Tool
    {
        Brush,
        Pick,
        Fill,
        Rectangle
    };

    int mSelectedTile = 0;
    f32 mAtlasZoom = 1.0f;
    f32 mMapZoom = 1.0f;
    Tool mTool = Tool::Brush;
    bool mPainting = false;
    bool mActionApplied = false;
    int mRectStartX = 0;
    int mRectStartZ = 0;
    int mRectEndX = 0;
    int mRectEndZ = 0;
    bool mRectangleErase = false;
};

} // namespace Radion

#endif // RADION_TILE_PAINTER_PANEL_H
