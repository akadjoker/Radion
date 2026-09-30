#ifndef RADION_TIMELINE_PANEL_H
#define RADION_TIMELINE_PANEL_H

#include "../BlenderPanel.h"
#include "Types.h"

namespace Radion
{

class TimelinePanel : public BlenderPanel
{
public:
    TimelinePanel(BlenderApplication& app);
    ~TimelinePanel() override;

    void onImGui() override;

private:
    void drawPlaybackControls();
    void drawAnimationClips();
    void drawTimelineRuler();
    void drawKeyframeTrack();

    f32 mZoom = 1.0f;
    f32 mScrollOffset = 0.0f;
};

} // namespace Radion

#endif // RADION_TIMELINE_PANEL_H
