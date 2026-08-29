#ifndef RADION_DEBUG_PANEL_H
#define RADION_DEBUG_PANEL_H

#include "EditorPanel.h"
#include "EnvironmentProbe.h"
#include "OffscreenTarget.h"
#include "Shadows.h"

namespace Radion
{
class DebugPanel final : public EditorPanel
{
public:
    explicit DebugPanel(EditorApplication& app);
    ~DebugPanel() override;

    void onImGui() override;

private:
    void drawSpatialDebugSection();
    void drawPhysicsDebugSection();
    void drawAIDebugSection();
    void drawSubmeshBoundsReadout();
    void drawSurfaceProbeReadout();
    void drawCascadePreviews();
    void drawProbePreviews();

    OffscreenTarget mPreview;
    // One target per cascade rather than mPreview reused behind a slider -
    // there is enough width to lay every cascade out side by side, and that
    // is a straight comparison ("is the far one starving for resolution")
    // a one-at-a-time view never gave without flipping back and forth.
    OffscreenTarget mCascadePreviews[MaxShadowCascades];
    OffscreenTarget mProbePreviews[EnvironmentProbe::FaceCount];
    bool mEnabled = false;
    int mView = 0;
    int mProbeIndex = 0;
    int mProbeMip = 0;
    int mRenderedProbeIndex = -1;
    int mRenderedProbeMip = -1;
    u64 mRenderedProbeCapture = ~u64(0);
    u32 mRenderedProbeSize = 0;
};
} // namespace Radion

#endif // RADION_DEBUG_PANEL_H
