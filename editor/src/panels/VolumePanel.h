#ifndef RADION_VOLUME_PANEL_H
#define RADION_VOLUME_PANEL_H

#include "EditorPanel.h"
#include "Mesh.h"
#include "VolumeGrid.h"
#include "VolumeMesher.h"

#include <string>

namespace Radion
{

class VolumePanel final : public EditorPanel
{
public:
    explicit VolumePanel(EditorApplication& app);
    ~VolumePanel() override;

    void onImGui() override;

private:
    enum BaseFill : int { FillEmpty, FillSolid, FillSphere, FillTerrain };
    enum BrushShape : int { BrushSphere, BrushBox };

    void drawVolumeSection();
    void drawBrushSection();
    void drawMeshSection();
    void createVolume();
    void applyBrush();
    void rebuildMesh(bool createObject);
    void writeVolumeFile();
    void loadVolumeFile();
    void drawGizmos();
    AABB brushBounds() const;
    std::string outputBase();

    Volume::GridSource* mGrid = nullptr;

    Math::vec3 mOrigin{-8.0f, -4.0f, -8.0f};
    int mDimensions[3] = {96, 48, 96};
    f32 mCellSize = 0.25f;
    int mBaseFill = FillTerrain;
    f32 mFillRadius = 6.0f;
    f32 mTerrainHeight = 0.0f;
    f32 mNoiseFrequency = 0.08f;
    f32 mNoiseAmplitude = 2.5f;
    int mNoiseSeed = 1337;

    int mBrushShape = BrushSphere;
    int mBrushOperation = static_cast<int>(Volume::VolumeOperation::Difference);
    Math::vec3 mBrushCenter{0.0f};
    f32 mBrushRadius = 1.5f;
    Math::vec3 mBrushHalfExtents{1.0f};
    bool mBrushAtCursor = true;
    bool mAutoRebuild = true;

    f32 mIsoLevel = 0.0f;
    bool mSmoothNormals = true;
    bool mPlanarUV = true;
    f32 mUVResolution = 1.0f;

    bool mShowVolumeBounds = true;
    bool mShowBrush = true;
    bool mShowChangedBounds = false;
    AABB mLastChanged;

    std::string mName = "volume";
    MeshHandle mMesh;
    u64 mObjectId = 0;
    u32 mBrushCount = 0;
    bool mMeshDirty = false;
    Volume::MeshingStats mStats;
    f32 mBuildMilliseconds = 0.0f;
};

} // namespace Radion

#endif // RADION_VOLUME_PANEL_H
