#include "PCH.h"

#include "panels/VolumePanel.h"

#include "AssetManager.h"
#include "DebugDraw3D.h"
#include "EditorApplication.h"
#include "FileSystem.h"
#include "GameObject.h"
#include "Hash.h"
#include "Log.h"
#include "MeshRenderer.h"
#include "Scene.h"
#include "Timer.h"
#include "VolumeCSG.h"

#include <IconsMaterialDesignIcons.h>
#include <cstring>
#include <imgui.h>

namespace Radion
{

namespace
{
void tooltip(const char* text)
{
    if (!ImGui::IsItemHovered())
        return;
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}
} // namespace

VolumePanel::VolumePanel(EditorApplication& app) : EditorPanel("Volume", app)
{
}

VolumePanel::~VolumePanel()
{
    delete mGrid;
}

std::string VolumePanel::outputBase()
{
    return app().assetBrowserRoot() + "/volumes/" + mName;
}

AABB VolumePanel::brushBounds() const
{
    const glm::vec3 half = mBrushShape == BrushSphere ? glm::vec3(mBrushRadius) : mBrushHalfExtents;
    AABB bounds;
    bounds.min = mBrushCenter - half;
    bounds.max = mBrushCenter + half;
    return bounds;
}

void VolumePanel::onImGui()
{
    if (mBrushAtCursor)
        mBrushCenter = app().cursor3D();

    drawVolumeSection();
    if (mGrid && mGrid->valid())
    {
        ImGui::Separator();
        drawBrushSection();
        ImGui::Separator();
        drawMeshSection();
        drawGizmos();
    }
}

void VolumePanel::drawVolumeSection()
{
    ImGui::SeparatorText("Volume");

    char nameBuffer[128];
    std::strncpy(nameBuffer, mName.c_str(), sizeof(nameBuffer) - 1);
    nameBuffer[sizeof(nameBuffer) - 1] = '\0';
    if (ImGui::InputText("Name", nameBuffer, sizeof(nameBuffer)))
        mName = nameBuffer;
    tooltip("Names the object in the scene and the files under assets/volumes.");

    ImGui::InputFloat3("Origin", &mOrigin.x);
    tooltip("World position of the volume's minimum corner.");
    ImGui::DragInt3("Samples", mDimensions, 1.0f, 2, 512);
    tooltip("Number of density samples per axis. The mesher walks one cell fewer than this on "
            "each axis, so 96 samples is 95 cells. Cost is the product of the three: doubling "
            "one axis doubles the whole build.");
    ImGui::DragFloat("Cell Size", &mCellSize, 0.01f, 0.01f, 8.0f, "%.3f");
    tooltip("World size of one cell. Smaller resolves finer detail and costs cubically.");

    const glm::vec3 extent = glm::vec3(mDimensions[0] - 1, mDimensions[1] - 1, mDimensions[2] - 1) *
                             mCellSize;
    ImGui::Text("Extent %.2f x %.2f x %.2f (%.1f M samples)", extent.x, extent.y, extent.z,
                double(mDimensions[0]) * mDimensions[1] * mDimensions[2] / 1000000.0);

    ImGui::Combo("Base Fill", &mBaseFill, "Empty\0Solid Box\0Sphere\0Terrain\0");
    tooltip("What the volume starts as. Empty means every brush has to add material; Terrain is "
            "a ground plane displaced by noise, which is the usual starting point for caves.");
    if (mBaseFill == FillSphere)
        ImGui::DragFloat("Fill Radius", &mFillRadius, 0.05f, 0.1f, 256.0f);
    if (mBaseFill == FillTerrain)
    {
        ImGui::DragFloat("Ground Height", &mTerrainHeight, 0.05f);
        tooltip("World Y the unperturbed ground sits at.");
        ImGui::DragFloat("Noise Frequency", &mNoiseFrequency, 0.001f, 0.001f, 2.0f, "%.4f");
        tooltip("Higher is more, smaller hills.");
        ImGui::DragFloat("Noise Amplitude", &mNoiseAmplitude, 0.05f, 0.0f, 64.0f);
        tooltip("How far the noise pushes the ground up and down, in world units.");
        ImGui::DragInt("Noise Seed", &mNoiseSeed, 1.0f, 0, 100000);
    }

    if (ImGui::Button(ICON_MDI_CUBE_OUTLINE " Create Volume", ImVec2(-1.0f, 0.0f)))
        createVolume();
    tooltip("Discards the current volume and allocates a new one. The mesh already built stays "
            "in the scene as its own asset.");

    if (!mGrid || !mGrid->valid())
    {
        ImGui::TextDisabled("No volume. Create one to start carving.");
        return;
    }

    const AABB bounds = mGrid->bounds();
    ImGui::Text("Bounds (%.1f %.1f %.1f) to (%.1f %.1f %.1f)", bounds.min.x, bounds.min.y,
                bounds.min.z, bounds.max.x, bounds.max.y, bounds.max.z);
    ImGui::Text("%u brush%s applied", mBrushCount, mBrushCount == 1 ? "" : "es");

    if (ImGui::Button(ICON_MDI_CONTENT_SAVE " Save .rvol"))
        writeVolumeFile();
    tooltip("Writes the density grid next to the mesh. The .rmesh is what the scene draws; the "
            ".rvol is what lets you keep carving in a later session.");
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_FOLDER_OPEN " Load .rvol"))
        loadVolumeFile();
}

void VolumePanel::drawBrushSection()
{
    ImGui::SeparatorText("Brush");

    ImGui::Combo("Shape", &mBrushShape, "Sphere\0Box\0");
    ImGui::Combo("Operation", &mBrushOperation, "Union (add)\0Difference (cut)\0Intersection\0");
    tooltip("Union welds the brush into the volume. Difference carves it away - a sphere dragged "
            "along a path is a tunnel. Intersection keeps only what both cover.");

    ImGui::Checkbox("Follow 3D Cursor", &mBrushAtCursor);
    tooltip("The brush sits on the viewport's 3D cursor, so clicking in the scene aims it. Turn "
            "off to type an exact position.");
    if (!mBrushAtCursor)
        ImGui::DragFloat3("Center", &mBrushCenter.x, 0.05f);

    if (mBrushShape == BrushSphere)
        ImGui::DragFloat("Radius", &mBrushRadius, 0.05f, 0.01f, 128.0f);
    else
        ImGui::DragFloat3("Half Extents", &mBrushHalfExtents.x, 0.05f, 0.01f, 128.0f);

    ImGui::Checkbox("Rebuild After Apply", &mAutoRebuild);
    tooltip("Remeshes the whole volume on every brush. Turn off on a large volume and press "
            "Rebuild Mesh when the carving is done.");

    if (ImGui::Button(ICON_MDI_BRUSH " Apply Brush", ImVec2(-1.0f, 0.0f)))
        applyBrush();
    tooltip("Writes the brush into the density grid. This is the CSG: the grid is the accumulated "
            "result, not a stack replayed on every sample.");
}

void VolumePanel::drawMeshSection()
{
    ImGui::SeparatorText("Mesh");

    ImGui::DragFloat("Iso Level", &mIsoLevel, 0.005f, -4.0f, 4.0f);
    tooltip("The density the surface is drawn at. Above zero shrinks the solid, below zero "
            "inflates it.");
    ImGui::Checkbox("Smooth Normals", &mSmoothNormals);
    tooltip("On, normals come from the density gradient, which is the true surface normal. Off "
            "recomputes them per face, which shows the voxel facets.");
    ImGui::Checkbox("Planar UVs", &mPlanarUV);
    if (mPlanarUV)
    {
        ImGui::DragFloat("UV Resolution", &mUVResolution, 0.01f, 0.01f, 64.0f);
        tooltip("World units per UV tile.");
    }

    ImGui::Checkbox("Volume Bounds", &mShowVolumeBounds);
    ImGui::SameLine();
    ImGui::Checkbox("Brush", &mShowBrush);
    ImGui::SameLine();
    ImGui::Checkbox("Last Change", &mShowChangedBounds);
    tooltip("The AABB the last brush actually touched - what a chunked rebuild would have to "
            "redo, and nothing more.");

    const bool built = mMesh.valid();
    if (ImGui::Button(built ? ICON_MDI_REFRESH " Rebuild Mesh" : ICON_MDI_PLAY " Build Mesh",
                      ImVec2(-1.0f, 0.0f)))
        rebuildMesh(!built);

    if (built && mMeshDirty)
    {
        if (ImGui::Button(ICON_MDI_CONTENT_SAVE " Write .rmesh", ImVec2(-1.0f, 0.0f)))
        {
            MeshData* data = app().importedMeshData(mMesh);
            if (data && AssetManager::getSingleton().saveMesh(*data, outputBase() + ".rmesh",
                                                              std::string()))
            {
                mMeshDirty = false;
                app().toasts().success("Written " + FileSystem::fileName(outputBase() + ".rmesh"));
            }
            else
                app().toasts().error("Could not write the mesh");
        }
        tooltip("The mesh in the scene is ahead of the file on disk - carving updates the GPU "
                "copy only, so the asset is rewritten when you ask, not on every stroke.");
    }

    if (mStats.triangles)
        ImGui::Text("%u triangles from %u cells, %llu samples, %.1f ms", mStats.triangles,
                    mStats.cells, static_cast<unsigned long long>(mStats.samples),
                    double(mBuildMilliseconds));
}

void VolumePanel::createVolume()
{
    delete mGrid;
    mGrid = new Volume::GridSource(glm::uvec3(mDimensions[0], mDimensions[1], mDimensions[2]),
                                   mOrigin, mCellSize, -1.0f);
    if (!mGrid->valid())
    {
        app().toasts().error("Volume too large to allocate");
        delete mGrid;
        mGrid = nullptr;
        return;
    }

    const AABB bounds = mGrid->bounds();
    const glm::vec3 center = (bounds.min + bounds.max) * 0.5f;
    switch (mBaseFill)
    {
    case FillSolid:
    {
        const Volume::BoxSource box(center, (bounds.max - bounds.min) * 0.5f - glm::vec3(mCellSize));
        mGrid->fill(box);
        break;
    }
    case FillSphere:
    {
        const Volume::SphereSource sphere(center, mFillRadius);
        mGrid->fill(sphere);
        break;
    }
    case FillTerrain:
    {
        // Inside is below the ground, so the plane's normal points down and
        // its offset is the height: density is groundHeight - y.
        const Volume::PlaneSource ground(glm::vec3(0.0f, -1.0f, 0.0f), mTerrainHeight);
        Volume::NoiseSource terrain(ground, static_cast<u32>(mNoiseSeed), mNoiseFrequency,
                                    mNoiseAmplitude);
        mGrid->fill(terrain);
        break;
    }
    default:
        break;
    }

    mBrushCount = 0;
    mStats = Volume::MeshingStats{};
    app().toasts().info("Volume created");
}

void VolumePanel::applyBrush()
{
    if (!mGrid || !mGrid->valid())
        return;

    const auto operation = static_cast<Volume::VolumeOperation>(mBrushOperation);
    const AABB affected = brushBounds();
    if (mBrushShape == BrushSphere)
    {
        const Volume::SphereSource brush(mBrushCenter, mBrushRadius);
        mLastChanged = mGrid->apply(operation, brush, affected);
    }
    else
    {
        const Volume::BoxSource brush(mBrushCenter, mBrushHalfExtents);
        mLastChanged = mGrid->apply(operation, brush, affected);
    }

    if (mLastChanged.empty())
    {
        app().toasts().warning("Brush changed nothing - it is outside the volume");
        return;
    }
    ++mBrushCount;
    if (mAutoRebuild)
        rebuildMesh(!mMesh.valid());
}

void VolumePanel::rebuildMesh(bool createObject)
{
    if (!mGrid || !mGrid->valid())
        return;

    Volume::MeshingSettings settings;
    settings.bounds = mGrid->bounds();
    settings.voxelSize = mGrid->cellSize();
    settings.isoLevel = mIsoLevel;
    settings.generateUVs = false;

    Timer timer;
    MeshData data;
    const bool ok = Volume::buildMesh(*mGrid, settings, data, &mStats);
    timer.tick();
    mBuildMilliseconds = static_cast<f32>(timer.getElapsedTime() * 1000.0);
    if (!ok)
    {
        app().toasts().error("Invalid volume settings");
        return;
    }
    if (data.positions.empty())
    {
        app().toasts().warning("The volume is entirely solid or entirely empty");
        return;
    }

    AssetManager& assets = AssetManager::getSingleton();
    if (!mSmoothNormals)
        assets.recalculateNormals(data, false);
    if (mPlanarUV)
        assets.makePlanarUV(data, mUVResolution);
    assets.recalculateTangents(data);

    if (data.materials.empty())
    {
        data.materials.emplace_back();
        data.materials.back().name = mName;
        data.materials.back().nameHash = hashName(mName.c_str());
    }

    if (createObject)
    {
        FileSystem::getSingleton().createDirectory(app().assetBrowserRoot() + "/volumes");
        GameObject* object = app().adoptGeneratedMesh(mName, outputBase(), std::move(data));
        if (!object)
            return;
        MeshRenderer* renderer = object->getComponent<MeshRenderer>();
        mMesh = renderer ? renderer->mesh() : MeshHandle{};
        mObjectId = object->id();
        mMeshDirty = false;
        return;
    }

    app().registerImportedMesh(mMesh, std::move(data));
    if (!app().applyMeshEdit(mMesh))
    {
        app().toasts().error("Could not upload the rebuilt mesh");
        return;
    }
    mMeshDirty = true;
}

void VolumePanel::writeVolumeFile()
{
    if (!mGrid || !mGrid->valid())
        return;
    ByteArray encoded;
    if (!mGrid->save(encoded))
    {
        app().toasts().error("Could not encode the volume");
        return;
    }
    FileSystem& fs = FileSystem::getSingleton();
    fs.createDirectory(app().assetBrowserRoot() + "/volumes");
    const std::string path = outputBase() + ".rvol";
    if (fs.writeBinary(path, encoded))
        app().toasts().success("Saved " + FileSystem::fileName(path));
    else
        app().toasts().error("Could not write " + FileSystem::fileName(path));
}

void VolumePanel::loadVolumeFile()
{
    const std::string path = outputBase() + ".rvol";
    ByteArray encoded = FileSystem::getSingleton().readBinary(path);
    if (encoded.empty())
    {
        app().toasts().error("Could not read " + FileSystem::fileName(path));
        return;
    }
    encoded.seek(0);
    Volume::GridSource* loaded = new Volume::GridSource(glm::uvec3(1), glm::vec3(0.0f), 1.0f, -1.0f);
    if (!Volume::GridSource::load(encoded, *loaded))
    {
        delete loaded;
        app().toasts().error(FileSystem::fileName(path) + " is not a volume");
        return;
    }
    delete mGrid;
    mGrid = loaded;
    mOrigin = mGrid->origin();
    mCellSize = mGrid->cellSize();
    const glm::uvec3 dimensions = mGrid->dimensions();
    mDimensions[0] = static_cast<int>(dimensions.x);
    mDimensions[1] = static_cast<int>(dimensions.y);
    mDimensions[2] = static_cast<int>(dimensions.z);
    mBrushCount = 0;
    app().toasts().success("Loaded " + FileSystem::fileName(path));
}

void VolumePanel::drawGizmos()
{
    if (app().viewMode() != EditorApplication::ViewMode::Scene)
        return;

    if (mShowVolumeBounds)
        DebugDraw().box(mGrid->bounds(), Color::Cyan);
    if (mShowChangedBounds && !mLastChanged.empty())
        DebugDraw().box(mLastChanged, Color::Magenta);
    if (!mShowBrush)
        return;

    const Color color = mBrushOperation == static_cast<int>(Volume::VolumeOperation::Difference)
                            ? Color::Red
                            : Color::Green;
    if (mBrushShape == BrushBox)
    {
        DebugDraw().box(brushBounds(), color);
        return;
    }
    constexpr u32 segments = 32;
    DebugDraw().circle(mBrushCenter, glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), mBrushRadius,
                       segments, color);
    DebugDraw().circle(mBrushCenter, glm::vec3(1, 0, 0), glm::vec3(0, 0, 1), mBrushRadius,
                       segments, color);
    DebugDraw().circle(mBrushCenter, glm::vec3(0, 1, 0), glm::vec3(0, 0, 1), mBrushRadius,
                       segments, color);
}

} // namespace Radion
