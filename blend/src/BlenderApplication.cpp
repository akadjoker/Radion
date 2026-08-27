#include "PCH.h"
#include "BlenderApplication.h"
#include "BlenderPanel.h"
#include "BlenderTheme.h"
#include "Engine.h"
#include "FileSystem.h"
#include "HullMesh.h"
#include "LightmapUnwrapper.h"
#include "MeshClipper.h"
#include "Log.h"
#include "MaterialManager.h"
#include "ObjExporter.h"
#include "panels/ConsolePanel.h"
#include "panels/HierarchyPanel.h"
#include "panels/MaterialsPanel.h"
#include "panels/MeshHealthPanel.h"
#include "panels/PropertiesPanel.h"
#include "panels/TimelinePanel.h"
#include "panels/ViewportPanel.h"

#include "Math.h"
#include <imgui.h>
#include <imgui_internal.h> // DockBuilder* - building the first-run default layout
#include <utility>

using namespace Radion;

namespace
{
constexpr const char* kDockspaceId = "BlenderDockspaceId";
constexpr f32 kStatusBarHeight = 24.0f;
constexpr usize kUndoBudgetBytes = 256ull * 1024ull * 1024ull;
} // namespace

void BlenderApplication::logSink(LogLevel level, const char* message)
{
    ConsolePanel::pushEntry(level, message);
}

BlenderApplication::BlenderApplication(Engine& engine)
    : mEngine(engine), mRenderer(engine), mSelection(), mSettings()
{
    Log::setMode(LogMode::Verbose);
    Log::setSink(&BlenderApplication::logSink);
    mMeshData = new MeshData();
    mSettingsPath = FileSystem::getSingleton().prefPath("Radion", "Blender") + "blender_editor_settings.json";
    mSettings.load(mSettingsPath);
    buildPanels();
    mRenderer.initialize();
    mBatch.initialize();
}

BlenderApplication::~BlenderApplication()
{
    mSettings.save(mSettingsPath);
    mBatch.shutdown();
    mRenderer.shutdown();
    for (auto panel : mPanels)
        delete panel;
    mPanels.clear();
    delete mMeshData;
}

void BlenderApplication::run()
{
    while (mEngine.update())
    {
        const f32 deltaTime = Math::min(mEngine.getWindow().getDeltaTime(), 0.1f);
        runFrame(deltaTime);
        handleShortcuts();
        drawDockspace();
        drawFileDialog();
        drawSaveInfoPopup();
        drawPrimitivePopup();
        drawUnwrapPopup();
        drawBisectPopup();
        drawNewConfirmPopup();
        drawPreferencesPopup();

        for (BlenderPanel* panel : mPanels)
        {
            if (panel->active())
                panel->onImGui();
        }

        drawStatusBar();

        mEngine.flip();
    }
}

MeshData* BlenderApplication::currentMeshData()
{
    return mMeshData;
}

bool BlenderApplication::loadMesh(const std::string& path)
{
    if (!mMeshData)
        return false;

    MeshData loaded;
    if (!Assets().buildMeshData(MeshDesc::fromFile(path), loaded))
    {
        Log::error("BlenderApplication: failed to load mesh '%s'", path.c_str());
        return false;
    }

    *mMeshData = std::move(loaded);
    Assets().loadMeshDataMaterialTextures(*mMeshData);
    mSettings.general().lastOpenedMesh = path;
    mSettings.addRecentFile(path);
    mSubmeshVisible.clear();
    mDirty = false;
    mRenderer.invalidate();

    mSkeleton = Skeleton();
    mHasSkeleton = false;
    mAnimationClips.clear();
    mActiveClip = -1;
    mLocalPose.clear();
    mGlobalPose.clear();
    mBonePalette.clear();

    // A rig's skeleton and its bind-pose mesh routinely live in the same
    // file (an FBX export, most often) - try it on whatever was just loaded
    // before asking the user to point at anything separately. No skin data,
    // no point looking: a static mesh's own file is never going to resolve
    // to bones.
    if (!mMeshData->skin.empty() && Assets().importSkeleton(path, mSkeleton) && mSkeleton.finalize())
    {
        mHasSkeleton = true;
        Log::info("BlenderApplication: loaded skeleton (%u bones) from '%s'", mSkeleton.boneCount(),
                  path.c_str());

        AnimationClip embedded;
        if (Assets().importAnimation(path, mSkeleton, embedded) && embedded.duration() > 0.0f)
        {
            if (embedded.name().empty())
                embedded.setName(FileSystem::baseName(path));
            mAnimationClips.push_back(std::move(embedded));
            Log::info("BlenderApplication: found embedded animation '%s' (%.2fs)",
                      mAnimationClips.back().name().c_str(), mAnimationClips.back().duration());
            setActiveAnimationClip(0); // calls updateAnimationPose() itself
        }
        else
        {
            updateAnimationPose();
        }
    }

    return true;
}

bool BlenderApplication::importMesh(const std::string& path)
{
    return loadMesh(path);
}

bool BlenderApplication::appendAnimation(const std::string& path)
{
    if (!mHasSkeleton)
        return false;

    AnimationClip clip;
    if (!Assets().importAnimation(path, mSkeleton, clip))
    {
        Log::error("BlenderApplication: failed to import animation '%s'", path.c_str());
        return false;
    }
    addAnimationClip(std::move(clip), path);
    return true;
}

void BlenderApplication::addAnimationClip(AnimationClip clip, const std::string& sourcePath)
{
    if (clip.name().empty())
        clip.setName(FileSystem::baseName(sourcePath));

    mAnimationClips.push_back(std::move(clip));
    Log::info("BlenderApplication: appended animation '%s' (%.2fs, %zu clips total)",
              mAnimationClips.back().name().c_str(), mAnimationClips.back().duration(),
              mAnimationClips.size());
    setActiveAnimationClip(static_cast<s32>(mAnimationClips.size()) - 1);
}

bool BlenderApplication::appendMesh(const std::string& path)
{
    if (!mMeshData)
        return false;

    MeshData incoming;
    if (!Assets().buildMeshData(MeshDesc::fromFile(path), incoming))
    {
        Log::error("BlenderApplication: failed to load '%s' to append", path.c_str());
        return false;
    }

    recordUndo();
    const usize beforeVertexCount = mMeshData->positions.size();

    MeshMergeInput current;
    current.mesh = mMeshData;
    current.sourceName = "current";
    MeshMergeInput appended;
    appended.mesh = &incoming;
    appended.sourceName = FileSystem::baseName(path);

    MeshData merged;
    std::string error;
    if (!Assets().mergeMeshes({current, appended}, MeshMergeOptions(), merged, &error))
    {
        Log::error("BlenderApplication: failed to append '%s': %s", path.c_str(), error.c_str());
        discardUndo();
        return false;
    }

    *mMeshData = std::move(merged);
    Assets().loadMeshDataMaterialTextures(*mMeshData);
    mSubmeshVisible.clear();
    Log::info("BlenderApplication: appended mesh '%s' (%zu -> %zu vertices)", path.c_str(),
              beforeVertexCount, mMeshData->positions.size());
    applyMeshEdit();
    return true;
}

bool BlenderApplication::appendAsset(const std::string& path)
{
    if (mHasSkeleton)
    {
        AnimationClip clip;
        if (Assets().importAnimation(path, mSkeleton, clip) && clip.duration() > 0.0f)
        {
            addAnimationClip(std::move(clip), path);
            return true;
        }
    }
    return appendMesh(path);
}

void BlenderApplication::requestAppendAnimation()
{
    openFileDialog(ImGuiFileDialog::Mode::OpenFile, FileDialogAppendAnimation);
}

void BlenderApplication::removeAnimationClip(u32 index)
{
    if (index >= mAnimationClips.size())
        return;

    mAnimationClips.erase(mAnimationClips.begin() + index);
    if (mActiveClip == static_cast<s32>(index))
        setActiveAnimationClip(mAnimationClips.empty() ? -1 : 0);
    else if (mActiveClip > static_cast<s32>(index))
        --mActiveClip;
}

void BlenderApplication::setActiveAnimationClip(s32 index)
{
    if (index < 0 || static_cast<usize>(index) >= mAnimationClips.size())
    {
        mActiveClip = -1;
        return;
    }

    mActiveClip = index;
    mCurrentFrame = 0;
    mTotalFrames = Math::max(
        1u, static_cast<u32>(mAnimationClips[mActiveClip].duration() * kAnimationFramesPerSecond));
    updateAnimationPose();
}

void BlenderApplication::updateAnimationPose()
{
    if (!mHasSkeleton)
        return;

    // bindPose() both sizes mLocalPose to the skeleton's own bone count and
    // fills every bone's rest offset - sample() below only overwrites the
    // bones its own clip actually has a track for (see its own comment: a
    // bone with no track is left untouched, not reset), so this has to run
    // first every time. Skipping it whenever a clip is active left
    // mLocalPose sized 0 on the very first pose ever built (no bindPose()
    // call had ever run yet), which fails evaluate()'s size check silently -
    // mBonePalette stayed empty, MiniRenderer fell back to binding only
    // uBonePalette[0], and every vertex weighted to any other joint read
    // whatever garbage sat in the rest of that uniform array.
    mSkeleton.bindPose(mLocalPose);
    if (mActiveClip >= 0 && static_cast<usize>(mActiveClip) < mAnimationClips.size())
    {
        const f32 time = static_cast<f32>(mCurrentFrame) / kAnimationFramesPerSecond;
        mAnimationClips[static_cast<usize>(mActiveClip)].sample(time, mLocalPose);
    }
    mSkeleton.evaluate(mLocalPose, mGlobalPose, mBonePalette);
}

bool BlenderApplication::applyMeshEdit()
{
    if (!mMeshData)
        return false;

    mRenderer.invalidate();
    markDirty();
    return true;
}

bool BlenderApplication::deleteSubmesh(u32 index)
{
    if (!mMeshData || index >= mMeshData->submeshes.size())
        return false;

    recordUndo();

    const SubMesh& removed = mMeshData->submeshes[index];
    std::vector<u32>& indices = mMeshData->indices;
    indices.erase(indices.begin() + removed.indexOffset,
                 indices.begin() + removed.indexOffset + removed.indexCount);

    for (usize i = 0; i < mMeshData->submeshes.size(); ++i)
    {
        if (i == index)
            continue;
        SubMesh& other = mMeshData->submeshes[i];
        if (other.indexOffset > removed.indexOffset)
            other.indexOffset -= removed.indexCount;
    }

    mMeshData->submeshes.erase(mMeshData->submeshes.begin() + index);
    if (index < mSubmeshVisible.size())
        mSubmeshVisible.erase(mSubmeshVisible.begin() + index);

    if (mSelectedSubmesh == static_cast<s32>(index))
        mSelectedSubmesh = -1;
    else if (mSelectedSubmesh > static_cast<s32>(index))
        --mSelectedSubmesh;

    applyMeshEdit();
    return true;
}

bool BlenderApplication::isSubmeshVisible(u32 index)
{
    if (!mMeshData || index >= mMeshData->submeshes.size())
        return true;
    if (index >= mSubmeshVisible.size())
        mSubmeshVisible.resize(mMeshData->submeshes.size(), true);
    return mSubmeshVisible[index];
}

void BlenderApplication::setSubmeshVisible(u32 index, bool visible)
{
    if (!mMeshData || index >= mMeshData->submeshes.size())
        return;
    if (index >= mSubmeshVisible.size())
        mSubmeshVisible.resize(mMeshData->submeshes.size(), true);
    mSubmeshVisible[index] = visible;
}

void BlenderApplication::toggleSubmeshVisible(u32 index)
{
    setSubmeshVisible(index, !isSubmeshVisible(index));
}

void BlenderApplication::deleteSelectedVertices()
{
    if (!mMeshData || mSelection.selectedVertexCount() == 0)
        return;

    const usize count = mSelection.selectedVertexCount();
    recordUndo();
    Assets().deleteVertices(*mMeshData, mSelection.selectedVertices());
    mSelection.clearAll();
    mSelectedSubmesh = -1;
    Log::info("BlenderApplication: deleted %zu vertices", count);
    applyMeshEdit();
}

void BlenderApplication::deleteSelectedFaces()
{
    if (!mMeshData || mSelection.selectedFaceCount() == 0)
        return;

    const usize count = mSelection.selectedFaceCount();
    recordUndo();
    Assets().deleteFaces(*mMeshData, mSelection.selectedFaces());
    mSelection.clearAll();
    mSelectedSubmesh = -1;
    Log::info("BlenderApplication: deleted %zu faces", count);
    applyMeshEdit();
}

void BlenderApplication::groupSelectedFacesIntoSubmesh()
{
    if (!mMeshData || mSelection.selectedFaceCount() == 0)
        return;

    const usize count = mSelection.selectedFaceCount();
    recordUndo();
    if (Assets().groupFacesIntoSubmesh(*mMeshData, mSelection.selectedFaces()))
    {
        mSelection.clearAll();
        mSelectedSubmesh = static_cast<s32>(mMeshData->submeshes.size()) - 1;
        Log::info("BlenderApplication: grouped %zu faces into submesh %d", count, mSelectedSubmesh);
        applyMeshEdit();
    }
    else
    {
        discardUndo();
    }
}

void BlenderApplication::recordUndo()
{
    if (!mMeshData)
        return;
    mUndoStates.push_back(*mMeshData);
    mRedoStates.clear();
    trimUndoStates();
}

// A step is a whole copy of the mesh, so what the stack costs depends on what
// is loaded, not on how many edits were made: the same twenty steps are
// nothing on a crate and most of a gigabyte on a scanned model. The budget is
// in bytes for that reason, and the oldest steps go first. One step always
// survives, however big the mesh - a single undo is the least the tool can
// offer, and dropping it would make an edit on a large model unrecoverable.
void BlenderApplication::trimUndoStates()
{
    usize total = 0;
    for (usize i = 0; i < mUndoStates.size(); ++i)
        total += mUndoStates[i].memoryBytes();
    for (usize i = 0; i < mRedoStates.size(); ++i)
        total += mRedoStates[i].memoryBytes();

    while (total > kUndoBudgetBytes && mUndoStates.size() + mRedoStates.size() > 1)
    {
        // Redo first: it is the branch the user already walked away from.
        if (!mRedoStates.empty())
        {
            total -= mRedoStates.front().memoryBytes();
            mRedoStates.erase(mRedoStates.begin());
        }
        else
        {
            total -= mUndoStates.front().memoryBytes();
            mUndoStates.erase(mUndoStates.begin());
        }
    }
}

void BlenderApplication::discardUndo()
{
    if (!mUndoStates.empty())
        mUndoStates.pop_back();
}

void BlenderApplication::undo()
{
    if (mUndoStates.empty() || !mMeshData)
        return;
    mRedoStates.push_back(*mMeshData);
    *mMeshData = std::move(mUndoStates.back());
    mUndoStates.pop_back();
    trimUndoStates();
    mRenderer.invalidate();
    markDirty();
}

void BlenderApplication::redo()
{
    if (mRedoStates.empty() || !mMeshData)
        return;
    mUndoStates.push_back(*mMeshData);
    *mMeshData = std::move(mRedoStates.back());
    mRedoStates.pop_back();
    mRenderer.invalidate();
    markDirty();
}

void BlenderApplication::setCurrentFrame(u32 frame)
{
    if (frame < mTotalFrames)
    {
        mCurrentFrame = frame;
        updateAnimationPose();
    }
}

void BlenderApplication::play()
{
    mPlaying = true;
    mPlaybackTimer = 0.0f;
}

void BlenderApplication::stop()
{
    mPlaying = false;
    mCurrentFrame = 0;
    updateAnimationPose();
}

void BlenderApplication::insertKeyframe()
{
    // Nothing records a keyframe yet. Taking an undo snapshot and marking the
    // file dirty for an edit that never happened costs a whole mesh copy and
    // asks the user to save work that does not exist.
}

void BlenderApplication::deleteKeyframe(u32 frame)
{
    (void)frame;
}

bool BlenderApplication::hasKeyframe(u32 frame) const
{
    (void)frame;
    return false;
}

void BlenderApplication::markDirty()
{
    mDirty = true;
}

bool BlenderApplication::saveAs(const std::string& path)
{
    if (!mMeshData || mMeshData->positions.empty())
        return false;

    // saveMesh() always writes the engine's own .rmesh format regardless of
    // the extension it is handed - forcing it here keeps "Save" on a mesh
    // imported from .obj/.fbx from overwriting that original file with
    // .rmesh bytes under its old extension. Every import is already fully
    // converted to MeshData in memory the moment it loads; this just makes
    // where it lands on disk match what it already is.
    const std::string nativePath = FileSystem::withoutExtension(path) + ".rmesh";
    if (nativePath != path)
        Log::info("BlenderApplication: saving as native mesh '%s' (was '%s')", nativePath.c_str(),
                  path.c_str());

    if (!Assets().saveMesh(*mMeshData, nativePath))
    {
        Log::error("BlenderApplication: failed to save mesh '%s'", nativePath.c_str());
        return false;
    }

    const std::vector<Material> sidecar = Assets().materialsForSidecar(*mMeshData);
    const std::string materialPath = FileSystem::withoutExtension(nativePath) + ".material";
    if (!sidecar.empty() &&
        !MaterialManager::getSingleton().save(materialPath, sidecar))
    {
        Log::error("BlenderApplication: failed to save materials '%s'", materialPath.c_str());
    }

    mSettings.general().lastOpenedMesh = nativePath;
    mDirty = false;
    return true;
}

bool BlenderApplication::exportObj(const std::string& path)
{
    if (!mMeshData || mMeshData->positions.empty())
        return false;

    if (!ObjExporter::save(*mMeshData, path))
    {
        Log::error("BlenderApplication: failed to export OBJ '%s'", path.c_str());
        return false;
    }

    Log::info("BlenderApplication: exported OBJ '%s'", path.c_str());
    return true;
}

void BlenderApplication::buildPanels()
{
    mPanels.push_back(new ViewportPanel(*this));
    mPanels.push_back(new PropertiesPanel(*this));
    mPanels.push_back(new MeshHealthPanel(*this));
    mPanels.push_back(new HierarchyPanel(*this));
    mPanels.push_back(new TimelinePanel(*this));
    mPanels.push_back(new MaterialsPanel(*this));
    mPanels.push_back(new ConsolePanel(*this));
}

// Same shape as EditorApplication::drawDockspace() (editor/src/EditorApplication.cpp):
// a fullscreen, undockable host window carries the DockSpace, and the split
// is built once - after the user redocks anything, ImGui's own .ini
// persistence takes over and DockBuilderGetNode() stops seeing an empty node.
void BlenderApplication::drawDockspace()
{
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const ImVec2 dockSize(viewport->WorkSize.x, viewport->WorkSize.y - kStatusBarHeight);
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(dockSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGuiWindowFlags hostFlags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
                                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                 ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_MenuBar;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("BlenderDockspaceHost", nullptr, hostFlags);
    ImGui::PopStyleVar(3);

    drawMainMenuBar();

    const ImGuiID dockspaceId = ImGui::GetID(kDockspaceId);
    ImGui::DockSpace(dockspaceId, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);

    if (!mDockLayoutBuilt)
    {
        mDockLayoutBuilt = true;
        if (ImGui::DockBuilderGetNode(dockspaceId) == nullptr ||
            ImGui::DockBuilderGetNode(dockspaceId)->IsSplitNode() == false)
        {
            ImGui::DockBuilderRemoveNode(dockspaceId);
            ImGui::DockBuilderAddNode(dockspaceId, ImGuiDockNodeFlags_DockSpace);
            ImGui::DockBuilderSetNodeSize(dockspaceId, dockSize);

            // Blender's own default: a big centre viewport, and the right
            // edge stacked Properties over Hierarchy instead of splitting
            // them left/right - one object at a time to inspect, not two
            // panels competing for the same width. Timeline/mesh-edit/
            // console share the bottom strip as tabs.
            ImGuiID center, right, centerTop, bottom, propertiesTop, hierarchyBottom;
            ImGui::DockBuilderSplitNode(dockspaceId, ImGuiDir_Right, 0.22f, &right, &center);
            ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.28f, &bottom, &centerTop);
            ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.35f, &hierarchyBottom, &propertiesTop);

            ImGui::DockBuilderDockWindow("Viewport", centerTop);
            ImGui::DockBuilderDockWindow("Properties", propertiesTop);
            ImGui::DockBuilderDockWindow("Hierarchy", hierarchyBottom);
            ImGui::DockBuilderDockWindow("Timeline", bottom);
            ImGui::DockBuilderDockWindow("Materials", bottom);
            ImGui::DockBuilderDockWindow("Console", bottom);
            ImGui::DockBuilderFinish(dockspaceId);
        }
    }

    ImGui::End();
}

void BlenderApplication::drawMainMenuBar()
{
    if (!ImGui::BeginMenuBar())
        return;

    const bool hasMesh = mMeshData && !mMeshData->positions.empty();

    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("New", "Ctrl+N"))
        {
            if (mDirty)
                mNewConfirmRequested = true;
            else
                newDocument();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Load..."))
            openFileDialog(ImGuiFileDialog::Mode::OpenFile, FileDialogLoadMesh);
        if (ImGui::MenuItem("Import..."))
            openFileDialog(ImGuiFileDialog::Mode::OpenFile, FileDialogImportMesh);
        drawOpenRecentMenu();
        if (ImGui::MenuItem("Append Animation...", nullptr, false, mHasSkeleton))
            openFileDialog(ImGuiFileDialog::Mode::OpenFile, FileDialogAppendAnimation);
        if (!mHasSkeleton && ImGui::IsItemHovered())
            ImGui::SetTooltip("Load a skinned mesh first - an animation clip needs its skeleton "
                              "to sample onto.");
        ImGui::Separator();
        if (ImGui::MenuItem("Save", "Ctrl+S", false, hasMesh))
        {
            if (mSettings.general().lastOpenedMesh.empty())
                mSaveInfoRequested = true;
            else
                saveAs(mSettings.general().lastOpenedMesh);
        }
        if (ImGui::MenuItem("Save As...", nullptr, false, hasMesh))
            mSaveInfoRequested = true;
        if (ImGui::BeginMenu("Export", hasMesh))
        {
            if (ImGui::MenuItem("Wavefront OBJ..."))
                openFileDialog(ImGuiFileDialog::Mode::SaveFile, FileDialogExportObj);
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Exit"))
            mEngine.getWindow().requestClose();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Edit"))
    {
        if (ImGui::MenuItem("Undo", "Ctrl+Z"))
            undo();
        if (ImGui::MenuItem("Redo", "Ctrl+Y"))
            redo();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Windows"))
    {
        for (BlenderPanel* panel : mPanels)
        {
            bool visible = panel->active();
            if (ImGui::MenuItem(panel->title().c_str(), nullptr, &visible))
                panel->setActive(visible);
        }
        ImGui::Separator();
        if (ImGui::BeginMenu("Theme"))
        {
            int& themeIndex = mSettings.general().themeIndex;
            for (int i = 0; i < kBlenderThemeCount; ++i)
            {
                const bool selected = i == themeIndex;
                if (ImGui::MenuItem(blenderThemeName(static_cast<BlenderThemeKind>(i)), nullptr, selected))
                {
                    themeIndex = i;
                    applyBlenderThemeKind(static_cast<BlenderThemeKind>(i));
                }
            }
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Preferences..."))
            mPreferencesRequested = true;
        ImGui::EndMenu();
    }

    // Outside the block below: Add is how a mesh comes into being, so gating
    // it on there already being one locks the tool shut after File > New.
    drawAddMenu();

    ImGui::BeginDisabled(!hasMesh);
    drawSelectMenu();
    if (ImGui::BeginMenu("Vertex"))
    {
        drawVertexMenu();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Edge"))
    {
        drawEdgeMenu();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Face"))
    {
        drawFaceMenu();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Transform"))
    {
        drawTransformMenu();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Mesh"))
    {
        drawMeshMenu();
        ImGui::EndMenu();
    }
    ImGui::EndDisabled();

    ImGui::EndMenuBar();

    drawToolPopups();
}

void BlenderApplication::drawSelectMenu()
{
    if (!ImGui::BeginMenu("Select"))
        return;

    const BlenderSelection::SelectionMode mode = mSelection.mode();
    if (ImGui::MenuItem("Vertex", nullptr, mode == BlenderSelection::SelectionMode::Vertex))
        mSelection.setMode(BlenderSelection::SelectionMode::Vertex);
    if (ImGui::MenuItem("Edge", nullptr, mode == BlenderSelection::SelectionMode::Edge))
        mSelection.setMode(BlenderSelection::SelectionMode::Edge);
    if (ImGui::MenuItem("Face", nullptr, mode == BlenderSelection::SelectionMode::Face))
        mSelection.setMode(BlenderSelection::SelectionMode::Face);
    ImGui::Separator();
    if (ImGui::MenuItem("Select All", "A"))
        selectAllElements();
    if (ImGui::MenuItem("Deselect All", "Alt+A"))
        mSelection.clearAll();
    if (ImGui::MenuItem("Invert Selection", "Ctrl+I"))
        invertElementSelection();

    ImGui::Separator();
    const bool hasSelection = mSelection.selectedVertexCount() > 0 ||
                              mSelection.selectedFaceCount() > 0;
    ImGui::BeginDisabled(!mMeshData || !hasSelection);
    if (ImGui::MenuItem("Grow", "Ctrl++"))
        growSelection();
    if (ImGui::MenuItem("Shrink", "Ctrl+-"))
        shrinkSelection();
    if (ImGui::MenuItem("Select Linked", "L"))
        selectLinked();
    ImGui::EndDisabled();

    if (mMeshData && mMeshData->submeshes.size() > 1 &&
        ImGui::BeginMenu("Select Submesh"))
    {
        for (u32 i = 0; i < static_cast<u32>(mMeshData->submeshes.size()); ++i)
        {
            // Named by its material, the way the Properties panel labels the
            // same rows; a submesh carries no name of its own.
            const u32 slot = mMeshData->submeshes[i].materialSlot;
            const bool named = slot < mMeshData->materials.size() &&
                               !mMeshData->materials[slot].name.empty();
            const std::string label = named ? mMeshData->materials[slot].name
                                            : ("Submesh " + std::to_string(i));
            // A hidden submesh is unreachable by every other route, so
            // offering it here would mean clicking it and nothing happening.
            const bool visible = isSubmeshVisible(i);
            ImGui::PushID(static_cast<int>(i));
            ImGui::BeginDisabled(!visible);
            if (ImGui::MenuItem(label.c_str()))
                selectSubmeshFaces(i);
            ImGui::EndDisabled();
            if (!visible && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                ImGui::SetTooltip("Hidden");
            ImGui::PopID();
        }
        ImGui::EndMenu();
    }

    ImGui::EndMenu();
}

void BlenderApplication::growSelection()
{
    if (!mMeshData)
        return;

    std::vector<u32> grown;
    if (mSelection.mode() == BlenderSelection::SelectionMode::Face)
    {
        Assets().growFaceSelection(*mMeshData, mSelection.selectedFaces(), grown);
        mSelection.clearAll();
        for (usize i = 0; i < grown.size(); ++i)
            mSelection.selectFace(grown[i]);
    }
    else
    {
        Assets().growVertexSelection(*mMeshData, mSelection.selectedVertices(), grown);
        mSelection.clearAll();
        for (usize i = 0; i < grown.size(); ++i)
            mSelection.selectVertex(grown[i]);
    }

    dropHiddenFromSelection();
}

void BlenderApplication::shrinkSelection()
{
    if (!mMeshData || mSelection.mode() == BlenderSelection::SelectionMode::Face)
        return;

    std::vector<u32> shrunk;
    Assets().shrinkVertexSelection(*mMeshData, mSelection.selectedVertices(), shrunk);
    mSelection.clearAll();
    for (usize i = 0; i < shrunk.size(); ++i)
        mSelection.selectVertex(shrunk[i]);

    dropHiddenFromSelection();
}

void BlenderApplication::selectLinked()
{
    if (!mMeshData)
        return;

    std::vector<u32> linked;
    if (mSelection.mode() == BlenderSelection::SelectionMode::Face)
    {
        Assets().selectLinkedFaces(*mMeshData, mSelection.selectedFaces(), linked);
        mSelection.clearAll();
        for (usize i = 0; i < linked.size(); ++i)
            mSelection.selectFace(linked[i]);
    }
    else
    {
        Assets().selectLinkedVertices(*mMeshData, mSelection.selectedVertices(), linked);
        mSelection.clearAll();
        for (usize i = 0; i < linked.size(); ++i)
            mSelection.selectVertex(linked[i]);
    }

    dropHiddenFromSelection();
}

void BlenderApplication::selectSubmeshFaces(u32 submeshIndex)
{
    if (!mMeshData)
        return;

    std::vector<u32> faces;
    Assets().submeshFaces(*mMeshData, submeshIndex, faces);
    if (faces.empty())
        return;

    mSelection.setMode(BlenderSelection::SelectionMode::Face);
    mSelection.clearAll();
    for (usize i = 0; i < faces.size(); ++i)
        mSelection.selectFace(faces[i]);
}

void BlenderApplication::buildSelectableMask(std::vector<bool>& faceSelectable,
                                             std::vector<bool>& vertexSelectable)
{
    faceSelectable.clear();
    vertexSelectable.clear();
    if (!mMeshData)
        return;

    const MeshData& mesh = *mMeshData;
    const usize faceCount = mesh.indices.size() / 3;
    const usize vertexCount = mesh.positions.size();

    // No submeshes means nothing to hide behind: the mesh is one piece.
    const bool everythingVisible = mesh.submeshes.empty();
    faceSelectable.assign(faceCount, everythingVisible);
    vertexSelectable.assign(vertexCount, everythingVisible);
    if (everythingVisible)
        return;

    for (u32 s = 0; s < static_cast<u32>(mesh.submeshes.size()); ++s)
    {
        if (!isSubmeshVisible(s))
            continue;

        const SubMesh& submesh = mesh.submeshes[s];
        const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
        for (u64 i = submesh.indexOffset; i + 2 < end && i + 2 < mesh.indices.size(); i += 3)
        {
            const usize face = static_cast<usize>(i / 3);
            if (face < faceCount)
                faceSelectable[face] = true;

            for (u32 corner = 0; corner < 3; ++corner)
            {
                const u32 index = mesh.indices[static_cast<usize>(i) + corner];
                if (index < vertexCount)
                    vertexSelectable[index] = true;
            }
        }
    }
}

void BlenderApplication::dropHiddenFromSelection()
{
    if (!mMeshData || mMeshData->submeshes.empty())
        return;

    std::vector<bool> faceSelectable;
    std::vector<bool> vertexSelectable;
    buildSelectableMask(faceSelectable, vertexSelectable);

    const std::vector<u32> vertices = mSelection.selectedVertices();
    for (usize i = 0; i < vertices.size(); ++i)
        if (vertices[i] >= vertexSelectable.size() || !vertexSelectable[vertices[i]])
            mSelection.deselectVertex(vertices[i]);

    const std::vector<u32> faces = mSelection.selectedFaces();
    for (usize i = 0; i < faces.size(); ++i)
        if (faces[i] >= faceSelectable.size() || !faceSelectable[faces[i]])
            mSelection.deselectFace(faces[i]);
}

void BlenderApplication::selectAllElements()
{
    if (!mMeshData)
        return;
    mSelection.selectAll(static_cast<u32>(mMeshData->positions.size()),
                         static_cast<u32>(mMeshData->indices.size() / 3));
    dropHiddenFromSelection();
}

void BlenderApplication::invertElementSelection()
{
    if (!mMeshData)
        return;
    mSelection.invertSelection(static_cast<u32>(mMeshData->positions.size()),
                               static_cast<u32>(mMeshData->indices.size() / 3));
    dropHiddenFromSelection();
}

void BlenderApplication::deleteSelected()
{
    if (mSelection.mode() == BlenderSelection::SelectionMode::Vertex)
        deleteSelectedVertices();
    else if (mSelection.mode() == BlenderSelection::SelectionMode::Face)
        deleteSelectedFaces();
}

// Keyboard is how a modelling tool is actually driven; every one of these was
// already advertised next to its menu item with nothing bound behind it.
// WantCaptureKeyboard is what keeps X from deleting the selection while the
// user is typing an X into a file name.
void BlenderApplication::handleShortcuts()
{
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard || ImGui::IsAnyItemActive())
        return;

    if (io.KeyCtrl)
    {
        if (ImGui::IsKeyPressed(ImGuiKey_Z, false))
        {
            if (io.KeyShift)
                redo();
            else
                undo();
        }
        else if (ImGui::IsKeyPressed(ImGuiKey_Y, false))
            redo();
        else if (ImGui::IsKeyPressed(ImGuiKey_N, false))
        {
            if (mDirty)
                mNewConfirmRequested = true;
            else
                newDocument();
        }
        else if (ImGui::IsKeyPressed(ImGuiKey_I, false))
            invertElementSelection();
        else if (ImGui::IsKeyPressed(ImGuiKey_Equal, false) ||
                 ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, false))
            growSelection();
        else if (ImGui::IsKeyPressed(ImGuiKey_Minus, false) ||
                 ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, false))
            shrinkSelection();
        return;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_1, false))
        mSelection.setMode(BlenderSelection::SelectionMode::Vertex);
    else if (ImGui::IsKeyPressed(ImGuiKey_2, false))
        mSelection.setMode(BlenderSelection::SelectionMode::Edge);
    else if (ImGui::IsKeyPressed(ImGuiKey_3, false))
        mSelection.setMode(BlenderSelection::SelectionMode::Face);

    if (ImGui::IsKeyPressed(ImGuiKey_A, false))
    {
        if (io.KeyAlt)
            mSelection.clearAll();
        else
            selectAllElements();
    }

    if (ImGui::IsKeyPressed(ImGuiKey_X, false) || ImGui::IsKeyPressed(ImGuiKey_Delete, false))
        deleteSelected();

    if (ImGui::IsKeyPressed(ImGuiKey_E, false))
        extrudeSelectedFaces();

    if (ImGui::IsKeyPressed(ImGuiKey_L, false))
        selectLinked();
}

void BlenderApplication::drawVertexMenu()
{
    if (ImGui::MenuItem("Weld..."))
        ImGui::OpenPopup("WeldPopup");
    if (ImGui::MenuItem("Smooth..."))
        ImGui::OpenPopup("SmoothPopup");
    ImGui::BeginDisabled(!mMeshData || mSelection.selectedVertexCount() == 0);
    if (ImGui::MenuItem("Delete Selected", "X"))
        deleteSelectedVertices();
    ImGui::EndDisabled();
}

void BlenderApplication::drawEdgeMenu()
{
    // Edge mode has no selection behind it - updateSelectionInput() returns
    // straight away for it - so there is never anything here to delete.
    ImGui::BeginDisabled(true);
    if (ImGui::MenuItem("Delete Selected", "X"))
    {
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Edge selection is not implemented");
}

void BlenderApplication::drawFaceMenu()
{
    ImGui::SetNextItemWidth(150.0f);
    ImGui::SliderFloat("Extrude Distance", &mExtrudeDistance, -10.0f, 10.0f);
    ImGui::BeginDisabled(!mMeshData || mSelection.selectedFaceCount() == 0);
    if (ImGui::MenuItem("Extrude", "E"))
        extrudeSelectedFaces();
    ImGui::EndDisabled();

    ImGui::BeginDisabled(!mMeshData);
    if (ImGui::MenuItem("Flip Normals"))
    {
        recordUndo();
        // Winding and normals together: reversing the triangles alone leaves
        // every vertex normal pointing the way it did, and the surface lights
        // as though it never turned. Same pair AssetManager::scale() applies
        // when a negative factor mirrors a mesh.
        Assets().flipWinding(*mMeshData);
        Assets().recalculateNormals(*mMeshData, mSmoothNormals, mAngleWeightedNormals);
        applyMeshEdit();
    }
    if (ImGui::MenuItem("Recalc Normals"))
    {
        recordUndo();
        Assets().recalculateNormals(*mMeshData, mSmoothNormals, mAngleWeightedNormals);
        applyMeshEdit();
    }
    ImGui::EndDisabled();

    ImGui::BeginDisabled(!mMeshData || mSelection.selectedFaceCount() == 0);
    if (ImGui::MenuItem("Delete Selected", "X"))
        deleteSelectedFaces();
    ImGui::Separator();
    if (ImGui::MenuItem("Group Selected into Submesh"))
        groupSelectedFacesIntoSubmesh();
    ImGui::EndDisabled();
}

void BlenderApplication::applyFaceUVTransform(const Math::vec2& scale, f32 rotationDegrees,
                                              const Math::vec2& offset)
{
    if (!mMeshData)
        return;

    const usize beforeVertexCount = mMeshData->positions.size();
    recordUndo();

    if (!Assets().transformFaceUVs(*mMeshData, mSelection.selectedFaces(), scale, rotationDegrees,
                                   offset))
    {
        discardUndo();
        Log::warning("BlenderApplication: no UVs to transform");
        return;
    }

    const usize split = mMeshData->positions.size() - beforeVertexCount;
    if (split > 0)
        Log::info("BlenderApplication: retiled %u faces, %zu vertices split at the seam",
                  mSelection.selectedFaceCount(), split);
    else
        Log::info("BlenderApplication: retiled %u faces", mSelection.selectedFaceCount());

    applyMeshEdit();
}

void BlenderApplication::extrudeSelectedFaces()
{
    if (!mMeshData || mSelection.selectedFaceCount() == 0)
        return;

    recordUndo();

    const usize before = mMeshData->indices.size() / 3;
    std::vector<u32> raised;
    if (!Assets().extrudeFaces(*mMeshData, mSelection.selectedFaces(), mExtrudeDistance, &raised))
    {
        discardUndo();
        return;
    }

    // The index buffer was rebuilt, so the old face numbers mean nothing now.
    // Selecting what came out is also what lets a second Extrude carry on
    // from the first instead of acting on whatever inherited those numbers.
    mSelection.clearAll();
    for (usize i = 0; i < raised.size(); ++i)
        mSelection.selectFace(raised[i]);

    Assets().recalculateNormals(*mMeshData, mSmoothNormals, mAngleWeightedNormals);
    Log::info("BlenderApplication: extruded %zu faces by %.3f (%zu -> %zu triangles)",
              raised.size(), mExtrudeDistance, before, mMeshData->indices.size() / 3);
    applyMeshEdit();
}

::Radion::Math::vec3 BlenderApplication::transformPivot() const
{
    if (!mMeshData || mMeshData->positions.empty())
        return Math::vec3(0.0f);

    const std::vector<u32>& selected = mSelection.selectedVertices();
    const std::vector<Math::vec3>& positions = mMeshData->positions;

    Math::dvec3 sum(0.0);
    usize counted = 0;
    if (selected.empty())
    {
        for (usize i = 0; i < positions.size(); ++i)
            sum += Math::dvec3(positions[i]);
        counted = positions.size();
    }
    else
    {
        for (usize i = 0; i < selected.size(); ++i)
        {
            const usize index = static_cast<usize>(selected[i]);
            if (index >= positions.size())
                continue;
            sum += Math::dvec3(positions[index]);
            ++counted;
        }
    }

    if (counted == 0)
        return Math::vec3(0.0f);
    return Math::vec3(sum / static_cast<double>(counted));
}

bool BlenderApplication::beginGizmoDrag()
{
    if (mGizmoDragging)
        return true;
    if (!mMeshData || mMeshData->positions.empty())
        return false;

    recordUndo();

    mGizmoIndices = mSelection.selectedVertices();
    mGizmoPositions = mMeshData->positions;
    mGizmoNormals = mMeshData->normals;
    mGizmoTangents = mMeshData->tangents;
    // A mirrored drag reverses the winding, and the drag re-applies its whole
    // transform every frame: without the original indices to come back to,
    // crossing zero scale would flip the faces again on each one.
    mGizmoWinding = mMeshData->indices;

    mGizmoDragging = true;
    return true;
}

void BlenderApplication::updateGizmoDrag(const Math::mat4& worldDelta)
{
    if (!mGizmoDragging || !mMeshData)
        return;

    mMeshData->positions = mGizmoPositions;
    mMeshData->normals = mGizmoNormals;
    mMeshData->tangents = mGizmoTangents;
    mMeshData->indices = mGizmoWinding;

    // The gizmo's matrix already sits at the pivot, so the delta is applied
    // in world space rather than around the median a second time.
    Assets().transformVerticesAbout(*mMeshData, worldDelta, Math::vec3(0.0f), mGizmoIndices);
    applyMeshEdit();
}

void BlenderApplication::endGizmoDrag()
{
    if (!mGizmoDragging)
        return;

    mGizmoDragging = false;
    mGizmoIndices.clear();
    mGizmoIndices.shrink_to_fit();
    mGizmoPositions.clear();
    mGizmoPositions.shrink_to_fit();
    mGizmoNormals.clear();
    mGizmoNormals.shrink_to_fit();
    mGizmoTangents.clear();
    mGizmoTangents.shrink_to_fit();
    mGizmoWinding.clear();
    mGizmoWinding.shrink_to_fit();
}

void BlenderApplication::applyTransform(const Math::mat4& matrix, const char* verb)
{
    if (!mMeshData)
        return;

    recordUndo();
    Assets().transformVertices(*mMeshData, matrix, mSelection.selectedVertices());
    Log::info("BlenderApplication: %s %zu vertices", verb,
              mSelection.selectedVertexCount() > 0 ? mSelection.selectedVertexCount()
                                                   : mMeshData->positions.size());
    applyMeshEdit();
}

void BlenderApplication::newDocument()
{
    stop();

    if (mMeshData)
        mMeshData->clear();

    mSkeleton = Skeleton();
    mHasSkeleton = false;
    mAnimationClips.clear();
    mActiveClip = -1;
    mLocalPose.clear();
    mGlobalPose.clear();
    mBonePalette.clear();
    mCurrentFrame = 0;
    mPlaybackTimer = 0.0f;

    mSelection.clearAll();
    mSelectedSubmesh = -1;
    mSubmeshVisible.clear();

    // shrink_to_fit, not just clear: the whole point of discarding is to give
    // the memory back, and an undo stack over a large mesh is most of it.
    mUndoStates.clear();
    mUndoStates.shrink_to_fit();
    mRedoStates.clear();
    mRedoStates.shrink_to_fit();

    // Save writes to this path without asking. Leaving the old document's
    // path behind would make the first Save overwrite the file that was open
    // before, with an empty mesh.
    mSettings.general().lastOpenedMesh.clear();

    mDirty = false;
    mRenderer.invalidate();
    Log::info("BlenderApplication: new document");
}

void BlenderApplication::drawNewConfirmPopup()
{
    if (mNewConfirmRequested)
    {
        ImGui::OpenPopup("NewConfirmPopup");
        mNewConfirmRequested = false;
    }

    if (!ImGui::BeginPopupModal("NewConfirmPopup", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;

    ImGui::TextUnformatted("Discard the current mesh?");
    ImGui::TextDisabled("There are unsaved changes, and this cannot be undone.");
    ImGui::Separator();

    if (ImGui::Button("Discard", ImVec2(120.0f, 0.0f)))
    {
        newDocument();
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
        ImGui::CloseCurrentPopup();

    ImGui::EndPopup();
}

bool BlenderApplication::makeConvexHull()
{
    if (!mMeshData || mMeshData->positions.size() < 4)
        return false;

    MeshData hull;
    if (!Geometry::buildConvexHullMesh(mMeshData->positions, hull))
    {
        Log::error("BlenderApplication: convex hull failed");
        return false;
    }

    recordUndo();
    const usize beforeTriangles = mMeshData->indices.size() / 3;
    *mMeshData = std::move(hull);
    mSelection.clearAll();
    mSubmeshVisible.clear();
    mSelectedSubmesh = -1;

    Log::info("BlenderApplication: convex hull, %zu -> %zu triangles", beforeTriangles,
              mMeshData->indices.size() / 3);
    applyMeshEdit();
    return true;
}

void BlenderApplication::drawBisectPopup()
{
    if (mBisectPopupRequested)
    {
        ImGui::OpenPopup("BisectPopup");
        mBisectPopupRequested = false;
    }

    if (!ImGui::BeginPopup("BisectPopup"))
        return;

    ImGui::TextDisabled("Bisect");
    ImGui::Separator();

    ImGui::SetNextItemWidth(150.0f);
    ImGui::Combo("Axis", &mBisectAxis, "X\0Y\0Z\0");
    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragFloat("Offset", &mBisectOffset, 0.01f, -10000.0f, 10000.0f);
    ImGui::Checkbox("Keep positive side", &mBisectKeepPositive);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Which half survives: the one the axis points towards, or the other.");

    ImGui::Separator();
    if (ImGui::Button("Cut", ImVec2(-1.0f, 0.0f)))
    {
        bisectMesh();
        ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
}

bool BlenderApplication::bisectMesh()
{
    if (!mMeshData || mMeshData->positions.empty())
        return false;

    Math::vec3 normal(0.0f);
    normal[mBisectAxis] = 1.0f;

    MeshData cut;
    if (!clipMeshByPlane(*mMeshData, normal, mBisectOffset, mBisectKeepPositive, cut))
    {
        Log::warning("BlenderApplication: bisect left nothing - the plane misses the mesh, or "
                     "everything is on the discarded side");
        return false;
    }

    recordUndo();
    const usize beforeTriangles = mMeshData->indices.size() / 3;
    *mMeshData = std::move(cut);
    mSelection.clearAll();
    mSubmeshVisible.clear();

    Log::info("BlenderApplication: bisect, %zu -> %zu triangles", beforeTriangles,
              mMeshData->indices.size() / 3);
    applyMeshEdit();
    return true;
}

bool BlenderApplication::extractSelectedSubmesh()
{
    if (!mMeshData || mSelectedSubmesh < 0 ||
        static_cast<usize>(mSelectedSubmesh) >= mMeshData->submeshes.size())
        return false;

    MeshData extracted;
    if (!Assets().extractSubmesh(*mMeshData, static_cast<u32>(mSelectedSubmesh), extracted))
    {
        Log::error("BlenderApplication: could not extract submesh %d", mSelectedSubmesh);
        return false;
    }

    recordUndo();
    *mMeshData = std::move(extracted);
    mSelection.clearAll();
    mSubmeshVisible.clear();
    mSelectedSubmesh = -1;

    Log::info("BlenderApplication: extracted submesh (%zu vertices, %zu triangles)",
              mMeshData->positions.size(), mMeshData->indices.size() / 3);
    applyMeshEdit();
    return true;
}

void BlenderApplication::drawUnwrapPopup()
{
    if (mUnwrapPopupRequested)
    {
        ImGui::OpenPopup("UnwrapPopup");
        mUnwrapPopupRequested = false;
    }

    if (!ImGui::BeginPopup("UnwrapPopup"))
        return;

    ImGui::TextDisabled("Unwrap");
    ImGui::Separator();

    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragInt("Atlas resolution", &mUnwrapResolution, 16.0f, 0, 8192);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Zero is not 'no preference': it is the only value that guarantees "
                          "one page, sized to whatever the charts need. Anything else pins "
                          "the page size and lets the atlas spill onto several, which nothing "
                          "here reads.");

    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragInt("Padding", &mUnwrapPadding, 1.0f, 0, 64);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Texels left between charts. Too few and neighbours bleed into "
                          "each other when the texture is filtered.");

    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragFloat("Texels per unit", &mUnwrapTexelsPerUnit, 0.01f, 0.0f, 64.0f);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How much texture each world unit gets. The atlas grows with the "
                          "square of it. Zero lets xatlas pick.");

    ImGui::SetNextItemWidth(150.0f);
    ImGui::Combo("Write to", &mUnwrapTarget, "UV (texture)\0UV2 (lightmap)\0");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The unwrap always produces a second UV set. This says whether to "
                          "leave it there for a lightmap bake, or move it onto the ordinary "
                          "UVs, replacing them.");

    ImGui::Separator();
    ImGui::TextDisabled("Runs on this thread; a large mesh takes a while.");

    if (ImGui::Button("Unwrap", ImVec2(-1.0f, 0.0f)))
    {
        unwrapUVs();
        ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
}

bool BlenderApplication::unwrapUVs()
{
    if (!mMeshData || mMeshData->positions.empty())
        return false;

    LightmapUnwrapSettings settings;
    settings.resolution = static_cast<u32>(Math::max(mUnwrapResolution, 0));
    settings.padding = static_cast<u32>(Math::max(mUnwrapPadding, 0));
    settings.texelsPerUnit = Math::max(mUnwrapTexelsPerUnit, 0.0f);

    MeshData unwrapped;
    LightmapUnwrapResult result;
    LightmapUnwrapper unwrapper;
    if (!unwrapper.unwrap(*mMeshData, unwrapped, settings, &result))
    {
        Log::error("BlenderApplication: unwrap failed");
        return false;
    }

    recordUndo();

    if (mUnwrapTarget == 0)
        unwrapped.uvs = unwrapped.uvs2;

    const usize beforeVertexCount = mMeshData->positions.size();
    *mMeshData = std::move(unwrapped);

    // xatlas splits vertices at the seams, so the tangents no longer match
    // the UVs they were built from - and the ordinary UVs are what tangents
    // come from, so only the case that touched them needs redoing.
    if (mUnwrapTarget == 0 && !mMeshData->tangents.empty())
        Assets().recalculateTangents(*mMeshData);

    mSelection.clearAll();
    mSubmeshVisible.clear();

    Log::info("BlenderApplication: unwrapped into %ux%u, %u charts (%zu -> %zu vertices)",
              result.width, result.height, result.chartCount, beforeVertexCount,
              mMeshData->positions.size());

    applyMeshEdit();
    return true;
}

const char* BlenderApplication::primitiveName(PrimitiveType type)
{
    switch (type)
    {
    case PrimitiveType::Box: return "Cube";
    case PrimitiveType::Plane: return "Plane";
    case PrimitiveType::Sphere: return "Sphere";
    case PrimitiveType::Cylinder: return "Cylinder";
    case PrimitiveType::Cone: return "Cone";
    case PrimitiveType::Capsule: return "Capsule";
    case PrimitiveType::Torus: return "Torus";
    case PrimitiveType::Hills: return "Hills";
    }
    return "Primitive";
}

void BlenderApplication::drawAddMenu()
{
    if (!ImGui::BeginMenu("Add"))
        return;

    const BlenderApplication::PrimitiveType types[] = {
        PrimitiveType::Box,      PrimitiveType::Plane,   PrimitiveType::Sphere,
        PrimitiveType::Cylinder, PrimitiveType::Cone,    PrimitiveType::Capsule,
        PrimitiveType::Torus,    PrimitiveType::Hills,
    };

    for (u32 i = 0; i < static_cast<u32>(sizeof(types) / sizeof(types[0])); ++i)
    {
        if (ImGui::MenuItem(primitiveName(types[i])))
        {
            mPrimitiveType = types[i];
            mPrimitivePopupRequested = true;
        }
    }

    ImGui::EndMenu();
}

void BlenderApplication::drawPrimitivePopup()
{
    // OpenPopup() has to run outside the menu's own id stack, the same way
    // drawSaveInfoPopup() already handles it.
    if (mPrimitivePopupRequested)
    {
        ImGui::OpenPopup("PrimitivePopup");
        mPrimitivePopupRequested = false;
    }

    if (!ImGui::BeginPopup("PrimitivePopup"))
        return;

    ImGui::TextDisabled("%s", primitiveName(mPrimitiveType));
    ImGui::Separator();

    switch (mPrimitiveType)
    {
    case PrimitiveType::Box:
        ImGui::DragFloat3("Size", &mPrimitiveSize.x, 0.05f, 0.001f, 1000.0f);
        break;
    case PrimitiveType::Plane:
        ImGui::DragFloat("Width", &mPrimitiveSize.x, 0.05f, 0.001f, 1000.0f);
        ImGui::DragFloat("Depth", &mPrimitiveSize.z, 0.05f, 0.001f, 1000.0f);
        ImGui::DragInt("Segments X", &mPrimitiveSegmentsX, 1.0f, 1, 512);
        ImGui::DragInt("Segments Z", &mPrimitiveSegmentsZ, 1.0f, 1, 512);
        ImGui::DragFloat("UV Tiles", &mPrimitiveUvTiles, 0.05f, 0.001f, 128.0f);
        break;
    case PrimitiveType::Sphere:
        ImGui::DragFloat("Radius", &mPrimitiveRadius, 0.05f, 0.001f, 1000.0f);
        ImGui::DragInt("Rings", &mPrimitiveRings, 1.0f, 3, 256);
        ImGui::DragInt("Slices", &mPrimitiveSlices, 1.0f, 3, 256);
        break;
    case PrimitiveType::Cylinder:
    case PrimitiveType::Cone:
        ImGui::DragFloat("Radius", &mPrimitiveRadius, 0.05f, 0.001f, 1000.0f);
        ImGui::DragFloat("Height", &mPrimitiveHeight, 0.05f, 0.001f, 1000.0f);
        ImGui::DragInt("Slices", &mPrimitiveSlices, 1.0f, 3, 256);
        break;
    case PrimitiveType::Capsule:
        ImGui::DragFloat("Radius", &mPrimitiveRadius, 0.05f, 0.001f, 1000.0f);
        ImGui::DragFloat("Height", &mPrimitiveHeight, 0.05f, 0.001f, 1000.0f);
        ImGui::DragInt("Rings", &mPrimitiveRings, 1.0f, 3, 256);
        ImGui::DragInt("Slices", &mPrimitiveSlices, 1.0f, 3, 256);
        break;
    case PrimitiveType::Torus:
        ImGui::DragFloat("Radius", &mPrimitiveRadius, 0.05f, 0.001f, 1000.0f);
        ImGui::DragFloat("Tube", &mPrimitiveMinorRadius, 0.01f, 0.001f, 1000.0f);
        ImGui::DragInt("Segments", &mPrimitiveSlices, 1.0f, 3, 256);
        ImGui::DragInt("Sides", &mPrimitiveRings, 1.0f, 3, 256);
        break;
    case PrimitiveType::Hills:
        ImGui::DragFloat("Width", &mPrimitiveSize.x, 0.05f, 0.001f, 10000.0f);
        ImGui::DragFloat("Depth", &mPrimitiveSize.z, 0.05f, 0.001f, 10000.0f);
        ImGui::DragInt("Segments X", &mPrimitiveSegmentsX, 1.0f, 1, 1024);
        ImGui::DragInt("Segments Z", &mPrimitiveSegmentsZ, 1.0f, 1, 1024);
        ImGui::DragFloat("Height Scale", &mPrimitiveHeightScale, 0.05f, -1000.0f, 1000.0f);
        ImGui::DragFloat("UV Tiles", &mPrimitiveUvTiles, 0.05f, 0.001f, 128.0f);

        ImGui::Separator();
        if (mPrimitiveHeightmap.empty())
            ImGui::TextDisabled("No heightmap chosen");
        else
            ImGui::TextUnformatted(FileSystem::fileName(mPrimitiveHeightmap).c_str());
        if (!mPrimitiveHeightmap.empty() && ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", mPrimitiveHeightmap.c_str());

        if (ImGui::SmallButton("Heightmap..."))
            openFileDialog(ImGuiFileDialog::Mode::OpenFile, FileDialogHeightmap);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("An image whose red channel is read as height, 0 to 1, "
                              "multiplied by Height Scale.");
        break;
    }

    ImGui::Separator();

    // Hills is a plane displaced by an image; without one there is nothing to
    // displace it by, so both buttons stay off until it has been picked.
    const bool ready = mPrimitiveType != PrimitiveType::Hills || !mPrimitiveHeightmap.empty();
    ImGui::BeginDisabled(!ready);
    const bool hasMesh = mMeshData && !mMeshData->positions.empty();
    if (ImGui::Button("New Mesh", ImVec2(140.0f, 0.0f)))
    {
        if (createPrimitive(true))
            ImGui::CloseCurrentPopup();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Throws away what is loaded and starts from this shape.");

    ImGui::SameLine();
    ImGui::BeginDisabled(!hasMesh);
    if (ImGui::Button("Add to Mesh", ImVec2(140.0f, 0.0f)))
    {
        if (createPrimitive(false))
            ImGui::CloseCurrentPopup();
    }
    ImGui::EndDisabled();
    if (hasMesh && ImGui::IsItemHovered())
        ImGui::SetTooltip("Merges it in as its own submesh, keeping what is already there.");

    ImGui::EndDisabled();

    ImGui::EndPopup();
}

bool BlenderApplication::createPrimitive(bool replace)
{
    if (!mMeshData)
        return false;

    MeshDesc desc;
    switch (mPrimitiveType)
    {
    case PrimitiveType::Box:
        desc = MeshDesc::box(mPrimitiveSize);
        break;
    case PrimitiveType::Plane:
        desc = MeshDesc::plane(mPrimitiveSize.x, mPrimitiveSize.z,
                               static_cast<u32>(mPrimitiveSegmentsX),
                               static_cast<u32>(mPrimitiveSegmentsZ), mPrimitiveUvTiles);
        break;
    case PrimitiveType::Sphere:
        desc = MeshDesc::sphere(mPrimitiveRadius, static_cast<u32>(mPrimitiveRings),
                                static_cast<u32>(mPrimitiveSlices));
        break;
    case PrimitiveType::Cylinder:
        desc = MeshDesc::cylinder(mPrimitiveRadius, mPrimitiveHeight,
                                  static_cast<u32>(mPrimitiveSlices));
        break;
    case PrimitiveType::Cone:
        desc = MeshDesc::cone(mPrimitiveRadius, mPrimitiveHeight,
                              static_cast<u32>(mPrimitiveSlices));
        break;
    case PrimitiveType::Capsule:
        desc = MeshDesc::capsule(mPrimitiveRadius, mPrimitiveHeight,
                                 static_cast<u32>(mPrimitiveRings),
                                 static_cast<u32>(mPrimitiveSlices));
        break;
    case PrimitiveType::Torus:
        desc = MeshDesc::torus(mPrimitiveRadius, mPrimitiveMinorRadius,
                               static_cast<u32>(mPrimitiveSlices),
                               static_cast<u32>(mPrimitiveRings));
        break;
    case PrimitiveType::Hills:
        if (mPrimitiveHeightmap.empty())
            return false;
        desc = MeshDesc::hillsPlane(mPrimitiveSize.x, mPrimitiveSize.z,
                                    static_cast<u32>(mPrimitiveSegmentsX),
                                    static_cast<u32>(mPrimitiveSegmentsZ), mPrimitiveHeightmap,
                                    mPrimitiveHeightScale, mPrimitiveUvTiles);
        break;
    }

    MeshData built;
    if (!Assets().buildMeshData(desc, built))
    {
        Log::error("BlenderApplication: could not build a %s", primitiveName(mPrimitiveType));
        return false;
    }

    recordUndo();

    if (replace || mMeshData->positions.empty())
    {
        *mMeshData = std::move(built);
        mSelection.clearAll();
        mSubmeshVisible.clear();
        mSelectedSubmesh = -1;
    }
    else
    {
        MeshMergeInput current;
        current.mesh = mMeshData;
        current.sourceName = "current";
        MeshMergeInput incoming;
        incoming.mesh = &built;
        incoming.sourceName = primitiveName(mPrimitiveType);

        MeshData merged;
        std::string error;
        if (!Assets().mergeMeshes({current, incoming}, MeshMergeOptions(), merged, &error))
        {
            Log::error("BlenderApplication: could not add a %s: %s",
                       primitiveName(mPrimitiveType), error.c_str());
            discardUndo();
            return false;
        }

        *mMeshData = std::move(merged);
        mSelection.clearAll();
        mSubmeshVisible.clear();
    }

    Log::info("BlenderApplication: %s a %s (%zu vertices, %zu triangles)",
              replace ? "created" : "added", primitiveName(mPrimitiveType),
              mMeshData->positions.size(), mMeshData->indices.size() / 3);
    applyMeshEdit();
    return true;
}

void BlenderApplication::drawTransformMenu()
{
    ImGui::BeginDisabled(!mMeshData);

    // Which vertices this is about to move. Without it there is no way to
    // tell a selection edit from one that reshapes the whole model.
    if (mSelection.selectedVertexCount() > 0)
        ImGui::TextDisabled("%u selected vertices", mSelection.selectedVertexCount());
    else
        ImGui::TextDisabled("Whole mesh - nothing selected");
    ImGui::Separator();

    ImGui::SetNextItemWidth(150.0f);
    ImGui::SliderFloat("Scale Factor", &mScaleFactor, 0.1f, 10.0f);
    // No shortcut on these two: S and R put the interactive gizmo into scale
    // and rotate, and these apply a typed amount instead.
    if (ImGui::MenuItem("Scale"))
        applyTransform(Math::scale(Math::mat4(1.0f), Math::vec3(mScaleFactor)), "scaled");

    ImGui::SetNextItemWidth(150.0f);
    ImGui::Combo("Axis", &mRotationAxis, "X\0Y\0Z\0");
    ImGui::SetNextItemWidth(150.0f);
    ImGui::SliderFloat("Rotation Angle (deg)", &mRotationAngle, -180.0f, 180.0f);
    if (ImGui::MenuItem("Rotate"))
    {
        Math::vec3 axis(0.0f);
        axis[mRotationAxis] = 1.0f;
        applyTransform(Math::rotate(Math::mat4(1.0f), Math::radians(mRotationAngle), axis), "rotated");
    }

    ImGui::EndDisabled();
}

void BlenderApplication::drawMeshMenu()
{
    AssetManager& assets = Assets();

    ImGui::TextDisabled("Normals / Tangents");
    ImGui::Checkbox("Smooth", &mSmoothNormals);
    ImGui::SameLine();
    ImGui::Checkbox("Angle-weighted", &mAngleWeightedNormals);
    if (ImGui::MenuItem("Generate Normals"))
    {
        recordUndo();
        assets.recalculateNormals(*mMeshData, mSmoothNormals, mAngleWeightedNormals);
        applyMeshEdit();
    }
    if (ImGui::MenuItem("Generate Tangents"))
    {
        recordUndo();
        assets.recalculateTangents(*mMeshData);
        applyMeshEdit();
    }

    ImGui::Separator();
    ImGui::TextDisabled("UV");
    ImGui::SetNextItemWidth(150.0f);
    ImGui::Combo("##uvMode", &mUvMode, "Planar\0Cylindrical\0Spherical\0");
    if (mUvMode == 0)
    {
        ImGui::SetNextItemWidth(150.0f);
        ImGui::DragFloat("Resolution", &mUvResolutionU, 0.01f, 0.001f, 100.0f);
    }
    else
    {
        ImGui::SetNextItemWidth(150.0f);
        ImGui::DragFloat("U Tiles", &mUvResolutionU, 0.01f, 0.001f, 100.0f);
        ImGui::SetNextItemWidth(150.0f);
        ImGui::DragFloat("V Tiles", &mUvResolutionV, 0.01f, 0.001f, 100.0f);
    }
    // Projection is the cheap answer; this is the one that gives charts that
    // do not overlap.
    if (ImGui::MenuItem("Unwrap (xatlas)..."))
        mUnwrapPopupRequested = true;

    ImGui::Separator();
    ImGui::TextDisabled("Shape");
    if (ImGui::MenuItem("Center"))
    {
        recordUndo();
        assets.center(*mMeshData);
        applyMeshEdit();
    }
    if (ImGui::MenuItem("Center on Ground"))
    {
        recordUndo();
        assets.centerOnGround(*mMeshData);
        applyMeshEdit();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Centres on X and Z and drops the lowest point to y = 0, which is "
                          "what a prop that stands on the floor wants.");

    if (ImGui::MenuItem("Bisect..."))
        mBisectPopupRequested = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Cuts by a plane and keeps one side, splitting the triangles that "
                          "cross it. Keeps the UVs, unlike a CSG cut.");

    if (ImGui::MenuItem("Convex Hull"))
        makeConvexHull();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Replaces the mesh with the smallest convex shape containing it - "
                          "a collision proxy, not something to keep modelling.");

    ImGui::BeginDisabled(!mMeshData || mSelectedSubmesh < 0);
    if (ImGui::MenuItem("Extract Selected Submesh"))
        extractSelectedSubmesh();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Keeps only the submesh picked in Properties, re-indexed to stand "
                          "on its own.");
    ImGui::Separator();

    if (ImGui::MenuItem("Generate UV"))
    {
        recordUndo();
        if (mUvMode == 0)
            assets.makePlanarUV(*mMeshData, mUvResolutionU);
        else if (mUvMode == 1)
            assets.makeCylindricalUV(*mMeshData, mUvResolutionU, mUvResolutionV);
        else
            assets.makeSphericalUV(*mMeshData, mUvResolutionU, mUvResolutionV);
        applyMeshEdit();
    }

    ImGui::Separator();
    ImGui::TextDisabled("Submesh Structure");
    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragInt("Split Target (tris)", &mSplitTargetTriangles, 50.0f, 100, 100000);
    if (ImGui::MenuItem("Split Submeshes"))
    {
        recordUndo();
        assets.splitSubMeshes(*mMeshData, static_cast<u32>(mSplitTargetTriangles));
        applyMeshEdit();
    }
    if (ImGui::MenuItem("Join Submeshes"))
    {
        recordUndo();
        assets.mergeSubmeshes(*mMeshData, false);
        applyMeshEdit();
    }

    ImGui::Separator();
    ImGui::TextDisabled("Optimize");
    if (ImGui::MenuItem("Weld Vertices"))
    {
        recordUndo();
        const usize before = mMeshData->positions.size();
        const u32 removed = assets.weldVertices(*mMeshData);
        applyMeshEdit();
        mMeshToolStatus = "Weld: " + std::to_string(before) + " -> " +
                          std::to_string(mMeshData->positions.size()) + " verts (" +
                          std::to_string(removed) + " removed).";
    }
    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragFloat("Overdraw Threshold", &mOverdrawThreshold, 0.005f, 1.0f, 3.0f, "%.3f");
    if (ImGui::MenuItem("Optimize All"))
    {
        recordUndo();
        const usize beforeVerts = mMeshData->positions.size();
        const u32 removed = assets.weldVertices(*mMeshData);
        assets.optimizeVertexCache(*mMeshData);
        assets.optimizeOverdraw(*mMeshData, mOverdrawThreshold);
        assets.optimizeVertexFetch(*mMeshData);
        applyMeshEdit();
        mMeshToolStatus = "Optimize: " + std::to_string(beforeVerts) + " -> " +
                          std::to_string(mMeshData->positions.size()) + " verts (" +
                          std::to_string(removed) + " welded), indices reordered.";
    }
    ImGui::SetNextItemWidth(150.0f);
    ImGui::SliderFloat("Simplify Ratio", &mSimplifyRatio, 0.05f, 1.0f, "%.2f");
    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragFloat("Simplify Error", &mSimplifyError, 0.001f, 0.0001f, 0.5f, "%.4f");
    if (ImGui::MenuItem("Simplify"))
    {
        recordUndo();
        const usize beforeTris = mMeshData->indices.size() / 3;
        f32 reachedError = 0.0f;
        if (assets.simplifyMesh(*mMeshData, mSimplifyRatio, mSimplifyError, &reachedError))
        {
            assets.optimizeVertexFetch(*mMeshData);
            applyMeshEdit();
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Simplify: %zu -> %zu tris, error %.4f.", beforeTris,
                     mMeshData->indices.size() / 3, reachedError);
            mMeshToolStatus = buffer;
        }
        else
        {
            mUndoStates.pop_back();
            mMeshToolStatus = "Simplify failed: mesh has no editable geometry.";
        }
    }

    if (!mMeshToolStatus.empty())
    {
        ImGui::Separator();
        ImGui::TextWrapped("%s", mMeshToolStatus.c_str());
    }
}

void BlenderApplication::drawToolPopups()
{
    if (ImGui::BeginPopup("WeldPopup"))
    {
        ImGui::SliderFloat("Distance", &mWeldDistance, 0.0001f, 1.0f, "%.4f");
        if (ImGui::Button("Apply", ImVec2(-1.0f, 0.0f)))
        {
            recordUndo();
            const u32 removed = Assets().weldVertices(*mMeshData, mWeldDistance,
                                                       mSelection.selectedVertices());
            Log::info("BlenderApplication: weld removed %u vertices (distance %.4f)", removed,
                      mWeldDistance);
            mSelection.clearAll();
            applyMeshEdit();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (ImGui::BeginPopup("SmoothPopup"))
    {
        ImGui::SliderFloat("Strength", &mSmoothingStrength, 0.0f, 1.0f);
        if (ImGui::Button("Apply", ImVec2(-1.0f, 0.0f)))
        {
            recordUndo();
            Assets().smoothVertices(*mMeshData, mSmoothingStrength, 1, mSelection.selectedVertices());
            Log::info("BlenderApplication: smoothed %zu vertices (strength %.2f)",
                      mSelection.selectedVertexCount() > 0 ? mSelection.selectedVertexCount()
                                                            : mMeshData->positions.size(),
                      mSmoothingStrength);
            applyMeshEdit();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void BlenderApplication::drawSaveInfoPopup()
{
    // OpenPopup() runs here, outside the File menu's own window/ID stack -
    // called from inside a BeginMenu about to EndMenu() the same frame, it
    // never actually opened (see drawPreferencesPopup()'s own note).
    if (mSaveInfoRequested)
    {
        ImGui::OpenPopup("SaveInfoPopup");
        mSaveInfoRequested = false;
    }

    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("SaveInfoPopup", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;

    if (!mMeshData || mMeshData->positions.empty())
    {
        ImGui::TextUnformatted("No mesh loaded.");
    }
    else
    {
        ImGui::Text("Vertices: %zu", mMeshData->positions.size());
        ImGui::Text("Triangles: %zu", mMeshData->indices.size() / 3);
        ImGui::Text("Submeshes: %zu", mMeshData->submeshes.size());
        ImGui::Text("Materials: %zu", mMeshData->materials.size());
        ImGui::Text("Skinned: %s", mMeshData->skin.empty() ? "No" : "Yes");
        ImGui::Text("Source: %s", mSettings.general().lastOpenedMesh.empty()
                                      ? "(none)"
                                      : mSettings.general().lastOpenedMesh.c_str());
    }

    ImGui::Separator();
    if (ImGui::Button("Continue", ImVec2(120.0f, 0.0f)))
    {
        ImGui::CloseCurrentPopup();
        openFileDialog(ImGuiFileDialog::Mode::SaveFile, FileDialogSaveMesh);
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
        ImGui::CloseCurrentPopup();

    ImGui::EndPopup();
}

void BlenderApplication::drawOpenRecentMenu()
{
    const std::vector<std::string>& recentFiles = mSettings.general().recentFiles;
    if (!ImGui::BeginMenu("Open Recent", !recentFiles.empty()))
        return;

    // Snapshot the path before the click: loadMesh()/removeRecentFile() may
    // shuffle or shrink mSettings.general().recentFiles while this loop is
    // still walking it.
    std::string clicked;
    for (usize i = 0; i < recentFiles.size(); ++i)
    {
        const std::string& path = recentFiles[i];
        // ImGui takes an item's identity from its label, and two recent files
        // in different folders routinely share a base name - assets/city and
        // assets/city_bsxlm both hold a city.rmesh. Without an id of its own
        // per row, clicking the second opens the first.
        ImGui::PushID(static_cast<int>(i));
        if (ImGui::MenuItem(FileSystem::baseName(path).c_str()))
            clicked = path;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", path.c_str());
        ImGui::PopID();
    }

    ImGui::Separator();
    if (ImGui::MenuItem("Clear Recent"))
        mSettings.clearRecentFiles();

    ImGui::EndMenu();

    if (!clicked.empty() && !loadMesh(clicked))
        mSettings.removeRecentFile(clicked);
}

void BlenderApplication::openFileDialog(ImGuiFileDialog::Mode mode, FileDialogAction action)
{
    const std::string& lastDirectory = mSettings.general().lastOpenDirectory;
    const char* initialName = "";
    if (action == FileDialogSaveMesh)
        initialName = "mesh.rmesh";
    else if (action == FileDialogExportObj)
        initialName = "mesh.obj";

    mFileDialog.Open(mode,
                     lastDirectory.empty() ? std::filesystem::current_path()
                                           : std::filesystem::path(lastDirectory),
                     initialName);
    mFileDialogAction = action;
}

void BlenderApplication::drawFileDialog()
{
    if (mFileDialogAction == FileDialogNone)
        return;
    const std::filesystem::path root = std::filesystem::current_path();
    if (!mFileDialog.Render(root, root, root))
        return;
    const ImGuiFileDialog::Result result = mFileDialog.ConsumeResult();
    const FileDialogAction action = mFileDialogAction;
    mFileDialogAction = FileDialogNone;
    if (!result.accepted)
        return;

    mSettings.general().lastOpenDirectory = result.path.parent_path().string();
    if (action == FileDialogLoadMesh)
        loadMesh(result.path.string());
    else if (action == FileDialogImportMesh)
        importMesh(result.path.string());
    else if (action == FileDialogSaveMesh)
        saveAs(result.path.string());
    else if (action == FileDialogExportObj)
        exportObj(result.path.string());
    else if (action == FileDialogAppendAnimation)
        appendAnimation(result.path.string());
    else if (action == FileDialogHeightmap)
    {
        mPrimitiveHeightmap = result.path.string();
        // The popup closed when the dialog took over; bring it back with the
        // image now filled in rather than making the user find Add again.
        mPrimitivePopupRequested = true;
    }
}

void BlenderApplication::runFrame(f32 deltaTime)
{
    if (mPlaying)
    {
        mPlaybackTimer += deltaTime;
        const f32 frameDuration = 1.0f / kAnimationFramesPerSecond;
        if (mPlaybackTimer >= frameDuration)
        {
            mPlaybackTimer -= frameDuration;
            mCurrentFrame++;
            if (mCurrentFrame >= mTotalFrames)
            {
                if (mSettings.animation().autoLoop)
                    mCurrentFrame = 0;
                else
                    stop();
            }
            updateAnimationPose();
        }
    }
}

void BlenderApplication::drawStatusBar()
{
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + viewport->WorkSize.y - kStatusBarHeight));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, kStatusBarHeight));
    ImGui::SetNextWindowViewport(viewport->ID);

    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
                                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                                   ImGuiWindowFlags_NoSavedSettings |
                                   ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 4.0f));
    ImGui::Begin("BlenderStatusBar", nullptr, flags);
    ImGui::PopStyleVar(3);

    const f32 framerate = ImGui::GetIO().Framerate;
    ImGui::Text("%.0f FPS (%.2f ms)", framerate, framerate > 0.0f ? 1000.0f / framerate : 0.0f);

    if (mMeshData && !mMeshData->positions.empty())
    {
        ImGui::SameLine(0.0f, 24.0f);
        ImGui::Text("Verts %zu  Tris %zu", mMeshData->positions.size(), mMeshData->indices.size() / 3);

        ImGui::SameLine(0.0f, 24.0f);
        const char* modeName = "Vertex";
        switch (mSelection.mode())
        {
        case BlenderSelection::SelectionMode::Vertex: modeName = "Vertex"; break;
        case BlenderSelection::SelectionMode::Edge: modeName = "Edge"; break;
        case BlenderSelection::SelectionMode::Face: modeName = "Face"; break;
        }
        ImGui::Text("Select: %s", modeName);
    }

    ImGui::SameLine(0.0f, 24.0f);
    if (mDirty)
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "* Modified");
    else
        ImGui::TextDisabled("Saved");

    ImGui::End();
}

void BlenderApplication::drawPreferencesPopup()
{
    // "Preferences..." is the last item before Windows menu's EndMenu(),
    // which tears down that menu's popup-stack entry the same frame - an
    // OpenPopup() called from in there requested a popup the closing menu
    // then immediately cancelled, so it never actually opened (no dimmed
    // background, nothing). Deferred through a plain flag and opened here
    // instead, at the top level, outside any menu's own stack.
    if (mPreferencesRequested)
    {
        ImGui::OpenPopup("PreferencesPopup");
        mPreferencesRequested = false;
    }

    // Without an explicit position a freshly opened modal cascades from
    // wherever ImGui's window-position bookkeeping last left off - inside
    // this app's full-window DockSpace that can land it off to one side,
    // behind a docked panel, or clipped under the menu bar.
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("PreferencesPopup", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;

    BlenderSettings::ViewportSettings& viewport = mSettings.viewport();

    ImGui::TextDisabled("Viewport");
    ImGui::ColorEdit3("Background", &viewport.backgroundColor.x);
    ImGui::DragFloat("FOV", &viewport.fov, 0.5f, 10.0f, 120.0f);
    ImGui::DragFloat("Near Plane", &viewport.nearPlane, 0.01f, 0.001f, 10.0f);
    ImGui::DragFloat("Far Plane", &viewport.farPlane, 10.0f, 10.0f, 100000.0f);

    ImGui::Separator();
    ImGui::TextDisabled("Vertex / Face Colors");
    ImGui::ColorEdit3("Vertex", &viewport.vertexColor.x);
    ImGui::ColorEdit3("Selected Vertex", &viewport.selectedVertexColor.x);
    ImGui::DragFloat("Vertex Point Size", &mSettings.general().vertexPointSize, 0.1f, 1.0f, 20.0f);
    ImGui::ColorEdit3("Face Highlight", &viewport.faceHighlightColor.x);
    ImGui::SliderFloat("Face Highlight Alpha", &viewport.faceHighlightAlpha, 0.0f, 1.0f);
    ImGui::ColorEdit3("Face Edge Highlight", &viewport.faceEdgeHighlightColor.x);
    ImGui::ColorEdit3("Submesh Highlight", &viewport.submeshHighlightColor.x);
    ImGui::SliderFloat("Submesh Highlight Alpha", &viewport.submeshHighlightAlpha, 0.0f, 1.0f);
    ImGui::ColorEdit3("Box Select Rectangle", &viewport.boxSelectColor.x);
    ImGui::Checkbox("Color Submeshes Individually", &viewport.colorBySubmesh);
    ImGui::ColorEdit3("Normal Debug Vector", &viewport.normalVectorColor.x);
    ImGui::ColorEdit3("Tangent Debug Vector", &viewport.tangentVectorColor.x);
    ImGui::DragFloat("Debug Vector Length", &viewport.debugVectorLength, 0.01f, 0.01f, 5.0f);

    ImGui::Separator();
    if (ImGui::Button("Close", ImVec2(120.0f, 0.0f)))
    {
        mSettings.save(mSettingsPath);
        ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
}
