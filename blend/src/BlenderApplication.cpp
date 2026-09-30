#include "PCH.h"
#include "BlenderApplication.h"
#include "BlenderPanel.h"
#include "api/BlenderApiHost.h"
#include "BlenderTheme.h"
#include "Engine.h"
#include "FileSystem.h"
#include "HullMesh.h"
#include "LightmapUnwrapper.h"
#include "MeshClipper.h"
#include "Log.h"
#include "MaterialManager.h"
#include "mesh/MeshEdit.h"
#include "mesh/MeshPaint.h"
#include "GltfExporter.h"
#include "ObjExporter.h"
#include "panels/ConsolePanel.h"
#include "panels/HierarchyPanel.h"
#include "panels/MaterialsPanel.h"
#include "panels/UvEditorPanel.h"
#include "panels/MeshHealthPanel.h"
#include "panels/PropertiesPanel.h"
#include "panels/TimelinePanel.h"
#include "panels/ViewportPanel.h"

#include "Math.h"
#include <set>
#include <unordered_map>
#include <imgui.h>
#include <imgui_stdlib.h>
#include <imgui_internal.h>
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
    mApiPortField = mSettings.api().port;
    mApi = std::make_unique<BlenderApi::BlenderApiHost>(*this);
    buildPanels();
    mRenderer.initialize();
    mBatch.initialize();
}

BlenderApplication::~BlenderApplication()
{
    // A request still waiting on the frame loop must be released before anything it could touch goes away.
    mApi.reset();
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
        // Before anything draws: API commands change the document the frame then shows.
        mApi->pump();
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

bool BlenderApplication::startApi(const std::string& host, int port, const std::string& token,
                                  std::string* error)
{
    BlenderApi::ApiServerConfig config;
    config.host = host;
    config.port = port;
    config.token = token;

    std::string why;
    if (!mApi->start(config, &why))
    {
        Log::error("BlenderApplication: API not started: %s", why.c_str());
        mApiError = why;
        if (error)
            *error = why;
        return false;
    }
    mApiError.clear();
    // Not written to settings: a command-line port is for this run only; the Preferences' Start button saves one.
    mApiPortField = mApi->port();
    return true;
}

void BlenderApplication::stopApi()
{
    mApi->stop();
}

bool BlenderApplication::apiRunning() const
{
    return mApi->running();
}

int BlenderApplication::apiPort() const
{
    return mApi->port();
}

bool BlenderApplication::apiHasToken() const
{
    return mApi->hasToken();
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
    ++mMeshRevision;
    unhideAll();

    mSkeleton = Skeleton();
    mHasSkeleton = false;
    mAnimationClips.clear();
    mActiveClip = -1;
    mLocalPose.clear();
    mGlobalPose.clear();
    mBonePalette.clear();

    // A rig's skeleton usually lives in the same file as its mesh (FBX); try it before asking separately. No skin data
    // means no bones.
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

    // bindPose() sizes mLocalPose and fills rest offsets; sample() only overwrites bones with tracks, so this must run
    // first every time.
    // Otherwise the first pose leaves mLocalPose empty, evaluate() fails silently and MiniRenderer reads garbage for
    // joints > 0.
    mSkeleton.bindPose(mLocalPose);
    if (mActiveClip >= 0 && static_cast<usize>(mActiveClip) < mAnimationClips.size())
    {
        const f32 time = static_cast<f32>(mCurrentFrame) / kAnimationFramesPerSecond;
        mAnimationClips[static_cast<usize>(mActiveClip)].sample(time, mLocalPose);
    }
    mSkeleton.evaluate(mLocalPose, mGlobalPose, mBonePalette);
}

bool BlenderApplication::applyMeshEdit(bool positionsOnly)
{
    if (!mMeshData)
        return false;

    mRenderer.invalidate();
    if (!positionsOnly)
    {
        ++mMeshRevision;
        validateHidden();
    }
    markDirty();
    return true;
}

bool BlenderApplication::deleteSubmesh(u32 index)
{
    if (!mMeshData || index >= mMeshData->submeshes.size())
        return false;

    recordUndo();
    removeSubmeshData(index);
    applyMeshEdit();
    return true;
}

void BlenderApplication::removeSubmeshData(u32 index)
{
    const SubMesh removed = mMeshData->submeshes[index];
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

u32 BlenderApplication::snapSelectionToGrid(f32 step)
{
    if (!mMeshData || mMeshData->positions.empty() || !(step > 0.0f))
        return 0;

    std::vector<u32> vertices = editVertices();
    const bool whole = vertices.empty();
    const u32 count = whole ? static_cast<u32>(mMeshData->positions.size()) : static_cast<u32>(vertices.size());

    recordUndo();
    u32 moved = 0;
    for (u32 i = 0; i < count; ++i)
    {
        Math::vec3& p = mMeshData->positions[whole ? i : vertices[i]];
        const Math::vec3 snapped = Math::round(p / step) * step;
        if (snapped != p)
        {
            p = snapped;
            ++moved;
        }
    }

    if (moved == 0)
    {
        discardUndo();
        return 0;
    }
    Assets().computeBounds(*mMeshData);
    Assets().computeSubMeshBounds(*mMeshData);
    Log::info("BlenderApplication: snapped %u vertices to a %.4f grid", moved, step);
    applyMeshEdit();
    return moved;
}

u32 BlenderApplication::snapSelectionToVertices(f32 tolerance)
{
    if (!mMeshData || mMeshData->positions.empty() || !(tolerance > 0.0f))
        return 0;

    const std::vector<u32> selected = editVertices();
    if (selected.empty())
        return 0;

    const MeshTopology& topo = topology();
    const std::vector<Math::vec3>& positions = mMeshData->positions;

    std::vector<bool> isSelectedPoint(positions.size(), false);
    for (const u32 vertex : selected)
        isSelectedPoint[topo.canonical(vertex)] = true;

    // Targets: one per unselected point, found through a grid of tolerance-sized cells so a lookup only meets its
    // neighbours.
    struct Cell
    {
        s64 x, y, z;
        bool operator==(const Cell& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct CellHash
    {
        usize operator()(const Cell& c) const
        {
            return static_cast<usize>(static_cast<u64>(c.x) * 73856093ull ^ static_cast<u64>(c.y) * 19349663ull ^
                                      static_cast<u64>(c.z) * 83492791ull);
        }
    };
    auto cellOf = [tolerance](const Math::vec3& p)
    {
        return Cell{static_cast<s64>(std::floor(p.x / tolerance)), static_cast<s64>(std::floor(p.y / tolerance)),
                    static_cast<s64>(std::floor(p.z / tolerance))};
    };
    std::unordered_map<Cell, std::vector<u32>, CellHash> targets;
    for (u32 v = 0; v < positions.size(); ++v)
    {
        if (topo.canonical(v) == v && !isSelectedPoint[v])
            targets[cellOf(positions[v])].push_back(v);
    }

    // Decide every move before making any, so one snap cannot change where the next finds its target.
    std::unordered_map<u32, Math::vec3> destination;
    for (const u32 vertex : selected)
    {
        const u32 point = topo.canonical(vertex);
        if (destination.count(point))
            continue;

        const Math::vec3& from = positions[point];
        const Cell home = cellOf(from);
        f32 bestDistance = tolerance;
        bool found = false;
        Math::vec3 best(0.0f);
        for (s64 dx = -1; dx <= 1; ++dx)
            for (s64 dy = -1; dy <= 1; ++dy)
                for (s64 dz = -1; dz <= 1; ++dz)
                {
                    const auto cell = targets.find({home.x + dx, home.y + dy, home.z + dz});
                    if (cell == targets.end())
                        continue;
                    for (const u32 candidate : cell->second)
                    {
                        const f32 distance = Math::distance(positions[candidate], from);
                        if (distance <= bestDistance && positions[candidate] != from)
                        {
                            bestDistance = distance;
                            best = positions[candidate];
                            found = true;
                        }
                    }
                }
        if (found)
            destination[point] = best;
    }

    if (destination.empty())
        return 0;

    recordUndo();
    for (const u32 vertex : selected)
    {
        const auto it = destination.find(topo.canonical(vertex));
        if (it != destination.end())
            mMeshData->positions[vertex] = it->second;
    }
    Assets().computeBounds(*mMeshData);
    Assets().computeSubMeshBounds(*mMeshData);
    Log::info("BlenderApplication: snapped %zu points onto their nearest vertices", destination.size());
    applyMeshEdit();
    return static_cast<u32>(destination.size());
}

void BlenderApplication::setHiddenFaces(std::vector<u8>&& faces)
{
    bool any = false;
    for (const u8 hidden : faces)
    {
        if (hidden)
        {
            any = true;
            break;
        }
    }

    mHasHidden = any;
    mHiddenFaces = any ? std::move(faces) : std::vector<u8>();
    ++mHiddenRevision;
    if (mHasHidden)
        mRenderer.setHiddenFaces(mHiddenFaces.data(), static_cast<u32>(mHiddenFaces.size()));
    else
        mRenderer.setHiddenFaces(nullptr, 0);
}

void BlenderApplication::validateHidden()
{
    if (mHasHidden && mMeshData && mHiddenFaces.size() != mMeshData->indices.size() / 3)
    {
        Log::info("BlenderApplication: the edit changed the triangles, showing everything again");
        setHiddenFaces({});
    }
}

usize BlenderApplication::hiddenFaceCount() const
{
    usize count = 0;
    if (mHasHidden)
    {
        for (const u8 hidden : mHiddenFaces)
            count += hidden ? 1 : 0;
    }
    return count;
}

bool BlenderApplication::knifeCut(const Math::vec3& normal, f32 offset, std::string* error)
{
    if (!mMeshData || mMeshData->indices.empty())
    {
        if (error)
            *error = "the document has no mesh";
        return false;
    }

    recordUndo();
    std::vector<u64> cutEdges;
    std::string why;
    if (!MeshEdit::knife(*mMeshData, normal, offset, 1.0e-5f, &cutEdges, &why))
    {
        discardUndo();
        if (error)
            *error = why;
        return false;
    }

    mSelection.clearAll();
    mSelection.setMode(BlenderSelection::SelectionMode::Edge);
    mSelection.setEdges(cutEdges);
    Log::info("BlenderApplication: knife cut along a plane (%zu edges on the line)", cutEdges.size());
    applyMeshEdit();
    return true;
}

bool BlenderApplication::loopCutSelected(u32 cuts, std::string* error)
{
    if (!mMeshData || mSelection.selectedEdgeCount() == 0)
    {
        if (error)
            *error = "no edge is selected";
        return false;
    }

    recordUndo();
    std::vector<u64> created;
    std::string why;
    if (!MeshEdit::loopCut(*mMeshData, mSelection.selectedEdges().front(), cuts, &created, &why))
    {
        discardUndo();
        if (error)
            *error = why;
        return false;
    }

    mSelection.clearAll();
    mSelection.setMode(BlenderSelection::SelectionMode::Edge);
    mSelection.setEdges(created);
    Log::info("BlenderApplication: loop cut x%u (%zu new edges)", cuts, created.size());
    applyMeshEdit();
    return true;
}

bool BlenderApplication::insetSelection(f32 thickness, f32 depth, std::string* error)
{
    if (!mMeshData || mMeshData->indices.empty())
    {
        if (error)
            *error = "the document has no mesh";
        return false;
    }

    const std::vector<u32> faces = selectionFaces(false);
    if (faces.empty())
    {
        if (error)
            *error = "the selection does not contain a whole face";
        return false;
    }

    recordUndo();
    std::vector<u32> inner;
    std::string why;
    if (!MeshEdit::inset(*mMeshData, faces, thickness, depth, &inner, &why))
    {
        discardUndo();
        if (error)
            *error = why;
        return false;
    }

    // A raised or sunken region has walls that need normals of their own.
    if (depth != 0.0f && !mMeshData->normals.empty())
        Assets().recalculateNormals(*mMeshData, mSmoothNormals, mAngleWeightedNormals);

    mSelection.clearAll();
    mSelection.setMode(BlenderSelection::SelectionMode::Face);
    for (const u32 face : inner)
        mSelection.selectFace(face);
    Log::info("BlenderApplication: inset %zu faces (thickness %.3f, depth %.3f)", faces.size(), thickness, depth);
    applyMeshEdit();
    return true;
}

bool BlenderApplication::bevelSelectedEdges(f32 width, std::string* error)
{
    if (!mMeshData || mSelection.selectedEdgeCount() == 0)
    {
        if (error)
            *error = "no edges are selected";
        return false;
    }

    recordUndo();
    std::string why;
    if (!MeshEdit::bevel(*mMeshData, mSelection.selectedEdges(), width, &why))
    {
        discardUndo();
        if (error)
            *error = why;
        return false;
    }

    if (!mMeshData->normals.empty())
        Assets().recalculateNormals(*mMeshData, mSmoothNormals, mAngleWeightedNormals);
    const usize edges = mSelection.selectedEdgeCount();
    mSelection.clearAll();
    Log::info("BlenderApplication: bevelled %zu edges (width %.3f)", edges, width);
    applyMeshEdit();
    return true;
}

std::vector<u32> BlenderApplication::selectionFaces(bool partial)
{
    std::vector<u32> faces;
    if (!mMeshData || mMeshData->indices.empty())
        return faces;

    const usize faceCount = mMeshData->indices.size() / 3;
    const MeshTopology& topo = topology();

    std::vector<u8> chosen(faceCount, 0);
    for (const u32 face : mSelection.selectedFaces())
        if (face < faceCount)
            chosen[face] = 1;

    if (mSelection.selectedVertexCount() > 0)
    {
        std::vector<bool> point(mMeshData->positions.size(), false);
        for (const u32 vertex : mSelection.selectedVertices())
            if (vertex < point.size())
                point[topo.canonical(vertex)] = true;
        for (u32 face = 0; face < faceCount; ++face)
        {
            u32 selectedCorners = 0;
            for (u32 corner = 0; corner < 3; ++corner)
            {
                const u32 index = mMeshData->indices[face * 3 + corner];
                if (index < point.size() && point[topo.canonical(index)])
                    ++selectedCorners;
            }
            if (partial ? selectedCorners > 0 : selectedCorners == 3)
                chosen[face] = 1;
        }
    }

    if (mSelection.selectedEdgeCount() > 0)
    {
        if (partial)
        {
            for (const u64 key : mSelection.selectedEdges())
            {
                const s32 edge = topo.findEdge(static_cast<u32>(key >> 32), static_cast<u32>(key & 0xFFFFFFFFu));
                if (edge < 0)
                    continue;
                for (const u32 face : topo.edges()[static_cast<usize>(edge)].faces)
                    chosen[face] = 1;
            }
        }
        else
        {
            for (u32 face = 0; face < faceCount; ++face)
            {
                bool all = true;
                for (const s32 edge : topo.faceEdges(face))
                {
                    all = all && edge >= 0 &&
                          mSelection.isEdgeSelected(MeshTopology::edgeKey(topo.edges()[static_cast<usize>(edge)].a,
                                                                          topo.edges()[static_cast<usize>(edge)].b));
                }
                if (all)
                    chosen[face] = 1;
            }
        }
    }

    for (u32 face = 0; face < faceCount; ++face)
        if (chosen[face] && !isFaceHidden(face))
            faces.push_back(face);
    return faces;
}

bool BlenderApplication::subdivideSelection(u32 levels, bool smooth, std::string* error)
{
    if (!mMeshData || mMeshData->indices.empty())
    {
        if (error)
            *error = "the document has no mesh";
        return false;
    }

    const bool anySelected = mSelection.selectedVertexCount() > 0 || mSelection.selectedFaceCount() > 0 ||
                             mSelection.selectedEdgeCount() > 0;
    std::vector<u32> faces;
    if (anySelected)
    {
        faces = selectionFaces(false);
        if (faces.empty())
        {
            if (error)
                *error = "the selection does not contain a whole face";
            return false;
        }
    }
    else if (mHasHidden)
    {
        for (u32 face = 0; face < mMeshData->indices.size() / 3; ++face)
            if (!isFaceHidden(face))
                faces.push_back(face);
    }

    recordUndo();
    std::string why;
    if (!MeshEdit::subdivide(*mMeshData, faces, levels, smooth, &why))
    {
        discardUndo();
        if (error)
            *error = why;
        return false;
    }

    if (!mMeshData->normals.empty())
        Assets().recalculateNormals(*mMeshData, mSmoothNormals, mAngleWeightedNormals);
    mSelection.clearAll();
    Log::info("BlenderApplication: subdivided %s%zu faces x%u (%zu triangles now)", smooth ? "smooth " : "",
              faces.empty() ? mMeshData->indices.size() / 3 : faces.size(), levels, mMeshData->indices.size() / 3);
    applyMeshEdit();
    return true;
}

u32 BlenderApplication::turnSelectedEdges(std::string* error)
{
    if (!mMeshData || mSelection.selectedEdgeCount() == 0)
    {
        if (error)
            *error = "no edges are selected";
        return 0;
    }

    recordUndo();
    u32 turned = 0;
    std::string lastError;
    for (const u64 key : mSelection.selectedEdges())
    {
        std::string why;
        if (MeshEdit::turnEdge(*mMeshData, key, &why))
            ++turned;
        else
            lastError = why;
    }

    if (turned == 0)
    {
        discardUndo();
        if (error)
            *error = lastError;
        return 0;
    }
    // The turned edges no longer exist under those names.
    mSelection.clearAll();
    applyMeshEdit();
    return turned;
}

u32 BlenderApplication::splitSelectedEdges(f32 t, std::string* error)
{
    if (!mMeshData || mSelection.selectedEdgeCount() == 0)
    {
        if (error)
            *error = "no edges are selected";
        return 0;
    }

    std::vector<MeshEdit::EdgeSplit> splits;
    for (const u64 key : mSelection.selectedEdges())
        splits.push_back({key, t});

    recordUndo();
    MeshEdit::RefineResult result;
    std::string why;
    if (!MeshEdit::refineEdges(*mMeshData, splits, &result, &why))
    {
        discardUndo();
        if (error)
            *error = why;
        return 0;
    }

    // Select the new vertices so the cut can be moved or extruded straight away.
    mSelection.clearAll();
    mSelection.setMode(BlenderSelection::SelectionMode::Vertex);
    for (const MeshEdit::RefineResult::Midpoint& m : result.midpoints)
        mSelection.selectVertex(m.vertex);
    applyMeshEdit();
    return static_cast<u32>(splits.size());
}

u32 BlenderApplication::collapseSelectedEdges(f32 t, std::string* error)
{
    if (!mMeshData || mSelection.selectedEdgeCount() == 0)
    {
        if (error)
            *error = "no edges are selected";
        return 0;
    }

    recordUndo();
    u32 collapsed = 0;
    std::string lastError;
    for (const u64 key : mSelection.selectedEdges())
    {
        std::string why;
        if (MeshEdit::collapseEdge(*mMeshData, key, t, &why))
            ++collapsed;
        else
            lastError = why; // an earlier collapse may have taken this edge with it
    }

    if (collapsed == 0)
    {
        discardUndo();
        if (error)
            *error = lastError;
        return 0;
    }
    mSelection.clearAll();
    applyMeshEdit();
    return collapsed;
}

bool BlenderApplication::hideSelected()
{
    if (!mMeshData || mMeshData->indices.empty())
        return false;

    const usize faceCount = mMeshData->indices.size() / 3;
    const MeshTopology& topo = topology();

    std::vector<u8> hidden = mHasHidden && mHiddenFaces.size() == faceCount ? mHiddenFaces
                                                                           : std::vector<u8>(faceCount, 0);
    bool changed = false;
    auto hide = [&](u32 face)
    {
        if (face < faceCount && !hidden[face])
        {
            hidden[face] = 1;
            changed = true;
        }
    };

    for (const u32 face : mSelection.selectedFaces())
        hide(face);

    // Vertices and edges hide every triangle that uses them: a triangle cannot
    // stay behind with a corner missing.
    if (mSelection.selectedVertexCount() > 0 || mSelection.selectedEdgeCount() > 0)
    {
        std::vector<bool> point(mMeshData->positions.size(), false);
        for (const u32 vertex : mSelection.selectedVertices())
            if (vertex < point.size())
                point[topo.canonical(vertex)] = true;
        for (u32 face = 0; face < faceCount; ++face)
        {
            for (u32 corner = 0; corner < 3; ++corner)
            {
                const u32 index = mMeshData->indices[face * 3 + corner];
                if (index < point.size() && point[topo.canonical(index)])
                {
                    hide(face);
                    break;
                }
            }
        }
        for (const u64 key : mSelection.selectedEdges())
        {
            const s32 edge = topo.findEdge(static_cast<u32>(key >> 32), static_cast<u32>(key & 0xFFFFFFFFu));
            if (edge < 0)
                continue;
            for (const u32 face : topo.edges()[static_cast<usize>(edge)].faces)
                hide(face);
        }
    }

    if (!changed)
        return false;
    mSelection.clearAll();
    setHiddenFaces(std::move(hidden));
    return true;
}

bool BlenderApplication::hideUnselected()
{
    if (!mMeshData || mMeshData->indices.empty())
        return false;

    const usize faceCount = mMeshData->indices.size() / 3;
    const MeshTopology& topo = topology();

    // Which triangles count as selected depends on what is being selected: a
    // face itself; a triangle whose three corners are; a triangle whose three
    // edges are.
    std::vector<u8> keep(faceCount, 0);
    switch (mSelection.mode())
    {
    case BlenderSelection::SelectionMode::Face:
        for (const u32 face : mSelection.selectedFaces())
            if (face < faceCount)
                keep[face] = 1;
        break;
    case BlenderSelection::SelectionMode::Vertex:
    {
        std::vector<bool> point(mMeshData->positions.size(), false);
        for (const u32 vertex : mSelection.selectedVertices())
            if (vertex < point.size())
                point[topo.canonical(vertex)] = true;
        for (u32 face = 0; face < faceCount; ++face)
        {
            bool all = true;
            for (u32 corner = 0; corner < 3; ++corner)
            {
                const u32 index = mMeshData->indices[face * 3 + corner];
                all = all && index < point.size() && point[topo.canonical(index)];
            }
            keep[face] = all ? 1 : 0;
        }
        break;
    }
    case BlenderSelection::SelectionMode::Edge:
        for (u32 face = 0; face < faceCount; ++face)
        {
            bool all = true;
            for (const s32 edge : topo.faceEdges(face))
            {
                all = all && edge >= 0 &&
                      mSelection.isEdgeSelected(MeshTopology::edgeKey(topo.edges()[static_cast<usize>(edge)].a,
                                                                      topo.edges()[static_cast<usize>(edge)].b));
            }
            keep[face] = all ? 1 : 0;
        }
        break;
    }

    std::vector<u8> hidden(faceCount, 0);
    bool changed = false;
    for (u32 face = 0; face < faceCount; ++face)
    {
        const bool alreadyHidden = isFaceHidden(face);
        hidden[face] = (!keep[face] || alreadyHidden) ? 1 : 0;
        changed = changed || (hidden[face] && !alreadyHidden);
    }
    if (!changed)
        return false;
    mSelection.clearAll();
    setHiddenFaces(std::move(hidden));
    return true;
}

void BlenderApplication::unhideAll()
{
    if (mHasHidden || !mHiddenFaces.empty())
        setHiddenFaces({});
}

const std::vector<u8>& BlenderApplication::hiddenVertexFlags()
{
    if (mHiddenVerticesRevision == mHiddenRevision && mHiddenVertices.size() == (mMeshData ? mMeshData->positions.size() : 0))
        return mHiddenVertices;

    mHiddenVertices.clear();
    mHiddenVerticesRevision = mHiddenRevision;
    if (!mHasHidden || !mMeshData || mHiddenFaces.size() != mMeshData->indices.size() / 3)
        return mHiddenVertices;

    const usize vertexCount = mMeshData->positions.size();
    std::vector<u8> used(vertexCount, 0);
    std::vector<u8> shown(vertexCount, 0);
    for (usize face = 0; face < mHiddenFaces.size(); ++face)
    {
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 index = mMeshData->indices[face * 3 + corner];
            if (index >= vertexCount)
                continue;
            used[index] = 1;
            if (!mHiddenFaces[face])
                shown[index] = 1;
        }
    }
    mHiddenVertices.assign(vertexCount, 0);
    for (usize v = 0; v < vertexCount; ++v)
        mHiddenVertices[v] = (used[v] && !shown[v]) ? 1 : 0;
    return mHiddenVertices;
}

void BlenderApplication::deleteSelectedEdges()
{
    if (!mMeshData || mSelection.selectedEdgeCount() == 0)
        return;

    // An edge cannot be removed and leave the triangles beside it, so deleting one takes them with it (as Blender's
    // "Edges" delete).
    const MeshTopology& topo = topology();
    std::set<u32> faces;
    for (const u64 key : mSelection.selectedEdges())
    {
        const s32 edge = topo.findEdge(static_cast<u32>(key >> 32), static_cast<u32>(key & 0xFFFFFFFFu));
        if (edge < 0)
            continue;
        for (const u32 face : topo.edges()[static_cast<usize>(edge)].faces)
            faces.insert(face);
    }
    if (faces.empty())
        return;

    const usize edgeCount = mSelection.selectedEdgeCount();
    recordUndo();
    Assets().deleteFaces(*mMeshData, std::vector<u32>(faces.begin(), faces.end()));
    mSelection.clearAll();
    mSelectedSubmesh = -1;
    Log::info("BlenderApplication: deleted %zu edges (%zu faces)", edgeCount, faces.size());
    applyMeshEdit();
}

void BlenderApplication::groupSelectedFacesIntoSubmesh()
{
    if (!mMeshData || mSelection.selectedFaceCount() == 0)
        return;
    separateSelectedFaces();
}

bool BlenderApplication::separateSelectedFaces(s32* newPart, std::string* error)
{
    if (!mMeshData || mMeshData->indices.empty())
    {
        if (error)
            *error = "the document has no mesh";
        return false;
    }

    const std::vector<u32> faces = selectionFaces(false);
    if (faces.empty())
    {
        if (error)
            *error = "the selection does not contain a whole face";
        return false;
    }

    recordUndo();
    if (!Assets().groupFacesIntoSubmesh(*mMeshData, faces))
    {
        discardUndo();
        if (error)
            *error = "those faces already make up a whole part";
        return false;
    }

    mSelection.clearAll();
    mSelectedSubmesh = static_cast<s32>(mMeshData->submeshes.size()) - 1;
    if (newPart)
        *newPart = mSelectedSubmesh;
    Log::info("BlenderApplication: grouped %zu faces into submesh %d", faces.size(), mSelectedSubmesh);
    applyMeshEdit();
    return true;
}

u32 BlenderApplication::fillHoles(u32 maxEdges, std::string* error)
{
    if (!mMeshData || mMeshData->indices.empty())
    {
        if (error)
            *error = "the document has no mesh";
        return 0;
    }

    recordUndo();
    u32 filled = 0;
    std::string why;
    if (!MeshEdit::fillHoles(*mMeshData, mSelection.selectedEdges(), maxEdges, &filled, &why))
    {
        discardUndo();
        if (error)
            *error = why;
        return 0;
    }

    if (!mMeshData->normals.empty())
        Assets().recalculateNormals(*mMeshData, mSmoothNormals, mAngleWeightedNormals);
    mSelection.clearAll();
    Log::info("BlenderApplication: filled %u open borders", filled);
    applyMeshEdit();
    return filled;
}

bool BlenderApplication::bridgeBorders(std::string* error)
{
    if (!mMeshData || mMeshData->indices.empty())
    {
        if (error)
            *error = "the document has no mesh";
        return false;
    }

    recordUndo();
    std::string why;
    if (!MeshEdit::bridge(*mMeshData, mSelection.selectedEdges(), &why))
    {
        discardUndo();
        if (error)
            *error = why;
        return false;
    }

    if (!mMeshData->normals.empty())
        Assets().recalculateNormals(*mMeshData, mSmoothNormals, mAngleWeightedNormals);
    mSelection.clearAll();
    Log::info("BlenderApplication: bridged two borders");
    applyMeshEdit();
    return true;
}

bool BlenderApplication::mirrorGeometry(s32 axis, f32 offset, f32 weld, std::string* error)
{
    if (!mMeshData || mMeshData->indices.empty())
    {
        if (error)
            *error = "the document has no mesh";
        return false;
    }

    const bool anySelected = mSelection.selectedVertexCount() > 0 || mSelection.selectedFaceCount() > 0 ||
                             mSelection.selectedEdgeCount() > 0;
    std::vector<u32> faces;
    if (anySelected)
    {
        faces = selectionFaces(false);
        if (faces.empty())
        {
            if (error)
                *error = "the selection does not contain a whole face";
            return false;
        }
    }

    recordUndo();
    std::string why;
    if (!MeshEdit::mirror(*mMeshData, axis, offset, weld, faces, &why))
    {
        discardUndo();
        if (error)
            *error = why;
        return false;
    }

    mSelection.clearAll();
    Log::info("BlenderApplication: mirrored %s across %c = %.3f", faces.empty() ? "the mesh" : "the selection",
              "xyz"[axis], offset);
    applyMeshEdit();
    return true;
}

bool BlenderApplication::booleanParts(MeshEdit::BooleanOp op, u32 partA, u32 partB, u32 resolution,
                                      const std::string& name, s32* resultPart, std::string* error)
{
    if (!mMeshData || partA >= mMeshData->submeshes.size() || partB >= mMeshData->submeshes.size() ||
        partA == partB)
    {
        if (error)
            *error = "give two different parts of the mesh";
        return false;
    }

    MeshData a;
    MeshData b;
    if (!Assets().extractSubmesh(*mMeshData, partA, a) || !Assets().extractSubmesh(*mMeshData, partB, b))
    {
        if (error)
            *error = "could not read the two parts";
        return false;
    }

    MeshData combined;
    std::string why;
    if (!MeshEdit::booleanMeshes(a, b, op, resolution, combined, &why))
    {
        if (error)
            *error = why;
        return false;
    }

    // The result takes the first part's look and name.
    const SubMesh& first = mMeshData->submeshes[partA];
    Material material;
    if (first.materialSlot < mMeshData->materials.size())
        material = mMeshData->materials[first.materialSlot];
    if (!name.empty())
        material.name = name;
    combined.materials.assign(1, material);

    recordUndo();
    // Higher index first, so the lower one is still where it was.
    removeSubmeshData(std::max(partA, partB));
    removeSubmeshData(std::min(partA, partB));
    // Nothing references the old triangles' vertices any more.
    if (mMeshData->indices.empty())
    {
        mMeshData->clear();
    }
    else
    {
        Assets().compactGeometry(*mMeshData);
    }

    s32 index = -1;
    PartStyle keep;
    if (!appendPart(std::move(combined), Math::mat4(1.0f), keep, "Boolean", false, &index, false))
    {
        discardUndo();
        if (error)
            *error = "the combined shape could not be added";
        return false;
    }

    mSelection.clearAll();
    if (resultPart)
        *resultPart = index;
    Log::info("BlenderApplication: combined two parts (%zu triangles now)", mMeshData->indices.size() / 3);
    return true;
}

bool BlenderApplication::mergeParts(const std::vector<u32>& parts, std::string* error)
{
    if (!mMeshData || mMeshData->submeshes.empty())
    {
        if (error)
            *error = "the document has no parts";
        return false;
    }

    std::vector<u32> sorted = parts;
    std::sort(sorted.begin(), sorted.end());
    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());

    recordUndo();
    std::string why;
    if (!MeshEdit::mergeSubmeshes(*mMeshData, sorted, &why))
    {
        discardUndo();
        if (error)
            *error = why;
        return false;
    }

    // The viewport's per-part visibility and the picked part follow the renumbering.
    const u32 keep = sorted.front();
    for (usize k = sorted.size(); k-- > 1;)
        if (sorted[k] < mSubmeshVisible.size())
            mSubmeshVisible.erase(mSubmeshVisible.begin() + sorted[k]);
    if (keep < mSubmeshVisible.size())
        mSubmeshVisible[keep] = true;
    mSelectedSubmesh = -1;
    mSelection.clearAll();
    Log::info("BlenderApplication: joined %zu parts", sorted.size());
    applyMeshEdit();
    return true;
}

void BlenderApplication::setSymmetry(s32 axis, f32 offset)
{
    mSymmetryAxis = axis >= 0 && axis <= 2 ? axis : -1;
    mSymmetryOffset = offset;
}

std::vector<u32> BlenderApplication::symmetryPartners(const std::vector<u32>& vertices) const
{
    std::vector<u32> partners;
    if (!mMeshData || mSymmetryAxis < 0 || vertices.empty())
        return partners;

    const std::vector<Math::vec3>& positions = mMeshData->positions;
    const s32 axis = mSymmetryAxis;
    constexpr f32 kTolerance = 1.0e-4f;
    auto mirrored = [&](Math::vec3 p)
    {
        p[axis] = 2.0f * mSymmetryOffset - p[axis];
        return p;
    };

    // Every vertex by the cell it stands in, so a mirrored point finds what is at it without walking the mesh.
    struct Cell
    {
        s64 x, y, z;
        bool operator==(const Cell& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct CellHash
    {
        usize operator()(const Cell& c) const
        {
            return static_cast<usize>(static_cast<u64>(c.x) * 73856093ull ^ static_cast<u64>(c.y) * 19349663ull ^
                                      static_cast<u64>(c.z) * 83492791ull);
        }
    };
    auto cellOf = [](const Math::vec3& p)
    {
        return Cell{static_cast<s64>(std::floor(p.x / 1.0e-3f)), static_cast<s64>(std::floor(p.y / 1.0e-3f)),
                    static_cast<s64>(std::floor(p.z / 1.0e-3f))};
    };
    std::unordered_map<Cell, std::vector<u32>, CellHash> grid;
    grid.reserve(positions.size());
    for (u32 v = 0; v < positions.size(); ++v)
        grid[cellOf(positions[v])].push_back(v);

    std::vector<bool> inSet(positions.size(), false);
    for (const u32 v : vertices)
        if (v < inSet.size())
            inSet[v] = true;

    std::vector<bool> taken(positions.size(), false);
    for (const u32 v : vertices)
    {
        if (v >= positions.size())
            continue;
        // A vertex on the plane is its own partner: nothing separate to move.
        if (std::abs(positions[v][axis] - mSymmetryOffset) <= kTolerance)
            continue;

        const Math::vec3 target = mirrored(positions[v]);
        const Cell home = cellOf(target);
        for (s64 dx = -1; dx <= 1; ++dx)
            for (s64 dy = -1; dy <= 1; ++dy)
                for (s64 dz = -1; dz <= 1; ++dz)
                {
                    const auto found = grid.find({home.x + dx, home.y + dy, home.z + dz});
                    if (found == grid.end())
                        continue;
                    for (const u32 candidate : found->second)
                    {
                        if (inSet[candidate] || taken[candidate])
                            continue;
                        if (Math::distance(positions[candidate], target) <= kTolerance)
                        {
                            taken[candidate] = true;
                            partners.push_back(candidate);
                        }
                    }
                }
    }
    return partners;
}

void BlenderApplication::transformVerticesWorld(const Math::mat4& world, const std::vector<u32>& vertices)
{
    if (!mMeshData)
        return;

    // The partners must be found before anything moves.
    const std::vector<u32> partners = symmetryPartners(vertices);
    Assets().transformVerticesAbout(*mMeshData, world, Math::vec3(0.0f), vertices);
    if (!partners.empty())
    {
        Math::vec3 flip(1.0f);
        flip[mSymmetryAxis] = -1.0f;
        Math::mat4 reflect = Math::translate(Math::mat4(1.0f), Math::vec3(0.0f));
        Math::vec3 shift(0.0f);
        shift[mSymmetryAxis] = 2.0f * mSymmetryOffset;
        reflect = Math::translate(Math::mat4(1.0f), shift) * Math::scale(Math::mat4(1.0f), flip);
        // M * W * M, with M its own inverse: what the partner must do to mirror the move.
        Assets().transformVerticesAbout(*mMeshData, reflect * world * reflect, Math::vec3(0.0f), partners);
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

// A step is a whole mesh copy, so the stack's cost depends on the mesh, not the edit count: the budget is in bytes and
// the oldest steps go first.
// One step always survives, so an edit on a large model stays recoverable.
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
    ++mMeshRevision;
    validateHidden();
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
    ++mMeshRevision;
    validateHidden();
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
    // Nothing records a keyframe yet: a snapshot and dirty mark would copy the whole mesh and ask to save work that does
    // not exist.
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

    // saveMesh() always writes .rmesh whatever the extension; forcing it keeps Save on an imported .obj/.fbx from
    // overwriting the original with .rmesh bytes.
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

bool BlenderApplication::exportGltf(const std::string& path, std::string* error,
                                    std::vector<std::string>* warnings)
{
    if (!mMeshData || mMeshData->positions.empty())
    {
        if (error)
            *error = "the document has no mesh";
        return false;
    }

    std::string why;
    std::vector<std::string> localWarnings;
    std::vector<std::string>& notes = warnings ? *warnings : localWarnings;
    if (!GltfExporter::save(*mMeshData, path, &why, &notes))
    {
        Log::error("BlenderApplication: failed to export glTF '%s': %s", path.c_str(), why.c_str());
        if (error)
            *error = why;
        return false;
    }

    for (const std::string& note : notes)
        Log::warning("BlenderApplication: glTF export: %s", note.c_str());
    if (mHasSkeleton)
        Log::warning("BlenderApplication: glTF export is static geometry - the skeleton and "
                     "animations were not written");
    Log::info("BlenderApplication: exported glTF '%s'", path.c_str());
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
    mPanels.push_back(new UvEditorPanel(*this));
    mPanels.push_back(new ConsolePanel(*this));
}

// Same shape as EditorApplication::drawDockspace(): the split is built once; after a redock ImGui's .ini takes over and
// DockBuilderGetNode() stops seeing an empty node.
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

            // Big centre viewport, Properties over Hierarchy on the right edge, Timeline/mesh-edit/console as tabs in
            // the bottom strip.
            ImGuiID center, right, centerTop, bottom, propertiesTop, hierarchyBottom, viewportArea, uvArea;
            ImGui::DockBuilderSplitNode(dockspaceId, ImGuiDir_Right, 0.22f, &right, &center);
            ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.28f, &bottom, &centerTop);
            ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.35f, &hierarchyBottom, &propertiesTop);
            // The UV editor beside the viewport: editing UVs means watching both.
            ImGui::DockBuilderSplitNode(centerTop, ImGuiDir_Right, 0.36f, &uvArea, &viewportArea);

            ImGui::DockBuilderDockWindow("Viewport", viewportArea);
            ImGui::DockBuilderDockWindow("UV Editor", uvArea);
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
            if (ImGui::MenuItem("glTF Binary (.glb)..."))
                openFileDialog(ImGuiFileDialog::Mode::SaveFile, FileDialogExportGltf);
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

    // Outside the block below: Add creates the mesh, so gating it on one existing locks the tool shut after File > New.
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
    if (ImGui::BeginMenu("Paint"))
    {
        drawPaintMenu();
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
    ImGui::Separator();
    if (ImGui::MenuItem("Hide Selected", "H"))
        hideSelected();
    if (ImGui::MenuItem("Hide Unselected", "Shift+H"))
        hideUnselected();
    if (ImGui::MenuItem("Reveal Hidden", "Alt+H", false, mHasHidden))
        unhideAll();
    ImGui::Separator();
    if (ImGui::MenuItem("Select Linked", "L"))
        selectLinked();
    ImGui::EndDisabled();

    if (mMeshData && mMeshData->submeshes.size() > 1 &&
        ImGui::BeginMenu("Select Submesh"))
    {
        for (u32 i = 0; i < static_cast<u32>(mMeshData->submeshes.size()); ++i)
        {
            const u32 slot = mMeshData->submeshes[i].materialSlot;
            const bool named = slot < mMeshData->materials.size() &&
                               !mMeshData->materials[slot].name.empty();
            const std::string label = named ? mMeshData->materials[slot].name
                                            : ("Submesh " + std::to_string(i));
            // A hidden submesh is unreachable by every other route; offering it here would be a dead click.
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

    if (mSelection.mode() == BlenderSelection::SelectionMode::Edge)
    {
        const MeshTopology& topo = topology();
        std::set<u32> ends;
        for (const u64 key : mSelection.selectedEdges())
        {
            ends.insert(static_cast<u32>(key >> 32));
            ends.insert(static_cast<u32>(key & 0xFFFFFFFFu));
        }
        std::vector<u64> grown = mSelection.selectedEdges();
        for (const MeshTopology::Edge& edge : topo.edges())
        {
            if (ends.count(edge.a) || ends.count(edge.b))
                grown.push_back(MeshTopology::edgeKey(edge.a, edge.b));
        }
        mSelection.setEdges(grown);
        return;
    }

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

    if (mSelection.mode() == BlenderSelection::SelectionMode::Edge)
    {
        const MeshTopology& topo = topology();
        std::unordered_map<u32, std::vector<u64>> byVertex;
        for (const MeshTopology::Edge& edge : topo.edges())
        {
            const u64 key = MeshTopology::edgeKey(edge.a, edge.b);
            byVertex[edge.a].push_back(key);
            byVertex[edge.b].push_back(key);
        }

        std::set<u64> reached(mSelection.selectedEdges().begin(), mSelection.selectedEdges().end());
        std::vector<u64> frontier(reached.begin(), reached.end());
        while (!frontier.empty())
        {
            const u64 key = frontier.back();
            frontier.pop_back();
            for (const u32 end : {static_cast<u32>(key >> 32), static_cast<u32>(key & 0xFFFFFFFFu)})
            {
                for (const u64 other : byVertex[end])
                {
                    if (reached.insert(other).second)
                        frontier.push_back(other);
                }
            }
        }
        mSelection.setEdges(std::vector<u64>(reached.begin(), reached.end()));
        return;
    }

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

    // No submeshes and nothing hidden: nothing to hide behind, the mesh is one piece.
    const bool everythingVisible = mesh.submeshes.empty() && !mHasHidden;
    faceSelectable.assign(faceCount, everythingVisible);
    vertexSelectable.assign(vertexCount, everythingVisible);
    if (everythingVisible)
        return;

    // A mesh without submeshes is one range covering every index.
    std::vector<std::pair<u64, u64>> ranges;
    if (mesh.submeshes.empty())
    {
        ranges.emplace_back(0, mesh.indices.size());
    }
    else
    {
        for (u32 s = 0; s < static_cast<u32>(mesh.submeshes.size()); ++s)
        {
            if (!isSubmeshVisible(s))
                continue;
            const SubMesh& submesh = mesh.submeshes[s];
            ranges.emplace_back(submesh.indexOffset,
                                static_cast<u64>(submesh.indexOffset) + submesh.indexCount);
        }
    }

    for (const auto& range : ranges)
    {
        for (u64 i = range.first; i + 2 < range.second && i + 2 < mesh.indices.size(); i += 3)
        {
            const usize face = static_cast<usize>(i / 3);
            if (face >= faceCount || isFaceHidden(static_cast<u32>(face)))
                continue;
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
    if (!mMeshData || (mMeshData->submeshes.empty() && !mHasHidden))
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

    // An edge stays reachable while any triangle on it does.
    if (mSelection.selectedEdgeCount() > 0)
    {
        const MeshTopology& topo = topology();
        const std::vector<u64> edges = mSelection.selectedEdges();
        for (const u64 key : edges)
        {
            const s32 edge = topo.findEdge(static_cast<u32>(key >> 32), static_cast<u32>(key & 0xFFFFFFFFu));
            bool reachable = false;
            if (edge >= 0)
            {
                for (const u32 face : topo.edges()[static_cast<usize>(edge)].faces)
                    reachable = reachable || (face < faceSelectable.size() && faceSelectable[face]);
            }
            if (!reachable)
                mSelection.deselectEdge(key);
        }
    }
}

void BlenderApplication::selectAllElements()
{
    if (!mMeshData)
        return;
    if (mSelection.mode() == BlenderSelection::SelectionMode::Edge)
    {
        mSelection.clearAll();
        mSelection.setEdges(allEdgeKeys());
        dropHiddenFromSelection();
        return;
    }
    mSelection.selectAll(static_cast<u32>(mMeshData->positions.size()),
                         static_cast<u32>(mMeshData->indices.size() / 3));
    dropHiddenFromSelection();
}

void BlenderApplication::invertElementSelection()
{
    if (!mMeshData)
        return;
    if (mSelection.mode() == BlenderSelection::SelectionMode::Edge)
    {
        std::vector<u64> inverted;
        for (const u64 key : allEdgeKeys())
        {
            if (!mSelection.isEdgeSelected(key))
                inverted.push_back(key);
        }
        mSelection.setEdges(inverted);
        dropHiddenFromSelection();
        return;
    }
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
    else
        deleteSelectedEdges();
}

// WantCaptureKeyboard keeps X from deleting the selection while the user types an X into a file name.
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

    if (ImGui::IsKeyPressed(ImGuiKey_H, false))
    {
        if (io.KeyAlt)
            unhideAll();
        else if (io.KeyShift)
            hideUnselected();
        else
            hideSelected();
    }

    if (ImGui::IsKeyPressed(ImGuiKey_E, false))
        extrudeFaces(mExtrudeDistance);

    if (ImGui::IsKeyPressed(ImGuiKey_L, false))
        selectLinked();
}

void BlenderApplication::drawVertexMenu()
{
    if (ImGui::MenuItem("Snap to Grid"))
        snapSelectionToGrid(mSettings.snap().moveStep);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Rounds the selected vertices to the Move step in Preferences > Snap.");
    if (ImGui::MenuItem("Snap to Nearest Vertex"))
        snapSelectionToVertices(mSnapTolerance);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Moves each selected vertex onto the closest unselected one within the "
                          "snap distance.");
    ImGui::SetNextItemWidth(120.0f);
    ImGui::DragFloat("Snap Distance", &mSnapTolerance, 0.001f, 0.0001f, 10.0f, "%.4f");
    ImGui::Separator();
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
    const bool anyEdge = mSelection.selectedEdgeCount() > 0;
    std::string why;
    auto report = [&why]()
    {
        if (!why.empty())
            Log::warning("BlenderApplication: %s", why.c_str());
        why.clear();
    };
    if (ImGui::MenuItem("Turn Edge", nullptr, false, anyEdge))
        turnSelectedEdges(&why);
    report();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Flips the diagonal between two triangles.");
    if (ImGui::MenuItem("Split Edge", nullptr, false, anyEdge))
        splitSelectedEdges(0.5f, &why);
    report();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Adds a vertex in the middle of each selected edge.");
    if (ImGui::MenuItem("Fill Hole", nullptr, false, anyEdge))
        fillHoles(256, &why);
    report();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Closes the open border the selected edge is on.");
    if (ImGui::MenuItem("Bridge", nullptr, false, anyEdge))
        bridgeBorders(&why);
    report();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Joins two open borders (select an edge on each) with a strip of triangles.");
    ImGui::Separator();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::SliderInt("Loops", &mLoopCuts, 1, 8);
    if (ImGui::MenuItem("Loop Cut", nullptr, false, anyEdge))
        loopCutSelected(static_cast<u32>(mLoopCuts), &why);
    report();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Cuts the ring of quads through the first selected edge.");
    ImGui::SetNextItemWidth(120.0f);
    ImGui::DragFloat("Bevel Width", &mBevelWidth, 0.005f, 0.001f, 10.0f, "%.3f");
    if (ImGui::MenuItem("Bevel", nullptr, false, anyEdge))
        bevelSelectedEdges(mBevelWidth, &why);
    report();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Chamfers the selected edges. Edges may not share a vertex.");
    ImGui::Separator();
    if (ImGui::MenuItem("Collapse Edge", nullptr, false, anyEdge))
        collapseSelectedEdges(0.5f, &why);
    report();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Merges the ends of each selected edge at its middle.");
    ImGui::Separator();
    if (ImGui::MenuItem("Delete Selected", "X", false, anyEdge))
        deleteSelectedEdges();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Removes the selected edges and the triangles on either side of them.");
}

void BlenderApplication::drawFaceMenu()
{
    ImGui::SetNextItemWidth(150.0f);
    ImGui::SliderFloat("Extrude Distance", &mExtrudeDistance, -10.0f, 10.0f);
    ImGui::BeginDisabled(!mMeshData || mSelection.selectedFaceCount() == 0);
    if (ImGui::MenuItem("Extrude", "E"))
        extrudeFaces(mExtrudeDistance);
    ImGui::EndDisabled();

    ImGui::Separator();
    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragFloat("Inset Thickness", &mInsetThickness, 0.005f, 0.0f, 10.0f, "%.3f");
    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragFloat("Inset Depth", &mInsetDepth, 0.005f, -10.0f, 10.0f, "%.3f");
    ImGui::BeginDisabled(!mMeshData || mSelection.selectedFaceCount() == 0);
    if (ImGui::MenuItem("Inset"))
    {
        std::string why;
        if (!insetSelection(mInsetThickness, mInsetDepth, &why))
            Log::warning("BlenderApplication: %s", why.c_str());
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Shrinks the selected faces away from their border, leaving a ring of "
                          "triangles; Depth then raises or sinks the inner part.");
    ImGui::Separator();

    ImGui::BeginDisabled(!mMeshData);
    if (ImGui::MenuItem("Flip Normals"))
    {
        recordUndo();
        // Reverse winding and normals together, or the surface lights as though it never turned (as
        // AssetManager::scale() does for a negative factor).
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

const MeshTopology& BlenderApplication::topology()
{
    if (mTopologyRevision != mMeshRevision)
    {
        if (mMeshData)
            mTopology.build(*mMeshData);
        else
            mTopology = MeshTopology();
        mTopologyRevision = mMeshRevision;
    }
    return mTopology;
}

std::vector<u32> BlenderApplication::editVertices()
{
    std::vector<u32> vertices;
    if (!mMeshData)
        return vertices;

    const MeshTopology& topo = topology();
    const u32 vertexCount = static_cast<u32>(mMeshData->positions.size());

    std::vector<u32> seeds = mSelection.selectedVertices();
    for (const u32 face : mSelection.selectedFaces())
    {
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const usize at = static_cast<usize>(face) * 3 + corner;
            if (at < mMeshData->indices.size())
                seeds.push_back(mMeshData->indices[at]);
        }
    }
    for (const u64 key : mSelection.selectedEdges())
    {
        seeds.push_back(static_cast<u32>(key >> 32));
        seeds.push_back(static_cast<u32>(key & 0xFFFFFFFFu));
    }
    if (seeds.empty())
        return vertices;

    // Widen every seed to its canonical point, then collect everything at a selected point in one pass.
    std::vector<bool> selectedPoint(vertexCount, false);
    for (const u32 seed : seeds)
    {
        if (seed < vertexCount)
            selectedPoint[topo.canonical(seed)] = true;
    }
    for (u32 v = 0; v < vertexCount; ++v)
    {
        if (selectedPoint[topo.canonical(v)])
            vertices.push_back(v);
    }
    return vertices;
}

std::vector<u64> BlenderApplication::allEdgeKeys()
{
    std::vector<u64> keys;
    const MeshTopology& topo = topology();
    keys.reserve(topo.edges().size());
    for (const MeshTopology::Edge& edge : topo.edges())
        keys.push_back(MeshTopology::edgeKey(edge.a, edge.b));
    std::sort(keys.begin(), keys.end());
    return keys;
}

bool BlenderApplication::extrudeFaces(f32 distance)
{
    if (!mMeshData || mSelection.selectedFaceCount() == 0)
        return false;

    recordUndo();

    const usize before = mMeshData->indices.size() / 3;
    std::vector<u32> raised;
    if (!Assets().extrudeFaces(*mMeshData, mSelection.selectedFaces(), distance, &raised))
    {
        discardUndo();
        return false;
    }

    // The index buffer was rebuilt so old face numbers mean nothing; selecting the result lets a second Extrude continue
    // from the first.
    mSelection.clearAll();
    for (usize i = 0; i < raised.size(); ++i)
        mSelection.selectFace(raised[i]);

    Assets().recalculateNormals(*mMeshData, mSmoothNormals, mAngleWeightedNormals);
    Log::info("BlenderApplication: extruded %zu faces by %.3f (%zu -> %zu triangles)",
              raised.size(), distance, before, mMeshData->indices.size() / 3);
    applyMeshEdit();
    return true;
}

Math::vec3 BlenderApplication::transformPivot()
{
    if (!mMeshData || mMeshData->positions.empty())
        return Math::vec3(0.0f);

    const std::vector<u32> selected = editVertices();
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

    mGizmoIndices = editVertices();
    mGizmoPartners = symmetryPartners(mGizmoIndices);
    mGizmoPositions = mMeshData->positions;
    mGizmoNormals = mMeshData->normals;
    mGizmoTangents = mMeshData->tangents;
    // A mirrored drag reverses winding and re-applies its whole transform every frame; without the original indices,
    // crossing zero scale would flip faces each time.
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

    // The gizmo's matrix already sits at the pivot, so apply the delta in world space, not around the median again.
    Assets().transformVerticesAbout(*mMeshData, worldDelta, Math::vec3(0.0f), mGizmoIndices);
    if (!mGizmoPartners.empty())
    {
        Math::vec3 flip(1.0f);
        flip[mSymmetryAxis] = -1.0f;
        Math::vec3 shift(0.0f);
        shift[mSymmetryAxis] = 2.0f * mSymmetryOffset;
        const Math::mat4 reflect = Math::translate(Math::mat4(1.0f), shift) * Math::scale(Math::mat4(1.0f), flip);
        Assets().transformVerticesAbout(*mMeshData, reflect * worldDelta * reflect, Math::vec3(0.0f), mGizmoPartners);
    }
    applyMeshEdit(true);
}

void BlenderApplication::endGizmoDrag()
{
    if (!mGizmoDragging)
        return;

    mGizmoDragging = false;
    ++mMeshRevision;
    mGizmoIndices.clear();
    mGizmoIndices.shrink_to_fit();
    mGizmoPartners.clear();
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
    const std::vector<u32> vertices = editVertices();
    if (mSymmetryAxis >= 0 && !vertices.empty())
    {
        Math::dvec3 sum(0.0);
        for (const u32 v : vertices)
            sum += Math::dvec3(mMeshData->positions[v]);
        const Math::vec3 pivot = Math::vec3(sum / static_cast<double>(vertices.size()));
        transformVerticesWorld(Math::translate(Math::mat4(1.0f), pivot) * matrix * Math::translate(Math::mat4(1.0f), -pivot),
                               vertices);
    }
    else
    {
        Assets().transformVertices(*mMeshData, matrix, vertices);
    }
    Log::info("BlenderApplication: %s %zu vertices", verb,
              vertices.empty() ? mMeshData->positions.size() : vertices.size());
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

    // shrink_to_fit, not just clear: an undo stack over a large mesh is most of the memory.
    mUndoStates.clear();
    mUndoStates.shrink_to_fit();
    mRedoStates.clear();
    mRedoStates.shrink_to_fit();

    // Save writes to this path without asking; a stale path would overwrite the previously open file with an empty mesh.
    mSettings.general().lastOpenedMesh.clear();

    mDirty = false;
    mRenderer.invalidate();
    ++mMeshRevision;
    unhideAll();
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
    return bisectMesh(mBisectAxis, mBisectOffset, mBisectKeepPositive);
}

bool BlenderApplication::bisectMesh(s32 axis, f32 offset, bool keepPositive)
{
    if (!mMeshData || mMeshData->positions.empty() || axis < 0 || axis > 2)
        return false;

    Math::vec3 normal(0.0f);
    normal[axis] = 1.0f;

    MeshData cut;
    if (!clipMeshByPlane(*mMeshData, normal, offset, keepPositive, cut))
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
    UnwrapParams params;
    params.resolution = static_cast<u32>(Math::max(mUnwrapResolution, 0));
    params.padding = static_cast<u32>(Math::max(mUnwrapPadding, 0));
    params.texelsPerUnit = Math::max(mUnwrapTexelsPerUnit, 0.0f);
    params.target = mUnwrapTarget;
    return unwrapUVs(params);
}

bool BlenderApplication::unwrapUVs(const UnwrapParams& params)
{
    if (!mMeshData || mMeshData->positions.empty())
        return false;

    LightmapUnwrapSettings settings;
    settings.resolution = params.resolution;
    settings.padding = params.padding;
    settings.texelsPerUnit = params.texelsPerUnit;

    MeshData unwrapped;
    LightmapUnwrapResult result;
    LightmapUnwrapper unwrapper;
    if (!unwrapper.unwrap(*mMeshData, unwrapped, settings, &result))
    {
        Log::error("BlenderApplication: unwrap failed");
        return false;
    }

    recordUndo();

    if (params.target == 0)
        unwrapped.uvs = unwrapped.uvs2;

    const usize beforeVertexCount = mMeshData->positions.size();
    *mMeshData = std::move(unwrapped);

    // xatlas splits vertices at seams, so tangents no longer match the UVs; only the case that touched ordinary UVs
    // needs redoing.
    if (params.target == 0 && !mMeshData->tangents.empty())
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
    // OpenPopup() has to run outside the menu's id stack, as drawSaveInfoPopup() does.
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

    // Hills is a plane displaced by an image; both buttons stay off until one is picked.
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

BlenderApplication::PrimitiveParams BlenderApplication::primitiveParamsFromUi() const
{
    PrimitiveParams params;
    params.type = mPrimitiveType;
    params.size = mPrimitiveSize;
    params.radius = mPrimitiveRadius;
    params.minorRadius = mPrimitiveMinorRadius;
    params.height = mPrimitiveHeight;
    params.rings = mPrimitiveRings;
    params.slices = mPrimitiveSlices;
    params.segmentsX = mPrimitiveSegmentsX;
    params.segmentsZ = mPrimitiveSegmentsZ;
    params.uvTiles = mPrimitiveUvTiles;
    params.heightScale = mPrimitiveHeightScale;
    params.heightmap = mPrimitiveHeightmap;
    return params;
}

bool BlenderApplication::primitiveTypeFromName(const std::string& name, PrimitiveType& out)
{
    static const struct
    {
        const char* name;
        PrimitiveType type;
    } kNames[] = {
        {"box", PrimitiveType::Box},         {"cube", PrimitiveType::Box},
        {"plane", PrimitiveType::Plane},     {"sphere", PrimitiveType::Sphere},
        {"cylinder", PrimitiveType::Cylinder}, {"cone", PrimitiveType::Cone},
        {"capsule", PrimitiveType::Capsule}, {"torus", PrimitiveType::Torus},
        {"hills", PrimitiveType::Hills},
    };
    for (const auto& entry : kNames)
    {
        if (name == entry.name)
        {
            out = entry.type;
            return true;
        }
    }
    return false;
}

namespace
{
// Plane/Hills take their extent from size.x/size.z, Box from all three, as the Add popup presents them.
bool describePrimitive(const BlenderApplication::PrimitiveParams& p, MeshDesc& desc)
{
    using Type = BlenderApplication::PrimitiveType;
    switch (p.type)
    {
    case Type::Box:
        desc = MeshDesc::box(p.size);
        return true;
    case Type::Plane:
        desc = MeshDesc::plane(p.size.x, p.size.z, static_cast<u32>(p.segmentsX),
                               static_cast<u32>(p.segmentsZ), p.uvTiles);
        return true;
    case Type::Sphere:
        desc = MeshDesc::sphere(p.radius, static_cast<u32>(p.rings), static_cast<u32>(p.slices));
        return true;
    case Type::Cylinder:
        desc = MeshDesc::cylinder(p.radius, p.height, static_cast<u32>(p.slices));
        return true;
    case Type::Cone:
        desc = MeshDesc::cone(p.radius, p.height, static_cast<u32>(p.slices));
        return true;
    case Type::Capsule:
        desc = MeshDesc::capsule(p.radius, p.height, static_cast<u32>(p.rings),
                                 static_cast<u32>(p.slices));
        return true;
    case Type::Torus:
        desc = MeshDesc::torus(p.radius, p.minorRadius, static_cast<u32>(p.slices),
                               static_cast<u32>(p.rings));
        return true;
    case Type::Hills:
        if (p.heightmap.empty())
            return false;
        desc = MeshDesc::hillsPlane(p.size.x, p.size.z, static_cast<u32>(p.segmentsX),
                                    static_cast<u32>(p.segmentsZ), p.heightmap, p.heightScale,
                                    p.uvTiles);
        return true;
    }
    return false;
}

void applyPartStyle(MeshData& part, const BlenderApplication::PartStyle& style)
{
    if (part.materials.empty())
        part.materials.push_back(Material());
    for (SubMesh& submesh : part.submeshes)
    {
        if (submesh.materialSlot >= part.materials.size())
            submesh.materialSlot = 0;
    }

    for (usize i = 0; i < part.materials.size(); ++i)
    {
        Material& material = part.materials[i];
        if (!style.name.empty())
            material.name = style.name;
        if (style.hasColor)
            material.params.baseColor = style.color;
        if (style.hasRoughness)
            material.params.surface.x = style.roughness;
        if (style.hasMetallic)
            material.params.surface.y = style.metallic;
        material.paramsDirty = true;
    }
}
} // namespace

bool BlenderApplication::createPrimitive(bool replace)
{
    if (!mMeshData)
        return false;

    MeshDesc desc;
    if (!describePrimitive(primitiveParamsFromUi(), desc))
        return false;

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

bool BlenderApplication::buildPrimitive(const PrimitiveParams& params, MeshData& out)
{
    MeshDesc desc;
    if (!describePrimitive(params, desc))
        return false;
    if (!Assets().buildMeshData(desc, out))
    {
        Log::error("BlenderApplication: could not build a %s", primitiveName(params.type));
        return false;
    }
    return true;
}

bool BlenderApplication::createPrimitive(const PrimitiveParams& params, const Math::mat4& placement,
                                         const PartStyle& style, bool replace, s32* submeshOut)
{
    MeshData built;
    if (!buildPrimitive(params, built))
        return false;
    return appendPart(std::move(built), placement, style, primitiveName(params.type), replace,
                      submeshOut);
}

bool BlenderApplication::appendPart(MeshData part, const Math::mat4& placement,
                                    const PartStyle& style, const char* sourceName, bool replace,
                                    s32* submeshOut, bool undoStep)
{
    if (!mMeshData || part.positions.empty() || part.indices.empty())
        return false;

    if (part.submeshes.empty())
    {
        SubMesh whole;
        whole.indexCount = static_cast<u32>(part.indices.size());
        part.submeshes.push_back(whole);
    }
    applyPartStyle(part, style);
    Assets().transform(part, placement);

    if (undoStep)
        recordUndo();

    if (replace || mMeshData->positions.empty())
    {
        *mMeshData = std::move(part);
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
        incoming.mesh = &part;
        incoming.sourceName = sourceName;

        // Every part stays its own submesh even when two share a material, so each can be moved, restyled or deleted on
        // its own.
        MeshMergeOptions options;
        options.preserveSubmeshBoundaries = true;

        MeshData merged;
        std::string error;
        if (!Assets().mergeMeshes({current, incoming}, options, merged, &error))
        {
            Log::error("BlenderApplication: could not add a %s: %s", sourceName, error.c_str());
            if (undoStep)
                discardUndo();
            return false;
        }

        *mMeshData = std::move(merged);
        // The old vertices keep their numbers, so the selection and submesh visibility stay valid.
        Assets().computeBounds(*mMeshData);
        Assets().computeSubMeshBounds(*mMeshData);
    }

    if (submeshOut)
        *submeshOut = static_cast<s32>(mMeshData->submeshes.size()) - 1;
    Log::info("BlenderApplication: added part '%s' (%zu vertices, %zu triangles)",
              style.name.empty() ? sourceName : style.name.c_str(), mMeshData->positions.size(),
              mMeshData->indices.size() / 3);
    applyMeshEdit();
    return true;
}

std::vector<u32> BlenderApplication::submeshVertices(u32 index) const
{
    std::vector<u32> vertices;
    if (!mMeshData || index >= mMeshData->submeshes.size())
        return vertices;

    const SubMesh& submesh = mMeshData->submeshes[index];
    const u64 end = Math::min<u64>(static_cast<u64>(submesh.indexOffset) + submesh.indexCount,
                                  mMeshData->indices.size());
    for (u64 i = submesh.indexOffset; i < end; ++i)
        vertices.push_back(mMeshData->indices[static_cast<usize>(i)]);

    std::sort(vertices.begin(), vertices.end());
    vertices.erase(std::unique(vertices.begin(), vertices.end()), vertices.end());
    return vertices;
}

bool BlenderApplication::transformSubmesh(u32 index, const Math::mat4& matrix, const Math::vec3& pivot)
{
    const std::vector<u32> vertices = submeshVertices(index);
    if (vertices.empty())
        return false;

    recordUndo();
    Assets().transformVerticesAbout(*mMeshData, matrix, pivot, vertices);
    // transformVertices leaves winding alone; a mirrored part would be left inside out.
    if (Math::determinant(Math::mat3(matrix)) < 0.0f)
        Assets().flipWinding(*mMeshData, index);
    Assets().computeSubMeshBounds(*mMeshData);
    applyMeshEdit();
    return true;
}

bool BlenderApplication::duplicateSubmesh(u32 index, const Math::mat4& placement, s32* newIndex)
{
    if (!mMeshData || index >= mMeshData->submeshes.size())
        return false;

    MeshData copy;
    if (!Assets().extractSubmesh(*mMeshData, index, copy))
        return false;

    PartStyle keep;
    return appendPart(std::move(copy), placement, keep, "duplicate", false, newIndex);
}

std::vector<u32> BlenderApplication::selectedTriangles()
{
    std::vector<u32> triangles;
    if (!mMeshData)
        return triangles;
    if (mSelection.selectedFaceCount() > 0)
        return mSelection.selectedFaces();

    const std::vector<u32> vertices = editVertices();
    if (vertices.empty())
        return triangles;
    std::vector<bool> chosen(mMeshData->positions.size(), false);
    for (const u32 vertex : vertices)
        chosen[vertex] = true;
    const u32 count = static_cast<u32>(mMeshData->indices.size() / 3);
    for (u32 triangle = 0; triangle < count; ++triangle)
    {
        for (u32 corner = 0; corner < 3; ++corner)
        {
            if (chosen[mMeshData->indices[static_cast<usize>(triangle) * 3 + corner]])
            {
                triangles.push_back(triangle);
                break;
            }
        }
    }
    return triangles;
}

bool BlenderApplication::uvTargetVertices(UvTarget target, s32 part, std::vector<u32>& vertices, std::string* error)
{
    vertices.clear();
    auto fail = [error](const char* message)
    {
        if (error)
            *error = message;
        return false;
    };
    if (!mMeshData || mMeshData->positions.empty())
        return fail("the document has no mesh");

    switch (target)
    {
    case UvTarget::All:
        vertices.resize(mMeshData->positions.size());
        for (u32 i = 0; i < static_cast<u32>(vertices.size()); ++i)
            vertices[i] = i;
        return true;
    case UvTarget::Part:
        if (part < 0 || static_cast<usize>(part) >= mMeshData->submeshes.size())
            return fail("no such part");
        vertices = submeshVertices(static_cast<u32>(part));
        break;
    case UvTarget::Island:
    {
        const std::vector<u32> triangles = selectedTriangles();
        if (triangles.empty())
            return fail("nothing is selected");
        vertices = MeshUv::islandVertices(*mMeshData, triangles);
        break;
    }
    case UvTarget::Selection:
        if (mSelection.selectedFaceCount() > 0)
            vertices = MeshUv::verticesOfTriangles(*mMeshData, mSelection.selectedFaces());
        else
            vertices = editVertices();
        if (vertices.empty())
            return fail("nothing is selected");
        break;
    }
    if (vertices.empty())
        return fail("the target has no vertices");
    return true;
}

const std::vector<u8>* BlenderApplication::uvPinned()
{
    if (!mMeshData || mUvPinned.size() != mMeshData->positions.size())
    {
        mUvPinned.clear();
        return nullptr;
    }
    return &mUvPinned;
}

u32 BlenderApplication::uvPinnedCount()
{
    const std::vector<u8>* pinned = uvPinned();
    return pinned ? static_cast<u32>(std::count_if(pinned->begin(), pinned->end(), [](u8 v) { return v != 0; })) : 0;
}

void BlenderApplication::setUvPinned(const std::vector<u32>& vertices, bool pinned)
{
    if (!mMeshData)
        return;
    if (mUvPinned.size() != mMeshData->positions.size())
        mUvPinned.assign(mMeshData->positions.size(), 0);
    for (const u32 vertex : vertices)
    {
        if (vertex < mUvPinned.size())
            mUvPinned[vertex] = pinned ? 1 : 0;
    }
}

void BlenderApplication::clearUvPins()
{
    mUvPinned.clear();
}

u32 BlenderApplication::transformUvs(const std::vector<u32>& vertices, const Math::vec2& pivot,
                                     const MeshUv::Transform& change)
{
    if (!mMeshData || vertices.empty())
        return 0;
    recordUndo();
    const u32 moved = MeshUv::transform(*mMeshData, vertices, uvPinned(), pivot, change);
    applyMeshEdit(true);
    return moved;
}

u32 BlenderApplication::fitUvs(const std::vector<u32>& vertices, bool keepAspect, f32 margin)
{
    if (!mMeshData || vertices.empty())
        return 0;
    recordUndo();
    const u32 moved = MeshUv::fit(*mMeshData, vertices, uvPinned(), keepAspect, margin);
    applyMeshEdit(true);
    return moved;
}

u32 BlenderApplication::fitUvsPerPart(bool keepAspect, f32 margin)
{
    if (!mMeshData || mMeshData->submeshes.empty())
        return 0;
    recordUndo();
    u32 moved = 0;
    // Vertices shared between parts would be fitted twice; the first part wins.
    std::vector<bool> done(mMeshData->positions.size(), false);
    for (u32 part = 0; part < static_cast<u32>(mMeshData->submeshes.size()); ++part)
    {
        std::vector<u32> vertices = submeshVertices(part);
        vertices.erase(std::remove_if(vertices.begin(), vertices.end(), [&](u32 v) { return done[v]; }),
                       vertices.end());
        for (const u32 vertex : vertices)
            done[vertex] = true;
        moved += MeshUv::fit(*mMeshData, vertices, uvPinned(), keepAspect, margin);
    }
    applyMeshEdit(true);
    return moved;
}

bool BlenderApplication::boxMapUvs(UvTarget target, s32 part, f32 tile, const Math::vec2& offset, u32* added,
                                   std::string* error)
{
    if (!mMeshData)
        return false;
    std::vector<u32> triangles;
    if (target == UvTarget::Selection || target == UvTarget::Island)
    {
        triangles = selectedTriangles();
        if (target == UvTarget::Island && !triangles.empty())
        {
            const std::vector<u32> island = MeshUv::islands(*mMeshData);
            std::vector<bool> wanted(island.size() ? *std::max_element(island.begin(), island.end()) + 1 : 0, false);
            for (const u32 t : triangles)
                wanted[island[t]] = true;
            triangles.clear();
            for (u32 t = 0; t < static_cast<u32>(island.size()); ++t)
            {
                if (wanted[island[t]])
                    triangles.push_back(t);
            }
        }
        if (triangles.empty())
        {
            if (error)
                *error = "nothing is selected";
            return false;
        }
    }
    else if (target == UvTarget::Part)
    {
        if (part < 0 || static_cast<usize>(part) >= mMeshData->submeshes.size())
        {
            if (error)
                *error = "no such part";
            return false;
        }
        const SubMesh& submesh = mMeshData->submeshes[static_cast<u32>(part)];
        for (u32 i = submesh.indexOffset; i < submesh.indexOffset + submesh.indexCount; i += 3)
            triangles.push_back(i / 3);
    }
    else
    {
        triangles.resize(mMeshData->indices.size() / 3);
        for (u32 i = 0; i < static_cast<u32>(triangles.size()); ++i)
            triangles[i] = i;
    }

    recordUndo();
    const u32 split = MeshUv::boxMap(*mMeshData, triangles, tile, offset);
    if (added)
        *added = split;
    applyMeshEdit(split == 0);
    return true;
}

u32 BlenderApplication::paintSelection(const Math::vec4& color, f32 opacity, std::string* error)
{
    if (!mMeshData)
        return 0;
    const std::vector<u32> vertices = editVertices();
    if (vertices.empty())
    {
        if (error)
            *error = "nothing is selected - select vertices, faces or edges, or paint a part or a sphere";
        return 0;
    }
    recordUndo();
    const u32 changed = MeshPaint::paintVertices(*mMeshData, vertices, color, opacity);
    mSettings.viewport().showVertexColors = true;
    applyMeshEdit();
    return changed;
}

u32 BlenderApplication::paintPart(u32 part, const Math::vec4& color, f32 opacity)
{
    if (!mMeshData || part >= mMeshData->submeshes.size())
        return 0;
    recordUndo();
    const u32 changed = MeshPaint::paintVertices(*mMeshData, submeshVertices(part), color, opacity);
    mSettings.viewport().showVertexColors = true;
    applyMeshEdit();
    return changed;
}

u32 BlenderApplication::paintAll(const Math::vec4& color, f32 opacity)
{
    if (!mMeshData)
        return 0;
    std::vector<u32> vertices(mMeshData->positions.size());
    for (u32 i = 0; i < static_cast<u32>(vertices.size()); ++i)
        vertices[i] = i;
    recordUndo();
    const u32 changed = MeshPaint::paintVertices(*mMeshData, vertices, color, opacity);
    mSettings.viewport().showVertexColors = true;
    applyMeshEdit();
    return changed;
}

u32 BlenderApplication::paintSphere(const Math::vec3& center, f32 radius, f32 hardness, const Math::vec4& color,
                                    f32 opacity, s32 part)
{
    if (!mMeshData)
        return 0;
    std::vector<u32> subset;
    if (part >= 0)
    {
        if (static_cast<usize>(part) >= mMeshData->submeshes.size())
            return 0;
        subset = submeshVertices(static_cast<u32>(part));
    }
    recordUndo();
    const u32 changed =
        MeshPaint::paintSphere(*mMeshData, part >= 0 ? &subset : nullptr, center, radius, hardness, color, opacity);
    mSettings.viewport().showVertexColors = true;
    applyMeshEdit();
    return changed;
}

bool BlenderApplication::clearVertexColors(s32 part, bool selectionOnly, std::string* error)
{
    if (!mMeshData)
        return false;
    std::vector<u32> vertices;
    if (selectionOnly)
    {
        vertices = editVertices();
        if (vertices.empty())
        {
            if (error)
                *error = "nothing is selected";
            return false;
        }
    }
    else if (part >= 0)
    {
        if (static_cast<usize>(part) >= mMeshData->submeshes.size())
            return false;
        vertices = submeshVertices(static_cast<u32>(part));
    }
    recordUndo();
    MeshPaint::clear(*mMeshData, selectionOnly || part >= 0 ? &vertices : nullptr);
    applyMeshEdit();
    return true;
}

bool BlenderApplication::hasVertexColors() const
{
    return mMeshData && MeshPaint::hasColors(*mMeshData);
}

void BlenderApplication::ownMaterial(u32 index)
{
    SubMesh& submesh = mMeshData->submeshes[index];

    // Restyling a shared material would repaint other submeshes, so the part gets a private copy first.
    bool shared = false;
    for (usize i = 0; i < mMeshData->submeshes.size(); ++i)
    {
        if (i != index && mMeshData->submeshes[i].materialSlot == submesh.materialSlot)
            shared = true;
    }
    if (shared || submesh.materialSlot >= mMeshData->materials.size())
    {
        const Material source = submesh.materialSlot < mMeshData->materials.size()
                                    ? mMeshData->materials[submesh.materialSlot]
                                    : Material();
        const usize oldSlot = submesh.materialSlot;
        const usize slot = mMeshData->materials.size();
        mMeshData->materials.push_back(source);
        // The per-material file-name arrays run parallel to `materials` when in use; the copy starts with the same
        // textures.
        auto duplicatePath = [oldSlot, slot](std::vector<std::string>& paths)
        {
            if (paths.empty())
                return;
            paths.resize(slot);
            paths.push_back(oldSlot < slot ? paths[oldSlot] : std::string());
        };
        duplicatePath(mMeshData->materialTextureFiles);
        duplicatePath(mMeshData->materialNormalFiles);
        duplicatePath(mMeshData->materialSurfaceFiles);
        duplicatePath(mMeshData->materialEmissiveFiles);
        duplicatePath(mMeshData->materialHeightFiles);
        submesh.materialSlot = static_cast<u32>(slot);
    }
}

bool BlenderApplication::setPartTexture(u32 index, u32 slot, const std::string& path, std::string* error)
{
    auto fail = [error](const std::string& message)
    {
        if (error)
            *error = message;
        return false;
    };
    if (!mMeshData || index >= mMeshData->submeshes.size())
        return fail("no such part");
    if (slot != SlotAlbedo && slot != SlotNormal && slot != SlotSurface && slot != SlotEmissive)
        return fail("unsupported texture slot");
    if (!path.empty() && !FileSystem::getSingleton().exists(path))
        return fail("image file not found: " + path);

    recordUndo();
    ownMaterial(index);
    Material& material = mMeshData->materials[mMeshData->submeshes[index].materialSlot];
    const usize materialIndex = mMeshData->submeshes[index].materialSlot;

    // The import-time path arrays would put an old file back on the next load.
    std::vector<std::string>* importPaths = slot == SlotAlbedo    ? &mMeshData->materialTextureFiles
                                            : slot == SlotNormal  ? &mMeshData->materialNormalFiles
                                            : slot == SlotSurface ? &mMeshData->materialSurfaceFiles
                                                                  : &mMeshData->materialEmissiveFiles;
    if (materialIndex < importPaths->size())
        (*importPaths)[materialIndex].clear();

    if (path.empty())
        material.textures[slot] = MaterialTexture();
    else
    {
        MaterialTexture& texture = material.textures[slot];
        texture.texture = Assets().loadTexture(path, Material::colorSpaceFor(static_cast<MaterialSlot>(slot)));
        SamplerDesc sampler;
        sampler.filter = Filter::Anisotropic;
        sampler.wrapU = Wrap::Repeat;
        sampler.wrapV = Wrap::Repeat;
        sampler.wrapW = Wrap::Repeat;
        sampler.anisotropy = 8.0f;
        texture.sampler = Assets().getSampler(sampler);
        texture.source = TextureSource::Static;
        texture.file = path;
    }
    material.paramsDirty = true;
    applyMeshEdit();
    return true;
}

bool BlenderApplication::styleSubmesh(u32 index, const PartStyle& style, bool undoStep)
{
    if (!mMeshData || index >= mMeshData->submeshes.size())
        return false;

    if (undoStep)
        recordUndo();

    SubMesh& submesh = mMeshData->submeshes[index];
    ownMaterial(index);

    Material& material = mMeshData->materials[submesh.materialSlot];
    if (!style.name.empty())
        material.name = style.name;
    if (style.hasColor)
        material.params.baseColor = style.color;
    if (style.hasRoughness)
        material.params.surface.x = style.roughness;
    if (style.hasMetallic)
        material.params.surface.y = style.metallic;
    material.paramsDirty = true;

    applyMeshEdit();
    return true;
}

void BlenderApplication::drawPaintMenu()
{
    ImGui::BeginDisabled(!mMeshData);

    ImGui::ColorEdit3("Color", &mPaintColor.x);
    ImGui::SetNextItemWidth(150.0f);
    ImGui::SliderFloat("Opacity", &mPaintOpacity, 0.0f, 1.0f);
    ImGui::Separator();

    const Math::vec4 color(MeshPaint::toLinear(mPaintColor), 1.0f);
    if (ImGui::MenuItem("Paint Selection", nullptr, false, hasAnySelection()))
        paintSelection(color, mPaintOpacity);
    const bool hasPart = mSelectedSubmesh >= 0;
    if (ImGui::MenuItem("Paint Selected Part", nullptr, false, hasPart))
        paintPart(static_cast<u32>(mSelectedSubmesh), color, mPaintOpacity);
    if (ImGui::MenuItem("Paint Everything"))
        paintAll(color, mPaintOpacity);
    ImGui::Separator();

    if (ImGui::MenuItem("Clear Selection's Colors", nullptr, false, hasAnySelection()))
        clearVertexColors(-1, true);
    if (ImGui::MenuItem("Clear All Vertex Colors", nullptr, false, hasVertexColors()))
        clearVertexColors(-1, false);
    ImGui::Separator();

    ImGui::Checkbox("Show Vertex Colors", &mSettings.viewport().showVertexColors);
    ImGui::TextDisabled("Colors multiply the material and export as COLOR_0.");

    ImGui::EndDisabled();
}

void BlenderApplication::drawTransformMenu()
{
    ImGui::BeginDisabled(!mMeshData);

    // Which vertices this is about to move; otherwise a selection edit cannot be told from one reshaping the whole
    // model.
    if (mSelection.selectedVertexCount() > 0)
        ImGui::TextDisabled("%u selected vertices", mSelection.selectedVertexCount());
    else
        ImGui::TextDisabled("Whole mesh - nothing selected");
    ImGui::Separator();

    ImGui::SetNextItemWidth(150.0f);
    ImGui::SliderFloat("Scale Factor", &mScaleFactor, 0.1f, 10.0f);
    // No shortcut: S and R put the gizmo into scale/rotate, and these apply a typed amount.
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
    // Projection is cheap; this gives charts that do not overlap.
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

    ImGui::SetNextItemWidth(90.0f);
    ImGui::Combo("##knifeAxis", &mKnifeAxis, "X\0Y\0Z\0");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0f);
    ImGui::DragFloat("##knifeOffset", &mKnifeOffset, 0.01f, -1000.0f, 1000.0f, "%.3f");
    if (ImGui::MenuItem("Knife Cut"))
    {
        Math::vec3 normal(0.0f);
        normal[mKnifeAxis] = 1.0f;
        std::string why;
        if (!knifeCut(normal, mKnifeOffset, &why))
            Log::warning("BlenderApplication: %s", why.c_str());
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Cuts every triangle the plane crosses and keeps all the geometry, "
                          "unlike Bisect. The new line of edges is selected.");

    ImGui::SetNextItemWidth(90.0f);
    ImGui::Combo("##mirrorAxis", &mMirrorAxis, "X\0Y\0Z\0");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0f);
    ImGui::DragFloat("##mirrorOffset", &mMirrorOffset, 0.01f, -1000.0f, 1000.0f, "%.3f");
    if (ImGui::MenuItem("Mirror"))
    {
        std::string why;
        if (!mirrorGeometry(mMirrorAxis, mMirrorOffset, 1.0e-4f, &why))
            Log::warning("BlenderApplication: %s", why.c_str());
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Adds a mirror image of the selected faces (or the whole mesh) across the "
                          "plane. Vertices on the plane are shared.");

    {
        int symmetry = mSymmetryAxis + 1;
        ImGui::SetNextItemWidth(90.0f);
        if (ImGui::Combo("Symmetry", &symmetry, "Off\0X\0Y\0Z\0"))
            setSymmetry(symmetry - 1, mMirrorOffset);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("While on, moving vertices also moves the mirrored ones across the "
                              "plane at the Mirror offset above.");
    }

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
    ImGui::TextDisabled("Subdivide");
    ImGui::SetNextItemWidth(150.0f);
    ImGui::SliderInt("Levels", &mSubdivideLevels, 1, 4);
    std::string subdivideError;
    if (ImGui::MenuItem("Subdivide"))
        subdivideSelection(static_cast<u32>(mSubdivideLevels), false, &subdivideError);
    if (!subdivideError.empty())
        Log::warning("BlenderApplication: %s", subdivideError.c_str());
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Splits each selected triangle into four (the whole mesh when nothing is selected).");
    if (ImGui::MenuItem("Subdivide Smooth"))
        subdivideSelection(static_cast<u32>(mSubdivideLevels), true, &subdivideError);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Loop subdivision: rounds the surface as it adds triangles.");

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
                                                       editVertices());
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
            Assets().smoothVertices(*mMeshData, mSmoothingStrength, 1, editVertices());
            Log::info("BlenderApplication: smoothed the selection (strength %.2f)", mSmoothingStrength);
            applyMeshEdit();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void BlenderApplication::drawSaveInfoPopup()
{
    // OpenPopup() runs here, outside the File menu's window/ID stack; from inside BeginMenu it never opened (see
    // drawPreferencesPopup()).
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

    // Snapshot the path before the click: loadMesh()/removeRecentFile() may change recentFiles while this loop walks it.
    std::string clicked;
    for (usize i = 0; i < recentFiles.size(); ++i)
    {
        const std::string& path = recentFiles[i];
        // ImGui takes an item's identity from its label, and recent files in different folders share base names; without
        // a per-row id the second row opens the first.
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
    else if (action == FileDialogExportGltf)
        initialName = "mesh.glb";

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
    else if (action == FileDialogExportGltf)
        exportGltf(result.path.string());
    else if (action == FileDialogAppendAnimation)
        appendAnimation(result.path.string());
    else if (action == FileDialogHeightmap)
    {
        mPrimitiveHeightmap = result.path.string();
        // The popup closed when the dialog took over; reopen it with the image filled in.
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

    if (mSymmetryAxis >= 0)
    {
        ImGui::SameLine(0.0f, 24.0f);
        ImGui::TextColored(ImVec4(0.45f, 0.75f, 1.0f, 1.0f), "Sym %c", "XYZ"[mSymmetryAxis]);
    }
    if (apiRunning())
    {
        ImGui::SameLine(0.0f, 24.0f);
        ImGui::TextColored(ImVec4(0.4f, 0.85f, 0.5f, 1.0f), "API :%d", apiPort());
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
    // "Preferences..." is the last item before the Windows menu's EndMenu(), which cancels an OpenPopup() called from in
    // there; defer via a flag and open here at top level.
    if (mPreferencesRequested)
    {
        ImGui::OpenPopup("PreferencesPopup");
        mPreferencesRequested = false;
    }

    // Without an explicit position a modal cascades from ImGui's last window position, which in the full-window
    // DockSpace can land behind a docked panel or under the menu bar.
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
    ImGui::TextDisabled("Snap");
    BlenderSettings::SnapSettings& snapSettings = mSettings.snap();
    ImGui::DragFloat("Move Step", &snapSettings.moveStep, 0.01f, 0.0001f, 100.0f, "%.4f");
    ImGui::DragFloat("Rotate Step (deg)", &snapSettings.rotateStepDegrees, 0.5f, 0.01f, 180.0f);
    ImGui::DragFloat("Scale Step", &snapSettings.scaleStep, 0.005f, 0.0001f, 10.0f, "%.4f");
    ImGui::DragFloat("Vertex Snap Radius (px)", &snapSettings.vertexRadiusPixels, 0.5f, 1.0f, 100.0f);

    ImGui::Separator();
    ImGui::TextDisabled("API (HTTP, for scripts and AI tools)");
    const bool apiIsRunning = apiRunning();
    ImGui::BeginDisabled(apiIsRunning);
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("Port", &mApiPortField, 0, 0);
    ImGui::InputText("Token", &mApiTokenField, ImGuiInputTextFlags_Password);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Optional. When set, every request must send 'Authorization: Bearer <token>'.\n"
                          "Not saved - set RADION_BLENDER_API_TOKEN or --api-token to have it at startup.");
    if (!apiIsRunning)
    {
        if (ImGui::Button("Start API") &&
            startApi("127.0.0.1", Math::clamp(mApiPortField, 1, 65535), mApiTokenField))
            mSettings.api().port = apiPort();
    }
    else
    {
        ImGui::Text("Listening on http://127.0.0.1:%d%s", apiPort(),
                    apiHasToken() ? " (token required)" : "");
        if (ImGui::Button("Stop API"))
            stopApi();
    }
    if (!mApiError.empty())
        ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.3f, 1.0f), "%s", mApiError.c_str());
    ImGui::Checkbox("Start with the editor", &mSettings.api().enabled);

    ImGui::Separator();
    if (ImGui::Button("Close", ImVec2(120.0f, 0.0f)))
    {
        mSettings.save(mSettingsPath);
        ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
}
