#ifndef RADION_ASSETS_PANEL_H
#define RADION_ASSETS_PANEL_H

#include "Containers.h"
#include "EditorPanel.h"
#include "FileSystem.h"
#include "GPU.h"
#include "ImGuiFileDialog.h"

#include <filesystem>
#include <glm/vec3.hpp>
#include <string>
#include <vector>

namespace Radion
{

// Drag payload for a non-directory entry: the data is the file's path
// relative to whichever registered search path (project Assets/, engine
// Assets/, or an extra search path) actually contains it - see
// AssetsPanel::assetRelativePath() (not null-terminated in the payload
// buffer - construct a std::string from the pointer + size). Whoever accepts
// it (InspectorPanel's texture slots, for now) decides what extensions make
// sense for the drop target it landed on.
constexpr const char* kAssetFileDragPayload = "RADION_ASSET_FILE";

// Docked at the bottom of the layout. Browses the whole filesystem, not just
// the project's own assets - double-click a folder to enter it, ".." to go
// up past the project root and back again. Files are draggable onto
// Inspector drop targets (texture slots) when they resolve to a path under a
// registered search path; outside one, the entry is still browsable, just
// not usable as an asset. Grid view shows an image file's own texture scaled
// to the cell instead of just its icon - no separate thumbnail cache/
// pipeline (PLANO_EDITOR.md leaves that out of the first delivery), just the
// loaded texture AssetManager already caches by path.
class AssetsPanel final : public EditorPanel
{
public:
    explicit AssetsPanel(EditorApplication& app);

    void onImGui() override;

private:
    enum class ViewMode
    {
        Grid,
        List,
        Details
    };
    // Absolute, not relative to any single root - the browser walks anywhere
    // on disk, the same as ImGuiFileDialog's own "Up" button. Asset-consuming
    // actions (drag, Import/Load, Generate...) still need a path relative to
    // a registered search path, so those resolve it on demand through
    // assetRelativePath() rather than the browser tracking one itself.
    std::filesystem::path mCurrentDirectory;
    std::string mLastBrowserRoot; // detects a project switch so the browser resets to its root
    ViewMode mViewMode = ViewMode::Grid;
    f32 mThumbnailSize = 96.0f; // Grid view cell/icon size, the "Zoom" slider

    // Directory enumeration is an I/O operation, not UI state. Keep the
    // sorted result until navigation, an explicit Refresh, or an operation
    // performed by this panel changes files on disk.
    std::vector<FileSystem::DirEntry> mEntries;
    std::filesystem::path mCachedDirectory;
    bool mEntriesDirty = true;
    void refreshEntries(const std::filesystem::path& directory);

    // Left-hand folder tree, Blender file-browser style: subdirectories only,
    // each node lists its own children on demand so opening it is the only
    // I/O it costs - closed branches never touch disk. Rooted at a handful of
    // bookmarks (project Assets, engine Assets, extra search paths, and the
    // filesystem itself as an escape hatch) instead of one fixed project
    // root, so a bookmark click or the ".." breadcrumb can reach anywhere.
    // mTreeWidth is the draggable splitter's position against the grid/
    // list/table.
    // mTreeCache keeps each expanded node's subdirectory listing (and the
    // has-children peek behind its arrow) so an open branch costs its I/O
    // once, not every frame - refreshEntries() drops the whole cache
    // whenever anything marks mEntriesDirty.
    struct TreeDirectory
    {
        std::vector<std::string> names;
        std::vector<u8> hasChildren;
    };
    HashMap<std::string, TreeDirectory> mTreeCache;
    f32 mTreeWidth = 220.0f;
    void drawBookmark(const char* label, const std::filesystem::path& root);
    void drawDirectoryTree(const std::filesystem::path& directory);

    // assetRelativePath() below is stat-heavy (std::filesystem::is_directory
    // and ::relative against every registered root) - fine once, ruinous
    // called fresh for every visible row every frame on a FUSE-backed drive,
    // where each stat is a userspace round trip instead of a kernel call.
    // Every caller only ever asks about mCurrentDirectory/entry.name, so the
    // answer is stable for as long as the directory listing is; cached here
    // and dropped by refreshEntries() alongside mTreeCache.
    struct RelativePathResult
    {
        bool resolved = false;
        std::string relative;
    };
    HashMap<std::string, RelativePathResult> mRelativePathCache;

    // Lazily filled as image entries are drawn in Grid view - one GPU upload
    // per path for the panel's lifetime rather than every frame. Keyed by the
    // same asset-root-relative path AssetManager::loadTexture() caches by.
    HashMap<std::string, TextureHandle> mThumbnailCache;
    TextureHandle thumbnailFor(const std::string& relativePath);

    // Every asset-consuming action (thumbnails, drag-drop, Import/Load,
    // Generate..., Delete) needs a path relative to one of the registered
    // search paths - that is the only address AssetManager/FileSystem
    // understand. Tries the project's Assets/, the engine's own Assets/, and
    // each extra search path in turn; empty when the browser is looking at a
    // folder outside all of them (still fine to browse, just not to use).
    bool assetRelativePath(const std::filesystem::path& absolute, std::string& outRelative);

    // Browser history, same shape as a web browser's back/forward: every
    // directory actually visited (double-click, breadcrumb segment, ".." or
    // the arrows themselves) lands at mHistoryPosition, and going back never
    // drops what was ahead until a *new* place is visited from there - only
    // then does the forward branch get truncated away.
    std::vector<std::filesystem::path> mHistory;
    usize mHistoryPosition = 0;
    void navigateTo(const std::filesystem::path& directory);

    // Where a click in the grid asked to go, applied after the grid is done
    // drawing. Navigating in the middle of the loop would leave the rest of
    // that frame pairing mEntries - still the folder being left - with a
    // mCurrentDirectory that is already the folder being entered, and every
    // path built from the two is a file that does not exist.
    std::filesystem::path mPendingNavigation;

    // "Import" queues here instead of creating the GameObject right away -
    // drawImportPopup() asks for a starting transform first, since a mesh
    // export can land many orders of magnitude off the scene's own scale (a
    // whole building at 1 unit = 1cm is common), face the wrong way, or both
    // - and finding that out only after the object already exists means
    // redoing the import instead of just typing different numbers.
    bool mImportPending = false;
    std::string mImportPath; // relative to whichever root assetRelativePath() matched
    std::string mImportName; // file name, no extension - the popup title
    glm::vec3 mImportTranslation{0.0f}; // offset from the 3D cursor, not a world position
    glm::vec3 mImportRotationEuler{0.0f}; // degrees
    f32 mImportScale = 1.0f;
    bool mImportOptimize = false; // merge submeshes that share a material, cuts draw calls
    bool mImportSplit = false; // break oversized submeshes into spatially local pieces for the BVH
    int mImportSplitTriangles = 4000;
    void drawImportPopup();
    std::string importOutputBase();

    // Right-click > Delete. Queued the same way Import is, because deleting
    // a file cannot be undone by the editor's own undo stack - the popup is
    // the only thing standing between a stray click and a lost asset, and it
    // names the sidecars that would go with it.
    bool mDeletePending = false;
    std::string mDeletePath; // relative to whichever root assetRelativePath() matched
    void drawDeletePopup();

    // Creation belongs to the folder under the cursor, not necessarily the
    // directory currently open in the content view: right-clicking a folder
    // can create directly inside it without navigating away first.
    std::filesystem::path mCreateTargetDirectory;
    char mNewFolderName[128] = "New Folder";
    char mNewScriptName[128] = "NewScript";
    bool mOpenCreateFolderPopup = false;
    bool mOpenCreateScriptPopup = false;
    void openCreateFolderPopup(const std::filesystem::path& directory);
    void openCreateScriptPopup(const std::filesystem::path& directory);
    void drawCreateFolderPopup();
    void drawCreateScriptPopup();

    // Right-click an image asset > Generate Normal Map.../Generate
    // Heightmap... - Pixmap::generate_normal_map()/generate_heightmap()
    // read the clicked file, the dialog only asks where the result goes.
    // One dialog shared by both, told apart on completion by mGenerateKind,
    // same "single ImGuiFileDialog, disambiguated by an enum" shape Mesh
    // Tools' own Save dialog already uses for its four kinds.
    enum class GenerateKind
    {
        None,
        Normal,
        Height
    };
    ImGuiFileDialog mGenerateDialog;
    GenerateKind mGenerateKind = GenerateKind::None;
    std::string mGenerateSourcePath; // relative to whichever root assetRelativePath() matched
    void drawGeneratePopup();
};

} // namespace Radion

#endif // RADION_ASSETS_PANEL_H
