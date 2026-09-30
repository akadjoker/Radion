#ifndef RADION_ASSETS_PANEL_H
#define RADION_ASSETS_PANEL_H

#include "Containers.h"
#include "EditorPanel.h"
#include "FileSystem.h"
#include "GPU.h"
#include "ImGuiFileDialog.h"

#include <filesystem>
#include "Math.h"
#include <string>
#include <vector>

namespace Radion
{

// Drag payload for a file: its path relative to the registered search path containing it (assetRelativePath()); not null-terminated, build a std::string from pointer + size.
constexpr const char* kAssetFileDragPayload = "RADION_ASSET_FILE";

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
    // Absolute, not relative to a root: the browser walks anywhere on disk; asset actions resolve through assetRelativePath().
    std::filesystem::path mCurrentDirectory;
    std::string mLastBrowserRoot; // detects a project switch so the browser resets to its root
    ViewMode mViewMode = ViewMode::Grid;
    f32 mThumbnailSize = 96.0f; // Grid view cell/icon size, the "Zoom" slider

    // Directory listing is cached until navigation, Refresh, or a file change by this panel.
    std::vector<FileSystem::DirEntry> mEntries;
    std::filesystem::path mCachedDirectory;
    bool mEntriesDirty = true;
    void refreshEntries(const std::filesystem::path& directory);

    // Folder tree rooted at bookmarks; children are listed on demand. mTreeCache keeps expanded nodes' listings, dropped by refreshEntries() when mEntriesDirty is set.
    struct TreeDirectory
    {
        std::vector<std::string> names;
        std::vector<u8> hasChildren;
    };
    HashMap<std::string, TreeDirectory> mTreeCache;
    f32 mTreeWidth = 220.0f;
    void drawBookmark(const char* label, const std::filesystem::path& root);
    void drawDirectoryTree(const std::filesystem::path& directory);

    // assetRelativePath() is stat-heavy (slow on FUSE drives); cache per directory listing, dropped by refreshEntries() with mTreeCache.
    struct RelativePathResult
    {
        bool resolved = false;
        std::string relative;
    };
    HashMap<std::string, RelativePathResult> mRelativePathCache;

    // Filled lazily as Grid view draws images; keyed by the asset-root-relative path AssetManager::loadTexture() caches by.
    HashMap<std::string, TextureHandle> mThumbnailCache;
    TextureHandle thumbnailFor(const std::string& relativePath);

    // Path relative to a registered search path (project Assets/, engine Assets/, extras), the only address AssetManager/FileSystem understand; empty outside all of them.
    bool assetRelativePath(const std::filesystem::path& absolute, std::string& outRelative);

    // Back/forward history like a web browser: visiting a new place from mid-history truncates the forward branch.
    std::vector<std::filesystem::path> mHistory;
    usize mHistoryPosition = 0;
    void navigateTo(const std::filesystem::path& directory);

    // Navigation requested by a grid click is applied after the grid finishes; navigating mid-loop would pair mEntries with the wrong mCurrentDirectory.
    std::filesystem::path mPendingNavigation;

    // "Import" queues here; the popup asks for a starting transform first since exports can be far off the scene's scale or orientation.
    bool mImportPending = false;
    std::string mImportPath; // relative to whichever root assetRelativePath() matched
    std::string mImportName; // file name, no extension - the popup title
    Math::vec3 mImportTranslation{0.0f}; // offset from the 3D cursor, not a world position
    Math::vec3 mImportRotationEuler{0.0f}; // degrees
    f32 mImportScale = 1.0f;
    bool mImportOptimize = false; // merge submeshes that share a material, cuts draw calls
    bool mImportSplit = false; // break oversized submeshes into spatially local pieces for the BVH
    int mImportSplitTriangles = 4000;
    void drawImportPopup();
    std::string importOutputBase();

    // Instantiate on a .rprefab: no transform popup; it lands at the 3D cursor.
    void instantiatePrefab(const std::string& relativePath);

    // Queued like Import: file deletion is not covered by undo, so the popup is the only safeguard.
    bool mDeletePending = false;
    std::string mDeletePath; // relative to whichever root assetRelativePath() matched
    void drawDeletePopup();

    // Creation targets the folder under the cursor, not necessarily the open directory.
    std::filesystem::path mCreateTargetDirectory;
    char mNewFolderName[128] = "New Folder";
    char mNewScriptName[128] = "NewScript";
    bool mOpenCreateFolderPopup = false;
    bool mOpenCreateScriptPopup = false;
    void openCreateFolderPopup(const std::filesystem::path& directory);
    void openCreateScriptPopup(const std::filesystem::path& directory);
    void drawCreateFolderPopup();
    void drawCreateScriptPopup();

    // One file dialog shared by Normal Map/Heightmap generation, told apart by mGenerateKind.
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
