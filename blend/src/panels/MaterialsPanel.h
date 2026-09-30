#ifndef RADION_MATERIALS_PANEL_H
#define RADION_MATERIALS_PANEL_H

#include "../BlenderPanel.h"
#include "Containers.h"
#include "FileSystem.h"
#include "GPU.h"
#include "Types.h"

#include <string>
#include <vector>

namespace Radion
{

struct Material;

class MaterialsPanel : public BlenderPanel
{
public:
    explicit MaterialsPanel(BlenderApplication& app);
    ~MaterialsPanel() override;

    void onImGui() override;

private:
    enum class ViewMode
    {
        Grid,
        List,
        Details
    };

    void drawDirectoryTree();
    void drawDirectoryNode(const std::string& path);
    void drawToolbar();
    void drawBreadcrumb();
    void drawGrid();
    void drawList();
    void drawDetails();
    void drawInspector();

    void navigateTo(const std::string& path);
    void refreshEntries();
    void openEntry(const FileSystem::DirEntry& entry);
    void openMaterialFile(const std::string& path);
    void drawContextMenu(const FileSystem::DirEntry& entry);

    // Lazily filled as image entries are drawn in Grid view - one lookup
    // through AssetManager's own texture cache per path for the panel's
    // lifetime rather than every frame. Same approach as the editor's
    // AssetsPanel::thumbnailFor().
    HashMap<std::string, TextureHandle> mThumbnailCache;
    TextureHandle thumbnailFor(const std::string& path);

    std::string mRootDirectory;
    std::string mCurrentDirectory;
    std::string mSelectedFile;

    std::vector<std::string> mHistory;
    usize mHistoryPosition = 0;

    std::vector<FileSystem::DirEntry> mEntries;
    bool mEntriesDirty = true;

    ViewMode mViewMode = ViewMode::Grid;
    f32 mThumbnailSize = 72.0f;

    std::vector<Material> mLoadedMaterials;
    s32 mSelectedMaterial = -1;
};

} // namespace Radion

#endif // RADION_MATERIALS_PANEL_H
