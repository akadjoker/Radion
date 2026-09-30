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

    // Lazily filled as Grid view draws image entries; one AssetManager lookup per path rather than per frame.
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
