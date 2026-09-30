#include "PCH.h"
#include "MaterialsPanel.h"
#include "../BlenderApplication.h"
#include "../MaterialEditor.h"
#include "AssetManager.h"
#include "IconsMaterialDesignIcons.h"
#include "Material.h"
#include "MaterialManager.h"

#include <algorithm>
#include <imgui.h>

using namespace Radion;

namespace
{
std::string extensionOf(const std::string& name)
{
    const usize dot = name.find_last_of('.');
    if (dot == std::string::npos || dot + 1 >= name.size())
        return std::string();

    std::string extension = name.substr(dot + 1);
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char character)
                   {
                       return static_cast<char>(std::tolower(character));
                   });
    return extension;
}

bool isMaterialAsset(const std::string& extension)
{
    return extension == "material" || extension == "mat";
}

bool isMeshAsset(const std::string& extension)
{
    return extension == "obj" || extension == "fbx" || extension == "gltf" || extension == "glb" ||
          extension == "rmesh" || extension == "rstm" || extension == "mesh" ||
          extension == "b3d" || extension == "3ds" || extension == "ms3d";
}

bool isNativeMeshAsset(const std::string& extension)
{
    return extension == "rmesh" || extension == "rstm";
}

bool isImageAsset(const std::string& extension)
{
    return extension == "png" || extension == "jpg" || extension == "jpeg" ||
          extension == "tga" || extension == "bmp" || extension == "dds" || extension == "hdr" ||
          extension == "webp";
    // .exr left out on purpose, same reason as the editor's AssetsPanel: the
    // loader does not read it, so this would just fail every frame.
}

const char* iconForEntry(const FileSystem::DirEntry& entry)
{
    if (entry.isDirectory)
        return ICON_MDI_FOLDER;
    const std::string extension = extensionOf(entry.name);
    if (isMaterialAsset(extension))
        return ICON_MDI_PALETTE;
    if (isMeshAsset(extension))
        return ICON_MDI_CUBE_OUTLINE;
    return ICON_MDI_FILE;
}

ImVec4 iconColorForEntry(const FileSystem::DirEntry& entry)
{
    if (entry.isDirectory)
        return ImVec4(0.95f, 0.78f, 0.35f, 1.0f);
    const std::string extension = extensionOf(entry.name);
    if (isMaterialAsset(extension))
        return ImVec4(0.85f, 0.55f, 0.9f, 1.0f);
    if (isMeshAsset(extension))
        return ImVec4(0.4f, 0.75f, 0.9f, 1.0f);
    return ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
}
} // namespace

MaterialsPanel::MaterialsPanel(BlenderApplication& app)
    : BlenderPanel("Materials", app)
{
    mRootDirectory = RADION_ASSET_DIR;
    mCurrentDirectory = mRootDirectory;
    mHistory.push_back(mCurrentDirectory);
}

MaterialsPanel::~MaterialsPanel()
{
}

void MaterialsPanel::onImGui()
{
    if (ImGui::Begin(title().c_str()))
    {
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05f, 0.05f, 0.05f, 0.54f));
        ImGui::BeginChild("##materials_tree", ImVec2(220.0f, 0.0f), true, ImGuiWindowFlags_HorizontalScrollbar);
        drawDirectoryTree();
        ImGui::EndChild();
        ImGui::PopStyleColor();

        ImGui::SameLine();

        ImGui::BeginChild("##materials_content", ImVec2(0.0f, 0.0f), false);
        drawToolbar();
        drawBreadcrumb();
        ImGui::Separator();

        refreshEntries();

        if (mViewMode == ViewMode::List)
            drawList();
        else if (mViewMode == ViewMode::Details)
            drawDetails();
        else
            drawGrid();

        drawInspector();
        ImGui::EndChild();
    }
    ImGui::End();
}

void MaterialsPanel::drawDirectoryTree()
{
    ImGuiTreeNodeFlags rootFlags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_DefaultOpen;
    if (mCurrentDirectory == mRootDirectory)
        rootFlags |= ImGuiTreeNodeFlags_Selected;

    const bool rootOpen = ImGui::TreeNodeEx(ICON_MDI_FOLDER " assets", rootFlags);
    if (ImGui::IsItemClicked())
        navigateTo(mRootDirectory);

    if (rootOpen)
    {
        drawDirectoryNode(mRootDirectory);
        ImGui::TreePop();
    }
}

void MaterialsPanel::drawDirectoryNode(const std::string& path)
{
    std::vector<FileSystem::DirEntry> entries = FileSystem::getSingleton().listDirectory(path);
    std::sort(entries.begin(), entries.end(),
              [](const FileSystem::DirEntry& a, const FileSystem::DirEntry& b)
              {
                  return a.name < b.name;
              });

    for (const FileSystem::DirEntry& entry : entries)
    {
        if (!entry.isDirectory)
            continue;

        const std::string childPath = FileSystem::join(path, entry.name);
        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow;
        if (childPath == mCurrentDirectory)
            flags |= ImGuiTreeNodeFlags_Selected;

        ImGui::PushID(childPath.c_str());
        const bool open = ImGui::TreeNodeEx(entry.name.c_str(), flags, ICON_MDI_FOLDER " %s", entry.name.c_str());
        if (ImGui::IsItemClicked())
            navigateTo(childPath);

        if (open)
        {
            drawDirectoryNode(childPath);
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
}

void MaterialsPanel::drawToolbar()
{
    const ImGuiStyle& style = ImGui::GetStyle();

    ImGui::BeginDisabled(mHistoryPosition == 0);
    if (ImGui::Button(ICON_MDI_ARROW_LEFT))
    {
        --mHistoryPosition;
        mCurrentDirectory = mHistory[mHistoryPosition];
        mEntriesDirty = true;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();

    ImGui::BeginDisabled(mHistoryPosition + 1 >= mHistory.size());
    if (ImGui::Button(ICON_MDI_ARROW_RIGHT))
    {
        ++mHistoryPosition;
        mCurrentDirectory = mHistory[mHistoryPosition];
        mEntriesDirty = true;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();

    const f32 zoomSliderWidth = mViewMode == ViewMode::Grid ? 100.0f : 0.0f;
    const f32 viewButtonsWidth = ImGui::CalcTextSize(ICON_MDI_VIEW_GRID).x +
                                 ImGui::CalcTextSize(ICON_MDI_VIEW_LIST).x +
                                 ImGui::CalcTextSize(ICON_MDI_TABLE).x +
                                 ImGui::CalcTextSize(ICON_MDI_REFRESH).x +
                                 style.FramePadding.x * 8.0f + style.ItemSpacing.x * 3.0f +
                                 (zoomSliderWidth > 0.0f ? zoomSliderWidth + style.ItemSpacing.x : 0.0f);
    ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - viewButtonsWidth);

    if (zoomSliderWidth > 0.0f)
    {
        ImGui::SetNextItemWidth(zoomSliderWidth);
        ImGui::SliderFloat("##Zoom", &mThumbnailSize, 48.0f, 160.0f, ICON_MDI_MAGNIFY " %.0f");
        ImGui::SameLine();
    }

    if (ImGui::Button(ICON_MDI_REFRESH))
        mEntriesDirty = true;
    ImGui::SetItemTooltip("Refresh");
    ImGui::SameLine();

    if (ImGui::Button(ICON_MDI_VIEW_GRID))
        mViewMode = ViewMode::Grid;
    ImGui::SetItemTooltip("Icon grid");
    ImGui::SameLine();

    if (ImGui::Button(ICON_MDI_VIEW_LIST))
        mViewMode = ViewMode::List;
    ImGui::SetItemTooltip("List");
    ImGui::SameLine();

    if (ImGui::Button(ICON_MDI_TABLE))
        mViewMode = ViewMode::Details;
    ImGui::SetItemTooltip("Details");
}

void MaterialsPanel::drawBreadcrumb()
{
    std::string relative = mCurrentDirectory.substr(mRootDirectory.size());
    while (!relative.empty() && (relative.front() == '/' || relative.front() == '\\'))
        relative.erase(relative.begin());

    if (ImGui::Selectable("assets", false, ImGuiSelectableFlags_None, ImVec2(48.0f, 0.0f)))
        navigateTo(mRootDirectory);

    if (relative.empty())
        return;

    std::string accumulated = mRootDirectory;
    usize start = 0;
    while (start <= relative.size())
    {
        const usize slash = relative.find('/', start);
        const std::string segment = relative.substr(start, slash == std::string::npos ? slash : slash - start);
        accumulated = FileSystem::join(accumulated, segment);
        ImGui::SameLine(0.0f, 2.0f);
        ImGui::TextUnformatted("/");
        ImGui::SameLine(0.0f, 2.0f);
        ImGui::PushID(static_cast<int>(start));
        if (ImGui::Selectable(segment.c_str(), false, ImGuiSelectableFlags_None,
                              ImVec2(ImGui::CalcTextSize(segment.c_str()).x, 0.0f)))
            navigateTo(accumulated);
        ImGui::PopID();
        if (slash == std::string::npos)
            break;
        start = slash + 1;
    }
}

void MaterialsPanel::navigateTo(const std::string& path)
{
    if (path == mCurrentDirectory)
        return;

    mHistory.erase(mHistory.begin() + static_cast<std::ptrdiff_t>(mHistoryPosition) + 1, mHistory.end());
    mHistory.push_back(path);
    mHistoryPosition = mHistory.size() - 1;
    mCurrentDirectory = path;
    mEntriesDirty = true;
}

void MaterialsPanel::refreshEntries()
{
    if (!mEntriesDirty)
        return;

    mEntries = FileSystem::getSingleton().listDirectory(mCurrentDirectory);
    std::sort(mEntries.begin(), mEntries.end(),
              [](const FileSystem::DirEntry& a, const FileSystem::DirEntry& b)
              {
                  if (a.isDirectory != b.isDirectory)
                      return a.isDirectory > b.isDirectory;
                  return a.name < b.name;
              });
    mEntriesDirty = false;
}

void MaterialsPanel::openEntry(const FileSystem::DirEntry& entry)
{
    const std::string path = FileSystem::join(mCurrentDirectory, entry.name);
    if (entry.isDirectory)
        navigateTo(path);
    else if (isMaterialAsset(extensionOf(entry.name)))
        openMaterialFile(path);
}

void MaterialsPanel::drawList()
{
    ImGui::Indent(14.0f);
    if (mCurrentDirectory != mRootDirectory && ImGui::Selectable(ICON_MDI_ARROW_UP " .."))
        navigateTo(FileSystem::directoryOf(mCurrentDirectory));

    for (const FileSystem::DirEntry& entry : mEntries)
    {
        ImGui::PushID(entry.name.c_str());
        ImGui::PushStyleColor(ImGuiCol_Text, iconColorForEntry(entry));
        ImGui::TextUnformatted(iconForEntry(entry));
        ImGui::PopStyleColor();
        ImGui::SameLine();
        const std::string path = FileSystem::join(mCurrentDirectory, entry.name);
        if (ImGui::Selectable(entry.name.c_str(), path == mSelectedFile))
            openEntry(entry);
        drawContextMenu(entry);
        ImGui::PopID();
    }
    ImGui::Unindent(14.0f);
}

void MaterialsPanel::drawDetails()
{
    if (!ImGui::BeginTable("##materials_details", 2,
                           ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                               ImGuiTableFlags_SizingStretchProp))
        return;

    ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 100.0f);
    ImGui::TableHeadersRow();

    if (mCurrentDirectory != mRootDirectory)
    {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (ImGui::Selectable(ICON_MDI_ARROW_UP " ..", false, ImGuiSelectableFlags_SpanAllColumns))
            navigateTo(FileSystem::directoryOf(mCurrentDirectory));
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted("Folder");
    }

    for (const FileSystem::DirEntry& entry : mEntries)
    {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::PushID(entry.name.c_str());
        ImGui::PushStyleColor(ImGuiCol_Text, iconColorForEntry(entry));
        ImGui::TextUnformatted(iconForEntry(entry));
        ImGui::PopStyleColor();
        ImGui::SameLine();
        const std::string path = FileSystem::join(mCurrentDirectory, entry.name);
        if (ImGui::Selectable(entry.name.c_str(), path == mSelectedFile, ImGuiSelectableFlags_SpanAllColumns))
            openEntry(entry);
        drawContextMenu(entry);
        ImGui::PopID();
        ImGui::TableSetColumnIndex(1);
        const std::string extension = extensionOf(entry.name);
        ImGui::TextUnformatted(entry.isDirectory ? "Folder"
                               : isMaterialAsset(extension) ? "Material"
                               : isMeshAsset(extension) ? "Mesh"
                                                        : extension.c_str());
    }
    ImGui::EndTable();
}

TextureHandle MaterialsPanel::thumbnailFor(const std::string& path)
{
    const auto it = mThumbnailCache.find(path);
    if (it != mThumbnailCache.end())
        return it->second;

    // sRGB: a browser thumbnail is judged by eye like any other colour
    // image, never sampled as data - there is no material slot yet to ask
    // Material::colorSpaceFor() about.
    const TextureHandle texture = Assets().loadTexture(path, ColorSpace::sRGB);
    mThumbnailCache.emplace(path, texture);
    return texture;
}

void MaterialsPanel::drawGrid()
{
    const f32 cellSize = mThumbnailSize + 16.0f;
    const f32 panelWidth = ImGui::GetContentRegionAvail().x;
    int columnCount = static_cast<int>(panelWidth / cellSize);
    if (columnCount < 1)
        columnCount = 1;

    int column = 0;
    if (mCurrentDirectory != mRootDirectory)
    {
        ImGui::BeginGroup();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.78f, 0.35f, 1.0f));
        const bool up = ImGui::Button(ICON_MDI_ARROW_UP, ImVec2(mThumbnailSize, mThumbnailSize));
        ImGui::PopStyleColor();
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + mThumbnailSize);
        ImGui::TextWrapped("..");
        ImGui::PopTextWrapPos();
        ImGui::EndGroup();
        if (up)
            navigateTo(FileSystem::directoryOf(mCurrentDirectory));
        column = 1;
    }

    for (const FileSystem::DirEntry& entry : mEntries)
    {
        if (column > 0 && column % columnCount != 0)
            ImGui::SameLine();

        ImGui::PushID(entry.name.c_str());
        ImGui::BeginGroup();

        const std::string path = FileSystem::join(mCurrentDirectory, entry.name);
        const ImVec2 iconSize(mThumbnailSize, mThumbnailSize);
        const bool isImage = !entry.isDirectory && isImageAsset(extensionOf(entry.name));
        const TextureHandle thumbnail = isImage ? thumbnailFor(path) : TextureHandle();

        bool clicked = false;
        if (thumbnail.valid())
        {
            const ImTextureID textureId = static_cast<ImTextureID>(
                static_cast<uintptr_t>(GPU::getSingleton().nativeTextureId(thumbnail)));
            clicked = ImGui::ImageButton("##thumbnail", textureId, iconSize, ImVec2(0.0f, 1.0f),
                                         ImVec2(1.0f, 0.0f));
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                constexpr f32 previewSize = 256.0f;
                ImGui::Image(textureId, ImVec2(previewSize, previewSize), ImVec2(0.0f, 1.0f),
                            ImVec2(1.0f, 0.0f));
                ImGui::TextUnformatted(entry.name.c_str());
                ImGui::EndTooltip();
            }
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Text, iconColorForEntry(entry));
            clicked = ImGui::Button(iconForEntry(entry), iconSize);
            ImGui::PopStyleColor();
        }

        if (path == mSelectedFile)
        {
            ImGui::GetWindowDrawList()->AddRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax(),
                                                IM_COL32(255, 165, 0, 255), 0.0f, 0, 2.0f);
        }

        // Drop target is drawTextureSlot() below, in this same panel's
        // Inspector.
        if (isImage && ImGui::BeginDragDropSource())
        {
            ImGui::SetDragDropPayload(MaterialEditor::kTextureDragPayload, path.c_str(), path.size());
            ImGui::TextUnformatted(entry.name.c_str());
            ImGui::EndDragDropSource();
        }

        drawContextMenu(entry);

        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + mThumbnailSize);
        ImGui::TextWrapped("%s", entry.name.c_str());
        ImGui::PopTextWrapPos();
        ImGui::EndGroup();
        ImGui::PopID();

        if (clicked)
            openEntry(entry);

        ++column;
    }
}

void MaterialsPanel::drawInspector()
{
    if (mSelectedMaterial < 0 || static_cast<usize>(mSelectedMaterial) >= mLoadedMaterials.size())
        return;

    ImGui::Separator();
    Material& material = mLoadedMaterials[static_cast<usize>(mSelectedMaterial)];
    ImGui::Text("%s", material.name.empty() ? mSelectedFile.c_str() : material.name.c_str());

    MaterialEditor::drawFields(material);

    ImGui::Spacing();
    if (ImGui::Button("Save"))
        MaterialManager::getSingleton().save(mSelectedFile, mLoadedMaterials);
}

void MaterialsPanel::drawContextMenu(const FileSystem::DirEntry& entry)
{
    if (entry.isDirectory)
        return;

    const std::string extension = extensionOf(entry.name);
    if (!isMeshAsset(extension))
        return;

    if (!ImGui::BeginPopupContextItem())
        return;

    const std::string path = FileSystem::join(mCurrentDirectory, entry.name);
    const char* label = isNativeMeshAsset(extension) ? ICON_MDI_FILE_IMPORT " Load"
                                                      : ICON_MDI_IMPORT " Import";
    if (ImGui::MenuItem(label))
    {
        if (isNativeMeshAsset(extension))
            app().loadMesh(path);
        else
            app().importMesh(path);
    }

    // One entry regardless of what the file turns out to carry: against an
    // already-loaded skeleton it appends an animation clip, otherwise it
    // merges the file's own geometry into the current mesh - appendAsset()
    // is the one that decides which, the same way Load/Import above already
    // decide by extension rather than asking first.
    const bool hasMesh = app().currentMeshData() && !app().currentMeshData()->positions.empty();
    if (ImGui::MenuItem(ICON_MDI_PLUS " Append", nullptr, false, hasMesh))
        app().appendAsset(path);
    if (!hasMesh && ImGui::IsItemHovered())
        ImGui::SetTooltip("Load a mesh first - Append needs something to add onto.");

    ImGui::EndPopup();
}

void MaterialsPanel::openMaterialFile(const std::string& path)
{
    mLoadedMaterials.clear();
    mSelectedMaterial = -1;
    if (!MaterialManager::getSingleton().load(path, mLoadedMaterials))
        return;

    mSelectedFile = path;
    if (!mLoadedMaterials.empty())
        mSelectedMaterial = 0;
}
