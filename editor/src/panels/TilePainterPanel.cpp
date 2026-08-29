#include "PCH.h"

#include "panels/TilePainterPanel.h"

#include "EditorApplication.h"
#include "GameObject.h"
#include "MaterialManager.h"
#include "Scene.h"
#include "TiledTerrain.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace Radion
{

namespace
{
bool insideMap(TiledTerrain& terrain, int x, int z)
{
    return x >= 0 && x < static_cast<int>(terrain.mapWidth()) &&
          z >= 0 && z < static_cast<int>(terrain.mapHeight());
}

// TiledTerrain only carries an atlas material by name - resolving it here,
// rather than reading a texture pointer straight off the component, is the
// one place this panel's shape diverges from a component that owns its
// texture directly.
bool resolveAtlasTexture(const std::string& materialName, TextureHandle& out)
{
    if (materialName.empty() || !GPU::ready())
        return false;
    std::vector<Material> loaded;
    if (!MaterialManager::getSingleton().load(materialName, loaded) || loaded.empty())
        return false;
    out = loaded.front().textures[SlotAlbedo].texture;
    return out.valid();
}
} // namespace

TilePainterPanel::TilePainterPanel(EditorApplication& app) : EditorPanel("Tile Painter", app)
{
}

void TilePainterPanel::onImGui()
{
    GameObject* object = app().selection().resolve(app().scene());
    if (!object)
    {
        ImGui::TextDisabled("Select an object to paint its TiledTerrain.");
        return;
    }
    TiledTerrain* terrain = object->getComponent<TiledTerrain>();
    if (!terrain)
    {
        ImGui::TextDisabled("This object has no TiledTerrain component.");
        return;
    }
    u32 atlasWidth = 0, atlasHeight = 0;
    TextureHandle atlasTexture;
    if (terrain->atlasMaterial().empty() || !terrain->atlasSize(atlasWidth, atlasHeight) ||
        !resolveAtlasTexture(terrain->atlasMaterial(), atlasTexture))
    {
        ImGui::TextDisabled(
            "Assign an atlas material to the TiledTerrain in the Inspector first.");
        return;
    }

    const int tilesInSide = std::max(1, terrain->tilesInSide());
    const f32 tileStepX = static_cast<f32>(atlasWidth) / static_cast<f32>(tilesInSide);
    const f32 tileStepY = static_cast<f32>(atlasHeight) / static_cast<f32>(tilesInSide);
    const int mapW = static_cast<int>(terrain->mapWidth());
    const int mapH = static_cast<int>(terrain->mapHeight());

    ImGui::Text("%s  |  %u x %u cells", object->name().c_str(), terrain->mapWidth(),
               terrain->mapHeight());
    if (ImGui::RadioButton("Brush", mTool == Tool::Brush))
        mTool = Tool::Brush;
    ImGui::SameLine();
    if (ImGui::RadioButton("Pick", mTool == Tool::Pick))
        mTool = Tool::Pick;
    ImGui::SameLine();
    if (ImGui::RadioButton("Fill", mTool == Tool::Fill))
        mTool = Tool::Fill;
    ImGui::SameLine();
    if (ImGui::RadioButton("Rect", mTool == Tool::Rectangle))
        mTool = Tool::Rectangle;

    if (ImGui::SmallButton("Empty"))
        mSelectedTile = static_cast<int>(terrain->defaultTile());
    ImGui::SameLine();
    ImGui::TextDisabled("Tile: %d", mSelectedTile);
    ImGui::SliderFloat("Atlas zoom", &mAtlasZoom, 0.25f, 3.0f, "%.2fx");
    ImGui::SliderFloat("Map zoom", &mMapZoom, 0.25f, 3.0f, "%.2fx");
    ImGui::TextDisabled(
        "Left paints; right erases to the default tile. Pick reads a tile; fill affects "
        "connected cells.");

    ImGui::SeparatorText("Atlas");
    const ImTextureID atlasTextureId = static_cast<ImTextureID>(
        static_cast<uintptr_t>(GPU::getSingleton().nativeTextureId(atlasTexture)));
    const ImVec2 atlasImageSize(static_cast<f32>(atlasWidth) * mAtlasZoom,
                               static_cast<f32>(atlasHeight) * mAtlasZoom);
    const ImVec2 atlasOrigin = ImGui::GetCursorScreenPos();
    ImGui::Image(atlasTextureId, atlasImageSize);
    const bool atlasHovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    for (int y = 0; y < tilesInSide; ++y)
        for (int x = 0; x < tilesInSide; ++x)
            draw->AddRect(ImVec2(atlasOrigin.x + x * tileStepX * mAtlasZoom,
                                 atlasOrigin.y + y * tileStepY * mAtlasZoom),
                          ImVec2(atlasOrigin.x + (x + 1) * tileStepX * mAtlasZoom,
                                 atlasOrigin.y + (y + 1) * tileStepY * mAtlasZoom),
                          IM_COL32(255, 255, 255, 70));

    if (atlasHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const int x = static_cast<int>((mouse.x - atlasOrigin.x) / (tileStepX * mAtlasZoom));
        const int y = static_cast<int>((mouse.y - atlasOrigin.y) / (tileStepY * mAtlasZoom));
        if (x >= 0 && x < tilesInSide && y >= 0 && y < tilesInSide)
            mSelectedTile = y * tilesInSide + x;
    }
    {
        const int x = mSelectedTile % tilesInSide;
        const int y = mSelectedTile / tilesInSide;
        draw->AddRect(ImVec2(atlasOrigin.x + x * tileStepX * mAtlasZoom,
                             atlasOrigin.y + y * tileStepY * mAtlasZoom),
                      ImVec2(atlasOrigin.x + (x + 1) * tileStepX * mAtlasZoom,
                             atlasOrigin.y + (y + 1) * tileStepY * mAtlasZoom),
                      IM_COL32(255, 202, 52, 255), 0.0f, 0, 2.0f);
    }

    ImGui::SeparatorText("Map");
    ImGui::BeginChild("##tilemap_canvas", ImVec2(0.0f, 0.0f), true,
                      ImGuiWindowFlags_HorizontalScrollbar);
    const f32 drawCellW = tileStepX * mMapZoom;
    const f32 drawCellH = tileStepY * mMapZoom;
    const ImVec2 mapOrigin = ImGui::GetCursorScreenPos();
    const ImVec2 mapSize(static_cast<f32>(mapW) * drawCellW, static_cast<f32>(mapH) * drawCellH);
    draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(mapOrigin, ImVec2(mapOrigin.x + mapSize.x, mapOrigin.y + mapSize.y),
                        IM_COL32(20, 24, 31, 255));

    for (int z = 0; z < mapH; ++z)
    {
        for (int x = 0; x < mapW; ++x)
        {
            const ImVec2 min(mapOrigin.x + static_cast<f32>(x) * drawCellW,
                             mapOrigin.y + static_cast<f32>(z) * drawCellH);
            const ImVec2 max(min.x + drawCellW, min.y + drawCellH);
            glm::vec2 uvMin, uvMax;
            TiledTerrain::atlasUV(terrain->tile(static_cast<u32>(x), static_cast<u32>(z)),
                                  tilesInSide, uvMin, uvMax);
            draw->AddImage(atlasTextureId, min, max, ImVec2(uvMin.x, uvMin.y),
                          ImVec2(uvMax.x, uvMax.y));
            draw->AddRect(min, max, IM_COL32(255, 255, 255, 36));
        }
    }

    ImGui::SetCursorScreenPos(mapOrigin);
    ImGui::InvisibleButton("##tilemap_paint", mapSize);
    const bool hovered = ImGui::IsItemHovered();
    const bool leftDown = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    const bool rightDown = ImGui::IsMouseDown(ImGuiMouseButton_Right);
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const int cellX = static_cast<int>(std::floor((mouse.x - mapOrigin.x) / drawCellW));
    const int cellZ = static_cast<int>(std::floor((mouse.y - mapOrigin.y) / drawCellH));
    const bool erase = rightDown;
    const u8 paintTile = erase ? terrain->defaultTile() : static_cast<u8>(mSelectedTile);

    if (hovered && mTool == Tool::Pick && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
        insideMap(*terrain, cellX, cellZ))
    {
        mSelectedTile =
            static_cast<int>(terrain->tile(static_cast<u32>(cellX), static_cast<u32>(cellZ)));
        mTool = Tool::Brush;
    }
    else if (hovered && (leftDown || rightDown) && mTool != Tool::Pick)
    {
        if (!mPainting)
        {
            app().recordUndo();
            mPainting = true;
            mActionApplied = false;
            mRectStartX = mRectEndX = cellX;
            mRectStartZ = mRectEndZ = cellZ;
            mRectangleErase = erase;
        }
        if (mTool == Tool::Brush)
            TiledTerrain::paintCell(*terrain, cellX, cellZ, paintTile);
        else if (mTool == Tool::Fill && !mActionApplied)
        {
            TiledTerrain::fillCells(*terrain, cellX, cellZ, paintTile);
            mActionApplied = true;
        }
        else if (mTool == Tool::Rectangle)
        {
            mRectEndX = cellX;
            mRectEndZ = cellZ;
        }
    }
    if (mPainting && !leftDown && !rightDown)
    {
        if (mTool == Tool::Rectangle)
            TiledTerrain::paintRectangle(*terrain, mRectStartX, mRectStartZ, mRectEndX, mRectEndZ,
                                        mRectangleErase ? terrain->defaultTile()
                                                         : static_cast<u8>(mSelectedTile));
        mPainting = false;
        mActionApplied = false;
    }
    if (mPainting && mTool == Tool::Rectangle)
    {
        const int left = std::min(mRectStartX, mRectEndX);
        const int right = std::max(mRectStartX, mRectEndX);
        const int top = std::min(mRectStartZ, mRectEndZ);
        const int bottom = std::max(mRectStartZ, mRectEndZ);
        draw->AddRect(ImVec2(mapOrigin.x + static_cast<f32>(left) * drawCellW,
                             mapOrigin.y + static_cast<f32>(top) * drawCellH),
                      ImVec2(mapOrigin.x + static_cast<f32>(right + 1) * drawCellW,
                             mapOrigin.y + static_cast<f32>(bottom + 1) * drawCellH),
                      mRectangleErase ? IM_COL32(244, 94, 82, 255) : IM_COL32(255, 202, 52, 255),
                      0.0f, 0, 2.0f);
    }
    ImGui::EndChild();
}

} // namespace Radion
