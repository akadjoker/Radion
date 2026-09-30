#include "PCH.h"
#include "PropertiesPanel.h"
#include "../BlenderApplication.h"
#include "../MaterialEditor.h"
#include "Material.h"
#include "Mesh.h"

#include <cstdio>
#include <IconsMaterialDesignIcons.h>
#include <imgui.h>

using namespace Radion;

PropertiesPanel::PropertiesPanel(BlenderApplication& app)
    : BlenderPanel("Properties", app)
{
}

PropertiesPanel::~PropertiesPanel()
{
}

void PropertiesPanel::onImGui()
{
    if (ImGui::Begin(title().c_str()))
    {
        drawMeshInfo();
        ImGui::Separator();
        drawFaceUVTools();
        ImGui::Separator();
        drawSubmeshList();
    }
    ImGui::End();
}

void PropertiesPanel::drawMeshInfo()
{
    const MeshData* meshData = app().currentMeshData();
    if (!meshData || meshData->positions.empty())
    {
        ImGui::TextDisabled("No mesh loaded");
        return;
    }

    ImGui::Text("Vertices: %zu   Triangles: %zu", meshData->positions.size(),
               meshData->indices.size() / 3);
    ImGui::Text("Submeshes: %zu   Materials: %zu", meshData->submeshes.size(),
               meshData->materials.size());
    ImGui::Text("Selected: %u vertices, %u faces", app().selection().selectedVertexCount(),
               app().selection().selectedFaceCount());
}

void PropertiesPanel::drawFaceUVTools()
{
    MeshData* meshData = app().currentMeshData();
    if (!meshData || meshData->positions.empty())
        return;

    const u32 selectedFaces = app().selection().selectedFaceCount();
    if (!ImGui::TreeNodeEx("##uv", ImGuiTreeNodeFlags_DefaultOpen, "UV (%u faces selected)",
                           selectedFaces))
        return;

    if (meshData->uvs.size() != meshData->positions.size())
    {
        ImGui::TextDisabled("This mesh has no UVs.");
        ImGui::TextDisabled("Mesh > Generate UV makes them.");
        ImGui::TreePop();
        return;
    }

    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragFloat2("Tile", &mUVScale.x, 0.01f, -64.0f, 64.0f, "%.3f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Multiplies the UVs, so 2 repeats the texture twice across the "
                          "selection. Negative mirrors it.");

    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragFloat("Rotate", &mUVRotation, 1.0f, -180.0f, 180.0f, "%.1f deg");

    ImGui::SetNextItemWidth(150.0f);
    ImGui::DragFloat2("Offset", &mUVOffset.x, 0.005f, -64.0f, 64.0f, "%.3f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Slides the texture. One whole unit lands back where it started.");

    if (ImGui::SmallButton("Reset##uvfields"))
    {
        mUVScale = glm::vec2(1.0f);
        mUVOffset = glm::vec2(0.0f);
        mUVRotation = 0.0f;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Flip U"))
        mUVScale.x = -mUVScale.x;
    ImGui::SameLine();
    if (ImGui::SmallButton("Flip V"))
        mUVScale.y = -mUVScale.y;

    // Scaling and rotating happen around the centre of the selection's own UV
    // bounds, so the amounts above mean the same thing wherever the island
    // sits in the texture.
    const bool wholeMesh = selectedFaces == 0;
    if (ImGui::Button(wholeMesh ? "Apply to whole mesh" : "Apply to selected faces",
                      ImVec2(-1.0f, 0.0f)))
    {
        app().applyFaceUVTransform(mUVScale, mUVRotation, mUVOffset);
        mUVScale = glm::vec2(1.0f);
        mUVOffset = glm::vec2(0.0f);
        mUVRotation = 0.0f;
    }
    if (wholeMesh && ImGui::IsItemHovered())
        ImGui::SetTooltip("Nothing is selected, so this retiles every face.");

    ImGui::TreePop();
}

void PropertiesPanel::drawSubmeshList()
{
    MeshData* meshData = app().currentMeshData();
    if (!meshData || meshData->positions.empty())
        return;

    if (!ImGui::TreeNodeEx("##submeshes", ImGuiTreeNodeFlags_DefaultOpen, "Submeshes (%zu)",
                           meshData->submeshes.size()))
        return;

    s32 pendingDelete = -1;

    for (usize i = 0; i < meshData->submeshes.size(); ++i)
    {
        SubMesh& submesh = meshData->submeshes[i];
        ImGui::PushID(static_cast<int>(i));

        char indexLabel[32];
        snprintf(indexLabel, sizeof(indexLabel), "#%zu", i);
        const bool isSelected = app().selectedSubmesh() == static_cast<s32>(i);
        if (ImGui::Selectable(indexLabel, isSelected, 0, ImVec2(36.0f, 0.0f)))
            app().setSelectedSubmesh(isSelected ? -1 : static_cast<s32>(i));

        ImGui::SameLine();
        const bool visible = app().isSubmeshVisible(static_cast<u32>(i));
        if (ImGui::Button(visible ? ICON_MDI_EYE_OUTLINE : ICON_MDI_EYE_OFF_OUTLINE))
            app().toggleSubmeshVisible(static_cast<u32>(i));
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(visible ? "Hide in Viewport" : "Show in Viewport");

        ImGui::SameLine();
        ImGui::TextDisabled("%u tris", submesh.indexCount / 3);

        ImGui::SameLine();
        const f32 trashWidth = ImGui::GetFrameHeight();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - trashWidth -
                                ImGui::GetStyle().ItemSpacing.x);

        const char* currentName = submesh.materialSlot < meshData->materials.size()
                                      ? (meshData->materials[submesh.materialSlot].name.empty()
                                            ? "(unnamed)"
                                            : meshData->materials[submesh.materialSlot].name.c_str())
                                      : "(no material)";
        if (ImGui::BeginCombo("##material", currentName))
        {
            for (usize m = 0; m < meshData->materials.size(); ++m)
            {
                const std::string& name = meshData->materials[m].name;
                const std::string label = name.empty() ? "Material " + std::to_string(m) : name;
                const bool selected = submesh.materialSlot == static_cast<u32>(m);
                if (ImGui::Selectable(label.c_str(), selected))
                {
                    app().recordUndo();
                    submesh.materialSlot = static_cast<u32>(m);
                    app().applyMeshEdit();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::SameLine();
        if (ImGui::Button(ICON_MDI_TRASH_CAN_OUTLINE))
            pendingDelete = static_cast<s32>(i);

        if (submesh.materialSlot < meshData->materials.size() &&
            ImGui::TreeNodeEx("##material_fields", ImGuiTreeNodeFlags_None, "Material"))
        {
            Material& material = meshData->materials[submesh.materialSlot];
            ImGui::Indent(14.0f);
            // recordUndo() has to run before the widgets - they edit
            // `material` directly, so by the time drawFields() reports
            // whether anything changed, that change already landed in
            // *mMeshData. Snapshot first, then throw the snapshot away on
            // the "nothing changed" branch so an idle frame with the
            // section open does not pile up no-op undo steps.
            app().recordUndo();
            if (MaterialEditor::drawFields(material))
                app().applyMeshEdit();
            else
                app().discardUndo();
            ImGui::Unindent(14.0f);
            ImGui::TreePop();
        }

        ImGui::PopID();
    }

    ImGui::TreePop();

    if (pendingDelete >= 0)
        app().deleteSubmesh(static_cast<u32>(pendingDelete));
}
