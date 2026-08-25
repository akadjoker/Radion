#include "PCH.h"
#include "HierarchyPanel.h"
#include "../BlenderApplication.h"

#include <imgui.h>

using namespace Radion;

HierarchyPanel::HierarchyPanel(BlenderApplication& app)
    : BlenderPanel("Hierarchy", app)
{
}

HierarchyPanel::~HierarchyPanel()
{
}

void HierarchyPanel::onImGui()
{
    // Begin()/End() must pair unconditionally - an inactive tab in a shared
    // dock node returns false here without being closed, and skipping End()
    // in that case corrupts ImGui's window stack for every window after it.
    if (ImGui::Begin(title().c_str()))
    {
        ImGui::Checkbox("Show Bones", &mShowBones);

        ImGui::Separator();

        if (mShowBones)
            drawBoneTree();
        else
            drawObjectTree();
    }
    ImGui::End();
}

void HierarchyPanel::drawBoneTree()
{
    ImGui::Text("Skeleton Structure");
    ImGui::Indent();

    // TODO: Traverse skeleton and draw bone hierarchy
    // For now, just placeholder

    ImGui::TextDisabled("Load a rigged mesh to see skeleton");

    ImGui::Unindent();
}

void HierarchyPanel::drawObjectTree()
{
    ImGui::Text("Object Hierarchy");
    ImGui::Indent();

    // TODO: Draw loaded mesh parts/submeshes

    ImGui::TextDisabled("No mesh loaded");

    ImGui::Unindent();
}
