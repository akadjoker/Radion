#include "PCH.h"

#include "panels/ProfilerPanel.h"

#include "EditorApplication.h"
#include "Engine.h"
#include "Scene.h"

#include <imgui.h>

namespace Radion
{
void ProfilerPanel::onImGui()
{
    app().engine().drawProfilerContents();

    const Scene& scene = app().scene();
    const Scene::DynamicIndexStats stats = scene.dynamicIndexStats();
    if (ImGui::CollapsingHeader("Dynamic BVH", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Text("State: %s", scene.dynamicCullingEnabled() ? "enabled" : "disabled");
        ImGui::Text("Entries: %u   Nodes: %u   Depth: %u", stats.entryCount, stats.nodeCount,
                    stats.depth);
        ImGui::Text("Visited: %u   Accepted: %u", stats.nodesVisited, stats.entriesAccepted);
        ImGui::Text("Quality: %.1f%s", static_cast<double>(stats.quality),
                    stats.rebuildPending ? "   (rebuilding)" : "");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Total node surface area over the root's. It climbs while the tree "
                              "is only refitted and drops when the rebuild running in the "
                              "background is swapped in.");
    }
}
} // namespace Radion
