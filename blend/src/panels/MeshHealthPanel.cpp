#include "PCH.h"
#include "MeshHealthPanel.h"

#include "../BlenderApplication.h"
#include "Mesh.h"

#include <imgui.h>

using namespace Radion;

namespace
{
const ImVec4 kBad(0.95f, 0.45f, 0.35f, 1.0f);
const ImVec4 kGood(0.45f, 0.8f, 0.45f, 1.0f);

void formatBytes(usize bytes, char* out, usize size)
{
    if (bytes >= 1024ull * 1024ull)
        std::snprintf(out, size, "%.1f MB", static_cast<double>(bytes) / (1024.0 * 1024.0));
    else if (bytes >= 1024ull)
        std::snprintf(out, size, "%.1f KB", static_cast<double>(bytes) / 1024.0);
    else
        std::snprintf(out, size, "%zu B", bytes);
}

void streamRow(const char* label, bool present)
{
    ImGui::TextColored(present ? kGood : ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "%s", present ? "yes" : "no");
    ImGui::SameLine(60.0f);
    ImGui::TextUnformatted(label);
}
} // namespace

MeshHealthPanel::MeshHealthPanel(BlenderApplication& app) : BlenderPanel("Mesh Health", app)
{
}

MeshHealthPanel::~MeshHealthPanel()
{
}

void MeshHealthPanel::refresh()
{
    const MeshData* mesh = app().currentMeshData();
    if (!mesh)
    {
        mAnalyzed = false;
        mAnalyzedMesh = nullptr;
        return;
    }

    Assets().analyzeMesh(*mesh, mDiagnostics);
    mAnalyzedMesh = mesh;
    mAnalyzedVertexCount = mesh->positions.size();
    mAnalyzedIndexCount = mesh->indices.size();
    mAnalyzed = true;
}

void MeshHealthPanel::drawFault(bool bad, const char* label, const char* explanation)
{
    ImGui::TextColored(bad ? kBad : kGood, "%s", bad ? "!" : "ok");
    ImGui::SameLine(30.0f);
    ImGui::TextUnformatted(label);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", explanation);
}

void MeshHealthPanel::onImGui()
{
    if (!ImGui::Begin(title().c_str()))
    {
        ImGui::End();
        return;
    }

    const MeshData* mesh = app().currentMeshData();

    // The analysis is O(triangles) plus a hash per vertex, which is too much
    // to spend every frame on a large mesh. Re-run it when the mesh is
    // swapped or its size changes; an edit that keeps both counts (a
    // transform, say) cannot introduce any of the faults below.
    if (mAutoRefresh && mesh &&
        (mesh != mAnalyzedMesh || mesh->positions.size() != mAnalyzedVertexCount ||
         mesh->indices.size() != mAnalyzedIndexCount))
        refresh();

    if (ImGui::Button("Analyze"))
        refresh();
    ImGui::SameLine();
    ImGui::Checkbox("Auto", &mAutoRefresh);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Re-analyze whenever the mesh changes size");

    ImGui::Separator();

    if (!mesh || !mAnalyzed)
    {
        ImGui::TextDisabled("No mesh loaded.");
        ImGui::End();
        return;
    }

    drawContents();
    ImGui::End();
}

void MeshHealthPanel::drawContents()
{
    const AssetManager::Diagnostics& d = mDiagnostics;

    char memory[64];
    formatBytes(d.memoryBytes, memory, sizeof(memory));

    ImGui::Text("%zu vertices, %zu triangles", d.vertexCount, d.triangleCount);
    ImGui::Text("%zu submeshes, %zu materials", d.submeshCount, d.materialCount);
    ImGui::Text("%s in memory", memory);
    const glm::vec3 size = d.bounds.max - d.bounds.min;
    ImGui::Text("bounds %.3f x %.3f x %.3f", size.x, size.y, size.z);

    ImGui::Separator();
    ImGui::TextDisabled("Vertex streams");
    streamRow("normals", d.hasNormals);
    streamRow("tangents", d.hasTangents);
    streamRow("uvs", d.hasUvs);
    streamRow("uv2 (lightmap)", d.hasUvs2);
    streamRow("colors", d.hasColors);
    streamRow("skin", d.hasSkin);

    ImGui::Separator();
    ImGui::TextDisabled("Faults");

    drawFault(d.outOfRangeIndices > 0, "index range",
              "Indices naming a vertex the mesh does not have. Anything reading them "
              "reads past the end of the vertex arrays.");
    drawFault(d.trianglesTruncated, "triangle count",
              "The index buffer is not a whole number of triangles; the tail is ignored.");
    drawFault(d.submeshRangesInvalid, "submesh ranges",
              "A submesh claims indices past the end of the buffer, or a range that is "
              "not a whole number of triangles.");
    drawFault(d.streamsMismatched, "stream lengths",
              "An attribute array is present but not one entry per vertex. Every consumer "
              "walking it in parallel with the positions runs off the end.");
    drawFault(d.degenerateTriangles > 0, "degenerate triangles",
              "Triangles with a repeated corner or no area. They have no normal to give, "
              "and the zero they contribute drags down the smoothed normals of every "
              "vertex they touch.");
    drawFault(d.nonManifoldEdges > 0, "non-manifold edges",
              "Edges shared by three or more triangles. Nothing that walks the surface - "
              "smoothing, unwrapping, a physics hull - handles them.");
    drawFault(d.orphanVertices > 0, "orphan vertices",
              "Vertices no triangle refers to. Dead weight carried through every later "
              "pass; Optimize Vertex Fetch drops them.");
    drawFault(d.exactDuplicatePositions > 0, "duplicate positions",
              "Vertices sitting on exactly the same point. Weld collapses them. This is a "
              "lower bound - it does not look for near matches, which weld also catches.");

    ImGui::Separator();
    ImGui::TextDisabled("Counts");
    ImGui::Text("out-of-range indices  %u", d.outOfRangeIndices);
    ImGui::Text("degenerate triangles  %u", d.degenerateTriangles);
    ImGui::Text("orphan vertices       %u", d.orphanVertices);
    ImGui::Text("non-manifold edges    %u", d.nonManifoldEdges);
    ImGui::Text("duplicate positions   %u", d.exactDuplicatePositions);
    ImGui::Text("boundary edges        %u", d.boundaryEdges);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Edges used by a single triangle. Expected on an open surface, "
                          "and a hole in something meant to be closed.");
}
