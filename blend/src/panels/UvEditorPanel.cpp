#include "PCH.h"
#include "UvEditorPanel.h"

#include "../BlenderApplication.h"
#include "GPU.h"
#include "Material.h"
#include "Mesh.h"
#include "mesh/MeshUv.h"

#include <algorithm>
#include <cmath>
#include <imgui.h>

using namespace Radion;

namespace
{
constexpr f32 kPickRadius = 8.0f;
constexpr u32 kEdgeColor = IM_COL32(255, 200, 60, 200);
constexpr u32 kVertexColor = IM_COL32(230, 230, 230, 255);
constexpr u32 kSelectedColor = IM_COL32(255, 90, 40, 255);
constexpr u32 kPinnedColor = IM_COL32(70, 160, 255, 255);

ImVec2 toImVec(const Math::vec2& v)
{
    return ImVec2(v.x, v.y);
}

Math::vec2 toMath(const ImVec2& v)
{
    return Math::vec2(v.x, v.y);
}
} // namespace

UvEditorPanel::UvEditorPanel(BlenderApplication& app) : BlenderPanel("UV Editor", app)
{
}

void UvEditorPanel::validateSelection()
{
    const MeshData* mesh = app().currentMeshData();
    const usize count = mesh ? mesh->positions.size() : 0;
    if (mSelected.size() != count)
        mSelected.assign(count, 0);
}

std::vector<u32> UvEditorPanel::selectedVertices() const
{
    std::vector<u32> vertices;
    for (u32 i = 0; i < static_cast<u32>(mSelected.size()); ++i)
    {
        if (mSelected[i])
            vertices.push_back(i);
    }
    return vertices;
}

std::vector<u32> UvEditorPanel::operationTarget(const std::vector<u32>& shown) const
{
    std::vector<u32> selected = selectedVertices();
    return selected.empty() ? shown : selected;
}

void UvEditorPanel::frameView(const ImVec2& canvasSize)
{
    const f32 side = std::max(std::min(canvasSize.x, canvasSize.y) - 24.0f, 32.0f);
    mZoom = side;
    mPan = Math::vec2((canvasSize.x - side) * 0.5f, (canvasSize.y - side) * 0.5f);
}

void UvEditorPanel::onImGui()
{
    if (!ImGui::Begin(title().c_str()))
    {
        ImGui::End();
        return;
    }

    MeshData* mesh = app().currentMeshData();
    if (!mesh || mesh->positions.empty())
    {
        ImGui::TextDisabled("No mesh.");
        ImGui::End();
        return;
    }
    if (mesh->uvs.size() != mesh->positions.size())
    {
        ImGui::TextDisabled("The mesh has no texture coordinates.");
        if (ImGui::Button("Box Map Everything"))
        {
            u32 added = 0;
            app().boxMapUvs(BlenderApplication::UvTarget::All, -1, mBoxTile, Math::vec2(0.0f), &added, nullptr);
        }
        ImGui::End();
        return;
    }
    validateSelection();

    std::vector<u32> triangles;
    // A single-part mesh is that part whether or not it is "selected".
    const s32 part = app().selectedSubmesh() >= 0 ? app().selectedSubmesh() : (mesh->submeshes.size() == 1 ? 0 : -1);
    if (part >= 0 && static_cast<usize>(part) < mesh->submeshes.size())
    {
        const SubMesh& submesh = mesh->submeshes[static_cast<u32>(part)];
        for (u32 i = submesh.indexOffset; i + 2 < submesh.indexOffset + submesh.indexCount; i += 3)
            triangles.push_back(i / 3);
    }
    else
    {
        triangles.resize(mesh->indices.size() / 3);
        for (u32 i = 0; i < static_cast<u32>(triangles.size()); ++i)
            triangles[i] = i;
    }
    const std::vector<u32> shown = MeshUv::verticesOfTriangles(*mesh, triangles);

    drawToolbar(shown);
    drawCanvas(triangles, shown);
    ImGui::End();
}

void UvEditorPanel::drawToolbar(const std::vector<u32>& shown)
{
    MeshData* mesh = app().currentMeshData();
    const std::vector<u32> target = operationTarget(shown);
    const bool haveSelection = !selectedVertices().empty();
    auto pivotOf = [&](const std::vector<u32>& vertices)
    {
        const MeshUv::Rect rect = MeshUv::bounds(*mesh, vertices);
        return rect.valid ? rect.center() : Math::vec2(0.5f);
    };

    if (ImGui::Button("All"))
    {
        std::fill(mSelected.begin(), mSelected.end(), 0);
        for (const u32 vertex : shown)
            mSelected[vertex] = 1;
    }
    ImGui::SameLine();
    if (ImGui::Button("None"))
        std::fill(mSelected.begin(), mSelected.end(), 0);
    ImGui::SameLine();
    if (ImGui::Button("From Mesh Selection"))
    {
        std::fill(mSelected.begin(), mSelected.end(), 0);
        std::vector<u32> vertices;
        std::string why;
        if (app().uvTargetVertices(BlenderApplication::UvTarget::Selection, -1, vertices, &why))
        {
            for (const u32 v : vertices)
                mSelected[v] = 1;
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Select the UV vertices of the faces/vertices selected in the viewport");
    ImGui::SameLine();
    ImGui::TextDisabled("%zu vertices, %zu selected, %u pinned", shown.size(), selectedVertices().size(),
                        app().uvPinnedCount());

    ImGui::SetNextItemWidth(70.0f);
    ImGui::DragFloat("##rot", &mRotateStep, 1.0f, -360.0f, 360.0f, "%.0f deg");
    ImGui::SameLine();
    if (ImGui::Button("Rotate"))
    {
        MeshUv::Transform change;
        change.rotateDegrees = mRotateStep;
        app().transformUvs(target, pivotOf(target), change);
    }
    ImGui::SameLine();
    if (ImGui::Button("Flip U"))
    {
        MeshUv::Transform change;
        change.scale = Math::vec2(-1.0f, 1.0f);
        app().transformUvs(target, pivotOf(target), change);
    }
    ImGui::SameLine();
    if (ImGui::Button("Flip V"))
    {
        MeshUv::Transform change;
        change.scale = Math::vec2(1.0f, -1.0f);
        app().transformUvs(target, pivotOf(target), change);
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(60.0f);
    ImGui::DragFloat("##scale", &mScaleStep, 0.01f, 0.01f, 100.0f, "x%.2f");
    ImGui::SameLine();
    if (ImGui::Button("Scale"))
    {
        MeshUv::Transform change;
        change.scale = Math::vec2(mScaleStep);
        app().transformUvs(target, pivotOf(target), change);
    }
    ImGui::SameLine();
    if (ImGui::Button("Fit 0-1"))
        app().fitUvs(target, true, mFitMargin);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Scale and move %s so they fill the 0..1 square", haveSelection ? "the selection" : "everything shown");

    ImGui::SetNextItemWidth(95.0f);
    ImGui::DragFloat("##tile", &mBoxTile, 0.01f, 0.001f, 1000.0f, "%.2f /unit");
    ImGui::SameLine();
    if (ImGui::Button("Box Map"))
    {
        const s32 part = app().selectedSubmesh() >= 0 ? app().selectedSubmesh() : (mesh->submeshes.size() == 1 ? 0 : -1);
        u32 added = 0;
        if (part >= 0)
            app().boxMapUvs(BlenderApplication::UvTarget::Part, part, mBoxTile, Math::vec2(0.0f), &added, nullptr);
        else
            app().boxMapUvs(BlenderApplication::UvTarget::All, -1, mBoxTile, Math::vec2(0.0f), &added, nullptr);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Project the selected part (or everything) from the six box sides. Splits shared vertices.");
    ImGui::SameLine();
    ImGui::BeginDisabled(!haveSelection);
    if (ImGui::Button("Pin"))
        app().setUvPinned(selectedVertices(), true);
    ImGui::SameLine();
    if (ImGui::Button("Unpin"))
        app().setUvPinned(selectedVertices(), false);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Frame"))
        mFrameRequested = true;
    ImGui::SameLine();
    ImGui::Checkbox("Snap", &mSnap);
    ImGui::SameLine();
    ImGui::Checkbox("Texture", &mShowTexture);
}

void UvEditorPanel::drawCanvas(const std::vector<u32>& triangles, const std::vector<u32>& shown)
{
    MeshData* mesh = app().currentMeshData();
    ImVec2 available = ImGui::GetContentRegionAvail();
    available.x = std::max(available.x, 64.0f);
    available.y = std::max(available.y, 64.0f);

    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##uvcanvas", available,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonMiddle | ImGuiButtonFlags_MouseButtonRight);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();

    if (mFrameRequested)
    {
        frameView(available);
        mFrameRequested = false;
    }

    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(origin, ImVec2(origin.x + available.x, origin.y + available.y), true);
    draw->AddRectFilled(origin, ImVec2(origin.x + available.x, origin.y + available.y), IM_COL32(32, 32, 32, 255));

    const Math::vec2 corner = toMath(origin) + mPan;
    auto toScreen = [&](const Math::vec2& uv) { return corner + uv * mZoom; };
    auto toUv = [&](const Math::vec2& screen) { return (screen - corner) / mZoom; };

    bool drewTexture = false;
    if (mShowTexture)
    {
        const s32 part = app().selectedSubmesh() >= 0 ? app().selectedSubmesh() : (mesh->submeshes.size() == 1 ? 0 : -1);
        if (part >= 0 && static_cast<usize>(part) < mesh->submeshes.size())
        {
            const u32 slot = mesh->submeshes[static_cast<u32>(part)].materialSlot;
            if (slot < mesh->materials.size())
            {
                const TextureHandle texture = mesh->materials[slot].textures[SlotAlbedo].texture;
                if (texture.valid())
                {
                    const ImTextureID id = static_cast<ImTextureID>(
                        static_cast<uintptr_t>(GPU::getSingleton().nativeTextureId(texture)));
                    draw->AddImage(id, toImVec(toScreen(Math::vec2(0, 0))), toImVec(toScreen(Math::vec2(1, 1))),
                                   ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f));
                    drewTexture = true;
                }
            }
        }
    }
    if (!drewTexture)
        draw->AddRectFilled(toImVec(toScreen(Math::vec2(0, 0))), toImVec(toScreen(Math::vec2(1, 1))), IM_COL32(54, 54, 54, 255));
    if (!drewTexture && mShowTexture && mesh->submeshes.size() > 1 && app().selectedSubmesh() < 0)
        draw->AddText(ImVec2(origin.x + 8.0f, origin.y + 6.0f), IM_COL32(170, 170, 170, 255),
                      "Select a part to see its texture");
    draw->AddRect(toImVec(toScreen(Math::vec2(0, 0))), toImVec(toScreen(Math::vec2(1, 1))), IM_COL32(150, 150, 150, 255));

    if (mZoom > 120.0f)
    {
        for (int i = 1; i < 10; ++i)
        {
            const f32 t = static_cast<f32>(i) / 10.0f;
            draw->AddLine(toImVec(toScreen(Math::vec2(t, 0))), toImVec(toScreen(Math::vec2(t, 1))), IM_COL32(255, 255, 255, 18));
            draw->AddLine(toImVec(toScreen(Math::vec2(0, t))), toImVec(toScreen(Math::vec2(1, t))), IM_COL32(255, 255, 255, 18));
        }
    }

    for (const u32 triangle : triangles)
    {
        Math::vec2 p[3];
        for (u32 corner3 = 0; corner3 < 3; ++corner3)
            p[corner3] = toScreen(mesh->uvs[mesh->indices[static_cast<usize>(triangle) * 3 + corner3]]);
        for (u32 edge = 0; edge < 3; ++edge)
            draw->AddLine(toImVec(p[edge]), toImVec(p[(edge + 1) % 3]), kEdgeColor);
    }

    const std::vector<u8>* pinned = app().uvPinned();
    for (const u32 vertex : shown)
    {
        const Math::vec2 at = toScreen(mesh->uvs[vertex]);
        const bool isPinned = pinned && (*pinned)[vertex];
        const bool isSelected = mSelected[vertex] != 0;
        const f32 half = isSelected ? 3.5f : 2.5f;
        const u32 color = isSelected ? kSelectedColor : isPinned ? kPinnedColor : kVertexColor;
        draw->AddRectFilled(ImVec2(at.x - half, at.y - half), ImVec2(at.x + half, at.y + half), color);
        if (isPinned && isSelected)
            draw->AddRect(ImVec2(at.x - half - 2, at.y - half - 2), ImVec2(at.x + half + 2, at.y + half + 2), kPinnedColor);
    }

    const ImGuiIO& io = ImGui::GetIO();
    const Math::vec2 mouse = toMath(io.MousePos);

    if (hovered && io.MouseWheel != 0.0f)
    {
        // Zoom about the mouse: the UV point under it stays under it.
        const Math::vec2 before = toUv(mouse);
        mZoom = Math::clamp(mZoom * std::pow(1.15f, io.MouseWheel), 8.0f, 20000.0f);
        mPan = mouse - toMath(origin) - before * mZoom;
    }

    auto nearestVertex = [&]() -> s32
    {
        s32 best = -1;
        f32 bestDistance = kPickRadius;
        for (const u32 vertex : shown)
        {
            const f32 distance = Math::length(toScreen(mesh->uvs[vertex]) - mouse);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                best = static_cast<s32>(vertex);
            }
        }
        return best;
    };

    if (mDrag == Drag::None && hovered)
    {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle) || ImGui::IsMouseClicked(ImGuiMouseButton_Right))
        {
            mDrag = Drag::Pan;
            mDragStart = mouse;
        }
        else if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            const s32 hit = nearestVertex();
            if (hit >= 0)
            {
                if (!mSelected[static_cast<u32>(hit)])
                {
                    if (!io.KeyShift)
                        std::fill(mSelected.begin(), mSelected.end(), 0);
                    mSelected[static_cast<u32>(hit)] = 1;
                }
                else if (io.KeyShift)
                    mSelected[static_cast<u32>(hit)] = 0;
                mDrag = Drag::Move;
                mDragStart = mouse;
                mMoveOrigin.clear();
                mMoveRecorded = false;
                for (const u32 vertex : selectedVertices())
                {
                    if (!pinned || !(*pinned)[vertex])
                        mMoveOrigin.push_back({vertex, mesh->uvs[vertex]});
                }
            }
            else
            {
                if (!io.KeyShift)
                    std::fill(mSelected.begin(), mSelected.end(), 0);
                mDrag = Drag::Box;
                mDragStart = mouse;
            }
        }
    }

    if (mDrag == Drag::Pan)
    {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Middle) || ImGui::IsMouseDown(ImGuiMouseButton_Right))
        {
            mPan += toMath(io.MouseDelta);
        }
        else
            mDrag = Drag::None;
    }
    else if (mDrag == Drag::Move)
    {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            Math::vec2 delta = (mouse - mDragStart) / mZoom;
            if (mSnap && !mMoveOrigin.empty())
            {
                // Snap where the first vertex would land, and move the rest by the same amount.
                const Math::vec2 landing = mMoveOrigin.front().second + delta;
                const Math::vec2 snapped = Math::round(landing / mSnapStep) * mSnapStep;
                delta = snapped - mMoveOrigin.front().second;
            }
            if (Math::length(delta) > 0.0f && !mMoveOrigin.empty())
            {
                if (!mMoveRecorded)
                {
                    app().recordUndo();
                    mMoveRecorded = true;
                }
                for (const auto& [vertex, uv] : mMoveOrigin)
                    mesh->uvs[vertex] = uv + delta;
                app().applyMeshEdit(true);
            }
        }
        else
            mDrag = Drag::None;
    }
    else if (mDrag == Drag::Box)
    {
        const ImVec2 a = toImVec(mDragStart);
        const ImVec2 b = io.MousePos;
        draw->AddRect(a, b, IM_COL32(255, 165, 0, 255));
        draw->AddRectFilled(a, b, IM_COL32(255, 165, 0, 30));
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            const Math::vec2 low = Math::min(mDragStart, mouse);
            const Math::vec2 high = Math::max(mDragStart, mouse);
            for (const u32 vertex : shown)
            {
                const Math::vec2 at = toScreen(mesh->uvs[vertex]);
                if (at.x >= low.x && at.x <= high.x && at.y >= low.y && at.y <= high.y)
                    mSelected[vertex] = 1;
            }
            mDrag = Drag::None;
        }
    }
    (void)active;

    draw->PopClipRect();
}
