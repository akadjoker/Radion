#include "PCH.h"
#include "ViewportPanel.h"
#include "../BlenderApplication.h"
#include "Color.h"
#include "MiniBatch.h"

#include <glad.h>
#include "Math.h"
#include <IconsMaterialDesignIcons.h>
#include <ImGuizmo.h>
#include <imgui.h>

using namespace Radion;

ViewportPanel::ViewportPanel(BlenderApplication& app)
    : BlenderPanel("Viewport", app)
{
}

ViewportPanel::~ViewportPanel()
{
    for (RenderTarget& target : mTargets)
        target.destroy();
}

void ViewportPanel::setViewMode(usize index, ViewMode mode)
{
    if (index >= mViewModes.size())
        return;
    mViewModes[index] = mode;

    // Snap the fixed axis views to a sane default distance/target instead of
    // whatever the previous mode's orbit left behind - Blender's own numpad
    // views do not carry the perspective camera's distance across either.
    CameraState& camera = mCameras[index];
    if (mode != ViewMode::Perspective && camera.distance < 0.5f)
        camera.distance = 6.0f;
}

bool ViewportPanel::RenderTarget::ensure(s32 requestedWidth, s32 requestedHeight)
{
    requestedWidth = requestedWidth > 0 ? requestedWidth : 1;
    requestedHeight = requestedHeight > 0 ? requestedHeight : 1;
    if (fbo && width == requestedWidth && height == requestedHeight)
        return true;

    destroy();
    width = requestedWidth;
    height = requestedHeight;

    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glGenTextures(1, &colorTexture);
    glBindTexture(GL_TEXTURE_2D, colorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTexture, 0);

    glGenRenderbuffers(1, &depthRenderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, depthRenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER,
                              depthRenderbuffer);

    const bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (!complete)
    {
        destroy();
        return false;
    }
    return true;
}

void ViewportPanel::RenderTarget::destroy()
{
    if (fbo)
    {
        glDeleteFramebuffers(1, &fbo);
        fbo = 0;
    }
    if (colorTexture)
    {
        glDeleteTextures(1, &colorTexture);
        colorTexture = 0;
    }
    if (depthRenderbuffer)
    {
        glDeleteRenderbuffers(1, &depthRenderbuffer);
        depthRenderbuffer = 0;
    }
    width = height = 0;
}

// The tool keys live here rather than with the application's own shortcuts
// because mTool lives here - the panel owns which gizmo is up. Held off while
// a drag is running so a stray key cannot swap the operation mid-move.
void ViewportPanel::handleToolShortcuts()
{
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard || io.KeyCtrl || io.KeyAlt || ImGui::IsAnyItemActive() ||
        mGizmoDragging)
        return;

    if (ImGui::IsKeyPressed(ImGuiKey_Q, false))
        mTool = Tool::Select;
    else if (ImGui::IsKeyPressed(ImGuiKey_G, false))
        mTool = Tool::Move;
    else if (ImGui::IsKeyPressed(ImGuiKey_R, false))
        mTool = Tool::Rotate;
    else if (ImGui::IsKeyPressed(ImGuiKey_S, false))
        mTool = Tool::Scale;
}

void ViewportPanel::onImGui()
{
    ImGuizmo::BeginFrame();
    handleToolShortcuts();

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    if (ImGui::Begin(title().c_str(), nullptr, ImGuiWindowFlags_MenuBar))
    {
        drawViewportControls();

        ImGui::Dummy(ImVec2(0.0f, 6.0f));
        ImGui::Indent(6.0f);
        drawToolbar();
        ImGui::Unindent(6.0f);
        ImGui::Dummy(ImVec2(0.0f, 6.0f));

        switch (mLayoutMode)
        {
        case LayoutMode::Single:
            drawSingleView();
            break;
        case LayoutMode::ThreeWay:
            drawThreeWayLayout();
            break;
        case LayoutMode::FourWay:
            drawFourWayLayout();
            break;
        }
    }
    ImGui::End();
    ImGui::PopStyleVar();
}

void ViewportPanel::drawViewportControls()
{
    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("Layout"))
        {
            if (ImGui::MenuItem("Single", nullptr, mLayoutMode == LayoutMode::Single))
                mLayoutMode = LayoutMode::Single;
            if (ImGui::MenuItem("3-Way", nullptr, mLayoutMode == LayoutMode::ThreeWay))
                mLayoutMode = LayoutMode::ThreeWay;
            if (ImGui::MenuItem("4-Way", nullptr, mLayoutMode == LayoutMode::FourWay))
                mLayoutMode = LayoutMode::FourWay;
            ImGui::EndMenu();
        }

        if (mLayoutMode == LayoutMode::Single)
        {
            if (ImGui::BeginMenu("View"))
            {
                drawViewModeMenu(0);
                ImGui::EndMenu();
            }
        }

        ImGui::EndMenuBar();
    }
}

// Same shape as EditorApplication's own viewport toolbar
// (editor/src/panels/ViewportPanel.cpp:395-429): a row of tool buttons, the
// active one highlighted with ButtonActive, then Snap and Grid as separate
// toggles. Move/Rotate/Scale have no gizmo behind them yet and Snap has
// nothing to snap - only Select and Grid actually do anything today.
void ViewportPanel::drawToolbar()
{
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6.0f, ImGui::GetStyle().ItemSpacing.y));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 6.0f));

    struct ToolEntry
    {
        Tool tool;
        const char* icon;
        const char* tooltip;
    };
    static const ToolEntry kTools[] = {
        {Tool::Select, ICON_MDI_CURSOR_DEFAULT, "Select"},
        {Tool::Move, ICON_MDI_ARROW_ALL, "Move"},
        {Tool::Rotate, ICON_MDI_ROTATE_ORBIT, "Rotate"},
        {Tool::Scale, ICON_MDI_ARROW_EXPAND_ALL, "Scale"},
    };

    for (usize i = 0; i < 4; ++i)
    {
        if (i > 0)
            ImGui::SameLine();
        const bool active = mTool == kTools[i].tool;
        if (active)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(kTools[i].icon))
            mTool = kTools[i].tool;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", kTools[i].tooltip);
        if (active)
            ImGui::PopStyleColor();
    }

    ImGui::SameLine();
    ImGui::Dummy(ImVec2(10.0f, 0.0f));
    ImGui::SameLine();

    struct SelectModeEntry
    {
        BlenderSelection::SelectionMode mode;
        const char* icon;
        const char* tooltip;
    };
    static const SelectModeEntry kSelectModes[] = {
        {BlenderSelection::SelectionMode::Vertex, ICON_MDI_VECTOR_POINT, "Vertex select"},
        {BlenderSelection::SelectionMode::Edge, ICON_MDI_VECTOR_LINE, "Edge select"},
        {BlenderSelection::SelectionMode::Face, ICON_MDI_VECTOR_SQUARE, "Face select"},
    };

    BlenderSelection& selection = app().selection();
    for (usize i = 0; i < 3; ++i)
    {
        if (i > 0)
            ImGui::SameLine();
        const bool active = selection.mode() == kSelectModes[i].mode;
        if (active)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(kSelectModes[i].icon))
            selection.setMode(kSelectModes[i].mode);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", kSelectModes[i].tooltip);
        if (active)
            ImGui::PopStyleColor();
    }

    ImGui::SameLine();
    {
        const bool wasVisibleOnly = mSelectVisibleOnly;
        if (wasVisibleOnly)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(ICON_MDI_CUBE_SCAN))
            mSelectVisibleOnly = !mSelectVisibleOnly;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Select Visible Only (blocks clicking/boxing through occluded geometry)");
        if (wasVisibleOnly)
            ImGui::PopStyleColor();
    }

    ImGui::SameLine();
    ImGui::Dummy(ImVec2(10.0f, 0.0f));
    ImGui::SameLine();

    struct ShadingModeEntry
    {
        MiniRenderMode mode;
        const char* icon;
        const char* tooltip;
    };
    static const ShadingModeEntry kShadingModes[] = {
        {MiniRenderMode::Wireframe, ICON_MDI_CUBE_OUTLINE, "Wireframe"},
        {MiniRenderMode::Solid, ICON_MDI_CUBE, "Solid"},
        {MiniRenderMode::Textured, ICON_MDI_TEXTURE, "Textured"},
    };

    for (usize i = 0; i < 3; ++i)
    {
        if (i > 0)
            ImGui::SameLine();
        const bool active = mShadingMode == kShadingModes[i].mode;
        if (active)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(kShadingModes[i].icon))
            mShadingMode = kShadingModes[i].mode;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", kShadingModes[i].tooltip);
        if (active)
            ImGui::PopStyleColor();
    }

    ImGui::SameLine();
    {
        const bool wasUnlit = mUnlit;
        if (wasUnlit)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(ICON_MDI_LIGHTBULB_OUTLINE))
            mUnlit = !mUnlit;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Shadeless (flat color, no directional light)");
        if (wasUnlit)
            ImGui::PopStyleColor();
    }

    ImGui::SameLine();
    BlenderSettings::ViewportSettings& viewportSettings = app().settings().viewport();
    {
        const bool wasColorBySubmesh = viewportSettings.colorBySubmesh;
        if (wasColorBySubmesh)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(ICON_MDI_PALETTE))
            viewportSettings.colorBySubmesh = !viewportSettings.colorBySubmesh;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Color Each Submesh Differently");
        if (wasColorBySubmesh)
            ImGui::PopStyleColor();
    }

    ImGui::SameLine();
    ImGui::Dummy(ImVec2(10.0f, 0.0f));
    ImGui::SameLine();

    // Each button toggles its own view on/off, same click-active-again
    // convention as Snap/Grid/Shadeless above - not a separate "off" button
    // to hunt for.
    struct DebugViewEntry
    {
        MiniDebugView view;
        const char* icon;
        const char* tooltip;
    };
    static const DebugViewEntry kDebugViews[] = {
        {MiniDebugView::Normals, ICON_MDI_COMPASS, "Debug View: Normals"},
        {MiniDebugView::Tangents, ICON_MDI_VECTOR_TRIANGLE, "Debug View: Tangents"},
        {MiniDebugView::UVs, ICON_MDI_CHECKERBOARD, "Debug View: UVs"},
    };
    for (usize i = 0; i < 3; ++i)
    {
        if (i > 0)
            ImGui::SameLine();
        const bool active = mDebugView == kDebugViews[i].view;
        if (active)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(kDebugViews[i].icon))
            mDebugView = active ? MiniDebugView::None : kDebugViews[i].view;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", kDebugViews[i].tooltip);
        if (active)
            ImGui::PopStyleColor();
    }

    ImGui::SameLine();
    {
        const bool wasShowSkeleton = mShowSkeleton;
        if (wasShowSkeleton)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(ICON_MDI_BONE))
            mShowSkeleton = !mShowSkeleton;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Show Skeleton");
        if (wasShowSkeleton)
            ImGui::PopStyleColor();
    }

    ImGui::SameLine();
    ImGui::Dummy(ImVec2(10.0f, 0.0f));
    ImGui::SameLine();

    {
        const bool wasSnap = mSnap;
        if (wasSnap)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(ICON_MDI_MAGNET))
            mSnap = !mSnap;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Snap (not wired to a transform yet)");
        if (wasSnap)
            ImGui::PopStyleColor();
    }

    ImGui::SameLine();
    {
        const bool wasShowGrid = mShowGrid;
        if (wasShowGrid)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(ICON_MDI_GRID))
            mShowGrid = !mShowGrid;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Grid");
        if (wasShowGrid)
            ImGui::PopStyleColor();
    }

    ImGui::PopStyleVar(2);
}

namespace
{
ViewportPanel::ViewMode oppositeViewMode(ViewportPanel::ViewMode mode)
{
    using ViewMode = ViewportPanel::ViewMode;
    switch (mode)
    {
    case ViewMode::Top: return ViewMode::Bottom;
    case ViewMode::Bottom: return ViewMode::Top;
    case ViewMode::Front: return ViewMode::Back;
    case ViewMode::Back: return ViewMode::Front;
    case ViewMode::Left: return ViewMode::Right;
    case ViewMode::Right: return ViewMode::Left;
    default: return mode;
    }
}
} // namespace

void ViewportPanel::drawViewModeMenu(usize viewportIndex)
{
    const char* viewNames[] = {"Perspective", "Top", "Bottom", "Front", "Back", "Left", "Right"};

    for (int i = 0; i < 7; ++i)
    {
        if (ImGui::MenuItem(viewNames[i], nullptr, (int)mViewModes[viewportIndex] == i))
            setViewMode(viewportIndex, (ViewMode)i);
    }
}

void ViewportPanel::drawSingleView()
{
    ImVec2 size = ImGui::GetContentRegionAvail();
    if (ImGui::BeginChild("##viewport0", size, false, ImGuiWindowFlags_NoScrollWithMouse))
        drawViewportWindow(0, "##viewport0", mViewModes[0]);
    ImGui::EndChild();
}

void ViewportPanel::drawThreeWayLayout()
{
    ImVec2 availSize = ImGui::GetContentRegionAvail();
    f32 width = availSize.x;
    f32 height = availSize.y;

    if (ImGui::BeginChild("##viewport_top", ImVec2(width, height * 0.5f), false,
                          ImGuiWindowFlags_NoScrollWithMouse))
        drawViewportWindow(0, "##viewport_top", mViewModes[0]);
    ImGui::EndChild();

    if (ImGui::BeginChild("##viewport_left", ImVec2(width * 0.5f, height * 0.5f), false,
                          ImGuiWindowFlags_NoScrollWithMouse))
        drawViewportWindow(1, "##viewport_left", mViewModes[1]);
    ImGui::EndChild();

    ImGui::SameLine();

    if (ImGui::BeginChild("##viewport_right", ImVec2(width * 0.5f, height * 0.5f), false,
                          ImGuiWindowFlags_NoScrollWithMouse))
        drawViewportWindow(2, "##viewport_right", mViewModes[2]);
    ImGui::EndChild();
}

void ViewportPanel::drawFourWayLayout()
{
    ImVec2 availSize = ImGui::GetContentRegionAvail();
    f32 halfWidth = availSize.x * 0.5f;
    f32 halfHeight = availSize.y * 0.5f;

    if (ImGui::BeginChild("##viewport_topleft", ImVec2(halfWidth, halfHeight), false,
                          ImGuiWindowFlags_NoScrollWithMouse))
        drawViewportWindow(0, "##viewport_topleft", mViewModes[0]);
    ImGui::EndChild();

    ImGui::SameLine();

    if (ImGui::BeginChild("##viewport_topright", ImVec2(halfWidth, halfHeight), false,
                          ImGuiWindowFlags_NoScrollWithMouse))
        drawViewportWindow(1, "##viewport_topright", mViewModes[1]);
    ImGui::EndChild();

    if (ImGui::BeginChild("##viewport_bottomleft", ImVec2(halfWidth, halfHeight), false,
                          ImGuiWindowFlags_NoScrollWithMouse))
        drawViewportWindow(2, "##viewport_bottomleft", mViewModes[2]);
    ImGui::EndChild();

    ImGui::SameLine();

    if (ImGui::BeginChild("##viewport_bottomright", ImVec2(halfWidth, halfHeight), false,
                          ImGuiWindowFlags_NoScrollWithMouse))
        drawViewportWindow(3, "##viewport_bottomright", mViewModes[3]);
    ImGui::EndChild();
}

// Same spherical-offset camera EditorApplication's own ViewportPanel uses
// (editor/src/panels/ViewportPanel.cpp: updateNavigation()/forward's own
// sin/cos build), ported without the GameObject it normally writes into:
// Alt+LMB orbits, MMB pans, RMB looks (perspective only), wheel zooms.
void ViewportPanel::updateCameraNavigation(usize index, CameraState& camera, ViewMode mode)
{
    ImGuiIO& io = ImGui::GetIO();
    const bool hovered = ImGui::IsWindowHovered();
    const bool perspective = mode == ViewMode::Perspective;

    if (hovered && perspective && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && io.KeyAlt)
    {
        mOrbiting = true;
        mActiveViewport = static_cast<s32>(index);
    }
    if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
    {
        mPanning = true;
        mActiveViewport = static_cast<s32>(index);
    }
    if (hovered && perspective && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
    {
        mLooking = true;
        mActiveViewport = static_cast<s32>(index);
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
        mOrbiting = false;
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Middle))
        mPanning = false;
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right))
        mLooking = false;

    const bool active = mActiveViewport == static_cast<s32>(index);

    if (active && (mOrbiting || mLooking))
    {
        camera.yaw -= io.MouseDelta.x * 0.005f;
        camera.pitch -= io.MouseDelta.y * 0.005f;
        camera.pitch = Math::clamp(camera.pitch, -1.5f, 1.5f);
    }
    if (active && mPanning)
    {
        const Math::vec3 right(Math::cos(camera.yaw), 0.0f, Math::sin(camera.yaw));
        const Math::vec3 up(0.0f, 1.0f, 0.0f);
        const f32 scale = camera.distance * 0.001f;
        camera.target -= right * (io.MouseDelta.x * scale);
        camera.target += up * (io.MouseDelta.y * scale);
    }
    if (hovered && io.MouseWheel != 0.0f)
    {
        camera.distance -= io.MouseWheel * camera.distance * 0.15f;
        camera.distance = Math::clamp(camera.distance, 0.1f, 1000.0f);
    }
}

void ViewportPanel::computeMatrices(const CameraState& camera, ViewMode mode, f32 aspect,
                                    Math::mat4& view, Math::mat4& projection,
                                    Math::vec3& cameraPos) const
{
    constexpr f32 kNear = 0.05f;
    constexpr f32 kFar = 1000.0f;

    if (mode == ViewMode::Perspective)
    {
        const Math::vec3 forward(Math::sin(camera.yaw) * Math::cos(camera.pitch),
                                Math::sin(camera.pitch),
                                -Math::cos(camera.yaw) * Math::cos(camera.pitch));
        cameraPos = camera.target - forward * camera.distance;
        view = Math::lookAt(cameraPos, camera.target, Math::vec3(0.0f, 1.0f, 0.0f));
        projection = Math::perspective(Math::radians(60.0f), aspect, kNear, kFar);
        return;
    }

    Math::vec3 offset(0.0f);
    Math::vec3 up(0.0f, 1.0f, 0.0f);
    switch (mode)
    {
    case ViewMode::Top: offset = Math::vec3(0.0f, camera.distance, 0.0f); up = Math::vec3(0.0f, 0.0f, -1.0f); break;
    case ViewMode::Bottom: offset = Math::vec3(0.0f, -camera.distance, 0.0f); up = Math::vec3(0.0f, 0.0f, 1.0f); break;
    case ViewMode::Front: offset = Math::vec3(0.0f, 0.0f, camera.distance); break;
    case ViewMode::Back: offset = Math::vec3(0.0f, 0.0f, -camera.distance); break;
    case ViewMode::Left: offset = Math::vec3(-camera.distance, 0.0f, 0.0f); break;
    case ViewMode::Right: offset = Math::vec3(camera.distance, 0.0f, 0.0f); break;
    default: break;
    }
    cameraPos = camera.target + offset;
    view = Math::lookAt(cameraPos, camera.target, up);
    const f32 halfHeight = camera.distance * 0.5f;
    const f32 halfWidth = halfHeight * aspect;
    projection = Math::ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, kNear, kFar);
}

void ViewportPanel::drawViewportWindow(usize index, const char* name, ViewMode mode)
{
    (void)name;
    ImVec2 size = ImGui::GetContentRegionAvail();
    if (size.x < 1.0f || size.y < 1.0f)
        return;

    updateCameraNavigation(index, mCameras[index], mode);

    RenderTarget& target = mTargets[index];
    if (!target.ensure(static_cast<s32>(size.x), static_cast<s32>(size.y)))
    {
        ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.3f, 1.0f), "viewport target failed");
        return;
    }

    Math::mat4 view, projection;
    Math::vec3 cameraPos;
    computeMatrices(mCameras[index], mode, size.x / size.y, view, projection, cameraPos);

    GLint previousFbo = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previousFbo);

    glBindFramebuffer(GL_FRAMEBUFFER, target.fbo);
    glViewport(0, 0, target.width, target.height);
    glClearColor(0.145f, 0.145f, 0.145f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    MeshData* mesh = app().currentMeshData();
    std::vector<u8> submeshVisible;
    if (mesh)
    {
        MiniDrawParams params;
        params.mode = mShadingMode;
        params.colorBySubmesh = app().settings().viewport().colorBySubmesh;
        params.debugView = mDebugView;
        params.unlit = mUnlit;
        if (app().hasSkeleton() && !app().bonePalette().empty())
        {
            params.bonePalette = app().bonePalette().data();
            params.boneCount = static_cast<u32>(app().bonePalette().size());
        }

        submeshVisible.reserve(mesh->submeshes.size());
        for (u32 i = 0; i < static_cast<u32>(mesh->submeshes.size()); ++i)
            submeshVisible.push_back(app().isSubmeshVisible(i) ? 1 : 0);
        params.submeshVisible = submeshVisible.data();
        params.submeshVisibleCount = static_cast<u32>(submeshVisible.size());

        const BlenderSelection& selection = app().selection();
        if (selection.mode() == BlenderSelection::SelectionMode::Vertex)
        {
            const BlenderSettings::ViewportSettings& viewportSettings = app().settings().viewport();
            params.showVertexPoints = true;
            params.vertexColor = viewportSettings.vertexColor;
            params.selectedVertexColor = viewportSettings.selectedVertexColor;
            params.vertexPointSize = app().settings().general().vertexPointSize;
            uploadVertexSelection(*mesh, selection);
        }

        app().renderer().renderViewport(mesh, view, projection, cameraPos, params);
    }

    if (mShowGrid)
    {
        MiniBatch& batch = app().batch();
        batch.begin();
        batch.grid(0.0f, 20, 1.0f, true);
        batch.flush(projection * view);
    }

    if (mesh)
    {
        drawSelectionOverlay(mesh, projection * view);
        drawDebugVectorOverlay(mesh, projection * view);
    }
    if (mShowSkeleton)
        drawSkeletonOverlay(projection * view);

    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(previousFbo));

    ImGui::Image(static_cast<ImTextureID>(static_cast<u64>(target.colorTexture)), size, ImVec2(0.0f, 1.0f),
                ImVec2(1.0f, 0.0f));

    const char* viewModeNames[] = {"Persp", "Top", "Bottom", "Front", "Back", "Left", "Right"};
    ImVec2 overlayPos = ImGui::GetItemRectMin();

    if (mode != ViewMode::Perspective)
    {
        ImGui::PushID(static_cast<int>(index));
        ImGui::SetCursorScreenPos(ImVec2(overlayPos.x + 4.0f, overlayPos.y + 4.0f));
        if (ImGui::SmallButton(viewModeNames[(int)mode]))
            setViewMode(index, oppositeViewMode(mode));
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Switch to %s", viewModeNames[(int)oppositeViewMode(mode)]);
        ImGui::PopID();
    }
    else
    {
        ImGui::GetWindowDrawList()->AddText(ImVec2(overlayPos.x + 6.0f, overlayPos.y + 4.0f),
                                            IM_COL32(220, 220, 220, 200), viewModeNames[(int)mode]);
    }

    if (mode == ViewMode::Perspective && ImGui::IsWindowFocused())
        drawNavigationGizmo(mCameras[index], view, Math::vec2(overlayPos.x, overlayPos.y),
                            Math::vec2(size.x, size.y));

    drawTransformGizmo(index, mesh, view, projection, Math::vec2(overlayPos.x, overlayPos.y),
                       Math::vec2(size.x, size.y), mode != ViewMode::Perspective);

    updateSelectionInput(index, mesh, view, projection, cameraPos, Math::vec2(overlayPos.x, overlayPos.y),
                         Math::vec2(size.x, size.y), target);
}

void ViewportPanel::drawNavigationGizmo(CameraState& camera, const Math::mat4& view,
                                        const Math::vec2& imageMin, const Math::vec2& imageSize)
{
    constexpr f32 kGizmoSize = 90.0f;

    ImGuizmo::SetDrawlist();
    ImGuizmo::SetRect(imageMin.x, imageMin.y, imageSize.x, imageSize.y);

    Math::mat4 gizmoView = view;
    bool modified = false;
    ImGuizmo::ViewManipulate(Math::value_ptr(gizmoView), camera.distance,
                             ImVec2(imageMin.x + imageSize.x - kGizmoSize - 8.0f, imageMin.y + 8.0f),
                             ImVec2(kGizmoSize, kGizmoSize), 0, modified);
    if (!modified)
        return;

    const Math::mat4 cameraToWorld = Math::inverse(gizmoView);
    const Math::vec3 forward = Math::normalize(-Math::vec3(cameraToWorld[2]));
    camera.pitch = Math::asin(Math::clamp(forward.y, -1.0f, 1.0f));
    camera.yaw = Math::atan(forward.x, -forward.z);
}

void ViewportPanel::drawTransformGizmo(usize index, const MeshData* mesh, const Math::mat4& view,
                                       const Math::mat4& projection, const Math::vec2& imageMin,
                                       const Math::vec2& imageSize, bool orthographic)
{
    if (mTool == Tool::Select || !mesh || mesh->positions.empty())
    {
        if (mGizmoDragging && mGizmoViewport == static_cast<s32>(index))
        {
            app().endGizmoDrag();
            mGizmoDragging = false;
        }
        return;
    }

    // The gizmo follows the mouse from view to view but stays put once it is
    // there, rather than blinking out whenever the cursor leaves. A drag
    // holds its own view until released, so pulling the handle across a
    // neighbouring viewport does not hand the drag over mid-move.
    if (!mGizmoDragging && ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows))
        mGizmoViewport = static_cast<s32>(index);
    if (mGizmoViewport < 0)
        mGizmoViewport = static_cast<s32>(index);
    if (mGizmoViewport != static_cast<s32>(index))
        return;

    ImGuizmo::SetOrthographic(orthographic);
    ImGuizmo::SetDrawlist();
    ImGuizmo::SetRect(imageMin.x, imageMin.y, imageSize.x, imageSize.y);

    const ImGuizmo::OPERATION operation = mTool == Tool::Move     ? ImGuizmo::TRANSLATE
                                          : mTool == Tool::Rotate ? ImGuizmo::ROTATE
                                                                  : ImGuizmo::SCALE;

    // Between drags the gizmo sits on the pivot with no rotation or scale of
    // its own, so the matrix it hands back is exactly what the drag did.
    if (!mGizmoDragging)
        mGizmoMatrix = Math::translate(Math::mat4(1.0f), app().transformPivot());

    const f32 snapAmount = mTool == Tool::Move ? 1.0f : mTool == Tool::Rotate ? 15.0f : 0.1f;
    f32 snapValues[3] = {snapAmount, snapAmount, snapAmount};

    Math::mat4 manipulated = mGizmoMatrix;
    ImGuizmo::Manipulate(Math::value_ptr(view), Math::value_ptr(projection), operation,
                         ImGuizmo::WORLD, Math::value_ptr(manipulated), nullptr,
                         mSnap ? snapValues : nullptr);

    if (!ImGuizmo::IsUsing())
    {
        if (mGizmoDragging)
        {
            app().endGizmoDrag();
            mGizmoDragging = false;
        }
        return;
    }

    if (!mGizmoDragging)
    {
        if (!app().beginGizmoDrag())
            return;
        mGizmoDragging = true;
        mGizmoViewport = static_cast<s32>(index);
        mGizmoStartMatrix = mGizmoMatrix;
    }

    mGizmoMatrix = manipulated;
    app().updateGizmoDrag(manipulated * Math::inverse(mGizmoStartMatrix));
}

void ViewportPanel::uploadVertexSelection(const MeshData& mesh, const BlenderSelection& selection)
{
    const u32 vertexCount = static_cast<u32>(mesh.positions.size());
    if (vertexCount == 0)
        return;

    // Three things have to still hold for the GPU copy to be current: the
    // same mesh, the same selection, and no re-upload since - an edit in
    // place keeps the pointer and the selection while replacing the buffer
    // with a zeroed one, which would otherwise blank the highlight.
    MiniRenderer& renderer = app().renderer();
    if (&mesh == mUploadedSelectionMesh && selection.revision() == mUploadedSelectionRevision &&
        renderer.meshUploadRevision() == mUploadedMeshRevision &&
        mVertexSelectionFlags.size() == vertexCount)
        return;

    mVertexSelectionFlags.resize(vertexCount);
    selection.fillVertexFlags(mVertexSelectionFlags.data(), vertexCount);
    renderer.setVertexSelection(mVertexSelectionFlags.data(), vertexCount);

    mUploadedSelectionRevision = selection.revision();
    mUploadedMeshRevision = renderer.meshUploadRevision();
    mUploadedSelectionMesh = &mesh;
}

void ViewportPanel::drawSelectionOverlay(const MeshData* mesh, const Math::mat4& viewProjection)
{
    if (!mesh || mesh->positions.empty())
        return;

    BlenderSelection& selection = app().selection();
    const BlenderSettings::ViewportSettings& viewportSettings = app().settings().viewport();
    const s32 selectedSubmesh = app().selectedSubmesh();

    const bool drawVertexFace = selection.mode() == BlenderSelection::SelectionMode::Vertex ||
                                selection.mode() == BlenderSelection::SelectionMode::Face;
    const bool drawSubmesh =
        selectedSubmesh >= 0 && static_cast<usize>(selectedSubmesh) < mesh->submeshes.size();
    if (!drawVertexFace && !drawSubmesh)
        return;

    MiniBatch& batch = app().batch();
    batch.begin();

    if (drawSubmesh)
    {
        const SubMesh& submesh = mesh->submeshes[static_cast<usize>(selectedSubmesh)];
        const Math::vec4 highlight(viewportSettings.submeshHighlightColor,
                                  viewportSettings.submeshHighlightAlpha);
        const u32 end = submesh.indexOffset + submesh.indexCount;
        for (u32 i = submesh.indexOffset; i + 2 < end && i + 2 < mesh->indices.size(); i += 3)
        {
            const u32 i0 = mesh->indices[i];
            const u32 i1 = mesh->indices[i + 1];
            const u32 i2 = mesh->indices[i + 2];
            if (i0 >= mesh->positions.size() || i1 >= mesh->positions.size() ||
                i2 >= mesh->positions.size())
                continue;
            batch.triangle(mesh->positions[i0], mesh->positions[i1], mesh->positions[i2], highlight);
        }
    }

    // Vertex points are no longer batched here: MiniRenderer draws them from
    // the static vertex buffer in one call, with the selection as a stream of
    // its own, so a 150k vertex mesh costs nothing per frame.
    if (drawVertexFace && selection.mode() == BlenderSelection::SelectionMode::Face)
    {
        const Math::vec4 highlight(viewportSettings.faceHighlightColor, viewportSettings.faceHighlightAlpha);
        const Math::vec4 edgeHighlight(viewportSettings.faceEdgeHighlightColor, 1.0f);
        // Walking the selection, not every face in the mesh: the cost belongs
        // to what is highlighted, not to how big the model is.
        const std::vector<u32>& selectedFaces = selection.selectedFaces();
        const u32 faceCount = static_cast<u32>(mesh->indices.size() / 3);
        for (usize s = 0; s < selectedFaces.size(); ++s)
        {
            const u32 face = selectedFaces[s];
            if (face >= faceCount)
                continue;
            const u32 i0 = mesh->indices[face * 3 + 0];
            const u32 i1 = mesh->indices[face * 3 + 1];
            const u32 i2 = mesh->indices[face * 3 + 2];
            if (i0 >= mesh->positions.size() || i1 >= mesh->positions.size() ||
                i2 >= mesh->positions.size())
                continue;
            const Math::vec3& p0 = mesh->positions[i0];
            const Math::vec3& p1 = mesh->positions[i1];
            const Math::vec3& p2 = mesh->positions[i2];
            batch.triangle(p0, p1, p2, highlight);
            batch.line(p0, p1, edgeHighlight);
            batch.line(p1, p2, edgeHighlight);
            batch.line(p2, p0, edgeHighlight);
        }
    }

    batch.flush(viewProjection);
}

void ViewportPanel::drawDebugVectorOverlay(const MeshData* mesh, const Math::mat4& viewProjection)
{
    if (!mesh || mesh->positions.empty())
        return;
    if (mDebugView != MiniDebugView::Normals && mDebugView != MiniDebugView::Tangents)
        return;

    const bool showNormals = mDebugView == MiniDebugView::Normals;
    if (showNormals && mesh->normals.size() != mesh->positions.size())
        return;
    if (!showNormals && mesh->tangents.size() != mesh->positions.size())
        return;

    const BlenderSettings::ViewportSettings& viewportSettings = app().settings().viewport();
    const Math::vec4 color(showNormals ? viewportSettings.normalVectorColor
                                      : viewportSettings.tangentVectorColor,
                          1.0f);
    const f32 length = viewportSettings.debugVectorLength;

    MiniBatch& batch = app().batch();
    batch.begin();
    for (usize i = 0; i < mesh->positions.size(); ++i)
    {
        const Math::vec3& origin = mesh->positions[i];
        const Math::vec3 direction =
            showNormals ? mesh->normals[i] : Math::vec3(mesh->tangents[i]);
        batch.line(origin, origin + direction * length, color);
    }
    batch.flush(viewProjection);
}

void ViewportPanel::drawSkeletonOverlay(const Math::mat4& viewProjection)
{
    if (!app().hasSkeleton())
        return;

    const Skeleton& skeleton = app().skeleton();
    const std::vector<Math::mat4>& globalPose = app().globalPose();
    if (globalPose.size() != skeleton.boneCount())
        return;

    MiniBatch& batch = app().batch();
    batch.begin();

    const Math::vec4 boneColor(0.2f, 1.0f, 0.4f, 1.0f);
    const Math::vec4 jointColor(1.0f, 1.0f, 0.4f, 1.0f);
    for (u32 i = 0; i < skeleton.boneCount(); ++i)
    {
        const Math::vec3 jointPos(globalPose[i][3]);
        batch.point(jointPos, jointColor, 6.0f);

        const s32 parent = skeleton.bone(i).parent;
        if (parent >= 0 && static_cast<u32>(parent) < globalPose.size())
            batch.line(Math::vec3(globalPose[static_cast<u32>(parent)][3]), jointPos, boneColor);
    }

    batch.flush(viewProjection);
}

namespace
{
::Radion::Math::vec2 projectToScreen(const ::Radion::Math::vec3& worldPos,
                                     const ::Radion::Math::mat4& viewProjection,
                                     const ::Radion::Math::vec2& imageMin,
                                     const ::Radion::Math::vec2& imageSize, bool& inFront)
{
    const ::Radion::Math::vec4 clip = viewProjection * ::Radion::Math::vec4(worldPos, 1.0f);
    inFront = clip.w > 0.0001f;
    if (!inFront)
        return ::Radion::Math::vec2(0.0f);

    const ::Radion::Math::vec2 ndc(clip.x / clip.w, clip.y / clip.w);
    return ::Radion::Math::vec2(imageMin.x + (ndc.x * 0.5f + 0.5f) * imageSize.x,
                                imageMin.y + (1.0f - (ndc.y * 0.5f + 0.5f)) * imageSize.y);
}
} // namespace

// Unprojects the depth buffer's own value at the candidate's screen pixel
// back into a world-space point (the standard "read depth, rebuild world
// position" trick), then compares distance-from-camera against the
// candidate's own distance - projection-agnostic (works the same for the
// perspective view and the orthographic Top/Front/etc ones), unlike
// comparing raw window-space depth directly, whose non-linear precision
// made every candidate read as "in front" regardless of true occlusion.
bool ViewportPanel::readDepthRect(const RenderTarget& target, const Math::vec2& localMin,
                                  const Math::vec2& localMax, DepthRect& out)
{
    out.depth.clear();
    out.width = 0;
    out.height = 0;

    if (target.width <= 0 || target.height <= 0)
        return false;

    // The rect arrives in the image's top-left origin; GL reads bottom-up.
    const s32 x0 = static_cast<s32>(Math::floor(localMin.x));
    const s32 x1 = static_cast<s32>(Math::ceil(localMax.x));
    const s32 yTop = static_cast<s32>(Math::floor(localMin.y));
    const s32 yBottom = static_cast<s32>(Math::ceil(localMax.y));

    const s32 x = Math::max(0, x0);
    const s32 y = Math::max(0, target.height - 1 - yBottom);
    const s32 right = Math::min(target.width, x1 + 1);
    const s32 top = Math::min(target.height, target.height - yTop);

    if (right <= x || top <= y)
        return false;

    out.x = x;
    out.y = y;
    out.width = right - x;
    out.height = top - y;
    out.depth.resize(static_cast<usize>(out.width) * static_cast<usize>(out.height), 1.0f);

    GLint previousFbo = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previousFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, target.fbo);
    glReadPixels(out.x, out.y, out.width, out.height, GL_DEPTH_COMPONENT, GL_FLOAT,
                 out.depth.data());
    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(previousFbo));

    return true;
}

bool ViewportPanel::isScreenPointVisible(const DepthRect& depthRect, const Math::vec2& localPoint,
                                         const Math::vec3& worldPos,
                                         const Math::mat4& inverseViewProjection,
                                         const Math::vec3& cameraPos, const RenderTarget& target)
{
    const s32 px = static_cast<s32>(localPoint.x);
    const s32 py = target.height - 1 - static_cast<s32>(localPoint.y);
    if (px < 0 || py < 0 || px >= target.width || py >= target.height)
        return false;

    const s32 localX = px - depthRect.x;
    const s32 localY = py - depthRect.y;
    if (localX < 0 || localY < 0 || localX >= depthRect.width || localY >= depthRect.height)
        return false;

    const f32 bufferDepth =
        depthRect.depth[static_cast<usize>(localY) * static_cast<usize>(depthRect.width) +
                        static_cast<usize>(localX)];

    if (bufferDepth >= 1.0f)
        return true;

    const f32 ndcX = (localPoint.x / static_cast<f32>(target.width)) * 2.0f - 1.0f;
    const f32 ndcY = 1.0f - (localPoint.y / static_cast<f32>(target.height)) * 2.0f;
    const f32 ndcZ = bufferDepth * 2.0f - 1.0f;

    Math::vec4 surface4 = inverseViewProjection * Math::vec4(ndcX, ndcY, ndcZ, 1.0f);
    surface4 /= surface4.w;

    const f32 surfaceDistance = Math::length(Math::vec3(surface4) - cameraPos);
    const f32 candidateDistance = Math::length(worldPos - cameraPos);

    constexpr f32 kEpsilon = 0.02f;
    return candidateDistance <= surfaceDistance + kEpsilon;
}

void ViewportPanel::updateSelectionInput(usize index, const MeshData* mesh, const Math::mat4& view,
                                         const Math::mat4& projection, const Math::vec3& cameraPos,
                                         const Math::vec2& imageMin, const Math::vec2& imageSize,
                                         const RenderTarget& target)
{
    if (!mesh || mesh->positions.empty() || mTool != Tool::Select)
        return;

    BlenderSelection& selection = app().selection();
    if (selection.mode() == BlenderSelection::SelectionMode::Edge)
        return;

    ImGuiIO& io = ImGui::GetIO();
    const bool hovered = ImGui::IsWindowHovered();

    if (hovered && !io.KeyAlt && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        mBoxSelecting = true;
        mBoxSelectViewport = static_cast<s32>(index);
        mBoxSelectStart = Math::vec2(io.MousePos.x, io.MousePos.y);
    }

    if (!mBoxSelecting || mBoxSelectViewport != static_cast<s32>(index))
        return;

    const Math::vec2 current(io.MousePos.x, io.MousePos.y);
    const f32 dragDistance = Math::length(current - mBoxSelectStart);
    const bool isBox = dragDistance > 4.0f;

    if (isBox)
    {
        const Math::vec3& boxColor = app().settings().viewport().boxSelectColor;
        const Color selectionColor = Color::fromRGBFloat(boxColor.x, boxColor.y, boxColor.z);
        const ImU32 fillColor = IM_COL32(selectionColor.r(), selectionColor.g(), selectionColor.b(), 35);
        const ImU32 lineColor = IM_COL32(selectionColor.r(), selectionColor.g(), selectionColor.b(), 220);
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        const ImVec2 a(mBoxSelectStart.x, mBoxSelectStart.y);
        const ImVec2 b(current.x, current.y);
        drawList->AddRectFilled(a, b, fillColor);
        drawList->AddRect(a, b, lineColor, 0.0f, 0, 1.5f);
    }

    if (!ImGui::IsMouseReleased(ImGuiMouseButton_Left))
        return;

    mBoxSelecting = false;
    mBoxSelectViewport = -1;

    if (!io.KeyShift)
        selection.clearAll();

    const Math::mat4 viewProjection = projection * view;
    const Math::mat4 inverseViewProjection = Math::inverse(viewProjection);
    const Math::vec2 rectMin(Math::min(mBoxSelectStart.x, current.x), Math::min(mBoxSelectStart.y, current.y));
    const Math::vec2 rectMax(Math::max(mBoxSelectStart.x, current.x), Math::max(mBoxSelectStart.y, current.y));

    // One read for the whole operation, before any candidate is tested. A box
    // covers the dragged rectangle; a click tests only the winner, but that
    // winner is the nearest candidate within the pick radius below (10px for
    // a vertex, 14 for a face), so the read has to reach that far from the
    // cursor or the test falls outside the rect and rejects everything.
    std::vector<bool> faceSelectable;
    std::vector<bool> vertexSelectable;
    app().buildSelectableMask(faceSelectable, vertexSelectable);

    constexpr f32 kPickRadius = 16.0f;
    DepthRect depthRect;
    if (mSelectVisibleOnly)
    {
        const Math::vec2 readMin =
            isBox ? rectMin - imageMin : current - imageMin - Math::vec2(kPickRadius);
        const Math::vec2 readMax =
            isBox ? rectMax - imageMin : current - imageMin + Math::vec2(kPickRadius);
        readDepthRect(target, readMin, readMax, depthRect);
    }

    if (selection.mode() == BlenderSelection::SelectionMode::Vertex)
    {
        u32 bestIndex = 0;
        f32 bestDistance = 1e9f;
        Math::vec2 bestScreen(0.0f);
        bool found = false;

        for (u32 i = 0; i < static_cast<u32>(mesh->positions.size()); ++i)
        {
            if (i >= vertexSelectable.size() || !vertexSelectable[i])
                continue;

            bool inFront = false;
            const Math::vec2 screen = projectToScreen(mesh->positions[i], viewProjection, imageMin,
                                                     imageSize, inFront);
            if (!inFront)
                continue;

            if (isBox)
            {
                if (screen.x >= rectMin.x && screen.x <= rectMax.x && screen.y >= rectMin.y &&
                    screen.y <= rectMax.y &&
                    (!mSelectVisibleOnly ||
                     isScreenPointVisible(depthRect, screen - imageMin, mesh->positions[i],
                                          inverseViewProjection, cameraPos, target)))
                    selection.selectVertex(i);
            }
            else
            {
                const f32 distance = Math::length(screen - current);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestIndex = i;
                    bestScreen = screen;
                    found = true;
                }
            }
        }

        if (!isBox && found && bestDistance <= 10.0f &&
            (!mSelectVisibleOnly ||
             isScreenPointVisible(depthRect, bestScreen - imageMin, mesh->positions[bestIndex],
                                  inverseViewProjection, cameraPos, target)))
            selection.selectVertex(bestIndex);
    }
    else
    {
        const u32 faceCount = static_cast<u32>(mesh->indices.size() / 3);
        u32 bestFace = 0;
        bool found = false;

        if (!isBox)
        {
            // A click picks the triangle the cursor is actually over, not the
            // one whose centroid happens to be nearest. Comparing centroids
            // means a big face has to be clicked near its middle, and a small
            // one next to it wins anywhere else - on a floor made of two huge
            // triangles, most of the floor selects nothing at all.
            //
            // The nearest hit along the ray is the one in front, so this needs
            // no separate occlusion test: the depth read is for box select.
            const Ray ray = rayFromScreen(current.x - imageMin.x, current.y - imageMin.y,
                                          imageSize.x, imageSize.y, inverseViewProjection);

            f32 nearest = 0.0f;
            for (u32 face = 0; face < faceCount; ++face)
            {
                if (face >= faceSelectable.size() || !faceSelectable[face])
                    continue;

                const u32 i0 = mesh->indices[face * 3 + 0];
                const u32 i1 = mesh->indices[face * 3 + 1];
                const u32 i2 = mesh->indices[face * 3 + 2];
                if (i0 >= mesh->positions.size() || i1 >= mesh->positions.size() ||
                    i2 >= mesh->positions.size())
                    continue;

                f32 distance = 0.0f;
                if (!ray.intersects(mesh->positions[i0], mesh->positions[i1], mesh->positions[i2],
                                    distance))
                    continue;

                if (!found || distance < nearest)
                {
                    nearest = distance;
                    bestFace = face;
                    found = true;
                }
            }

            if (found)
                selection.selectFace(bestFace);
            return;
        }

        for (u32 face = 0; face < faceCount; ++face)
        {
            if (face >= faceSelectable.size() || !faceSelectable[face])
                continue;

            const u32 i0 = mesh->indices[face * 3 + 0];
            const u32 i1 = mesh->indices[face * 3 + 1];
            const u32 i2 = mesh->indices[face * 3 + 2];
            if (i0 >= mesh->positions.size() || i1 >= mesh->positions.size() ||
                i2 >= mesh->positions.size())
                continue;

            const Math::vec3 centroid =
                (mesh->positions[i0] + mesh->positions[i1] + mesh->positions[i2]) / 3.0f;
            bool inFront = false;
            const Math::vec2 screen = projectToScreen(centroid, viewProjection, imageMin, imageSize, inFront);
            if (!inFront)
                continue;

            if (screen.x >= rectMin.x && screen.x <= rectMax.x && screen.y >= rectMin.y &&
                screen.y <= rectMax.y &&
                (!mSelectVisibleOnly ||
                 isScreenPointVisible(depthRect, screen - imageMin, centroid,
                                      inverseViewProjection, cameraPos, target)))
                selection.selectFace(face);
        }
    }
}
