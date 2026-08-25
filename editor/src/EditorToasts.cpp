#include "PCH.h"

#include "EditorToasts.h"

#include <IconsMaterialDesignIcons.h>
#include <imgui.h>

namespace Radion
{

namespace
{

constexpr f32 kLifetimeSeconds = 4.0f;
constexpr f32 kFadeSeconds = 0.4f;
constexpr usize kMaxVisible = 6;

ImVec4 colorFor(ToastKind kind)
{
    switch (kind)
    {
    case ToastKind::Success: return ImVec4(0.35f, 0.80f, 0.45f, 1.0f);
    case ToastKind::Warning: return ImVec4(1.00f, 0.75f, 0.25f, 1.0f);
    case ToastKind::Error:   return ImVec4(0.95f, 0.40f, 0.40f, 1.0f);
    case ToastKind::Info:    break;
    }
    return ImVec4(0.70f, 0.80f, 0.95f, 1.0f);
}

const char* iconFor(ToastKind kind)
{
    switch (kind)
    {
    case ToastKind::Success: return ICON_MDI_CHECK_CIRCLE;
    case ToastKind::Warning: return ICON_MDI_ALERT;
    case ToastKind::Error:   return ICON_MDI_CLOSE_CIRCLE;
    case ToastKind::Info:    break;
    }
    return ICON_MDI_INFORMATION;
}

} // namespace

void EditorToasts::push(ToastKind kind, std::string message)
{
    if (message.empty())
        return;
    // The same message arriving again refreshes the one on screen instead of
    // stacking a duplicate - a bake that reports per step would otherwise
    // push the corner off the screen.
    for (Toast& toast : mToasts)
    {
        if (toast.kind == kind && toast.message == message)
        {
            toast.remaining = kLifetimeSeconds;
            return;
        }
    }
    Toast toast;
    toast.kind = kind;
    toast.message = std::move(message);
    toast.remaining = kLifetimeSeconds;
    toast.total = kLifetimeSeconds;
    mToasts.push_back(std::move(toast));
    if (mToasts.size() > kMaxVisible)
        mToasts.erase(mToasts.begin());
}

void EditorToasts::info(std::string message)
{
    push(ToastKind::Info, std::move(message));
}
void EditorToasts::success(std::string message)
{
    push(ToastKind::Success, std::move(message));
}
void EditorToasts::warning(std::string message)
{
    push(ToastKind::Warning, std::move(message));
}
void EditorToasts::error(std::string message)
{
    push(ToastKind::Error, std::move(message));
}

void EditorToasts::update(f32 deltaTime)
{
    for (usize i = mToasts.size(); i > 0; --i)
    {
        Toast& toast = mToasts[i - 1];
        toast.remaining -= deltaTime;
        if (toast.remaining <= 0.0f)
            mToasts.erase(mToasts.begin() + static_cast<std::ptrdiff_t>(i - 1));
    }
}

void EditorToasts::clear()
{
    mToasts.clear();
}

void EditorToasts::draw()
{
    if (mToasts.empty())
        return;

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const f32 margin = 12.0f;
    f32 y = viewport->WorkPos.y + viewport->WorkSize.y - margin;

    for (usize i = mToasts.size(); i > 0; --i)
    {
        const Toast& toast = mToasts[i - 1];
        const f32 alpha = glm::min(toast.remaining / kFadeSeconds, 1.0f);

        ImGui::SetNextWindowBgAlpha(0.85f * alpha);
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - margin, y),
                                ImGuiCond_Always, ImVec2(1.0f, 1.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);

        char name[32];
        std::snprintf(name, sizeof(name), "##toast%zu", i);
        const ImGuiWindowFlags flags =
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoDocking;
        if (ImGui::Begin(name, nullptr, flags))
        {
            ImGui::TextColored(colorFor(toast.kind), "%s", iconFor(toast.kind));
            ImGui::SameLine();
            ImGui::TextUnformatted(toast.message.c_str());
            y -= ImGui::GetWindowSize().y + 6.0f;
        }
        ImGui::End();
        ImGui::PopStyleVar();
    }
}

} // namespace Radion
