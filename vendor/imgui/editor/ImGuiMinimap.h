#ifndef RADION_IMGUI_MINIMAP_H
#define RADION_IMGUI_MINIMAP_H

#include "TextEditor.h"

#include <algorithm>

namespace ImGuiMinimap
{

inline void Render(const char* id, TextEditor& editor, float width, float height, ImFont* font)
{
    (void)font;
    ImGui::BeginChild(id, ImVec2(width, height), true,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 region = ImGui::GetContentRegionAvail();
    const int totalLines = editor.GetLineCount();
    if (totalLines <= 0)
    {
        ImGui::EndChild();
        return;
    }

    const float lineHeight = std::max(1.0f, std::min(3.0f, region.y / static_cast<float>(totalLines)));
    constexpr float charWidth = 1.2f;
    const int maxChars = static_cast<int>(region.x / charWidth);
    const float totalHeight = lineHeight * totalLines;
    float scrollOffset = 0.0f;
    if (totalHeight > region.y)
    {
        const float progress = static_cast<float>(editor.GetFirstVisibleLine()) /
                               static_cast<float>(std::max(1, totalLines - 1));
        scrollOffset = progress * (totalHeight - region.y);
    }

    const int firstLine = std::max(0, static_cast<int>(scrollOffset / lineHeight) - 1);
    const int lastLine = std::min(totalLines,
                                  firstLine + static_cast<int>(region.y / lineHeight) + 2);
    for (int line = firstLine; line < lastLine; ++line)
    {
        const float y = origin.y + line * lineHeight - scrollOffset;
        if (y + lineHeight < origin.y || y > origin.y + region.y)
            continue;

        const int chars = std::min(editor.GetLineLengthRaw(line), maxChars);
        const int step = std::max(1, chars / 40);
        for (int column = 0; column < chars; column += step)
        {
            const ImU32 color = (editor.GetLineGlyphColor(line, column) & 0x00FFFFFF) | 0xCC000000;
            const float x = origin.x + column * charWidth;
            const float blockWidth = std::min(charWidth * step, region.x - column * charWidth);
            drawList->AddRectFilled(ImVec2(x, y), ImVec2(x + blockWidth, y + lineHeight), color);
        }
    }

    const float visibleStart = origin.y + editor.GetFirstVisibleLine() * lineHeight - scrollOffset;
    const float visibleEnd = origin.y + (editor.GetLastVisibleLine() + 1) * lineHeight - scrollOffset;
    const ImVec2 highlightMin(origin.x, std::max(visibleStart, origin.y));
    const ImVec2 highlightMax(origin.x + region.x, std::min(visibleEnd, origin.y + region.y));
    drawList->AddRectFilled(highlightMin, highlightMax, IM_COL32(255, 255, 255, 30));
    drawList->AddRect(highlightMin, highlightMax, IM_COL32(255, 255, 255, 60));

    if (ImGui::IsWindowHovered() && ImGui::IsMouseDown(ImGuiMouseButton_Left))
    {
        const float relativeY = ImGui::GetMousePos().y - origin.y + scrollOffset;
        const int target = std::clamp(static_cast<int>(relativeY / lineHeight), 0, totalLines - 1);
        editor.SetViewAtLine(target, TextEditor::SetViewAtLineMode::Centered);
        editor.SetCursorPosition(target, 0);
    }

    ImGui::EndChild();
}

} // namespace ImGuiMinimap

#endif // RADION_IMGUI_MINIMAP_H
