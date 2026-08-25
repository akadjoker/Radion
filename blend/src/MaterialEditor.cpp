#include "PCH.h"
#include "MaterialEditor.h"
#include "AssetManager.h"
#include "GPU.h"
#include "IconsMaterialDesignIcons.h"
#include "Material.h"

#include <imgui.h>

using namespace Radion;

namespace
{
// True if `material` changed - same convention drawTextureSlot() uses.
bool drawFlagCheckbox(const char* label, MaterialFlags flag, Material& material)
{
    bool value = (material.flags & flag) != 0;
    if (!ImGui::Checkbox(label, &value))
        return false;
    material.flags = value ? (material.flags | flag) : (material.flags & ~static_cast<u32>(flag));
    return true;
}
} // namespace

bool MaterialEditor::drawFields(Material& material)
{
    bool changed = false;

    static const char* kBlendNames[] = {"Opaque",  "Alpha",     "Additive",   "Multiply",
                                        "Premultiplied Alpha", "Add Colors", "Subtract Colors"};
    int blend = static_cast<int>(material.blend);
    if (ImGui::Combo("Blend Mode", &blend, kBlendNames, IM_ARRAYSIZE(kBlendNames)))
    {
        material.blend = static_cast<BlendMode>(blend);
        changed = true;
    }

    static const char* kCullNames[] = {"None", "Back", "Front"};
    int cull = static_cast<int>(material.cull);
    ImGui::BeginDisabled((material.flags & MaterialTwoSided) != 0);
    if (ImGui::Combo("Cull Mode", &cull, kCullNames, IM_ARRAYSIZE(kCullNames)))
    {
        material.cull = static_cast<CullMode>(cull);
        changed = true;
    }
    ImGui::EndDisabled();

    bool depthWrite = (material.flags & MaterialNoDepthWrite) == 0;
    if (ImGui::Checkbox("Depth Write", &depthWrite))
    {
        material.flags = depthWrite ? (material.flags & ~static_cast<u32>(MaterialNoDepthWrite))
                                    : (material.flags | MaterialNoDepthWrite);
        changed = true;
    }

    ImGui::Spacing();
    changed |= ImGui::ColorEdit3("Base Color", &material.params.baseColor.x);
    changed |= ImGui::DragFloat("Roughness", &material.params.surface.x, 0.01f, 0.0f, 1.0f);
    changed |= ImGui::DragFloat("Metallic", &material.params.surface.y, 0.01f, 0.0f, 1.0f);
    if (material.params.surface.w <= 0.0f)
        material.params.surface.w = 1.0f;
    changed |= ImGui::DragFloat("Normal Strength", &material.params.surface.w, 0.01f, 0.0f, 4.0f);

    changed |= ImGui::ColorEdit3("Emissive", &material.params.emissive.x);
    changed |= ImGui::SliderFloat("Glow Strength", &material.params.emissive.w, 0.0f, 12.0f, "%.1f");

    ImGui::Spacing();
    changed |= drawFlagCheckbox("Two Sided", MaterialTwoSided, material);
    ImGui::SameLine();
    changed |= drawFlagCheckbox("Alpha Test", MaterialAlphaTest, material);
    if (material.flags & MaterialAlphaTest)
        changed |= ImGui::SliderFloat("Alpha Cutoff", &material.params.surface.z, 0.0f, 1.0f);

    changed |= drawFlagCheckbox("Cast Shadow", MaterialCastShadow, material);
    ImGui::SameLine();
    changed |= drawFlagCheckbox("Receive Shadow", MaterialReceiveShadow, material);

    // Mirror/Reflection/Parallax need a probe or a planar capture blend has
    // no scene to provide, and Metallic-Roughness Map is just a different
    // packing of the Surface slot below - none of the four belong here
    // while blend is PBR-only, straight albedo/normal/surface/emissive.
    ImGui::Spacing();
    ImGui::TextDisabled("Drag an image from Materials' grid onto a slot");
    changed |= drawTextureSlot("Albedo", SlotAlbedo, material);
    changed |= drawTextureSlot("Normal", SlotNormal, material);
    changed |= drawTextureSlot("Surface", SlotSurface, material);
    changed |= drawTextureSlot("Emissive", SlotEmissive, material);

    return changed;
}

bool MaterialEditor::drawTextureSlot(const char* label, u32 slot, Material& material)
{
    ImGui::PushID(label);
    MaterialTexture& texture = material.textures[slot];

    ImGui::TextUnformatted(label);
    ImGui::SameLine(90.0f);

    const std::string buttonLabel = texture.file.empty() ? "(drop image here)" : texture.file;
    const f32 clearWidth = ImGui::GetFrameHeight();
    ImGui::Button(buttonLabel.c_str(), ImVec2(-clearWidth - ImGui::GetStyle().ItemSpacing.x, 0.0f));

    bool changed = false;
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kTextureDragPayload))
        {
            const std::string path(static_cast<const char*>(payload->Data), payload->DataSize);
            texture.texture =
                Assets().loadTexture(path, Material::colorSpaceFor(static_cast<MaterialSlot>(slot)));
            // Matches MaterialParserInternal's own defaults for a freshly
            // assigned slot - otherwise it reads back as point/clamp until
            // the sidecar is saved and reloaded.
            SamplerDesc sampler;
            sampler.filter = Filter::Anisotropic;
            sampler.wrapU = Wrap::Repeat;
            sampler.wrapV = Wrap::Repeat;
            sampler.wrapW = Wrap::Repeat;
            sampler.anisotropy = 8.0f;
            texture.sampler = Assets().getSampler(sampler);
            texture.source = TextureSource::Static;
            texture.file = path;
            changed = true;
        }
        ImGui::EndDragDropTarget();
    }

    ImGui::SameLine();
    ImGui::BeginDisabled(texture.file.empty());
    if (ImGui::Button(ICON_MDI_TRASH_CAN_OUTLINE, ImVec2(clearWidth, 0.0f)))
    {
        texture = MaterialTexture();
        changed = true;
    }
    ImGui::EndDisabled();

    ImGui::PopID();
    return changed;
}
