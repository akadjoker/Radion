#ifndef RADION_BLENDER_MATERIAL_EDITOR_H
#define RADION_BLENDER_MATERIAL_EDITOR_H

#include "Types.h"

namespace Radion
{

struct Material;

// Shared material property widgets - MaterialsPanel's Inspector and
// PropertiesPanel's per-submesh section both edit the same Material struct
// and want the same fields, so the drawing lives here once instead of twice.
class MaterialEditor
{
public:
    MaterialEditor() = delete;

    // Drag payload for an image entry in MaterialsPanel's grid, accepted by
    // drawTextureSlot() below. Both ends live inside blend, unlike the
    // editor's AssetsPanel/InspectorPanel pair which needs a payload shared
    // across separate panel classes for the same reason.
    static constexpr const char* kTextureDragPayload = "RADION_BLEND_TEXTURE_FILE";

    // Blend/cull/depth-write, base color, roughness/metallic/normal
    // strength, emissive, the flags a preview-less .material sidecar can
    // still carry, and UV tiling/offset. Returns true if anything changed.
    static bool drawFields(Material& material);

    // One texture slot: current file (or "drop image here"), a drag-drop
    // target for kTextureDragPayload, and a clear button. `slot` is a
    // MaterialSlot value. Returns true if the slot changed.
    static bool drawTextureSlot(const char* label, u32 slot, Material& material);
};

} // namespace Radion

#endif // RADION_BLENDER_MATERIAL_EDITOR_H
