#ifndef RADION_BLENDER_MATERIAL_EDITOR_H
#define RADION_BLENDER_MATERIAL_EDITOR_H

#include "Types.h"

namespace Radion
{

struct Material;

class MaterialEditor
{
public:
    MaterialEditor() = delete;

    // Drag payload for an image entry in MaterialsPanel's grid, accepted by drawTextureSlot().
    static constexpr const char* kTextureDragPayload = "RADION_BLEND_TEXTURE_FILE";

    // Returns true if anything changed.
    static bool drawFields(Material& material);

    // `slot` is a MaterialSlot value; returns true if the slot changed.
    static bool drawTextureSlot(const char* label, u32 slot, Material& material);
};

} // namespace Radion

#endif // RADION_BLENDER_MATERIAL_EDITOR_H
