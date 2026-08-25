#ifndef RADION_MATERIAL_SLOT_NAMES_H
#define RADION_MATERIAL_SLOT_NAMES_H

// Shared with tools/exporter, a separate CMake project - kept dependency-free
// so it can be included without linking radion_render. Order matches
// MaterialSlot in Material.h.
constexpr int kMaterialSlotCount = 8;

constexpr const char* kMaterialSlotNames[kMaterialSlotCount] = {
    "Albedo", "Normal",   "Surface",  "Emissive",
    "Detail", "ColorMap", "Lightmap", "Height",
};

#endif // RADION_MATERIAL_SLOT_NAMES_H
