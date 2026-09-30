#ifndef RADION_MATERIAL_PARSER_H
#define RADION_MATERIAL_PARSER_H

#include "Material.h"

#include <string>
#include <vector>

namespace Radion
{

// Colour space is stated only when it disagrees with the slot.
enum class ColorSpaceOverride : u8
{
    FromSlot,
    Linear,
    sRGB
};

struct MaterialTextureSource
{
    std::string name;
    std::string file;
    std::string target;
    std::vector<std::string> frames;
    TextureSource source = TextureSource::None;
    // Not Linear: that is GL_LINEAR minification, which ignores the mip chain generateMips builds.
    Filter filter = Filter::Anisotropic;
    Wrap wrap = Wrap::Repeat;
    Math::vec2 scrollSpeed = Math::vec2(0.0f);
    f32 rotateSpeed = 0.0f;
    ColorSpaceOverride colorSpace = ColorSpaceOverride::FromSlot;
    bool generateMips = true;
    u8 slot = MaterialSlotCount;
};

struct MaterialDefinition
{
    std::string name;
    Material material;
    std::vector<MaterialTextureSource> textures;
};

struct MaterialParseError
{
    std::string message;
    u32 line = 0;
    u32 column = 0;
};

class MaterialParser
{
public:
    static bool parse(const std::string& text, std::vector<MaterialDefinition>& materials,
                      MaterialParseError* error = nullptr);
    static bool parseFile(const std::string& filename, std::vector<MaterialDefinition>& materials,
                          MaterialParseError* error = nullptr);
};

} // namespace Radion

#endif // RADION_MATERIAL_PARSER_H
