#include "PCH.h"

#include "TextureDecode.h"

#include "DDSImage.h"
#include "FileSystem.h"
#include "Log.h"
#include "Pixmap.h"

#include <algorithm>
#include <cctype>

namespace Radion
{

namespace
{

bool hasDdsExtension(const std::string& path)
{
    constexpr const char* ext = ".dds";
    const usize extLen = 4;
    if (path.size() < extLen)
        return false;
    return std::equal(path.end() - static_cast<long>(extLen), path.end(), ext,
                      [](char a, char b)
                      { return std::tolower(static_cast<u8>(a)) == std::tolower(static_cast<u8>(b)); });
}

} // namespace

// DDS carries no colour-space tag of its own, so the caller's requested
// space picks between the linear and sRGB GPU format - same job formatFor()
// does for uncompressed sources below. BC5 has no sRGB variant: it only ever
// holds normal maps, which are linear anyway.
Format ddsFormatFor(Format base, ColorSpace space)
{
    if (space != ColorSpace::sRGB)
        return base;

    switch (base)
    {
    case Format::BC1_RGBA:
        return Format::BC1_RGBA_sRGB;
    case Format::BC3_RGBA:
        return Format::BC3_RGBA_sRGB;
    case Format::BC7_RGBA:
        return Format::BC7_RGBA_sRGB;
    default:
        return base;
    }
}

Format formatFor(int components, ColorSpace space)
{
    switch (components)
    {
    case 1:
        return Format::R8;
    case 2:
        return Format::RG8;
    default:
        return space == ColorSpace::sRGB ? Format::RGBA8_sRGB : Format::RGBA8;
    }
}

DecodedTexture decodeTextureFile(const std::string& filename, ColorSpace space, bool generateMips,
                                 u32 mipLimit)
{
    DecodedTexture result;

    if (hasDdsExtension(filename))
    {
        ByteArray ddsBytes = FileSystem::getSingleton().readBinary(filename);
        DDSImage* dds = new DDSImage();
        if (ddsBytes.empty() || !dds->loadFromMemory(ddsBytes.data(), ddsBytes.size()))
        {
            Log::error("TextureDecode: failed to decode '%s'", filename.c_str());
            delete dds;
            return result;
        }

        if (dds->format() == Format::RGBA8)
        {
            result.desc.type = TextureType::Tex2D;
            result.desc.format = space == ColorSpace::sRGB ? Format::RGBA8_sRGB : Format::RGBA8;
            result.desc.width = dds->width();
            result.desc.height = dds->height();
            result.desc.mips = generateMips ? mipLimit : 1;
            result.desc.usage = TextureSampled;
            result.desc.data = dds->mipData(0);
            result.dds = dds;
            result.ok = true;
            return result;
        }

        result.ddsMips.resize(dds->mipCount());
        for (u32 mip = 0; mip < dds->mipCount(); ++mip)
            result.ddsMips[mip] = {dds->mipData(mip), dds->mipSize(mip)};

        result.desc.type = TextureType::Tex2D;
        result.desc.format = ddsFormatFor(dds->format(), space);
        result.desc.width = dds->width();
        result.desc.height = dds->height();
        result.desc.mips = dds->mipCount();
        result.desc.usage = TextureSampled;
        result.desc.compressedMips = result.ddsMips.data();
        result.desc.compressedMipCount = static_cast<u32>(result.ddsMips.size());
        result.dds = dds;
        result.ok = true;
        return result;
    }

    ByteArray bytes = FileSystem::getSingleton().readBinary(filename);
    Pixmap* pixmap = new Pixmap();
    if (bytes.empty() || bytes.size() > 0xFFFFFFFFu ||
        !pixmap->load_from_memory(bytes.data(), static_cast<u32>(bytes.size())))
    {
        Log::error("TextureDecode: failed to decode '%s'", filename.c_str());
        delete pixmap;
        return result;
    }

    // The GPU has no plain RGB8 format, only RGBA8 - a 3-component source
    // needs a padded alpha channel before it can be uploaded.
    Pixmap* converted = pixmap->components == 3 ? pixmap->convert_to_rgba() : nullptr;
    Pixmap* source = converted ? converted : pixmap;
    if (converted)
        delete pixmap;

    // R8/RG8 have no sRGB counterpart here, so a one- or two-channel file asked
    // for as colour would sample undecoded and quietly read too bright. Say so
    // instead of letting it pass.
    if (space == ColorSpace::sRGB && source->components < 3)
        Log::warning("TextureDecode: '%s' has %d channel(s) and cannot be sRGB; loading it linear",
                     filename.c_str(), source->components);

    result.desc.type = TextureType::Tex2D;
    result.desc.format = formatFor(source->components, space);
    result.desc.width = static_cast<u32>(source->width);
    result.desc.height = static_cast<u32>(source->height);
    result.desc.mips = generateMips ? mipLimit : 1; // 0 = full chain, TextureDesc's own convention
    result.desc.usage = TextureSampled;
    result.desc.data = source->pixels;
    result.desc.debugName = nullptr; // caller sets this - the filename std::string outlives us here
    result.pixmap = source;
    result.ok = true;
    return result;
}

void releaseDecodedTexture(DecodedTexture& texture)
{
    delete texture.pixmap;
    delete texture.dds;
    texture.pixmap = nullptr;
    texture.dds = nullptr;
}

} // namespace Radion
