#ifndef RADION_TEXTURE_DECODE_H
#define RADION_TEXTURE_DECODE_H

#include "GPU.h"
#include "Material.h"

#include <string>
#include <vector>

namespace Radion
{
class Pixmap;
class DDSImage;

// Reads and decodes a texture file into a TextureDesc for GPU upload. Pure CPU work with no GL calls, so AsyncTextureLoader may call it from a worker thread; AssetManager::loadTexture() uses the same function.
struct DecodedTexture
{
    bool ok = false;
    TextureDesc desc;

    // Backing storage desc.data/desc.compressedMips point into; own until the GPU upload has run. Exactly one of pixmap/dds is set on success.
    Pixmap* pixmap = nullptr;
    DDSImage* dds = nullptr;
    std::vector<CompressedMip> ddsMips;

    // No destructor on purpose: freed explicitly by releaseDecodedTexture(); copies crossing the worker-to-main queue would otherwise race on free.
};

DecodedTexture decodeTextureFile(const std::string& filename, ColorSpace space, bool generateMips,
                                 u32 mipLimit);

// Shared with loadCubemap()'s decode so the linear/sRGB choice never disagrees between loaders.
Format formatFor(int components, ColorSpace space);
Format ddsFormatFor(Format base, ColorSpace space);

// Frees what decodeTextureFile() allocated; call once the GPU upload has finished.
void releaseDecodedTexture(DecodedTexture& texture);

} // namespace Radion

#endif // RADION_TEXTURE_DECODE_H
