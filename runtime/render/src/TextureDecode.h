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

// Everything AssetManager::loadTexture() used to do minus the GPU upload -
// reads the file and decodes it (stb_image via Pixmap, or DDSImage for
// block-compressed) into a TextureDesc ready for GPU::createTexture()/
// replaceTexture(). Pure CPU work, no GL calls anywhere in here, which is
// the one property that lets AsyncTextureLoader call this from a worker
// thread - the synchronous AssetManager::loadTexture() path uses the exact
// same function so the two never drift apart.
struct DecodedTexture
{
    bool ok = false;
    TextureDesc desc;

    // Backing storage desc.data/desc.compressedMips point into - own until
    // the GPU upload (createTexture()/replaceTexture()) has actually run,
    // then free. Exactly one of pixmap/dds is set on success.
    Pixmap* pixmap = nullptr;
    DDSImage* dds = nullptr;
    std::vector<CompressedMip> ddsMips;

    // No destructor of its own on purpose - pixmap/dds are freed by
    // releaseDecodedTexture(), explicitly, once the GPU upload built from
    // `desc` is done with them. Letting a destructor free them here would
    // make every copy in and out of a queue (this crosses one, worker thread
    // to main thread) a use-after-free race waiting to happen.
};

DecodedTexture decodeTextureFile(const std::string& filename, ColorSpace space, bool generateMips,
                                 u32 mipLimit);

// Shared with loadCubemap()'s own decode (never off-thread, six faces packed
// into one buffer by hand instead of going through decodeTextureFile()), so
// the linear/sRGB format choice for a given source never disagrees between
// the two loaders.
Format formatFor(int components, ColorSpace space);
Format ddsFormatFor(Format base, ColorSpace space);

// Frees whatever decodeTextureFile() allocated - call once the GPU upload
// built from `texture.desc` has finished (createTexture()/replaceTexture()
// have already copied out everything they need by the time they return).
void releaseDecodedTexture(DecodedTexture& texture);

} // namespace Radion

#endif // RADION_TEXTURE_DECODE_H
