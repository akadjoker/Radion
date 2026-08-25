#ifndef RADION_DDS_IMAGE_H
#define RADION_DDS_IMAGE_H

#include "ByteArray.h"
#include "GPU.h"
#include "Types.h"

#include <vector>

namespace Radion
{

// Parses a DDS file straight into its block-compressed mip chain - no
// decoding, no recompression, just the bytes the GPU can upload as-is. Only
// the block formats the pipeline actually authors textures in are understood
// (BC1/DXT1, BC3/DXT5, BC5/ATI2, and BC7 through the DX10 extended header);
// anything else fails to load rather than guessing at a format.
class DDSImage
{
public:
    DDSImage();
    ~DDSImage();

    bool loadFromMemory(const u8* bytes, usize size);

    bool isValid() const
    {
        return mFormat != Format::Unknown;
    }

    // Always the linear variant (DDS carries no colour-space information) -
    // the caller picks the sRGB one when the slot calls for it.
    Format format() const
    {
        return mFormat;
    }
    u32 width() const
    {
        return mWidth;
    }
    u32 height() const
    {
        return mHeight;
    }
    u32 mipCount() const
    {
        return static_cast<u32>(mMips.size());
    }

    const u8* mipData(u32 mip) const;
    u32 mipSize(u32 mip) const;

private:
    struct MipRange
    {
        usize offset;
        u32 size;
    };

    ByteArray mData; // owns the whole file; mips point into it by offset
    std::vector<MipRange> mMips;
    Format mFormat = Format::Unknown;
    u32 mWidth = 0;
    u32 mHeight = 0;
};

} // namespace Radion

#endif // RADION_DDS_IMAGE_H
