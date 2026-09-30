#ifndef RADION_DDS_IMAGE_H
#define RADION_DDS_IMAGE_H

#include "ByteArray.h"
#include "GPU.h"
#include "Types.h"

#include <vector>

namespace Radion
{

// Parses a DDS into its block-compressed mip chain (BC1, BC3, BC5, BC7 via DX10 header); no decoding.
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

    // Always the linear variant; the caller picks sRGB.
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
