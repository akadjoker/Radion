#include "PCH.h"

#include "DDSImage.h"
#include "Log.h"

#include <cstring>

namespace Radion
{

namespace
{

constexpr u32 kMagic = 0x20534444; // "DDS " (little-endian)
constexpr u32 kFlagFourCC = 0x4;

constexpr u32 fourCC(char a, char b, char c, char d)
{
    return static_cast<u32>(static_cast<u8>(a)) | (static_cast<u32>(static_cast<u8>(b)) << 8) |
          (static_cast<u32>(static_cast<u8>(c)) << 16) | (static_cast<u32>(static_cast<u8>(d)) << 24);
}

u32 blockBytesFor(Format format)
{
    switch (format)
    {
    case Format::BC1_RGBA:
        return 8;
    case Format::BC3_RGBA:
    case Format::BC5_RG:
    case Format::BC7_RGBA:
        return 16;
    default:
        return 0;
    }
}

// sRGB DX10 variants map to linear; the caller picks per slot.
Format formatForDxgi(u32 dxgi)
{
    switch (dxgi)
    {
    case 71: // BC1_UNORM
    case 72: // BC1_UNORM_SRGB
        return Format::BC1_RGBA;
    case 77: // BC3_UNORM
    case 78: // BC3_UNORM_SRGB
        return Format::BC3_RGBA;
    case 83: // BC5_UNORM
    case 84: // BC5_SNORM
        return Format::BC5_RG;
    case 98: // BC7_UNORM
    case 99: // BC7_UNORM_SRGB
        return Format::BC7_RGBA;
    default:
        return Format::Unknown;
    }
}

// u64: width+3 and blocksWide*blocksHigh*blockBytes overflow u32 near its top end.
u64 mipLevelSize(Format format, u32 width, u32 height)
{
    if (format == Format::RGBA8)
        return static_cast<u64>(width) * static_cast<u64>(height) * 4u;
    const u64 blocksWide = width > 0 ? (static_cast<u64>(width) + 3) / 4 : 1;
    const u64 blocksHigh = height > 0 ? (static_cast<u64>(height) + 3) / 4 : 1;
    return blocksWide * blocksHigh * static_cast<u64>(blockBytesFor(format));
}

} // namespace

DDSImage::DDSImage()
{
}

DDSImage::~DDSImage()
{
}

bool DDSImage::loadFromMemory(const u8* bytes, usize size)
{
    mFormat = Format::Unknown;
    mMips.clear();

    if (!bytes || size < 128)
        return false;

    ByteArray header(const_cast<u8*>(bytes), size);
    if (header.readU32() != kMagic)
        return false;

    header.seek(12, ByteArray::SeekBegin); // dwHeight sits after dwSize+dwFlags
    const u32 height = header.readU32();
    const u32 width = header.readU32();
    header.readU32();
    header.readU32();
    const u32 mipMapCountField = header.readU32();

    header.seek(76, ByteArray::SeekBegin); // ddspf, past dwReserved1[11]
    header.readU32();
    const u32 pfFlags = header.readU32();
    const u32 pfFourCC = header.readU32();

    if (!(pfFlags & kFlagFourCC))
    {
        // Uncompressed variants expand to RGBA8 via masks.
        const u32 bits = header.readU32();
        const u32 rMask = header.readU32();
        const u32 gMask = header.readU32();
        const u32 bMask = header.readU32();
        const u32 aMask = header.readU32();
        if ((bits != 24 && bits != 32) || rMask == 0 || gMask == 0 || bMask == 0)
        {
            Log::error("DDSImage: unsupported uncompressed pixel format");
            return false;
        }
        mFormat = Format::RGBA8;
        if (width == 0 || height == 0)
            return false;
        mWidth = width;
        mHeight = height;
        const u32 levels = mipMapCountField > 0 ? mipMapCountField : 1;
        usize sourceOffset = 128;
        u64 totalOutput = 0;
        u32 outputWidth = width;
        u32 outputHeight = height;
        for (u32 mip = 0; mip < levels; ++mip)
        {
            totalOutput += static_cast<u64>(outputWidth) * outputHeight * 4u;
            outputWidth = outputWidth > 1 ? outputWidth / 2 : 1;
            outputHeight = outputHeight > 1 ? outputHeight / 2 : 1;
        }
        if (totalOutput > static_cast<u64>(-1))
            return false;
        ByteArray converted(static_cast<usize>(totalOutput));
        usize outputOffset = 0;
        u32 mipWidth = width;
        u32 mipHeight = height;
        for (u32 mip = 0; mip < levels; ++mip)
        {
            // Tightly packed RGB rows, not DWORD-aligned: the legacy pitch reads past the file at 2x2/1x1 mips.
            const u64 sourceRow = (static_cast<u64>(mipWidth) * bits + 7u) / 8u;
            const u64 sourceSize = sourceRow * mipHeight;
            const u64 outputSize = static_cast<u64>(mipWidth) * mipHeight * 4u;
            if (sourceSize > static_cast<u64>(-1) - sourceOffset ||
                sourceOffset + sourceSize > size || outputSize > 0xFFFFFFFFu)
                return false;
            auto extract = [](u32 value, u32 mask) -> u8
            {
                if (!mask) return 0;
                u32 shift = 0;
                while ((mask & (1u << shift)) == 0 && shift < 32) ++shift;
                const u32 maxValue = mask >> shift;
                return static_cast<u8>((((value & mask) >> shift) * 255u + maxValue / 2u) /
                                       maxValue);
            };
            for (u32 y = 0; y < mipHeight; ++y)
                for (u32 x = 0; x < mipWidth; ++x)
                {
                    const u8* source = bytes + sourceOffset + y * sourceRow + x * (bits / 8);
                    u32 value = 0;
                    std::memcpy(&value, source, bits / 8);
                    u8* destination = converted.data() + outputOffset +
                                      (static_cast<usize>(y) * mipWidth + x) * 4;
                    destination[0] = extract(value, rMask);
                    destination[1] = extract(value, gMask);
                    destination[2] = extract(value, bMask);
                    destination[3] = aMask ? extract(value, aMask) : 255;
                }
            mMips.push_back({outputOffset, static_cast<u32>(outputSize)});
            outputOffset += static_cast<usize>(outputSize);
            sourceOffset += static_cast<usize>(sourceSize);
            mipWidth = mipWidth > 1 ? mipWidth / 2 : 1;
            mipHeight = mipHeight > 1 ? mipHeight / 2 : 1;
        }
        mData = std::move(converted);
        return true;
    }

    usize payloadOffset = 128;
    if (pfFourCC == fourCC('D', 'X', 'T', '1'))
        mFormat = Format::BC1_RGBA;
    else if (pfFourCC == fourCC('D', 'X', 'T', '5'))
        mFormat = Format::BC3_RGBA;
    else if (pfFourCC == fourCC('A', 'T', 'I', '2') || pfFourCC == fourCC('B', 'C', '5', 'U'))
        mFormat = Format::BC5_RG;
    else if (pfFourCC == fourCC('D', 'X', '1', '0'))
    {
        if (size < 148)
        {
            Log::error("DDSImage: DX10 header runs past the end of the file");
            return false;
        }
        header.seek(128, ByteArray::SeekBegin);
        const u32 dxgiFormat = header.readU32();
        const u32 resourceDimension = header.readU32();
        header.readU32();
        const u32 arraySize = header.readU32();
        if (resourceDimension != 3 || arraySize > 1)
        {
            Log::error("DDSImage: only single 2D DX10 textures are read, not arrays or volumes");
            return false;
        }
        mFormat = formatForDxgi(dxgiFormat);
        if (mFormat == Format::Unknown)
        {
            Log::error("DDSImage: DXGI format %u is not a block format this reads", dxgiFormat);
            return false;
        }
        payloadOffset = 148;
    }
    else
    {
        Log::error("DDSImage: unsupported FourCC (only DXT1/DXT5/ATI2/DX10 are wired up)");
        return false;
    }

    if (width == 0 || height == 0)
    {
        Log::error("DDSImage: zero width/height");
        mFormat = Format::Unknown;
        return false;
    }

    // Mip count is capped at 1 + floor(log2(max(width, height))).
    u32 maxLevels = 1;
    for (u32 dimension = width > height ? width : height; dimension > 1; dimension >>= 1)
        ++maxLevels;
    const u32 levels =
        mipMapCountField > 0 ? (mipMapCountField < maxLevels ? mipMapCountField : maxLevels) : 1;

    mWidth = width;
    mHeight = height;

    usize offset = payloadOffset;
    u32 mipWidth = width;
    u32 mipHeight = height;
    for (u32 mip = 0; mip < levels; ++mip)
    {
        const u64 levelSize = mipLevelSize(mFormat, mipWidth, mipHeight);
        if (levelSize > static_cast<u64>(-1) - offset || offset + levelSize > size ||
            levelSize > static_cast<u64>(0xFFFFFFFFu))
        {
            Log::error("DDSImage: mip chain runs past the end of the file");
            mMips.clear();
            mFormat = Format::Unknown;
            return false;
        }

        mMips.push_back({offset, static_cast<u32>(levelSize)});
        offset += levelSize;
        mipWidth = mipWidth > 1 ? mipWidth / 2 : 1;
        mipHeight = mipHeight > 1 ? mipHeight / 2 : 1;
    }

    // Own the payload: the caller's buffer may be freed after return.
    mData = ByteArray(size);
    if (!mData.data())
    {
        Log::error("DDSImage: out of memory copying %zu bytes", size);
        mMips.clear();
        mFormat = Format::Unknown;
        return false;
    }
    std::memcpy(mData.data(), bytes, size);

    return true;
}

const u8* DDSImage::mipData(u32 mip) const
{
    if (mip >= mMips.size())
        return nullptr;
    return mData.data() + mMips[mip].offset;
}

u32 DDSImage::mipSize(u32 mip) const
{
    if (mip >= mMips.size())
        return 0;
    return mMips[mip].size;
}

} // namespace Radion
