#include "PCH.h"

#include "RadionFormat.h"

#include "ByteArray.h"

namespace Radion::AssetFormat
{

Writer::Writer(ByteArray& output) : mOutput(output)
{
}

void Writer::header(Radion::u32 magic, Radion::u32 flags)
{
    writeU32(magic);
    writeU32(Version);
    writeU32(flags);
}

Radion::u64 Writer::beginChunk(Radion::u32 id)
{
    writeU32(id);
    const Radion::u64 marker = mOutput.tell();
    writeU64(0);
    return marker;
}

bool Writer::endChunk(Radion::u64 marker)
{
    const Radion::u64 end = mOutput.tell();
    if (marker > end || end - marker < sizeof(Radion::u64))
        return false;
    mOutput.seek(static_cast<s64>(marker));
    writeU64(end - marker - sizeof(Radion::u64));
    mOutput.seek(static_cast<s64>(end));
    return true;
}

void Writer::string(const std::string& value)
{
    writeU32(static_cast<Radion::u32>(value.size()));
    bytes(value.data(), value.size());
}

void Writer::writeU8(Radion::u8 value)
{
    mOutput.writeU8(value);
}
void Writer::writeU16(Radion::u16 value)
{
    mOutput.writeU16(value);
}
void Writer::writeU32(Radion::u32 value)
{
    mOutput.writeU32(value);
}
void Writer::writeS32(Radion::s32 value)
{
    mOutput.writeU32(static_cast<Radion::u32>(value));
}
void Writer::writeU64(Radion::u64 value)
{
    mOutput.writeU32(static_cast<Radion::u32>(value));
    mOutput.writeU32(static_cast<Radion::u32>(value >> 32));
}
void Writer::writeF32(Radion::f32 value)
{
    Radion::u32 bits;
    std::memcpy(&bits, &value, sizeof(bits));
    writeU32(bits);
}
void Writer::bytes(const void* data, usize size)
{
    mOutput.writeBytes(data, size);
}

Reader::Reader(ByteArray& input) : mInput(input), mLimit(input.size())
{
}

bool Reader::header(Radion::u32 expectedMagic, std::string* error)
{
    Radion::u32 magic = 0;
    Radion::u32 version = 0;
    Radion::u32 flags = 0;
    if (!readU32(magic) || !readU32(version) || !readU32(flags))
    {
        if (error)
            *error = "file is smaller than the Radion header";
        return false;
    }
    if (magic != expectedMagic)
    {
        if (error)
            *error = "wrong Radion asset magic";
        return false;
    }
    if ((version >> 16) != VersionMajor)
    {
        if (error)
            *error = "unsupported Radion asset major version";
        return false;
    }
    if ((flags & LittleEndian) == 0)
    {
        if (error)
            *error = "big-endian Radion assets are not supported";
        return false;
    }
    return true;
}

bool Reader::next(ChunkHeader& chunk)
{
    return readU32(chunk.id) && readU64(chunk.size) && chunk.size <= remaining();
}

bool Reader::enter(const ChunkHeader& chunk)
{
    if (mParentLimit != 0 || chunk.size > remaining())
        return false;
    mParentLimit = mLimit;
    mLimit = mInput.tell() + chunk.size;
    return true;
}

void Reader::leave()
{
    if (mParentLimit == 0)
        return;
    mInput.seek(static_cast<s64>(mLimit));
    mLimit = mParentLimit;
    mParentLimit = 0;
}

bool Reader::string(std::string& value)
{
    Radion::u32 size = 0;
    if (!readU32(size) || size > remaining())
        return false;
    value.resize(size);
    return size == 0 || bytes(value.data(), size);
}

bool Reader::skip(Radion::u64 size)
{
    if (!available(size))
        return false;
    mInput.seek(static_cast<s64>(size), ByteArray::SeekCurrent);
    return true;
}

bool Reader::readU8(Radion::u8& value)
{
    if (!available(1))
        return false;
    value = mInput.readU8();
    return true;
}
bool Reader::readU16(Radion::u16& value)
{
    if (!available(2))
        return false;
    value = mInput.readU16();
    return true;
}
bool Reader::readU32(Radion::u32& value)
{
    if (!available(4))
        return false;
    value = mInput.readU32();
    return true;
}
bool Reader::readU64(Radion::u64& value)
{
    Radion::u32 low = 0;
    Radion::u32 high = 0;
    if (!readU32(low) || !readU32(high))
        return false;
    value = static_cast<Radion::u64>(low) | (static_cast<Radion::u64>(high) << 32);
    return true;
}
bool Reader::readS32(Radion::s32& value)
{
    Radion::u32 bits = 0;
    if (!readU32(bits))
        return false;
    value = static_cast<Radion::s32>(bits);
    return true;
}
bool Reader::readF32(Radion::f32& value)
{
    Radion::u32 bits = 0;
    if (!readU32(bits))
        return false;
    std::memcpy(&value, &bits, sizeof(value));
    return std::isfinite(value);
}
bool Reader::bytes(void* data, usize size)
{
    if (!available(size))
        return false;
    mInput.readBytes(data, size);
    return true;
}

u64 Reader::remaining() const
{
    const Radion::u64 position = mInput.tell();
    return position <= mLimit ? mLimit - position : 0;
}

bool Reader::available(Radion::u64 size) const
{
    return size <= remaining();
}

} // namespace Radion::AssetFormat
