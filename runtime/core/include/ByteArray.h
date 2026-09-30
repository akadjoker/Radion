#ifndef RADION_BYTE_ARRAY_H
#define RADION_BYTE_ARRAY_H

#include "Types.h"

#include <string>

namespace Radion
{

class ByteArray
{
public:
    ByteArray();

    explicit ByteArray(usize size);

    // Non-owning view by default (never freed/realloc'd); takeOwnership=true hands it over.
    ByteArray(uint8* data, usize size, bool takeOwnership = false);

    ~ByteArray();

    ByteArray(const ByteArray&) = delete;
    ByteArray& operator=(const ByteArray&) = delete;

    ByteArray(ByteArray&& other) noexcept;
    ByteArray& operator=(ByteArray&& other) noexcept;

    uint8* data() const
    {
        return mData;
    }
    usize size() const
    {
        return mSize;
    }
    bool empty() const
    {
        return mSize == 0;
    }

    uint8& operator[](usize i)
    {
        return mData[i];
    }
    const uint8& operator[](usize i) const
    {
        return mData[i];
    }

    void releaseOwnership()
    {
        mOwns = false;
    }

    void reserve(usize capacity);

    void clear();

    usize tell() const
    {
        return mPos;
    }
    // Subtraction, not addition: mPos + n can overflow on untrusted lengths.
    bool canRead(usize n) const
    {
        return mPos <= mSize && n <= mSize - mPos;
    }

    enum SeekOrigin
    {
        SeekBegin,
        SeekCurrent,
        SeekEnd
    };

    // Resulting position is clamped to 0 on the low end; no upper clamp, so
    // seeking past size() then writing is a valid way to extend the buffer.
    void seek(long long offset, SeekOrigin origin = SeekBegin);

    uint8 readU8();
    uint16 readU16();
    uint32 readU32();
    uint64 readU64();
    f32 readF32();
    s8 readS8();
    s16 readS16();
    s32 readS32();
    bool readBool();
    char readChar();

    // '\n'-terminated, no length prefix (Ogre .mesh string convention).
    std::string readString();

    std::string readString(usize length);

    void readBytes(void* dst, usize n);

    // Appends at the cursor; seek() first to overwrite in place. Returns false and writes nothing
    // on failure (OOM, or past a non-owning view's capacity).
    bool writeU8(uint8 v);
    bool writeU16(uint16 v);
    bool writeU32(uint32 v);
    bool writeU64(uint64 v);
    bool writeF32(f32 v);
    bool writeBool(bool v);
    bool writeChar(char v);

    bool writeString(const std::string& s);

    // Raw bytes + '\n'; round-trips with readString().
    bool writeLine(const std::string& s);

    bool writeBytes(const void* src, usize n);

private:
    // False leaves mData/mCapacity untouched.
    bool ensureCapacity(usize neededTotal);

    uint8* mData;
    usize mSize;
    usize mCapacity;
    bool mOwns;
    usize mPos;
};

} // namespace Radion

#endif // RADION_BYTE_ARRAY_H
