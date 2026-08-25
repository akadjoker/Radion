/** ****************************************************************************
  Radion Engine - byte buffer with a sequential read/write cursor. Doubles as
  a binary file reader (chunk-based formats like Ogre's .mesh) and as a
  growable byte-stream writer/serializer, sharing one cursor: a fresh
  instance is normally used for one purpose at a time (read everything, or
  write everything), though seek() + write lets you patch bytes in place too.
**************************************************************************** */
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

    // Pre-sized and owning - mSize/mCapacity both equal `size`. For "read
    // exactly this many bytes into a fresh buffer" callers (see
    // FileSystem::readBinary()).
    explicit ByteArray(usize size);

    // Wraps existing memory. Non-owning by default (view only - writes
    // within the existing size succeed, but nothing here will ever
    // free/realloc it); pass takeOwnership=true to hand it over instead.
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
    } // logical length (bytes valid/written)
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

    // Grows the backing allocation to at least `capacity` bytes without
    // changing size()/tell() - use before a burst of writeBytes() calls to
    // avoid repeated reallocation. No-op on a non-owning view.
    void reserve(usize capacity);

    // Resets size() and the cursor to 0. Keeps the current allocation (and
    // its capacity) for reuse, same as std::vector::clear().
    void clear();

    // --- Sequential cursor, shared by both read and write ---
    usize tell() const
    {
        return mPos;
    }
    // Subtraction, not addition: mPos + n can overflow when n comes from
    // untrusted data (a malformed asset's declared length, e.g.), which used
    // to turn an out-of-range read into one that looked in-range.
    bool canRead(usize n) const
    {
        return mPos <= mSize && n <= mSize - mPos;
    }

    enum SeekOrigin
    {
        SeekBegin,   // offset from byte 0 (matches the old seek(pos) behaviour)
        SeekCurrent, // offset from tell()
        SeekEnd      // offset from size() - negative offset seeks backward from the end
    };

    // Resulting position is clamped to 0 on the low end; no upper clamp, so
    // seeking past size() then writing is a valid way to extend the buffer.
    void seek(long long offset, SeekOrigin origin = SeekBegin);

    // --- Read ---
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

    // '\n'-terminated, no length prefix, no null byte (Ogre .mesh string
    // convention - see OgreMeshReader).
    std::string readString();

    // Fixed-length read, regardless of content (no terminator handling).
    std::string readString(usize length);

    void readBytes(void* dst, usize n);

    // --- Write (appends at the cursor, growing the buffer if needed and
    // owned; seek() first to overwrite existing bytes in place instead) ---
    // Each returns false on failure (OOM growing, or writing past a
    // non-owning view's fixed capacity) and writes nothing in that case - the
    // buffer's size/cursor are exactly as if the call had not been made.
    // Callers that never write past what reserve()/the constructor already
    // guaranteed can safely ignore the result, as most call sites do.
    bool writeU8(uint8 v);
    bool writeU16(uint16 v);
    bool writeU32(uint32 v);
    bool writeU64(uint64 v);
    bool writeF32(f32 v);
    bool writeBool(bool v);
    bool writeChar(char v);

    // Raw bytes, no length prefix or terminator.
    bool writeString(const std::string& s);

    // Raw bytes + '\n', round-trips with readString() (no-arg version).
    bool writeLine(const std::string& s);

    bool writeBytes(const void* src, usize n);

private:
    // False on failure (an overflowing request, a non-owning view asked to
    // grow, or the allocator itself failing) - mData/mCapacity are left
    // exactly as they were, never partially updated.
    bool ensureCapacity(usize neededTotal);

    uint8* mData;
    usize mSize;     // logical length
    usize mCapacity; // allocated length
    bool mOwns;
    usize mPos;
};

} // namespace Radion

#endif // RADION_BYTE_ARRAY_H
