#include "PCH.h"

#include "ByteArray.h"

#include "Log.h"

#include <cstdlib>
#include <cstring>

namespace Radion
{

ByteArray::ByteArray() : mData(nullptr), mSize(0), mCapacity(0), mOwns(false), mPos(0)
{
}

ByteArray::ByteArray(usize size)
    : mData(size ? static_cast<uint8*>(std::malloc(size)) : nullptr), mSize(0), mCapacity(0),
      mOwns(true), mPos(0)
{
    // size/capacity only agree with the constructor's argument once the
    // allocation actually succeeded - claiming `size` bytes of a null
    // mData used to make every subsequent read/write believe there was a
    // buffer to touch.
    if (size && !mData)
        Log::error("ByteArray: out of memory allocating %zu bytes", size);
    else
    {
        mSize = size;
        mCapacity = size;
    }
}

ByteArray::ByteArray(uint8* data, usize size, bool takeOwnership)
    : mData(data), mSize(size), mCapacity(size), mOwns(takeOwnership), mPos(0)
{
}

ByteArray::~ByteArray()
{
    if (mOwns && mData)
        std::free(mData);
}

ByteArray::ByteArray(ByteArray&& other) noexcept
    : mData(other.mData), mSize(other.mSize), mCapacity(other.mCapacity), mOwns(other.mOwns),
      mPos(other.mPos)
{
    other.mData = nullptr;
    other.mSize = 0;
    other.mCapacity = 0;
    other.mOwns = false;
    other.mPos = 0;
}

ByteArray& ByteArray::operator=(ByteArray&& other) noexcept
{
    if (this != &other)
    {
        if (mOwns && mData)
            std::free(mData);
        mData = other.mData;
        mSize = other.mSize;
        mCapacity = other.mCapacity;
        mOwns = other.mOwns;
        mPos = other.mPos;
        other.mData = nullptr;
        other.mSize = 0;
        other.mCapacity = 0;
        other.mOwns = false;
        other.mPos = 0;
    }
    return *this;
}

bool ByteArray::ensureCapacity(usize neededTotal)
{
    if (neededTotal <= mCapacity)
        return true;

    if (mData && !mOwns)
    {
        Log::error("ByteArray: can't grow a non-owning view past its original capacity");
        return false;
    }

    // Doubling until >= neededTotal, but checked: neededTotal itself may
    // already be within a factor of two of SIZE_MAX (an attacker-controlled
    // chunk size, a corrupted length prefix), and `*= 2` past that either
    // wraps to a small number - silently under-allocating - or never
    // reaches neededTotal at all, looping forever.
    usize newCapacity = mCapacity == 0 ? 64 : mCapacity;
    while (newCapacity < neededTotal)
    {
        constexpr usize kMax = static_cast<usize>(-1);
        if (newCapacity > kMax / 2)
        {
            Log::error("ByteArray: requested capacity %zu is not representable", neededTotal);
            return false;
        }
        newCapacity *= 2;
    }

    uint8* newData = static_cast<uint8*>(std::realloc(mData, newCapacity));
    if (!newData)
    {
        Log::error("ByteArray: out of memory growing to %zu bytes", newCapacity);
        return false;
    }
    mData = newData;
    mCapacity = newCapacity;
    mOwns = true;
    return true;
}

void ByteArray::reserve(usize capacity)
{
    ensureCapacity(capacity);
}

void ByteArray::clear()
{
    mSize = 0;
    mPos = 0;
}

void ByteArray::seek(long long offset, SeekOrigin origin)
{
    long long base = 0;
    switch (origin)
    {
    case SeekBegin:
        base = 0;
        break;
    case SeekCurrent:
        base = (long long)mPos;
        break;
    case SeekEnd:
        base = (long long)mSize;
        break;
    }

    long long newPos = base + offset;
    mPos = newPos < 0 ? 0 : (usize)newPos;
}

uint8 ByteArray::readU8()
{
    uint8 v = 0;
    readBytes(&v, 1);
    return v;
}
uint16 ByteArray::readU16()
{
    uint16 v = 0;
    readBytes(&v, 2);
    return v;
}
uint32 ByteArray::readU32()
{
    uint32 v = 0;
    readBytes(&v, 4);
    return v;
}
uint64 ByteArray::readU64()
{
    uint64 v = 0;
    readBytes(&v, sizeof(v));
    return v;
}
f32 ByteArray::readF32()
{
    f32 v = 0.0f;
    readBytes(&v, sizeof(f32));
    return v;
}
s8 ByteArray::readS8()
{
    s8 v = 0;
    readBytes(&v, 1);
    return v;
}
s16 ByteArray::readS16()
{
    s16 v = 0;
    readBytes(&v, 2);
    return v;
}
s32 ByteArray::readS32()
{
    s32 v = 0;
    readBytes(&v, 4);
    return v;
}
bool ByteArray::readBool()
{
    uint8 v = 0;
    readBytes(&v, 1);
    return v != 0;
}
char ByteArray::readChar()
{
    char v = 0;
    readBytes(&v, 1);
    return v;
}

std::string ByteArray::readString()
{
    std::string s;
    while (mPos < mSize && mData[mPos] != '\n')
        s.push_back((char)mData[mPos++]);
    if (mPos < mSize)
        ++mPos; // consume the newline
    return s;
}

std::string ByteArray::readString(usize length)
{
    std::string s;
    if (length == 0)
        return s;
    s.resize(length);
    readBytes(&s[0], length);
    return s;
}

void ByteArray::readBytes(void* dst, usize n)
{
    if (!canRead(n))
    {
        Log::error("ByteArray: read past end of buffer");
        memset(dst, 0, n);
        mPos = mSize;
        return;
    }
    memcpy(dst, mData + mPos, n);
    mPos += n;
}

bool ByteArray::writeU8(uint8 v)
{
    return writeBytes(&v, 1);
}
bool ByteArray::writeU16(uint16 v)
{
    return writeBytes(&v, 2);
}
bool ByteArray::writeU32(uint32 v)
{
    return writeBytes(&v, 4);
}
bool ByteArray::writeU64(uint64 v)
{
    return writeBytes(&v, sizeof(v));
}
bool ByteArray::writeF32(f32 v)
{
    return writeBytes(&v, sizeof(v));
}
bool ByteArray::writeBool(bool v)
{
    uint8 b = v ? 1 : 0;
    return writeBytes(&b, 1);
}
bool ByteArray::writeChar(char v)
{
    return writeBytes(&v, 1);
}

bool ByteArray::writeString(const std::string& s)
{
    return s.empty() || writeBytes(s.data(), s.size());
}

bool ByteArray::writeLine(const std::string& s)
{
    // Both must succeed, or a caller checking the result cannot tell a line
    // that got its text but not its terminator from one that got neither.
    const bool wroteText = writeString(s);
    return writeU8((uint8)'\n') && wroteText;
}

bool ByteArray::writeBytes(const void* src, usize n)
{
    if (n == 0)
        return true;

    // Checked, not `mPos + n`: mPos and n both ultimately come from calls a
    // caller controls the size of (a string, a chunk being re-serialised),
    // and their sum can wrap before ever reaching ensureCapacity().
    if (mPos > static_cast<usize>(-1) - n)
    {
        Log::error("ByteArray: write of %zu bytes at offset %zu overflows", n, mPos);
        return false;
    }
    const usize neededTotal = mPos + n;
    if (!ensureCapacity(neededTotal))
        return false; // ensureCapacity already logged why

    memcpy(mData + mPos, src, n);
    mPos = neededTotal;
    if (mPos > mSize)
        mSize = mPos;
    return true;
}

} // namespace Radion
