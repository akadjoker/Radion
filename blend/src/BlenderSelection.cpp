#include "PCH.h"
#include "BlenderSelection.h"

#include <cstring>

using namespace Radion;

namespace
{
constexpr u32 kBitsPerWord = 64;

u32 wordCountFor(u32 count)
{
    return (count + kBitsPerWord - 1) / kBitsPerWord;
}

u32 trailingZeros(u64 value)
{
#if defined(__GNUC__) || defined(__clang__)
    return static_cast<u32>(__builtin_ctzll(value));
#elif defined(_MSC_VER)
    unsigned long index = 0;
    _BitScanForward64(&index, value);
    return static_cast<u32>(index);
#else
    u32 count = 0;
    while (!(value & 1))
    {
        value >>= 1;
        ++count;
    }
    return count;
#endif
}

u32 popCount(u64 value)
{
#if defined(__GNUC__) || defined(__clang__)
    return static_cast<u32>(__builtin_popcountll(value));
#else
    u32 count = 0;
    while (value)
    {
        value &= value - 1;
        ++count;
    }
    return count;
#endif
}

u32 countBits(const std::vector<u64>& bits)
{
    u32 total = 0;
    for (u32 word = 0; word < static_cast<u32>(bits.size()); ++word)
        total += popCount(bits[word]);
    return total;
}
} // namespace

BlenderSelection::BlenderSelection()
    : mMode(SelectionMode::Vertex)
{
}

BlenderSelection::~BlenderSelection()
{
}

bool BlenderSelection::testBit(const std::vector<u64>& bits, u32 index)
{
    const u32 word = index / kBitsPerWord;
    if (word >= bits.size())
        return false;
    return (bits[word] & (u64(1) << (index % kBitsPerWord))) != 0;
}

bool BlenderSelection::setBit(std::vector<u64>& bits, u32 index)
{
    const u32 word = index / kBitsPerWord;
    if (word >= bits.size())
        bits.resize(word + 1, 0);

    const u64 mask = u64(1) << (index % kBitsPerWord);
    if (bits[word] & mask)
        return false;
    bits[word] |= mask;
    return true;
}

bool BlenderSelection::clearBit(std::vector<u64>& bits, u32 index)
{
    const u32 word = index / kBitsPerWord;
    if (word >= bits.size())
        return false;

    const u64 mask = u64(1) << (index % kBitsPerWord);
    if (!(bits[word] & mask))
        return false;
    bits[word] &= ~mask;
    return true;
}

void BlenderSelection::rebuild(const std::vector<u64>& bits, std::vector<u32>& list)
{
    list.clear();
    for (u32 word = 0; word < static_cast<u32>(bits.size()); ++word)
    {
        u64 value = bits[word];
        while (value)
        {
            // Lowest set bit first, so the list comes out ascending.
            list.push_back(word * kBitsPerWord + trailingZeros(value));
            value &= value - 1;
        }
    }
}

void BlenderSelection::selectVertex(u32 index)
{
    if (setBit(mVertexBits, index))
    {
        ++mVertexCount;
        ++mRevision;
    }
}

void BlenderSelection::deselectVertex(u32 index)
{
    if (clearBit(mVertexBits, index))
    {
        --mVertexCount;
        ++mRevision;
    }
}

void BlenderSelection::toggleVertex(u32 index)
{
    if (isVertexSelected(index))
        deselectVertex(index);
    else
        selectVertex(index);
}

bool BlenderSelection::isVertexSelected(u32 index) const
{
    return testBit(mVertexBits, index);
}

const std::vector<u32>& BlenderSelection::selectedVertices() const
{
    if (mVertexListRevision != mRevision)
    {
        rebuild(mVertexBits, mVertexList);
        mVertexListRevision = mRevision;
    }
    return mVertexList;
}

void BlenderSelection::selectFace(u32 index)
{
    if (setBit(mFaceBits, index))
    {
        ++mFaceCount;
        ++mRevision;
    }
}

void BlenderSelection::deselectFace(u32 index)
{
    if (clearBit(mFaceBits, index))
    {
        --mFaceCount;
        ++mRevision;
    }
}

void BlenderSelection::toggleFace(u32 index)
{
    if (isFaceSelected(index))
        deselectFace(index);
    else
        selectFace(index);
}

bool BlenderSelection::isFaceSelected(u32 index) const
{
    return testBit(mFaceBits, index);
}

const std::vector<u32>& BlenderSelection::selectedFaces() const
{
    if (mFaceListRevision != mRevision)
    {
        rebuild(mFaceBits, mFaceList);
        mFaceListRevision = mRevision;
    }
    return mFaceList;
}

void BlenderSelection::clearAll()
{
    mVertexBits.clear();
    mFaceBits.clear();
    mVertexCount = 0;
    mFaceCount = 0;
    ++mRevision;
}

void BlenderSelection::selectAll(u32 vertexCount, u32 faceCount)
{
    clearAll();

    if (mMode == SelectionMode::Vertex && vertexCount > 0)
    {
        mVertexBits.assign(wordCountFor(vertexCount), ~u64(0));
        // The last word runs past vertexCount; those bits belong to no vertex
        // and would be handed out by selectedVertices() as real indices.
        const u32 tail = vertexCount % kBitsPerWord;
        if (tail)
            mVertexBits.back() = (u64(1) << tail) - 1;
        mVertexCount = vertexCount;
    }
    else if (mMode == SelectionMode::Face && faceCount > 0)
    {
        mFaceBits.assign(wordCountFor(faceCount), ~u64(0));
        const u32 tail = faceCount % kBitsPerWord;
        if (tail)
            mFaceBits.back() = (u64(1) << tail) - 1;
        mFaceCount = faceCount;
    }
}

void BlenderSelection::invertSelection(u32 vertexCount, u32 faceCount)
{
    // Counted back from the bits rather than as count-minus-selected: the
    // resize below can drop words left over from a larger mesh, and those
    // bits were part of the old total.
    if (mMode == SelectionMode::Vertex)
    {
        mVertexBits.resize(wordCountFor(vertexCount), 0);
        for (u32 word = 0; word < static_cast<u32>(mVertexBits.size()); ++word)
            mVertexBits[word] = ~mVertexBits[word];

        const u32 tail = vertexCount % kBitsPerWord;
        if (tail && !mVertexBits.empty())
            mVertexBits.back() &= (u64(1) << tail) - 1;

        mVertexCount = countBits(mVertexBits);
    }
    else if (mMode == SelectionMode::Face)
    {
        mFaceBits.resize(wordCountFor(faceCount), 0);
        for (u32 word = 0; word < static_cast<u32>(mFaceBits.size()); ++word)
            mFaceBits[word] = ~mFaceBits[word];

        const u32 tail = faceCount % kBitsPerWord;
        if (tail && !mFaceBits.empty())
            mFaceBits.back() &= (u64(1) << tail) - 1;

        mFaceCount = countBits(mFaceBits);
    }

    ++mRevision;
}

void BlenderSelection::fillVertexFlags(u8* out, u32 count) const
{
    if (!out || count == 0)
        return;

    std::memset(out, 0, count);
    for (u32 word = 0; word < static_cast<u32>(mVertexBits.size()); ++word)
    {
        u64 value = mVertexBits[word];
        if (!value)
            continue;
        const u32 base = word * kBitsPerWord;
        while (value)
        {
            const u32 index = base + trailingZeros(value);
            if (index < count)
                out[index] = 1;
            value &= value - 1;
        }
    }
}
