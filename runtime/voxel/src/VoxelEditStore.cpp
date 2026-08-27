#include "VoxelEditStore.h"

#include "VoxelWorld.h"

#include <cstring>

namespace Radion
{
namespace Voxel
{
namespace
{
constexpr u32 StoreMagic = 0x584F5652u; // "RVOX"
constexpr u32 StoreVersion = 1;

void appendU32(std::vector<u8>& bytes, u32 value)
{
    bytes.insert(bytes.end(), reinterpret_cast<const u8*>(&value),
                 reinterpret_cast<const u8*>(&value) + sizeof(value));
}

void appendS32(std::vector<u8>& bytes, s32 value)
{
    bytes.insert(bytes.end(), reinterpret_cast<const u8*>(&value),
                 reinterpret_cast<const u8*>(&value) + sizeof(value));
}

void appendU16(std::vector<u8>& bytes, u16 value)
{
    bytes.insert(bytes.end(), reinterpret_cast<const u8*>(&value),
                 reinterpret_cast<const u8*>(&value) + sizeof(value));
}

bool readBytes(const std::vector<u8>& bytes, usize& cursor, void* target, usize size)
{
    if (cursor + size > bytes.size())
        return false;
    std::memcpy(target, bytes.data() + cursor, size);
    cursor += size;
    return true;
}
} // namespace

void VoxelEditStore::record(VoxelCoord position, BlockId block)
{
    const ChunkCoord coordinate = VoxelWorld::chunkFor(position);
    const u16 index = static_cast<u16>(VoxelChunk::localIndex(VoxelWorld::localFor(position)));
    std::vector<VoxelEditRecord>& records = mChunks[coordinate];
    for (VoxelEditRecord& record : records)
    {
        if (record.index == index)
        {
            record.block = block;
            return;
        }
    }
    records.push_back({index, block});
}

bool VoxelEditStore::apply(VoxelChunk& chunk) const
{
    const auto it = mChunks.find(chunk.coordinate());
    if (it == mChunks.end())
        return false;

    for (const VoxelEditRecord& record : it->second)
    {
        const s32 index = static_cast<s32>(record.index);
        const s32 x = index % VoxelChunk::Size;
        const s32 y = (index / VoxelChunk::Size) % VoxelChunk::Size;
        const s32 z = index / (VoxelChunk::Size * VoxelChunk::Size);
        chunk.setBlock({x, y, z}, record.block);
    }
    return true;
}

bool VoxelEditStore::contains(ChunkCoord coordinate) const
{
    return mChunks.find(coordinate) != mChunks.end();
}

usize VoxelEditStore::recordCount() const
{
    usize count = 0;
    for (const auto& entry : mChunks)
        count += entry.second.size();
    return count;
}

void VoxelEditStore::clear()
{
    mChunks.clear();
}

void VoxelEditStore::write(std::vector<u8>& bytes) const
{
    bytes.clear();
    appendU32(bytes, StoreMagic);
    appendU32(bytes, StoreVersion);
    appendU32(bytes, static_cast<u32>(mChunks.size()));
    for (const auto& entry : mChunks)
    {
        appendS32(bytes, entry.first.x);
        appendS32(bytes, entry.first.y);
        appendS32(bytes, entry.first.z);
        appendU32(bytes, static_cast<u32>(entry.second.size()));
        for (const VoxelEditRecord& record : entry.second)
        {
            appendU16(bytes, record.index);
            appendU16(bytes, record.block);
        }
    }
}

bool VoxelEditStore::read(const std::vector<u8>& bytes)
{
    clear();
    usize cursor = 0;
    u32 magic = 0;
    u32 version = 0;
    u32 chunkCount = 0;
    if (!readBytes(bytes, cursor, &magic, sizeof(magic)) || magic != StoreMagic)
        return false;
    if (!readBytes(bytes, cursor, &version, sizeof(version)) || version != StoreVersion)
        return false;
    if (!readBytes(bytes, cursor, &chunkCount, sizeof(chunkCount)))
        return false;

    for (u32 index = 0; index < chunkCount; ++index)
    {
        ChunkCoord coordinate;
        u32 recordCount = 0;
        if (!readBytes(bytes, cursor, &coordinate.x, sizeof(coordinate.x)) ||
            !readBytes(bytes, cursor, &coordinate.y, sizeof(coordinate.y)) ||
            !readBytes(bytes, cursor, &coordinate.z, sizeof(coordinate.z)) ||
            !readBytes(bytes, cursor, &recordCount, sizeof(recordCount)))
        {
            clear();
            return false;
        }

        std::vector<VoxelEditRecord> records;
        records.reserve(recordCount);
        for (u32 record = 0; record < recordCount; ++record)
        {
            VoxelEditRecord entry;
            if (!readBytes(bytes, cursor, &entry.index, sizeof(entry.index)) ||
                !readBytes(bytes, cursor, &entry.block, sizeof(entry.block)) ||
                entry.index >= VoxelChunk::Volume)
            {
                clear();
                return false;
            }
            records.push_back(entry);
        }
        mChunks.emplace(coordinate, std::move(records));
    }
    return true;
}

} // namespace Voxel
} // namespace Radion
