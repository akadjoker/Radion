#ifndef RADION_FILE_PACK_H
#define RADION_FILE_PACK_H

#include "Archive.h"
#include "ByteArray.h"
#include "Containers.h"
#include "Types.h"

#include <cstdio>
#include <string>
#include <vector>

namespace Radion
{

// Radion archive: deflated, optionally encrypted. Layout:
//   header      64 bytes, the only part stored in the clear
//   data        entry bytes back to back, in order added
//   directory   table plus name blob, encrypted as one region
// Only the directory is resident; reads are not thread-safe (one shared file cursor).
class FilePack : public Archive
{
public:
    static constexpr u32 Version = 1;
    static constexpr usize SaltSize = 16;
    static constexpr usize HeaderSize = 64;
    static constexpr usize TocEntrySize = 32;

    enum PackFlags
    {
        PackEncrypted = 1 << 0
    };

    enum EntryFlags
    {
        EntryDeflated = 1 << 0
    };

    FilePack();
    ~FilePack() override;

    FilePack(const FilePack&) = delete;
    FilePack& operator=(const FilePack&) = delete;

    // Key must match the one used to write (empty for none); a wrong key is rejected here.
    bool open(const std::string& path, const std::string& key);
    // Reads an in-memory pack; the bytes are borrowed and must outlive this FilePack.
    bool openFromMemory(const u8* data, usize size, const std::string& key);
    void close();
    bool isOpen() const;

    bool exists(const std::string& name) const override;
    ByteArray readBinary(const std::string& name) const override;

    u32 entryCount() const;
    const std::string& entryName(u32 index) const;
    u32 entrySizeRaw(u32 index) const;
    u32 entrySizeStored(u32 index) const;

private:
    struct Entry
    {
        std::string name;
        u64 dataOffset;
        u32 sizeStored;
        u32 sizeRaw;
        u32 crc;
        u16 flags;
    };

    bool readDirectory(const char* label, const std::string& key);
    bool readAt(u64 offset, void* destination, usize size) const;

    std::FILE* mFile;
    const u8* mMemory;
    usize mMemorySize;
    std::vector<Entry> mEntries;
    HashMap<std::string, u32> mIndex;
    u8 mKey[32];
    bool mEncrypted;
};

// Builds a FilePack; entries are held in memory, already deflated, until write().
class FilePackWriter
{
public:
    FilePackWriter();
    ~FilePackWriter();

    FilePackWriter(const FilePackWriter&) = delete;
    FilePackWriter& operator=(const FilePackWriter&) = delete;

    // An empty key means no encryption, not a key of the empty string.
    void setKey(const std::string& key);
    void setCompressionLevel(int level);

    // `name` is what FilePack::readBinary() is asked for; adding a name twice fails.
    bool addFile(const std::string& name, const std::string& path);
    bool addData(const std::string& name, const u8* data, usize size);

    bool write(const std::string& path);

    u32 entryCount() const;
    u64 rawBytes() const;
    u64 storedBytes() const;

private:
    struct Pending
    {
        std::string name;
        ByteArray stored;
        u32 sizeRaw;
        u32 crc;
        u16 flags;
    };

    std::vector<Pending> mPending;
    HashMap<std::string, u32> mIndex;
    std::string mKey;
    int mLevel;
    u64 mRawBytes;
    u64 mStoredBytes;
};

} // namespace Radion

#endif // RADION_FILE_PACK_H
