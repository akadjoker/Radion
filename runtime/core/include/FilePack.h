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

// Radion's own archive: one file holding many, deflated and optionally
// encrypted, laid out as
//
//   header      64 bytes, the only part ever stored in the clear
//   data        every entry's bytes, back to back, in the order added
//   directory   the table below plus the name blob, encrypted as one region
//
// Unlike ZipArchive, which keeps the whole archive in memory, only the
// directory is resident: an entry's bytes are read from the open file when
// asked for. Reads are not thread-safe - one file cursor is shared.
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

    // Pass the same key the pack was written with, or an empty string for one
    // written without. A wrong key is rejected here, with its own message,
    // rather than surfacing later as entries that fail their checksum.
    bool open(const std::string& path, const std::string& key);
    // Reads a pack that is already in memory - a build's own embedded copy,
    // typically. The bytes are borrowed, not copied: they must outlive this
    // FilePack, which static storage does by definition.
    bool openFromMemory(const u8* data, usize size, const std::string& key);
    void close();
    bool isOpen() const;

    bool exists(const std::string& name) const override;
    ByteArray readBinary(const std::string& name) const override;

    u32 entryCount() const;
    // Empty / zero for an index past entryCount().
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

// Builds a FilePack. Entries are held in memory until write(), already
// deflated, so a pack costs about its own compressed size to assemble.
class FilePackWriter
{
public:
    FilePackWriter();
    ~FilePackWriter();

    FilePackWriter(const FilePackWriter&) = delete;
    FilePackWriter& operator=(const FilePackWriter&) = delete;

    // An empty key writes a pack with no encryption at all, not one keyed on
    // the empty string.
    void setKey(const std::string& key);
    // miniz deflate level, 0 (store) to 9. Defaults to 9.
    void setCompressionLevel(int level);

    // `name` is what FilePack::readBinary() will be asked for; `path` is where
    // the bytes come from now. Adding a name twice fails - the second one
    // could never be read back.
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
