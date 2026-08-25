#include "PCH.h"

#include "ChaCha20.h"
#include "FilePack.h"
#include "Log.h"
#include "miniz.h"

#include <cstring>
#include <random>

namespace Radion
{

namespace
{

const char kMagic[4] = {'R', 'P', 'A', 'K'};

// Distinct from every entry nonce, which are block indices counting up from 0.
void directoryNonce(u8 nonce[ChaCha20::NonceSize])
{
    std::memset(nonce, 0xff, ChaCha20::NonceSize);
}

void entryNonce(u8 nonce[ChaCha20::NonceSize], u32 index)
{
    std::memset(nonce, 0, ChaCha20::NonceSize);
    nonce[0] = static_cast<u8>(index);
    nonce[1] = static_cast<u8>(index >> 8);
    nonce[2] = static_cast<u8>(index >> 16);
    nonce[3] = static_cast<u8>(index >> 24);
}

void cipher(const u8 key[ChaCha20::KeySize], const u8 nonce[ChaCha20::NonceSize], u8* data,
            usize size)
{
    ChaCha20 stream;
    stream.setKey(key);
    stream.setNonce(nonce, 0);
    stream.process(data, size);
}

u32 keyChecksum(const u8 key[ChaCha20::KeySize])
{
    return static_cast<u32>(mz_crc32(MZ_CRC32_INIT, key, ChaCha20::KeySize));
}

std::string normalize(const std::string& name)
{
    std::string out = name;
    for (usize i = 0; i < out.size(); ++i)
        if (out[i] == '\\')
            out[i] = '/';
    return out;
}

const std::string kEmptyName;

} // namespace

// ------------------------------------------------------------------- reading

FilePack::FilePack() : mFile(nullptr), mMemory(nullptr), mMemorySize(0), mEncrypted(false)
{
    std::memset(mKey, 0, sizeof(mKey));
}

FilePack::~FilePack()
{
    close();
}

void FilePack::close()
{
    if (mFile)
    {
        std::fclose(mFile);
        mFile = nullptr;
    }
    mMemory = nullptr;
    mMemorySize = 0;
    mEntries.clear();
    mIndex.clear();
    std::memset(mKey, 0, sizeof(mKey));
    mEncrypted = false;
}

bool FilePack::isOpen() const
{
    return mFile != nullptr || mMemory != nullptr;
}

bool FilePack::readAt(u64 offset, void* destination, usize size) const
{
    if (size == 0)
        return true;

    if (mMemory)
    {
        // Subtraction, never addition: offset comes out of the file's own
        // header, and offset + size can wrap on a malformed pack.
        if (offset > mMemorySize || size > mMemorySize - offset)
            return false;
        std::memcpy(destination, mMemory + offset, size);
        return true;
    }

    if (!mFile)
        return false;
    if (std::fseek(mFile, static_cast<long>(offset), SEEK_SET) != 0)
        return false;
    return std::fread(destination, 1, size, mFile) == size;
}

bool FilePack::open(const std::string& path, const std::string& key)
{
    close();

    mFile = std::fopen(path.c_str(), "rb");
    if (!mFile)
    {
        Log::error("FilePack: could not open %s", path.c_str());
        return false;
    }

    return readDirectory(path.c_str(), key);
}

bool FilePack::openFromMemory(const u8* data, usize size, const std::string& key)
{
    close();

    if (!data || size == 0)
    {
        Log::error("FilePack: no bytes to open as a pack");
        return false;
    }

    mMemory = data;
    mMemorySize = size;

    return readDirectory("<memory>", key);
}

bool FilePack::readDirectory(const char* label, const std::string& key)
{
    u8 header[HeaderSize];
    if (!readAt(0, header, HeaderSize))
    {
        Log::error("FilePack: %s is too short to hold a header", label);
        close();
        return false;
    }

    ByteArray view(header, HeaderSize);
    char magic[4];
    view.readBytes(magic, 4);
    if (std::memcmp(magic, kMagic, 4) != 0)
    {
        Log::error("FilePack: %s is not a pack (bad magic)", label);
        close();
        return false;
    }

    const u32 version = view.readU32();
    if (version != Version)
    {
        Log::error("FilePack: %s is version %u, this build reads version %u", label, version,
                   Version);
        close();
        return false;
    }

    const u32 flags = view.readU32();
    const u32 entryCount = view.readU32();
    const u64 tocOffset = view.readU64();
    const u32 tocSize = view.readU32();
    const u32 namesSize = view.readU32();
    u8 salt[SaltSize];
    view.readBytes(salt, SaltSize);
    const u32 keyCheck = view.readU32();

    mEncrypted = (flags & PackEncrypted) != 0;
    if (mEncrypted)
    {
        if (key.empty())
        {
            Log::error("FilePack: %s is encrypted and no key was given", label);
            close();
            return false;
        }
        ChaCha20::deriveKey(key, salt, mKey);
        if (keyChecksum(mKey) != keyCheck)
        {
            Log::error("FilePack: wrong key for %s", label);
            close();
            return false;
        }
    }
    else if (!key.empty())
    {
        Log::warning("FilePack: %s carries no encryption; the key given is unused", label);
    }

    if (tocSize != entryCount * static_cast<u32>(TocEntrySize))
    {
        Log::error("FilePack: %s declares %u entries but a %u byte table", label, entryCount,
                   tocSize);
        close();
        return false;
    }

    const usize directorySize = static_cast<usize>(tocSize) + namesSize;
    ByteArray directory(directorySize);
    if (!readAt(tocOffset, directory.data(), directorySize))
    {
        Log::error("FilePack: could not read the directory of %s", label);
        close();
        return false;
    }

    if (mEncrypted)
    {
        u8 nonce[ChaCha20::NonceSize];
        directoryNonce(nonce);
        cipher(mKey, nonce, directory.data(), directorySize);
    }

    const char* names = reinterpret_cast<const char*>(directory.data()) + tocSize;

    mEntries.reserve(entryCount);
    for (u32 i = 0; i < entryCount; ++i)
    {
        const u32 nameOffset = directory.readU32();
        const u32 nameLength = directory.readU32();
        Entry entry;
        entry.dataOffset = directory.readU64();
        entry.sizeStored = directory.readU32();
        entry.sizeRaw = directory.readU32();
        entry.crc = directory.readU32();
        entry.flags = directory.readU16();
        directory.readU16();

        if (static_cast<usize>(nameOffset) + nameLength > namesSize)
        {
            Log::error("FilePack: %s has an entry whose name falls outside the name blob", label);
            close();
            return false;
        }
        entry.name.assign(names + nameOffset, nameLength);

        mIndex[entry.name] = static_cast<u32>(mEntries.size());
        mEntries.push_back(std::move(entry));
    }

    return true;
}

bool FilePack::exists(const std::string& name) const
{
    return isOpen() && mIndex.find(normalize(name)) != mIndex.end();
}

ByteArray FilePack::readBinary(const std::string& name) const
{
    if (!isOpen())
        return ByteArray();

    const std::string key = normalize(name);
    HashMap<std::string, u32>::const_iterator it = mIndex.find(key);
    if (it == mIndex.end())
        return ByteArray();

    const Entry& entry = mEntries[it->second];

    ByteArray stored(entry.sizeStored);
    if (!readAt(entry.dataOffset, stored.data(), entry.sizeStored))
    {
        Log::error("FilePack: could not read '%s'", key.c_str());
        return ByteArray();
    }

    if (mEncrypted)
    {
        u8 nonce[ChaCha20::NonceSize];
        entryNonce(nonce, it->second);
        cipher(mKey, nonce, stored.data(), entry.sizeStored);
    }

    ByteArray raw;
    if (entry.flags & EntryDeflated)
    {
        ByteArray inflated(entry.sizeRaw);
        mz_ulong destinationSize = entry.sizeRaw;
        const int status = mz_uncompress(inflated.data(), &destinationSize, stored.data(),
                                         static_cast<mz_ulong>(entry.sizeStored));
        if (status != MZ_OK || destinationSize != entry.sizeRaw)
        {
            Log::error("FilePack: '%s' failed to inflate (%s)", key.c_str(), mz_error(status));
            return ByteArray();
        }
        raw = std::move(inflated);
    }
    else
    {
        raw = std::move(stored);
    }

    const u32 crc = static_cast<u32>(mz_crc32(MZ_CRC32_INIT, raw.data(), raw.size()));
    if (crc != entry.crc)
    {
        Log::error("FilePack: '%s' failed its checksum", key.c_str());
        return ByteArray();
    }

    return raw;
}

u32 FilePack::entryCount() const
{
    return static_cast<u32>(mEntries.size());
}

const std::string& FilePack::entryName(u32 index) const
{
    if (index >= mEntries.size())
        return kEmptyName;
    return mEntries[index].name;
}

u32 FilePack::entrySizeRaw(u32 index) const
{
    if (index >= mEntries.size())
        return 0;
    return mEntries[index].sizeRaw;
}

u32 FilePack::entrySizeStored(u32 index) const
{
    if (index >= mEntries.size())
        return 0;
    return mEntries[index].sizeStored;
}

// ------------------------------------------------------------------- writing

FilePackWriter::FilePackWriter() : mLevel(MZ_BEST_COMPRESSION), mRawBytes(0), mStoredBytes(0)
{
}

FilePackWriter::~FilePackWriter()
{
}

void FilePackWriter::setKey(const std::string& key)
{
    mKey = key;
}

void FilePackWriter::setCompressionLevel(int level)
{
    if (level < 0)
        level = 0;
    if (level > MZ_BEST_COMPRESSION)
        level = MZ_BEST_COMPRESSION;
    mLevel = level;
}

bool FilePackWriter::addFile(const std::string& name, const std::string& path)
{
    std::FILE* file = std::fopen(path.c_str(), "rb");
    if (!file)
    {
        Log::error("FilePackWriter: could not open %s", path.c_str());
        return false;
    }

    std::fseek(file, 0, SEEK_END);
    const long length = std::ftell(file);
    std::fseek(file, 0, SEEK_SET);
    if (length < 0)
    {
        Log::error("FilePackWriter: could not measure %s", path.c_str());
        std::fclose(file);
        return false;
    }

    ByteArray data(static_cast<usize>(length));
    if (length > 0 && std::fread(data.data(), 1, static_cast<usize>(length), file) !=
                          static_cast<usize>(length))
    {
        Log::error("FilePackWriter: could not read %s", path.c_str());
        std::fclose(file);
        return false;
    }
    std::fclose(file);

    return addData(name, data.data(), data.size());
}

bool FilePackWriter::addData(const std::string& name, const u8* data, usize size)
{
    const std::string key = normalize(name);
    if (key.empty())
    {
        Log::error("FilePackWriter: an entry cannot have an empty name");
        return false;
    }
    if (mIndex.find(key) != mIndex.end())
    {
        Log::error("FilePackWriter: '%s' was already added", key.c_str());
        return false;
    }
    if (size > 0xffffffffull)
    {
        Log::error("FilePackWriter: '%s' is larger than 4 GiB", key.c_str());
        return false;
    }

    Pending pending;
    pending.name = key;
    pending.sizeRaw = static_cast<u32>(size);
    pending.crc = static_cast<u32>(mz_crc32(MZ_CRC32_INIT, data, size));
    pending.flags = 0;

    bool deflated = false;
    if (mLevel > 0 && size > 0)
    {
        mz_ulong bound = mz_compressBound(static_cast<mz_ulong>(size));
        ByteArray compressed(static_cast<usize>(bound));
        mz_ulong compressedSize = bound;
        const int status =
            mz_compress2(compressed.data(), &compressedSize, data, static_cast<mz_ulong>(size),
                         mLevel);
        // Storing it raw when deflate did not help keeps the reader from
        // paying to inflate bytes that never got smaller.
        if (status == MZ_OK && compressedSize < size)
        {
            ByteArray trimmed(static_cast<usize>(compressedSize));
            std::memcpy(trimmed.data(), compressed.data(), static_cast<usize>(compressedSize));
            pending.stored = std::move(trimmed);
            pending.flags |= FilePack::EntryDeflated;
            deflated = true;
        }
    }

    if (!deflated)
    {
        ByteArray plain(size);
        if (size > 0)
            std::memcpy(plain.data(), data, size);
        pending.stored = std::move(plain);
    }

    mRawBytes += pending.sizeRaw;
    mStoredBytes += pending.stored.size();
    mIndex[key] = static_cast<u32>(mPending.size());
    mPending.push_back(std::move(pending));
    return true;
}

bool FilePackWriter::write(const std::string& path)
{
    u8 salt[FilePack::SaltSize];
    std::random_device source;
    for (usize i = 0; i < FilePack::SaltSize; ++i)
        salt[i] = static_cast<u8>(source() & 0xffu);

    const bool encrypted = !mKey.empty();
    u8 key[ChaCha20::KeySize];
    std::memset(key, 0, sizeof(key));
    if (encrypted)
        ChaCha20::deriveKey(mKey, salt, key);

    std::string names;
    ByteArray toc(mPending.size() * FilePack::TocEntrySize);
    u64 offset = FilePack::HeaderSize;
    for (usize i = 0; i < mPending.size(); ++i)
    {
        const Pending& pending = mPending[i];
        toc.writeU32(static_cast<u32>(names.size()));
        toc.writeU32(static_cast<u32>(pending.name.size()));
        toc.writeU64(offset);
        toc.writeU32(static_cast<u32>(pending.stored.size()));
        toc.writeU32(pending.sizeRaw);
        toc.writeU32(pending.crc);
        toc.writeU16(pending.flags);
        toc.writeU16(0);
        names += pending.name;
        offset += pending.stored.size();
    }

    std::FILE* file = std::fopen(path.c_str(), "wb");
    if (!file)
    {
        Log::error("FilePackWriter: could not create %s", path.c_str());
        return false;
    }

    ByteArray header(FilePack::HeaderSize);
    header.writeBytes(kMagic, 4);
    header.writeU32(FilePack::Version);
    header.writeU32(encrypted ? static_cast<u32>(FilePack::PackEncrypted) : 0u);
    header.writeU32(static_cast<u32>(mPending.size()));
    header.writeU64(offset);
    header.writeU32(static_cast<u32>(toc.size()));
    header.writeU32(static_cast<u32>(names.size()));
    header.writeBytes(salt, FilePack::SaltSize);
    header.writeU32(encrypted ? keyChecksum(key) : 0u);
    header.writeU32(0);
    header.writeU32(0);
    header.writeU32(0);

    bool ok = std::fwrite(header.data(), 1, FilePack::HeaderSize, file) == FilePack::HeaderSize;

    for (usize i = 0; ok && i < mPending.size(); ++i)
    {
        ByteArray& stored = mPending[i].stored;
        if (encrypted && stored.size() > 0)
        {
            u8 nonce[ChaCha20::NonceSize];
            entryNonce(nonce, static_cast<u32>(i));
            cipher(key, nonce, stored.data(), stored.size());
        }
        if (stored.size() > 0)
            ok = std::fwrite(stored.data(), 1, stored.size(), file) == stored.size();
    }

    if (ok)
    {
        const usize directorySize = toc.size() + names.size();
        ByteArray directory(directorySize);
        std::memcpy(directory.data(), toc.data(), toc.size());
        std::memcpy(directory.data() + toc.size(), names.data(), names.size());
        if (encrypted && directorySize > 0)
        {
            u8 nonce[ChaCha20::NonceSize];
            directoryNonce(nonce);
            cipher(key, nonce, directory.data(), directorySize);
        }
        ok = std::fwrite(directory.data(), 1, directorySize, file) == directorySize;
    }

    std::fclose(file);

    if (!ok)
        Log::error("FilePackWriter: writing %s failed part way through", path.c_str());
    return ok;
}

u32 FilePackWriter::entryCount() const
{
    return static_cast<u32>(mPending.size());
}

u64 FilePackWriter::rawBytes() const
{
    return mRawBytes;
}

u64 FilePackWriter::storedBytes() const
{
    return mStoredBytes;
}

} // namespace Radion
