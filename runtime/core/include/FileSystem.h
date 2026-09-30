#ifndef RADION_FILE_SYSTEM_H
#define RADION_FILE_SYSTEM_H

#include "ByteArray.h"
#include "Containers.h"
#include "Types.h"

#include <string>
#include <vector>

namespace Radion
{

class Archive;

class FileSystem
{
public:
    FileSystem();
    ~FileSystem();

    FileSystem(const FileSystem&) = delete;
    FileSystem& operator=(const FileSystem&) = delete;

    void addSearchPath(const std::string& path);
    // Exact string match, no normalization; no-op if never added.
    void removeSearchPath(const std::string& path);
    const std::vector<std::string>& getSearchPaths() const
    {
        return mSearchPaths;
    }

    // Mounted archives are searched before search paths, in mount order.
    bool mountArchive(const std::string& zipPath);
    bool mountPack(const std::string& packPath, const std::string& key);
    // Consulted only after all search paths fail; for a build's embedded fallback files. Disk still wins.
    bool mountFallbackPack(const u8* data, usize size, const std::string& key);
    void unmountAll();

    bool exists(const std::string& name) const;

    std::string resolve(const std::string& name) const;

    ByteArray readBinary(const std::string& name) const;
    std::string readText(const std::string& name) const;

    // Takes a real path (no search-path resolution); archives are never involved.
    bool writeBinary(const std::string& path, const ByteArray& data) const;
    bool writeText(const std::string& path, const std::string& text) const;

    // Real on-disk listing: not archive-aware or search-path-relative.
    struct DirEntry
    {
        std::string name;
        bool isDirectory;
        uint64 size;        // bytes; 0 for directories
        int64 modifiedTime; // seconds since epoch
    };
    std::vector<DirEntry> listDirectory(const std::string& path) const;
    bool isDirectory(const std::string& path) const;
    bool createDirectory(const std::string& path) const;
    // Takes a real path, not a search-path name. Directories and archive members are never touched.
    bool removeFile(const std::string& path) const;
    static std::string fileName(const std::string& path);
    static std::string baseName(const std::string& path);
    static std::string directoryOf(const std::string& path);
    static std::string extensionOf(const std::string& path);
    static std::string withoutExtension(const std::string& path);
    static std::string join(const std::string& directory, const std::string& name);

    std::string getWorkingDirectory() const;
    bool setWorkingDirectory(const std::string& path);

    // Per-(organization, application) writable folder, created if missing:
    //   Windows: %APPDATA%/<organization>/<application>/
    //   Linux:   ~/.local/share/<application>/
    //   macOS:   ~/Library/Application Support/<organization>/<application>/
    // Empty on failure; trailing slash included (as SDL_GetPrefPath).
    std::string prefPath(const std::string& organization, const std::string& application) const;

    static FileSystem& getSingleton();

private:
    bool fileReadable(const std::string& path) const;
    bool existsOnDisk(const std::string& name) const;
    std::string resolveOnDisk(const std::string& name) const;
    ByteArray readBinaryOnDisk(const std::string& name) const;
    bool writeBinaryOnDisk(const std::string& path, const ByteArray& data) const;

    std::vector<std::string> mSearchPaths;
    // Owned; mArchives is searched before the disk, mFallbacks after.
    std::vector<Archive*> mArchives;
    std::vector<Archive*> mFallbacks;

    // Names already resolved on disk (a miss costs one open per search path). Failures are not
    // cached so a later-written file is found; appending a path cannot invalidate entries.
    mutable HashMap<std::string, std::string> mResolvedOnDisk;
};

} // namespace Radion

#endif // RADION_FILE_SYSTEM_H
