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

    // Registers a folder to search for assets, tried in the order added.
    void addSearchPath(const std::string& path);
    // Undoes addSearchPath() for the exact string given - no path
    // normalization, pass the same value that was added. A no-op if it was
    // never added (already gone, or never there).
    void removeSearchPath(const std::string& path);
    const std::vector<std::string>& getSearchPaths() const
    {
        return mSearchPaths;
    }

    // Mounted archives answer before the search paths do, in the order
    // mounted: a name present in a pack is read from it even when the same
    // name sits loose on disk.
    bool mountArchive(const std::string& zipPath);
    // `key` must match the one the pack was written with, and is empty for a
    // pack written without one.
    bool mountPack(const std::string& packPath, const std::string& key);
    // The opposite priority: consulted only after the search paths have all
    // failed. For a build's own embedded copy of the files it cannot start
    // without - whatever is on disk still wins, so editing a shader in the
    // assets folder keeps working exactly as it did before.
    bool mountFallbackPack(const u8* data, usize size, const std::string& key);
    void unmountAll();

    bool exists(const std::string& name) const;

    std::string resolve(const std::string& name) const;

    ByteArray readBinary(const std::string& name) const;
    std::string readText(const std::string& name) const;

    // Writes straight to `path` (no search-path resolution - unlike
    // reading, there's no existing file to find candidates for, so the
    // caller gives a real path, e.g. one of its own getSearchPaths()
    // entries + a filename). Archives are read-only and never involved.
    // Returns false on any I/O failure (logged by the platform backend).
    bool writeBinary(const std::string& path, const ByteArray& data) const;
    bool writeText(const std::string& path, const std::string& text) const;

    // Real on-disk directory browsing (file/folder-open dialogs, etc.) —
    // unlike exists()/readBinary()/... above, this is NOT archive-aware and
    // NOT search-path-relative: it lists exactly what's on disk at `path`,
    // same distinction Irrlicht draws between IFileSystem (path resolution)
    // and IFileList (a literal directory's contents). Implemented once here
    // (not per-platform) via POSIX dirent/stat or Win32 FindFirstFile.
    struct DirEntry
    {
        std::string name; // filename/dirname only, no path
        bool isDirectory;
        uint64 size;        // bytes; 0 for directories
        int64 modifiedTime; // seconds since epoch
    };
    std::vector<DirEntry> listDirectory(const std::string& path) const;
    bool isDirectory(const std::string& path) const;
    bool createDirectory(const std::string& path) const;
    // Deletes one file on disk. Like writeBinary() this takes a real path,
    // not a search-path-relative name: deleting is not something to do to
    // whichever candidate a search happened to find first. Directories and
    // archive members are never touched.
    bool removeFile(const std::string& path) const;
    static std::string fileName(const std::string& path);
    static std::string baseName(const std::string& path);
    static std::string directoryOf(const std::string& path);
    static std::string extensionOf(const std::string& path);
    static std::string withoutExtension(const std::string& path);
    static std::string join(const std::string& directory, const std::string& name);

    std::string getWorkingDirectory() const;
    bool setWorkingDirectory(const std::string& path);

    // Where a build's own files (settings, save games, logs) belong instead
    // of the working directory - the engine has no say in what that is once
    // installed, and on some platforms it is not even writable. One real
    // folder per (organization, application), created if missing:
    //   Windows: %APPDATA%/<organization>/<application>/
    //   Linux:   ~/.local/share/<application>/
    //   macOS:   ~/Library/Application Support/<organization>/<application>/
    // Empty on failure. Trailing slash included, matching SDL_GetPrefPath.
    std::string prefPath(const std::string& organization, const std::string& application) const;

    // Process-wide SDL filesystem instance.
    static FileSystem& getSingleton();

private:
    bool fileReadable(const std::string& path) const;
    bool existsOnDisk(const std::string& name) const;
    std::string resolveOnDisk(const std::string& name) const;
    ByteArray readBinaryOnDisk(const std::string& name) const;
    bool writeBinaryOnDisk(const std::string& path, const ByteArray& data) const;

    std::vector<std::string> mSearchPaths;
    // Both owned by this FileSystem; deleted in the destructor. mArchives is
    // searched before the disk, mFallbacks after it.
    std::vector<Archive*> mArchives;
    std::vector<Archive*> mFallbacks;

    // Names already found on disk, and where. A miss costs one open attempt
    // per search path, and the same name is asked for over and over - a
    // level's material file alone resolves the same handful of textures once
    // per material that uses them, which is hundreds of walks down the whole
    // list. Only successful lookups are kept: a name that did not resolve is
    // retried, so a file written after being asked for is still found.
    // Appending a search path cannot invalidate an entry, since resolution
    // takes the first path that answers and new ones go on the end.
    mutable HashMap<std::string, std::string> mResolvedOnDisk;
};

} // namespace Radion

#endif // RADION_FILE_SYSTEM_H
