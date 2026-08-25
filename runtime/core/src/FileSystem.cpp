#include "PCH.h"

#include "FileSystem.h"

#include "Archive.h"
#include "FilePack.h"
#include "Log.h"
#include "ZipArchive.h"

#include <SDL2/SDL.h>
#include <algorithm>
#include <cstdio>

#if defined(_WIN32)
#include <direct.h>
#include <windows.h>
#define getcwd _getcwd
#define chdir _chdir
#else
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace Radion
{

FileSystem::FileSystem()
{
}

FileSystem::~FileSystem()
{
    unmountAll();
}

void FileSystem::unmountAll()
{
    for (Archive* archive : mArchives)
        delete archive;
    mArchives.clear();

    for (Archive* archive : mFallbacks)
        delete archive;
    mFallbacks.clear();
}

void FileSystem::addSearchPath(const std::string& path)
{
    mSearchPaths.push_back(path);
}

void FileSystem::removeSearchPath(const std::string& path)
{
    mSearchPaths.erase(std::remove(mSearchPaths.begin(), mSearchPaths.end(), path),
                       mSearchPaths.end());
    // Anything resolved through the path just dropped now points at a place
    // this filesystem no longer looks.
    mResolvedOnDisk.clear();
}

bool FileSystem::fileReadable(const std::string& path) const
{
    SDL_RWops* file = SDL_RWFromFile(path.c_str(), "rb");
    if (!file)
        return false;

    const Sint64 size = SDL_RWsize(file);
    SDL_RWclose(file);
    return size >= 0;
}

std::string FileSystem::resolveOnDisk(const std::string& name) const
{
    const auto cached = mResolvedOnDisk.find(name);
    if (cached != mResolvedOnDisk.end())
        return cached->second;

    if (fileReadable(name))
    {
        mResolvedOnDisk[name] = name;
        return name;
    }

    for (const std::string& directory : mSearchPaths)
    {
        const std::string path = directory.empty() ? name : directory + "/" + name;
        if (fileReadable(path))
        {
            mResolvedOnDisk[name] = path;
            return path;
        }
    }

    return {};
}

bool FileSystem::existsOnDisk(const std::string& name) const
{
    return !resolveOnDisk(name).empty();
}

ByteArray FileSystem::readBinaryOnDisk(const std::string& name) const
{
    const std::string path = resolveOnDisk(name);
    if (path.empty())
    {
        Log::error("FileSystem: file not found: %s", name.c_str());
        return {};
    }

    SDL_RWops* file = SDL_RWFromFile(path.c_str(), "rb");
    if (!file)
    {
        Log::error("FileSystem: failed to open %s", path.c_str());
        return {};
    }

    const Sint64 fileSize = SDL_RWsize(file);
    if (fileSize <= 0)
    {
        SDL_RWclose(file);
        Log::error("FileSystem: invalid file size for %s", path.c_str());
        return {};
    }

    ByteArray data(static_cast<usize>(fileSize));
    const usize bytesRead = SDL_RWread(file, data.data(), 1, data.size());
    SDL_RWclose(file);

    if (bytesRead != data.size())
    {
        Log::error("FileSystem: short read on %s", path.c_str());
        return {};
    }

    return data;
}

bool FileSystem::writeBinaryOnDisk(const std::string& path, const ByteArray& data) const
{
    SDL_RWops* file = SDL_RWFromFile(path.c_str(), "wb");
    if (!file)
    {
        Log::error("FileSystem: failed to open %s for writing", path.c_str());
        return false;
    }

    const usize bytesWritten = data.empty() ? 0 : SDL_RWwrite(file, data.data(), 1, data.size());
    SDL_RWclose(file);

    if (bytesWritten != data.size())
    {
        Log::error("FileSystem: short write on %s", path.c_str());
        return false;
    }

    return true;
}

bool FileSystem::mountArchive(const std::string& zipPath)
{
    ByteArray data = readBinaryOnDisk(zipPath);
    if (data.empty())
    {
        Log::error("FileSystem: could not read archive %s", zipPath.c_str());
        return false;
    }

    ZipArchive* archive = new ZipArchive();
    if (!archive->openFromMemory(std::move(data)))
    {
        Log::error("FileSystem: failed to open archive %s", zipPath.c_str());
        delete archive;
        return false;
    }

    mArchives.push_back(archive);
    return true;
}

bool FileSystem::mountPack(const std::string& packPath, const std::string& key)
{
    FilePack* pack = new FilePack();
    if (!pack->open(packPath, key))
    {
        delete pack;
        return false;
    }

    mArchives.push_back(pack);
    return true;
}

bool FileSystem::mountFallbackPack(const u8* data, usize size, const std::string& key)
{
    FilePack* pack = new FilePack();
    if (!pack->openFromMemory(data, size, key))
    {
        delete pack;
        return false;
    }

    mFallbacks.push_back(pack);
    return true;
}

bool FileSystem::exists(const std::string& name) const
{
    for (const Archive* archive : mArchives)
    {
        if (archive->exists(name))
            return true;
    }
    if (existsOnDisk(name))
        return true;
    for (const Archive* archive : mFallbacks)
    {
        if (archive->exists(name))
            return true;
    }
    return false;
}

std::string FileSystem::resolve(const std::string& name) const
{
    return resolveOnDisk(name);
}

ByteArray FileSystem::readBinary(const std::string& name) const
{
    for (const Archive* archive : mArchives)
    {
        if (archive->exists(name))
            return archive->readBinary(name);
    }

    ByteArray onDisk = readBinaryOnDisk(name);
    if (!onDisk.empty())
        return onDisk;

    for (const Archive* archive : mFallbacks)
    {
        if (archive->exists(name))
            return archive->readBinary(name);
    }
    return onDisk;
}

std::string FileSystem::readText(const std::string& name) const
{
    ByteArray data = readBinary(name);
    if (data.empty())
        return std::string();
    return std::string(reinterpret_cast<const char*>(data.data()), data.size());
}

bool FileSystem::writeBinary(const std::string& path, const ByteArray& data) const
{
    return writeBinaryOnDisk(path, data);
}

bool FileSystem::writeText(const std::string& path, const std::string& text) const
{
    // Non-owning view over the string's own bytes - writeBinaryOnDisk only
    // reads from it, never frees or reallocates it.
    ByteArray view(const_cast<uint8*>(reinterpret_cast<const uint8*>(text.data())), text.size(),
                   false);
    return writeBinaryOnDisk(path, view);
}

std::vector<FileSystem::DirEntry> FileSystem::listDirectory(const std::string& path) const
{
    std::vector<DirEntry> entries;

#if defined(_WIN32)
    std::string pattern = path.empty() ? "*" : path + "\\*";
    WIN32_FIND_DATAA findData;
    HANDLE h = FindFirstFileA(pattern.c_str(), &findData);
    if (h == INVALID_HANDLE_VALUE)
        return entries;

    do
    {
        std::string name = findData.cFileName;
        if (name == "." || name == "..")
            continue;
        DirEntry entry;
        entry.name = name;
        entry.isDirectory = (findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
        entry.size = (static_cast<uint64>(findData.nFileSizeHigh) << 32) | findData.nFileSizeLow;
        int64 fileTime = (static_cast<int64>(findData.ftLastWriteTime.dwHighDateTime) << 32) |
                         findData.ftLastWriteTime.dwLowDateTime;
        entry.modifiedTime = fileTime / 10000000LL - 11644473600LL;
        entries.push_back(entry);
    } while (FindNextFileA(h, &findData));
    FindClose(h);
#else
    DIR* dir = opendir(path.empty() ? "." : path.c_str());
    if (!dir)
        return entries;

    struct dirent* de;
    while ((de = readdir(dir)) != nullptr)
    {
        std::string name = de->d_name;
        if (name == "." || name == "..")
            continue;

        std::string full = path.empty() ? name : path + "/" + name;
        struct stat st;
        bool haveStat = stat(full.c_str(), &st) == 0;
        DirEntry entry;
        entry.name = name;
        entry.isDirectory = haveStat && S_ISDIR(st.st_mode);
        entry.size = haveStat ? static_cast<uint64>(st.st_size) : 0;
        entry.modifiedTime = haveStat ? static_cast<int64>(st.st_mtime) : 0;
        entries.push_back(entry);
    }
    closedir(dir);
#endif

    return entries;
}

bool FileSystem::isDirectory(const std::string& path) const
{
#if defined(_WIN32)
    DWORD attrs = GetFileAttributesA(path.c_str());
    return attrs != INVALID_FILE_ATTRIBUTES && (attrs & FILE_ATTRIBUTE_DIRECTORY);
#else
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
#endif
}

bool FileSystem::createDirectory(const std::string& path) const
{
    if (isDirectory(path))
        return true;
#if defined(_WIN32)
    return _mkdir(path.c_str()) == 0;
#else
    return mkdir(path.c_str(), 0755) == 0;
#endif
}

bool FileSystem::removeFile(const std::string& path) const
{
    if (path.empty() || isDirectory(path))
        return false;
    if (std::remove(path.c_str()) != 0)
    {
        Log::error("FileSystem: could not delete '%s'", path.c_str());
        return false;
    }
    return true;
}

std::string FileSystem::prefPath(const std::string& organization, const std::string& application) const
{
    char* path = SDL_GetPrefPath(organization.c_str(), application.c_str());
    if (!path)
    {
        Log::error("FileSystem: SDL_GetPrefPath failed for %s/%s", organization.c_str(),
                  application.c_str());
        return {};
    }
    // SDL already creates the directory tree; nothing left to do but copy
    // the string out of SDL's own allocator.
    std::string result(path);
    SDL_free(path);
    return result;
}

std::string FileSystem::fileName(const std::string& path)
{
    const usize slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

std::string FileSystem::baseName(const std::string& path)
{
    return withoutExtension(fileName(path));
}

std::string FileSystem::directoryOf(const std::string& path)
{
    const usize slash = path.find_last_of("/\\");
    return slash == std::string::npos ? std::string() : path.substr(0, slash);
}

std::string FileSystem::extensionOf(const std::string& path)
{
    const std::string name = fileName(path);
    const usize dot = name.find_last_of('.');
    return (dot == std::string::npos || dot == 0) ? std::string() : name.substr(dot);
}

std::string FileSystem::withoutExtension(const std::string& path)
{
    const usize slash = path.find_last_of("/\\");
    const usize dot = path.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash) ||
        dot == 0 || (slash != std::string::npos && dot == slash + 1))
        return path;
    return path.substr(0, dot);
}

std::string FileSystem::join(const std::string& directory, const std::string& name)
{
    if (directory.empty())
        return name;
    if (name.empty())
        return directory;
    const char last = directory.back();
    if (last == '/' || last == '\\')
        return directory + name;
    return directory + "/" + name;
}

std::string FileSystem::getWorkingDirectory() const
{
    char buf[1024];
    if (!getcwd(buf, sizeof(buf)))
        return "";
    return std::string(buf);
}

bool FileSystem::setWorkingDirectory(const std::string& path)
{
    return chdir(path.c_str()) == 0;
}

FileSystem& FileSystem::getSingleton()
{
    static FileSystem instance;
    return instance;
}

} // namespace Radion
