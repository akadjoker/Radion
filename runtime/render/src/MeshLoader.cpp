#include "PCH.h"

#include "MeshLoader.h"

#include "ByteArray.h"
#include "FileSystem.h"
#include "Log.h"

namespace Radion
{

namespace
{

std::string extensionOf(const std::string& filename)
{
    const usize slash = filename.find_last_of("/\\");
    const usize dot = filename.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash) ||
        dot + 1 >= filename.size())
        return {};

    std::string extension = filename.substr(dot + 1);
    for (char& c : extension)
    {
        if (c >= 'A' && c <= 'Z')
            c = static_cast<char>(c - 'A' + 'a');
    }
    return extension;
}

} // namespace

MeshLoader::MeshLoader()
{
}

MeshLoader::~MeshLoader()
{
    for (MeshImporter* importer : mImporters)
        delete importer;
}

void MeshLoader::addImporter(MeshImporter* importer)
{
    if (!importer)
        return;
    mImporters.push_back(importer);
}

bool MeshLoader::load(const std::string& filename, MeshData& mesh)
{
    FileSystem& files = FileSystem::getSingleton();
    ByteArray data = files.readBinary(filename);
    if (data.empty())
    {
        Log::error("MeshLoader: could not read '%s'", filename.c_str());
        return false;
    }

    return loadFromMemory(filename, data, mesh);
}

bool MeshLoader::loadFromMemory(const std::string& virtualName, ByteArray& data, MeshData& mesh)
{
    const std::string extension = extensionOf(virtualName);
    if (extension.empty())
    {
        Log::error("MeshLoader: '%s' has no file extension", virtualName.c_str());
        return false;
    }

    MeshImporter* selected = nullptr;
    for (MeshImporter* importer : mImporters)
    {
        if (importer->supports(extension.c_str()))
        {
            selected = importer;
            break;
        }
    }

    if (!selected)
    {
        Log::error("MeshLoader: no importer registered for '.%s'", extension.c_str());
        return false;
    }

    if (data.empty())
    {
        Log::error("MeshLoader: no data supplied for '%s'", virtualName.c_str());
        return false;
    }

    MeshData loaded;
    const usize originalPosition = data.tell();
    data.seek(0);
    const bool imported = selected->import(virtualName, data, FileSystem::getSingleton(), loaded);
    data.seek(static_cast<long long>(originalPosition));
    if (!imported)
    {
        Log::error("MeshLoader: importer failed for '%s'", virtualName.c_str());
        return false;
    }
    if (loaded.positions.empty() || loaded.indices.empty())
    {
        Log::error("MeshLoader: importer returned an empty mesh for '%s'", virtualName.c_str());
        return false;
    }

    mesh = std::move(loaded);
    return true;
}

} // namespace Radion
