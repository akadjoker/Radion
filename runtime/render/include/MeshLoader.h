#ifndef RADION_MESH_LOADER_H
#define RADION_MESH_LOADER_H

#include "Mesh.h"

#include <string>
#include <vector>

namespace Radion
{

class ByteArray;
class FileSystem;

// A format importer only decodes memory. Any secondary file it needs must be
// read through the supplied FileSystem, never through platform or std I/O.
class MeshImporter
{
public:
    virtual ~MeshImporter() = default;

    virtual bool supports(const char* extension) const = 0;
    virtual bool import(const std::string& filename, ByteArray& data, FileSystem& files,
                        MeshData& mesh) = 0;
};

class MeshLoader
{
public:
    MeshLoader();
    ~MeshLoader();

    MeshLoader(const MeshLoader&) = delete;
    MeshLoader& operator=(const MeshLoader&) = delete;

    // Takes ownership. Importers are tried in registration order.
    void addImporter(MeshImporter* importer);

    // Reads through FileSystem into ByteArray, then delegates decoding. Output
    // is only replaced after a successful import.
    bool load(const std::string& filename, MeshData& mesh);

    // Imports bytes already in memory. virtualName supplies the extension and
    // base directory for secondary assets; the main file is never reopened.
    bool loadFromMemory(const std::string& virtualName, ByteArray& data, MeshData& mesh);

private:
    std::vector<MeshImporter*> mImporters;
};

} // namespace Radion

#endif // RADION_MESH_LOADER_H
