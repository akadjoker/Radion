#ifndef RADION_MESH_IMPORTER_H
#define RADION_MESH_IMPORTER_H

#include "MeshLoader.h"

namespace Radion
{

class RadionMeshImporter final : public MeshImporter
{
public:
    bool supports(const char* extension) const override;
    bool import(const std::string& filename, ByteArray& data, FileSystem& files,
                MeshData& mesh) override;
};

bool saveRadionMesh(const std::string& filename, const MeshData& mesh,
                    const std::string& skeletonFile = std::string());

} // namespace Radion

#endif // RADION_MESH_IMPORTER_H
