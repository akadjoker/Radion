#ifndef RADION_OBJ_IMPORTER_H
#define RADION_OBJ_IMPORTER_H

#include "MeshLoader.h"

namespace Radion
{

class ObjImporter final : public MeshImporter
{
public:
    explicit ObjImporter(bool mergeSameMaterial = true);

    bool supports(const char* extension) const override;
    bool import(const std::string& filename, ByteArray& data, FileSystem& files,
                MeshData& mesh) override;

private:
    bool mMergeSameMaterial;
};

} // namespace Radion

#endif // RADION_OBJ_IMPORTER_H
