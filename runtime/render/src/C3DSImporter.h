#ifndef RADION_C3DS_IMPORTER_H
#define RADION_C3DS_IMPORTER_H

#include "MeshLoader.h"

namespace Radion
{

class C3DSImporter final : public MeshImporter
{
public:
    C3DSImporter() = default;

    bool supports(const char* extension) const override;
    bool import(const std::string& filename, ByteArray& data, FileSystem& files,
                MeshData& mesh) override;
};

} // namespace Radion

#endif // RADION_C3DS_IMPORTER_H
