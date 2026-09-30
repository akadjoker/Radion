#ifndef RADION_BSP_IMPORTER_H
#define RADION_BSP_IMPORTER_H

#include "MeshLoader.h"

namespace Radion
{

// Quake III IBSP v46 importer; converts Z-up to Y-up. Entities and brushes are out of scope.
class BSPImporter final : public MeshImporter
{
public:
    explicit BSPImporter(u32 patchTessellation = 5, bool worldspawnOnly = true);

    bool supports(const char* extension) const override;
    bool import(const std::string& filename, ByteArray& data, FileSystem& files,
                MeshData& mesh) override;

private:
    u32 mPatchTessellation;
    bool mWorldspawnOnly;
};

} // namespace Radion

#endif // RADION_BSP_IMPORTER_H
