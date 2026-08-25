#ifndef RADION_BSP_IMPORTER_H
#define RADION_BSP_IMPORTER_H

#include "MeshLoader.h"

namespace Radion
{

// Quake III Arena BSP (IBSP version 46) geometry importer. It converts the
// source Z-up coordinates to Radion's Y-up space and imports polygon/mesh
// faces plus quadratic Bezier patches. BSP entities and collision brushes are
// deliberately outside MeshImporter's MeshData-only contract.
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
