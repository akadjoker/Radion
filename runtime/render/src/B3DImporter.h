#ifndef RADION_B3D_IMPORTER_H
#define RADION_B3D_IMPORTER_H

#include "MeshLoader.h"

namespace Radion
{

class Skeleton;
class AnimationClip;

class B3DImporter final : public MeshImporter
{
public:
    B3DImporter() = default;

    bool supports(const char* extension) const override;
    bool import(const std::string& filename, ByteArray& data, FileSystem& files,
                MeshData& mesh) override;
};

bool loadB3DSkeleton(const std::string& filename, FileSystem& files, Skeleton& skeleton);
bool loadB3DAnimation(const std::string& filename, FileSystem& files, const Skeleton& skeleton,
                      AnimationClip& clip);

} // namespace Radion

#endif // RADION_B3D_IMPORTER_H
