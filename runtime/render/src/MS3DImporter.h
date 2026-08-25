#ifndef RADION_MS3D_IMPORTER_H
#define RADION_MS3D_IMPORTER_H

#include "MeshLoader.h"

namespace Radion
{

class Skeleton;
class AnimationClip;

class MS3DImporter final : public MeshImporter
{
public:
    MS3DImporter() = default;

    bool supports(const char* extension) const override;
    bool import(const std::string& filename, ByteArray& data, FileSystem& files,
                MeshData& mesh) override;
};

bool loadMS3DSkeleton(const std::string& filename, FileSystem& files, Skeleton& skeleton);
bool loadMS3DAnimation(const std::string& filename, FileSystem& files, const Skeleton& skeleton,
                       AnimationClip& clip);

} // namespace Radion

#endif // RADION_MS3D_IMPORTER_H
