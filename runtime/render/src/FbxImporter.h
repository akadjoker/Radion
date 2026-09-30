#ifndef RADION_FBX_IMPORTER_H
#define RADION_FBX_IMPORTER_H

#include "MeshLoader.h"

namespace Radion
{

class Skeleton;
class AnimationClip;

// FBX mesh importer built on ofbx; skeleton and animation load through the free functions below.
class FbxImporter final : public MeshImporter
{
public:
    FbxImporter() = default;

    bool supports(const char* extension) const override;
    bool import(const std::string& filename, ByteArray& data, FileSystem& files,
                MeshData& mesh) override;
};

bool loadFbxSkeleton(const std::string& filename, FileSystem& files, Skeleton& skeleton);
// keepRootMotion false: pin the root's horizontal position to bind pose for an in-place animation.
bool loadFbxAnimation(const std::string& filename, FileSystem& files, const Skeleton& skeleton,
                      AnimationClip& clip, bool keepRootMotion = true);

} // namespace Radion

#endif // RADION_FBX_IMPORTER_H
