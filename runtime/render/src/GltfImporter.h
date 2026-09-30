#ifndef RADION_GLTF_IMPORTER_H
#define RADION_GLTF_IMPORTER_H

#include "MeshLoader.h"

namespace Radion
{

class Skeleton;
class AnimationClip;

// glTF 2.0 / GLB mesh importer; skeleton and animation load through the free functions below.
class GltfImporter final : public MeshImporter
{
public:
    GltfImporter() = default;

    bool supports(const char* extension) const override;
    bool import(const std::string& filename, ByteArray& data, FileSystem& files,
                MeshData& mesh) override;
};

bool loadGltfSkeleton(const std::string& filename, FileSystem& files, Skeleton& skeleton);
bool loadGltfAnimation(const std::string& filename, FileSystem& files, const Skeleton& skeleton,
                       AnimationClip& clip);

} // namespace Radion

#endif // RADION_GLTF_IMPORTER_H
