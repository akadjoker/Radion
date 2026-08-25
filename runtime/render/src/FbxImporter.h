#ifndef RADION_FBX_IMPORTER_H
#define RADION_FBX_IMPORTER_H

#include "MeshLoader.h"

namespace Radion
{

class Skeleton;
class AnimationClip;

// FBX mesh importer built on top of ofbx (runtime/render/src/ofbx.h).
// Follows the same shape as B3DImporter/GltfImporter: geometry is decoded into
// a MeshData by FbxImporter::import(), skeleton and animation clips are loaded
// through the two free functions below.
class FbxImporter final : public MeshImporter
{
public:
    FbxImporter() = default;

    bool supports(const char* extension) const override;
    bool import(const std::string& filename, ByteArray& data, FileSystem& files,
                MeshData& mesh) override;
};

bool loadFbxSkeleton(const std::string& filename, FileSystem& files, Skeleton& skeleton);
// If keepRootMotion is false, the functional root bone's horizontal position is
// pinned to the bind pose and only vertical motion relative to the first frame is
// kept. This converts a locomotion animation into an in-place animation, which
// is what most game loops expect.
bool loadFbxAnimation(const std::string& filename, FileSystem& files, const Skeleton& skeleton,
                      AnimationClip& clip, bool keepRootMotion = true);

} // namespace Radion

#endif // RADION_FBX_IMPORTER_H
