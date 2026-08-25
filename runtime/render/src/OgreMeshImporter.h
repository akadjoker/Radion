#ifndef RADION_OGRE_MESH_IMPORTER_H
#define RADION_OGRE_MESH_IMPORTER_H

#include "MeshLoader.h"
#include "Skeleton.h"

namespace Radion
{

struct OgreModelData
{
    MeshData mesh;
    Skeleton skeleton;
    std::vector<AnimationClip> animations;
    std::string skeletonLink;
    bool hasSkeleton = false;
};

// Imports the mesh and resolves its linked Ogre skeleton through FileSystem.
// Unlike MeshImporter::import(), this entry point preserves skinning and clips.
bool loadOgreModel(const std::string& meshFilename, ByteArray& meshBytes, FileSystem& files,
                   OgreModelData& output);

// MeshLoader adapter. Use loadOgreModel() when the skeleton and animations are
// needed in addition to the bind-pose mesh.
class OgreMeshImporter final : public MeshImporter
{
public:
    bool supports(const char* extension) const override;
    bool import(const std::string& filename, ByteArray& data, FileSystem& files,
                MeshData& mesh) override;
};

} // namespace Radion

#endif // RADION_OGRE_MESH_IMPORTER_H
