#ifndef RADION_TREE_RENDER_H
#define RADION_TREE_RENDER_H

#include "GPU.h"
#include "Mesh.h"
#include "RenderTechnique.h"

#include <vector>

namespace Radion
{

// One planted tree. Four floats plus four, matching TreeInstance in tree.vert.
// `normal` is unused by the shader today and kept because the layout is shared
// with the impostor path, which orients its quad by it.
struct TreeInstanceData
{
    glm::vec3 position = glm::vec3(0.0f);
    f32 scale = 1.0f;
    glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
    f32 rotation = 0.0f; // radians
};

// One species' worth of trees for the frame. The pass reads the instance array
// straight out of the component, so nothing is copied on the way.
//
// One command per species: same mesh, same three textures, so the whole
// species draws as two instanced calls (trunk, then leaves).
struct TreeDrawCommand
{
    MeshHandle mesh;
    const TreeInstanceData* instances = nullptr;
    u32 instanceCount = 0;

    // Submesh 0 is the bark, submesh 1 the twig cards - the order
    // AssetManager::buildTree() writes them in.
    TextureHandle bark;
    TextureHandle barkNormal;
    TextureHandle twigTexture;

    // The mesh's own height in metres. Forest scales each species to its
    // target height at build time, so the shader needs this only to turn a
    // vertex's height back into a 0..1 fraction for the wind and the AO.
    f32 modelHeight = 1.0f;

    f32 wind = 1.0f;
    f32 alphaCut = 0.4f;
    f32 bumpForce = 1.0f;
    bool castShadow = true;

    // ---- Impostors ----
    //
    // Beyond `swapDistance` a tree is a photographed quad instead of a mesh.
    // The caller splits its own instances into the two lists: those inside
    // swapDistance + swapBand go in `instances` above and draw as geometry,
    // those outside swapDistance - swapBand come here. The overlap is the band,
    // where both draw and the impostor fades in over the mesh.
    const TreeInstanceData* impostorInstances = nullptr;
    u32 impostorInstanceCount = 0;
    bool impostorsEnabled = false;
    f32 swapDistance = 120.0f;
    f32 swapBand = 12.0f;

    // Quad width over height. A tree is taller than it is wide, and a square
    // quad would leave the crown floating in empty space.
    f32 impostorWidth = 0.85f;

    // Identifies the species across frames, so the pass knows whose photographs
    // it already holds. Bump `impostorRevision` when the mesh changes and the
    // pass re-photographs it - with a fixed mesh this was ambiguous; here it is
    // known exactly.
    u32 impostorKey = 0;
    u32 impostorRevision = 0;
};

class TreeRenderQueue
{
public:
    static TreeRenderQueue& getSingleton();

    void clear();
    void submit(const TreeDrawCommand& command);
    const std::vector<TreeDrawCommand>& commands() const;

private:
    std::vector<TreeDrawCommand> mCommands;
};

TreeRenderQueue& TreeDraws();

// Instanced draw with the tree pipeline, which is what the generic mesh path
// could not give: the wind is a vertex-shader displacement that needs the
// vertex's own local position, and the leaves need a depth shader that
// alpha-tests. Both live here rather than in lit.vert/depth.frag, where they
// would cost every other mesh in the scene a branch it never takes.
RenderTechnique* createTreePass();

} // namespace Radion

#endif // RADION_TREE_RENDER_H
