#ifndef RADION_TREE_RENDER_H
#define RADION_TREE_RENDER_H

#include "GPU.h"
#include "Mesh.h"
#include "RenderTechnique.h"

#include <vector>

namespace Radion
{

// One planted tree, matching TreeInstance in tree.vert. `normal` is unused by the shader but kept because the impostor path shares the layout and orients its quad by it.
struct TreeInstanceData
{
    Math::vec3 position = Math::vec3(0.0f);
    f32 scale = 1.0f;
    Math::vec3 normal = Math::vec3(0.0f, 1.0f, 0.0f);
    f32 rotation = 0.0f; // radians
};

// One species' worth of trees for the frame; instance arrays are read in place. Same mesh and textures, so a species draws as two instanced calls (trunk, leaves).
struct TreeDrawCommand
{
    MeshHandle mesh;
    const TreeInstanceData* instances = nullptr;
    u32 instanceCount = 0;

    // Submesh 0 is the bark, submesh 1 the twig cards (the order AssetManager::buildTree() writes).
    TextureHandle bark;
    TextureHandle barkNormal;
    TextureHandle twigTexture;

    // The mesh's height in metres; the shader turns a vertex height back into a 0..1 fraction for wind and AO.
    f32 modelHeight = 1.0f;

    f32 wind = 1.0f;
    f32 alphaCut = 0.4f;
    f32 bumpForce = 1.0f;
    bool castShadow = true;

    // Beyond `swapDistance` a tree is a photographed quad. Instances inside swapDistance + swapBand go in `instances` as geometry; those outside swapDistance - swapBand go here.
    // In the overlap band both draw and the impostor fades in.
    const TreeInstanceData* impostorInstances = nullptr;
    u32 impostorInstanceCount = 0;
    bool impostorsEnabled = false;
    f32 swapDistance = 120.0f;
    f32 swapBand = 12.0f;

    // Quad width over height; a square quad would leave the crown floating in empty space.
    f32 impostorWidth = 0.85f;

    // Identifies the species across frames so the pass knows which photographs it holds; bump `impostorRevision` when the mesh changes to re-photograph it.
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

// Instanced draw with the tree pipeline: wind is a vertex displacement needing local position, and leaves need an alpha-testing depth shader. Kept here rather than in lit.vert/depth.frag so other meshes pay no extra branch.
RenderTechnique* createTreePass();

} // namespace Radion

#endif // RADION_TREE_RENDER_H
