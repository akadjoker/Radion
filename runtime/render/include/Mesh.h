#ifndef RADION_MESH_H
#define RADION_MESH_H

#include "Containers.h"
#include "GPU.h"
#include "Material.h"
#include "Math.h"
#include "Types.h"

#include <string>
#include <vector>

namespace Radion

{

using MeshHandle = Handle<struct MeshTag>;

// A submesh carries its own box: half of a building can sit outside the
// frustum, and without it the choice is to draw all of it or none of it.
struct SubMesh
{
    u32 indexOffset = 0;
    u32 indexCount = 0;
    u32 materialSlot = 0;
    // Page of the baked lightmap atlas used by this submesh. Kept in the
    // reserved submesh field of .rmesh; zero is the ordinary single-atlas case.
    u32 lightmapPage = 0;
    AABB bounds;
    bool visible = true;
};

// Physics and picking read this, never the vertex buffer: once uploaded the
// GPU copy cannot be read back without stalling the frame. Positions only,
// because collision has no use for normals or uvs.
struct CollisionMesh
{
    std::vector<glm::vec3> positions;
    std::vector<u32> indices;
    AABB bounds;
};

// Positions live in their own stream because the shadow and depth passes read
// nothing else. Interleaved with the rest, a cascade would pull 52 bytes per
// vertex to use 12, three or four times a frame.
enum MeshStream : u8
{
    StreamPosition = 0,
    StreamAttribs = 1,
    StreamSkin = 2,

    MeshStreamCount = 3
};

// Everything the colour pass needs and the depth passes do not.
struct MeshAttribs
{
    glm::vec3 normal;
    glm::vec4 tangent;
    glm::vec2 uv;
    u32 color;
    glm::vec2 uv2;
};

struct MeshSkinVertex
{
    u8 joints[4] = {0, 0, 0, 0};
    glm::vec4 weights = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
};

// The mesh while it is still being built: loaded, edited, measured. Nothing
// here talks to the GPU; MeshManager::upload is the one way across. Attributes
// live in separate arrays so an operation that only moves positions never
// walks over normals and uvs it will not read.
struct MeshData
{
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec4> tangents;
    std::vector<glm::vec2> uvs;
    // A second UV set, parallel to `uvs` - a lightmap's own unwrap. Empty
    // for every importer/builder that has no such data; upload() fills
    // MeshAttribs::uv2 with (0,0) per vertex in that case, same as it
    // already does for a short/absent `uvs`.
    std::vector<glm::vec2> uvs2;
    std::vector<u32> colors;
    std::vector<MeshSkinVertex> skin;
    std::vector<u32> indices;

    std::vector<SubMesh> submeshes;
    std::vector<Material> materials;
    // Import-time albedo paths, parallel to materials. createMesh() resolves
    // them through AssetManager and does not retain them in the GPU Mesh.
    std::vector<std::string> materialTextureFiles;

    // The same, for normal maps (an OBJ's map_bump / bump). Kept as its own
    // array rather than folded into the one above: a material can have either,
    // both or neither, and index i has to keep meaning material i.
    std::vector<std::string> materialNormalFiles;

    // Same convention, the rest of a PBR material's channels - a glTF's own
    // pbrMetallicRoughness.metallicRoughnessTexture, occlusionTexture and
    // emissiveTexture. Empty for every importer that has no such data (FBX's
    // classic Diffuse/Normal/Specular has no metallic-roughness or occlusion
    // slot to read one from).
    std::vector<std::string> materialSurfaceFiles;  // roughness/metalness/AO packed texture
    std::vector<std::string> materialEmissiveFiles;
    std::vector<std::string> materialHeightFiles; // ambient occlusion, in this engine's Height slot

    AABB bounds;

    usize vertexCount() const;
    usize triangleCount() const;
    // Bytes the vertex, index and submesh arrays hold. For anything keeping
    // copies of a mesh around - an undo stack, most obviously - where the
    // count of copies says nothing about what they cost: the same twenty
    // steps are a rounding error on a crate and hundreds of megabytes on a
    // scanned model. Ignores the material name strings, which no amount of
    // them adds up next to the geometry.
    usize memoryBytes() const;
    void clear();

    // Grows every attribute array that is already in use, so the arrays never
    // drift out of step with positions.
    void resizeVertices(usize count);
};

struct Mesh
{
    BufferHandle positionBuffer;
    BufferHandle attribBuffer;
    BufferHandle skinBuffer;
    BufferHandle indexBuffer;
    IndexType indexType = IndexType::U32;

    // False for a mesh built over a shared index buffer it does not own -
    // Landscape's chunks all point at the one buffer LandscapeIndices builds,
    // and destroying one chunk must not take the other three hundred down
    // with it. True (the default) is every other mesh in the engine, where
    // AssetManager::release() destroying everything is exactly right.
    bool ownsIndexBuffer = true;

    // Positions only, for shadow and depth. The full layout adds stream 1.
    VertexLayout depthLayout;
    VertexLayout colorLayout;

    std::vector<SubMesh> submeshes;
    std::vector<Material> materials;

    AABB bounds;
    u32 vertexCount = 0;
    u32 indexCount = 0;

    bool isSkinned() const
    {
        return skinBuffer.valid();
    }
};

} // namespace Radion

#endif // RADION_MESH_H
