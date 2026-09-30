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

// Each submesh has its own box so half a building outside the frustum can be culled.
struct SubMesh
{
    u32 indexOffset = 0;
    u32 indexCount = 0;
    u32 materialSlot = 0;
    // Baked lightmap atlas page, stored in the reserved submesh field of .rmesh; zero = single atlas.
    u32 lightmapPage = 0;
    AABB bounds;
    bool visible = true;
};

// Physics and picking read this, never the vertex buffer (GPU readback stalls). Positions only.
struct CollisionMesh
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    AABB bounds;
};

// Positions in their own stream because shadow/depth passes read nothing else; interleaved, a cascade would pull 52 bytes per vertex to use 12.
enum MeshStream : u8
{
    StreamPosition = 0,
    StreamAttribs = 1,
    StreamSkin = 2,

    MeshStreamCount = 3
};

struct MeshAttribs
{
    Math::vec3 normal;
    Math::vec4 tangent;
    Math::vec2 uv;
    u32 color;
    Math::vec2 uv2;
};

struct MeshSkinVertex
{
    u8 joints[4] = {0, 0, 0, 0};
    Math::vec4 weights = Math::vec4(1.0f, 0.0f, 0.0f, 0.0f);
};

// The mesh while being built; nothing here touches the GPU (upload() is the way across).
// Attributes are separate arrays so position-only operations skip normals and uvs.
struct MeshData
{
    std::vector<Math::vec3> positions;
    std::vector<Math::vec3> normals;
    std::vector<Math::vec4> tangents;
    std::vector<Math::vec2> uvs;
    // Second UV set parallel to `uvs` (lightmap unwrap); empty when absent, upload() then fills MeshAttribs::uv2 with (0,0).
    std::vector<Math::vec2> uvs2;
    std::vector<u32> colors;
    std::vector<MeshSkinVertex> skin;
    std::vector<u32> indices;

    std::vector<SubMesh> submeshes;
    std::vector<Material> materials;
    // Import-time albedo paths, parallel to materials; createMesh() resolves them and does not retain them.
    std::vector<std::string> materialTextureFiles;

    // Normal map paths; a separate array since a material may have either, both or neither, and index i must mean material i.
    std::vector<std::string> materialNormalFiles;

    // Same convention for the other PBR channels (glTF metallicRoughness/occlusion/emissive); empty when the importer has none.
    std::vector<std::string> materialSurfaceFiles;  // roughness/metalness/AO packed texture
    std::vector<std::string> materialEmissiveFiles;
    std::vector<std::string> materialHeightFiles; // ambient occlusion, in this engine's Height slot

    AABB bounds;

    usize vertexCount() const;
    usize triangleCount() const;
    // Bytes held by vertex, index and submesh arrays (ignores material names); for things keeping mesh copies, such as an undo stack.
    usize memoryBytes() const;
    void clear();

    // Grows every in-use attribute array, keeping them in step with positions.
    void resizeVertices(usize count);
};

struct Mesh
{
    BufferHandle positionBuffer;
    BufferHandle attribBuffer;
    BufferHandle skinBuffer;
    BufferHandle indexBuffer;
    IndexType indexType = IndexType::U32;

    // False for a mesh over a shared index buffer it does not own (Landscape chunks), so destroying one does not free the rest.
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
