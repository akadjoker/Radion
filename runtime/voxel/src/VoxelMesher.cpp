#include "VoxelMesher.h"

#include <algorithm>
#include <vector>

namespace Radion
{
namespace Voxel
{
namespace
{
struct FaceInfo
{
    VoxelCoord neighbourOffset;
    BlockFace face;
    glm::vec3 normal;
    glm::vec3 corners[4];
    VoxelCoord uAxis;
    VoxelCoord vAxis;
    // The greedy sweep's u axis is chosen to keep the quad's winding facing
    // out, and on two of the side faces that axis is vertical. Texture space
    // must not follow it there: a grass side would stand on end. Swapping the
    // two texture coordinates puts the tile's v back along world +Y without
    // touching geometry or winding.
    bool swapUv;
};

const FaceInfo Faces[] = {
    {{-1, 0, 0},
     BlockFace::NegativeX,
     {-1.0f, 0.0f, 0.0f},
    {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 0.0f}},
    {0, 0, 1},
    {0, 1, 0},
    false},
    {{1, 0, 0},
     BlockFace::PositiveX,
     {1.0f, 0.0f, 0.0f},
    {{1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 1.0f}},
    {0, 1, 0},
    {0, 0, 1},
    true},
    {{0, -1, 0},
     BlockFace::NegativeY,
     {0.0f, -1.0f, 0.0f},
    {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
    {1, 0, 0},
    {0, 0, 1},
    false},
    {{0, 1, 0},
     BlockFace::PositiveY,
     {0.0f, 1.0f, 0.0f},
    {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 0.0f}},
    {0, 0, 1},
    {1, 0, 0},
    true},
    {{0, 0, -1},
     BlockFace::NegativeZ,
     {0.0f, 0.0f, -1.0f},
    {{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
    {0, 1, 0},
    {1, 0, 0},
    true},
    {{0, 0, 1},
     BlockFace::PositiveZ,
     {0.0f, 0.0f, 1.0f},
    {{0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 1.0f}},
    {1, 0, 0},
    {0, 1, 0},
    false},
};

bool occludesFaces(const BlockDefinition& block)
{
    return block.solid && !block.transparent && block.renderType == BlockRenderType::Opaque;
}

bool faceIsVisible(BlockId blockId, const BlockDefinition& block, BlockId neighbourId,
                   const BlockDefinition& neighbour)
{
    if (neighbourId == AirBlockId)
        return true;

    if (occludesFaces(neighbour))
        return false;

    // Water/water and leaves/leaves do not need an invisible plane between
    // them, while different non-opaque materials still need their boundary.
    return block.renderType == BlockRenderType::Opaque || blockId != neighbourId;
}

// One cell of the greedy sweep. Ambient occlusion belongs in the key, not
// only in the vertex: merging two cells whose corners are occluded
// differently would stretch one cell's shading across both.
struct MaskCell
{
    BlockId block = AirBlockId;
    u8 occlusion[4] = {3, 3, 3, 3};
};

bool sameCell(const MaskCell& a, const MaskCell& b)
{
    return a.block == b.block && a.occlusion[0] == b.occlusion[0] &&
           a.occlusion[1] == b.occlusion[1] && a.occlusion[2] == b.occlusion[2] &&
           a.occlusion[3] == b.occlusion[3];
}

bool occludingBlockAt(const VoxelNeighbourhood& neighbourhood, const BlockRegistry& blocks,
                      VoxelCoord position)
{
    const BlockId id = neighbourhood.block(position);
    if (id == AirBlockId)
        return false;
    const BlockDefinition* definition = blocks.find(id);
    return definition && occludesFaces(*definition);
}

VoxelCoord offsetBy(VoxelCoord base, VoxelCoord axis, s32 amount)
{
    return {base.x + axis.x * amount, base.y + axis.y * amount, base.z + axis.z * amount};
}

// The classic three-neighbour test, per corner of a face: two sides and the
// diagonal. Two sides touching means the corner is fully closed off, whatever
// sits on the diagonal. Returns 3 for open and 0 for fully occluded.
u8 cornerOcclusion(const VoxelNeighbourhood& neighbourhood, const BlockRegistry& blocks,
                   VoxelCoord local, const VoxelCoord& normal, const VoxelCoord& uAxis,
                   const VoxelCoord& vAxis, s32 uStep, s32 vStep)
{
    const VoxelCoord front = offsetBy(local, normal, 1);
    const bool side1 = occludingBlockAt(neighbourhood, blocks, offsetBy(front, uAxis, uStep));
    const bool side2 = occludingBlockAt(neighbourhood, blocks, offsetBy(front, vAxis, vStep));
    if (side1 && side2)
        return 0;
    const bool corner = occludingBlockAt(
        neighbourhood, blocks, offsetBy(offsetBy(front, uAxis, uStep), vAxis, vStep));
    return static_cast<u8>(3 - (static_cast<s32>(side1) + static_cast<s32>(side2) +
                                static_cast<s32>(corner)));
}

// A voxel vertex needs 27 of the 48 bytes MeshAttribs would spend on it, so it
// travels packed in MeshData::colors and voxel.vert unpacks it. The layout is
// shared with that shader and neither side may move a field alone:
//   0-2   face index, into the six normals
//   3-4   ambient occlusion, 0 darkest
//   5-9   atlas column
//   10-14 atlas row
//   15-20 u, in tiles across the merged quad
//   21-26 v
u32 packVertex(u8 faceIndex, u8 occlusion, u16 atlasX, u16 atlasY, u32 u, u32 v)
{
    return (static_cast<u32>(faceIndex) & 0x7u) | ((static_cast<u32>(occlusion) & 0x3u) << 3) |
           ((static_cast<u32>(atlasX) & 0x1Fu) << 5) | ((static_cast<u32>(atlasY) & 0x1Fu) << 10) |
           ((u & 0x3Fu) << 15) | ((v & 0x3Fu) << 21);
}

u32 slotFor(BlockRenderType type)
{
    switch (type)
    {
    case BlockRenderType::Cutout:
        return VoxelMeshData::CutoutSlot;
    case BlockRenderType::Transparent:
        return VoxelMeshData::TransparentSlot;
    case BlockRenderType::Opaque:
    default:
        return VoxelMeshData::OpaqueSlot;
    }
}

glm::vec2 rotateUv(const glm::vec2& uv, s32 width, s32 height, BlockFaceRotation rotation)
{
    switch (rotation)
    {
    case BlockFaceRotation::Clockwise90:
        return {uv.y, static_cast<f32>(width) - uv.x};
    case BlockFaceRotation::HalfTurn:
        return {static_cast<f32>(width) - uv.x, static_cast<f32>(height) - uv.y};
    case BlockFaceRotation::CounterClockwise90:
        return {static_cast<f32>(height) - uv.y, uv.x};
    case BlockFaceRotation::None:
    default:
        return uv;
    }
}

void appendFace(MeshData& mesh, std::vector<u32>& indices, AABB& passBounds,
                const VoxelCoord& local, const FaceInfo& face, const BlockFaceMaterial& material,
                VoxelMesher::Settings settings, s32 width, s32 height, const MaskCell& cell)
{
    const u32 base = static_cast<u32>(mesh.positions.size());
    const glm::vec3 origin(static_cast<f32>(local.x), static_cast<f32>(local.y),
                           static_cast<f32>(local.z));
    const glm::vec3 uEdge = face.corners[1] - face.corners[0];
    const glm::vec3 vEdge = face.corners[3] - face.corners[0];
    const glm::vec3 corners[] = {
        face.corners[0],
        face.corners[0] + uEdge * static_cast<f32>(width),
        face.corners[0] + uEdge * static_cast<f32>(width) + vEdge * static_cast<f32>(height),
        face.corners[0] + vEdge * static_cast<f32>(height),
    };
    const f32 tileWidth = 1.0f / std::max<u16>(settings.atlasColumns, 1);
    const f32 tileHeight = 1.0f / std::max<u16>(settings.atlasRows, 1);
    // Texture extents follow the face's texture basis, which the swap may have
    // exchanged with the sweep's.
    const s32 texWidth = face.swapUv ? height : width;
    const s32 texHeight = face.swapUv ? width : height;
    glm::vec2 uvs[] = {
        {0.0f, 0.0f},
        {static_cast<f32>(width), 0.0f},
        {static_cast<f32>(width), static_cast<f32>(height)},
        {0.0f, static_cast<f32>(height)},
    };
    for (glm::vec2& uv : uvs)
    {
        if (face.swapUv)
            uv = {uv.y, uv.x};
        uv = rotateUv(uv, texWidth, texHeight, material.rotation);
    }
    const glm::vec2 atlasOrigin(static_cast<f32>(material.atlasX) * tileWidth,
                                static_cast<f32>(material.atlasY) * tileHeight);

    for (u32 i = 0; i < 4; ++i)
    {
        mesh.positions.push_back(origin + corners[i]);
        glm::vec2 uv = uvs[i];
        if (material.flipVertical)
            uv.y = static_cast<f32>(texHeight) - uv.y;
        // Normals, tangents and both UV sets stay out of the mesh: the packed
        // word carries what a voxel actually varies, and voxel.vert rebuilds
        // the rest. Filling them would cost 48 bytes a vertex here and again
        // on the way to the GPU, for values the shader already knows.
        mesh.colors.push_back(packVertex(static_cast<u8>(face.face), cell.occlusion[i],
                                         material.atlasX, material.atlasY,
                                         static_cast<u32>(uv.x), static_cast<u32>(uv.y)));
        mesh.bounds.expand(mesh.positions.back());
        // Each pass keeps its own box: they share one vertex stream now, and
        // the render list culls a submesh by the bounds it was given. Handing
        // all three the whole chunk submits water and leaves wherever any part
        // of the chunk is visible.
        passBounds.expand(mesh.positions.back());
    }
    // The diagonal follows the darker pair of corners. Split the other way and
    // the quad's two triangles interpolate opposite gradients, which shows up
    // as a crease running across an otherwise flat wall.
    const s32 diagonal = static_cast<s32>(cell.occlusion[0]) + cell.occlusion[2] -
                         static_cast<s32>(cell.occlusion[1]) - cell.occlusion[3];
    if (diagonal > 0)
        indices.insert(indices.end(), {base + 1, base + 2, base + 3, base + 1, base + 3, base});
    else
        indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
}

VoxelCoord localFor(const FaceInfo& face, s32 plane, s32 u, s32 v)
{
    VoxelCoord local;
    const VoxelCoord normal = face.neighbourOffset;
    if (normal.x != 0)
        local.x = plane;
    else if (normal.y != 0)
        local.y = plane;
    else
        local.z = plane;
    local.x += face.uAxis.x * u + face.vAxis.x * v;
    local.y += face.uAxis.y * u + face.vAxis.y * v;
    local.z += face.uAxis.z * u + face.vAxis.z * v;
    return local;
}

} // namespace

void VoxelMeshData::clear()
{
    mesh.clear();
}

bool VoxelMeshData::hasSlot(u32 materialSlot) const
{
    for (const SubMesh& submesh : mesh.submeshes)
    {
        if (submesh.materialSlot == materialSlot)
            return true;
    }
    return false;
}

VoxelMeshData VoxelMesher::buildChunk(const VoxelWorld& world, const VoxelChunk& chunk,
                                      const BlockRegistry& blocks, Settings settings)
{
    VoxelNeighbourhood neighbourhood;
    neighbourhood.gather(world, chunk.coordinate());
    return buildChunk(neighbourhood, blocks, settings);
}

VoxelMeshData VoxelMesher::buildChunk(const VoxelNeighbourhood& neighbourhood,
                                      const BlockRegistry& blocks, Settings settings)
{
    VoxelMeshData result;
    // One bucket per pass while the sweep runs, concatenated at the end. The
    // vertices are shared, so nothing is copied twice.
    std::vector<u32> passIndices[3];
    AABB passBounds[3];
    MaskCell mask[static_cast<usize>(VoxelChunk::Size) * VoxelChunk::Size];
    for (const FaceInfo& face : Faces)
    {
        for (s32 plane = 0; plane < VoxelChunk::Size; ++plane)
        {
            std::fill(std::begin(mask), std::end(mask), MaskCell());
            for (s32 v = 0; v < VoxelChunk::Size; ++v)
            {
                for (s32 u = 0; u < VoxelChunk::Size; ++u)
                {
                    const VoxelCoord local = localFor(face, plane, u, v);
                    const BlockId id = neighbourhood.block(local);
                    if (id == AirBlockId)
                        continue;
                    const BlockDefinition* definition = blocks.find(id);
                    if (!definition)
                        continue;
                    const BlockId neighbourId = neighbourhood.block(local.x + face.neighbourOffset.x,
                                                                    local.y + face.neighbourOffset.y,
                                                                    local.z + face.neighbourOffset.z);
                    const BlockDefinition* neighbour = blocks.find(neighbourId);
                    if (!neighbour)
                        neighbour = &blocks.air();
                    if (!faceIsVisible(id, *definition, neighbourId, *neighbour))
                        continue;

                    MaskCell& cell = mask[static_cast<usize>(v) * VoxelChunk::Size + u];
                    cell.block = id;
                    if (!settings.ambientOcclusion)
                        continue;
                    // Corners in the same order appendFace emits them: the
                    // sweep's (u,v) origin, then along u, then the far corner,
                    // then along v.
                    cell.occlusion[0] = cornerOcclusion(neighbourhood, blocks, local,
                                                        face.neighbourOffset, face.uAxis,
                                                        face.vAxis, -1, -1);
                    cell.occlusion[1] = cornerOcclusion(neighbourhood, blocks, local,
                                                        face.neighbourOffset, face.uAxis,
                                                        face.vAxis, 1, -1);
                    cell.occlusion[2] = cornerOcclusion(neighbourhood, blocks, local,
                                                        face.neighbourOffset, face.uAxis,
                                                        face.vAxis, 1, 1);
                    cell.occlusion[3] = cornerOcclusion(neighbourhood, blocks, local,
                                                        face.neighbourOffset, face.uAxis,
                                                        face.vAxis, -1, 1);
                }
            }

            for (s32 v = 0; v < VoxelChunk::Size; ++v)
            {
                for (s32 u = 0; u < VoxelChunk::Size;)
                {
                    const usize index = static_cast<usize>(v) * VoxelChunk::Size + u;
                    const MaskCell cell = mask[index];
                    if (cell.block == AirBlockId)
                    {
                        ++u;
                        continue;
                    }

                    s32 width = 1;
                    while (u + width < VoxelChunk::Size &&
                           sameCell(mask[index + static_cast<usize>(width)], cell))
                        ++width;
                    s32 height = 1;
                    while (v + height < VoxelChunk::Size)
                    {
                        bool matches = true;
                        for (s32 column = 0; column < width; ++column)
                            if (!sameCell(
                                    mask[static_cast<usize>(v + height) * VoxelChunk::Size + u +
                                         column],
                                    cell))
                            {
                                matches = false;
                                break;
                            }
                        if (!matches)
                            break;
                        ++height;
                    }

                    const BlockDefinition* definition = blocks.find(cell.block);
                    const u32 slot = slotFor(definition->renderType);
                    appendFace(result.mesh, passIndices[slot], passBounds[slot],
                               localFor(face, plane, u, v), face,
                               definition->faces[static_cast<usize>(face.face)], settings, width,
                               height, cell);
                    for (s32 row = 0; row < height; ++row)
                        for (s32 column = 0; column < width; ++column)
                            mask[static_cast<usize>(v + row) * VoxelChunk::Size + u + column] =
                                MaskCell();
                    u += width;
                }
            }
        }
    }

    for (u32 slot = 0; slot < 3; ++slot)
    {
        if (passIndices[slot].empty())
            continue;
        SubMesh submesh;
        submesh.indexOffset = static_cast<u32>(result.mesh.indices.size());
        submesh.indexCount = static_cast<u32>(passIndices[slot].size());
        submesh.materialSlot = slot;
        submesh.bounds = passBounds[slot];
        result.mesh.submeshes.push_back(submesh);
        result.mesh.indices.insert(result.mesh.indices.end(), passIndices[slot].begin(),
                                   passIndices[slot].end());
    }
    return result;
}

} // namespace Voxel
} // namespace Radion
