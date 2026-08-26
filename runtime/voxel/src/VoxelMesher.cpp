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
    Math::vec3 normal;
    Math::vec3 corners[4];
    VoxelCoord uAxis;
    VoxelCoord vAxis;
};

const FaceInfo Faces[] = {
    {{-1, 0, 0},
     BlockFace::NegativeX,
     {-1.0f, 0.0f, 0.0f},
    {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 0.0f}},
    {0, 0, 1},
    {0, 1, 0}},
    {{1, 0, 0},
     BlockFace::PositiveX,
     {1.0f, 0.0f, 0.0f},
    {{1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 1.0f}},
    {0, 1, 0},
    {0, 0, 1}},
    {{0, -1, 0},
     BlockFace::NegativeY,
     {0.0f, -1.0f, 0.0f},
    {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
    {1, 0, 0},
    {0, 0, 1}},
    {{0, 1, 0},
     BlockFace::PositiveY,
     {0.0f, 1.0f, 0.0f},
    {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 0.0f}},
    {0, 0, 1},
    {1, 0, 0}},
    {{0, 0, -1},
     BlockFace::NegativeZ,
     {0.0f, 0.0f, -1.0f},
    {{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
    {0, 1, 0},
    {1, 0, 0}},
    {{0, 0, 1},
     BlockFace::PositiveZ,
     {0.0f, 0.0f, 1.0f},
    {{0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 1.0f}},
    {1, 0, 0},
    {0, 1, 0}},
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

MeshData& meshFor(VoxelMeshData& result, BlockRenderType type)
{
    switch (type)
    {
    case BlockRenderType::Cutout:
        return result.cutout;
    case BlockRenderType::Transparent:
        return result.transparent;
    case BlockRenderType::Opaque:
    default:
        return result.opaque;
    }
}

Math::vec2 rotateUv(const Math::vec2& uv, s32 width, s32 height, BlockFaceRotation rotation)
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

void appendFace(MeshData& mesh, const VoxelCoord& local, const FaceInfo& face,
                const BlockFaceMaterial& material, VoxelMesher::Settings settings, s32 width,
                s32 height)
{
    const u32 base = static_cast<u32>(mesh.positions.size());
    const Math::vec3 origin(static_cast<f32>(local.x), static_cast<f32>(local.y),
                           static_cast<f32>(local.z));
    const Math::vec3 uEdge = face.corners[1] - face.corners[0];
    const Math::vec3 vEdge = face.corners[3] - face.corners[0];
    const Math::vec3 corners[] = {
        face.corners[0],
        face.corners[0] + uEdge * static_cast<f32>(width),
        face.corners[0] + uEdge * static_cast<f32>(width) + vEdge * static_cast<f32>(height),
        face.corners[0] + vEdge * static_cast<f32>(height),
    };
    const f32 tileWidth = 1.0f / std::max<u16>(settings.atlasColumns, 1);
    const f32 tileHeight = 1.0f / std::max<u16>(settings.atlasRows, 1);
    const Math::vec2 uvs[] = {
        rotateUv({0.0f, 0.0f}, width, height, material.rotation),
        rotateUv({static_cast<f32>(width), 0.0f}, width, height, material.rotation),
        rotateUv({static_cast<f32>(width), static_cast<f32>(height)}, width, height,
                 material.rotation),
        rotateUv({0.0f, static_cast<f32>(height)}, width, height, material.rotation),
    };
    const Math::vec2 atlasOrigin(static_cast<f32>(material.atlasX) * tileWidth,
                                static_cast<f32>(material.atlasY) * tileHeight);

    for (u32 i = 0; i < 4; ++i)
    {
        mesh.positions.push_back(origin + corners[i]);
        mesh.normals.push_back(face.normal);
        Math::vec2 uv = uvs[i];
        if (material.flipVertical)
            uv.y = static_cast<f32>(height) - uv.y;
        mesh.uvs.push_back(uv);
        mesh.uvs2.push_back(atlasOrigin);
        mesh.bounds.expand(mesh.positions.back());
    }
    mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
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

void finishMesh(MeshData& mesh)
{
    if (!mesh.indices.empty())
        mesh.submeshes.push_back({0, static_cast<u32>(mesh.indices.size()), 0, 0, mesh.bounds});
}
} // namespace

void VoxelMeshData::clear()
{
    opaque.clear();
    cutout.clear();
    transparent.clear();
}

VoxelMeshData VoxelMesher::buildChunk(const VoxelWorld& world, const VoxelChunk& chunk,
                                      const BlockRegistry& blocks, Settings settings)
{
    VoxelMeshData result;
    const ChunkCoord chunkCoordinate = chunk.coordinate();
    std::vector<BlockId> mask(static_cast<usize>(VoxelChunk::Size) * VoxelChunk::Size);
    for (const FaceInfo& face : Faces)
    {
        for (s32 plane = 0; plane < VoxelChunk::Size; ++plane)
        {
            std::fill(mask.begin(), mask.end(), AirBlockId);
            for (s32 v = 0; v < VoxelChunk::Size; ++v)
            {
                for (s32 u = 0; u < VoxelChunk::Size; ++u)
                {
                    const VoxelCoord local = localFor(face, plane, u, v);
                    const BlockId id = chunk.block(local);
                    const BlockDefinition* definition = blocks.find(id);
                    if (!definition)
                        continue;
                    const VoxelCoord worldPosition = VoxelWorld::worldFor(chunkCoordinate, local);
                    const VoxelCoord neighbourPosition = {worldPosition.x + face.neighbourOffset.x,
                                                          worldPosition.y + face.neighbourOffset.y,
                                                          worldPosition.z + face.neighbourOffset.z};
                    const BlockId neighbourId = world.block(neighbourPosition);
                    const BlockDefinition* neighbour = blocks.find(neighbourId);
                    if (!neighbour)
                        neighbour = &blocks.air();
                    if (!faceIsVisible(id, *definition, neighbourId, *neighbour))
                        continue;

                    mask[static_cast<usize>(v) * VoxelChunk::Size + u] = id;
                }
            }

            for (s32 v = 0; v < VoxelChunk::Size; ++v)
            {
                for (s32 u = 0; u < VoxelChunk::Size;)
                {
                    const usize index = static_cast<usize>(v) * VoxelChunk::Size + u;
                    const BlockId id = mask[index];
                    if (id == AirBlockId)
                    {
                        ++u;
                        continue;
                    }

                    s32 width = 1;
                    while (u + width < VoxelChunk::Size &&
                           mask[index + static_cast<usize>(width)] == id)
                        ++width;
                    s32 height = 1;
                    while (v + height < VoxelChunk::Size)
                    {
                        bool matches = true;
                        for (s32 column = 0; column < width; ++column)
                            if (mask[static_cast<usize>(v + height) * VoxelChunk::Size + u + column] != id)
                            {
                                matches = false;
                                break;
                            }
                        if (!matches)
                            break;
                        ++height;
                    }

                    const BlockDefinition* definition = blocks.find(id);
                    appendFace(meshFor(result, definition->renderType), localFor(face, plane, u, v),
                               face, definition->faces[static_cast<usize>(face.face)], settings,
                               width, height);
                    for (s32 row = 0; row < height; ++row)
                        for (s32 column = 0; column < width; ++column)
                            mask[static_cast<usize>(v + row) * VoxelChunk::Size + u + column] = AirBlockId;
                    u += width;
                }
            }
        }
    }

    finishMesh(result.opaque);
    finishMesh(result.cutout);
    finishMesh(result.transparent);
    return result;
}

} // namespace Voxel
} // namespace Radion
