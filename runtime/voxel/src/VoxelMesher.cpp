#include "VoxelMesher.h"

#include <algorithm>

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
};

const FaceInfo Faces[] = {
    {{-1, 0, 0},
     BlockFace::NegativeX,
     {-1.0f, 0.0f, 0.0f},
     {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 0.0f}}},
    {{1, 0, 0},
     BlockFace::PositiveX,
     {1.0f, 0.0f, 0.0f},
     {{1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 1.0f}}},
    {{0, -1, 0},
     BlockFace::NegativeY,
     {0.0f, -1.0f, 0.0f},
     {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}}},
    {{0, 1, 0},
     BlockFace::PositiveY,
     {0.0f, 1.0f, 0.0f},
     {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 0.0f}}},
    {{0, 0, -1},
     BlockFace::NegativeZ,
     {0.0f, 0.0f, -1.0f},
     {{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}}},
    {{0, 0, 1},
     BlockFace::PositiveZ,
     {0.0f, 0.0f, 1.0f},
     {{0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 1.0f}}},
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

void appendFace(MeshData& mesh, const VoxelCoord& local, const FaceInfo& face,
                const BlockFaceMaterial& material, VoxelMesher::Settings settings)
{
    const u32 base = static_cast<u32>(mesh.positions.size());
    const glm::vec3 origin(static_cast<f32>(local.x), static_cast<f32>(local.y),
                           static_cast<f32>(local.z));
    const f32 tileWidth = 1.0f / std::max<u16>(settings.atlasColumns, 1);
    const f32 tileHeight = 1.0f / std::max<u16>(settings.atlasRows, 1);
    const f32 inset = settings.atlasTilePixels == 0
                          ? 0.0f
                          : 0.5f / (static_cast<f32>(settings.atlasColumns) *
                                    static_cast<f32>(settings.atlasTilePixels));
    const f32 u0 = static_cast<f32>(material.atlasX) * tileWidth + inset;
    const f32 v0 = static_cast<f32>(material.atlasY) * tileHeight + inset;
    const f32 u1 = u0 + tileWidth - inset * 2.0f;
    const f32 v1 = v0 + tileHeight - inset * 2.0f;
    const glm::vec2 uvs[] = {{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}};

    for (u32 i = 0; i < 4; ++i)
    {
        mesh.positions.push_back(origin + face.corners[i]);
        mesh.normals.push_back(face.normal);
        mesh.uvs.push_back(uvs[i]);
        mesh.bounds.expand(mesh.positions.back());
    }
    mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
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
    for (s32 z = 0; z < VoxelChunk::Size; ++z)
    {
        for (s32 y = 0; y < VoxelChunk::Size; ++y)
        {
            for (s32 x = 0; x < VoxelChunk::Size; ++x)
            {
                const VoxelCoord local{x, y, z};
                const BlockId id = chunk.block(local);
                if (id == AirBlockId)
                    continue;

                const BlockDefinition* definition = blocks.find(id);
                if (!definition)
                    continue;

                const VoxelCoord worldPosition = VoxelWorld::worldFor(chunkCoordinate, local);
                for (const FaceInfo& face : Faces)
                {
                    const VoxelCoord neighbourPosition = {worldPosition.x + face.neighbourOffset.x,
                                                          worldPosition.y + face.neighbourOffset.y,
                                                          worldPosition.z + face.neighbourOffset.z};
                    const BlockId neighbourId = world.block(neighbourPosition);
                    const BlockDefinition* neighbour = blocks.find(neighbourId);
                    if (!neighbour)
                        neighbour = &blocks.air();
                    if (!faceIsVisible(id, *definition, neighbourId, *neighbour))
                        continue;

                    appendFace(meshFor(result, definition->renderType), local, face,
                               definition->faces[static_cast<usize>(face.face)], settings);
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
