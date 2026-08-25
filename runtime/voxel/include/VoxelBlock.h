#ifndef RADION_VOXEL_BLOCK_H
#define RADION_VOXEL_BLOCK_H

#include "Types.h"

#include <array>
#include <glm/glm.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace Radion
{
namespace Voxel
{

using BlockId = u16;

constexpr BlockId AirBlockId = 0;
constexpr BlockId InvalidBlockId = static_cast<BlockId>(~BlockId{0});

enum class BlockRenderType : u8
{
    Opaque,
    Cutout,
    Transparent
};

enum class BlockFace : u8
{
    NegativeX = 0,
    PositiveX,
    NegativeY,
    PositiveY,
    NegativeZ,
    PositiveZ,
    Count
};

// A tile in a texture atlas.  The renderer will turn these coordinates into
// UVs; chunks and world generation never need to know about texture assets.
// `color` is the face's tint, written straight into the meshed vertices so a
// block shows distinct faces before an atlas texture exists.
struct BlockFaceMaterial
{
    u16 atlasX = 0;
    u16 atlasY = 0;
    glm::vec4 color = glm::vec4(1.0f);
};

struct BlockDefinition
{
    std::string name;
    bool solid = true;
    bool transparent = false;
    bool blocksLight = true;
    u8 emittedLight = 0;
    BlockRenderType renderType = BlockRenderType::Opaque;
    std::array<BlockFaceMaterial, static_cast<usize>(BlockFace::Count)> faces = {};
};

// Stable block IDs belong to the registry, rather than the mesher or world
// generator.  This lets a project extend its block set without teaching each
// subsystem about individual materials.
class BlockRegistry
{
public:
    BlockRegistry();

    BlockId registerBlock(BlockDefinition definition);

    const BlockDefinition* find(BlockId id) const;
    BlockDefinition* find(BlockId id);
    BlockId findId(const std::string& name) const;

    usize size() const
    {
        return mDefinitions.size();
    }
    const BlockDefinition& air() const;

private:
    std::vector<BlockDefinition> mDefinitions;
    std::unordered_map<std::string, BlockId> mIdsByName;
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_BLOCK_H
