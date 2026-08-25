#include "VoxelBlock.h"

#include <limits>
#include <utility>

namespace Radion
{
namespace Voxel
{

BlockRegistry::BlockRegistry()
{
    BlockDefinition air;
    air.name = "air";
    air.solid = false;
    air.transparent = true;
    air.blocksLight = false;
    air.renderType = BlockRenderType::Transparent;
    mDefinitions.push_back(air);
    mIdsByName.emplace(air.name, AirBlockId);
}

BlockId BlockRegistry::registerBlock(BlockDefinition definition)
{
    if (definition.name.empty() || mIdsByName.find(definition.name) != mIdsByName.end()
        || mDefinitions.size() >= static_cast<usize>(std::numeric_limits<BlockId>::max()))
    {
        return InvalidBlockId;
    }

    const BlockId id = static_cast<BlockId>(mDefinitions.size());
    mIdsByName.emplace(definition.name, id);
    mDefinitions.push_back(std::move(definition));
    return id;
}

const BlockDefinition* BlockRegistry::find(BlockId id) const
{
    return id < mDefinitions.size() ? &mDefinitions[id] : nullptr;
}

BlockDefinition* BlockRegistry::find(BlockId id)
{
    return id < mDefinitions.size() ? &mDefinitions[id] : nullptr;
}

BlockId BlockRegistry::findId(const std::string& name) const
{
    const auto it = mIdsByName.find(name);
    return it == mIdsByName.end() ? InvalidBlockId : it->second;
}

const BlockDefinition& BlockRegistry::air() const
{
    return mDefinitions[AirBlockId];
}

} // namespace Voxel
} // namespace Radion
