#include "VoxelEditHistory.h"

namespace Radion
{
namespace Voxel
{

bool VoxelEditHistory::setBlock(VoxelWorld& world, VoxelCoord position, BlockId block)
{
    const BlockId before = world.block(position);
    if (before == block || !world.setBlock(position, block))
        return false;

    if (mCursor < mEdits.size())
        mEdits.resize(mCursor);
    mEdits.push_back({position, before, block});
    mCursor = mEdits.size();
    return true;
}

bool VoxelEditHistory::undo(VoxelWorld& world)
{
    if (!canUndo())
        return false;

    const VoxelEdit& edit = mEdits[mCursor - 1];
    if (world.block(edit.position) != edit.before && !world.setBlock(edit.position, edit.before))
        return false;
    --mCursor;
    return true;
}

bool VoxelEditHistory::redo(VoxelWorld& world)
{
    if (!canRedo())
        return false;

    const VoxelEdit& edit = mEdits[mCursor];
    if (world.block(edit.position) != edit.after && !world.setBlock(edit.position, edit.after))
        return false;
    ++mCursor;
    return true;
}

void VoxelEditHistory::clear()
{
    mEdits.clear();
    mCursor = 0;
}

} // namespace Voxel
} // namespace Radion