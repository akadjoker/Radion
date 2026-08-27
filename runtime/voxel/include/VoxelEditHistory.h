#ifndef RADION_VOXEL_EDIT_HISTORY_H
#define RADION_VOXEL_EDIT_HISTORY_H

#include "VoxelWorld.h"

#include <vector>

namespace Radion
{
namespace Voxel
{

struct VoxelEdit
{
    VoxelCoord position;
    BlockId before = AirBlockId;
    BlockId after = AirBlockId;
};

// Keeps edits independent from terrain generation. A saved world can store
// the seed plus these edits and rebuild the same baseline before applying them.
class VoxelEditHistory
{
public:
    bool setBlock(VoxelWorld& world, VoxelCoord position, BlockId block);
    bool undo(VoxelWorld& world);
    bool redo(VoxelWorld& world);

    void clear();
    bool canUndo() const { return mCursor > 0; }
    bool canRedo() const { return mCursor < mEdits.size(); }
    usize size() const { return mEdits.size(); }
    usize cursor() const { return mCursor; }
    const std::vector<VoxelEdit>& edits() const { return mEdits; }

private:
    std::vector<VoxelEdit> mEdits;
    usize mCursor = 0;
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_EDIT_HISTORY_H