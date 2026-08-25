#include "PCH.h"

#include "EditorSelection.h"

#include "Scene.h"

#include <algorithm>

namespace Radion
{

void EditorSelection::select(u64 objectId)
{
    mSelectedId = objectId;
    mSelectedIds.clear();
    if (objectId != 0)
        mSelectedIds.push_back(objectId);
}

void EditorSelection::toggle(u64 objectId)
{
    if (objectId == 0)
        return;
    const auto found = std::find(mSelectedIds.begin(), mSelectedIds.end(), objectId);
    if (found != mSelectedIds.end())
    {
        mSelectedIds.erase(found);
        // Dropping the primary hands that role to whatever is still selected,
        // so the inspector never ends up pointed at nothing while a set is
        // still highlighted.
        if (mSelectedId == objectId)
            mSelectedId = mSelectedIds.empty() ? 0 : mSelectedIds.back();
        return;
    }
    mSelectedIds.push_back(objectId);
    mSelectedId = objectId;
}

bool EditorSelection::isSelected(u64 objectId) const
{
    return objectId != 0 &&
           std::find(mSelectedIds.begin(), mSelectedIds.end(), objectId) != mSelectedIds.end();
}

const std::vector<u64>& EditorSelection::selectedIds() const
{
    return mSelectedIds;
}

usize EditorSelection::count() const
{
    return mSelectedIds.size();
}

void EditorSelection::clear()
{
    mSelectedId = 0;
    mSelectedIds.clear();
}

bool EditorSelection::hasSelection() const
{
    return mSelectedId != 0;
}

u64 EditorSelection::selectedId() const
{
    return mSelectedId;
}

GameObject* EditorSelection::resolve(Scene& scene) const
{
    return mSelectedId != 0 ? scene.findGameObject(mSelectedId) : nullptr;
}

} // namespace Radion
