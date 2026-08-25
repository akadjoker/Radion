#ifndef RADION_EDITOR_SELECTION_H
#define RADION_EDITOR_SELECTION_H

#include "Types.h"

#include <vector>

namespace Radion
{
class Scene;
class GameObject;

 
class EditorSelection
{
public:
    void select(u64 objectId);
    void clear();
    bool hasSelection() const;
    u64 selectedId() const;

    // Multi-selection, for operations that act on a set rather than on one
    // object - deleting a batch of markers, say. The primary selection
    // (selectedId()) is always the last one added and stays what the
    // inspector and the gizmo drive, so nothing single-object had to learn
    // about sets.
    void toggle(u64 objectId);
    bool isSelected(u64 objectId) const;
    const std::vector<u64>& selectedIds() const;
    usize count() const;

    GameObject* resolve(Scene& scene) const;

private:
    u64 mSelectedId = 0;
    std::vector<u64> mSelectedIds;
};

} // namespace Radion

#endif // RADION_EDITOR_SELECTION_H
