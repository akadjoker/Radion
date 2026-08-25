#ifndef RADION_BLENDER_SELECTION_H
#define RADION_BLENDER_SELECTION_H

#include "Types.h"
#include <vector>

namespace Radion
{

// Selection state for blender — tracks which vertices and faces are selected.
//
// Membership is a bitmask, not a list to search: isVertexSelected() runs in
// the viewport's per-vertex draw loop and again inside every selectVertex(),
// so a linear scan makes both the drawing and a box select quadratic in the
// number of selected items. The index list callers iterate is rebuilt from
// the bits only when one of them asks and something has changed since.
class BlenderSelection
{
public:
    enum class SelectionMode : u8
    {
        Vertex,
        Face,
        Edge
    };

    BlenderSelection();
    ~BlenderSelection();

    SelectionMode mode() const
    {
        return mMode;
    }
    void setMode(SelectionMode mode)
    {
        mMode = mode;
    }

    // Vertex selection
    void selectVertex(u32 index);
    void deselectVertex(u32 index);
    void toggleVertex(u32 index);
    bool isVertexSelected(u32 index) const;
    const std::vector<u32>& selectedVertices() const;

    // Face selection
    void selectFace(u32 index);
    void deselectFace(u32 index);
    void toggleFace(u32 index);
    bool isFaceSelected(u32 index) const;
    const std::vector<u32>& selectedFaces() const;

    // Clear selection
    void clearAll();
    void selectAll(u32 vertexCount, u32 faceCount);
    void invertSelection(u32 vertexCount, u32 faceCount);

    // Query
    u32 selectedVertexCount() const
    {
        return mVertexCount;
    }
    u32 selectedFaceCount() const
    {
        return mFaceCount;
    }

    // Bumped by every change. Lets a viewport tell in constant time whether
    // the GPU-side copy it uploaded is still current, instead of comparing
    // the selection itself.
    u64 revision() const
    {
        return mRevision;
    }

    // One byte per vertex, nonzero where selected, for handing the selection
    // to the renderer as a vertex stream. Writes exactly `count` bytes.
    void fillVertexFlags(u8* out, u32 count) const;

private:
    static bool testBit(const std::vector<u64>& bits, u32 index);
    static bool setBit(std::vector<u64>& bits, u32 index);
    static bool clearBit(std::vector<u64>& bits, u32 index);
    static void rebuild(const std::vector<u64>& bits, std::vector<u32>& list);

    SelectionMode mMode = SelectionMode::Vertex;

    std::vector<u64> mVertexBits;
    std::vector<u64> mFaceBits;
    u32 mVertexCount = 0;
    u32 mFaceCount = 0;
    u64 mRevision = 0;

    mutable std::vector<u32> mVertexList;
    mutable std::vector<u32> mFaceList;
    mutable u64 mVertexListRevision = 0;
    mutable u64 mFaceListRevision = 0;
};

} // namespace Radion

#endif // RADION_BLENDER_SELECTION_H
