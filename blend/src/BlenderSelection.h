#ifndef RADION_BLENDER_SELECTION_H
#define RADION_BLENDER_SELECTION_H

#include "Types.h"
#include <set>
#include <vector>

namespace Radion
{

// Membership is a bitmask: isVertexSelected() runs per vertex in the draw loop, so a list scan would be quadratic.
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

    void selectVertex(u32 index);
    void deselectVertex(u32 index);
    void toggleVertex(u32 index);
    bool isVertexSelected(u32 index) const;
    const std::vector<u32>& selectedVertices() const;

    void selectFace(u32 index);
    void deselectFace(u32 index);
    void toggleFace(u32 index);
    bool isFaceSelected(u32 index) const;
    const std::vector<u32>& selectedFaces() const;

    // An edge is named by MeshTopology::edgeKey(), so the selection survives anything that leaves the mesh alone.
    void selectEdge(u64 key);
    void deselectEdge(u64 key);
    void toggleEdge(u64 key);
    bool isEdgeSelected(u64 key) const;
    // Ascending.
    const std::vector<u64>& selectedEdges() const;
    void setEdges(const std::vector<u64>& keys);

    void clearAll();
    void selectAll(u32 vertexCount, u32 faceCount);
    void invertSelection(u32 vertexCount, u32 faceCount);

    u32 selectedVertexCount() const
    {
        return mVertexCount;
    }
    u32 selectedFaceCount() const
    {
        return mFaceCount;
    }
    u32 selectedEdgeCount() const
    {
        return static_cast<u32>(mEdges.size());
    }

    // Bumped by every change; lets a viewport cheaply tell whether its GPU copy is current.
    u64 revision() const
    {
        return mRevision;
    }

    // One byte per vertex, nonzero = selected; writes exactly `count` bytes.
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

    std::set<u64> mEdges;
    mutable std::vector<u64> mEdgeList;
    mutable u64 mEdgeListRevision = 0;
};

} // namespace Radion

#endif // RADION_BLENDER_SELECTION_H
