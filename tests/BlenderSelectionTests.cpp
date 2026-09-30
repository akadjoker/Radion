#include "PCH.h"

#include "BlenderSelection.h"

#include <cstdio>
#include <vector>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "BlenderSelectionTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

void testSelectAndDeselect()
{
    BlenderSelection selection;
    CHECK(selection.selectedVertexCount() == 0);
    CHECK(selection.selectedVertices().empty());

    selection.selectVertex(5);
    selection.selectVertex(200);
    selection.selectVertex(63);
    selection.selectVertex(64);

    CHECK(selection.selectedVertexCount() == 4);
    CHECK(selection.isVertexSelected(5));
    CHECK(selection.isVertexSelected(63));
    CHECK(selection.isVertexSelected(64));
    CHECK(selection.isVertexSelected(200));
    CHECK(!selection.isVertexSelected(6));
    CHECK(!selection.isVertexSelected(100000));

    const std::vector<u32>& list = selection.selectedVertices();
    CHECK(list.size() == 4);
    CHECK(list[0] == 5 && list[1] == 63 && list[2] == 64 && list[3] == 200);

    selection.deselectVertex(63);
    CHECK(selection.selectedVertexCount() == 3);
    CHECK(!selection.isVertexSelected(63));
    CHECK(selection.selectedVertices().size() == 3);

    selection.deselectVertex(999);
    CHECK(selection.selectedVertexCount() == 3);

    selection.toggleVertex(5);
    CHECK(!selection.isVertexSelected(5));
    selection.toggleVertex(5);
    CHECK(selection.isVertexSelected(5));
    CHECK(selection.selectedVertexCount() == 3);
}

// A double select must not inflate the count, which is kept by hand with a bitmask.
void testDoubleSelectKeepsCount()
{
    BlenderSelection selection;
    selection.selectVertex(7);
    selection.selectVertex(7);
    selection.selectVertex(7);
    CHECK(selection.selectedVertexCount() == 1);
    CHECK(selection.selectedVertices().size() == 1);

    selection.setMode(BlenderSelection::SelectionMode::Face);
    selection.selectFace(3);
    selection.selectFace(3);
    CHECK(selection.selectedFaceCount() == 1);
}

// Bits past the count must be masked or selectedVertices() returns them as real indices.
void testSelectAllMasksTheTail()
{
    BlenderSelection selection;
    selection.selectAll(100, 0);

    CHECK(selection.selectedVertexCount() == 100);
    const std::vector<u32>& list = selection.selectedVertices();
    CHECK(list.size() == 100);
    CHECK(list.front() == 0);
    CHECK(list.back() == 99);
    CHECK(selection.isVertexSelected(99));
    CHECK(!selection.isVertexSelected(100));
    CHECK(!selection.isVertexSelected(127));

    BlenderSelection exact;
    exact.selectAll(128, 0);
    CHECK(exact.selectedVertexCount() == 128);
    CHECK(exact.selectedVertices().size() == 128);
    CHECK(exact.selectedVertices().back() == 127);
    CHECK(!exact.isVertexSelected(128));

    BlenderSelection empty;
    empty.selectAll(0, 0);
    CHECK(empty.selectedVertexCount() == 0);

    CHECK(selection.selectedFaceCount() == 0);
}

void testInvert()
{
    BlenderSelection selection;
    selection.selectVertex(0);
    selection.selectVertex(70);

    selection.invertSelection(100, 0);
    CHECK(selection.selectedVertexCount() == 98);
    CHECK(!selection.isVertexSelected(0));
    CHECK(!selection.isVertexSelected(70));
    CHECK(selection.isVertexSelected(1));
    CHECK(selection.isVertexSelected(99));
    CHECK(!selection.isVertexSelected(100));
    CHECK(selection.selectedVertices().size() == 98);

    selection.invertSelection(100, 0);
    CHECK(selection.selectedVertexCount() == 2);
    CHECK(selection.isVertexSelected(0));
    CHECK(selection.isVertexSelected(70));

    BlenderSelection fresh;
    fresh.invertSelection(70, 0);
    CHECK(fresh.selectedVertexCount() == 70);
    CHECK(fresh.selectedVertices().back() == 69);
}

// Invert against a smaller mesh: the new count must come from the remaining bits, not the old total.
void testInvertAfterMeshShrinks()
{
    BlenderSelection selection;
    selection.selectAll(1000, 0);
    CHECK(selection.selectedVertexCount() == 1000);

    selection.invertSelection(50, 0);
    CHECK(selection.selectedVertexCount() == 0);
    CHECK(selection.selectedVertices().empty());
    CHECK(!selection.isVertexSelected(0));
    CHECK(!selection.isVertexSelected(60));
}

void testClearAll()
{
    BlenderSelection selection;
    selection.selectVertex(1);
    selection.setMode(BlenderSelection::SelectionMode::Face);
    selection.selectFace(2);

    selection.clearAll();
    CHECK(selection.selectedVertexCount() == 0);
    CHECK(selection.selectedFaceCount() == 0);
    CHECK(selection.selectedVertices().empty());
    CHECK(selection.selectedFaces().empty());
    CHECK(!selection.isVertexSelected(1));
    CHECK(!selection.isFaceSelected(2));
}

void testFillVertexFlags()
{
    BlenderSelection selection;
    selection.selectVertex(0);
    selection.selectVertex(64);
    selection.selectVertex(65);
    selection.selectVertex(200);

    std::vector<u8> flags(100, 0xcd);
    selection.fillVertexFlags(flags.data(), static_cast<u32>(flags.size()));

    CHECK(flags[0] == 1);
    CHECK(flags[64] == 1);
    CHECK(flags[65] == 1);
    CHECK(flags[1] == 0);
    CHECK(flags[99] == 0);

    u32 set = 0;
    for (usize i = 0; i < flags.size(); ++i)
    {
        CHECK(flags[i] == 0 || flags[i] == 1);
        set += flags[i];
    }
    CHECK(set == 3);
}

// A no-op must not bump the revision (needless GPU re-upload); a real change must.
void testRevision()
{
    BlenderSelection selection;
    const u64 start = selection.revision();

    selection.selectVertex(4);
    const u64 afterSelect = selection.revision();
    CHECK(afterSelect != start);

    selection.selectVertex(4);
    CHECK(selection.revision() == afterSelect);

    selection.deselectVertex(9);
    CHECK(selection.revision() == afterSelect);

    selection.deselectVertex(4);
    CHECK(selection.revision() != afterSelect);
}

// The list is rebuilt lazily; a change between two requests must show in the second.
void testListFollowsChanges()
{
    BlenderSelection selection;
    selection.selectVertex(1);
    CHECK(selection.selectedVertices().size() == 1);

    selection.selectVertex(2);
    CHECK(selection.selectedVertices().size() == 2);

    selection.deselectVertex(1);
    const std::vector<u32>& list = selection.selectedVertices();
    CHECK(list.size() == 1);
    CHECK(list[0] == 2);

    selection.clearAll();
    CHECK(selection.selectedVertices().empty());
}

void testFacesAreIndependent()
{
    BlenderSelection selection;
    selection.selectVertex(3);
    selection.selectFace(3);

    CHECK(selection.isVertexSelected(3));
    CHECK(selection.isFaceSelected(3));

    selection.deselectVertex(3);
    CHECK(!selection.isVertexSelected(3));
    CHECK(selection.isFaceSelected(3));
    CHECK(selection.selectedFaceCount() == 1);
}

void testEdges()
{
    BlenderSelection selection;
    CHECK(selection.selectedEdgeCount() == 0);
    CHECK(selection.selectedEdges().empty());

    const u64 a = (u64(3) << 32) | 9;
    const u64 b = (u64(1) << 32) | 2;
    selection.selectEdge(a);
    selection.selectEdge(b);
    selection.selectEdge(a);
    CHECK(selection.selectedEdgeCount() == 2);
    CHECK(selection.isEdgeSelected(a));
    CHECK(!selection.isEdgeSelected(a + 1));

    const std::vector<u64>& list = selection.selectedEdges();
    CHECK(list.size() == 2 && list[0] == b && list[1] == a);

    const u64 before = selection.revision();
    selection.deselectEdge(12345);
    CHECK(selection.revision() == before);
    selection.deselectEdge(a);
    CHECK(selection.revision() != before);
    CHECK(selection.selectedEdgeCount() == 1);
    CHECK(selection.selectedEdges().size() == 1);

    selection.toggleEdge(a);
    CHECK(selection.isEdgeSelected(a));
    selection.toggleEdge(a);
    CHECK(!selection.isEdgeSelected(a));

    selection.setEdges({a, a, b});
    CHECK(selection.selectedEdgeCount() == 2);

    selection.selectVertex(4);
    selection.selectFace(1);
    CHECK(selection.selectedEdgeCount() == 2);
    selection.clearAll();
    CHECK(selection.selectedEdgeCount() == 0);
    CHECK(selection.selectedVertexCount() == 0 && selection.selectedFaceCount() == 0);
}

} // namespace

int main()
{
    testEdges();
    testSelectAndDeselect();
    testDoubleSelectKeepsCount();
    testSelectAllMasksTheTail();
    testInvert();
    testInvertAfterMeshShrinks();
    testClearAll();
    testFillVertexFlags();
    testRevision();
    testListFollowsChanges();
    testFacesAreIndependent();

    if (gFailures)
        std::fprintf(stderr, "%d blender selection test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
