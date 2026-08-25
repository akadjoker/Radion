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
    // Well past anything ever touched: must answer, not read out of bounds.
    CHECK(!selection.isVertexSelected(100000));

    // The list comes out ascending, whatever order they were selected in.
    const std::vector<u32>& list = selection.selectedVertices();
    CHECK(list.size() == 4);
    CHECK(list[0] == 5 && list[1] == 63 && list[2] == 64 && list[3] == 200);

    selection.deselectVertex(63);
    CHECK(selection.selectedVertexCount() == 3);
    CHECK(!selection.isVertexSelected(63));
    CHECK(selection.selectedVertices().size() == 3);

    // Deselecting what was never selected changes nothing.
    selection.deselectVertex(999);
    CHECK(selection.selectedVertexCount() == 3);

    selection.toggleVertex(5);
    CHECK(!selection.isVertexSelected(5));
    selection.toggleVertex(5);
    CHECK(selection.isVertexSelected(5));
    CHECK(selection.selectedVertexCount() == 3);
}

// Selecting the same index twice used to be harmless because the list was
// searched first. With a bitmask the count is kept by hand, so a double
// select must not inflate it.
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

// The bits past the count belong to no vertex. If the last word is not
// masked they come back from selectedVertices() as real indices, and
// whatever consumes them - deleteVertices, smoothVertices - runs off the end
// of the mesh.
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

    // An exact multiple of the word size has no tail to mask.
    BlenderSelection exact;
    exact.selectAll(128, 0);
    CHECK(exact.selectedVertexCount() == 128);
    CHECK(exact.selectedVertices().size() == 128);
    CHECK(exact.selectedVertices().back() == 127);
    CHECK(!exact.isVertexSelected(128));

    BlenderSelection empty;
    empty.selectAll(0, 0);
    CHECK(empty.selectedVertexCount() == 0);

    // selectAll follows the mode: faces stay untouched in vertex mode.
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

    // Twice is the identity.
    selection.invertSelection(100, 0);
    CHECK(selection.selectedVertexCount() == 2);
    CHECK(selection.isVertexSelected(0));
    CHECK(selection.isVertexSelected(70));

    // From nothing, invert is everything.
    BlenderSelection fresh;
    fresh.invertSelection(70, 0);
    CHECK(fresh.selectedVertexCount() == 70);
    CHECK(fresh.selectedVertices().back() == 69);
}

// A selection made on a big mesh, then inverted against a smaller one: the
// words holding the old high indices are dropped, and the new count has to
// come from the bits that remain rather than from the old total.
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

// What the viewport uploads to the GPU. Anything wrong here shows as the
// wrong vertices lighting up.
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
    // Index 200 is past the buffer and must simply not be written.
    CHECK(set == 3);
}

// The viewport re-uploads on a revision change alone, so a no-op must not
// bump it (needless GPU traffic) and a real change must (a stale selection
// on screen otherwise).
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

// The list is rebuilt lazily from the bits; asking for it twice with a
// change in between has to give the second answer, not the cached first.
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

} // namespace

int main()
{
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
