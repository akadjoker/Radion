#include "PCH.h"

#include "ScreenDraw.h"

#include <cstdio>
#include <cstring>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "ScreenDrawTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

void testCommandsAreQueuedAndCleared()
{
    ScreenDraw& queue = ScreenDraws();
    queue.clear();
    CHECK(queue.empty());

    queue.line(0.0f, 0.0f, 10.0f, 10.0f, Color::White);
    queue.rect(0.0f, 0.0f, 5.0f, 5.0f, Color::Red);
    queue.sprite(TextureHandle(), 0.0f, 0.0f, 8.0f, 8.0f, Color::Green);
    queue.text(0.0f, 0.0f, 12.0f, Color::Blue, "hud");
    queue.fade(Color::Black);

    CHECK(!queue.empty());
    const std::vector<ScreenDrawCommand>& commands = queue.commands();
    CHECK(commands.size() == 5);

    int lines = 0, rects = 0, sprites = 0, texts = 0;
    for (const ScreenDrawCommand& command : commands)
    {
        switch (command.type)
        {
        case ScreenDrawCommandType::Line:
            ++lines;
            break;
        case ScreenDrawCommandType::Rect:
            ++rects;
            break;
        case ScreenDrawCommandType::Sprite:
            ++sprites;
            break;
        case ScreenDrawCommandType::Text:
            ++texts;
            break;
        }
    }
    CHECK(lines == 1);
    // fade() also queues a Rect command, so rect() + fade() makes two.
    CHECK(rects == 2);
    CHECK(sprites == 1);
    CHECK(texts == 1);

    queue.clear();
    CHECK(queue.empty());
    CHECK(queue.commands().empty());
}

void testLayerOrderIsStable()
{
    ScreenDraw& queue = ScreenDraws();
    queue.clear();

    // Submission order: id 0 (layer 1), id 1 (layer 0), id 2 (layer 1),
    // id 3 (layer 0). A stable sort by ascending layer must produce
    // [1, 3, 0, 2] - the layer-0 pair and the layer-1 pair each keeping
    // their own submission order.
    queue.rect(0.0f, 0.0f, 0.0f, 0.0f, Color::White, true, 1);
    queue.rect(0.0f, 0.0f, 0.0f, 1.0f, Color::White, true, 0);
    queue.rect(0.0f, 0.0f, 0.0f, 2.0f, Color::White, true, 1);
    queue.rect(0.0f, 0.0f, 0.0f, 3.0f, Color::White, true, 0);

    const std::vector<ScreenDrawCommand>& commands = queue.commands();
    CHECK(commands.size() == 4);
    if (commands.size() == 4)
    {
        const f32 expectedId[4] = {1.0f, 3.0f, 0.0f, 2.0f};
        const s32 expectedLayer[4] = {0, 0, 1, 1};
        for (int i = 0; i < 4; ++i)
        {
            CHECK(commands[static_cast<usize>(i)].h == expectedId[i]);
            CHECK(commands[static_cast<usize>(i)].layer == expectedLayer[i]);
        }
    }

    queue.clear();
}

void testTextIsCopiedNotAliased()
{
    ScreenDraw& queue = ScreenDraws();
    queue.clear();

    char source[16];
    std::strcpy(source, "hello");
    queue.text(1.0f, 2.0f, 14.0f, Color::White, source, 3);
    // The queue must have copied the string already - stomping the source
    // buffer right after submitting must not change what comes back out.
    std::memset(source, 'X', sizeof(source));

    const std::vector<ScreenDrawCommand>& commands = queue.commands();
    CHECK(commands.size() == 1);
    if (!commands.empty())
    {
        const ScreenDrawCommand& command = commands[0];
        CHECK(command.type == ScreenDrawCommandType::Text);
        CHECK(command.layer == 3);
        CHECK(command.textLength == 5);
        CHECK(std::strcmp(queue.textAt(command.textOffset), "hello") == 0);
    }

    queue.clear();
}

void testClearKeepsCapacity()
{
    ScreenDraw& queue = ScreenDraws();
    queue.clear();

    for (int i = 0; i < 2000; ++i)
        queue.rect(static_cast<f32>(i), 0.0f, 1.0f, 1.0f, Color::White);

    const usize grownCapacity = queue.commands().capacity();
    CHECK(grownCapacity >= 2000);

    queue.clear();
    CHECK(queue.empty());
    // clear() must keep the vector's storage - reallocating every frame is
    // exactly what a per-frame command queue cannot afford.
    CHECK(queue.commands().capacity() == grownCapacity);
}

void testFadeCoversWholeScreen()
{
    ScreenDraw& queue = ScreenDraws();
    queue.clear();

    queue.fade(Color::Black, 5);
    const std::vector<ScreenDrawCommand>& commands = queue.commands();
    CHECK(commands.size() == 1);
    if (!commands.empty())
    {
        const ScreenDrawCommand& command = commands[0];
        CHECK(command.type == ScreenDrawCommandType::Rect);
        CHECK(command.fullscreen);
        CHECK(command.layer == 5);

        const FloatRect hd = ScreenDraw::resolvedRect(command, 1920.0f, 1080.0f);
        CHECK(hd.x == 0.0f && hd.y == 0.0f);
        CHECK(hd.width == 1920.0f && hd.height == 1080.0f);

        // Not baked at submission time - the same command resolves to
        // whatever resolution it is asked about.
        const FloatRect sd = ScreenDraw::resolvedRect(command, 640.0f, 480.0f);
        CHECK(sd.width == 640.0f && sd.height == 480.0f);
    }

    queue.clear();
}

} // namespace

int main()
{
    testCommandsAreQueuedAndCleared();
    testLayerOrderIsStable();
    testTextIsCopiedNotAliased();
    testClearKeepsCapacity();
    testFadeCoversWholeScreen();
    if (gFailures)
        std::fprintf(stderr, "%d screen draw test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
