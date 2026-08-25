#pragma once

// Multi-page atlas packing on top of stb_rect_pack (vendor/stb -
// Skyline-BF, much better density than a hand-rolled shelf packer).
//
// A single page is capped at max_width x max_height (real GPU texture
// size limits, typically 2048 or 4096) - whatever doesn't fit spills
// into a new page rather than failing, so a big FPG naturally becomes
// atlas_0.png, atlas_1.png, ... each with its own sidecar entries.
//
// Usage:
//   std::vector<RectPacker::Input> in = { {code, w, h}, ... };
//   auto pages = RectPacker::PackPages(in, 2048, 2048);
//   // pages[i].placed[j] -> which page + xy each input landed on

#include <cstdint>
#include <vector>

namespace RectPacker
{
    struct Input
    {
        int32_t id = 0;      // caller-defined tag (e.g. FPG sprite code)
        int32_t width = 0;
        int32_t height = 0;
    };

    struct PlacedRect
    {
        int32_t id = 0;
        int32_t x = 0;
        int32_t y = 0;
        int32_t width = 0;
        int32_t height = 0;
    };

    struct Page
    {
        std::vector<PlacedRect> placed;
        int32_t atlas_width = 0;
        int32_t atlas_height = 0;
    };

    // Packs `inputs` across as many pages as needed. A single rect wider
    // or taller than max_width/max_height on its own is impossible to
    // place and is silently dropped (callers should validate sprite
    // sizes against the page cap before calling, e.g. in the UI).
    std::vector<Page> PackPages(std::vector<Input> inputs, int32_t max_width, int32_t max_height, int32_t padding = 1);
}
