#include "RectPacker.h"

//#define STB_RECT_PACK_IMPLEMENTATION
#include "stb_rect_pack.h"

#include <algorithm>

namespace RectPacker
{
    namespace
    {
        // Tries to place every rect in `remaining` onto one page. Anything
        // that doesn't fit is left in `remaining` (stb_rect_pack sets
        // was_packed=0 for those) for the caller to retry on the next page.
        Page PackOnePage(std::vector<stbrp_rect>& remaining, int32_t max_width, int32_t max_height)
        {
            Page page;
            page.atlas_width = max_width;
            page.atlas_height = max_height;

            const int32_t node_count = max_width;
            std::vector<stbrp_node> nodes(node_count);

            stbrp_context ctx;
            stbrp_init_target(&ctx, max_width, max_height, nodes.data(), node_count);
            stbrp_pack_rects(&ctx, remaining.data(), static_cast<int>(remaining.size()));

            std::vector<stbrp_rect> still_remaining;
            int32_t used_w = 0;
            int32_t used_h = 0;

            for (const stbrp_rect& r : remaining)
            {
                if (r.was_packed)
                {
                    PlacedRect placed;
                    placed.id = static_cast<int32_t>(r.id);
                    placed.x = r.x;
                    placed.y = r.y;
                    placed.width = r.w;
                    placed.height = r.h;
                    page.placed.push_back(placed);
                    used_w = std::max(used_w, r.x + r.w);
                    used_h = std::max(used_h, r.y + r.h);
                }
                else
                {
                    still_remaining.push_back(r);
                }
            }

            // Trim the page to what was actually used - callers exporting a
            // PNG want a tight atlas, not a full max_width x max_height blank
            // canvas on the last (usually mostly-empty) page.
            page.atlas_width = std::max(used_w, 1);
            page.atlas_height = std::max(used_h, 1);

            remaining = std::move(still_remaining);
            return page;
        }
    }

    std::vector<Page> PackPages(std::vector<Input> inputs, int32_t max_width, int32_t max_height, int32_t padding)
    {
        std::vector<Page> pages;

        std::vector<stbrp_rect> rects;
        rects.reserve(inputs.size());
        for (const Input& in : inputs)
        {
            if (in.width > max_width || in.height > max_height)
            {
                continue; // can never fit on any page - caller should validate before this
            }
            stbrp_rect r{};
            r.id = in.id;
            r.w = static_cast<stbrp_coord>(in.width + padding);
            r.h = static_cast<stbrp_coord>(in.height + padding);
            rects.push_back(r);
        }

        // Each PackOnePage pass removes everything that fit; loop until
        // nothing is left (or nothing more can be placed, to avoid an
        // infinite loop if stb_rect_pack ever fails to make progress).
        while (!rects.empty())
        {
            const size_t before = rects.size();
            Page page = PackOnePage(rects, max_width, max_height);
            if (page.placed.empty())
            {
                break; // no progress - remaining rects can't be packed
            }
            // Padding was added to w/h for packing; report the caller's
            // original (unpadded) size back out.
            for (PlacedRect& p : page.placed)
            {
                p.width -= padding;
                p.height -= padding;
            }
            pages.push_back(std::move(page));
            if (rects.size() == before)
            {
                break;
            }
        }

        return pages;
    }
}
