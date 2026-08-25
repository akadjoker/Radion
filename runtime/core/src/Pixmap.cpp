#include "PCH.h"

#include "Pixmap.h"

#include "Color.h"
#include "FileSystem.h"
#include "Log.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
 
#include "stb_image.h"
#include "stb_image_write.h"

namespace Radion
{

// ── helpers ──────────────────────────────────────────────────────────────

static inline u8 clamp_to_byte(int value)
{
    return static_cast<u8>(value < 0 ? 0 : (value > 255 ? 255 : value));
}

static inline u8 mix_byte(u8 dst, u8 src, float alpha)
{
    const float ca = alpha < 0.0f ? 0.0f : (alpha > 1.0f ? 1.0f : alpha);
    const float ia = 1.0f - ca;
    return clamp_to_byte(
        static_cast<int>(std::lround(static_cast<float>(src) * ca + static_cast<float>(dst) * ia)));
}

static Color blend_src_over(const Color& src, const Color& dst, float opacity)
{
    const float co = opacity < 0.0f ? 0.0f : (opacity > 1.0f ? 1.0f : opacity);
    if (co <= 0.0f)
        return dst;

    const u32 srcA = clamp_to_byte(
        static_cast<int>(std::lround((static_cast<float>(src.a()) / 255.0f) * co * 255.0f)));
    if (srcA == 0)
        return dst;

    const u32 srcR = src.r(), srcG = src.g(), srcB = src.b();
    u32 dstA = dst.a(), dstR = dst.r(), dstG = dst.g(), dstB = dst.b();

    dstA -= (dstA * srcA) / 255;
    const u32 outA = dstA + srcA;
    if (outA == 0)
        return Color(0, 0, 0, 0);

    return Color(static_cast<u8>((dstR * dstA + srcR * srcA) / outA),
                 static_cast<u8>((dstG * dstA + srcG * srcA) / outA),
                 static_cast<u8>((dstB * dstA + srcB * srcA) / outA), static_cast<u8>(outA));
}

static int select_components(int src_comp)
{
    return (src_comp == 2 || src_comp == 4) ? 4 : 3;
}

static bool has_extension(const char* fn, const char* ext)
{
    if (!fn || !ext)
        return false;
    const char* dot = strrchr(fn, '.');
    if (!dot)
        return false;
    return strcmp(dot, ext) == 0;
}

static void log_err(const char* msg, const char* detail)
{
    Log::error("[PIXMAP] %s: %s", msg, detail ? detail : "");
}

static bool assign_loaded_pixels(Pixmap& pm, const unsigned char* src, int w, int h, int src_comp)
{
    if (!src || w <= 0 || h <= 0 || src_comp < 1 || src_comp > 4)
        return false;

    const int nc = select_components(src_comp);
    const size_t total = static_cast<size_t>(w) * static_cast<size_t>(h) * 4;
    unsigned char* np = static_cast<unsigned char*>(malloc(total));
    if (!np)
    {
        log_err("malloc failed", "");
        return false;
    }
    memset(np, 0, total);

    if (nc == src_comp)
    {
        memcpy(np, src, static_cast<size_t>(w) * static_cast<size_t>(h) * static_cast<size_t>(nc));
    }
    else
    {
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x)
            {
                const size_t si =
                    (static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x)) *
                    static_cast<size_t>(src_comp);
                const size_t di =
                    (static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x)) *
                    static_cast<size_t>(nc);
                const unsigned char r = src[si + 0];
                const unsigned char g = (src_comp >= 3) ? src[si + 1] : r;
                const unsigned char b = (src_comp >= 3) ? src[si + 2] : r;
                const unsigned char a = (src_comp == 2)   ? src[si + 1]
                                        : (src_comp == 4) ? src[si + 3]
                                                          : 255;
                if (nc == 3)
                {
                    np[di + 0] = r;
                    np[di + 1] = g;
                    np[di + 2] = b;
                }
                else
                {
                    np[di + 0] = r;
                    np[di + 1] = g;
                    np[di + 2] = b;
                    np[di + 3] = a;
                }
            }
    }

    if (pm.pixels)
        free(pm.pixels);
    pm.pixels = np;
    pm.width = w;
    pm.height = h;
    pm.components = nc;
    return true;
}

static bool build_save_buf(const Pixmap& pm, std::vector<unsigned char>& scr, int& sc,
                           const unsigned char*& sp)
{
    if (!pm.pixels || pm.width <= 0 || pm.height <= 0)
        return false;
    if (pm.components == 1 || pm.components == 3 || pm.components == 4)
    {
        sc = pm.components;
        sp = pm.pixels;
        return true;
    }
    sc = 4;
    scr.resize(static_cast<size_t>(pm.width) * static_cast<size_t>(pm.height) * 4);
    for (int y = 0; y < pm.height; y++)
        for (int x = 0; x < pm.width; x++)
        {
            const size_t di = static_cast<size_t>(y * pm.width + x) * 4;
            const Color c = pm.get_pixel_color(x, y);
            scr[di + 0] = c.r();
            scr[di + 1] = c.g();
            scr[di + 2] = c.b();
            scr[di + 3] = c.a();
        }
    sp = scr.data();
    return true;
}

static bool save_tga(const Pixmap& pm, const char* fn)
{
    const bool gray = (pm.components == 1);
    const int bpp = gray ? 8 : ((pm.components == 4 || pm.components == 2) ? 32 : 24);
    unsigned char hdr[18] = {};
    hdr[2] = gray ? 3 : 2;
    hdr[12] = static_cast<unsigned char>(pm.width & 0xFF);
    hdr[13] = static_cast<unsigned char>((pm.width >> 8) & 0xFF);
    hdr[14] = static_cast<unsigned char>(pm.height & 0xFF);
    hdr[15] = static_cast<unsigned char>((pm.height >> 8) & 0xFF);
    hdr[16] = static_cast<unsigned char>(bpp);
    hdr[17] = static_cast<unsigned char>((bpp == 32 ? 8 : 0) | 0x20);

    ByteArray data;
    data.reserve(sizeof(hdr) + static_cast<usize>(pm.width) * static_cast<usize>(pm.height) *
                                    static_cast<usize>(bpp / 8));
    data.writeBytes(hdr, sizeof(hdr));

    std::vector<unsigned char> row(static_cast<size_t>(pm.width) * static_cast<size_t>(bpp / 8));
    for (int y = 0; y < pm.height; y++)
    {
        if (gray)
            for (int x = 0; x < pm.width; x++)
                row[x] = pm.get_pixel_color(x, y).r();
        else if (bpp == 24)
            for (int x = 0; x < pm.width; x++)
            {
                const Color c = pm.get_pixel_color(x, y);
                const size_t o = static_cast<size_t>(x) * 3;
                row[o + 0] = c.b();
                row[o + 1] = c.g();
                row[o + 2] = c.r();
            }
        else
            for (int x = 0; x < pm.width; x++)
            {
                const Color c = pm.get_pixel_color(x, y);
                const size_t o = static_cast<size_t>(x) * 4;
                row[o + 0] = c.b();
                row[o + 1] = c.g();
                row[o + 2] = c.r();
                row[o + 3] = c.a();
            }
        data.writeBytes(row.data(), row.size());
    }

    if (!FileSystem::getSingleton().writeBinary(fn, data))
    {
        log_err("TGA open failed", fn);
        return false;
    }
    return true;
}

// ── constructors / destructor ────────────────────────────────────────────

Pixmap::Pixmap() : pixels(nullptr), components(0), width(0), height(0)
{
}

Pixmap::~Pixmap()
{
    if (pixels)
        free(pixels);
}

Pixmap::Pixmap(const Pixmap& img, const IntRect& crop)
{
    width = crop.width;
    height = crop.height;
    components = img.components;
    pixels = static_cast<unsigned char*>(
        malloc(static_cast<size_t>(width) * static_cast<size_t>(height) * 4));
    memset(pixels, 0, static_cast<size_t>(width) * static_cast<size_t>(height) * 4);
    for (int y = static_cast<int>(crop.y), off = 0; y < static_cast<int>(crop.y + crop.height); y++)
    {
        memcpy(pixels + off, img.pixels + (y * img.width + static_cast<int>(crop.x)) * components,
               static_cast<size_t>(static_cast<int>(crop.width)) * static_cast<size_t>(components));
        off += static_cast<int>(crop.width) * components;
    }
}

Pixmap::Pixmap(int w, int h, int comp) : components(comp), width(w), height(h)
{
    pixels =
        static_cast<unsigned char*>(malloc(static_cast<size_t>(w) * static_cast<size_t>(h) * 4));
    memset(pixels, 0, static_cast<size_t>(w) * static_cast<size_t>(h) * 4);
}

Pixmap::Pixmap(int w, int h, int comp, unsigned char* data) : components(comp), width(w), height(h)
{
    pixels =
        static_cast<unsigned char*>(malloc(static_cast<size_t>(w) * static_cast<size_t>(h) * 4));
    memset(pixels, 0, static_cast<size_t>(w) * static_cast<size_t>(h) * 4);
    memcpy(pixels, data,
           static_cast<size_t>(w) * static_cast<size_t>(h) * static_cast<size_t>(comp));
}

// ── pixel ops ────────────────────────────────────────────────────────────

void Pixmap::set_pixel(u32 x, u32 y, u8 r, u8 g, u8 b, u8 a)
{
    if (x >= static_cast<u32>(width) || y >= static_cast<u32>(height))
        return;

    if (components == 1)
    {
        float gray = static_cast<float>(r) * 0.299f + static_cast<float>(g) * 0.587f +
                     static_cast<float>(b) * 0.114f;
        pixels[y * width + x] = static_cast<unsigned char>(gray);
    }
    else if (components == 2)
    {
        float gray = static_cast<float>(r) * 0.299f + static_cast<float>(g) * 0.587f +
                     static_cast<float>(b) * 0.114f;
        pixels[(y * width + x) * 2] = static_cast<unsigned char>(gray);
        pixels[(y * width + x) * 2 + 1] = a;
    }
    else if (components == 3)
    {
        pixels[(y * width + x) * 3] = r;
        pixels[(y * width + x) * 3 + 1] = g;
        pixels[(y * width + x) * 3 + 2] = b;
    }
    else
    {
        pixels[(y * width + x) * 4] = r;
        pixels[(y * width + x) * 4 + 1] = g;
        pixels[(y * width + x) * 4 + 2] = b;
        pixels[(y * width + x) * 4 + 3] = a;
    }
}

void Pixmap::set_pixel(u32 x, u32 y, u32 rgba)
{
    set_pixel(x, y, static_cast<u8>(rgba), static_cast<u8>(rgba >> 8), static_cast<u8>(rgba >> 16),
              static_cast<u8>(rgba >> 24));
}

u32 Pixmap::get_pixel(u32 x, u32 y) const
{
    if (x >= static_cast<u32>(width) || y >= static_cast<u32>(height))
        return 0;

    if (components == 1)
        return pixels[y * width + x];
    if (components == 2)
        return pixels[(y * width + x) * 2] |
               (static_cast<u32>(pixels[(y * width + x) * 2 + 1]) << 8);
    if (components == 3)
        return pixels[(y * width + x) * 3] |
               (static_cast<u32>(pixels[(y * width + x) * 3 + 1]) << 8) |
               (static_cast<u32>(pixels[(y * width + x) * 3 + 2]) << 16);
    return pixels[(y * width + x) * 4] | (static_cast<u32>(pixels[(y * width + x) * 4 + 1]) << 8) |
           (static_cast<u32>(pixels[(y * width + x) * 4 + 2]) << 16) |
           (static_cast<u32>(pixels[(y * width + x) * 4 + 3]) << 24);
}

Color Pixmap::get_pixel_color(u32 x, u32 y) const
{
    if (x >= static_cast<u32>(width) || y >= static_cast<u32>(height))
        return Color(0, 0, 0, 0);

    u8 r = 0, g = 0, b = 0, a = 255;
    if (components == 1)
        r = g = b = pixels[y * width + x];
    else if (components == 2)
    {
        r = g = b = pixels[(y * width + x) * 2];
        a = pixels[(y * width + x) * 2 + 1];
    }
    else if (components == 3)
    {
        r = pixels[(y * width + x) * 3];
        g = pixels[(y * width + x) * 3 + 1];
        b = pixels[(y * width + x) * 3 + 2];
    }
    else
    {
        r = pixels[(y * width + x) * 4];
        g = pixels[(y * width + x) * 4 + 1];
        b = pixels[(y * width + x) * 4 + 2];
        a = pixels[(y * width + x) * 4 + 3];
    }
    return Color(r, g, b, a);
}

// ── fill ──────────────────────────────────────────────────────────────────

void Pixmap::fill(u8 r, u8 g, u8 b, u8 a)
{
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
            set_pixel(static_cast<u32>(x), static_cast<u32>(y), r, g, b, a);
}

void Pixmap::fill(u32 rgba)
{
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
            set_pixel(static_cast<u32>(x), static_cast<u32>(y), rgba);
}

void Pixmap::clear()
{
    if (pixels)
        memset(pixels, 0,
               static_cast<size_t>(width) * static_cast<size_t>(height) *
                   static_cast<size_t>(components));
}

// ── file I/O ──────────────────────────────────────────────────────────────

bool Pixmap::load(const char* fn)
{
    // resolve through the filesystem so registered search folders apply
    // (demos run from any working directory)
    ByteArray data = FileSystem::getSingleton().readBinary(fn);
    if (data.size() == 0)
    {
        log_err("Failed to load image", fn);
        return false;
    }
    bool ok = load_from_memory(data.data(), data.size());
    if (ok)
        fprintf(stdout, "[PIXMAP] Load: %s (%d,%d) bpp:%d\n", fn, width, height, components);
    return ok;
}

bool Pixmap::load_from_memory(const unsigned char* buf, u32 sz)
{
    if (!buf || sz == 0)
    {
        log_err("Invalid buffer", "");
        return false;
    }
    int w = 0, h = 0, sc = 0;
    unsigned char* ld = stbi_load_from_memory(buf, static_cast<int>(sz), &w, &h, &sc, 0);
    if (!ld)
    {
        log_err("Failed to load from memory", stbi_failure_reason());
        return false;
    }
    bool ok = assign_loaded_pixels(*this, ld, w, h, sc);
    stbi_image_free(ld);
    return ok;
}

bool Pixmap::save(const char* fn)
{
    if (!pixels)
    {
        log_err("Null pixels, cannot save", fn);
        return false;
    }
    if (has_extension(fn, ".tga"))
        return save_tga(*this, fn);

    std::vector<unsigned char> scr;
    int sc = 0;
    const unsigned char* sp = nullptr;
    if (!build_save_buf(*this, scr, sc, sp))
    {
        log_err("Build save buffer failed", fn);
        return false;
    }

    bool ok = false;
    if (has_extension(fn, ".bmp"))
    {
        ok = stbi_write_bmp(fn, width, height, sc, sp) != 0;
        if (!ok)
            log_err("BMP save failed", fn);
    }
    else if (has_extension(fn, ".png"))
    {
        ok = stbi_write_png(fn, width, height, sc, sp, width * sc) != 0;
        if (!ok)
            log_err("PNG save failed", fn);
    }
    else
        log_err("Unsupported format", fn);
    return ok;
}

// ── transforms ────────────────────────────────────────────────────────────

void Pixmap::flip_vertical()
{
    if (!pixels)
        return;
    int rs = width * components;
    unsigned char* row = static_cast<unsigned char*>(malloc(static_cast<size_t>(rs)));
    for (int y = 0; y < height / 2; y++)
    {
        unsigned char* s = pixels + y * rs;
        unsigned char* d = pixels + (height - y - 1) * rs;
        memcpy(row, s, static_cast<size_t>(rs));
        memcpy(s, d, static_cast<size_t>(rs));
        memcpy(d, row, static_cast<size_t>(rs));
    }
    free(row);
}

void Pixmap::flip_horizontal()
{
    if (!pixels)
        return;
    int rs = width * components;
    unsigned char* tmp = static_cast<unsigned char*>(malloc(static_cast<size_t>(components)));
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width / 2; x++)
        {
            unsigned char* s = pixels + y * rs + x * components;
            unsigned char* d = pixels + y * rs + (width - x - 1) * components;
            memcpy(tmp, s, static_cast<size_t>(components));
            memcpy(s, d, static_cast<size_t>(components));
            memcpy(d, tmp, static_cast<size_t>(components));
        }
    free(tmp);
}

void Pixmap::tint(u8 r, u8 g, u8 b)
{
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            Color c = get_pixel_color(x, y);
            set_pixel(static_cast<u32>(x), static_cast<u32>(y), static_cast<u8>((c.r() * r) / 255),
                      static_cast<u8>((c.g() * g) / 255), static_cast<u8>((c.b() * b) / 255),
                      c.a());
        }
}

Pixmap* Pixmap::convert_to_rgba() const
{
    if (components == 4)
        return new Pixmap(width, height, components, pixels);
    Pixmap* r = new Pixmap(width, height, 4);
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            Color c = get_pixel_color(x, y);
            r->set_pixel(static_cast<u32>(x), static_cast<u32>(y), c.r(), c.g(), c.b(), c.a());
        }
    return r;
}

Pixmap* Pixmap::resize(int nw, int nh) const
{
    Pixmap* r = new Pixmap(nw, nh, components);
    for (int y = 0; y < nh; y++)
        for (int x = 0; x < nw; x++)
            r->set_pixel(static_cast<u32>(x), static_cast<u32>(y),
                         get_pixel((x * width) / nw, (y * height) / nh));
    return r;
}

// ── drawing ───────────────────────────────────────────────────────────────

void Pixmap::draw_line(int x1, int y1, int x2, int y2, const Color& color)
{
    int dx = abs(x2 - x1), dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1, sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    while (true)
    {
        set_pixel(static_cast<u32>(x1), static_cast<u32>(y1), color.r(), color.g(), color.b(),
                  color.a());
        if (x1 == x2 && y1 == y2)
            break;
        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y1 += sy;
        }
    }
}

void Pixmap::draw_rect(int x, int y, int w, int h, const Color& color, bool filled)
{
    if (!pixels)
        return;
    if (filled)
    {
        for (int dy = 0; dy < h; dy++)
            for (int dx = 0; dx < w; dx++)
            {
                int px = x + dx, py = y + dy;
                if (px >= 0 && px < width && py >= 0 && py < height)
                    set_pixel(static_cast<u32>(px), static_cast<u32>(py), color.r(), color.g(),
                              color.b(), color.a());
            }
    }
    else
    {
        for (int dx = 0; dx < w; dx++)
        {
            if (x + dx >= 0 && x + dx < width && y >= 0 && y < height)
                set_pixel(x + dx, y, color.r(), color.g(), color.b(), color.a());
            if (x + dx >= 0 && x + dx < width && y + h - 1 >= 0 && y + h - 1 < height)
                set_pixel(x + dx, y + h - 1, color.r(), color.g(), color.b(), color.a());
        }
        for (int dy = 0; dy < h; dy++)
        {
            if (x >= 0 && x < width && y + dy >= 0 && y + dy < height)
                set_pixel(x, y + dy, color.r(), color.g(), color.b(), color.a());
            if (x + w - 1 >= 0 && x + w - 1 < width && y + dy >= 0 && y + dy < height)
                set_pixel(x + w - 1, y + dy, color.r(), color.g(), color.b(), color.a());
        }
    }
}

void Pixmap::draw_circle(int cx, int cy, int radius, const Color& color, bool filled)
{
    if (!pixels)
        return;
    if (filled)
    {
        int r2 = radius * radius;
        for (int y = -radius; y <= radius; y++)
            for (int x = -radius; x <= radius; x++)
                if (x * x + y * y <= r2)
                {
                    int px = cx + x, py = cy + y;
                    if (px >= 0 && px < width && py >= 0 && py < height)
                        set_pixel(static_cast<u32>(px), static_cast<u32>(py), color.r(), color.g(),
                                  color.b(), color.a());
                }
    }
    else
    {
        int x = 0, y = radius, d = 3 - 2 * radius;
        while (x <= y)
        {
            auto pt = [&](int px, int py)
            {
                if (px >= 0 && px < width && py >= 0 && py < height)
                    set_pixel(static_cast<u32>(px), static_cast<u32>(py), color.r(), color.g(),
                              color.b(), color.a());
            };
            pt(cx + x, cy + y);
            pt(cx - x, cy + y);
            pt(cx + x, cy - y);
            pt(cx - x, cy - y);
            pt(cx + y, cy + x);
            pt(cx - y, cy + x);
            pt(cx + y, cy - x);
            pt(cx - y, cy - x);
            if (d < 0)
                d += 4 * x + 6;
            else
            {
                d += 4 * (x - y) + 10;
                y--;
            }
            x++;
        }
    }
}

void Pixmap::blend_pixel(u32 x, u32 y, const Color& color, float opacity, BlendMode mode)
{
    if (x >= static_cast<u32>(width) || y >= static_cast<u32>(height) || !pixels)
        return;
    const float co = opacity < 0.0f ? 0.0f : (opacity > 1.0f ? 1.0f : opacity);
    if (co <= 0.0f)
        return;

    const Color dst = get_pixel_color(x, y);
    Color out = dst;

    switch (mode)
    {
    case BlendMode::copy:
        out = Color(color.r(), color.g(), color.b(), mix_byte(dst.a(), color.a(), co));
        break;
    case BlendMode::add:
        out =
            Color(clamp_to_byte(dst.r() +
                                static_cast<int>(std::lround(static_cast<float>(color.r()) * co))),
                  clamp_to_byte(dst.g() +
                                static_cast<int>(std::lround(static_cast<float>(color.g()) * co))),
                  clamp_to_byte(dst.b() +
                                static_cast<int>(std::lround(static_cast<float>(color.b()) * co))),
                  clamp_to_byte(dst.a() +
                                static_cast<int>(std::lround(static_cast<float>(color.a()) * co))));
        break;
    case BlendMode::multiply:
        out = Color(
            mix_byte(dst.r(),
                     clamp_to_byte(static_cast<int>(
                         (static_cast<float>(dst.r()) * static_cast<float>(color.r())) / 255.0f)),
                     co),
            mix_byte(dst.g(),
                     clamp_to_byte(static_cast<int>(
                         (static_cast<float>(dst.g()) * static_cast<float>(color.g())) / 255.0f)),
                     co),
            mix_byte(dst.b(),
                     clamp_to_byte(static_cast<int>(
                         (static_cast<float>(dst.b()) * static_cast<float>(color.b())) / 255.0f)),
                     co),
            mix_byte(dst.a(), color.a(), co));
        break;
    default:
        out = blend_src_over(color, dst, co);
        break;
    }
    set_pixel(x, y, out.r(), out.g(), out.b(), out.a());
}

void Pixmap::draw_pixmap(const Pixmap& src, int x, int y)
{
    if (!pixels || !src.pixels)
        return;
    for (int sy = 0; sy < src.height; sy++)
        for (int sx = 0; sx < src.width; sx++)
        {
            int dx = x + sx, dy = y + sy;
            if (dx < 0 || dx >= width || dy < 0 || dy >= height)
                continue;
            Color c = src.get_pixel_color(sx, sy);
            if (src.components == 4 || src.components == 2)
            {
                if (c.a() == 255)
                    set_pixel(static_cast<u32>(dx), static_cast<u32>(dy), c.r(), c.g(), c.b(),
                              c.a());
                else if (c.a() > 0)
                {
                    Color dd = get_pixel_color(dx, dy);
                    float al = c.a() / 255.0f;
                    set_pixel(static_cast<u32>(dx), static_cast<u32>(dy),
                              static_cast<u8>(c.r() * al + dd.r() * (1.0f - al)),
                              static_cast<u8>(c.g() * al + dd.g() * (1.0f - al)),
                              static_cast<u8>(c.b() * al + dd.b() * (1.0f - al)), dd.a());
                }
            }
            else
                set_pixel(static_cast<u32>(dx), static_cast<u32>(dy), c.r(), c.g(), c.b(), 255);
        }
}

void Pixmap::draw_pixmap(const Pixmap& src, int x, int y, const IntRect& sr)
{
    if (!pixels || !src.pixels)
        return;
    int sx = sr.x, sy = sr.y, sw = sr.width, sh = sr.height;
    if (sx < 0)
    {
        sw += sx;
        x -= sx;
        sx = 0;
    }
    if (sy < 0)
    {
        sh += sy;
        y -= sy;
        sy = 0;
    }
    if (sx + sw > src.width)
        sw = src.width - sx;
    if (sy + sh > src.height)
        sh = src.height - sy;

    for (int iy = 0; iy < sh; iy++)
        for (int ix = 0; ix < sw; ix++)
        {
            int dx = x + ix, dy = y + iy;
            if (dx < 0 || dx >= width || dy < 0 || dy >= height)
                continue;
            Color c = src.get_pixel_color(sx + ix, sy + iy);
            if (src.components == 4 || src.components == 2)
            {
                if (c.a() == 255)
                    set_pixel(static_cast<u32>(dx), static_cast<u32>(dy), c.r(), c.g(), c.b(),
                              c.a());
                else if (c.a() > 0)
                {
                    Color dd = get_pixel_color(dx, dy);
                    float al = c.a() / 255.0f;
                    set_pixel(static_cast<u32>(dx), static_cast<u32>(dy),
                              static_cast<u8>(c.r() * al + dd.r() * (1.0f - al)),
                              static_cast<u8>(c.g() * al + dd.g() * (1.0f - al)),
                              static_cast<u8>(c.b() * al + dd.b() * (1.0f - al)), dd.a());
                }
            }
            else
                set_pixel(static_cast<u32>(dx), static_cast<u32>(dy), c.r(), c.g(), c.b(), 255);
        }
}

void Pixmap::draw_pixmap_blended(const Pixmap& src, int x, int y, float opacity, BlendMode mode)
{
    draw_pixmap_blended(src, x, y, IntRect{0, 0, src.width, src.height}, opacity, mode);
}

void Pixmap::draw_pixmap_blended(const Pixmap& src, int x, int y, const IntRect& sr, float opacity,
                                 BlendMode mode)
{
    if (!pixels || !src.pixels)
        return;
    int sx = sr.x, sy = sr.y, sw = sr.width, sh = sr.height;
    if (sx < 0)
    {
        sw += sx;
        x -= sx;
        sx = 0;
    }
    if (sy < 0)
    {
        sh += sy;
        y -= sy;
        sy = 0;
    }
    if (sx + sw > src.width)
        sw = src.width - sx;
    if (sy + sh > src.height)
        sh = src.height - sy;

    for (int iy = 0; iy < sh; iy++)
        for (int ix = 0; ix < sw; ix++)
        {
            int dx = x + ix, dy = y + iy;
            if (dx < 0 || dx >= width || dy < 0 || dy >= height)
                continue;
            blend_pixel(static_cast<u32>(dx), static_cast<u32>(dy),
                        src.get_pixel_color(sx + ix, sy + iy), opacity, mode);
        }
}

// ── copy ──────────────────────────────────────────────────────────────────

void Pixmap::copy_region(const Pixmap& src, const IntRect& sr, int dx, int dy)
{
    if (!src.pixels || !pixels)
        return;
    if (src.components != components)
        return;

    int sx = sr.x, sy = sr.y, sw = sr.width, sh = sr.height;
    if (sx < 0)
    {
        sw += sx;
        dx -= sx;
        sx = 0;
    }
    if (sy < 0)
    {
        sh += sy;
        dy -= sy;
        sy = 0;
    }
    if (sx + sw > src.width)
        sw = src.width - sx;
    if (sy + sh > src.height)
        sh = src.height - sy;

    for (int y = 0; y < sh; y++)
        for (int x = 0; x < sw; x++)
        {
            int tx = dx + x, ty = dy + y;
            if (tx >= 0 && tx < width && ty >= 0 && ty < height)
            {
                Color c = src.get_pixel_color(sx + x, sy + y);
                set_pixel(static_cast<u32>(tx), static_cast<u32>(ty), c.r(), c.g(), c.b(), c.a());
            }
        }
}

// ── color ops ─────────────────────────────────────────────────────────────

void Pixmap::replace_color(const Color& from, const Color& to, float threshold)
{
    if (!pixels)
        return;
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            Color c = get_pixel_color(x, y);
            float dr = std::abs(static_cast<float>(c.r()) - static_cast<float>(from.r())) / 255.0f;
            float dg = std::abs(static_cast<float>(c.g()) - static_cast<float>(from.g())) / 255.0f;
            float db = std::abs(static_cast<float>(c.b()) - static_cast<float>(from.b())) / 255.0f;
            float da = std::abs(static_cast<float>(c.a()) - static_cast<float>(from.a())) / 255.0f;
            if ((dr + dg + db + da) / 4.0f <= threshold)
                set_pixel(static_cast<u32>(x), static_cast<u32>(y), to.r(), to.g(), to.b(), to.a());
        }
}

void Pixmap::set_color_key(const Color& key, float threshold)
{
    if (!pixels || (components != 4 && components != 2))
        return;
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            Color c = get_pixel_color(x, y);
            float dr = std::abs(static_cast<float>(c.r()) - static_cast<float>(key.r())) / 255.0f;
            float dg = std::abs(static_cast<float>(c.g()) - static_cast<float>(key.g())) / 255.0f;
            float db = std::abs(static_cast<float>(c.b()) - static_cast<float>(key.b())) / 255.0f;
            if ((dr + dg + db) / 3.0f <= threshold)
                set_pixel(static_cast<u32>(x), static_cast<u32>(y), c.r(), c.g(), c.b(), 0);
        }
}

// ── filters ───────────────────────────────────────────────────────────────

Pixmap* Pixmap::apply_blur(int radius) const
{
    if (!pixels || radius < 1)
        return nullptr;
    Pixmap* r = new Pixmap(width, height, components);
    int ks = radius * 2 + 1, ka = ks * ks;
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            int sr = 0, sg = 0, sb = 0, sa = 0;
            for (int ky = -radius; ky <= radius; ky++)
                for (int kx = -radius; kx <= radius; kx++)
                {
                    int px = std::max(0, std::min(width - 1, x + kx));
                    int py = std::max(0, std::min(height - 1, y + ky));
                    Color c = get_pixel_color(px, py);
                    sr += c.r();
                    sg += c.g();
                    sb += c.b();
                    sa += c.a();
                }
            r->set_pixel(static_cast<u32>(x), static_cast<u32>(y), static_cast<u8>(sr / ka),
                         static_cast<u8>(sg / ka), static_cast<u8>(sb / ka),
                         static_cast<u8>(sa / ka));
        }
    return r;
}

Pixmap* Pixmap::apply_gaussian_blur(int /*radius*/) const
{
    if (!pixels)
        return nullptr;
    Pixmap* r = new Pixmap(width, height, components);
    float k[3][3] = {{1 / 16.0f, 2 / 16.0f, 1 / 16.0f},
                     {2 / 16.0f, 4 / 16.0f, 2 / 16.0f},
                     {1 / 16.0f, 2 / 16.0f, 1 / 16.0f}};
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            float sr = 0, sg = 0, sb = 0, sa = 0;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++)
                {
                    int px = std::max(0, std::min(width - 1, x + kx)),
                        py = std::max(0, std::min(height - 1, y + ky));
                    Color c = get_pixel_color(px, py);
                    float w = k[ky + 1][kx + 1];
                    sr += c.r() * w;
                    sg += c.g() * w;
                    sb += c.b() * w;
                    sa += c.a() * w;
                }
            r->set_pixel(static_cast<u32>(x), static_cast<u32>(y), static_cast<u8>(sr),
                         static_cast<u8>(sg), static_cast<u8>(sb), static_cast<u8>(sa));
        }
    return r;
}

Pixmap* Pixmap::apply_sharpen() const
{
    if (!pixels)
        return nullptr;
    Pixmap* r = new Pixmap(width, height, components);
    float k[3][3] = {{0, -1, 0}, {-1, 5, -1}, {0, -1, 0}};
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            float sr = 0, sg = 0, sb = 0;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++)
                {
                    int px = std::max(0, std::min(width - 1, x + kx)),
                        py = std::max(0, std::min(height - 1, y + ky));
                    Color c = get_pixel_color(px, py);
                    float w = k[ky + 1][kx + 1];
                    sr += c.r() * w;
                    sg += c.g() * w;
                    sb += c.b() * w;
                }
            sr = std::max(0.0f, std::min(sr, 255.0f));
            sg = std::max(0.0f, std::min(sg, 255.0f));
            sb = std::max(0.0f, std::min(sb, 255.0f));
            Color o = get_pixel_color(x, y);
            r->set_pixel(static_cast<u32>(x), static_cast<u32>(y), static_cast<u8>(sr),
                         static_cast<u8>(sg), static_cast<u8>(sb), o.a());
        }
    return r;
}

Pixmap* Pixmap::apply_edge_detection() const
{
    if (!pixels)
        return nullptr;
    Pixmap* r = new Pixmap(width, height, components);
    float sx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    float sy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            float gx = 0, gy = 0;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++)
                {
                    int px = std::max(0, std::min(width - 1, x + kx)),
                        py = std::max(0, std::min(height - 1, y + ky));
                    Color c = get_pixel_color(px, py);
                    float g = c.r() * 0.299f + c.g() * 0.587f + c.b() * 0.114f;
                    gx += g * sx[ky + 1][kx + 1];
                    gy += g * sy[ky + 1][kx + 1];
                }
            float m = std::min(sqrtf(gx * gx + gy * gy), 255.0f);
            u8 e = static_cast<u8>(m);
            r->set_pixel(static_cast<u32>(x), static_cast<u32>(y), e, e, e, 255);
        }
    return r;
}

Pixmap* Pixmap::apply_emboss() const
{
    if (!pixels)
        return nullptr;
    Pixmap* r = new Pixmap(width, height, components);
    float k[3][3] = {{-2, -1, 0}, {-1, 1, 1}, {0, 1, 2}};
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            float sr = 0, sg = 0, sb = 0;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++)
                {
                    int px = std::max(0, std::min(width - 1, x + kx)),
                        py = std::max(0, std::min(height - 1, y + ky));
                    Color c = get_pixel_color(px, py);
                    float w = k[ky + 1][kx + 1];
                    sr += c.r() * w;
                    sg += c.g() * w;
                    sb += c.b() * w;
                }
            sr = std::max(0.0f, std::min(sr + 128.0f, 255.0f));
            sg = std::max(0.0f, std::min(sg + 128.0f, 255.0f));
            sb = std::max(0.0f, std::min(sb + 128.0f, 255.0f));
            Color o = get_pixel_color(x, y);
            r->set_pixel(static_cast<u32>(x), static_cast<u32>(y), static_cast<u8>(sr),
                         static_cast<u8>(sg), static_cast<u8>(sb), o.a());
        }
    return r;
}

// ── texture generation ───────────────────────────────────────────────────

Pixmap* Pixmap::generate_heightmap() const
{
    if (!pixels)
        return nullptr;
    Pixmap* r = new Pixmap(width, height, 1);
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            const Color c = get_pixel_color(x, y);
            const u8 gray = clamp_to_byte(static_cast<int>(
                c.r() * 0.299f + c.g() * 0.587f + c.b() * 0.114f));
            r->set_pixel(static_cast<u32>(x), static_cast<u32>(y), gray, gray, gray, 255);
        }
    return r;
}

Pixmap* Pixmap::generate_normal_map(float strength) const
{
    if (!pixels)
        return nullptr;
    Pixmap* r = new Pixmap(width, height, 4);
    // Same Sobel kernels apply_edge_detection() already uses - there the
    // gradient magnitude IS the output; here gx/gy are the surface slope
    // along each axis, which is what a normal actually needs.
    float sx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    float sy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            float gx = 0.0f, gy = 0.0f;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++)
                {
                    const int px = std::max(0, std::min(width - 1, x + kx));
                    const int py = std::max(0, std::min(height - 1, y + ky));
                    const Color c = get_pixel_color(px, py);
                    const float h = (c.r() * 0.299f + c.g() * 0.587f + c.b() * 0.114f) / 255.0f;
                    gx += h * sx[ky + 1][kx + 1];
                    gy += h * sy[ky + 1][kx + 1];
                }
            float nx = -gx * strength;
            float ny = -gy * strength;
            float nz = 1.0f;
            const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 0.00001f)
            {
                nx /= len;
                ny /= len;
                nz /= len;
            }
            const u8 nr = clamp_to_byte(static_cast<int>((nx * 0.5f + 0.5f) * 255.0f));
            const u8 ng = clamp_to_byte(static_cast<int>((ny * 0.5f + 0.5f) * 255.0f));
            const u8 nb = clamp_to_byte(static_cast<int>((nz * 0.5f + 0.5f) * 255.0f));
            r->set_pixel(static_cast<u32>(x), static_cast<u32>(y), nr, ng, nb, 255);
        }
    return r;
}

// ── crop ──────────────────────────────────────────────────────────────────

Pixmap* Pixmap::crop(const IntRect& rect) const
{
    if (!pixels)
        return nullptr;
    int x = rect.x, y = rect.y, w = rect.width, h = rect.height;
    if (x < 0)
    {
        w += x;
        x = 0;
    }
    if (y < 0)
    {
        h += y;
        y = 0;
    }
    if (x >= width || y >= height)
        return nullptr;
    if (x + w > width)
        w = width - x;
    if (y + h > height)
        h = height - y;
    if (w <= 0 || h <= 0)
        return nullptr;

    Pixmap* r = new Pixmap(w, h, components);
    if (!r->pixels)
    {
        delete r;
        return nullptr;
    }
    int bpr = w * components;
    for (int row = 0; row < h; row++)
        memcpy(r->pixels + row * bpr, pixels + ((y + row) * width + x) * components,
               static_cast<size_t>(bpr));
    return r;
}

Pixmap* Pixmap::crop(int x, int y, int w, int h) const
{
    IntRect r = {x, y, w, h};
    return crop(r);
}

Pixmap* Pixmap::crop_extended(const IntRect& rect, bool fill_transparent) const
{
    if (!pixels)
        return nullptr;
    int x = rect.x, y = rect.y, w = rect.width, h = rect.height;
    if (w <= 0 || h <= 0)
        return nullptr;

    Pixmap* r = new Pixmap(w, h, components);
    if (!r->pixels)
    {
        delete r;
        return nullptr;
    }

    if (fill_transparent && (components == 4 || components == 2))
        r->clear();
    else
        r->fill(static_cast<u8>(0), static_cast<u8>(0), static_cast<u8>(0), static_cast<u8>(255));

    int sx0 = std::max(0, x), sy0 = std::max(0, y);
    int sx1 = std::min(width, x + w), sy1 = std::min(height, y + h);
    if (sx0 >= sx1 || sy0 >= sy1)
        return r;

    int dox = sx0 - x, doy = sy0 - y;
    int cw = sx1 - sx0, ch = sy1 - sy0, bpr = cw * components;
    for (int row = 0; row < ch; row++)
        memcpy(r->pixels + ((doy + row) * w + dox) * components,
               pixels + ((sy0 + row) * width + sx0) * components, static_cast<size_t>(bpr));
    return r;
}

} // namespace Radion
