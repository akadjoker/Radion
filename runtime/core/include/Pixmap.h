#ifndef RADION_PIXMAP_H
#define RADION_PIXMAP_H

#include "Color.h"
#include "Math.h"
#include "Types.h"

namespace Radion
{

class Pixmap
{
public:
    enum class BlendMode
    {
        copy,
        alpha,
        add,
        multiply
    };

    Pixmap();
    ~Pixmap();
    Pixmap(int w, int h, int components);
    Pixmap(int w, int h, int components, unsigned char* data);
    Pixmap(const Pixmap& image, const IntRect& crop);
    Pixmap(const Pixmap& other) = delete;
    Pixmap& operator=(const Pixmap& other) = delete;

    void set_pixel(u32 x, u32 y, u8 r, u8 g, u8 b, u8 a);
    void set_pixel(u32 x, u32 y, u32 rgba);
    u32 get_pixel(u32 x, u32 y) const;
    Color get_pixel_color(u32 x, u32 y) const;

    void fill(u8 r, u8 g, u8 b, u8 a);
    void fill(u32 rgba);
    void clear();

    bool save(const char* file_name);
    bool load(const char* file_name);
    bool load_from_memory(const unsigned char* buffer, u32 bytesRead);

    void flip_vertical();
    void flip_horizontal();
    void tint(u8 r, u8 g, u8 b);

    Pixmap* convert_to_rgba() const;
    Pixmap* resize(int newWidth, int newHeight) const;
    Pixmap* crop(const IntRect& rect) const;
    Pixmap* crop(int x, int y, int w, int h) const;
    Pixmap* crop_extended(const IntRect& rect, bool fill_transparent = true) const;

    void draw_line(int x1, int y1, int x2, int y2, const Color& color);
    void draw_rect(int x, int y, int w, int h, const Color& color, bool fill = false);
    void draw_circle(int cx, int cy, int radius, const Color& color, bool fill = false);
    void draw_pixmap(const Pixmap& source, int x, int y);
    void draw_pixmap(const Pixmap& source, int x, int y, const IntRect& src_rect);
    void blend_pixel(u32 x, u32 y, const Color& color, float opacity = 1.0f,
                     BlendMode mode = BlendMode::alpha);
    void draw_pixmap_blended(const Pixmap& source, int x, int y, float opacity = 1.0f,
                             BlendMode mode = BlendMode::alpha);
    void draw_pixmap_blended(const Pixmap& source, int x, int y, const IntRect& src_rect,
                             float opacity = 1.0f, BlendMode mode = BlendMode::alpha);

    void copy_region(const Pixmap& source, const IntRect& src_rect, int dst_x, int dst_y);

    void replace_color(const Color& from, const Color& to, float threshold = 0.0f);
    void set_color_key(const Color& key, float threshold = 0.0f);

    Pixmap* apply_blur(int radius) const;
    Pixmap* apply_gaussian_blur(int radius) const;
    Pixmap* apply_sharpen() const;
    Pixmap* apply_edge_detection() const;
    Pixmap* apply_emboss() const;

    // Texture generation: reads `this` as a colour or grayscale source.
    // Luminance is treated as height (0 = low, 255 = high).
    Pixmap* generate_heightmap() const;
    // Sobel height gradient packed as a tangent-space normal (n*0.5+0.5). Higher `strength` = stronger bumps; 1.0-4.0 typical.
    Pixmap* generate_normal_map(float strength = 2.0f) const;

    bool is_valid() const
    {
        return pixels != nullptr;
    }
    int get_size() const
    {
        return width * height * components;
    }
    bool has_alpha() const
    {
        return components == 2 || components == 4;
    }

    unsigned char* pixels;
    int components;
    int width;
    int height;
};

} // namespace Radion

#endif // RADION_PIXMAP_H
