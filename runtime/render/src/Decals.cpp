#include "PCH.h"

#include "Decals.h"

#include "FileSystem.h"
#include "Log.h"
#include "Pixmap.h"

#include "Math.h"
#include <cmath>
#include <cstring>
#include "Math.h" // Math::rotation(from, to)

namespace Radion
{

namespace
{

f32 hash2(s32 x, s32 y, u32 seed)
{
    // Unsigned: the wraparound is intended.
    u32 h =
        static_cast<u32>(x) * 374761393u + static_cast<u32>(y) * 668265263u + seed * 2654435761u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return static_cast<f32>(h ^ (h >> 16)) / 4294967295.0f;
}

f32 valueNoise(f32 x, f32 y, u32 seed)
{
    const s32 xi = static_cast<s32>(std::floor(x));
    const s32 yi = static_cast<s32>(std::floor(y));
    const f32 xf = x - static_cast<f32>(xi);
    const f32 yf = y - static_cast<f32>(yi);
    const f32 u = xf * xf * (3.0f - 2.0f * xf);
    const f32 v = yf * yf * (3.0f - 2.0f * yf);

    const f32 a = hash2(xi, yi, seed);
    const f32 b = hash2(xi + 1, yi, seed);
    const f32 c = hash2(xi, yi + 1, seed);
    const f32 d = hash2(xi + 1, yi + 1, seed);
    return (a * (1 - u) + b * u) * (1 - v) + (c * (1 - u) + d * u) * v;
}

f32 fbm(f32 x, f32 y, u32 seed, s32 octaves = 4)
{
    f32 sum = 0.0f, amp = 0.5f, freq = 1.0f;
    for (s32 i = 0; i < octaves; ++i)
    {
        sum += valueNoise(x * freq, y * freq, seed + static_cast<u32>(i) * 977u) * amp;
        amp *= 0.5f;
        freq *= 2.0f;
    }
    return sum;
}

u8 toByte(f32 v)
{
    return static_cast<u8>(Math::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f);
}

// Bit-cast int to float like HLSL asfloat(); MakeProjection() uses it for the layer index.
f32 intAsFloat(s32 v)
{
    f32 f;
    std::memcpy(&f, &v, sizeof(f));
    return f;
}

} // namespace

bool DecalSystem::create(u32 textureDim, u32 maxLayers)
{
    mDim = textureDim;
    mMaxLayers = maxLayers;
    mLayerCount = 0;

    TextureDesc desc;
    desc.type = TextureType::Tex2DArray;
    desc.width = mDim;
    desc.height = mDim;
    desc.depth = mMaxLayers;
    desc.format = Format::RGBA8;
    desc.mips = 0; // full chain: ApplyDecals() samples with textureGrad
    desc.usage = TextureSampled;
    desc.debugName = "decals.albedo";
    mAlbedo = GPU::getSingleton().createTexture(desc);

    desc.debugName = "decals.normal";
    mNormal = GPU::getSingleton().createTexture(desc);

    desc.debugName = "decals.surface";
    mSurface = GPU::getSingleton().createTexture(desc);

    if (!mAlbedo.valid() || !mNormal.valid() || !mSurface.valid())
    {
        Log::error("DecalSystem: failed to create the texture arrays");
        return false;
    }

    Log::info("DecalSystem: %u layer(s) of %ux%u (3 maps: color, normal, surface)", mMaxLayers,
              mDim, mDim);
    return true;
}

void DecalSystem::shutdown()
{
    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mAlbedo);
    gpu.destroy(mNormal);
    gpu.destroy(mSurface);
    mDecals.clear();
    mLayerCount = 0;
}

s32 DecalSystem::reserveLayer()
{
    if (!mAlbedo.valid() || mLayerCount >= mMaxLayers)
    {
        Log::error("DecalSystem: out of layers (%u/%u)", mLayerCount, mMaxLayers);
        return -1;
    }
    return static_cast<s32>(mLayerCount++);
}

void DecalSystem::uploadLayer(u32 layer, const std::vector<u8>& albedo,
                              const std::vector<u8>& normal, const std::vector<u8>& surface)
{
    GPU& gpu = GPU::getSingleton();
    gpu.updateTexture(mAlbedo, 0, layer, 0, 0, mDim, mDim, albedo.data());
    gpu.updateTexture(mNormal, 0, layer, 0, 0, mDim, mDim, normal.data());
    gpu.updateTexture(mSurface, 0, layer, 0, 0, mDim, mDim, surface.data());
    gpu.generateMips(mAlbedo);
    gpu.generateMips(mNormal);
    gpu.generateMips(mSurface);
}

s32 DecalSystem::addProcedural(Procedural kind, u32 seed)
{
    const s32 layer = reserveLayer();
    if (layer < 0)
        return -1;

    const s32 n = static_cast<s32>(mDim);
    std::vector<f32> height(static_cast<usize>(n) * n, 0.0f);
    std::vector<f32> alpha(static_cast<usize>(n) * n, 0.0f);
    std::vector<Math::vec3> tint(static_cast<usize>(n) * n, Math::vec3(1.0f));
    std::vector<f32> rough(static_cast<usize>(n) * n, 0.8f);

    for (s32 y = 0; y < n; ++y)
    {
        for (s32 x = 0; x < n; ++x)
        {
            const usize i = static_cast<usize>(y) * n + x;
            // Centred coordinates: -1..1, r=1 at the circle's edge.
            const f32 u = (static_cast<f32>(x) + 0.5f) / static_cast<f32>(n) * 2.0f - 1.0f;
            const f32 v = (static_cast<f32>(y) + 0.5f) / static_cast<f32>(n) * 2.0f - 1.0f;
            const f32 r = std::sqrt(u * u + v * v);
            const f32 ang = std::atan2(v, u);

            switch (kind)
            {
            case Procedural::BulletHole:
            {
                // Irregular edge: radius varies with angle, else it reads as a perfect circle.
                const f32 wobble =
                    fbm(std::cos(ang) * 3.0f + 8.0f, std::sin(ang) * 3.0f + 8.0f, seed, 3);
                const f32 rEdge = 0.62f + (wobble - 0.5f) * 0.28f;
                const f32 rHole = rEdge * 0.42f;

                alpha[i] = 1.0f - Math::clamp((r - rEdge * 0.55f) / (rEdge * 0.45f), 0.0f, 1.0f);
                alpha[i] *= alpha[i];

                const f32 chips = fbm(std::cos(ang) * 9.0f, std::sin(ang) * 9.0f, seed + 31u, 2);
                if (r > rHole && r < rEdge)
                    alpha[i] = Math::clamp(alpha[i] + (chips - 0.45f) * 0.9f, 0.0f, 1.0f);

                if (r < rHole)
                    height[i] = -1.0f + (r / rHole) * 0.45f;
                else
                    height[i] = 0.55f * std::exp(-((r - rHole) * 6.0f) * ((r - rHole) * 6.0f));

                tint[i] = Math::vec3(0.10f + 0.35f * Math::clamp((r - rHole) /
                                                                    Math::max(0.01f, rEdge - rHole),
                                                                0.0f, 1.0f));
                rough[i] = 0.95f;
                break;
            }
            case Procedural::ScorchMark:
            {
                const f32 noise = fbm(u * 2.6f + 5.0f, v * 2.6f + 5.0f, seed, 5);
                const f32 rr = r * (0.75f + noise * 0.55f);
                f32 a = Math::clamp(1.0f - rr, 0.0f, 1.0f);
                alpha[i] = a * a * (3.0f - 2.0f * a);

                height[i] = 0.0f;
                tint[i] = Math::vec3(0.06f + 0.20f * noise);
                rough[i] = 0.98f;
                break;
            }
            case Procedural::Crack:
            {
                // Fissures: |noise-0.5| inverted gives continuous lines.
                const f32 noise = fbm(u * 3.2f, v * 3.2f, seed, 4);
                const f32 line = 1.0f - Math::clamp(std::fabs(noise - 0.5f) * 14.0f, 0.0f, 1.0f);
                const f32 fade = Math::clamp(1.0f - r, 0.0f, 1.0f);

                alpha[i] = Math::clamp(line * fade * 1.4f, 0.0f, 1.0f);
                height[i] = -alpha[i];
                tint[i] = Math::vec3(0.15f);
                rough[i] = 0.9f;
                break;
            }
            case Procedural::Blood:
            {
                const f32 wobble =
                    fbm(std::cos(ang) * 4.0f + 3.0f, std::sin(ang) * 4.0f + 3.0f, seed, 3);
                const f32 rEdge = 0.55f + (wobble - 0.5f) * 0.5f;
                f32 a = Math::clamp(1.0f - r / rEdge, 0.0f, 1.0f);
                a = a * a * (3.0f - 2.0f * a);

                const f32 speckle =
                    fbm(u * 10.0f + 20.0f, v * 10.0f + 20.0f, seed + 17u, 3);
                if (r >= rEdge && r < rEdge * 2.2f && speckle > 0.62f)
                    a = Math::max(a, (speckle - 0.62f) * 2.4f *
                                        Math::clamp(1.0f - (r - rEdge) / rEdge, 0.0f, 1.0f));

                alpha[i] = Math::clamp(a, 0.0f, 1.0f);
                height[i] = 0.0f;
                const f32 depth = Math::clamp(1.0f - r / Math::max(rEdge, 0.01f), 0.0f, 1.0f);
                tint[i] = Math::mix(Math::vec3(0.30f, 0.02f, 0.02f), Math::vec3(0.10f, 0.005f, 0.006f),
                                   depth);
                rough[i] = 0.35f;
                break;
            }
            }
        }
    }

    std::vector<u8> mapAlbedo(static_cast<usize>(n) * n * 4);
    std::vector<u8> mapNormal(static_cast<usize>(n) * n * 4);
    std::vector<u8> mapSurface(static_cast<usize>(n) * n * 4);

    for (s32 y = 0; y < n; ++y)
    {
        for (s32 x = 0; x < n; ++x)
        {
            const usize i = static_cast<usize>(y) * n + x;

            const s32 xm = (x > 0) ? x - 1 : x;
            const s32 xp = (x < n - 1) ? x + 1 : x;
            const s32 ym = (y > 0) ? y - 1 : y;
            const s32 yp = (y < n - 1) ? y + 1 : y;
            const f32 hL = height[static_cast<usize>(y) * n + xm];
            const f32 hR = height[static_cast<usize>(y) * n + xp];
            const f32 hD = height[static_cast<usize>(ym) * n + x];
            const f32 hU = height[static_cast<usize>(yp) * n + x];

            // Relief scale; without it normals come out nearly flat.
            const f32 scale = static_cast<f32>(n) * 0.02f;
            const Math::vec3 normal =
                Math::normalize(Math::vec3((hL - hR) * scale, (hD - hU) * scale, 1.0f));

            mapAlbedo[i * 4 + 0] = toByte(tint[i].x);
            mapAlbedo[i * 4 + 1] = toByte(tint[i].y);
            mapAlbedo[i * 4 + 2] = toByte(tint[i].z);
            mapAlbedo[i * 4 + 3] = toByte(alpha[i]);

            // Tangent-space in [0,1]; only RG is read, the shader reconstructs z.
            mapNormal[i * 4 + 0] = toByte(normal.x * 0.5f + 0.5f);
            mapNormal[i * 4 + 1] = toByte(normal.y * 0.5f + 0.5f);
            mapNormal[i * 4 + 2] = toByte(normal.z * 0.5f + 0.5f);
            mapNormal[i * 4 + 3] = 255;

            mapSurface[i * 4 + 0] = toByte(rough[i]);
            mapSurface[i * 4 + 1] = 0;
            mapSurface[i * 4 + 2] = toByte(0.5f);
            mapSurface[i * 4 + 3] = 255;
        }
    }

    uploadLayer(static_cast<u32>(layer), mapAlbedo, mapNormal, mapSurface);

    static const char* names[] = {"bullet hole", "scorch mark", "crack", "blood"};
    Log::info("DecalSystem: layer %d generated: %s", layer, names[static_cast<s32>(kind)]);
    return layer;
}

s32 DecalSystem::addFromFiles(const std::string& albedoPath, const std::string& normalPath,
                              const std::string& surfacePath)
{
    const s32 layer = reserveLayer();
    if (layer < 0)
        return -1;

    const usize bytes = static_cast<usize>(mDim) * mDim * 4;
    std::vector<u8> mapAlbedo(bytes, 255);
    std::vector<u8> mapNormal(bytes, 0);
    std::vector<u8> mapSurface(bytes, 0);
    for (usize i = 0; i < bytes; i += 4)
    {
        mapNormal[i + 0] = 128;
        mapNormal[i + 1] = 128;
        mapNormal[i + 2] = 255;
        mapNormal[i + 3] = 255;
        mapSurface[i + 0] = 200;
        mapSurface[i + 1] = 0;
        mapSurface[i + 2] = 128;
        mapSurface[i + 3] = 255;
    }

    struct Source
    {
        const std::string* path;
        std::vector<u8>* destination;
    };
    const Source sources[] = {
        {&albedoPath, &mapAlbedo}, {&normalPath, &mapNormal}, {&surfacePath, &mapSurface}};

    FileSystem& files = FileSystem::getSingleton();
    for (const Source& source : sources)
    {
        if (source.path->empty())
            continue;

        ByteArray fileBytes = files.readBinary(*source.path);
        Pixmap pixmap;
        if (fileBytes.empty() || fileBytes.size() > 0xFFFFFFFFu ||
            !pixmap.load_from_memory(fileBytes.data(), static_cast<u32>(fileBytes.size())))
        {
            Log::error("DecalSystem: failed to open '%s'", source.path->c_str());
            continue;
        }
        Pixmap* converted = pixmap.components == 3 ? pixmap.convert_to_rgba() : nullptr;
        const Pixmap& image = converted ? *converted : pixmap;

        // Every layer in the array has to share the array's own dimensions;
        for (u32 y = 0; y < mDim; ++y)
        {
            const u32 sy = (static_cast<u32>(image.height) == mDim)
                               ? y
                               : (y * static_cast<u32>(image.height)) / mDim;
            for (u32 x = 0; x < mDim; ++x)
            {
                const u32 sx = (static_cast<u32>(image.width) == mDim)
                                   ? x
                                   : (x * static_cast<u32>(image.width)) / mDim;
                const u8* p =
                    image.pixels + (static_cast<usize>(sy) * image.width + sx) * image.components;
                u8* d = source.destination->data() + (static_cast<usize>(y) * mDim + x) * 4;
                if (image.components >= 4)
                {
                    d[0] = p[0];
                    d[1] = p[1];
                    d[2] = p[2];
                    d[3] = p[3];
                }
                else
                {
                    d[0] = p[0];
                    d[1] = image.components >= 2 ? p[1] : p[0];
                    d[2] = image.components >= 3 ? p[2] : p[0];
                    d[3] = 255;
                }
            }
        }
        delete converted;
    }

    uploadLayer(static_cast<u32>(layer), mapAlbedo, mapNormal, mapSurface);
    Log::info("DecalSystem: layer %d loaded from '%s'", layer, albedoPath.c_str());
    return layer;
}

s32 DecalSystem::addDecal(const Decal& decal)
{
    // Lighting::submitDecals() shares RenderList::MaxLights with scene lights; unbounded decals would starve them.
    constexpr usize kMaxDecals = 200;
    if (mDecals.size() >= kMaxDecals)
        mDecals.erase(mDecals.begin());
    mDecals.push_back(decal);
    return static_cast<s32>(mDecals.size()) - 1;
}

Math::mat4 DecalSystem::makeProjection(const Decal& decal)
{
    // Box -> world, inverted; half-extent is 0.5 since the local box runs -1..1.
    const Math::mat4 boxToWorld = Math::translate(Math::mat4(1.0f), decal.position) *
                                 Math::mat4_cast(decal.rotation) *
                                 Math::scale(Math::mat4(1.0f), decal.size * 0.5f);

    Math::mat4 projection = Math::inverse(boxToWorld);

    // Layer index rides in row 3 of the matrix: dead space the shader never reads.
    projection[0][3] = intAsFloat(decal.layer);
    projection[1][3] = 0.0f;
    projection[2][3] = 0.0f;
    projection[3][3] = 1.0f;
    return projection;
}

s32 DecalSystem::placeOnSurface(const Math::vec3& position, const Math::vec3& normal, s32 layer,
                                f32 size, f32 thickness, f32 rotationRadians,
                                const Math::vec3& color, f32 opacity)
{
    Decal decal;
    // Rotates box +Z onto the surface normal (the slope fade compares against +Z).
    const Math::quat aligned = Math::rotation(Math::vec3(0.0f, 0.0f, 1.0f), Math::normalize(normal));
    decal.rotation = aligned * Math::angleAxis(rotationRadians, Math::vec3(0.0f, 0.0f, 1.0f));

    // Box is centred ON the surface so edge fade is 1 there; pulled back, the surface would sit on the cap at fade 0.
    decal.position = position;

    decal.size = Math::vec3(size, size, thickness);
    decal.layer = layer;
    decal.color = color;
    decal.opacity = opacity;
    decal.slopePower = 8.0f;
    decal.normalStrength = 1.0f;
    return addDecal(decal);
}

} // namespace Radion
