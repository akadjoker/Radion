#include "PCH.h"

#include "Decals.h"

#include "FileSystem.h"
#include "Log.h"
#include "Pixmap.h"

#include <glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <cmath>
#include <cstring>
#include <glm/gtx/quaternion.hpp> // glm::rotation(from, to)

namespace Radion
{

namespace
{

// ------------------------------------------------------- procedural noise

f32 hash2(s32 x, s32 y, u32 seed)
{
    // Unsigned throughout: the wraparound is the point, and doing it in a
    // signed type would be overflow, not hashing.
    u32 h =
        static_cast<u32>(x) * 374761393u + static_cast<u32>(y) * 668265263u + seed * 2654435761u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return static_cast<f32>(h ^ (h >> 16)) / 4294967295.0f;
}

// Value noise with smooth interpolation. A decal mask only needs irregularity
// at the edge, not Perlin's extra cost.
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
    return static_cast<u8>(glm::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f);
}

// Packs an int into a float's bit pattern, the way HLSL's asfloat()/asint()
// do. Same trick MakeProjection() uses to smuggle the layer index through a
// dead matrix component.
f32 intAsFloat(s32 v)
{
    f32 f;
    std::memcpy(&f, &v, sizeof(f));
    return f;
}

} // namespace

// --------------------------------------------------------------- creation

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

// ---------------------------------------------------------- procedural fields
//
// Each generator produces a HEIGHT field and an ALPHA field, and the three
// maps come out of those. Normals come from central differences on the
// height, the same as baking a normal map from a heightmap without ever
// leaving the loop.

s32 DecalSystem::addProcedural(Procedural kind, u32 seed)
{
    const s32 layer = reserveLayer();
    if (layer < 0)
        return -1;

    const s32 n = static_cast<s32>(mDim);
    std::vector<f32> height(static_cast<usize>(n) * n, 0.0f);
    std::vector<f32> alpha(static_cast<usize>(n) * n, 0.0f);
    // RGB, not a single darkening scalar - every prior kind still only ever
    // wrote the same value into all three channels (grayscale), Blood is the
    // first that needs an actual hue.
    std::vector<Math::Vec3> tint(static_cast<usize>(n) * n, Math::Vec3(1.0f));
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
                // Irregular edge: the effective radius varies with angle, or
                // it reads as a perfect circle - a plastic sticker, not damage.
                const f32 wobble =
                    fbm(std::cos(ang) * 3.0f + 8.0f, std::sin(ang) * 3.0f + 8.0f, seed, 3);
                const f32 rEdge = 0.62f + (wobble - 0.5f) * 0.28f;
                const f32 rHole = rEdge * 0.42f;

                alpha[i] = 1.0f - glm::clamp((r - rEdge * 0.55f) / (rEdge * 0.45f), 0.0f, 1.0f);
                alpha[i] *= alpha[i]; // tighter edge

                // Radial chips flying off the hole.
                const f32 chips = fbm(std::cos(ang) * 9.0f, std::sin(ang) * 9.0f, seed + 31u, 2);
                if (r > rHole && r < rEdge)
                    alpha[i] = glm::clamp(alpha[i] + (chips - 0.45f) * 0.9f, 0.0f, 1.0f);

                // Crater: deep floor, raised rim.
                if (r < rHole)
                    height[i] = -1.0f + (r / rHole) * 0.45f;
                else
                    height[i] = 0.55f * std::exp(-((r - rHole) * 6.0f) * ((r - rHole) * 6.0f));

                tint[i] = Math::Vec3(0.10f + 0.35f * glm::clamp((r - rHole) /
                                                                    glm::max(0.01f, rEdge - rHole),
                                                                0.0f, 1.0f));
                rough[i] = 0.95f;
                break;
            }
            case Procedural::ScorchMark:
            {
                // Burn: soft smudge, no relief, very rough.
                const f32 noise = fbm(u * 2.6f + 5.0f, v * 2.6f + 5.0f, seed, 5);
                const f32 rr = r * (0.75f + noise * 0.55f);
                f32 a = glm::clamp(1.0f - rr, 0.0f, 1.0f);
                alpha[i] = a * a * (3.0f - 2.0f * a); // smoothstep

                height[i] = 0.0f;
                tint[i] = Math::Vec3(0.06f + 0.20f * noise);
                rough[i] = 0.98f;
                break;
            }
            case Procedural::Crack:
            {
                // Fissures: bands where the noise passes near zero. Taking
                // |noise-0.5| and inverting gives continuous lines instead of
                // blobs, and they branch on their own.
                const f32 noise = fbm(u * 3.2f, v * 3.2f, seed, 4);
                const f32 line = 1.0f - glm::clamp(std::fabs(noise - 0.5f) * 14.0f, 0.0f, 1.0f);
                const f32 fade = glm::clamp(1.0f - r, 0.0f, 1.0f);

                alpha[i] = glm::clamp(line * fade * 1.4f, 0.0f, 1.0f);
                height[i] = -alpha[i]; // V-shaped groove
                tint[i] = Math::Vec3(0.15f);
                rough[i] = 0.9f;
                break;
            }
            case Procedural::Blood:
            {
                // A central pool, irregular edge like BulletHole's, plus a
                // few satellite droplets scattered by a second, higher-
                // frequency field just outside it - a splash, not a circle.
                const f32 wobble =
                    fbm(std::cos(ang) * 4.0f + 3.0f, std::sin(ang) * 4.0f + 3.0f, seed, 3);
                const f32 rEdge = 0.55f + (wobble - 0.5f) * 0.5f;
                f32 a = glm::clamp(1.0f - r / rEdge, 0.0f, 1.0f);
                a = a * a * (3.0f - 2.0f * a); // smoothstep, soft pool edge

                const f32 speckle =
                    fbm(u * 10.0f + 20.0f, v * 10.0f + 20.0f, seed + 17u, 3);
                if (r >= rEdge && r < rEdge * 2.2f && speckle > 0.62f)
                    a = glm::max(a, (speckle - 0.62f) * 2.4f *
                                        glm::clamp(1.0f - (r - rEdge) / rEdge, 0.0f, 1.0f));

                alpha[i] = glm::clamp(a, 0.0f, 1.0f);
                height[i] = 0.0f; // wet, not relief
                // Darker where it pools deepest, at the centre.
                const f32 depth = glm::clamp(1.0f - r / glm::max(rEdge, 0.01f), 0.0f, 1.0f);
                tint[i] = glm::mix(Math::Vec3(0.30f, 0.02f, 0.02f), Math::Vec3(0.10f, 0.005f, 0.006f),
                                   depth);
                rough[i] = 0.35f; // wet sheen, unlike the others' matte damage
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

            // Relief scale. Without it neighbouring texels differ by almost
            // nothing and the normals come out nearly flat.
            const f32 scale = static_cast<f32>(n) * 0.02f;
            const Math::Vec3 normal =
                glm::normalize(Math::Vec3((hL - hR) * scale, (hD - hU) * scale, 1.0f));

            mapAlbedo[i * 4 + 0] = toByte(tint[i].r);
            mapAlbedo[i * 4 + 1] = toByte(tint[i].g);
            mapAlbedo[i * 4 + 2] = toByte(tint[i].b);
            mapAlbedo[i * 4 + 3] = toByte(alpha[i]);

            // Tangent-space in [0,1]. Only RG is read: the shader puts z=1
            // back in itself.
            mapNormal[i * 4 + 0] = toByte(normal.x * 0.5f + 0.5f);
            mapNormal[i * 4 + 1] = toByte(normal.y * 0.5f + 0.5f);
            mapNormal[i * 4 + 2] = toByte(normal.z * 0.5f + 0.5f);
            mapNormal[i * 4 + 3] = 255;

            mapSurface[i * 4 + 0] = toByte(rough[i]); // roughness
            mapSurface[i * 4 + 1] = 0;                // metallic
            mapSurface[i * 4 + 2] = toByte(0.5f);     // reflectance
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
    // Neutral normal (0,0,1) and a plausible surface, for whichever map is
    // missing.
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
        // nearest-neighbour is enough since decal art already comes in at
        // roughly the right size.
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
    // Lighting::submitDecals() shares RenderList::MaxLights (256) with the
    // scene's actual lights and stops submitting once that fills, oldest
    // decal first - so with nothing ever removed, a long session's early
    // decals (a hundred bullet holes from a shooting range that never
    // stops) permanently squat every slot and every decal placed after that
    // point renders precisely nowhere, forever, with no error to see. Kept
    // well under 256 so real lights still have room in the same budget.
    // Erasing the front is O(n), but n stays at kMaxDecals, not the whole
    // session's decal count - a few hundred Math::Vec3/quat moves is nothing
    // next to the GPU upload right after this.
    constexpr usize kMaxDecals = 200;
    if (mDecals.size() >= kMaxDecals)
        mDecals.erase(mDecals.begin());
    mDecals.push_back(decal);
    return static_cast<s32>(mDecals.size()) - 1;
}

// --------------------------------------------------------------- projection

Math::Mat4 DecalSystem::makeProjection(const Decal& decal)
{
    // Box -> world, then inverted. The 0.5 is because the local box runs
    // -1..1: a decal 2 units wide has a half-extent of 1.
    const Math::Mat4 boxToWorld = glm::translate(Math::Mat4(1.0f), decal.position) *
                                 glm::mat4_cast(decal.rotation) *
                                 glm::scale(Math::Mat4(1.0f), decal.size * 0.5f);

    Math::Mat4 projection = glm::inverse(boxToWorld);

    // The layer index travels in the 4th ROW of the matrix.
    //
    // Why there and not a field of its own: the transform only ever uses rows
    // 0..2 (the shader stops at the result's .xyz), so row 3 is dead space
    // that is already in place - it reaches the shader with the matrix, at no
    // extra cost. In glm, projection[c][r] is column c, row r, same as GLSL,
    // so writing projection[c][3] only touches the result's .w, which nothing
    // reads.
    projection[0][3] = intAsFloat(decal.layer);
    projection[1][3] = 0.0f;
    projection[2][3] = 0.0f;
    projection[3][3] = 1.0f;
    return projection;
}

s32 DecalSystem::placeOnSurface(const Math::Vec3& position, const Math::Vec3& normal, s32 layer,
                                f32 size, f32 thickness, f32 rotationRadians,
                                const Math::Vec3& color, f32 opacity)
{
    Decal decal;
    // Rotates the box's +Z onto the surface normal - it is +Z the slope fade
    // compares the fragment's normal against.
    const Math::Quaternion aligned = glm::rotation(Math::Vec3(0.0f, 0.0f, 1.0f), glm::normalize(normal));
    // Spin around that same normal, so repeated stamps of the same texture do
    // not read as copies.
    decal.rotation = aligned * glm::angleAxis(rotationRadians, Math::Vec3(0.0f, 0.0f, 1.0f));

    // The box is centred ON the surface, not pulled back. Centred, the
    // surface lands at box.z = 0, where the edge fade (1-|z|^8) is 1, and the
    // box still reaches half its thickness to either side - the slack that
    // covers uneven ground and geometry slightly in front or behind. Pulled
    // back by half the thickness instead, the surface would sit at box.z = 1,
    // the box's own cap, where the edge fade is exactly 0 - invisible on
    // flat ground, only partly visible on anything curved.
    decal.position = position;

    decal.size = Math::Vec3(size, size, thickness);
    decal.layer = layer;
    decal.color = color;
    decal.opacity = opacity;
    decal.slopePower = 8.0f;
    decal.normalStrength = 1.0f;
    return addDecal(decal);
}

} // namespace Radion
