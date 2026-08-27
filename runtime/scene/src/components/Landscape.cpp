#include "PCH.h"

#include "Landscape.h"

#include "AssetManager.h"
#include "GameObject.h"
#include "MaterialManager.h"
#include "Noise.h"
#include "RenderList.h"
#include "Timer.h"

namespace Radion
{

namespace
{

f32 inverseLerp(f32 a, f32 b, f32 v)
{
    return (b - a) == 0.0f ? 0.0f : (v - a) / (b - a);
}

void blendHeight(Landscape::Modifier::Blend blend, f32 weight, f32& height, f32 value)
{
    switch (blend)
    {
    default:
    case Landscape::Modifier::Blend::Normal:
        height = Math::mix(height, value, weight);
        break;
    case Landscape::Modifier::Blend::Multiply:
        height *= value * weight;
        break;
    case Landscape::Modifier::Blend::Additive:
        height += value * weight;
        break;
    }
}

f32 sampleHeightmapPixel(const std::vector<u8>& data, u32 width, s32 px, s32 py)
{
    const usize i = static_cast<usize>(px) + static_cast<usize>(py) * width;
    return static_cast<f32>(data[i]) / 255.0f;
}

// One vertex's non-position attributes, interleaved: normal, uv, weights.
// 36 bytes, matching the stride landscape.vert's stream 1 declares. Built as
// raw floats rather than a struct so nothing here depends on how a compiler
// happens to pad Math::vec4 - the GPU reads exactly the bytes this writes.
constexpr u32 kAttribFloats = 3 + 2 + 4;

// ---------------------------------------------------------------------------
// The shared index buffer: crack-free LOD in two sentences.
//
// A chunk is Landscape::ChunkWidth x ChunkWidth vertices, ALWAYS, at every
// LOD - the outer ring never changes density, only the interior is walked
// with a step of (1 << lod). That is what makes the shared edge between two
// chunks at different LODs always have the same vertex count on both sides,
// so it always coincides - no cracks, no skirts, no per-neighbour-LOD index
// variants.
//
// Built once, for the whole world: switching a chunk's LOD is switching which
// range of THIS buffer its one SubMesh points at. Zero uploads, zero copies.
// ---------------------------------------------------------------------------
struct LandscapeLod
{
    u32 indexOffset = 0;
    u32 indexCount = 0;
};

struct LandscapeIndices
{
    BufferHandle buffer;
    std::vector<LandscapeLod> lods;
    VertexLayout layout;

    static LandscapeIndices& instance()
    {
        static LandscapeIndices state = build();
        return state;
    }

private:
    static LandscapeIndices build()
    {
        LandscapeIndices state;

        constexpr s32 width = static_cast<s32>(Landscape::ChunkWidth);
        // log2(64) + 1 = 7: one LOD per halving of the interior's resolution,
        // down to the coarsest step that still fits inside the interior.
        const s32 maxLod = static_cast<s32>(std::log2(width - 3)) + 1;

        std::vector<u32> indices;
        state.lods.resize(static_cast<usize>(maxLod));

        for (s32 lod = 0; lod < maxLod; ++lod)
        {
            state.lods[static_cast<usize>(lod)].indexOffset = static_cast<u32>(indices.size());

            if (lod == 0)
            {
                // LOD 0: the full grid, no distinction between border and
                // interior.
                for (s32 x = 0; x < width - 1; ++x)
                {
                    for (s32 z = 0; z < width - 1; ++z)
                    {
                        const s32 lowerLeft = x + z * width;
                        const s32 lowerRight = (x + 1) + z * width;
                        const s32 topLeft = x + (z + 1) * width;
                        const s32 topRight = (x + 1) + (z + 1) * width;

                        indices.push_back(static_cast<u32>(topLeft));
                        indices.push_back(static_cast<u32>(lowerLeft));
                        indices.push_back(static_cast<u32>(lowerRight));
                        indices.push_back(static_cast<u32>(topLeft));
                        indices.push_back(static_cast<u32>(lowerRight));
                        indices.push_back(static_cast<u32>(topRight));
                    }
                }
            }
            else
            {
                const s32 step = 1 << lod;

                // ---- interior, stepped ----
                // 1 .. width-2: index 0 and width-1 are the border ring and
                // are handled separately below.
                for (s32 x = 1; x < width - 2; x += step)
                {
                    for (s32 z = 1; z < width - 2; z += step)
                    {
                        const s32 lowerLeft = x + z * width;
                        const s32 lowerRight = (x + step) + z * width;
                        const s32 topLeft = x + (z + step) * width;
                        const s32 topRight = (x + step) + (z + step) * width;

                        indices.push_back(static_cast<u32>(topLeft));
                        indices.push_back(static_cast<u32>(lowerLeft));
                        indices.push_back(static_cast<u32>(lowerRight));
                        indices.push_back(static_cast<u32>(topLeft));
                        indices.push_back(static_cast<u32>(lowerRight));
                        indices.push_back(static_cast<u32>(topRight));
                    }
                }

                // ---- the four borders, WITHOUT the step ----
                // 'connection' is the nearest interior vertex: dividing by
                // step rounds the border column to its matching interior
                // column, and the "+1"/"(step+1)/2" make that rounding land
                // in the middle instead of always the same side.
                auto fillTriangle = [&](s32 midStep, s32 current, s32 neighbour, s32 connection,
                                        s32 connectionPrev, bool flip)
                {
                    if (flip)
                    {
                        indices.push_back(static_cast<u32>(current));
                        indices.push_back(static_cast<u32>(connection));
                        indices.push_back(static_cast<u32>(neighbour));
                    }
                    else
                    {
                        indices.push_back(static_cast<u32>(current));
                        indices.push_back(static_cast<u32>(neighbour));
                        indices.push_back(static_cast<u32>(connection));
                    }
                    if (midStep == step / 2)
                    {
                        if (flip)
                        {
                            indices.push_back(static_cast<u32>(current));
                            indices.push_back(static_cast<u32>(connectionPrev));
                            indices.push_back(static_cast<u32>(connection));
                        }
                        else
                        {
                            indices.push_back(static_cast<u32>(current));
                            indices.push_back(static_cast<u32>(connection));
                            indices.push_back(static_cast<u32>(connectionPrev));
                        }
                    }
                };

                // bottom (z = 0)
                for (s32 x = 0; x < width - 1; ++x)
                {
                    const s32 z = 0;
                    const s32 current = x + z * width;
                    const s32 neighbour = x + 1 + z * width;
                    const s32 connection =
                        1 + ((x + (step + 1) / 2 - 1) / step) * step + (z + 1) * width;
                    const s32 connectionPrev =
                        1 + (((x - 1) + (step + 1) / 2 - 1) / step) * step + (z + 1) * width;
                    fillTriangle((x - 1) % step, current, neighbour, connection, connectionPrev,
                                false);
                }
                // top (z = width-1) - winding flipped: this side faces the
                // other way.
                for (s32 x = 0; x < width - 1; ++x)
                {
                    const s32 z = width - 1;
                    const s32 current = x + z * width;
                    const s32 neighbour = x + 1 + z * width;
                    const s32 connection =
                        1 + ((x + (step + 1) / 2 - 1) / step) * step + (z - 1) * width;
                    const s32 connectionPrev =
                        1 + (((x - 1) + (step + 1) / 2 - 1) / step) * step + (z - 1) * width;
                    fillTriangle((x - 1) % step, current, neighbour, connection, connectionPrev,
                                true);
                }
                // left (x = 0)
                for (s32 z = 0; z < width - 1; ++z)
                {
                    const s32 x = 0;
                    const s32 current = x + z * width;
                    const s32 neighbour = x + (z + 1) * width;
                    const s32 connection =
                        x + 1 + (((z + (step + 1) / 2 - 1) / step) * step + 1) * width;
                    const s32 connectionPrev =
                        x + 1 + ((((z - 1) + (step + 1) / 2 - 1) / step) * step + 1) * width;
                    fillTriangle((z - 1) % step, current, neighbour, connection, connectionPrev,
                                true);
                }
                // right (x = width-1)
                for (s32 z = 0; z < width - 1; ++z)
                {
                    const s32 x = width - 1;
                    const s32 current = x + z * width;
                    const s32 neighbour = x + (z + 1) * width;
                    const s32 connection =
                        x - 1 + (((z + (step + 1) / 2 - 1) / step) * step + 1) * width;
                    const s32 connectionPrev =
                        x - 1 + ((((z - 1) + (step + 1) / 2 - 1) / step) * step + 1) * width;
                    fillTriangle((z - 1) % step, current, neighbour, connection, connectionPrev,
                                false);
                }
            }

            state.lods[static_cast<usize>(lod)].indexCount =
                static_cast<u32>(indices.size()) - state.lods[static_cast<usize>(lod)].indexOffset;
        }

        // ---- flip triangle winding ----
        // Everything above is index for index the same algorithm as the
        // reference. What differs is the front-face convention, and that does
        // not come from the algorithm - it comes from the rasteriser state
        // the reference's engine defaults to (clockwise front faces), where
        // OpenGL defaults to counter-clockwise. Porting the index order as-is
        // draws the terrain inside-out.
        //
        // The alternative would be glFrontFace(GL_CW) around every terrain
        // draw - a literal translation of that state - but that is three call
        // sites (cascades, prepass, scene) and three places the state could
        // leak into the rest of the frame. Swapping the last two indices of
        // every triangle once, here, has no state at all. Normals need no
        // equivalent fix: they come from a cross product computed separately
        // in buildChunk() and already point up.
        for (usize i = 0; i + 2 < indices.size(); i += 3)
            std::swap(indices[i + 1], indices[i + 2]);

        GPU& gpu = GPU::getSingleton();
        BufferDesc desc;
        desc.size = indices.size() * sizeof(u32);
        desc.usage = BufferIndex;
        desc.residency = Residency::Static;
        desc.stride = sizeof(u32);
        desc.data = indices.data();
        desc.debugName = "landscape.indices";
        state.buffer = gpu.createBuffer(desc);

        state.layout.streamCount = 2;
        state.layout.streams[0].stride = sizeof(Math::vec3);
        state.layout.streams[1].stride = kAttribFloats * sizeof(f32);
        state.layout.attribCount = 4;
        state.layout.attribs[0] = {0, 0, 0, AttribFormat::Float3}; // position
        state.layout.attribs[1] = {1, 1, 0, AttribFormat::Float3}; // normal
        state.layout.attribs[2] = {2, 1, 12, AttribFormat::Float2}; // uv
        state.layout.attribs[3] = {3, 1, 20, AttribFormat::Float4}; // weights

        Log::info("Landscape: %d LODs, %zu shared indices (%zu KB)", maxLod, indices.size(),
                  indices.size() * sizeof(u32) / 1024);
        for (usize i = 0; i < state.lods.size(); ++i)
            Log::info("Landscape:   LOD %zu: %u triangles", i, state.lods[i].indexCount / 3);

        return state;
    }
};

} // namespace

Landscape::Landscape() : Component(Type)
{
    mMaterial.flags |= MaterialLit | MaterialLandscape;
}

void Landscape::onDestroy()
{
    for (auto& entry : mChunks)
        releaseChunk(entry.second);
    mChunks.clear();
}

Material& Landscape::material()
{
    return mMaterial;
}

const Material& Landscape::material() const
{
    return mMaterial;
}

void Landscape::restart()
{
    for (auto& entry : mChunks)
        releaseChunk(entry.second);
    mChunks.clear();
    mPriorityInvalidation.clear();

    // The SAME seed for every modifier, not a distinct one derived per
    // modifier. An earlier version of this used an LCG to spread them out,
    // and that broke Voronoi outright: at a seed in the billions, a float
    // only has steps of a few hundred at that magnitude, so the variation in
    // compute_sin(seed * hash) - where hash lives in [0,1] - disappears
    // before it reaches the sine at all. One seed for everyone keeps it in a
    // range where the perturbation still perturbs.
    for (Modifier& modifier : modifiers)
    {
        modifier.seed = config.seed;
        modifier.perlin.initialise(config.seed);
    }

    mSeeded = true;
}

f32 Landscape::sampleHeight(const Math::vec2& worldPosition) const
{
    // Height accumulates in [0,1] through the modifier stack and only goes
    // to world units at the very end.
    f32 height = 0.0f;
    for (const Modifier& modifier : modifiers)
    {
        if (!modifier.enabled)
            continue;

        switch (modifier.kind)
        {
        case Modifier::Kind::Perlin:
        {
            const Math::vec2 p = worldPosition * modifier.frequency;
            const f32 value =
                modifier.perlin.compute(p.x, p.y, 0.0f, modifier.octaves) * 0.5f + 0.5f;
            blendHeight(modifier.blend, modifier.weight, height, value);
            break;
        }
        case Modifier::Kind::Voronoi:
        {
            Math::vec2 p = worldPosition * modifier.frequency;
            // The perturbation displaces the sample with Perlin BEFORE
            // Voronoi evaluates it. Without this the cells look geometric and
            // give themselves away as Voronoi immediately; with it the ridges
            // meander.
            if (modifier.perturbation > 0.0f)
            {
                const f32 angle = modifier.perlin.compute(p.x, p.y, 0.0f, 6) * 6.283185307f;
                p.x += std::sin(angle) * modifier.perturbation;
                p.y += std::cos(angle) * modifier.perturbation;
            }
            const Noise::Voronoi::Result result =
                Noise::Voronoi::compute(p.x, p.y, static_cast<f32>(modifier.seed));
            const f32 value = std::pow(
                1.0f - Math::clamp((result.distance - modifier.shape) * modifier.fade, 0.0f, 1.0f),
                Math::max(0.0001f, modifier.falloff));
            blendHeight(modifier.blend, modifier.weight, height, value);
            break;
        }
        case Modifier::Kind::Heightmap:
        {
            if (modifier.heightmapData.empty() || modifier.heightmapWidth == 0 ||
                modifier.heightmapHeight == 0)
                break;

            const Math::vec2 p = worldPosition * modifier.frequency;
            const Math::vec2 pixel(p.x + static_cast<f32>(modifier.heightmapWidth) * 0.5f,
                                  p.y + static_cast<f32>(modifier.heightmapHeight) * 0.5f);

            // Outside the image: the modifier does nothing. That is what lets
            // an authored island sit in the middle of an otherwise infinite
            // procedural world - the Perlin carries on alone past the image's
            // edge with no seam, because nothing here ever touches height at
            // all outside it.
            if (pixel.x < 0.0f || pixel.x >= static_cast<f32>(modifier.heightmapWidth) ||
                pixel.y < 0.0f || pixel.y >= static_cast<f32>(modifier.heightmapHeight))
                break;

            const f32 fx = pixel.x - std::floor(pixel.x);
            const f32 fy = pixel.y - std::floor(pixel.y);
            const s32 x0 = static_cast<s32>(pixel.x);
            const s32 y0 = static_cast<s32>(pixel.y);
            const s32 x1 = Math::min(x0 + 1, static_cast<s32>(modifier.heightmapWidth) - 1);
            const s32 y1 = Math::min(y0 + 1, static_cast<s32>(modifier.heightmapHeight) - 1);

            const f32 a = Math::mix(sampleHeightmapPixel(modifier.heightmapData, modifier.heightmapWidth, x0, y0),
                                   sampleHeightmapPixel(modifier.heightmapData, modifier.heightmapWidth, x1, y0), fx);
            const f32 b = Math::mix(sampleHeightmapPixel(modifier.heightmapData, modifier.heightmapWidth, x0, y1),
                                   sampleHeightmapPixel(modifier.heightmapData, modifier.heightmapWidth, x1, y1), fx);
            const f32 value = Math::mix(a, b, fy);

            // ---- fade out at the border ----
            // Without this the image's edge is a STEP: height comes from the
            // image with weight 0.85 just inside, and from the noise alone
            // just outside, and the two values have no reason to agree - a
            // vertical wall all the way round the image. The band lets the
            // image's influence fall to zero before the edge, so the
            // procedural takes over seamlessly. borderFadePixels at 0
            // restores the un-faded behaviour.
            f32 influence = 1.0f;
            if (modifier.borderFadePixels > 0.0f)
            {
                const f32 dx = Math::min(pixel.x,
                                        static_cast<f32>(modifier.heightmapWidth - 1) - pixel.x);
                const f32 dy = Math::min(pixel.y,
                                        static_cast<f32>(modifier.heightmapHeight - 1) - pixel.y);
                const f32 d = Math::clamp(Math::min(dx, dy) / modifier.borderFadePixels, 0.0f, 1.0f);
                influence = d * d * (3.0f - 2.0f * d); // smoothstep
            }
            if (influence <= 0.0f)
                break;

            // Can't use the shared blendHeight(): it reads the modifier's
            // plain weight, and this needs the weight ATTENUATED by the fade
            // band. Same three blend modes, with that weight in its place.
            const f32 v = value * modifier.heightmapAmount;
            const f32 w = modifier.weight * influence;
            switch (modifier.blend)
            {
            default:
            case Modifier::Blend::Normal:
                height = Math::mix(height, v, w);
                break;
            case Modifier::Blend::Multiply:
                height *= v * w;
                break;
            case Modifier::Blend::Additive:
                height += v * w;
                break;
            }
            break;
        }
        }
    }
    return Math::mix(config.bottomLevel, config.topLevel, height);
}

bool Landscape::buildChunk(const ChunkKey& key, Chunk& chunk)
{
    const LandscapeIndices& indices = LandscapeIndices::instance();
    if (!indices.buffer.valid())
        return false;

    const f32 scale = config.chunkScale;
    const f32 chunkSpan = static_cast<f32>(ChunkWidth - 1) * scale;
    chunk.position = Math::vec3(static_cast<f32>(key.x) * chunkSpan, 0.0f,
                               static_cast<f32>(key.z) * chunkSpan);

    // Height grid WITH PADDING: 68x68 instead of 67x67. Normals need the
    // neighbour one step to the right and one step up, and without the
    // padding that means re-evaluating the noise (expensive) or inventing a
    // normal at the border (visible as a seam between chunks).
    constexpr u32 paddedWidth = ChunkWidth + 1;
    std::vector<f32> padded(static_cast<usize>(paddedWidth) * paddedWidth);
    for (u32 cz = 0; cz < paddedWidth; ++cz)
    {
        for (u32 cx = 0; cx < paddedWidth; ++cx)
        {
            const f32 x = (static_cast<f32>(cx) - ChunkHalfWidth) * scale;
            const f32 z = (static_cast<f32>(cz) - ChunkHalfWidth) * scale;
            padded[cx + cz * paddedWidth] =
                sampleHeight(Math::vec2(chunk.position.x + x, chunk.position.z + z));
        }
    }

    std::vector<Math::vec3> positions(VertexCount);
    std::vector<f32> attribs(static_cast<usize>(VertexCount) * kAttribFloats);
    chunk.heights.resize(VertexCount);

    f32 minY = 1e30f;
    f32 maxY = -1e30f;
    bool slopeCastShadow = false;

    for (u32 cz = 0; cz < ChunkWidth; ++cz)
    {
        for (u32 cx = 0; cx < ChunkWidth; ++cx)
        {
            const u32 index = cx + cz * ChunkWidth;
            const f32 x = (static_cast<f32>(cx) - ChunkHalfWidth) * scale;
            const f32 z = (static_cast<f32>(cz) - ChunkHalfWidth) * scale;
            const f32 height = padded[cx + cz * paddedWidth];

            // Normal and tangent from the two neighbours. Real spacing
            // (`scale`) rather than a literal +1, so this stays correct when
            // chunkScale is not 1 - with chunkScale = 1 the two agree anyway.
            const Math::vec3 c0(x, height, z);
            const Math::vec3 c1(x + scale, padded[(cx + 1) + cz * paddedWidth], z);
            const Math::vec3 c2(x, padded[cx + (cz + 1) * paddedWidth], z + scale);

            const Math::vec3 t = c1 - c2;
            const Math::vec3 b = c0 - c1;
            const Math::vec3 n = Math::normalize(Math::cross(t, b));

            const f32 slopeAmount = 1.0f - Math::clamp(n.y, 0.0f, 1.0f);
            // One sloped vertex is enough to put the whole chunk in the
            // shadow view: flat ground never shadows itself, so flat chunks
            // stay out of it.
            if (slopeAmount > 0.1f)
                slopeCastShadow = true;

            const f32 regionBase = 1.0f; // always 1: the floor under everything
            const f32 regionSlope = config.slopeThreshold == 0.0f
                                        ? 1.0f
                                        : Math::smoothstep(0.0f, config.slopeThreshold, slopeAmount);
            f32 regionLow =
                config.lowAltitudeThreshold == 0.0f
                    ? 1.0f
                    : Math::smoothstep(0.0f, config.lowAltitudeThreshold,
                                      inverseLerp(0.0f, config.bottomLevel, height));
            f32 regionHigh =
                config.highAltitudeThreshold == 0.0f
                    ? 1.0f
                    : Math::smoothstep(0.0f, config.highAltitudeThreshold,
                                      inverseLerp(0.0f, config.topLevel, height));

            // Slope SUBTRACTS from the altitude regions. Without this, snow
            // climbs vertical cliff faces - the classic procedural-terrain
            // tell. A 300m cliff gets rock, not snow.
            regionLow = Math::clamp(regionLow - regionSlope, 0.0f, 1.0f);
            regionHigh = Math::clamp(regionHigh - regionSlope, 0.0f, 1.0f);

            positions[index] = Math::vec3(x, height, z);

            f32* attrib = &attribs[static_cast<usize>(index) * kAttribFloats];
            attrib[0] = n.x;
            attrib[1] = n.y;
            attrib[2] = n.z;
            attrib[3] = static_cast<f32>(cx) / static_cast<f32>(ChunkWidth - 1);
            attrib[4] = static_cast<f32>(cz) / static_cast<f32>(ChunkWidth - 1);
            attrib[5] = regionBase;
            attrib[6] = regionSlope;
            attrib[7] = regionLow;
            attrib[8] = regionHigh;

            chunk.heights[index] = height;
            minY = Math::min(minY, height);
            maxY = Math::max(maxY, height);
        }
    }

    chunk.castShadow = slopeCastShadow;
    chunk.minimumY = minY;
    chunk.maximumY = maxY;

    // Bounding sphere, in the chunk's OWN local space - see the note on
    // Landscape's owner transform in cull(). Reference formula: a box's
    // circumscribed sphere from its half-extents.
    chunk.sphereCentre = chunk.position + Math::vec3(0.0f, (minY + maxY) * 0.5f, 0.0f);
    const f32 halfSpan = chunkSpan * 0.5f;
    const f32 halfY = (maxY - minY) * 0.5f;
    chunk.sphereRadius = std::sqrt(2.0f * halfSpan * halfSpan + halfY * halfY);

    releaseChunk(chunk);

    GPU& gpu = GPU::getSingleton();
    BufferDesc positionDesc;
    positionDesc.size = positions.size() * sizeof(Math::vec3);
    positionDesc.usage = BufferVertex;
    positionDesc.residency = Residency::Static;
    positionDesc.stride = sizeof(Math::vec3);
    positionDesc.data = positions.data();
    positionDesc.debugName = "landscape.chunk.position";
    const BufferHandle positionBuffer = gpu.createBuffer(positionDesc);

    BufferDesc attribDesc;
    attribDesc.size = attribs.size() * sizeof(f32);
    attribDesc.usage = BufferVertex;
    attribDesc.residency = Residency::Static;
    attribDesc.stride = kAttribFloats * sizeof(f32);
    attribDesc.data = attribs.data();
    attribDesc.debugName = "landscape.chunk.attrib";
    const BufferHandle attribBuffer = gpu.createBuffer(attribDesc);

    if (!positionBuffer.valid() || !attribBuffer.valid())
    {
        gpu.destroy(positionBuffer);
        gpu.destroy(attribBuffer);
        return false;
    }

    Mesh mesh;
    mesh.positionBuffer = positionBuffer;
    mesh.attribBuffer = attribBuffer;
    mesh.indexBuffer = indices.buffer;
    mesh.indexType = IndexType::U32;
    mesh.ownsIndexBuffer = false; // the shared buffer outlives every chunk
    mesh.colorLayout = indices.layout;
    mesh.depthLayout = indices.layout;
    mesh.vertexCount = VertexCount;
    mesh.indexCount = indices.lods[0].indexCount;

    SubMesh submesh;
    submesh.indexOffset = indices.lods[0].indexOffset;
    submesh.indexCount = indices.lods[0].indexCount;
    submesh.materialSlot = 0;
    for (const Math::vec3& p : positions)
        submesh.bounds.expand(p);
    mesh.bounds = submesh.bounds;
    mesh.submeshes = {submesh};

    chunk.mesh = Assets().adoptMesh(mesh);
    chunk.built = chunk.mesh.valid();
    return chunk.built;
}

void Landscape::releaseChunk(Chunk& chunk)
{
    if (chunk.mesh.valid())
        Assets().destroyMesh(chunk.mesh);
    chunk.mesh = MeshHandle();
    chunk.built = false;
}

bool Landscape::buildChunkWithinBudget(const ChunkKey& key, Timer& buildTimer)
{
    Chunk& chunk = mChunks[key];
    if (chunk.built && !chunk.invalidated)
        return true;
    buildTimer.tick();
    if (static_cast<f32>(buildTimer.getElapsedTime() * 1000.0) > config.generationBudgetMs)
        return false;
    buildChunk(key, chunk);
    chunk.invalidated = false;
    ++mBuiltLastFrame;
    return true;
}

bool Landscape::requestChunk(s32 offsetX, s32 offsetZ, Timer& buildTimer)
{
    return buildChunkWithinBudget(ChunkKey{mCentreChunk.x + offsetX, mCentreChunk.z + offsetZ},
                                  buildTimer);
}

void Landscape::update(const Math::vec3& cameraPosition)
{
    if (!mSeeded)
        restart();

    const f32 chunkSpan = static_cast<f32>(ChunkWidth - 1) * config.chunkScale;

    if (config.centreToCamera)
    {
        mCentreChunk.x = static_cast<s32>(
            std::floor((cameraPosition.x + ChunkHalfWidth * config.chunkScale) / chunkSpan));
        mCentreChunk.z = static_cast<s32>(
            std::floor((cameraPosition.z + ChunkHalfWidth * config.chunkScale) / chunkSpan));
    }

    // Remove chunks that fell outside the radius. Chebyshev distance, not
    // Euclidean: the spiral grows in SQUARE rings, and Euclidean would trim a
    // disc out of it and flicker the corners in and out every frame.
    if (config.removal)
    {
        for (auto it = mChunks.begin(); it != mChunks.end();)
        {
            const s32 distance = Math::max(std::abs(mCentreChunk.x - it->first.x),
                                          std::abs(mCentreChunk.z - it->first.z));
            if (distance > static_cast<s32>(config.generationRadius) + 1)
            {
                releaseChunk(it->second);
                it = mChunks.erase(it);
            }
            else
                ++it;
        }
    }

    // Spiral generation, with a time budget.
    Timer buildTimer;
    mBuiltLastFrame = 0;

    // Invalidation queue, BEFORE the spiral: an edit has to show up now, not
    // once the whole world has finished generating. It is a QUEUE and not a
    // list because it may not fit the budget - next frame picks up the older
    // requests before the newer ones.
    bool budgetSpent = false;
    while (!mPriorityInvalidation.empty())
    {
        const ChunkKey key = mPriorityInvalidation.front();
        auto it = mChunks.find(key);
        // Can appear more than once in the queue; skip if already rebuilt.
        if (it == mChunks.end() || !it->second.invalidated)
        {
            mPriorityInvalidation.pop_front();
            continue;
        }
        if (!buildChunkWithinBudget(key, buildTimer))
        {
            budgetSpent = true;
            break;
        }
        mPriorityInvalidation.pop_front();
    }

    // Square spiral out from the centre.
    if (!budgetSpent && requestChunk(0, 0, buildTimer))
    {
        for (u32 growth = 0; growth < config.generationRadius; ++growth)
        {
            const s32 side = 2 * (static_cast<s32>(growth) + 1);
            s32 x = -static_cast<s32>(growth) - 1;
            s32 z = -static_cast<s32>(growth) - 1;
            bool budgetOut = false;
            for (s32 i = 0; i < side && !budgetOut; ++i)
            {
                if (!requestChunk(x, z, buildTimer))
                    budgetOut = true;
                ++x;
            }
            for (s32 i = 0; i < side && !budgetOut; ++i)
            {
                if (!requestChunk(x, z, buildTimer))
                    budgetOut = true;
                ++z;
            }
            for (s32 i = 0; i < side && !budgetOut; ++i)
            {
                if (!requestChunk(x, z, buildTimer))
                    budgetOut = true;
                --x;
            }
            for (s32 i = 0; i < side && !budgetOut; ++i)
            {
                if (!requestChunk(x, z, buildTimer))
                    budgetOut = true;
                --z;
            }
            if (budgetOut)
                break;
        }
    }

    buildTimer.tick();
    mLastBuildMilliseconds = static_cast<f32>(buildTimer.getElapsedTime() * 1000.0);
}

void Landscape::cull(const Frustum& frustum)
{
    // Chunk spheres are tested in the LOCAL space they were built in - the
    // same simplifying assumption Terrain already makes for its own patch
    // culling in this engine: a terrain sits at its owner's origin and is not
    // expected to be moved, rotated or scaled. RenderList::submit() still
    // applies the real transform for the final safety-net test, so moving the
    // GameObject cannot make anything draw wrong - only this pass's own
    // visible/LOD bookkeeping would be measuring from the wrong place.
    mVisible = 0;
    mShadowChunks = 0;
    for (auto& entry : mChunks)
    {
        Chunk& chunk = entry.second;
        if (!chunk.built)
        {
            chunk.visible = false;
            continue;
        }

        chunk.visible = frustum.intersects(Sphere{chunk.sphereCentre, chunk.sphereRadius});
        if (chunk.visible)
            ++mVisible;
        if (chunk.castShadow)
            ++mShadowChunks;
    }
}

u32 Landscape::pickLod(const Chunk& chunk, const Math::vec3& cameraPosition) const
{
    const LandscapeIndices& indices = LandscapeIndices::instance();
    const s32 maxLod = static_cast<s32>(indices.lods.size()) - 1;
    const f32 chunkSpan = static_cast<f32>(ChunkWidth - 1) * config.chunkScale;

    // Distance to the chunk's sphere, in "chunk widths". A chunk one width
    // away sits at LOD 0, two away at LOD 1, four away at LOD 2: double the
    // distance halves the density, which is what keeps a triangle's size on
    // screen roughly constant.
    const f32 distance =
        Math::max(0.0f, Math::distance(cameraPosition, chunk.sphereCentre) - chunk.sphereRadius);
    const f32 ratio = distance / chunkSpan;
    const s32 lod =
        static_cast<s32>(std::floor(std::log2(Math::max(1.0f, ratio)))) + config.lodBias;
    return static_cast<u32>(Math::clamp(lod, 0, maxLod));
}

void Landscape::submitCamera(RenderList& list, const Math::mat4& transform,
                             const Math::vec3& cameraPosition)
{
    const LandscapeIndices& indices = LandscapeIndices::instance();
    if (!indices.buffer.valid() || mChunks.empty())
        return;

    AssetManager& assets = Assets();
    MaterialManager& materials = MaterialManager::getSingleton();
    materials.resolvePipeline(mMaterial, indices.layout);
    materials.sync(mMaterial);

    for (u32 i = 0; i < 8; ++i)
        mLodHistogram[i] = 0;

    u32 triangles = 0;
    for (auto& entry : mChunks)
    {
        Chunk& chunk = entry.second;
        if (!chunk.built || !chunk.visible)
            continue;

        Mesh* mesh = assets.getMesh(chunk.mesh);
        if (!mesh || mesh->submeshes.empty())
            continue;

        // Switching LOD is switching the submesh's index range - the one
        // thing the shared index buffer buys. No upload, no buffer swap, no
        // rebuilt geometry.
        const u32 lod = pickLod(chunk, cameraPosition);
        chunk.lastLod = lod;
        if (lod < 8)
            ++mLodHistogram[lod];

        const LandscapeLod& range = indices.lods[lod];
        mesh->submeshes[0].indexOffset = range.indexOffset;
        mesh->submeshes[0].indexCount = range.indexCount;

        const Math::mat4 model = Math::translate(transform, chunk.position);
        list.submit(chunk.mesh, *mesh, model, &mMaterial, 1);
        triangles += range.indexCount / 3;
    }
    mTriangles = triangles;
}

void Landscape::submitShadow(RenderList& list, const Math::mat4& transform,
                             const Math::vec3& cameraPosition)
{
    const LandscapeIndices& indices = LandscapeIndices::instance();
    if (!indices.buffer.valid() || mChunks.empty())
        return;

    AssetManager& assets = Assets();
    const s32 maxLod = static_cast<s32>(indices.lods.size()) - 1;

    for (auto& entry : mChunks)
    {
        Chunk& chunk = entry.second;
        // Only chunks with slope enter the shadow view: a flat chunk cannot
        // shadow itself, and whatever stands on top of it is a different
        // object with its own cast-shadow flag.
        if (!chunk.built || !chunk.castShadow)
            continue;

        Mesh* mesh = assets.getMesh(chunk.mesh);
        if (!mesh || mesh->submeshes.empty())
            continue;

        // One LOD coarser than the scene view uses: the shadow map has no
        // resolution to show the difference, and this halves the triangles.
        const u32 lod =
            static_cast<u32>(Math::clamp(static_cast<s32>(pickLod(chunk, cameraPosition)) + 1, 0,
                                        maxLod));
        const LandscapeLod& range = indices.lods[lod];
        mesh->submeshes[0].indexOffset = range.indexOffset;
        mesh->submeshes[0].indexCount = range.indexCount;

        const Math::mat4 model = Math::translate(transform, chunk.position);
        list.submit(chunk.mesh, *mesh, model, &mMaterial, 1);
    }
}

void Landscape::invalidateRegion(const Math::vec2& centreXZ, f32 radius)
{
    for (auto& entry : mChunks)
    {
        Chunk& chunk = entry.second;
        if (chunk.invalidated)
            continue; // already queued

        // A VERTICAL cylinder, not a sphere: the chunk's height is exactly
        // what is about to change, so it cannot be part of the test.
        const Math::vec2 centre(chunk.sphereCentre.x, chunk.sphereCentre.z);
        const f32 reach = radius + chunk.sphereRadius;
        if (Math::dot(centre - centreXZ, centre - centreXZ) > reach * reach)
            continue;

        chunk.invalidated = true;
        mPriorityInvalidation.push_back(entry.first);
    }
}

void Landscape::invalidateAll()
{
    for (auto& entry : mChunks)
    {
        if (entry.second.invalidated)
            continue;
        entry.second.invalidated = true;
        mPriorityInvalidation.push_back(entry.first);
    }
}

u32 Landscape::pendingInvalidations() const
{
    return static_cast<u32>(mPriorityInvalidation.size());
}

f32 Landscape::heightAt(f32 worldX, f32 worldZ) const
{
    // Evaluates the modifier stack directly rather than reading a built
    // chunk's height grid - more expensive, but it works anywhere, even
    // where no chunk has been generated yet, which is what placing objects
    // needs.
    return sampleHeight(Math::vec2(worldX, worldZ));
}

bool Landscape::sculptRaise(const Math::vec3& worldCenter, f32 radius, f32 amount)
{
    return sculpt(worldCenter, radius, amount, false);
}

bool Landscape::sculptLower(const Math::vec3& worldCenter, f32 radius, f32 amount)
{
    return sculpt(worldCenter, radius, -amount, false);
}

bool Landscape::sculptSmooth(const Math::vec3& worldCenter, f32 radius, f32 strength)
{
    return sculpt(worldCenter, radius, Math::clamp(strength, 0.0f, 1.0f), true);
}

bool Landscape::sculpt(const Math::vec3& worldCenter, f32 radius, f32 amount, bool smoothing)
{
    if (radius <= 0.0f || amount == 0.0f)
        return false;

    Modifier* target = nullptr;
    for (Modifier& modifier : modifiers)
    {
        if (modifier.kind == Modifier::Kind::Heightmap && !modifier.heightmapData.empty())
        {
            target = &modifier;
            break;
        }
    }
    if (!target)
        return false;

    // Same pixel conversion sampleHeight() uses for this modifier - keeps
    // the brush aligned with what it is actually painting.
    const Math::vec2 centerPixel(
        worldCenter.x * target->frequency + static_cast<f32>(target->heightmapWidth) * 0.5f,
        worldCenter.z * target->frequency + static_cast<f32>(target->heightmapHeight) * 0.5f);
    const f32 pixelRadius = radius * target->frequency;
    if (pixelRadius <= 0.0f)
        return false;

    const s32 width = static_cast<s32>(target->heightmapWidth);
    const s32 height = static_cast<s32>(target->heightmapHeight);
    const s32 minX = Math::clamp(static_cast<s32>(std::floor(centerPixel.x - pixelRadius)), 0, width - 1);
    const s32 maxX = Math::clamp(static_cast<s32>(std::ceil(centerPixel.x + pixelRadius)), 0, width - 1);
    const s32 minZ = Math::clamp(static_cast<s32>(std::floor(centerPixel.y - pixelRadius)), 0, height - 1);
    const s32 maxZ = Math::clamp(static_cast<s32>(std::ceil(centerPixel.y + pixelRadius)), 0, height - 1);

    std::vector<u8> source;
    if (smoothing)
        source = target->heightmapData;

    bool changed = false;
    for (s32 z = minZ; z <= maxZ; ++z)
    {
        for (s32 x = minX; x <= maxX; ++x)
        {
            const f32 distance =
                Math::length(Math::vec2(static_cast<f32>(x), static_cast<f32>(z)) - centerPixel);
            if (distance > pixelRadius)
                continue;
            const f32 falloff = 1.0f - Math::smoothstep(0.0f, pixelRadius, distance);
            const usize index = static_cast<usize>(x) + static_cast<usize>(z) * target->heightmapWidth;

            f32 value = static_cast<f32>(target->heightmapData[index]) / 255.0f;
            if (smoothing)
            {
                f32 average = 0.0f;
                u32 samples = 0;
                for (s32 dz = -1; dz <= 1; ++dz)
                {
                    for (s32 dx = -1; dx <= 1; ++dx)
                    {
                        const s32 sx = Math::clamp(x + dx, 0, width - 1);
                        const s32 sz = Math::clamp(z + dz, 0, height - 1);
                        const usize sourceIndex =
                            static_cast<usize>(sx) + static_cast<usize>(sz) * target->heightmapWidth;
                        average += static_cast<f32>(source[sourceIndex]) / 255.0f;
                        ++samples;
                    }
                }
                const f32 original = static_cast<f32>(source[index]) / 255.0f;
                value = Math::mix(original, average / static_cast<f32>(samples), amount * falloff);
            }
            else
            {
                value += amount * falloff;
            }

            target->heightmapData[index] =
                static_cast<u8>(Math::clamp(value, 0.0f, 1.0f) * 255.0f);
            changed = true;
        }
    }
    if (!changed)
        return false;

    invalidateRegion(Math::vec2(worldCenter.x, worldCenter.z), radius);
    return true;
}

bool Landscape::sculptFlatten(const Math::vec3& worldCenter, f32 radius, f32 targetHeight,
                              f32 strength)
{
    if (radius <= 0.0f || strength <= 0.0f)
        return false;

    Modifier* target = nullptr;
    for (Modifier& modifier : modifiers)
    {
        if (modifier.kind == Modifier::Kind::Heightmap && !modifier.heightmapData.empty())
        {
            target = &modifier;
            break;
        }
    }
    if (!target)
        return false;

    const Math::vec2 centerPixel(
        worldCenter.x * target->frequency + static_cast<f32>(target->heightmapWidth) * 0.5f,
        worldCenter.z * target->frequency + static_cast<f32>(target->heightmapHeight) * 0.5f);
    const f32 pixelRadius = radius * target->frequency;
    if (pixelRadius <= 0.0f)
        return false;

    const s32 width = static_cast<s32>(target->heightmapWidth);
    const s32 height = static_cast<s32>(target->heightmapHeight);
    const s32 minX = Math::clamp(static_cast<s32>(std::floor(centerPixel.x - pixelRadius)), 0, width - 1);
    const s32 maxX = Math::clamp(static_cast<s32>(std::ceil(centerPixel.x + pixelRadius)), 0, width - 1);
    const s32 minZ = Math::clamp(static_cast<s32>(std::floor(centerPixel.y - pixelRadius)), 0, height - 1);
    const s32 maxZ = Math::clamp(static_cast<s32>(std::ceil(centerPixel.y + pixelRadius)), 0, height - 1);

    const f32 targetNormalised = Math::clamp(targetHeight, 0.0f, 1.0f);
    bool changed = false;
    for (s32 z = minZ; z <= maxZ; ++z)
    {
        for (s32 x = minX; x <= maxX; ++x)
        {
            const f32 distance =
                Math::length(Math::vec2(static_cast<f32>(x), static_cast<f32>(z)) - centerPixel);
            if (distance > pixelRadius)
                continue;
            const f32 falloff = 1.0f - Math::smoothstep(0.0f, pixelRadius, distance);
            const usize index = static_cast<usize>(x) + static_cast<usize>(z) * target->heightmapWidth;

            const f32 value = static_cast<f32>(target->heightmapData[index]) / 255.0f;
            const f32 mixed =
                Math::mix(value, targetNormalised, Math::clamp(strength * falloff, 0.0f, 1.0f));
            target->heightmapData[index] = static_cast<u8>(Math::clamp(mixed, 0.0f, 1.0f) * 255.0f);
            changed = true;
        }
    }
    if (!changed)
        return false;

    invalidateRegion(Math::vec2(worldCenter.x, worldCenter.z), radius);
    return true;
}

u32 Landscape::chunkCount() const
{
    return static_cast<u32>(mChunks.size());
}

u32 Landscape::visibleChunkCount() const
{
    return mVisible;
}

u32 Landscape::shadowChunkCount() const
{
    return mShadowChunks;
}

u32 Landscape::triangleCount() const
{
    return mTriangles;
}

u32 Landscape::builtLastFrame() const
{
    return mBuiltLastFrame;
}

f32 Landscape::lastBuildMilliseconds() const
{
    return mLastBuildMilliseconds;
}

u32 Landscape::lodHistogram(u32 lod) const
{
    return lod < 8 ? mLodHistogram[lod] : 0;
}

u32 Landscape::lodCount() const
{
    return static_cast<u32>(LandscapeIndices::instance().lods.size());
}

} // namespace Radion
