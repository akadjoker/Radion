#include "PCH.h"

#include "MaterialManager.h"

#include "AssetManager.h"
#include "FileSystem.h"
#include "MaterialParser.inl"
#include "MaterialSlotNames.h"
#include "Math.h"
#include "Pixmap.h"

#include <chrono>
#include <filesystem>
#include <sstream>

namespace Radion
{

ColorSpace Material::colorSpaceFor(MaterialSlot slot)
{
    switch (slot)
    {
    case SlotAlbedo:
    case SlotEmissive:
        return ColorSpace::sRGB;
    default:
        return ColorSpace::Linear;
    }
}

ColorSpace Material::colorSpaceFor(MaterialSlot slot, u32 flags)
{
    if (flags & MaterialTerrain)
    {
        // Terrain reuses these data-oriented slots as colour layers. The
        // splat map remains linear because its RGBA values are weights.
        if (slot == SlotAlbedo || slot == SlotNormal || slot == SlotSurface ||
            slot == SlotDetail || slot == SlotHeight)
            return ColorSpace::sRGB;
    }
    return colorSpaceFor(slot);
}

namespace
{

const char* kUnlitVertexFile = "shaders/unlit.vert";
const char* kUnlitFragmentFile = "shaders/unlit.frag";
const char* kLitVertexFile = "shaders/lit.vert";
const char* kLandscapeVertexFile = "shaders/landscape.vert";
const char* kLitFragmentFile = "shaders/lit.frag";
const char* kWaterVertexFile = "shaders/water.vert";
const char* kWaterFragmentFile = "shaders/water.frag";

HashMap<std::string, std::vector<MaterialDefinition>> gMaterialSources;

std::string materialSourceKey(const std::string& filename)
{
    const std::string resolved = FileSystem::getSingleton().resolve(filename);
    const std::filesystem::path path = resolved.empty() ? std::filesystem::path(filename)
                                                         : std::filesystem::path(resolved);
    return path.lexically_normal().string();
}

std::string directoryOf(const std::string& filename)
{
    const usize slash = filename.find_last_of("/\\");
    return slash == std::string::npos ? std::string() : filename.substr(0, slash + 1);
}

std::string resolveMaterialPath(const std::string& materialFile, const std::string& assetFile)
{
    if (assetFile.empty() || FileSystem::getSingleton().exists(assetFile))
        return assetFile;
    return directoryOf(materialFile) + assetFile;
}

// Loads every frame of a Sequence texture into one Tex2DArray, same shape
// Decals.cpp uses for its arrays. All frames must share one size - a
// mismatched frame fails rather than resampling it and desyncing the loop.
TextureHandle loadSequenceTexture(const std::string& materialFile,
                                  const std::vector<std::string>& frames, ColorSpace space)
{
    if (frames.empty())
        return TextureHandle();

    FileSystem& files = FileSystem::getSingleton();
    std::vector<Pixmap*> images;
    images.reserve(frames.size());
    u32 width = 0;
    u32 height = 0;
    bool ok = true;

    for (const std::string& frameFile : frames)
    {
        const std::string path = resolveMaterialPath(materialFile, frameFile);
        ByteArray bytes = files.readBinary(path);
        Pixmap* pixmap = new Pixmap();
        if (bytes.empty() || bytes.size() > 0xFFFFFFFFu ||
            !pixmap->load_from_memory(bytes.data(), static_cast<u32>(bytes.size())))
        {
            Log::error("MaterialManager: failed to open sequence frame '%s'", path.c_str());
            ok = false;
        }
        else if (images.empty())
        {
            width = static_cast<u32>(pixmap->width);
            height = static_cast<u32>(pixmap->height);
        }
        else if (static_cast<u32>(pixmap->width) != width ||
                static_cast<u32>(pixmap->height) != height)
        {
            Log::error("MaterialManager: sequence frame '%s' is %dx%d, expected %ux%u",
                       path.c_str(), pixmap->width, pixmap->height, width, height);
            ok = false;
        }
        images.push_back(pixmap);
    }

    TextureHandle handle;
    if (ok && width > 0 && height > 0)
    {
        TextureDesc desc;
        desc.type = TextureType::Tex2DArray;
        desc.width = width;
        desc.height = height;
        desc.depth = static_cast<u32>(images.size());
        desc.format = space == ColorSpace::sRGB ? Format::RGBA8_sRGB : Format::RGBA8;
        desc.mips = 0; // full chain, generated below once every slice is in
        desc.usage = TextureSampled;
        desc.debugName = "material.sequence";
        handle = GPU::getSingleton().createTexture(desc);

        if (handle.valid())
        {
            GPU& gpu = GPU::getSingleton();
            for (u32 layer = 0; layer < images.size(); ++layer)
            {
                Pixmap* pixmap = images[layer];
                if (!pixmap->pixels)
                    continue;
                Pixmap* converted = pixmap->components == 3 ? pixmap->convert_to_rgba() : nullptr;
                const Pixmap& image = converted ? *converted : *pixmap;
                gpu.updateTexture(handle, 0, layer, 0, 0, width, height, image.pixels);
                delete converted;
            }
            gpu.generateMips(handle);
        }
    }

    for (Pixmap* pixmap : images)
        delete pixmap;

    return handle;
}

const char* blendName(BlendMode blend)
{
    static const char* names[] = {
        "Opaque",    "Alpha",         "Additive", "Multiply", "PremultipliedAlpha",
        "AddColors", "SubtractColors"};
    return names[static_cast<u8>(blend)];
}

const char* cullName(CullMode cull)
{
    static const char* names[] = {"None", "Back", "Front"};
    return names[static_cast<u8>(cull)];
}

const char* slotName(u8 slot)
{
    return slot < kMaterialSlotCount ? kMaterialSlotNames[slot] : kMaterialSlotNames[7];
}

const char* sourceName(TextureSource source)
{
    static const char* names[] = {"None", "Static", "Sequence", "RenderTarget"};
    return names[static_cast<u8>(source)];
}

const char* filterName(Filter filter)
{
    static const char* names[] = {"Point", "Linear", "Trilinear", "Anisotropic"};
    return names[static_cast<u8>(filter)];
}

const char* wrapName(Wrap wrap)
{
    static const char* names[] = {"Repeat", "Mirror", "Clamp", "Border"};
    return names[static_cast<u8>(wrap)];
}

const char* curveName(Curve curve)
{
    static const char* names[] = {"Linear", "SineWave", "PingPong", "Noise"};
    return names[static_cast<u8>(curve)];
}

void writeQuoted(std::ostream& out, const std::string& value)
{
    out << '"';
    for (char c : value)
    {
        if (c == '"' || c == '\\')
            out << '\\';
        out << c;
    }
    out << '"';
}

bool writeMaterialAtomically(const std::string& filename, const std::string& text)
{
    const std::filesystem::path destination(filename);
    const std::filesystem::path temporary = destination.string() + ".tmp";
    const std::filesystem::path backup = destination.string() + ".bak";
    std::error_code error;
    std::filesystem::remove(temporary, error);
    error.clear();
    if (!FileSystem::getSingleton().writeText(temporary.string(), text))
        return false;

    const bool hadDestination = std::filesystem::exists(destination, error) && !error;
    if (hadDestination)
    {
        std::filesystem::remove(backup, error);
        error.clear();
        std::filesystem::rename(destination, backup, error);
        if (error)
        {
            Log::error("MaterialManager: cannot prepare atomic save '%s': %s", filename.c_str(),
                       error.message().c_str());
            std::filesystem::remove(temporary, error);
            return false;
        }
    }

    error.clear();
    std::filesystem::rename(temporary, destination, error);
    if (error)
    {
        Log::error("MaterialManager: cannot commit material save '%s': %s", filename.c_str(),
                   error.message().c_str());
        if (hadDestination)
        {
            std::error_code restoreError;
            std::filesystem::rename(backup, destination, restoreError);
            if (restoreError)
                Log::error("MaterialManager: backup remains at '%s' after restore failed: %s",
                           backup.string().c_str(), restoreError.message().c_str());
        }
        std::filesystem::remove(temporary, error);
        return false;
    }

    if (hadDestination)
        std::filesystem::remove(backup, error);
    return true;
}

const char* animationProperty(const MaterialAnim& animation)
{
    if (animation.field == 0)
        return "baseColor";
    if (animation.field == 1)
        return "emissive";
    if (animation.field == 2 && animation.mask == 0x1)
        return "roughness";
    if (animation.field == 2 && animation.mask == 0x2)
        return "metalness";
    if (animation.field == 2 && animation.mask == 0x4)
        return "alphaCut";
    if (animation.field == 2)
        return "surface";
    if (animation.field == 3)
        return "uvTransform";
    if (animation.field == 4)
        return "uvAnim";
    if (animation.field == 5)
        return "sequence";
    if (animation.field == 6 && animation.mask == 0x1)
        return "distortion";
    if (animation.field == 6)
        return "custom0";
    return "custom1";
}

void writeVec4(std::ostringstream& text, const glm::vec4& value)
{
    text << '(' << value.x << ", " << value.y << ", " << value.z << ", " << value.w << ')';
}

void writeFlags(std::ostringstream& text, u32 flags)
{
    u32 count = 0;
    const MaterialFlagName* entries = MaterialManager::flagNames(count);
    bool comma = false;
    text << '[';
    for (u32 i = 0; i < count; ++i)
    {
        if (!(flags & entries[i].bit))
            continue;
        if (comma)
            text << ", ";
        text << entries[i].name;
        comma = true;
    }
    text << ']';
}

u64 packPipelineKey(const PipelineKey& key)
{
    return (static_cast<u64>(static_cast<u8>(key.category)) << 56) |
           (static_cast<u64>(static_cast<u8>(key.blend)) << 48) |
           (static_cast<u64>(static_cast<u8>(key.cull)) << 40) |
           (static_cast<u64>(key.pass) << 32) | static_cast<u64>(key.flags);
}

bool layoutsEqual(const VertexLayout& a, const VertexLayout& b)
{
    if (a.attribCount != b.attribCount || a.streamCount != b.streamCount)
        return false;
    for (u8 i = 0; i < a.attribCount; ++i)
    {
        const VertexAttrib& x = a.attribs[i];
        const VertexAttrib& y = b.attribs[i];
        if (x.location != y.location || x.stream != y.stream || x.offset != y.offset ||
            x.format != y.format)
            return false;
    }
    for (u8 i = 0; i < a.streamCount; ++i)
    {
        if (a.streams[i].stride != b.streams[i].stride ||
            a.streams[i].perInstance != b.streams[i].perInstance)
            return false;
    }
    return true;
}

u64 hashLayout(const VertexLayout& layout)
{
    u64 hash = 1469598103934665603ull;
    const auto append = [&hash](u64 value)
    {
        hash ^= value;
        hash *= 1099511628211ull;
    };

    append(layout.attribCount);
    append(layout.streamCount);
    for (u8 i = 0; i < layout.attribCount; ++i)
    {
        const VertexAttrib& attrib = layout.attribs[i];
        append(attrib.location);
        append(attrib.stream);
        append(attrib.offset);
        append(static_cast<u8>(attrib.format));
    }
    for (u8 i = 0; i < layout.streamCount; ++i)
    {
        append(layout.streams[i].stride);
        append(layout.streams[i].perInstance ? 1u : 0u);
    }
    return hash;
}

u32 variantFlagsOf(const Material& material)
{
    u32 flags = 0;
    if (material.textures[SlotAlbedo].texture.valid())
        flags |= VariantHasAlbedo;
    if (material.textures[SlotDetail].texture.valid())
    {
        flags |= VariantHasDetail;
        if (material.textures[SlotDetail].source == TextureSource::Sequence)
            flags |= VariantHasDetailSequence;
    }
    if (material.textures[SlotNormal].texture.valid())
        flags |= VariantHasNormal;
    if (material.textures[SlotSurface].texture.valid())
        flags |= VariantHasSurface;
    if (material.textures[SlotColorMap].texture.valid())
        flags |= VariantHasColorMap;
    if (material.textures[SlotEmissive].texture.valid())
        flags |= VariantHasEmissive;
    if (material.textures[SlotLightmap].texture.valid())
        flags |= VariantHasLightmap;
    if (material.textures[SlotHeight].texture.valid())
        flags |= VariantHasHeight;
    return flags;
}

// GLSL requires #version to be the file's first line, so a variant #define
// cannot simply be prepended - it goes right after it instead.
std::string withVariantDefines(const std::string& source, u32 variantFlags, u8 pass)
{
    if (variantFlags == 0 && (pass & MaterialPipelineNoTemporal) == 0)
        return source;

    const usize versionEnd = source.find('\n');
    std::string result = source.substr(0, versionEnd + 1);
    if (variantFlags & MaterialSkinned)
        result += "#define MATERIAL_SKINNED 1\n";
    if (variantFlags & MaterialReceiveShadow)
        result += "#define RECEIVES_SHADOW 1\n";
    if (variantFlags & MaterialLandscape)
        result += "#define LANDSCAPE_REGIONS 1\n";
    if (variantFlags & MaterialTerrain)
        result += "#define TERRAIN_SURFACE 1\n";
    if (variantFlags & MaterialTerrainClassic)
        result += "#define TERRAIN_CLASSIC 1\n";
    if (variantFlags & MaterialVoxelAtlas)
        result += "#define VOXEL_ATLAS 1\n";
    if (variantFlags & MaterialReflection)
        result += "#define HAS_REFLECTION 1\n";
    if (variantFlags & MaterialMirror)
        result += "#define HAS_MIRROR 1\n";
    if (variantFlags & MaterialParallax)
        result += "#define HAS_PARALLAX 1\n";
    if (variantFlags & MaterialMetallicRoughnessMap)
        result += "#define HAS_METALLIC_ROUGHNESS_MAP 1\n";
    if (variantFlags & MaterialSpecularGlossinessMap)
        result += "#define HAS_SPECULAR_GLOSSINESS_MAP 1\n";
    if (variantFlags & VariantHasAlbedo)
        result += "#define HAS_ALBEDO 1\n";
    if (variantFlags & VariantHasDetail)
        result += "#define HAS_DETAIL 1\n";
    if (variantFlags & VariantHasDetailSequence)
        result += "#define DETAIL_SEQUENCE 1\n";
    if (variantFlags & VariantHasNormal)
        result += "#define HAS_NORMAL 1\n";
    if (variantFlags & VariantHasSurface)
        result += "#define HAS_SURFACE 1\n";
    if (variantFlags & VariantHasColorMap)
        result += "#define HAS_COLORMAP 1\n";
    if (variantFlags & VariantHasEmissive)
        result += "#define HAS_EMISSIVE 1\n";
    if (variantFlags & VariantHasLightmap)
        result += "#define HAS_LIGHTMAP 1\n";
    if (variantFlags & VariantHasHeight)
        result += "#define HAS_HEIGHT 1\n";
    if (pass & MaterialPipelineNoTemporal)
        result += "#define NO_TEMPORAL 1\n";
    result += source.substr(versionEnd + 1);
    return result;
}

glm::vec4* paramField(MaterialParams& params, u8 index)
{
    switch (index)
    {
    case 0:
        return &params.baseColor;
    case 1:
        return &params.emissive;
    case 2:
        return &params.surface;
    case 3:
        return &params.uvTransform;
    case 4:
        return &params.uvAnim;
    case 5:
        return &params.sequence;
    case 6:
        return &params.custom0;
    case 7:
        return &params.custom1;
    default:
        return nullptr;
    }
}

// Every curve returns 0..1 so the caller only has to interpolate min..max.
f32 evaluateCurve(Curve curve, f32 time, f32 speed, f32 phase)
{
    const f32 t = time * speed + phase;

    switch (curve)
    {
    case Curve::Linear:
        return t - std::floor(t);
    case Curve::SineWave:
        return 0.5f + 0.5f * std::sin(t * 6.28318530718f);
    case Curve::PingPong:
    {
        const f32 wrapped = t - std::floor(t);
        return wrapped < 0.5f ? wrapped * 2.0f : 2.0f - wrapped * 2.0f;
    }
    case Curve::Noise:
    {
        // Value noise: interpolate between two hashed integers so the result is
        // deterministic per material and does not jump between frames.
        const f32 floored = std::floor(t);
        const f32 frac = t - floored;
        const u32 a = static_cast<u32>(static_cast<s32>(floored));
        const u32 b = a + 1u;
        const f32 ha = static_cast<f32>((a * 1103515245u + 12345u) & 0xFFFFu) / 65535.0f;
        const f32 hb = static_cast<f32>((b * 1103515245u + 12345u) & 0xFFFFu) / 65535.0f;
        const f32 smooth = frac * frac * (3.0f - 2.0f * frac);
        return ha + (hb - ha) * smooth;
    }
    }
    return 0.0f;
}

} // namespace

bool PipelineKey::operator==(const PipelineKey& other) const
{
    return category == other.category && blend == other.blend && cull == other.cull &&
           pass == other.pass && flags == other.flags;
}

bool PipelineCacheKey::operator==(const PipelineCacheKey& other) const
{
    return key == other.key && layoutsEqual(layout, other.layout);
}

usize PipelineCacheKeyHash::operator()(const PipelineCacheKey& cacheKey) const
{
    u64 packed = packPipelineKey(cacheKey.key);
    packed ^= hashLayout(cacheKey.layout) + 0x9E3779B97F4A7C15ull + (packed << 6) + (packed >> 2);
    return static_cast<usize>(packed);
}

MaterialManager::MaterialManager()
{
}

MaterialManager& MaterialManager::getSingleton()
{
    static MaterialManager instance;
    return instance;
}

const MaterialFlagName* MaterialManager::flagNames(u32& count)
{
    static const MaterialFlagName entries[] = {
        {MaterialCastShadow, "CastShadow"},
        {MaterialReceiveShadow, "ReceiveShadow"},
        {MaterialTwoSided, "TwoSided"},
        {MaterialAlphaTest, "AlphaTest"},
        {MaterialRefraction, "Refraction"},
        {MaterialReflection, "Reflection"},
        {MaterialSkinned, "Skinned"},
        {MaterialNoDepthWrite, "NoDepthWrite"},
        {MaterialAnimated, "Animated"},
        {MaterialLit, "Lit"},
        {MaterialLandscape, "Landscape"},
        {MaterialTerrain, "Terrain"},
        {MaterialTerrainClassic, "TerrainClassic"},
        {MaterialMirror, "Mirror"},
        {MaterialParallax, "Parallax"},
        {MaterialMetallicRoughnessMap, "MetallicRoughnessMap"},
        {MaterialSpecularGlossinessMap, "SpecularGlossinessMap"}};
    count = static_cast<u32>(sizeof(entries) / sizeof(entries[0]));
    return entries;
}

u32 MaterialManager::flagBit(const char* name)
{
    if (!name)
        return 0;
    u32 count = 0;
    const MaterialFlagName* entries = flagNames(count);
    for (u32 i = 0; i < count; ++i)
    {
        if (std::strcmp(entries[i].name, name) == 0)
            return entries[i].bit;
    }
    return 0;
}

RenderCategory MaterialManager::categoryOf(const Material& material) const
{
    if (material.flags & MaterialRefraction)
        return RenderCategory::Refraction;
    if (material.blend != BlendMode::Opaque)
        return RenderCategory::Transparent;
    if (material.flags & MaterialAlphaTest)
        return RenderCategory::AlphaTest;
    return RenderCategory::Opaque;
}

PipelineKey MaterialManager::pipelineKeyOf(const Material& material, u8 pass) const
{
    PipelineKey key;
    key.category = categoryOf(material);
    key.blend = material.blend;
    key.cull = (material.flags & MaterialTwoSided) ? CullMode::None : material.cull;
    key.pass = pass;

    // Only the flags that change the shader belong in the key. CastShadow
    // picks which pass runs and does not change its code, so it stays out.
    // ReceiveShadow is in: lit.frag branches on it to skip the cascade lookup
    // entirely, which is one extra pipeline and the only way the flag means
    // anything at all - before this it was authored, stored, and ignored.
    key.flags = (material.flags & (MaterialAlphaTest | MaterialSkinned | MaterialRefraction |
                                   MaterialReflection | MaterialMirror | MaterialAnimated |
                                   MaterialLit | MaterialReceiveShadow | MaterialLandscape |
                                   MaterialTerrain | MaterialTerrainClassic |
                                   MaterialVoxelAtlas |
                                   MaterialParallax | MaterialMetallicRoughnessMap |
                                   MaterialSpecularGlossinessMap)) |
                variantFlagsOf(material);
    return key;
}

bool MaterialManager::load(const std::string& filename, std::vector<Material>& materials) const
{
    // Before the parse and the texture loads it triggers, for the same
    // reason the mesh load logs first: this is where a big .material file
    // (Bistro's is thousands of entries, each pulling its own textures)
    // holds the editor still, and only a line printed up front says so.
    Log::info("MaterialManager: loading materials '%s'...", filename.c_str());
    const auto started = std::chrono::steady_clock::now();

    std::vector<MaterialDefinition> definitions;
    MaterialParseError error;
    if (!MaterialParser::parseFile(filename, definitions, &error))
    {
        Log::error("MaterialManager: %s:%u:%u: %s", filename.c_str(), error.line, error.column,
                   error.message.c_str());
        return false;
    }

    // The parse is over; everything past here is texture decoding and GPU
    // uploads, which is the slow half. Bracketed by its own pair of lines so
    // the two costs can be told apart instead of only seeing one total.
    const auto parsedAt = std::chrono::steady_clock::now();
    Log::info("MaterialManager: parsed %zu definition(s) from '%s' in %.0f ms, loading their "
              "textures...",
              definitions.size(), filename.c_str(),
              std::chrono::duration<f64, std::milli>(parsedAt - started).count());

    std::vector<Material> loadedMaterials;
    loadedMaterials.reserve(definitions.size());
    AssetManager& assets = Assets();
    usize textureSlots = 0;
    usize samplerSlots = 0;
    for (const MaterialDefinition& definition : definitions)
    {
        Material loaded = definition.material;
        for (const MaterialTextureSource& source : definition.textures)
        {
            if (source.slot >= MaterialSlotCount)
                continue;

            MaterialTexture& texture = loaded.textures[source.slot];
            if (source.source == TextureSource::Static)
            {
                if (source.file.empty())
                {
                    Log::error("MaterialManager: texture '%s' in material '%s' has no file",
                               source.name.c_str(), definition.name.c_str());
                    return false;
                }
                ColorSpace space = Material::colorSpaceFor(
                    static_cast<MaterialSlot>(source.slot), loaded.flags);
                if (source.colorSpace == ColorSpaceOverride::Linear)
                    space = ColorSpace::Linear;
                else if (source.colorSpace == ColorSpaceOverride::sRGB)
                    space = ColorSpace::sRGB;

                // Streamed, not decoded here: a level's sidecar names
                // hundreds of textures and each one costs hundreds of
                // milliseconds, so decoding them in this loop stalls whoever
                // called for as long as all of them together. The handle is
                // valid immediately and the pixels arrive over the following
                // frames - the same choice loadMeshMaterialTextures() already
                // makes for the textures a mesh names itself.
                ++textureSlots;
                texture.texture =
                    assets.loadTextureAsync(resolveMaterialPath(filename, source.file), space,
                                            source.generateMips);
                texture.file = source.file;
            }
            else if (source.source == TextureSource::Sequence)
            {
                if (source.frames.empty())
                {
                    Log::error("MaterialManager: sequence texture '%s' in material '%s' has no "
                               "frames",
                               source.name.c_str(), definition.name.c_str());
                    return false;
                }
                ColorSpace space = Material::colorSpaceFor(
                    static_cast<MaterialSlot>(source.slot), loaded.flags);
                if (source.colorSpace == ColorSpaceOverride::Linear)
                    space = ColorSpace::Linear;
                else if (source.colorSpace == ColorSpaceOverride::sRGB)
                    space = ColorSpace::sRGB;

                texture.texture = loadSequenceTexture(filename, source.frames, space);
            }

            SamplerDesc sampler;
            sampler.filter = source.filter;
            sampler.wrapU = source.wrap;
            sampler.wrapV = source.wrap;
            sampler.wrapW = source.wrap;
            // Asking for anisotropic filtering and leaving the amount at 1
            // gets plain trilinear back. Ground and walls seen at a grazing
            // angle are exactly what it is for, so the material system picks
            // a real amount; the backend clamps it to what the GPU supports.
            if (sampler.filter == Filter::Anisotropic)
                sampler.anisotropy = 8.0f;
            ++samplerSlots;
            texture.sampler = assets.getSampler(sampler);
        }
        loadedMaterials.push_back(loaded);
    }
    Log::info("MaterialManager: textures for '%s' done in %.0f ms (%zu texture slots, %zu samplers)",
              filename.c_str(),
              std::chrono::duration<f64, std::milli>(std::chrono::steady_clock::now() - parsedAt)
                  .count(),
              textureSlots, samplerSlots);

    // Imported meshes carry the material-slot order used by their submeshes.
    // Material files are commonly authored in a different order, so when the
    // destination already has the same named slots, reorder the parsed
    // definitions to the mesh's order instead of silently assigning a
    // material to the wrong geometry. Older callers with unnamed/generic
    // slots retain the historical positional behaviour.
    if (materials.size() == loadedMaterials.size() && !materials.empty())
    {
        std::vector<Material> reordered(materials.size());
        std::vector<bool> used(loadedMaterials.size(), false);
        bool canMatchByName = true;
        for (usize slot = 0; slot < materials.size(); ++slot)
        {
            const std::string& wanted = materials[slot].name;
            usize found = loadedMaterials.size();
            for (usize candidate = 0; candidate < loadedMaterials.size(); ++candidate)
            {
                if (!used[candidate] && loadedMaterials[candidate].name == wanted)
                {
                    found = candidate;
                    break;
                }
            }
            if (wanted.empty() || found == loadedMaterials.size())
            {
                canMatchByName = false;
                break;
            }
            used[found] = true;
            reordered[slot] = std::move(loadedMaterials[found]);
        }
        if (canMatchByName)
            loadedMaterials = std::move(reordered);
    }

    // Metadata such as sequence frames/filter/wrap belongs to this sidecar,
    // not globally to a 32-bit material-name hash. Common names like Default
    // occur in unrelated assets and must never contaminate each other's save.
    gMaterialSources[materialSourceKey(filename)] = definitions;
    materials.swap(loadedMaterials);
    Log::info("MaterialManager: loaded %zu material(s) from '%s' in %.0f ms", materials.size(),
              filename.c_str(),
              std::chrono::duration<f64, std::milli>(std::chrono::steady_clock::now() - started)
                  .count());
    return true;
}

bool MaterialManager::save(const std::string& filename,
                           const std::vector<Material>& materials) const
{
    std::ostringstream text;
    const auto sourceFile = gMaterialSources.find(materialSourceKey(filename));
    for (const Material& material : materials)
    {
        // Materials built in code (a default material for a mesh with no
        // authored one, or one patched at runtime for a lightmap bake) never
        // pass through the parser, so they have no entry here. That is not a
        // reason to refuse the save: fall back to an empty definition and let
        // the per-slot loop below reconstruct its textures straight from the
        // runtime Material.
        MaterialDefinition fallback;
        fallback.name = material.name.empty() ? std::to_string(material.nameHash) : material.name;
        const MaterialDefinition* source = &fallback;
        if (sourceFile != gMaterialSources.end())
        {
            for (const MaterialDefinition& candidate : sourceFile->second)
            {
                if ((!material.name.empty() && candidate.name == material.name) ||
                    (material.name.empty() && candidate.material.nameHash == material.nameHash))
                {
                    source = &candidate;
                    break;
                }
            }
        }
        text << "material ";
        writeQuoted(text, source->name);
        text << "\n{\n";
        text << "    flags ";
        writeFlags(text, material.flags);
        text << "\n    blendMode " << blendName(material.blend);
        text << "\n    cullMode " << cullName(material.cull) << "\n\n";
        text << "    properties\n    {\n        baseColor ";
        writeVec4(text, material.params.baseColor);
        text << "\n        emissive ";
        writeVec4(text, material.params.emissive);
        text << "\n        surface ";
        writeVec4(text, material.params.surface);
        text << "\n        uvTransform ";
        writeVec4(text, material.params.uvTransform);
        text << "\n        uvAnim ";
        writeVec4(text, material.params.uvAnim);
        text << "\n        sequence ";
        writeVec4(text, material.params.sequence);
        // These vectors are technique-owned. Named aliases only covered a
        // subset of their components and silently dropped Mirror's bump/zoom
        // (custom0.zw) plus custom1.w. Raw vectors are understood by the
        // parser and preserve every editor-authored value for every technique.
        text << "\n        custom0 ";
        writeVec4(text, material.params.custom0);
        text << "\n        custom1 ";
        writeVec4(text, material.params.custom1);
        text << "\n    }\n";

        bool hasGeneratedTexture = false;
        for (u32 slot = 0; slot < MaterialSlotCount; ++slot)
            if (material.textures[slot].file.size() > 0)
                hasGeneratedTexture = true;
        const bool hasTextures = !source->textures.empty() || hasGeneratedTexture;
        if (hasTextures)
            text << "\n    textures\n    {\n";
        for (const MaterialTextureSource& texture : source->textures)
        {
            const MaterialTexture& runtimeTexture = material.textures[texture.slot];
            // Runtime state is authoritative. In particular, a default
            // MaterialTexture is how the Inspector represents Clear Textures;
            // falling back to the parsed source here resurrected deleted maps.
            if (runtimeTexture.source == TextureSource::None)
                continue;
            if (runtimeTexture.source == TextureSource::Static && runtimeTexture.file.empty())
                continue;

            text << "        texture ";
            writeQuoted(text, texture.name);
            text << "\n        {\n";
            text << "            slot " << slotName(texture.slot) << "\n";
            text << "            type " << sourceName(runtimeTexture.source) << "\n";
            if (runtimeTexture.source == TextureSource::Static)
            {
                text << "            file ";
                writeQuoted(text, runtimeTexture.file);
                text << "\n";
            }
            if (runtimeTexture.source == TextureSource::RenderTarget && !texture.target.empty())
            {
                text << "            source ";
                writeQuoted(text, texture.target);
                text << "\n";
            }
            text << "            filter " << filterName(texture.filter) << "\n";
            text << "            wrap " << wrapName(texture.wrap) << "\n";
            // Only written when it disagrees with the slot, so a re-read gives
            // back the same material without carrying the default around.
            if (texture.colorSpace != ColorSpaceOverride::FromSlot)
                text << "            srgb "
                     << (texture.colorSpace == ColorSpaceOverride::sRGB ? "true" : "false") << "\n";
            text << "            generateMips " << (texture.generateMips ? "true" : "false")
                 << "\n";
            if (runtimeTexture.source == TextureSource::Sequence && !texture.frames.empty())
            {
                text << "            frames [";
                for (usize i = 0; i < texture.frames.size(); ++i)
                {
                    if (i)
                        text << ", ";
                    writeQuoted(text, texture.frames[i]);
                }
                text << "]\n";
            }
            text << "        }\n";
        }
        for (u32 slot = 0; slot < MaterialSlotCount; ++slot)
        {
            const MaterialTexture& runtimeTexture = material.textures[slot];
            bool present = false;
            for (const MaterialTextureSource& sourceTexture : source->textures)
                if (sourceTexture.slot == slot)
                    present = true;
            if (present || runtimeTexture.file.empty())
                continue;
            text << "        texture ";
            writeQuoted(text, slotName(static_cast<u8>(slot)));
            text << "\n        {\n";
            text << "            slot " << slotName(static_cast<u8>(slot)) << "\n";
            text << "            type Static\n            file ";
            writeQuoted(text, runtimeTexture.file);
            // Same defaults used by the Inspector at drop time and by a
            // parsed MaterialTextureSource. The image must not change its
            // filtering or UV wrap merely because the scene was reopened.
            text << "\n            filter Anisotropic\n            wrap Repeat\n"
                    "            generateMips true\n        }\n";
        }
        if (hasTextures)
            text << "    }\n";

        if (material.animCount)
            text << "\n    animations\n    {\n";
        for (u8 i = 0; i < material.animCount; ++i)
        {
            const MaterialAnim& animation = material.anims[i];
            text << "        animation" << static_cast<u32>(i) << "\n        {\n";
            text << "            property \"" << animationProperty(animation) << "\"\n";
            text << "            type " << curveName(animation.curve) << "\n";
            text << "            speed " << animation.speed << "\n";
            text << "            phase " << animation.phase << "\n";
            text << "            min ";
            writeVec4(text, animation.min);
            text << "\n            max ";
            writeVec4(text, animation.max);
            text << "\n        }\n";
        }
        if (material.animCount)
            text << "    }\n";
        text << "}\n\n";
    }

    return writeMaterialAtomically(filename, text.str());
}

void MaterialManager::animate(Material& material, f32 time) const
{
    if (material.animCount == 0)
        return;

    for (u8 i = 0; i < material.animCount; ++i)
    {
        const MaterialAnim& anim = material.anims[i];
        glm::vec4* field = paramField(material.params, anim.field);
        if (!field)
            continue;

        const f32 t = evaluateCurve(anim.curve, time, anim.speed, anim.phase);
        const glm::vec4 value = anim.min + (anim.max - anim.min) * t;

        for (u8 component = 0; component < 4; ++component)
        {
            if (anim.mask & (1u << component))
                (*field)[component] = value[component];
        }
    }

    material.paramsDirty = true;
}

void MaterialManager::sync(Material& material) const
{
    GPU& gpu = GPU::getSingleton();

    if (!material.paramsBuffer.valid())
    {
        BufferDesc desc;
        desc.size = sizeof(MaterialParams);
        desc.usage = BufferUniform;
        desc.residency = Residency::Dynamic;
        desc.debugName = "material.params";
        material.paramsBuffer = gpu.createBuffer(desc);
        material.paramsDirty = true;
    }

    if (!material.paramsDirty)
        return;

    gpu.updateBuffer(material.paramsBuffer, 0, sizeof(MaterialParams), &material.params);
    material.paramsDirty = false;
}

PipelineHandle MaterialManager::resolvePipeline(Material& material, const VertexLayout& layout,
                                                u8 pass)
{
    const char* vertexFile = nullptr;
    const char* fragmentFile = nullptr;
    const char* debugName = nullptr;
    switch (categoryOf(material))
    {
    case RenderCategory::Refraction:
        vertexFile = kWaterVertexFile;
        fragmentFile = kWaterFragmentFile;
        debugName = "material.water";
        break;
    default:
        if (material.flags & MaterialLandscape)
        {
            // Same fragment shader as an ordinary Lit material - only the
            // LANDSCAPE_REGIONS variant define differs, added below through
            // the usual variant-flag path. The vertex shader differs for
            // real: a chunk's attributes are position/normal/uv/weights, not
            // lit.vert's position/normal/tangent/uv/colour.
            vertexFile = kLandscapeVertexFile;
            fragmentFile = kLitFragmentFile;
            debugName = "material.landscape";
        }
        else if (material.flags & MaterialLit)
        {
            vertexFile = kLitVertexFile;
            fragmentFile = kLitFragmentFile;
            debugName = "material.lit";
        }
        else
        {
            vertexFile = kUnlitVertexFile;
            fragmentFile = kUnlitFragmentFile;
            debugName = "material.forward";
        }
        break;
    }

    const PipelineKey key = pipelineKeyOf(material, pass);
    const PipelineCacheKey cacheKey{key, layout};

    auto it = mPipelines.find(cacheKey);
    if (it != mPipelines.end())
    {
        if (pass == MaterialPipelineForward)
            material.pipeline = it->second;
        return it->second;
    }

    AssetManager& assets = Assets();
    const std::string& vertexSource = assets.loadShader(vertexFile);
    const std::string& fragmentSource = assets.loadShader(fragmentFile);
    if (vertexSource.empty() || fragmentSource.empty())
    {
        Log::error("MaterialManager: could not load shader source for '%s'", debugName);
        return PipelineHandle();
    }

    // Cached per PipelineKey (which already folds in variantFlagsOf), so this
    // string work only happens once per distinct variant, not per material.
    const std::string vertexVariant = withVariantDefines(vertexSource, key.flags, pass);
    const std::string fragmentVariant = withVariantDefines(fragmentSource, key.flags, pass);

    PipelineDesc desc;
    desc.vs.code = vertexVariant.c_str();
    desc.vs.name = vertexFile;
    desc.fs.code = fragmentVariant.c_str();
    desc.fs.name = fragmentFile;
    desc.layout = layout;
    desc.blend.mode = material.blend;
    desc.depth.write = (material.flags & MaterialNoDepthWrite) == 0;
    desc.raster.cull = (material.flags & MaterialTwoSided) ? CullMode::None : material.cull;
    desc.debugName = debugName;

    PipelineHandle handle = GPU::getSingleton().createPipeline(desc);
    if (!handle.valid())
    {
        Log::error("MaterialManager: failed to compile the '%s' pipeline", debugName);
        return handle;
    }

    mPipelines.emplace(cacheKey, handle);
    if (pass == MaterialPipelineForward)
        material.pipeline = handle;
    return handle;
}

void MaterialManager::release(Material& material) const
{
    if (!material.paramsBuffer.valid())
        return;

    // Scene objects in demos are commonly stack-owned and therefore outlive
    // an explicit engine.shutdown(). Their MeshRenderer destructors still
    // clear material overrides, but at that point the device and all of its
    // pools have already gone away. The handle must still be invalidated;
    // there is simply no live GPU resource left to destroy.
    if (GPU* gpu = GPU::tryGet())
        gpu->destroy(material.paramsBuffer);
    material.paramsBuffer = BufferHandle();
    material.paramsDirty = true;
}

void MaterialManager::destroyAllPipelines()
{
    GPU& gpu = GPU::getSingleton();
    for (auto& entry : mPipelines)
        gpu.destroy(entry.second);
    mPipelines.clear();
}

} // namespace Radion
