#include "PCH.h"

#include "AssetManager.h"
#include "AsyncTextureLoader.h"
#include "DDSImage.h"
#include "FileSystem.h"
#include "Pixmap.h"
#include "TextureDecode.h"

#include <algorithm>
#include <cctype>
#include <chrono>

namespace Radion
{

namespace
{

bool sameSampler(const SamplerDesc& a, const SamplerDesc& b)
{
    return a.filter == b.filter && a.wrapU == b.wrapU && a.wrapV == b.wrapV && a.wrapW == b.wrapW &&
           a.anisotropy == b.anisotropy && a.mipBias == b.mipBias && a.compare == b.compare &&
           a.border[0] == b.border[0] && a.border[1] == b.border[1] && a.border[2] == b.border[2] &&
           a.border[3] == b.border[3];
}

std::string textureCacheKey(const std::string& filename, ColorSpace space, bool generateMips,
                            u32 mipLimit)
{
    return filename + (space == ColorSpace::sRGB ? "\x1fsrgb" : "\x1flinear") +
           (generateMips ? "\x1fmips" : "\x1fno-mips") + "\x1f" + std::to_string(mipLimit);
}

const char* const kTextureExtensions[] = {".png", ".jpg", ".jpeg", ".tga", ".dds", ".bmp"};

std::string resolveTextureFile(const std::string& filename)
{
    FileSystem& fs = FileSystem::getSingleton();
    if (filename.empty() || fs.exists(filename))
        return filename;
    const usize dot = filename.find_last_of('.');
    const usize slash = filename.find_last_of("/\\");
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
        return filename;
    const std::string stem = filename.substr(0, dot);
    for (const char* extension : kTextureExtensions)
    {
        const std::string candidate = stem + extension;
        if (candidate != filename && fs.exists(candidate))
        {
            Log::info("AssetManager: '%s' not found, using '%s'", filename.c_str(),
                      candidate.c_str());
            return candidate;
        }
    }
    return filename;
}

// One suffix set per authoring convention, already ordered +X -X +Y -Y +Z -Z
// to match GL's cube-face order. FR/BK (and Front/Back) are the classic
// front-back ambiguity between tools that export skyboxes - if a loaded sky
// reads mirrored front-to-back, swap the last two entries of the offending
// table below.
struct CubemapConvention
{
    const char* suffix[6];
};

constexpr CubemapConvention kConventionRTLF = {
    {"_RT.jpg", "_LF.jpg", "_UP.jpg", "_DN.jpg", "_FR.jpg", "_BK.jpg"}};
constexpr CubemapConvention kConventionPXNX = {
    {"_px.jpg", "_nx.jpg", "_py.jpg", "_ny.jpg", "_pz.jpg", "_nz.jpg"}};
constexpr CubemapConvention kConventionSkyBox = {
    {"/SkyBox_Right.jpg", "/SkyBox_Left.jpg", "/SkyBox_Top.jpg", "/SkyBox_Bottom.jpg",
     "/SkyBox_Front.jpg", "/SkyBox_Back.jpg"}};

bool tryCubemapConvention(const std::string& baseName, const CubemapConvention& convention,
                          std::string faces[6])
{
    FileSystem& fs = FileSystem::getSingleton();
    for (int i = 0; i < 6; ++i)
    {
        faces[i] = baseName + convention.suffix[i];
        if (!fs.exists(faces[i]))
            return false;
    }
    return true;
}

} // namespace

// ----------------------------------------------------------------- textures

TextureHandle AssetManager::reloadTexture(const std::string& filename, ColorSpace space,
                                          bool generateMips, u32 mipLimit)
{
    const std::string cacheKey = textureCacheKey(filename, space, generateMips, mipLimit);
    auto cached = mLoadedTextures.find(cacheKey);
    if (cached != mLoadedTextures.end())
        destroyTexture(mTextures[cached->second].handle);
    return loadTexture(filename, space, generateMips, mipLimit);
}

TextureHandle AssetManager::registerTextureEntry(const std::string& filename,
                                                 const std::string& cacheKey, TextureHandle handle)
{
    TextureEntry entry;
    entry.name = filename;
    entry.cacheKey = cacheKey;
    entry.handle = handle;
    const usize index = mTextures.size();
    mTextures.push_back(entry);
    mLoadedTextures[cacheKey] = index;
    mTexturesByName.emplace(filename, index);
    mTexturesByHandle[packHandle(handle)] = index;
    return handle;
}

TextureHandle AssetManager::loadTexture(const std::string& filename, ColorSpace space,
                                        bool generateMips, u32 mipLimit)
{
    const std::string cacheKey = textureCacheKey(filename, space, generateMips, mipLimit);
    auto cached = mLoadedTextures.find(cacheKey);
    if (cached != mLoadedTextures.end())
        return mTextures[cached->second].handle;

    // Not logged per texture - a Bistro-sized material file pulls thousands
    // and the console becomes useless. Only the ones slow enough to be worth
    // knowing about are named, which is what a load that stalls needs.
    const auto started = std::chrono::steady_clock::now();
    DecodedTexture decoded =
        decodeTextureFile(resolveTextureFile(filename), space, generateMips, mipLimit);
    const f64 decodeMilliseconds =
        std::chrono::duration<f64, std::milli>(std::chrono::steady_clock::now() - started).count();
    if (decodeMilliseconds > 50.0)
        Log::info("AssetManager: texture '%s' took %.0f ms to decode", filename.c_str(),
                  decodeMilliseconds);
    if (!decoded.ok)
    {
        Log::error("AssetManager: failed to load '%s', using the default checker",
                   filename.c_str());
        return getDefaultTexture();
    }
    decoded.desc.debugName = filename.c_str();

    TextureHandle handle = GPU::getSingleton().createTexture(decoded.desc);
    releaseDecodedTexture(decoded);

    if (!handle.valid())
    {
        Log::error("AssetManager: GPU upload failed for '%s', using the default checker",
                   filename.c_str());
        return getDefaultTexture();
    }

    return registerTextureEntry(filename, cacheKey, handle);
}

TextureHandle AssetManager::loadTextureAsync(const std::string& filename, ColorSpace space,
                                             bool generateMips, u32 mipLimit)
{
    const std::string cacheKey = textureCacheKey(filename, space, generateMips, mipLimit);
    auto cached = mLoadedTextures.find(cacheKey);
    if (cached != mLoadedTextures.end())
        return mTextures[cached->second].handle;

    // Opaque mid-grey, 1x1 - unique to this request. Never the shared default
    // checker: AsyncTextureLoader::processCompleted() rebuilds whatever GPU
    // object sits behind this exact handle once decoding finishes, and doing
    // that to the shared checker would corrupt it for every other user still
    // relying on it looking like a checker.
    const u32 placeholderColor = 0xFF808080u;
    TextureDesc placeholderDesc;
    placeholderDesc.type = TextureType::Tex2D;
    placeholderDesc.format = Format::RGBA8;
    placeholderDesc.width = 1;
    placeholderDesc.height = 1;
    placeholderDesc.mips = 1;
    placeholderDesc.usage = TextureSampled;
    placeholderDesc.data = &placeholderColor;
    placeholderDesc.debugName = "texture.async_placeholder";
    TextureHandle placeholder = GPU::getSingleton().createTexture(placeholderDesc);
    if (!placeholder.valid())
        return getDefaultTexture();

    registerTextureEntry(filename, cacheKey, placeholder);
    AsyncTextureLoader::getSingleton().enqueue(placeholder, resolveTextureFile(filename), space,
                                               generateMips, mipLimit);
    return placeholder;
}

void AssetManager::processAsyncTextureLoads()
{
    AsyncTextureLoader::getSingleton().processCompleted();
}

TextureHandle AssetManager::loadCubemap(const std::string faces[6], const std::string& cacheName,
                                        ColorSpace space, bool generateMips)
{
    const std::string cacheKey = textureCacheKey(cacheName, space, generateMips, 0) + "\x1f"
                                                                                     "cube";
    auto cached = mLoadedTextures.find(cacheKey);
    if (cached != mLoadedTextures.end())
        return mTextures[cached->second].handle;

    FileSystem& fs = FileSystem::getSingleton();
    Pixmap pixmaps[6];
    Pixmap* converted[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    const Pixmap* source[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

    auto releaseConverted = [&converted]()
    {
        for (Pixmap*& pixmap : converted)
        {
            delete pixmap;
            pixmap = nullptr;
        }
    };

    for (int i = 0; i < 6; ++i)
    {
        ByteArray bytes = fs.readBinary(faces[i]);
        if (bytes.empty() || bytes.size() > 0xFFFFFFFFu ||
            !pixmaps[i].load_from_memory(bytes.data(), static_cast<u32>(bytes.size())))
        {
            Log::error("AssetManager: failed to load cubemap face '%s'", faces[i].c_str());
            releaseConverted();
            return TextureHandle();
        }
        // Anything that is not already RGBA, not just the 3-component case:
        // the faces are packed below at a fixed four bytes per pixel, so a
        // one- or two-channel face would be read past its own buffer and
        // uploaded as a format it is not.
        converted[i] = pixmaps[i].components != 4 ? pixmaps[i].convert_to_rgba() : nullptr;
        source[i] = converted[i] ? converted[i] : &pixmaps[i];
    }

    const int width = source[0]->width;
    const int height = source[0]->height;
    if (width != height)
    {
        Log::error("AssetManager: cubemap face '%s' is not square (%dx%d)", faces[0].c_str(),
                   width, height);
        releaseConverted();
        return TextureHandle();
    }
    for (int i = 1; i < 6; ++i)
    {
        if (source[i]->width != width || source[i]->height != height)
        {
            Log::error("AssetManager: cubemap face '%s' is %dx%d, expected %dx%d like '%s'",
                       faces[i].c_str(), source[i]->width, source[i]->height, width, height,
                       faces[0].c_str());
            releaseConverted();
            return TextureHandle();
        }
    }

    // The GL backend uploads all six faces with one glTextureSubImage3D call
    // (depth = 6), so they have to sit back to back in one buffer, in GL's
    // +X -X +Y -Y +Z -Z order, before the GPU ever sees them.
    const usize faceBytes = static_cast<usize>(width) * static_cast<usize>(height) * 4;
    std::vector<u8> buffer(faceBytes * 6);
    for (int i = 0; i < 6; ++i)
        std::memcpy(buffer.data() + i * faceBytes, source[i]->pixels, faceBytes);

    TextureDesc desc;
    desc.type = TextureType::TexCube;
    desc.format = formatFor(4, space);
    desc.width = static_cast<u32>(width);
    desc.height = static_cast<u32>(height);
    desc.depth = 6;
    desc.mips = generateMips ? 0 : 1; // 0 = full chain, per TextureDesc's own convention
    desc.usage = TextureSampled;
    desc.data = buffer.data();
    desc.debugName = cacheName.c_str();

    TextureHandle handle = GPU::getSingleton().createTexture(desc);
    releaseConverted();

    if (!handle.valid())
    {
        Log::error("AssetManager: GPU upload failed for cubemap '%s'", cacheName.c_str());
        return TextureHandle();
    }

    TextureEntry entry;
    entry.name = cacheName;
    entry.cacheKey = cacheKey;
    entry.handle = handle;
    const usize index = mTextures.size();
    mTextures.push_back(entry);
    mLoadedTextures[cacheKey] = index;
    mTexturesByName.emplace(cacheName, index);
    mTexturesByHandle[packHandle(handle)] = index;

    return handle;
}

TextureHandle AssetManager::loadCubemap(const std::string& baseName, ColorSpace space,
                                        bool generateMips)
{
    std::string faces[6];
    if (tryCubemapConvention(baseName, kConventionRTLF, faces) ||
        tryCubemapConvention(baseName, kConventionPXNX, faces) ||
        tryCubemapConvention(baseName, kConventionSkyBox, faces))
    {
        return loadCubemap(faces, baseName, space, generateMips);
    }

    Log::error("AssetManager: no cubemap faces found for '%s'", baseName.c_str());
    return TextureHandle();
}

std::vector<std::string> AssetManager::listCubemaps(const std::string& directory) const
{
    std::vector<std::string> names;
    FileSystem& files = FileSystem::getSingleton();

    // `directory` is relative to a search path, the way a texture name is,
    // but listing needs a real path - so the search paths are walked here.
    // The same name can turn up under more than one of them, hence the
    // dedupe at the end.
    for (const std::string& root : files.getSearchPaths())
    {
        const std::string path = root.empty() ? directory : root + "/" + directory;
        if (!files.isDirectory(path))
            continue;

        for (const FileSystem::DirEntry& entry : files.listDirectory(path))
        {
            const std::string base = directory + "/" + entry.name;
            std::string faces[6];

            if (entry.isDirectory)
            {
                if (tryCubemapConvention(base, kConventionSkyBox, faces))
                    names.push_back(base);
                continue;
            }

            // Match only the first face of each convention: the other five
            // name the same cubemap, and matching them too would list it six
            // times over.
            for (const CubemapConvention* convention : {&kConventionRTLF, &kConventionPXNX})
            {
                const std::string suffix = convention->suffix[0];
                if (base.size() <= suffix.size() ||
                    base.compare(base.size() - suffix.size(), suffix.size(), suffix) != 0)
                    continue;
                const std::string stem = base.substr(0, base.size() - suffix.size());
                if (tryCubemapConvention(stem, *convention, faces))
                    names.push_back(stem);
                break;
            }
        }
    }

    std::sort(names.begin(), names.end());
    names.erase(std::unique(names.begin(), names.end()), names.end());
    return names;
}

TextureHandle AssetManager::createTexture(const std::string& name, const TextureDesc& desc)
{
    TextureHandle existing = getTexture(name);
    if (existing.valid())
        return existing;

    TextureHandle handle = GPU::getSingleton().createTexture(desc);
    if (!handle.valid())
        return handle;

    TextureEntry entry;
    entry.name = name;
    entry.handle = handle;
    const usize index = mTextures.size();
    mTextures.push_back(entry);
    mTexturesByName[name] = index;
    mTexturesByHandle[packHandle(handle)] = index;
    return handle;
}

TextureHandle AssetManager::getDefaultTexture()
{
    if (mDefaultTexture.valid())
        return mDefaultTexture;

    constexpr u32 kSize = 16;
    constexpr u32 kCheck = 4;
    std::vector<u32> pixels(kSize * kSize);
    for (u32 y = 0; y < kSize; ++y)
    {
        for (u32 x = 0; x < kSize; ++x)
        {
            const bool odd = ((x / kCheck) + (y / kCheck)) & 1u;
            pixels[y * kSize + x] = odd ? 0xFFFF00FFu : 0xFF000000u; // magenta / black, RGBA8
        }
    }

    TextureDesc desc;
    desc.type = TextureType::Tex2D;
    desc.format = Format::RGBA8;
    desc.width = kSize;
    desc.height = kSize;
    desc.mips = 0; // full chain - storage is immutable, 1 here would mean forever
    desc.usage = TextureSampled;
    desc.data = pixels.data();
    desc.debugName = "texture.default_checker";

    mDefaultTexture = createTexture("__default_checker__", desc);
    if (mDefaultTexture.valid())
        GPU::getSingleton().generateMips(mDefaultTexture);
    return mDefaultTexture;
}

TextureHandle AssetManager::getTexture(const std::string& name) const
{
    auto it = mTexturesByName.find(name);
    if (it == mTexturesByName.end())
        return TextureHandle();
    return mTextures[it->second].handle;
}

TextureHandle AssetManager::getTextureByIndex(usize index) const
{
    if (index >= mTextures.size())
        return TextureHandle();
    return mTextures[index].handle;
}

usize AssetManager::textureCount() const
{
    return mTextures.size();
}

const std::string& AssetManager::textureName(TextureHandle handle) const
{
    static const std::string kEmpty;
    auto it = mTexturesByHandle.find(packHandle(handle));
    if (it == mTexturesByHandle.end())
        return kEmpty;
    return mTextures[it->second].name;
}

void AssetManager::destroyTexture(TextureHandle handle)
{
    auto it = mTexturesByHandle.find(packHandle(handle));
    if (it == mTexturesByHandle.end())
        return;

    const usize index = it->second;
    if (mDefaultTexture == handle)
        mDefaultTexture = TextureHandle();

    if (!mTextures[index].cacheKey.empty())
        mLoadedTextures.erase(mTextures[index].cacheKey);
    auto named = mTexturesByName.find(mTextures[index].name);
    if (named != mTexturesByName.end() && named->second == index)
    {
        mTexturesByName.erase(named);
        for (usize candidate = 0; candidate < mTextures.size(); ++candidate)
        {
            if (candidate != index && mTextures[candidate].handle.valid() &&
                mTextures[candidate].name == mTextures[index].name)
            {
                mTexturesByName[mTextures[index].name] = candidate;
                break;
            }
        }
    }
    mTexturesByHandle.erase(it);

    GPU::getSingleton().destroy(handle);

    // Tombstone rather than swap-erase: indices already handed out by
    // getByIndex() must stay valid for entries that were not destroyed.
    mTextures[index] = TextureEntry();
}

void AssetManager::destroyAllTextures()
{
    GPU& gpu = GPU::getSingleton();
    for (TextureEntry& entry : mTextures)
    {
        if (entry.handle.valid())
            gpu.destroy(entry.handle);
    }
    mTextures.clear();
    mTexturesByName.clear();
    mLoadedTextures.clear();
    mTexturesByHandle.clear();
    mDefaultTexture = TextureHandle();

    destroyAllSamplers();
}

SamplerHandle AssetManager::getSampler(const SamplerDesc& desc)
{
    for (const SamplerEntry& entry : mSamplers)
    {
        if (sameSampler(entry.desc, desc))
            return entry.handle;
    }

    SamplerHandle handle = GPU::getSingleton().createSampler(desc);
    if (!handle.valid())
        return handle;

    mSamplers.push_back({desc, handle});
    return handle;
}

void AssetManager::destroyAllSamplers()
{
    GPU& gpu = GPU::getSingleton();
    for (SamplerEntry& entry : mSamplers)
        gpu.destroy(entry.handle);
    mSamplers.clear();
}

} // namespace Radion
