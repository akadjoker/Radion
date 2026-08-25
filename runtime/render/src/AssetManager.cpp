#include "PCH.h"

#include "AssetManager.h"

#include "B3DImporter.h"
#include "C3DSImporter.h"
#include "FileSystem.h"
#if RADION_ENABLE_FBX_LOADER
#include "FbxImporter.h"
#endif
#include "GltfImporter.h"
#include "MS3DImporter.h"
#include "MaterialManager.h"
#include "ObjImporter.h"
#include "OgreMeshImporter.h"
#include "RadionMeshImporter.h"

namespace Radion
{

namespace
{

constexpr int kMaxIncludeDepth = 16;

// A line of the form `#include "path"` - leading whitespace before the '#' is
// allowed, anything past the closing quote is ignored rather than rejected so
// a trailing comment does not break it.
bool parseInclude(const std::string& line, std::string& path)
{
    usize i = 0;
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i])))
        ++i;

    static const std::string kDirective = "#include";
    if (line.compare(i, kDirective.size(), kDirective) != 0)
        return false;
    i += kDirective.size();

    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i])))
        ++i;
    if (i >= line.size() || line[i] != '"')
        return false;
    ++i;

    const usize end = line.find('"', i);
    if (end == std::string::npos)
        return false;

    path = line.substr(i, end - i);
    return true;
}

} // namespace

// ----------------------------------------------------------------- lifetime

AssetManager::AssetManager()
{
    // Registered once, here, instead of every demo building its own
    // MeshLoader and remembering which importer(s) a mesh file needs -
    // MeshLoader owns what it is handed, so these are never deleted anywhere
    // else.
    mMeshLoader.addImporter(new RadionMeshImporter());
    mMeshLoader.addImporter(new ObjImporter());
    mMeshLoader.addImporter(new OgreMeshImporter());
    mMeshLoader.addImporter(new B3DImporter());
    mMeshLoader.addImporter(new C3DSImporter());
    mMeshLoader.addImporter(new MS3DImporter());
    mMeshLoader.addImporter(new GltfImporter());
#if RADION_ENABLE_FBX_LOADER
    mMeshLoader.addImporter(new FbxImporter());
#endif
}

bool AssetManager::importMesh(const std::string& filename, MeshData& out)
{
    return mMeshLoader.load(filename, out);
}

namespace
{
std::string lowerExtension(const std::string& file)
{
    const usize dot = file.find_last_of('.');
    if (dot == std::string::npos)
        return std::string();
    std::string extension = file.substr(dot + 1);
    for (char& c : extension)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return extension;
}
} // namespace

bool AssetManager::importSkeleton(const std::string& file, Skeleton& skeleton)
{
    const std::string extension = lowerExtension(file);
    if (extension == "rskel")
        return RadionSkeletonIO::loadSkeleton(file, skeleton);
    FileSystem& files = FileSystem::getSingleton();
#if RADION_ENABLE_FBX_LOADER
    if (extension == "fbx")
        return loadFbxSkeleton(file, files, skeleton);
#endif
    if (extension == "b3d")
        return loadB3DSkeleton(file, files, skeleton);
    if (extension == "ms3d")
        return loadMS3DSkeleton(file, files, skeleton);
    if (extension == "gltf" || extension == "glb")
        return loadGltfSkeleton(file, files, skeleton);
    Log::error("AssetManager: no skeleton importer for '%s'", file.c_str());
    return false;
}

bool AssetManager::importAnimation(const std::string& file, const Skeleton& skeleton,
                                   AnimationClip& clip, bool keepRootMotion)
{
    const std::string extension = lowerExtension(file);
    if (extension == "ranim")
        return RadionSkeletonIO::loadAnimation(file, skeleton, clip);
    FileSystem& files = FileSystem::getSingleton();
#if RADION_ENABLE_FBX_LOADER
    if (extension == "fbx")
        return loadFbxAnimation(file, files, skeleton, clip, keepRootMotion);
#endif
    if (extension == "b3d")
        return loadB3DAnimation(file, files, skeleton, clip);
    if (extension == "ms3d")
        return loadMS3DAnimation(file, files, skeleton, clip);
    if (extension == "gltf" || extension == "glb")
        return loadGltfAnimation(file, files, skeleton, clip);
    Log::error("AssetManager: no animation importer for '%s'", file.c_str());
    return false;
}

AssetManager& AssetManager::getSingleton()
{
    static AssetManager instance;
    return instance;
}

AssetManager& Assets()
{
    return AssetManager::getSingleton();
}

void AssetManager::shutdown()
{
    // Mesh decoding is CPU-only, but its worker may still be reading the
    // filesystem while the engine tears down the asset registries.
    if (mMeshInFlight.result.valid())
        mMeshInFlight.result.wait();
    mMeshInFlight = PendingMesh();
    mQueuedMeshes.clear();

    // Meshes first: releasing one also releases its materials' param buffers.
    // Textures then take their samplers with them, and the shader text last -
    // it owns nothing on the GPU, only the strings the pipelines were built
    // from.
    destroyAllMeshes();
    destroyAllTextures();
    reloadAllShaders();
    // AssetManager is a singleton and outlives Engine/GPUDevice restarts. A
    // named target left behind here is a handle from the device that just
    // shut down; the next GPUDevice starts its pools from zero, so the same
    // index/generation can validly resolve to an unrelated texture on the new
    // device instead of merely being stale. resolveRenderTarget() also
    // confirms the handle against the live device before returning it, but
    // clearing here is what keeps a producer that forgets to republish from
    // resolving to someone else's texture in the first place.
    mNamedTargets.clear();
}

// ------------------------------------------------------------------ shaders

std::string AssetManager::expandShader(const std::string& filename, int depth)
{
    if (depth > kMaxIncludeDepth)
    {
        Log::error("AssetManager: '%s' - #include nesting too deep (cycle?)", filename.c_str());
        return std::string();
    }

    const std::string text = FileSystem::getSingleton().readText(filename);
    if (text.empty())
    {
        Log::error("AssetManager: could not read shader '%s'", filename.c_str());
        return std::string();
    }

    std::string result;
    result.reserve(text.size());

    usize lineStart = 0;
    while (lineStart <= text.size())
    {
        const usize lineEnd = text.find('\n', lineStart);
        const bool last = lineEnd == std::string::npos;
        const std::string line =
            text.substr(lineStart, last ? std::string::npos : lineEnd - lineStart);

        std::string includePath;
        if (parseInclude(line, includePath))
        {
            // Relative to the file doing the including, the way every other
            // #include works. Without it a shared chunk could only be included
            // from a shader that happens to sit in the working directory.
            const usize directory = filename.find_last_of('/');
            if (directory != std::string::npos && includePath.front() != '/')
                includePath = filename.substr(0, directory + 1) + includePath;
            result += expandShader(includePath, depth + 1);
        }
        else
            result += line;
        result += '\n';

        if (last)
            break;
        lineStart = lineEnd + 1;
    }

    return result;
}

const std::string& AssetManager::loadShader(const std::string& filename)
{
    auto it = mShaderSources.find(filename);
    if (it != mShaderSources.end())
        return it->second;

    return mShaderSources.emplace(filename, expandShader(filename, 0)).first->second;
}

void AssetManager::reloadShader(const std::string& filename)
{
    // Erasing only this exact key never touches the shaders that #include
    // it: their already-expanded text stays cached, so editing a shared
    // .glsl chunk and hot-reloading its parent claimed success while the
    // program that actually ran had not changed. Nothing here tracks the
    // include graph, so the correct minimal fix is the same one
    // reloadAllShaders() already does - clear every expanded source, not
    // just one filename's.
    (void)filename;
    reloadAllShaders();
}

void AssetManager::reloadAllShaders()
{
    mShaderSources.clear();

    // Clearing the source text was never the whole story: MaterialManager's
    // pipeline cache and every Material's own `pipeline` field still pointed
    // at programs compiled from the old text, so "reload" could report
    // success while the same GLSL kept running. Destroying the cache is
    // enough to fix that without walking every material by hand -
    // resolvePipeline() runs once per material every frame already (see
    // Scene::buildRenderList()), misses the now-empty cache, and recompiles
    // from the freshly-reloaded source on its own.
    MaterialManager::getSingleton().destroyAllPipelines();
}

// ------------------------------------------------------------ named targets

void AssetManager::publishRenderTarget(u32 nameHash, TextureHandle texture)
{
    mNamedTargets[nameHash] = texture;
}

void AssetManager::publishRenderTarget(const char* name, TextureHandle texture)
{
    publishRenderTarget(hashName(name), texture);
}

TextureHandle AssetManager::resolveRenderTarget(u32 nameHash) const
{
    auto it = mNamedTargets.find(nameHash);
    if (it == mNamedTargets.end())
        return TextureHandle();

    // A generation match against a texture pool that was recreated (a device
    // restart, a resize that destroyed and rebuilt the target) is not proof
    // the handle still names the same texture the publisher meant - only that
    // some texture happens to sit at that index/generation now. Confirming
    // against the live device is what tells a resolved-but-wrong handle apart
    // from a genuinely resolved one; tryGet() rather than getSingleton()
    // because this can be called before a device exists at all.
    GPU* gpu = GPU::tryGet();
    TextureDesc info;
    if (!gpu || !gpu->textureInfo(it->second, info))
        return TextureHandle();

    return it->second;
}

} // namespace Radion
