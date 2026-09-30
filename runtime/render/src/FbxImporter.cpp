// FBX importer built on ofbx (runtime/render/src/ofbx.h).
//
// Geometry, materials, skeleton and animation are loaded through the same three
// entry points the other importers expose: FbxImporter::import() for the mesh,
// loadFbxSkeleton() for the bone hierarchy, and loadFbxAnimation() for the
// first animation stack.
//
// ofbx triangulates the polygon data for us. Matrices coming from ofbx are
// column-major double[16] (same memory layout as Math::mat4), so they are copied
// component by component into Math::mat4. FBX's texture V origin is at the
// bottom of the image, same as OBJ, so UVs are flipped to match the engine's
// top-left convention.

#include "PCH.h"

#include "FbxImporter.h"

#include "ByteArray.h"
#include "FileSystem.h"
#include "Log.h"
#include "Material.h"
#include "Math.h"
#include "Pixmap.h"
#include "Skeleton.h"
#include "ofbx.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "Math.h"
#include "Math.h"
#include "Math.h"
#include <limits>
#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif
#include "Math.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Radion
{

namespace
{

std::string directoryOf(const std::string& filename)
{
    const usize slash = filename.find_last_of("/\\");
    return slash == std::string::npos ? std::string() : filename.substr(0, slash + 1);
}

std::string joinPath(const std::string& directory, const std::string& name)
{
    if (name.empty() || directory.empty())
        return name;
    if (name[0] == '/' || name[0] == '\\' || (name.size() > 1 && name[1] == ':'))
        return name;
    return directory + name;
}

// A texture reference embedded in an FBX is routinely the file's ORIGINAL
// absolute path on whoever exported it, not one relative to this file -
// Mixamo's own temp directories are a common example, and joinPath() alone
// turns that into a path that never resolves. LumixEngine's own importer
// (model_importer.cpp's findTexture()) hits the exact same thing and falls
// back to the bare filename beside the mesh, then that mesh's own textures/
// subfolder - this does the same, minus the multi-extension guessing Lumix
// also does, since the embedded name already carries the right one here.
std::string resolveTextureFile(FileSystem& files, const std::string& directory,
                               const std::string& embeddedPath)
{
    if (embeddedPath.empty())
        return std::string();
    const std::string joined = joinPath(directory, embeddedPath);
    if (files.exists(joined))
        return joined;

    const usize slash = embeddedPath.find_last_of("/\\");
    const std::string baseName =
        slash == std::string::npos ? embeddedPath : embeddedPath.substr(slash + 1);
    const std::string besideMesh = directory + baseName;
    if (files.exists(besideMesh))
        return besideMesh;
    const std::string inTextures = directory + "textures/" + baseName;
    if (files.exists(inTextures))
        return inTextures;

    // Nothing resolved - keep the joined path so the eventual "file not
    // found" still names something a user can recognise and go fix.
    return joined;
}

std::string stem(const std::string& path)
{
    const usize slash = path.find_last_of("/\\");
    const usize base = slash == std::string::npos ? 0 : slash + 1;
    const usize dot = path.find_last_of('.');
    if (dot == std::string::npos || dot < base)
        return path.substr(base);
    return path.substr(base, dot - base);
}

// An "Embed Media" FBX export carries its textures as Video objects inside
// the binary itself rather than as sibling files - resolveTextureFile()
// alone leaves those as a name nothing on disk answers to (routinely the
// exporting machine's own temp path, e.g. a Mixamo download's
// ".../yaku_j_ignite.fbm/...png"). This decodes the embedded bytes once
// (Pixmap already reads PNG/JPEG straight from memory) and writes them out
// as a real file next to the source FBX, so the rest of the pipeline never
// has to know the texture did not ship as its own file - same idea as
// GltfImporter's resolveImageFile() for a .glb's buffer-embedded images.
// `realDirectory` is the source FBX's own directory already resolved to a
// writable disk path (FileSystem::writeBinary()/Pixmap::save() need one,
// unlike reading, which resolves search paths on its own).
std::string extractEmbeddedTexture(FileSystem& files, const std::string& realDirectory,
                                   const std::string& textureFolder, const ofbx::Texture* texture)
{
    if (!texture || realDirectory.empty() || textureFolder.empty())
        return std::string();

    const ofbx::DataView embedded = texture->getEmbeddedData();
    // The FBX 'R' (raw binary) property hands back its own 4-byte length
    // prefix ahead of the payload (see TextureImpl::getEmbeddedData() in
    // ofbx.cpp) - the actual image starts 4 bytes in.
    if (!embedded.begin || embedded.end <= embedded.begin + 4)
        return std::string();

    const ofbx::DataView nameView =
        texture->getRelativeFileName().begin ? texture->getRelativeFileName() : texture->getFileName();
    std::string baseName = nameView.begin ? std::string(nameView.begin, nameView.end) : std::string();
    const usize slash = baseName.find_last_of("/\\");
    if (slash != std::string::npos)
        baseName = baseName.substr(slash + 1);
    if (baseName.empty())
        baseName = "embedded_" + std::to_string(reinterpret_cast<uintptr_t>(embedded.begin));

    const std::string relativeName = textureFolder + "/" + baseName;
    const std::string realPath = realDirectory + relativeName;
    if (!files.exists(realPath))
    {
        files.createDirectory(realDirectory + textureFolder);
        Pixmap pixmap;
        if (!pixmap.load_from_memory(embedded.begin + 4,
                                     static_cast<u32>(embedded.end - embedded.begin - 4)) ||
            !pixmap.save(realPath.c_str()))
            return std::string();
    }
    return realDirectory + relativeName;
}

u32 hashName(const std::string& name)
{
    u32 hash = 2166136261u;
    for (char c : name)
    {
        hash ^= static_cast<u8>(c);
        hash *= 16777619u;
    }
    return hash;
}

// ofbx stores matrices as column-major double[16]. Copy them straight into a
// Math::mat4 (also column-major) without any transpose.
Math::mat4 toMat4(const ofbx::Matrix& m)
{
    Math::mat4 out;
    for (int i = 0; i < 16; ++i)
        Math::value_ptr(out)[i] = static_cast<f32>(m.m[i]);
    return out;
}

Math::vec3 toVec3(const ofbx::Vec3& v)
{
    return Math::vec3(static_cast<f32>(v.x), static_cast<f32>(v.y), static_cast<f32>(v.z));
}

Math::vec2 toVec2(const ofbx::Vec2& v)
{
    return Math::vec2(static_cast<f32>(v.x), static_cast<f32>(v.y));
}

f32 sampleScalarChannel(const std::vector<f32>& times, const std::vector<f32>& values, f32 time,
                        f32 fallback)
{
    const usize count = std::min(times.size(), values.size());
    if (count == 0)
        return fallback;
    if (count == 1 || time <= times.front())
        return values.front();
    if (time >= times[count - 1])
        return values[count - 1];
    const usize next = static_cast<usize>(
        std::upper_bound(times.begin(), times.begin() + count, time) - times.begin());
    const usize previous = next - 1;
    const f32 span = times[next] - times[previous];
    const f32 amount = span > 1e-6f ? (time - times[previous]) / span : 0.0f;
    return Math::mix(values[previous], values[next], amount);
}

void appendTimes(std::vector<f32>& destination, const std::vector<f32>& source)
{
    destination.insert(destination.end(), source.begin(), source.end());
}

void sortUniqueTimes(std::vector<f32>& times)
{
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end(),
                            [](f32 a, f32 b)
                            {
                                return std::fabs(a - b) < 1e-6f;
                            }),
                times.end());
}

// Keep the four strongest skin weights for a vertex, same as B3DImporter.
struct WeightSlots
{
    s32 ids[4] = {-1, -1, -1, -1};
    f32 weights[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    void add(s32 bone, f32 weight)
    {
        if (bone < 0 || weight <= 0.0f)
            return;
        if (weight > weights[0])
        {
            ids[3] = ids[2];
            weights[3] = weights[2];
            ids[2] = ids[1];
            weights[2] = weights[1];
            ids[1] = ids[0];
            weights[1] = weights[0];
            ids[0] = bone;
            weights[0] = weight;
        }
        else if (weight > weights[1])
        {
            ids[3] = ids[2];
            weights[3] = weights[2];
            ids[2] = ids[1];
            weights[2] = weights[1];
            ids[1] = bone;
            weights[1] = weight;
        }
        else if (weight > weights[2])
        {
            ids[3] = ids[2];
            weights[3] = weights[2];
            ids[2] = bone;
            weights[2] = weight;
        }
        else if (weight > weights[3])
        {
            ids[3] = bone;
            weights[3] = weight;
        }
    }
};

// Parse one ofbx scene from raw bytes. The caller becomes the owner and must
// call scene->destroy() when done.
ofbx::IScene* parseFbx(const u8* data, usize size)
{
    return ofbx::load(data, static_cast<int>(size),
                      static_cast<ofbx::u64>(ofbx::LoadFlags::TRIANGULATE));
}

// Get a stable name out of an ofbx object, falling back to its id.
std::string objectName(const ofbx::Object& obj)
{
    if (obj.name[0] != '\0')
        return std::string(obj.name);
    return "object_" + std::to_string(obj.id);
}

// Mixamo and Blender spell a few of the same bones differently. Canonicalize
// the Blender spellings to the Mixamo ones so a mesh and its animation,
// exported by different tools, still match by name.
std::string canonicalBoneName(const std::string& name)
{
    if (name == "Chest")
        return "Spine2";
    if (name == "LeftToes")
        return "LeftToeBase";
    if (name == "RightToes")
        return "RightToeBase";
    return name;
}

struct FbxBoneMap
{
    std::vector<const ofbx::Object*> bones;
    std::unordered_map<u64, s32> indexById;
};

bool isSkeletonNode(const ofbx::Object* object)
{
    if (!object)
        return false;
    return object->getType() == ofbx::Object::Type::LIMB_NODE ||
           object->getType() == ofbx::Object::Type::NULL_NODE;
}

// Armatures commonly contain Null nodes between LimbNodes. Dropping one
// changes both the parent index and local bind transform.
bool buildBoneMap(const ofbx::IScene& scene, FbxBoneMap& result)
{
    std::unordered_set<u64> required;
    const auto requireChain = [&](const ofbx::Object* leaf)
    {
        for (const ofbx::Object* object = leaf; isSkeletonNode(object);
             object = object->getParent())
            required.insert(object->id);
    };

    for (int i = 0; i < scene.getAllObjectCount(); ++i)
    {
        const ofbx::Object* object = scene.getAllObjects()[i];
        if (object && object->getType() == ofbx::Object::Type::LIMB_NODE)
            requireChain(object);
    }
    for (int mi = 0; mi < scene.getMeshCount(); ++mi)
    {
        const ofbx::Mesh* mesh = scene.getMesh(mi);
        const ofbx::Geometry* geometry = mesh ? mesh->getGeometry() : nullptr;
        const ofbx::Skin* skin = geometry ? geometry->getSkin() : nullptr;
        if (!skin)
            continue;
        for (int ci = 0; ci < skin->getClusterCount(); ++ci)
        {
            const ofbx::Cluster* cluster = skin->getCluster(ci);
            if (cluster)
                requireChain(cluster->getLink());
        }
    }

    result = FbxBoneMap();
    while (result.bones.size() < required.size())
    {
        bool progressed = false;
        for (int i = 0; i < scene.getAllObjectCount(); ++i)
        {
            const ofbx::Object* object = scene.getAllObjects()[i];
            if (!object || required.find(object->id) == required.end() ||
                result.indexById.find(object->id) != result.indexById.end())
                continue;
            const ofbx::Object* parent = object->getParent();
            if (isSkeletonNode(parent) && required.find(parent->id) != required.end() &&
                result.indexById.find(parent->id) == result.indexById.end())
                continue;
            if (result.bones.size() >= 256)
                return false;
            result.indexById[object->id] = static_cast<s32>(result.bones.size());
            result.bones.push_back(object);
            progressed = true;
        }
        if (!progressed)
            return false;
    }
    return true;
}

// Find the first animated bone that carries locomotion. Start at the top of the
// hierarchy and skip common dummy root nodes (Reference, root, RootNode). The
// first non-dummy bone with a position track is treated as the functional root
// for in-place conversion.
s32 findFunctionalRoot(const FbxBoneMap& boneMap, const std::vector<s32>& parentAnim,
                       const std::vector<s32>& animToSkeleton,
                       const std::vector<BoneTrack>& tracks)
{
    const usize count = parentAnim.size();
    std::vector<bool> placed(count, false);
    std::vector<s32> order;
    order.reserve(count);
    while (order.size() < count)
    {
        bool progressed = false;
        for (usize i = 0; i < count; ++i)
        {
            if (placed[i])
                continue;
            if (parentAnim[i] < 0 || placed[static_cast<usize>(parentAnim[i])])
            {
                placed[i] = true;
                order.push_back(static_cast<s32>(i));
                progressed = true;
            }
        }
        if (!progressed)
            break;
    }

    for (s32 a : order)
    {
        const usize au = static_cast<usize>(a);
        if (animToSkeleton[au] < 0)
            continue;
        const usize skelIndex = static_cast<usize>(animToSkeleton[au]);
        if (tracks[skelIndex].times.empty())
            continue;
        const std::string name = objectName(*boneMap.bones[au]);
        if (name == "Reference" || name == "RootNode" || name == "root" || name == "Root")
            continue;
        return a;
    }
    return -1;
}

} // namespace

// ------------------------------------------------------------------ importer

bool FbxImporter::supports(const char* extension) const
{
    return extension && std::strcmp(extension, "fbx") == 0;
}

bool FbxImporter::import(const std::string& filename, ByteArray& data, FileSystem& files,
                         MeshData& mesh)
{
    ofbx::IScene* scene = parseFbx(data.data(), data.size());
    if (!scene)
    {
        Log::error("FbxImporter: '%s' failed to parse (%s)", filename.c_str(), ofbx::getError());
        return false;
    }

    mesh.clear();

    const std::string directory = directoryOf(filename);
    // Only needed for an embedded texture's own extraction
    // (extractEmbeddedTexture()) - an external one is still read the normal
    // search-path-aware way via `directory` above, so this staying empty (an
    // unresolvable `filename`) only means an embedded texture falls back to
    // importing blank, not that nothing imports at all.
    const std::string resolvedFilename = files.resolve(filename);
    const std::string realDirectory =
        directoryOf(resolvedFilename.empty() ? filename : resolvedFilename);
    const std::string textureFolder = stem(filename) + "_textures";

    // One scene-global bone map shared by every mesh. Cluster order is local
    // to each Skin and is not a skeleton index.
    FbxBoneMap boneMap;
    if (!buildBoneMap(*scene, boneMap))
    {
        Log::error("FbxImporter: '%s' has an invalid hierarchy or more than 256 bones",
                   filename.c_str());
        scene->destroy();
        return false;
    }

    // Pre-pass: does any mesh in the file have a skin? If yes, we allocate a
    // skin vertex for every vertex so the arrays stay in lockstep.
    bool hasSkin = false;
    for (int mi = 0; mi < scene->getMeshCount() && !hasSkin; ++mi)
    {
        const ofbx::Mesh* fbxMesh = scene->getMesh(mi);
        const ofbx::Geometry* geom = fbxMesh ? fbxMesh->getGeometry() : nullptr;
        if (geom && geom->getSkin() && geom->getSkin()->getClusterCount() > 0)
            hasSkin = true;
    }

    // Material bookkeeping across the whole file. Different meshes can reuse the
    // same ofbx Material object; we map each unique pointer to one slot.
    std::unordered_map<const ofbx::Material*, u32> materialSlot;
    const auto slotForMaterial = [&](const ofbx::Material* material) -> u32
    {
        if (!material)
            return 0;

        auto found = materialSlot.find(material);
        if (found != materialSlot.end())
            return found->second;

        const u32 slot = static_cast<u32>(mesh.materials.size());
        materialSlot[material] = slot;

        Material out;
        out.name = objectName(*material);
        out.nameHash = hashName(out.name);

        const ofbx::Color diffuse = material->getDiffuseColor();
        out.params.baseColor = Math::vec4(static_cast<f32>(diffuse.r), static_cast<f32>(diffuse.g),
                                         static_cast<f32>(diffuse.b), 1.0f);

        const ofbx::Texture* albedo = material->getTexture(ofbx::Texture::DIFFUSE);
        std::string albedoFile = extractEmbeddedTexture(files, realDirectory, textureFolder, albedo);
        if (albedoFile.empty())
        {
            std::string albedoName;
            if (albedo && albedo->getRelativeFileName().begin)
                albedoName = std::string(albedo->getRelativeFileName().begin,
                                         albedo->getRelativeFileName().end);
            if (albedoName.empty() && albedo && albedo->getFileName().begin)
                albedoName = std::string(albedo->getFileName().begin, albedo->getFileName().end);
            albedoFile = resolveTextureFile(files, directory, albedoName);
        }

        const ofbx::Texture* normal = material->getTexture(ofbx::Texture::NORMAL);
        std::string normalFile = extractEmbeddedTexture(files, realDirectory, textureFolder, normal);
        if (normalFile.empty())
        {
            std::string normalName;
            if (normal && normal->getRelativeFileName().begin)
                normalName = std::string(normal->getRelativeFileName().begin,
                                         normal->getRelativeFileName().end);
            if (normalName.empty() && normal && normal->getFileName().begin)
                normalName = std::string(normal->getFileName().begin, normal->getFileName().end);
            normalFile = resolveTextureFile(files, directory, normalName);
        }

        mesh.materials.push_back(out);
        mesh.materialTextureFiles.push_back(albedoFile);
        mesh.materialNormalFiles.push_back(normalFile);
        return slot;
    };

    for (int mi = 0; mi < scene->getMeshCount(); ++mi)
    {
        const ofbx::Mesh* fbxMesh = scene->getMesh(mi);
        const ofbx::Geometry* geom = fbxMesh ? fbxMesh->getGeometry() : nullptr;
        if (!geom)
            continue;

        const int vertexCount = geom->getVertexCount();
        const int indexCount = geom->getIndexCount();
        if (vertexCount <= 0 || indexCount <= 0)
            continue;

        const ofbx::Vec3* srcPositions = geom->getVertices();
        const ofbx::Vec3* srcNormals = geom->getNormals();
        const ofbx::Vec3* srcTangents = geom->getTangents();
        const ofbx::Vec2* srcUvs = geom->getUVs(0);
        const int* srcIndices = geom->getFaceIndices();
        const int* srcMaterials = geom->getMaterials();
        const ofbx::Skin* skin = geom->getSkin();
        const bool skinned = skin && skin->getClusterCount() > 0;
        // Start with the mesh node transform; attachments may replace it
        // with a transform relative to their bone below.
        Math::mat4 nodeXform = toMat4(fbxMesh->getGlobalTransform());
        s32 attachmentBone = -1;
        if (!skinned && hasSkin)
        {
            // Facial meshes can be exported as separate objects with an
            // Armature modifier but without a usable FBX cluster. Never guess
            // their bone from position: that changes with each animation.
            const std::string meshName = objectName(*fbxMesh);
            const bool facial = meshName.find("Eye") != std::string::npos ||
                                meshName.find("eye") != std::string::npos ||
                                meshName.find("Eyelid") != std::string::npos ||
                                meshName.find("Teeth") != std::string::npos ||
                                meshName.find("Tongue") != std::string::npos;
            const char* preferred = nullptr;
            if (meshName.find("Le_Eye") != std::string::npos ||
                meshName.find("LeftEye") != std::string::npos)
                preferred = "LeftEye";
            else if (meshName.find("Ri_Eye") != std::string::npos ||
                     meshName.find("RightEye") != std::string::npos)
                preferred = "RightEye";
            else if (meshName.find("Teeth") != std::string::npos ||
                     meshName.find("Tongue") != std::string::npos)
                preferred = "Jaw";
            else if (facial)
                preferred = "Head";
            const ofbx::Object* attachment = nullptr;
            for (const ofbx::Object* candidate : boneMap.bones)
            {
                const auto it = boneMap.indexById.find(candidate->id);
                if (preferred && it != boneMap.indexById.end() && objectName(*candidate) == preferred)
                {
                    attachmentBone = it->second;
                    attachment = candidate;
                    break;
                }
            }
            // Non-facial unskinned attachments may still be children of a bone.
            if (!attachment)
                for (const ofbx::Object* parent = fbxMesh->getParent(); parent;
                     parent = parent->getParent())
                {
                    const auto it = boneMap.indexById.find(parent->id);
                    if (it != boneMap.indexById.end())
                    {
                        attachmentBone = it->second;
                        attachment = parent;
                        break;
                    }
                }
            if (attachment)
                nodeXform = Math::inverse(toMat4(attachment->getGlobalTransform())) *
                            toMat4(fbxMesh->getGlobalTransform());
        }

        // Bake static meshes with their node transform. Skinned vertices stay
        // in the common mesh-local bind space used by the skin palette.
        if (skinned)
            nodeXform = Math::mat4(1.0f);
        const Math::mat4 geomXform = toMat4(fbxMesh->getGeometricMatrix());
        Math::mat4 finalXform = nodeXform * geomXform;
        bool finiteTransform = true;
        for (int m = 0; m < 16; ++m)
            finiteTransform = finiteTransform && std::isfinite(Math::value_ptr(finalXform)[m]);
        if (!finiteTransform)
        {
            Log::error("FbxImporter: '%s' has an invalid mesh transform; using identity",
                       filename.c_str());
            finalXform = Math::mat4(1.0f);
        }
        const Math::mat3 normalMatrix =
            skinned ? Math::mat3(1.0f) : Math::mat3(Math::transpose(Math::inverse(finalXform)));

        const u32 vertexBase = static_cast<u32>(mesh.positions.size());
        const u32 firstIndex = static_cast<u32>(mesh.indices.size());

        mesh.positions.resize(vertexBase + vertexCount);
        mesh.normals.resize(vertexBase + vertexCount);
        mesh.tangents.resize(vertexBase + vertexCount);
        mesh.uvs.resize(vertexBase + vertexCount);
        if (hasSkin)
            mesh.skin.resize(vertexBase + vertexCount);

        // Build skin weights first so the per-vertex copy is trivial.
        std::vector<WeightSlots> weights;
        if (skinned)
        {
            weights.resize(vertexCount);
            for (int ci = 0; ci < skin->getClusterCount(); ++ci)
            {
                const ofbx::Cluster* cluster = skin->getCluster(ci);
                if (!cluster || !cluster->getLink())
                    continue;
                const auto mappedBone = boneMap.indexById.find(cluster->getLink()->id);
                if (mappedBone == boneMap.indexById.end())
                {
                    Log::error("FbxImporter: '%s' has a skin cluster linked to an unknown bone",
                               filename.c_str());
                    scene->destroy();
                    mesh.clear();
                    return false;
                }
                const int* clusterIndices = cluster->getIndices();
                const double* clusterWeights = cluster->getWeights();
                const int count = cluster->getIndicesCount();
                const s32 boneId = mappedBone->second;
                for (int i = 0; i < count; ++i)
                {
                    const int idx = clusterIndices[i];
                    if (idx >= 0 && idx < vertexCount)
                        weights[idx].add(boneId, static_cast<f32>(clusterWeights[i]));
                }
            }
        }

        for (int vi = 0; vi < vertexCount; ++vi)
        {
            Math::vec3 p = toVec3(srcPositions[vi]);
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            {
                Log::error("FbxImporter: '%s' contains a non-finite vertex at %d; clamping",
                           filename.c_str(), vi);
                p = Math::vec3(0.0f);
            }
            Math::vec3 outputPosition = Math::vec3(finalXform * Math::vec4(p, 1.0f));
            if (!std::isfinite(outputPosition.x) || !std::isfinite(outputPosition.y) ||
                !std::isfinite(outputPosition.z))
                outputPosition = p;
            mesh.positions[vertexBase + vi] = outputPosition;

            Math::vec3 n(0.0f, 1.0f, 0.0f);
            if (srcNormals)
            {
                n = toVec3(srcNormals[vi]);
                if (Math::dot(n, n) > 1e-8f)
                    n = Math::normalize(normalMatrix * n);
            }
            mesh.normals[vertexBase + vi] = n;

            Math::vec4 tangent(1.0f, 0.0f, 0.0f, 1.0f);
            if (srcTangents)
            {
                const ofbx::Vec3 t = srcTangents[vi];
                tangent = Math::vec4(static_cast<f32>(t.x), static_cast<f32>(t.y),
                                    static_cast<f32>(t.z), 1.0f);
            }
            mesh.tangents[vertexBase + vi] = tangent;

            Math::vec2 uv(0.0f, 0.0f);
            if (srcUvs)
            {
                uv = toVec2(srcUvs[vi]);
                uv.y = 1.0f - uv.y; // FBX bottom-left origin -> engine top-left
            }
            mesh.uvs[vertexBase + vi] = uv;

            if (hasSkin)
            {
                MeshSkinVertex skinVertex;
                if (skinned)
                {
                    const WeightSlots& slots = weights[vi];
                    const f32 sum =
                        slots.weights[0] + slots.weights[1] + slots.weights[2] + slots.weights[3];
                    if (sum <= 1e-8f)
                    {
                        skinVertex.joints[0] = 0;
                        skinVertex.weights = Math::vec4(1.0f, 0.0f, 0.0f, 0.0f);
                    }
                    else
                    {
                        const s32 b0 = slots.ids[0] >= 0 ? slots.ids[0] : 0;
                        const s32 b1 = slots.ids[1] >= 0 ? slots.ids[1] : b0;
                        const s32 b2 = slots.ids[2] >= 0 ? slots.ids[2] : b0;
                        const s32 b3 = slots.ids[3] >= 0 ? slots.ids[3] : b0;
                        skinVertex.joints[0] = static_cast<u8>(b0);
                        skinVertex.joints[1] = static_cast<u8>(b1);
                        skinVertex.joints[2] = static_cast<u8>(b2);
                        skinVertex.joints[3] = static_cast<u8>(b3);
                        skinVertex.weights =
                            Math::vec4(slots.weights[0] / sum, slots.weights[1] / sum,
                                      slots.weights[2] / sum, slots.weights[3] / sum);
                    }
                }
                else
                {
                    skinVertex.joints[0] = static_cast<u8>(attachmentBone >= 0 ? attachmentBone : 0);
                    skinVertex.weights = Math::vec4(1.0f, 0.0f, 0.0f, 0.0f);
                }
                mesh.skin[vertexBase + vi] = skinVertex;
            }
        }

        // Collect indices grouped by material so each SubMesh owns one contiguous
        // range. This avoids surprises if per-triangle material indices are not
        // already sorted in the file.
        std::unordered_map<u32, std::vector<u32>> indicesBySlot;
        for (int ii = 0; ii < indexCount; ii += 3)
        {
            if (ii + 2 >= indexCount)
                continue;
            const int i0 = srcIndices[ii] < 0 ? -srcIndices[ii] - 1 : srcIndices[ii];
            const int i1 = srcIndices[ii + 1] < 0 ? -srcIndices[ii + 1] - 1 : srcIndices[ii + 1];
            const int i2 = srcIndices[ii + 2] < 0 ? -srcIndices[ii + 2] - 1 : srcIndices[ii + 2];
            if (i0 < 0 || i0 >= vertexCount || i1 < 0 || i1 >= vertexCount || i2 < 0 ||
                i2 >= vertexCount)
                continue;
            const int triangleIndex = ii / 3;
            const int materialIndex = srcMaterials ? srcMaterials[triangleIndex] : 0;
            const ofbx::Material* material =
                (materialIndex >= 0 && materialIndex < fbxMesh->getMaterialCount())
                    ? fbxMesh->getMaterial(materialIndex)
                    : nullptr;
            const u32 slot = slotForMaterial(material);

            std::vector<u32>& out = indicesBySlot[slot];
            out.push_back(vertexBase + static_cast<u32>(i0));
            out.push_back(vertexBase + static_cast<u32>(i1));
            out.push_back(vertexBase + static_cast<u32>(i2));
        }

        for (auto& kv : indicesBySlot)
        {
            SubMesh submesh;
            submesh.indexOffset = static_cast<u32>(mesh.indices.size());
            submesh.indexCount = static_cast<u32>(kv.second.size());
            submesh.materialSlot = kv.first;
            mesh.indices.insert(mesh.indices.end(), kv.second.begin(), kv.second.end());
            mesh.submeshes.push_back(submesh);
        }

        (void)firstIndex;
    }

    scene->destroy();

    if (mesh.positions.empty() || mesh.indices.empty())
    {
        Log::error("FbxImporter: '%s' has no usable geometry", filename.c_str());
        mesh.clear();
        return false;
    }

    // ForwardPass decides the skinning pipeline from the MATERIAL's Skinned
    // flag, while DepthPass keys off the mesh. A skinned FBX must mark its
    // materials, or the colour pass renders the static bind pose under an
    // animated shadow.
    if (hasSkin)
        for (Material& material : mesh.materials)
            material.flags |= MaterialSkinned;

    mesh.bounds = AABB();
    for (const Math::vec3& position : mesh.positions)
        mesh.bounds.expand(position);
    for (SubMesh& submesh : mesh.submeshes)
    {
        submesh.bounds = AABB();
        const u32 endIndex = submesh.indexOffset + submesh.indexCount;
        for (u32 i = submesh.indexOffset; i < endIndex; ++i)
            submesh.bounds.expand(mesh.positions[mesh.indices[i]]);
    }

    Log::info("FbxImporter: '%s' - %u verts, %u indices, %zu submeshes, %zu materials",
              filename.c_str(), static_cast<u32>(mesh.positions.size()),
              static_cast<u32>(mesh.indices.size()), mesh.submeshes.size(), mesh.materials.size());

    return true;
}

// ------------------------------------------------------------ skeleton loader

bool loadFbxSkeleton(const std::string& filename, FileSystem& files, Skeleton& skeleton)
{
    ByteArray data = files.readBinary(filename);
    if (data.empty())
    {
        Log::error("FbxImporter: could not read '%s'", filename.c_str());
        return false;
    }

    ofbx::IScene* scene = parseFbx(data.data(), data.size());
    if (!scene)
    {
        Log::error("FbxImporter: '%s' failed to parse (%s)", filename.c_str(), ofbx::getError());
        return false;
    }

    FbxBoneMap boneMap;
    if (!buildBoneMap(*scene, boneMap) || boneMap.bones.empty())
    {
        Log::error("FbxImporter: '%s' has no skeleton", filename.c_str());
        scene->destroy();
        return false;
    }

    // Gather inverse bind matrices from the clusters that link back to each limb.
    std::unordered_map<u64, Math::mat4> inverseBindByBoneId;
    std::unordered_map<u64, Math::mat4> bindGlobalByBoneId;
    for (int mi = 0; mi < scene->getMeshCount(); ++mi)
    {
        const ofbx::Mesh* mesh = scene->getMesh(mi);
        const ofbx::Geometry* geom = mesh ? mesh->getGeometry() : nullptr;
        if (!geom)
            continue;
        const ofbx::Skin* skin = geom->getSkin();
        if (!skin)
            continue;
        for (int ci = 0; ci < skin->getClusterCount(); ++ci)
        {
            const ofbx::Cluster* cluster = skin->getCluster(ci);
            if (!cluster || !cluster->getLink())
                continue;
            inverseBindByBoneId[cluster->getLink()->id] =
                Math::inverse(toMat4(cluster->getTransformLinkMatrix()));
            bindGlobalByBoneId[cluster->getLink()->id] = toMat4(cluster->getTransformLinkMatrix());
        }
    }

    skeleton = Skeleton();
    for (const ofbx::Object* obj : boneMap.bones)
    {
        const std::string name = objectName(*obj);
        const ofbx::Object* parent = obj->getParent();
        const s32 parentIndex =
            (parent && boneMap.indexById.find(parent->id) != boneMap.indexById.end())
                ? boneMap.indexById[parent->id]
                : -1;

        Math::mat4 bindGlobal = toMat4(obj->getGlobalTransform());
        auto bindGlobalIt = bindGlobalByBoneId.find(obj->id);
        if (bindGlobalIt != bindGlobalByBoneId.end())
            bindGlobal = bindGlobalIt->second;
        Math::mat4 parentBindGlobal(1.0f);
        if (parent && boneMap.indexById.find(parent->id) != boneMap.indexById.end())
        {
            auto parentBind = bindGlobalByBoneId.find(parent->id);
            parentBindGlobal = parentBind != bindGlobalByBoneId.end()
                                   ? parentBind->second
                                   : toMat4(parent->getGlobalTransform());
        }
        const Math::mat4 bindLocal =
            parentIndex >= 0 ? Math::inverse(parentBindGlobal) * bindGlobal : bindGlobal;

        auto it = inverseBindByBoneId.find(obj->id);
        const Math::mat4 inverseBind = it != inverseBindByBoneId.end()
                                          ? it->second
                                          : Math::inverse(toMat4(obj->getGlobalTransform()));

        if (!skeleton.addBone(name, parentIndex, bindLocal, inverseBind))
        {
            scene->destroy();
            return false;
        }
    }

    scene->destroy();
    return skeleton.finalize();
}

// ----------------------------------------------------------- animation loader

bool loadFbxAnimation(const std::string& filename, FileSystem& files, const Skeleton& skeleton,
                      AnimationClip& clip, bool keepRootMotion)
{
    if (skeleton.empty())
        return false;

    ByteArray data = files.readBinary(filename);
    if (data.empty())
    {
        Log::error("FbxImporter: could not read '%s'", filename.c_str());
        return false;
    }

    ofbx::IScene* scene = parseFbx(data.data(), data.size());
    if (!scene)
    {
        Log::error("FbxImporter: '%s' failed to parse (%s)", filename.c_str(), ofbx::getError());
        return false;
    }

    if (scene->getAnimationStackCount() == 0)
    {
        Log::error("FbxImporter: '%s' has no animations", filename.c_str());
        scene->destroy();
        return false;
    }

    FbxBoneMap boneMap;
    if (!buildBoneMap(*scene, boneMap) || boneMap.bones.empty())
    {
        Log::error("FbxImporter: '%s' has no skeleton to animate", filename.c_str());
        scene->destroy();
        return false;
    }

    // Match the animation's bones to the loaded skeleton by name, not by
    // order. The animation may live in a separate FBX whose hierarchy or bone
    // ordering differs from the mesh's (Blender re-exports add Reference/Pivot/
    // Root nodes and reshuffle children). BoneTrack::bone stores a skeleton
    // index, and unmapped animation bones are simply skipped.
    const usize animBoneCount = boneMap.bones.size();
    std::vector<s32> animToSkeleton(animBoneCount, -1);
    u32 matched = 0;

    // Canonical name -> skeleton index, so either the Mixamo or Blender
    // spelling resolves to the same bone.
    std::unordered_map<std::string, s32> skeletonByCanonicalName;
    for (u32 b = 0; b < skeleton.boneCount(); ++b)
        skeletonByCanonicalName[canonicalBoneName(skeleton.bone(b).name)] = static_cast<s32>(b);

    for (usize a = 0; a < animBoneCount; ++a)
    {
        const auto it =
            skeletonByCanonicalName.find(canonicalBoneName(objectName(*boneMap.bones[a])));
        if (it != skeletonByCanonicalName.end())
        {
            animToSkeleton[a] = it->second;
            ++matched;
        }
    }
    if (matched == 0)
    {
        Log::error("FbxImporter: '%s' - no animation bone matched the loaded skeleton by name",
                   filename.c_str());
        scene->destroy();
        return false;
    }
    // Do not silently play an animation on a different character. A partial
    // name match can look plausible while leaving unmatched limbs in the bind
    // pose, which produces detached/deformed body parts. Animation exports
    // may omit a few helper bones, but the animated hierarchy itself must be
    // mostly shared with the loaded skeleton.
    const u32 minimumMatches = static_cast<u32>(std::ceil(animBoneCount * 0.80));
    if (matched < minimumMatches)
    {
        Log::error("FbxImporter: '%s' is incompatible with the loaded skeleton "
                   "(%u/%zu bones matched; need at least %u)",
                   filename.c_str(), matched, animBoneCount, minimumMatches);
        scene->destroy();
        return false;
    }
    u32 hierarchyMatches = 0;
    u32 hierarchyChecks = 0;
    for (usize a = 0; a < animBoneCount; ++a)
    {
        if (animToSkeleton[a] < 0)
            continue;
        const ofbx::Object* parent = boneMap.bones[a]->getParent();
        if (!parent)
            continue;
        const auto parentIt = boneMap.indexById.find(parent->id);
        if (parentIt == boneMap.indexById.end() || animToSkeleton[parentIt->second] < 0)
            continue;
        ++hierarchyChecks;
        const s32 skeletonParent = skeleton.bone(static_cast<u32>(animToSkeleton[a])).parent;
        if (skeletonParent == animToSkeleton[parentIt->second])
            ++hierarchyMatches;
    }
    if (hierarchyChecks > 0 && hierarchyMatches * 10 < hierarchyChecks * 8)
    {
        Log::error("FbxImporter: '%s' has an incompatible bone hierarchy "
                   "(%u/%u parent links matched)",
                   filename.c_str(), hierarchyMatches, hierarchyChecks);
        scene->destroy();
        return false;
    }
    Log::info("FbxImporter: '%s' matched %u/%zu animation bones by name", filename.c_str(), matched,
              animBoneCount);

    const ofbx::AnimationStack* stack = nullptr;
    const ofbx::AnimationLayer* layer = nullptr;
    for (int si = 0; si < scene->getAnimationStackCount() && !layer; ++si)
    {
        const ofbx::AnimationStack* candidate = scene->getAnimationStack(si);
        for (int li = 0; candidate && li < 16; ++li)
        {
            const ofbx::AnimationLayer* candidateLayer = candidate->getLayer(li);
            bool hasCurves = false;
            if (candidateLayer)
            {
                const char* properties[] = {"Lcl Translation", "Lcl Rotation", "Lcl Scaling"};
                for (const ofbx::Object* object : boneMap.bones)
                {
                    for (const char* property : properties)
                    {
                        if (candidateLayer->getCurveNode(*object, property))
                        {
                            hasCurves = true;
                            break;
                        }
                    }
                    if (hasCurves)
                        break;
                }
            }
            if (hasCurves)
            {
                stack = candidate;
                layer = candidateLayer;
                break;
            }
        }
    }
    if (!layer)
    {
        Log::error("FbxImporter: '%s' has no animation layer", filename.c_str());
        scene->destroy();
        return false;
    }

    clip = AnimationClip();
    // Blender exports every action under the generic "Take 001"; the filename
    // stem (e.g. "Idle", "Run_0") is unique and meaningful, so prefer it over
    // the generic stack name. Mixamo's "mixamo.com" is kept as-is.
    const std::string stackName = stack->name[0] != '\0' ? std::string(stack->name) : std::string();
    clip.setName(!stackName.empty() && stackName != "Take 001" ? stackName : stem(filename));

    // Aggregate channels per target bone. A bone can have up to three curve
    // nodes (translation, rotation, scale), each with up to three XYZ curves.
    struct Channels
    {
        const ofbx::Object* object = nullptr;
        std::vector<f32> posT[3];
        std::vector<f32> posV[3];
        std::vector<f32> rotT[3];
        std::vector<f32> rotV[3];
        std::vector<f32> sclT[3];
        std::vector<f32> sclV[3];
    };
    std::unordered_map<s32, Channels> perBone;

    const char* properties[3] = {"Lcl Translation", "Lcl Rotation", "Lcl Scaling"};
    for (const ofbx::Object* bone : boneMap.bones)
    {
        auto it = boneMap.indexById.find(bone->id);
        if (it == boneMap.indexById.end())
            continue;
        const s32 boneIndex = it->second;
        Channels& channels = perBone[boneIndex];
        channels.object = bone;
        for (int type = 0; type < 3; ++type)
        {
            const ofbx::AnimationCurveNode* node = layer->getCurveNode(*bone, properties[type]);
            if (!node)
                continue;
            for (int axis = 0; axis < 3; ++axis)
            {
                const ofbx::AnimationCurve* curve = node->getCurve(axis);
                if (!curve || curve->getKeyCount() <= 0)
                    continue;
                std::vector<f32>* times = type == 0   ? &channels.posT[axis]
                                          : type == 1 ? &channels.rotT[axis]
                                                      : &channels.sclT[axis];
                std::vector<f32>* values = type == 0   ? &channels.posV[axis]
                                           : type == 1 ? &channels.rotV[axis]
                                                       : &channels.sclV[axis];
                const ofbx::i64* keyTimes = curve->getKeyTime();
                const float* keyValues = curve->getKeyValue();
                const int keyCount = curve->getKeyCount();
                times->reserve(static_cast<usize>(keyCount));
                values->reserve(static_cast<usize>(keyCount));
                for (int ki = 0; ki < keyCount; ++ki)
                {
                    times->push_back(static_cast<f32>(ofbx::fbxTimeToSeconds(keyTimes[ki])));
                    values->push_back(static_cast<f32>(keyValues[ki]));
                }
            }
        }
    }

    std::vector<f32> times;
    for (const auto& kv : perBone)
    {
        const Channels& channels = kv.second;
        for (int axis = 0; axis < 3; ++axis)
        {
            appendTimes(times, channels.posT[axis]);
            appendTimes(times, channels.rotT[axis]);
            appendTimes(times, channels.sclT[axis]);
        }
    }
    sortUniqueTimes(times);

    // Parent indices inside the animation file's own hierarchy. Blender-style
    // exports can be re-rooted or re-ordered relative to the mesh skeleton, so
    // the animation side walks its own chain instead of borrowing the loaded
    // skeleton's parent list.
    std::vector<s32> parentAnim(animBoneCount, -1);
    for (usize a = 0; a < animBoneCount; ++a)
    {
        const ofbx::Object* parentObject = boneMap.bones[a]->getParent();
        const auto parentIt =
            parentObject ? boneMap.indexById.find(parentObject->id) : boneMap.indexById.end();
        if (parentObject && parentIt != boneMap.indexById.end())
            parentAnim[a] = parentIt->second;
    }

    // The true bind pose lives in the skin clusters' TransformLink, not in the
    // nodes' Lcl Translation/Rotation (which Blender writes as the first
    // animation frame). Reconstruct the authored bind the same way
    // loadFbxSkeleton() does, so authoredBindGlobal == skeletonBindGlobal for
    // a consistent file and the delta maps bind -> animated correctly.
    std::unordered_map<u64, Math::mat4> clusterBindGlobal;
    for (int mi = 0; mi < scene->getMeshCount(); ++mi)
    {
        const ofbx::Mesh* mesh = scene->getMesh(mi);
        const ofbx::Geometry* geom = mesh ? mesh->getGeometry() : nullptr;
        const ofbx::Skin* skin = geom ? geom->getSkin() : nullptr;
        if (!skin)
            continue;
        for (int ci = 0; ci < skin->getClusterCount(); ++ci)
        {
            const ofbx::Cluster* cluster = skin->getCluster(ci);
            if (cluster && cluster->getLink())
                clusterBindGlobal[cluster->getLink()->id] =
                    toMat4(cluster->getTransformLinkMatrix());
        }
    }

    // Local (parent-relative) bind transform of every animation bone.
    std::vector<Math::mat4> authoredBindLocal(animBoneCount, Math::mat4(1.0f));
    for (usize a = 0; a < animBoneCount; ++a)
    {
        const ofbx::Object* object = boneMap.bones[a];
        const auto bindIt = clusterBindGlobal.find(object->id);
        if (bindIt != clusterBindGlobal.end() && parentAnim[a] >= 0)
        {
            const auto parentBindIt =
                clusterBindGlobal.find(boneMap.bones[static_cast<usize>(parentAnim[a])]->id);
            authoredBindLocal[a] = parentBindIt != clusterBindGlobal.end()
                                       ? Math::inverse(parentBindIt->second) * bindIt->second
                                       : bindIt->second;
        }
        else
            authoredBindLocal[a] = toMat4(object->getLocalTransform());
    }

    const usize skeletonBoneCount = skeleton.boneCount();
    std::vector<BoneTrack> tracks(skeletonBoneCount);
    for (usize s = 0; s < skeletonBoneCount; ++s)
        tracks[s].bone = static_cast<s32>(s);
    // Sample in LOCAL space: each animation bone's delta (bind local ->
    // animated local) is applied to the mapped skeleton bone's bind local.
    // Working per bone instead of through world-space globals stops a
    // different root position/rotation between two files from leaking huge
    // translations into bones far from the origin.
    for (f32 time : times)
    {
        for (usize a = 0; a < animBoneCount; ++a)
        {
            const s32 skeletonIndex = animToSkeleton[a];
            if (skeletonIndex < 0)
                continue;
            const usize s = static_cast<usize>(skeletonIndex);

            const ofbx::Object* object = boneMap.bones[a];
            const auto channelIt = perBone.find(static_cast<s32>(a));
            const Channels* channels = channelIt != perBone.end() ? &channelIt->second : nullptr;
            const ofbx::Vec3 defaultT = object->getLocalTranslation();
            const ofbx::Vec3 defaultR = object->getLocalRotation();
            const ofbx::Vec3 defaultS = object->getLocalScaling();
            const ofbx::Vec3 translation = {
                channels ? sampleScalarChannel(channels->posT[0], channels->posV[0], time,
                                               static_cast<f32>(defaultT.x))
                         : defaultT.x,
                channels ? sampleScalarChannel(channels->posT[1], channels->posV[1], time,
                                               static_cast<f32>(defaultT.y))
                         : defaultT.y,
                channels ? sampleScalarChannel(channels->posT[2], channels->posV[2], time,
                                               static_cast<f32>(defaultT.z))
                         : defaultT.z};
            const ofbx::Vec3 rotation = {
                channels ? sampleScalarChannel(channels->rotT[0], channels->rotV[0], time,
                                               static_cast<f32>(defaultR.x))
                         : defaultR.x,
                channels ? sampleScalarChannel(channels->rotT[1], channels->rotV[1], time,
                                               static_cast<f32>(defaultR.y))
                         : defaultR.y,
                channels ? sampleScalarChannel(channels->rotT[2], channels->rotV[2], time,
                                               static_cast<f32>(defaultR.z))
                         : defaultR.z};
            const ofbx::Vec3 scale = {
                channels ? sampleScalarChannel(channels->sclT[0], channels->sclV[0], time,
                                               static_cast<f32>(defaultS.x))
                         : defaultS.x,
                channels ? sampleScalarChannel(channels->sclT[1], channels->sclV[1], time,
                                               static_cast<f32>(defaultS.y))
                         : defaultS.y,
                channels ? sampleScalarChannel(channels->sclT[2], channels->sclV[2], time,
                                               static_cast<f32>(defaultS.z))
                         : defaultS.z};

            const Math::mat4 authoredLocal = toMat4(object->evalLocal(translation, rotation, scale));
            const Math::mat4 deltaLocal = Math::inverse(authoredBindLocal[a]) * authoredLocal;
            const Math::mat4 correctedLocal =
                skeleton.bone(static_cast<u32>(s)).bindLocal * deltaLocal;

            Math::vec3 outScale, outTranslation, skew;
            Math::vec4 perspective;
            Math::quat outRotation;
            if (!Math::decompose(correctedLocal, outScale, outRotation, outTranslation, skew,
                                perspective))
                continue;
            BoneTrack& track = tracks[s];
            if (!track.rotations.empty() && Math::dot(track.rotations.back(), outRotation) < 0.0f)
                outRotation = -outRotation;
            track.times.push_back(time);
            track.positions.push_back(outTranslation);
            track.rotations.push_back(Math::normalize(outRotation));
            track.scales.push_back(outScale);
        }
    }
    // Optional in-place conversion. Some exporters bake a large world-space offset
    // into the hips (or a similar functional root), which teleports the character
    // away from the origin. For in-place playback we pin the horizontal position
    // to the bind pose and preserve only the vertical bobbing, referenced against
    // the LOWEST point of the root track rather than frame 0 - for a locomotion
    // cycle that lowest point is the weight-bearing/ground-contact frame, the one
    // actually at the bind pose's standing height; frame 0 lands there only by
    // chance (mid-stride, a foot lifted, is just as likely), and pinning to it
    // would carry that stride phase's own height into every frame, floating the
    // whole loop by however far off the ground frame 0 happened to be.
    // Rotations are left untouched.
    if (!keepRootMotion)
    {
        const s32 rootAnim = findFunctionalRoot(boneMap, parentAnim, animToSkeleton, tracks);
        if (rootAnim >= 0)
        {
            const usize skelRoot = static_cast<usize>(animToSkeleton[static_cast<usize>(rootAnim)]);
            BoneTrack& rootTrack = tracks[skelRoot];
            if (!rootTrack.positions.empty())
            {
                Math::vec3 lowestPosition = rootTrack.positions.front();
                for (const Math::vec3& position : rootTrack.positions)
                    if (position.y < lowestPosition.y)
                        lowestPosition = position;
                const Math::vec3 bindPosition =
                    Math::vec3(skeleton.bone(static_cast<u32>(skelRoot)).bindLocal[3]);
                const f32 verticalOffset = lowestPosition.y - bindPosition.y;
                for (Math::vec3& position : rootTrack.positions)
                {
                    position.x = bindPosition.x;
                    position.z = bindPosition.z;
                    position.y -= verticalOffset;
                }
                Log::info(
                    "FbxImporter: '%s' converted to in-place (root '%s' lowest-frame offset %.3f %.3f %.3f)",
                    filename.c_str(),
                    objectName(*boneMap.bones[static_cast<usize>(rootAnim)]).c_str(),
                    lowestPosition.x - bindPosition.x, verticalOffset,
                    lowestPosition.z - bindPosition.z);
            }
        }
    }

    for (BoneTrack& track : tracks)
        if (!track.times.empty())
            clip.tracks().push_back(std::move(track));

    const f32 firstKeyTime = times.empty() ? 0.0f : times.front();
    const f32 lastKeyTime = times.empty() ? 0.0f : times.back();
    if (!times.empty())
        for (BoneTrack& track : clip.tracks())
            for (f32& time : track.times)
                time -= firstKeyTime;

    scene->destroy();

    if (clip.tracks().empty())
    {
        Log::error("FbxImporter: '%s' - no channel targeted a bone in the loaded skeleton",
                   filename.c_str());
        return false;
    }

    const f32 clipDuration = lastKeyTime - firstKeyTime;
    clip.setDuration(clipDuration > 0.0f ? clipDuration : 1.0f);
    Log::info("FbxImporter: anim '%s' dur=%.3f tracks=%zu", clip.name().c_str(), clip.duration(),
              clip.tracks().size());
    return true;
}

} // namespace Radion
