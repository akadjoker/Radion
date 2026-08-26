// glTF 2.0 / GLB importer, ported from docs/GLTFLoader.cpp and reshaped to
// the engine's importer pattern (see B3DImporter/MS3DImporter): geometry goes
// through GltfImporter::import() into a MeshData, the skeleton and the
// animation clips through loadGltfSkeleton()/loadGltfAnimation().
//
// The reference loaded a whole SkinnedMesh in one go; here the caller drives
// the same three steps the native formats do. The one structural difference
// that mattered in the reference applies unchanged: glTF gives each TRS
// channel its own independent time array (and any channel may be absent for a
// given bone), while Radion's BoneTrack drives position/rotation/scale in
// lockstep off ONE shared `times` array - so channels are resampled onto a
// unified per-bone time array (the union of all its channels' keyframe times)
// before building the track.
//
// File I/O never goes through raw stdio: cgltf's file callbacks are routed
// through FileSystem, matching the other importers' contract ("a format
// importer only decodes memory; any secondary file it needs must be read
// through the supplied FileSystem").

#include "PCH.h"

#include "GltfImporter.h"

#include "ByteArray.h"
#include "FileSystem.h"
#include "Log.h"
#include "Material.h"
#include "Math.h"
#include "Pixmap.h"
#include "Skeleton.h"

#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <unordered_map>
#include <vector>

namespace Radion
{

namespace
{

// cgltf reads external buffers/images through file callbacks; route them
// through Radion's FileSystem (which searches the registered asset folders)
// instead of fopen, matching every other importer's contract.
cgltf_result gltfFileRead(const cgltf_memory_options*, const cgltf_file_options* fileOptions,
                          const char* path, cgltf_size* size, void** data)
{
    FileSystem* files = static_cast<FileSystem*>(fileOptions->user_data);
    ByteArray bytes = files->readBinary(path);
    if (bytes.empty())
        return cgltf_result_file_not_found;

    void* memory = CGLTF_MALLOC(bytes.size());
    if (!memory)
        return cgltf_result_out_of_memory;
    std::memcpy(memory, bytes.data(), bytes.size());
    *size = bytes.size();
    *data = memory;
    return cgltf_result_success;
}

void gltfFileRelease(const cgltf_memory_options*, const cgltf_file_options*, void* data)
{
    CGLTF_FREE(data);
}

// Parses an in-memory glTF/GLB file and loads its buffers. External buffers
// (a .gltf's .bin, an image) go through the file callbacks above, which
// resolve against FileSystem.
cgltf_data* parseGltf(FileSystem& files, const void* data, cgltf_size size,
                      const std::string& filename)
{
    cgltf_options options = {};
    options.file.read = gltfFileRead;
    options.file.release = gltfFileRelease;
    options.file.user_data = &files;

    cgltf_data* gltf = nullptr;
    if (cgltf_parse(&options, data, size, &gltf) != cgltf_result_success)
        return nullptr;
    if (cgltf_load_buffers(&options, gltf, filename.c_str()) != cgltf_result_success)
    {
        cgltf_free(gltf);
        return nullptr;
    }
    return gltf;
}

u8 clampBone(cgltf_uint value, cgltf_size jointCount)
{
    const cgltf_uint maxValue = jointCount > 0 ? static_cast<cgltf_uint>(jointCount - 1) : 0u;
    if (value > maxValue)
        value = maxValue;
    return static_cast<u8>(value);
}

// T * R * S - the engine's own convention (see BoneAttachment: Translate * Rotate).
glm::mat4 nodeLocalMatrix(const cgltf_node& node)
{
    if (node.has_matrix)
        return glm::make_mat4(node.matrix); // column-major flat copy, no transpose needed

    const glm::vec3 t = node.has_translation ? glm::vec3(node.translation[0], node.translation[1],
                                                         node.translation[2])
                                             : glm::vec3(0.0f);
    // glTF stores a rotation as [x, y, z, w]; glm::quat's constructor takes
    // (w, x, y, z). Passing the array straight through turns every rotation
    // into a different one about a different axis.
    const glm::quat r = node.has_rotation ? glm::quat(node.rotation[3], node.rotation[0],
                                                      node.rotation[1], node.rotation[2])
                                          : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::vec3 s =
        node.has_scale ? glm::vec3(node.scale[0], node.scale[1], node.scale[2]) : glm::vec3(1.0f);
    return glm::translate(glm::mat4(1.0f), t) * glm::mat4_cast(r) * glm::scale(glm::mat4(1.0f), s);
}

glm::mat4 nodeGlobalMatrix(const cgltf_node& node)
{
    std::vector<glm::mat4> chain;
    const cgltf_node* current = &node;
    while (current)
    {
        chain.push_back(nodeLocalMatrix(*current));
        current = current->parent;
    }
    glm::mat4 result(1.0f);
    for (auto it = chain.rbegin(); it != chain.rend(); ++it)
        result = result * (*it);
    return result;
}

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

std::string stem(const std::string& path)
{
    const usize slash = path.find_last_of("/\\");
    const usize base = slash == std::string::npos ? 0 : slash + 1;
    const usize dot = path.find_last_of('.');
    if (dot == std::string::npos || dot < base)
        return path.substr(base);
    return path.substr(base, dot - base);
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

// A glTF image is either a bare `uri` (an external file, the common .gltf
// case) or embedded straight in the binary buffer (`buffer_view`, how a
// single-file .glb like FlightHelmet.glb usually ships every texture) - only
// the first ever had a path this importer's file arrays could point at, so
// an embedded one silently imported as no texture at all (a plain white
// material). This decodes it once (Pixmap already reads PNG/JPEG straight
// from memory) and writes it out as a real file next to the source glTF, so
// the rest of the pipeline - a plain relative path, same as an external
// texture - never has to know the difference. `realDirectory` is the
// source glTF's own directory already resolved to a writable disk path
// (FileSystem::writeBinary()/Pixmap::save() need one, unlike reading, which
// resolves search paths on its own).
// A glTF may name an image the export never shipped: MSFT_texture_dds lists
// the .png as the base source and the .dds that actually exists only inside
// the extension, and cgltf has no field for that extension. Rather than parse
// it, the file next to it with the same stem is taken - the same rule the
// texture loader already applies at load time, applied here too so the path
// written into the .material names a file that is really there.
std::string existingImageFile(const std::string& relative, const std::string& realDirectory)
{
    FileSystem& files = FileSystem::getSingleton();
    if (relative.empty() || realDirectory.empty() || files.exists(realDirectory + relative))
        return relative;
    const usize dot = relative.find_last_of('.');
    const usize slash = relative.find_last_of("/\\");
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
        return relative;
    const std::string base = relative.substr(0, dot);
    static const char* const extensions[] = {".dds", ".png", ".jpg", ".jpeg", ".tga", ".bmp"};
    for (const char* extension : extensions)
    {
        const std::string candidate = base + extension;
        if (candidate != relative && files.exists(realDirectory + candidate))
            return candidate;
    }
    return relative;
}

std::string resolveImageFile(const cgltf_texture* texture, const std::string& realDirectory,
                             const std::string& textureFolder)
{
    if (!texture || !texture->image)
        return std::string();
    const cgltf_image& image = *texture->image;
    if (image.uri && image.uri[0])
        return existingImageFile(image.uri, realDirectory);
    if (realDirectory.empty() || textureFolder.empty() || !image.buffer_view ||
        !image.buffer_view->buffer || !image.buffer_view->buffer->data)
        return std::string();

    const bool isPng = image.mime_type && std::strstr(image.mime_type, "png") != nullptr;
    const std::string relativeName = textureFolder + "/embedded_" +
                                     std::to_string(image.buffer_view->offset) +
                                     (isPng ? ".png" : ".jpg");
    const std::string realPath = realDirectory + relativeName;
    FileSystem& files = FileSystem::getSingleton();
    if (!files.exists(realPath))
    {
        files.createDirectory(realDirectory + textureFolder);
        const u8* bytes =
            static_cast<const u8*>(image.buffer_view->buffer->data) + image.buffer_view->offset;
        Pixmap pixmap;
        if (!pixmap.load_from_memory(bytes, static_cast<u32>(image.buffer_view->size)) ||
            !pixmap.save(realPath.c_str()))
            return std::string();
    }
    return relativeName;
}

// Maps one glTF material onto one Radion Material, one slot per distinct
// cgltf_material* (deduplicated through `sources`). PBR factors land in
// params; albedo/normal texture URIs land in the parallel file arrays.
u32 materialSlotFor(const cgltf_primitive& prim, const std::string& directory,
                    const std::string& realDirectory, const std::string& textureFolder,
                    std::vector<const cgltf_material*>& sources, MeshData& mesh)
{
    const cgltf_material* material = prim.material;
    if (!material)
        return 0;

    for (usize i = 0; i < sources.size(); ++i)
        if (sources[i] == material)
            return static_cast<u32>(i);

    const u32 slot = static_cast<u32>(mesh.materials.size());
    sources.push_back(material);

    Material out;
    out.name = material->name ? material->name : "";
    out.nameHash = hashName(out.name);

    if (material->has_pbr_metallic_roughness)
    {
        const cgltf_pbr_metallic_roughness& pbr = material->pbr_metallic_roughness;
        out.params.baseColor = glm::vec4(pbr.base_color_factor[0], pbr.base_color_factor[1],
                                         pbr.base_color_factor[2], pbr.base_color_factor[3]);
        out.params.surface.x = pbr.roughness_factor; // roughness
        out.params.surface.y = pbr.metallic_factor;  // metalness
    }
    else if (material->has_pbr_specular_glossiness)
    {
        // KHR_materials_pbrSpecularGlossiness. Without this branch a material
        // that carries no pbrMetallicRoughness at all - every one of the
        // Bistro's 254 - imports with no base colour and no albedo texture.
        const cgltf_pbr_specular_glossiness& pbr = material->pbr_specular_glossiness;
        out.params.baseColor = glm::vec4(pbr.diffuse_factor[0], pbr.diffuse_factor[1],
                                         pbr.diffuse_factor[2], pbr.diffuse_factor[3]);
        out.params.surface.x = 1.0f - pbr.glossiness_factor;
        // No texture to read them from leaves only the constant factors, and
        // a specular factor is what says metal here - lit.frag's own
        // dielectric baseline is 0.04.
        const f32 specular = std::max(std::max(pbr.specular_factor[0], pbr.specular_factor[1]),
                                      pbr.specular_factor[2]);
        out.params.surface.y = glm::clamp((specular - 0.04f) / 0.96f, 0.0f, 1.0f);
        out.params.custom0 = glm::vec4(pbr.specular_factor[0], pbr.specular_factor[1],
                                       pbr.specular_factor[2], pbr.glossiness_factor);
    }
    out.params.emissive = glm::vec4(material->emissive_factor[0], material->emissive_factor[1],
                                    material->emissive_factor[2], 1.0f);
    // normal_texture.scale is glTF's normalScale - lit.frag's uNormalStrength
    // (surface.w), 1.0 by default same as the spec's own default. Only
    // meaningful once a normal map is actually bound below.
    out.params.surface.w = material->normal_texture.texture ? material->normal_texture.scale : 1.0f;

    if (material->alpha_mode == cgltf_alpha_mode_mask)
    {
        out.flags |= MaterialAlphaTest;
        out.params.surface.z = material->alpha_cutoff; // alphaCut
    }
    else if (material->alpha_mode == cgltf_alpha_mode_blend)
    {
        out.blend = BlendMode::Alpha;
    }
    if (material->double_sided)
        out.flags |= MaterialTwoSided;

    const cgltf_texture* albedo = nullptr;
    if (material->has_pbr_metallic_roughness)
        albedo = material->pbr_metallic_roughness.base_color_texture.texture;
    else if (material->has_pbr_specular_glossiness)
        albedo = material->pbr_specular_glossiness.diffuse_texture.texture;
    const std::string albedoFile = resolveImageFile(albedo, realDirectory, textureFolder);

    const cgltf_texture* normal = material->normal_texture.texture;
    const std::string normalFile = resolveImageFile(normal, realDirectory, textureFolder);

    // SlotSurface otherwise means the legacy specular map lit.frag reads by
    // default - the flag is what tells it which of the three packings this
    // texture really is. See Material.h's own comment on them.
    const cgltf_texture* surface = nullptr;
    u32 surfaceFlag = 0;
    if (material->has_pbr_metallic_roughness)
    {
        surface = material->pbr_metallic_roughness.metallic_roughness_texture.texture;
        surfaceFlag = MaterialMetallicRoughnessMap;
    }
    else if (material->has_pbr_specular_glossiness)
    {
        surface = material->pbr_specular_glossiness.specular_glossiness_texture.texture;
        surfaceFlag = MaterialSpecularGlossinessMap;
    }
    const std::string surfaceFile = resolveImageFile(surface, realDirectory, textureFolder);
    if (!surfaceFile.empty())
        out.flags |= surfaceFlag;

    const cgltf_texture* emissive = material->emissive_texture.texture;
    const std::string emissiveFile = resolveImageFile(emissive, realDirectory, textureFolder);

    mesh.materials.push_back(out);
    mesh.materialTextureFiles.push_back(joinPath(directory, albedoFile));
    mesh.materialNormalFiles.push_back(joinPath(directory, normalFile));
    mesh.materialSurfaceFiles.push_back(joinPath(directory, surfaceFile));
    mesh.materialEmissiveFiles.push_back(joinPath(directory, emissiveFile));
    return slot;
}

// Per-channel glTF keyframes for one bone, before resampling.
struct RawBoneChannels
{
    std::vector<f32> posT;
    std::vector<glm::vec3> posV;
    std::vector<f32> rotT;
    std::vector<glm::quat> rotV;
    std::vector<f32> sclT;
    std::vector<glm::vec3> sclV;
};

glm::vec3 sampleVec3Channel(const std::vector<f32>& t, const std::vector<glm::vec3>& v, f32 time,
                            const glm::vec3& fallback)
{
    if (t.empty())
        return fallback;
    if (t.size() == 1 || time <= t.front())
        return v.front();
    if (time >= t.back())
        return v.back();
    size_t k1 = static_cast<size_t>(std::upper_bound(t.begin(), t.end(), time) - t.begin());
    size_t k0 = k1 - 1;
    const f32 span = t[k1] - t[k0];
    const f32 f = span > 1e-6f ? (time - t[k0]) / span : 0.0f;
    return v[k0] + (v[k1] - v[k0]) * f;
}

glm::quat sampleQuatChannel(const std::vector<f32>& t, const std::vector<glm::quat>& v, f32 time,
                            const glm::quat& fallback)
{
    if (t.empty())
        return fallback;
    if (t.size() == 1 || time <= t.front())
        return v.front();
    if (time >= t.back())
        return v.back();
    size_t k1 = static_cast<size_t>(std::upper_bound(t.begin(), t.end(), time) - t.begin());
    size_t k0 = k1 - 1;
    const f32 span = t[k1] - t[k0];
    const f32 f = span > 1e-6f ? (time - t[k0]) / span : 0.0f;
    return glm::normalize(v[k0] + (v[k1] - v[k0]) * f);
}

// glTF gives each TRS channel its own independent time array, while Radion's
// BoneTrack drives position/rotation/scale in lockstep off ONE shared `times`
// array. This resamples every channel onto the union of the bone's keyframe
// times before building the track.
void buildClipTracks(const cgltf_animation& src,
                     const std::unordered_map<const cgltf_node*, int>& boneOf,
                     const std::vector<LocalPose>& bindPose, AnimationClip& outClip)
{
    std::unordered_map<int, RawBoneChannels> perBone;

    for (cgltf_size ci = 0; ci < src.channels_count; ++ci)
    {
        const cgltf_animation_channel& channel = src.channels[ci];
        if (!channel.sampler || !channel.target_node)
            continue;
        auto it = boneOf.find(channel.target_node);
        if (it == boneOf.end())
            continue; // channel targets a node outside this skeleton
        const int boneIndex = it->second;

        cgltf_accessor* input = channel.sampler->input;
        cgltf_accessor* output = channel.sampler->output;
        if (!input || !output)
            continue;
        const cgltf_size keyCount = std::min(input->count, output->count);
        RawBoneChannels& raw = perBone[boneIndex];

        for (cgltf_size ki = 0; ki < keyCount; ++ki)
        {
            f32 t = 0.0f;
            cgltf_accessor_read_float(input, ki, &t, 1);
            if (channel.target_path == cgltf_animation_path_type_translation)
            {
                f32 v[3];
                cgltf_accessor_read_float(output, ki, v, 3);
                raw.posT.push_back(t);
                raw.posV.emplace_back(v[0], v[1], v[2]);
            }
            else if (channel.target_path == cgltf_animation_path_type_rotation)
            {
                f32 v[4];
                cgltf_accessor_read_float(output, ki, v, 4);
                raw.rotT.push_back(t);
                raw.rotV.push_back(glm::normalize(glm::quat(v[3], v[0], v[1], v[2])));
            }
            else if (channel.target_path == cgltf_animation_path_type_scale)
            {
                f32 v[3];
                cgltf_accessor_read_float(output, ki, v, 3);
                raw.sclT.push_back(t);
                raw.sclV.emplace_back(v[0], v[1], v[2]);
            }
            // weights (morph targets): no Radion track to fill
        }
    }

    f32 clipDuration = 0.0f;
    for (auto& kv : perBone)
    {
        const int boneIndex = kv.first;
        RawBoneChannels& raw = kv.second;

        // unified time array = sorted, de-duplicated union of whichever
        // channels this bone actually has
        std::vector<f32> unified;
        unified.reserve(raw.posT.size() + raw.rotT.size() + raw.sclT.size());
        unified.insert(unified.end(), raw.posT.begin(), raw.posT.end());
        unified.insert(unified.end(), raw.rotT.begin(), raw.rotT.end());
        unified.insert(unified.end(), raw.sclT.begin(), raw.sclT.end());
        if (unified.empty())
            continue;
        std::sort(unified.begin(), unified.end());
        unified.erase(std::unique(unified.begin(), unified.end(),
                                  [](f32 a, f32 b)
                                  {
                                      return std::fabs(a - b) < 1e-6f;
                                  }),
                      unified.end());

        const LocalPose& bind = bindPose[static_cast<size_t>(boneIndex)];

        BoneTrack track;
        track.bone = boneIndex;
        track.times = unified;
        track.positions.reserve(unified.size());
        track.rotations.reserve(unified.size());
        track.scales.reserve(unified.size());
        for (f32 time : unified)
        {
            track.positions.push_back(sampleVec3Channel(raw.posT, raw.posV, time, bind.position));
            track.rotations.push_back(sampleQuatChannel(raw.rotT, raw.rotV, time, bind.rotation));
            track.scales.push_back(sampleVec3Channel(raw.sclT, raw.sclV, time, bind.scale));
        }
        clipDuration = std::max(clipDuration, unified.back());
        outClip.tracks().push_back(std::move(track));
    }
    outClip.setDuration(clipDuration > 0.0f ? clipDuration : 1.0f);
}

} // namespace

// ------------------------------------------------------------------ importer

bool GltfImporter::supports(const char* extension) const
{
    return extension && (std::strcmp(extension, "gltf") == 0 || std::strcmp(extension, "glb") == 0);
}

bool GltfImporter::import(const std::string& filename, ByteArray& data, FileSystem& files,
                          MeshData& mesh)
{
    cgltf_data* gltf = parseGltf(files, data.data(), data.size(), filename);
    if (!gltf)
    {
        Log::error("GltfImporter: '%s' failed to parse", filename.c_str());
        return false;
    }

    mesh.clear();

    // Pre-pass: does the file skin anything at all? If yes, skin is filled
    // for every vertex (defaulting to joint 0) so the streams never drift.
    bool hasSkin = false;
    for (cgltf_size ni = 0; ni < gltf->nodes_count && !hasSkin; ++ni)
    {
        const cgltf_node& node = gltf->nodes[ni];
        if (!node.mesh)
            continue;
        for (cgltf_size pi = 0; pi < node.mesh->primitives_count; ++pi)
        {
            const cgltf_primitive& prim = node.mesh->primitives[pi];
            bool seenJoints = false;
            bool seenWeights = false;
            for (cgltf_size ai = 0; ai < prim.attributes_count; ++ai)
            {
                const cgltf_attribute& a = prim.attributes[ai];
                if (a.type == cgltf_attribute_type_joints && a.index == 0)
                    seenJoints = true;
                else if (a.type == cgltf_attribute_type_weights && a.index == 0)
                    seenWeights = true;
            }
            if (seenJoints && seenWeights)
            {
                hasSkin = true;
                break;
            }
        }
    }

    const std::string directory = directoryOf(filename);
    // Only needed for an embedded image's own extraction (resolveImageFile())
    // - an external one is still read the normal search-path-aware way via
    // `directory` above, so this staying empty (an unresolvable `filename`)
    // only means embedded textures fall back to importing blank, not that
    // nothing imports at all.
    const std::string resolvedFilename = files.resolve(filename);
    const std::string realDirectory =
        directoryOf(resolvedFilename.empty() ? filename : resolvedFilename);
    // A .glb carries every texture inside the binary chunk, so importing one
    // writes them out as real files. They go in a folder of their own named
    // after the mesh rather than loose beside it - a single flight helmet is
    // fifteen images, and dropping those into whatever directory the .glb
    // happened to sit in buries it.
    const std::string textureFolder = stem(filename) + "_textures";
    std::vector<const cgltf_material*> materialSources;

    for (cgltf_size ni = 0; ni < gltf->nodes_count; ++ni)
    {
        const cgltf_node& node = gltf->nodes[ni];
        if (!node.mesh)
            continue;

        const bool skinned = node.skin != nullptr && node.skin->joints_count > 0;
        // A skinned mesh is stored in its own node-local space and driven by
        // the skeleton; node transforms are only baked into static geometry.
        const glm::mat4 nodeXform = skinned ? glm::mat4(1.0f) : nodeGlobalMatrix(node);
        const glm::mat3 normalMatrix =
            skinned ? glm::mat3(1.0f)
                    : glm::mat3(glm::transpose(glm::inverse(glm::mat3(nodeXform))));
        const cgltf_size jointCount = node.skin ? node.skin->joints_count : 0;

        for (cgltf_size pi = 0; pi < node.mesh->primitives_count; ++pi)
        {
            const cgltf_primitive& prim = node.mesh->primitives[pi];
            if (prim.type != cgltf_primitive_type_triangles)
                continue;

            cgltf_accessor* pos = nullptr;
            cgltf_accessor* nor = nullptr;
            cgltf_accessor* tan = nullptr;
            cgltf_accessor* uv = nullptr;
            cgltf_accessor* jnt = nullptr;
            cgltf_accessor* wgt = nullptr;
            for (cgltf_size ai = 0; ai < prim.attributes_count; ++ai)
            {
                const cgltf_attribute& a = prim.attributes[ai];
                if (a.type == cgltf_attribute_type_position)
                    pos = a.data;
                else if (a.type == cgltf_attribute_type_normal)
                    nor = a.data;
                else if (a.type == cgltf_attribute_type_tangent)
                    tan = a.data;
                else if (a.type == cgltf_attribute_type_texcoord && a.index == 0)
                    uv = a.data;
                else if (a.type == cgltf_attribute_type_joints && a.index == 0)
                    jnt = a.data;
                else if (a.type == cgltf_attribute_type_weights && a.index == 0)
                    wgt = a.data;
            }
            if (!pos)
                continue;

            const u32 vertexBase = static_cast<u32>(mesh.positions.size());
            const u32 firstIndex = static_cast<u32>(mesh.indices.size());
            const cgltf_size vertexCount = pos->count;

            mesh.positions.resize(vertexBase + vertexCount);
            mesh.normals.resize(vertexBase + vertexCount);
            mesh.tangents.resize(vertexBase + vertexCount);
            mesh.uvs.resize(vertexBase + vertexCount);
            if (hasSkin)
                mesh.skin.resize(vertexBase + vertexCount);

            for (cgltf_size vi = 0; vi < vertexCount; ++vi)
            {
                f32 f[4] = {0.0f, 0.0f, 0.0f, 0.0f};

                cgltf_accessor_read_float(pos, vi, f, 3);
                const glm::vec3 p(f[0], f[1], f[2]);
                mesh.positions[vertexBase + vi] =
                    skinned ? p : glm::vec3(nodeXform * glm::vec4(p, 1.0f));

                glm::vec3 n(0.0f, 1.0f, 0.0f);
                if (nor)
                {
                    cgltf_accessor_read_float(nor, vi, f, 3);
                    n = glm::vec3(f[0], f[1], f[2]);
                    if (!skinned && glm::dot(n, n) > 1e-8f)
                        n = glm::normalize(normalMatrix * n);
                }
                mesh.normals[vertexBase + vi] = n;

                glm::vec4 tangent(1.0f, 0.0f, 0.0f, 1.0f);
                if (tan)
                {
                    cgltf_accessor_read_float(tan, vi, f, 4);
                    tangent = glm::vec4(f[0], f[1], f[2], f[3]);
                }
                mesh.tangents[vertexBase + vi] = tangent;

                glm::vec2 uvCoord(0.0f, 0.0f);
                if (uv)
                {
                    cgltf_accessor_read_float(uv, vi, f, 2);
                    uvCoord = glm::vec2(f[0], f[1]);
                }
                mesh.uvs[vertexBase + vi] = uvCoord;

                if (hasSkin)
                {
                    MeshSkinVertex skinVertex;
                    if (jnt && wgt)
                    {
                        cgltf_uint ji[4] = {0, 0, 0, 0};
                        f32 wv[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                        cgltf_accessor_read_uint(jnt, vi, ji, 4);
                        cgltf_accessor_read_float(wgt, vi, wv, 4);
                        for (int k = 0; k < 4; ++k)
                            skinVertex.joints[k] = clampBone(ji[k], jointCount);
                        const f32 sum = wv[0] + wv[1] + wv[2] + wv[3];
                        if (sum > 1e-6f)
                            skinVertex.weights =
                                glm::vec4(wv[0] / sum, wv[1] / sum, wv[2] / sum, wv[3] / sum);
                    }
                    mesh.skin[vertexBase + vi] = skinVertex;
                }
            }

            if (prim.indices)
            {
                for (cgltf_size ii = 0; ii < prim.indices->count; ++ii)
                    mesh.indices.push_back(
                        vertexBase + static_cast<u32>(cgltf_accessor_read_index(prim.indices, ii)));
            }
            else
            {
                for (cgltf_size vi = 0; vi < vertexCount; ++vi)
                    mesh.indices.push_back(vertexBase + static_cast<u32>(vi));
            }

            SubMesh submesh;
            submesh.indexOffset = firstIndex;
            submesh.indexCount = static_cast<u32>(mesh.indices.size()) - firstIndex;
            submesh.materialSlot =
                materialSlotFor(prim, directory, realDirectory, textureFolder, materialSources,
                                mesh);
            mesh.submeshes.push_back(submesh);
        }
    }

    if (mesh.positions.empty() || mesh.indices.empty())
    {
        Log::error("GltfImporter: '%s' has no usable geometry", filename.c_str());
        mesh.clear();
        cgltf_free(gltf);
        return false;
    }

    mesh.bounds = AABB();
    for (const glm::vec3& position : mesh.positions)
        mesh.bounds.expand(position);
    for (SubMesh& submesh : mesh.submeshes)
    {
        submesh.bounds = AABB();
        const u32 endIndex = submesh.indexOffset + submesh.indexCount;
        for (u32 i = submesh.indexOffset; i < endIndex; ++i)
            submesh.bounds.expand(mesh.positions[mesh.indices[i]]);
    }

    Log::info("GltfImporter: '%s' - %u verts, %u indices, %zu submeshes, %zu materials",
              filename.c_str(), static_cast<u32>(mesh.positions.size()),
              static_cast<u32>(mesh.indices.size()), mesh.submeshes.size(), mesh.materials.size());

    cgltf_free(gltf);
    return true;
}

// ------------------------------------------------------------ skeleton/anim

bool loadGltfSkeleton(const std::string& filename, FileSystem& files, Skeleton& skeleton)
{
    ByteArray data = files.readBinary(filename);
    if (data.empty())
    {
        Log::error("GltfImporter: could not read '%s'", filename.c_str());
        return false;
    }

    cgltf_data* gltf = parseGltf(files, data.data(), data.size(), filename);
    if (!gltf)
    {
        Log::error("GltfImporter: '%s' failed to parse", filename.c_str());
        return false;
    }

    // first node with a mesh + skin
    const cgltf_skin* skin = nullptr;
    for (cgltf_size i = 0; i < gltf->nodes_count; ++i)
    {
        if (gltf->nodes[i].mesh && gltf->nodes[i].skin && gltf->nodes[i].skin->joints_count > 0)
        {
            skin = gltf->nodes[i].skin;
            break;
        }
    }
    if (!skin)
    {
        Log::error("GltfImporter: '%s' has no skinned node", filename.c_str());
        cgltf_free(gltf);
        return false;
    }

    std::unordered_map<const cgltf_node*, int> boneOf;
    for (cgltf_size i = 0; i < skin->joints_count; ++i)
        boneOf[skin->joints[i]] = static_cast<int>(i);

    skeleton = Skeleton();
    for (cgltf_size i = 0; i < skin->joints_count; ++i)
    {
        const cgltf_node* joint = skin->joints[i];
        const char* name = (joint && joint->name) ? joint->name : nullptr;
        std::string fallback = "joint_" + std::to_string(i);

        s32 parent = -1;
        if (joint && joint->parent)
        {
            auto it = boneOf.find(joint->parent);
            if (it != boneOf.end())
                parent = static_cast<s32>(it->second);
        }

        glm::mat4 inverseBind = glm::mat4(1.0f);
        if (skin->inverse_bind_matrices && i < skin->inverse_bind_matrices->count)
        {
            f32 ibm[16];
            cgltf_accessor_read_float(skin->inverse_bind_matrices, i, ibm, 16);
            inverseBind = glm::make_mat4(ibm);
        }

        const glm::mat4 bindLocal = nodeLocalMatrix(*joint);

        if (!skeleton.addBone(name ? name : fallback.c_str(), parent, bindLocal, inverseBind))
        {
            cgltf_free(gltf);
            return false;
        }
    }
    cgltf_free(gltf);
    return skeleton.finalize();
}

bool loadGltfAnimation(const std::string& filename, FileSystem& files, const Skeleton& skeleton,
                       AnimationClip& clip)
{
    if (skeleton.empty())
        return false;

    ByteArray data = files.readBinary(filename);
    if (data.empty())
    {
        Log::error("GltfImporter: could not read '%s'", filename.c_str());
        return false;
    }

    cgltf_data* gltf = parseGltf(files, data.data(), data.size(), filename);
    if (!gltf)
    {
        Log::error("GltfImporter: '%s' failed to parse", filename.c_str());
        return false;
    }
    if (gltf->animations_count == 0)
    {
        Log::error("GltfImporter: '%s' has no animations", filename.c_str());
        cgltf_free(gltf);
        return false;
    }

    // Channels target nodes by pointer, but this file has its own node
    // objects - rebind by name against the already-loaded skeleton, same
    // contract as the native format's animation loader.
    std::unordered_map<const cgltf_node*, int> boneOf;
    for (cgltf_size i = 0; i < gltf->nodes_count; ++i)
    {
        const cgltf_node& node = gltf->nodes[i];
        if (!node.name)
            continue;
        const s32 index = skeleton.findBone(node.name);
        if (index >= 0)
            boneOf[&node] = index;
    }

    std::vector<LocalPose> bindPose(skeleton.boneCount());
    skeleton.bindPose(bindPose);

    clip = AnimationClip();
    for (cgltf_size ai = 0; ai < gltf->animations_count; ++ai)
    {
        const cgltf_animation& animation = gltf->animations[ai];
        AnimationClip candidate;
        candidate.setName((animation.name && animation.name[0]) ? animation.name : stem(filename));
        buildClipTracks(animation, boneOf, bindPose, candidate);
        if (candidate.tracks().empty())
            continue;
        clip = std::move(candidate);
        Log::info("GltfImporter: anim '%s' dur=%.3f tracks=%zu", clip.name().c_str(),
                  clip.duration(), clip.tracks().size());
        cgltf_free(gltf);
        return true;
    }

    Log::error("GltfImporter: '%s' - no channel targeted a bone in the loaded skeleton",
               filename.c_str());
    cgltf_free(gltf);
    return false;
}

} // namespace Radion
