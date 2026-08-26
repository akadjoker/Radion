#include "PCH.h"

#include "OgreMeshImporter.h"

#include "ByteArray.h"
#include "FileSystem.h"
#include "Hash.h"
#include "Log.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <unordered_map>

#include "Math.h"

namespace Radion
{

namespace
{

// Chunk ids (OgreMain/include/OgreMeshFileFormat.h).
constexpr u16 M_HEADER = 0x1000;
constexpr u16 M_MESH = 0x3000;
constexpr u16 M_SUBMESH = 0x4000;
constexpr u16 M_SUBMESH_OPERATION = 0x4010;
constexpr u16 M_SUBMESH_BONE_ASSIGNMENT = 0x4100;
constexpr u16 M_SUBMESH_TEXTURE_ALIAS = 0x4200;
constexpr u16 M_GEOMETRY = 0x5000;
constexpr u16 M_GEOMETRY_VERTEX_DECLARATION = 0x5100;
constexpr u16 M_GEOMETRY_VERTEX_ELEMENT = 0x5110;
constexpr u16 M_GEOMETRY_VERTEX_BUFFER = 0x5200;
constexpr u16 M_GEOMETRY_VERTEX_BUFFER_DATA = 0x5210;
constexpr u16 M_MESH_SKELETON_LINK = 0x6000;
constexpr u16 M_MESH_BONE_ASSIGNMENT = 0x7000;

constexpr u16 SKELETON_HEADER = 0x1000;
constexpr u16 SKELETON_BLENDMODE = 0x1010;
constexpr u16 SKELETON_BONE = 0x2000;
constexpr u16 SKELETON_BONE_PARENT = 0x3000;
constexpr u16 SKELETON_ANIMATION = 0x4000;
constexpr u16 SKELETON_ANIMATION_BASEINFO = 0x4010;
constexpr u16 SKELETON_ANIMATION_TRACK = 0x4100;
constexpr u16 SKELETON_ANIMATION_TRACK_KEYFRAME = 0x4110;
constexpr u16 SKELETON_ANIMATION_LINK = 0x5000;

constexpr usize OGRE_CHUNK_HEADER_SIZE = 6;
constexpr usize MAX_OGRE_STRING_LENGTH = 4096;
constexpr u32 MAX_OGRE_VERTICES = 16u * 1024u * 1024u;
constexpr u32 MAX_OGRE_INDICES = 64u * 1024u * 1024u;

// Vertex element semantics (Ogre's VertexElementSemantic).
constexpr u16 OGRE_VES_POSITION = 1;
constexpr u16 OGRE_VES_NORMAL = 4;
constexpr u16 OGRE_VES_DIFFUSE = 5;
constexpr u16 OGRE_VES_TEXTURE_COORDINATES = 7;
constexpr u16 OGRE_VES_TANGENT = 9;

// Vertex element types (Ogre's VertexElementType).
constexpr u16 OGRE_VET_FLOAT1 = 0;
constexpr u16 OGRE_VET_FLOAT2 = 1;
constexpr u16 OGRE_VET_FLOAT3 = 2;
constexpr u16 OGRE_VET_FLOAT4 = 3;
constexpr u16 OGRE_VET_COLOUR = 4;
constexpr u16 OGRE_VET_COLOUR_ARGB = 10;
constexpr u16 OGRE_VET_COLOUR_ABGR = 11;

struct OgreVertex
{
    float x = 0, y = 0, z = 0;
    float nx = 0, ny = 0, nz = 0;
    // UV set 0 (the base texture's tiling unwrap) and set 1 (a baked
    // lightmap's own unwrap). MeshData::uvs/uvs2 carry exactly these two;
    // a mesh with a third set still loses it.
    float u = 0, v = 0;
    float u1 = 0, v1 = 0;
    float tx = 0, ty = 0, tz = 0, tw = 1.0f;
    u8 color[4] = {255, 255, 255, 255};
};

// What a given geometry chunk actually carries.
struct GeometryLayout
{
    bool hasNormal = false;
    bool hasTangent = false;
    bool hasColor = false;
    bool hasUV0 = false;
    bool hasUV1 = false;
    bool droppedExtraUVSets = false;
};

struct ChunkHeader
{
    u16 id;
    u32 length; // includes this 6-byte header
};

bool readChunkHeader(ByteArray& r, usize parentEnd, ChunkHeader& h, usize& childEnd)
{
    if (parentEnd > r.size() || r.tell() > parentEnd || parentEnd - r.tell() < OGRE_CHUNK_HEADER_SIZE)
        return false;
    h.id = r.readU16();
    h.length = r.readU32();
    if (h.length < OGRE_CHUNK_HEADER_SIZE)
        return false;
    const usize payloadLength = static_cast<usize>(h.length - OGRE_CHUNK_HEADER_SIZE);
    if (payloadLength > parentEnd - r.tell())
        return false;
    childEnd = r.tell() + payloadLength;
    return true;
}

bool readOgreString(ByteArray& r, usize endPos, std::string& value)
{
    if (endPos > r.size() || r.tell() > endPos)
        return false;
    const usize available = endPos - r.tell();
    const usize scanLength = std::min(available, MAX_OGRE_STRING_LENGTH + usize(1));
    const u8* begin = r.data() + r.tell();
    const void* terminator = std::memchr(begin, '\n', scanLength);
    if (!terminator)
        return false;
    const usize length = static_cast<const u8*>(terminator) - begin;
    value.assign(reinterpret_cast<const char*>(begin), length);
    r.seek(static_cast<long long>(r.tell() + length + 1));
    return true;
}

usize vertexElementSize(u16 type)
{
    switch (type)
    {
    case OGRE_VET_FLOAT1: return sizeof(float);
    case OGRE_VET_FLOAT2: return sizeof(float) * 2;
    case OGRE_VET_FLOAT3: return sizeof(float) * 3;
    case OGRE_VET_FLOAT4: return sizeof(float) * 4;
    case OGRE_VET_COLOUR:
    case OGRE_VET_COLOUR_ARGB:
    case OGRE_VET_COLOUR_ABGR: return sizeof(u32);
    default: return 0;
    }
}

bool readVec3(ByteArray& r, usize end, Math::vec3& value)
{
    if (end > r.size() || r.tell() > end || end - r.tell() < sizeof(float) * 3)
        return false;
    value.x = r.readF32();
    value.y = r.readF32();
    value.z = r.readF32();
    return true;
}

bool readQuat(ByteArray& r, usize end, Math::quat& value)
{
    if (end > r.size() || r.tell() > end || end - r.tell() < sizeof(float) * 4)
        return false;
    // OgreSerializer writes quaternions as x, y, z, w even though Ogre's
    // Quaternion fields and constructor are ordered w, x, y, z.
    value.x = r.readF32();
    value.y = r.readF32();
    value.z = r.readF32();
    value.w = r.readF32();
    const float length2 = Math::dot(value, value);
    if (!std::isfinite(length2) || length2 <= 1e-12f)
        return false;
    value = Math::normalize(value);
    return true;
}

Math::mat4 localMatrix(const Math::vec3& position, const Math::quat& rotation,
                      const Math::vec3& scale)
{
    return Math::translate(Math::mat4(1.0f), position) * Math::mat4_cast(rotation) *
           Math::scale(Math::mat4(1.0f), scale);
}

struct OgreBoneData
{
    std::string name;
    u16 handle = 0;
    s32 parent = -1;
    Math::vec3 position = Math::vec3(0.0f);
    Math::quat rotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    Math::vec3 scale = Math::vec3(1.0f);
};

struct PendingTrack
{
    u16 handle = 0;
    std::vector<f32> times;
    std::vector<Math::vec3> translations;
    std::vector<Math::quat> rotations;
    std::vector<Math::vec3> scales;
};

struct PendingAnimation
{
    std::string name;
    f32 duration = 0.0f;
    std::vector<PendingTrack> tracks;
};

bool readSkeletonAnimation(ByteArray& r, usize animationEnd, PendingAnimation& animation)
{
    if (!readOgreString(r, animationEnd, animation.name) || animationEnd - r.tell() < sizeof(f32))
        return false;
    animation.duration = r.readF32();
    if (!std::isfinite(animation.duration) || animation.duration < 0.0f)
        return false;

    while (r.tell() < animationEnd)
    {
        ChunkHeader chunk;
        usize chunkEnd = 0;
        if (!readChunkHeader(r, animationEnd, chunk, chunkEnd))
            return false;
        if (chunk.id == SKELETON_ANIMATION_BASEINFO)
        {
            // Base animation baking is handled in a later pass. Parse it now
            // so it is never mistaken for a track or silently read as bytes.
            std::string baseName;
            if (!readOgreString(r, chunkEnd, baseName) || chunkEnd - r.tell() != sizeof(f32))
                return false;
            r.readF32();
        }
        else if (chunk.id == SKELETON_ANIMATION_TRACK)
        {
            if (chunkEnd - r.tell() < sizeof(u16))
                return false;
            PendingTrack track;
            track.handle = r.readU16();
            while (r.tell() < chunkEnd)
            {
                ChunkHeader keyChunk;
                usize keyEnd = 0;
                if (!readChunkHeader(r, chunkEnd, keyChunk, keyEnd) ||
                    keyChunk.id != SKELETON_ANIMATION_TRACK_KEYFRAME)
                    return false;
                constexpr usize required = sizeof(f32) * 8;
                const usize payload = keyEnd - r.tell();
                if (payload != required && payload != required + sizeof(f32) * 3)
                    return false;
                const f32 time = r.readF32();
                Math::quat rotation;
                Math::vec3 translation;
                Math::vec3 scale(1.0f);
                if (!std::isfinite(time) || !readQuat(r, keyEnd, rotation) ||
                    !readVec3(r, keyEnd, translation))
                    return false;
                if (r.tell() < keyEnd && !readVec3(r, keyEnd, scale))
                    return false;
                if (!track.times.empty() && time < track.times.back())
                    return false;
                track.times.push_back(time);
                track.rotations.push_back(rotation);
                track.translations.push_back(translation);
                track.scales.push_back(scale);
            }
            animation.tracks.push_back(std::move(track));
        }
        else
            r.seek(static_cast<long long>(chunkEnd));
    }
    return true;
}

bool loadOgreSkeletonBytes(const std::string& filename, ByteArray& r, Skeleton& skeleton,
                           std::vector<AnimationClip>& clips,
                           std::unordered_map<u16, u8>& handleMap)
{
    if (!r.canRead(sizeof(u16)) || r.readU16() != SKELETON_HEADER)
        return false;
    std::string version;
    if (!readOgreString(r, r.size(), version) ||
        (version != "[Serializer_v1.10]" && version != "[Serializer_v1.80]"))
    {
        Log::error("OgreSkeletonImporter: '%s' has unsupported version '%s'", filename.c_str(),
                   version.c_str());
        return false;
    }

    std::vector<OgreBoneData> bones;
    std::unordered_map<u16, usize> tempByHandle;
    std::vector<std::pair<u16, u16>> parents;
    std::vector<PendingAnimation> animations;
    while (r.tell() < r.size())
    {
        ChunkHeader chunk;
        usize chunkEnd = 0;
        if (!readChunkHeader(r, r.size(), chunk, chunkEnd))
            return false;
        if (chunk.id == SKELETON_BLENDMODE)
        {
            if (chunkEnd - r.tell() != sizeof(u16))
                return false;
            r.readU16();
        }
        else if (chunk.id == SKELETON_BONE)
        {
            OgreBoneData bone;
            // Serializer_v1.10's historical calcBoneSize() omits the bone
            // name even though the bytes are written. Ogre compensates for
            // this internally; mirror that adjustment before using the end.
            if (!readOgreString(r, r.size(), bone.name) ||
                bone.name.size() + 1 > r.size() - chunkEnd)
                return false;
            chunkEnd += bone.name.size() + 1;
            if (chunkEnd - r.tell() != 30 && chunkEnd - r.tell() != 42)
                return false;
            bone.handle = r.readU16();
            if (bone.handle > 255 || tempByHandle.count(bone.handle) != 0 ||
                !readVec3(r, chunkEnd, bone.position) || !readQuat(r, chunkEnd, bone.rotation))
                return false;
            if (r.tell() < chunkEnd && !readVec3(r, chunkEnd, bone.scale))
                return false;
            tempByHandle[bone.handle] = bones.size();
            bones.push_back(std::move(bone));
        }
        else if (chunk.id == SKELETON_BONE_PARENT)
        {
            if (chunkEnd - r.tell() != sizeof(u16) * 2)
                return false;
            const u16 child = r.readU16();
            const u16 parent = r.readU16();
            parents.emplace_back(child, parent);
        }
        else if (chunk.id == SKELETON_ANIMATION)
        {
            PendingAnimation animation;
            if (!readSkeletonAnimation(r, chunkEnd, animation))
                return false;
            animations.push_back(std::move(animation));
        }
        else if (chunk.id == SKELETON_ANIMATION_LINK)
        {
            std::string link;
            if (!readOgreString(r, chunkEnd, link) || chunkEnd - r.tell() != sizeof(f32))
                return false;
            Log::error("OgreSkeletonImporter: linked animation skeleton '%s' is not yet supported",
                       link.c_str());
            return false;
        }
        else
            r.seek(static_cast<long long>(chunkEnd)); // blend mode / linked animations
    }

    for (const auto& relation : parents)
    {
        const auto child = tempByHandle.find(relation.first);
        const auto parent = tempByHandle.find(relation.second);
        if (child == tempByHandle.end() || parent == tempByHandle.end() || child == parent)
            return false;
        bones[child->second].parent = static_cast<s32>(parent->second);
    }

    std::vector<Math::mat4> globals(bones.size(), Math::mat4(1.0f));
    std::vector<u8> state(bones.size(), 0);
    std::function<bool(usize)> buildGlobal = [&](usize index)
    {
        if (state[index] == 2)
            return true;
        if (state[index] == 1)
            return false;
        state[index] = 1;
        const s32 parent = bones[index].parent;
        if (parent >= 0 && (!buildGlobal(static_cast<usize>(parent))))
            return false;
        const Math::mat4 local = localMatrix(bones[index].position, bones[index].rotation,
                                           bones[index].scale);
        globals[index] = parent >= 0 ? globals[static_cast<usize>(parent)] * local : local;
        state[index] = 2;
        return true;
    };
    for (usize i = 0; i < bones.size(); ++i)
        if (!buildGlobal(i))
            return false;

    for (usize i = 0; i < bones.size(); ++i)
    {
        handleMap[bones[i].handle] = static_cast<u8>(i);
        if (!skeleton.addBone(bones[i].name, bones[i].parent,
                              localMatrix(bones[i].position, bones[i].rotation, bones[i].scale),
                              Math::inverse(globals[i])))
            return false;
    }
    if (!skeleton.finalize())
        return false;

    for (PendingAnimation& source : animations)
    {
        AnimationClip clip;
        clip.setName(source.name);
        clip.setDuration(source.duration);
        for (PendingTrack& sourceTrack : source.tracks)
        {
            const auto mapped = tempByHandle.find(sourceTrack.handle);
            if (mapped == tempByHandle.end())
                return false;
            const OgreBoneData& bind = bones[mapped->second];
            BoneTrack track;
            track.bone = static_cast<s32>(mapped->second);
            track.times = std::move(sourceTrack.times);
            for (usize key = 0; key < track.times.size(); ++key)
            {
                track.positions.push_back(bind.position + sourceTrack.translations[key]);
                Math::quat rotation = Math::normalize(bind.rotation * sourceTrack.rotations[key]);
                if (!track.rotations.empty() && Math::dot(track.rotations.back(), rotation) < 0.0f)
                    rotation = -rotation;
                track.rotations.push_back(rotation);
                track.scales.push_back(bind.scale * sourceTrack.scales[key]);
            }
            clip.tracks().push_back(std::move(track));
        }
        clips.push_back(std::move(clip));
    }
    return true;
}

struct VertexElementDecl
{
    u16 source, type, semantic, offset, index;
};

struct VertexBufferBind
{
    u16 vertexSize;
    std::vector<u8> data;
};

struct OgreBoneAssignment
{
    u32 vertex = 0;
    u16 bone = 0;
    f32 weight = 0.0f;
};

bool readBoneAssignment(ByteArray& r, usize end, OgreBoneAssignment& assignment)
{
    if (end - r.tell() != sizeof(u32) + sizeof(u16) + sizeof(f32))
        return false;
    assignment.vertex = r.readU32();
    assignment.bone = r.readU16();
    assignment.weight = r.readF32();
    return assignment.bone <= 255 && std::isfinite(assignment.weight) && assignment.weight >= 0.0f;
}

void applyAssignments(MeshData& mesh, u32 vertexBase, usize vertexCount,
                      const std::vector<OgreBoneAssignment>& assignments)
{
    if (mesh.skin.size() < vertexBase + vertexCount)
        mesh.skin.resize(vertexBase + vertexCount);
    std::vector<std::vector<OgreBoneAssignment>> perVertex(vertexCount);
    for (const OgreBoneAssignment& assignment : assignments)
        if (assignment.vertex < vertexCount && assignment.weight > 0.0f)
            perVertex[assignment.vertex].push_back(assignment);

    for (usize vertex = 0; vertex < vertexCount; ++vertex)
    {
        auto& influences = perVertex[vertex];
        std::stable_sort(influences.begin(), influences.end(),
                         [](const OgreBoneAssignment& a, const OgreBoneAssignment& b)
                         { return a.weight > b.weight; });
        const usize count = std::min<usize>(4, influences.size());
        f32 total = 0.0f;
        for (usize i = 0; i < count; ++i)
            total += influences[i].weight;
        MeshSkinVertex& skin = mesh.skin[vertexBase + vertex];
        if (total <= 1e-8f)
            continue; // explicit fallback remains joint 0 with weight 1
        skin.weights = Math::vec4(0.0f);
        for (usize i = 0; i < count; ++i)
        {
            skin.joints[i] = static_cast<u8>(influences[i].bone);
            skin.weights[static_cast<int>(i)] = influences[i].weight / total;
        }
    }
}

// M_GEOMETRY: vertex count, a declaration (one element per attribute) and one
// or more raw interleaved vertex buffers, one per declared "source" (bind
// index). Extracts POSITION/NORMAL/TANGENT/DIFFUSE and TEXCOORD sets 0/1 -
// what MeshData (uvs/uvs2) can express.
bool readGeometry(ByteArray& r, usize endPos, std::vector<OgreVertex>& outVerts,
                  GeometryLayout& outLayout)
{
    if (endPos > r.size() || r.tell() > endPos || endPos - r.tell() < sizeof(u32))
        return false;
    const u32 vertexCount = r.readU32();
    if (vertexCount > MAX_OGRE_VERTICES)
        return false;

    std::vector<VertexElementDecl> decl;
    std::map<u16, VertexBufferBind> buffers;

    while (r.tell() < endPos)
    {
        ChunkHeader ch;
        usize childEnd = 0;
        if (!readChunkHeader(r, endPos, ch, childEnd))
            return false;

        if (ch.id == M_GEOMETRY_VERTEX_DECLARATION)
        {
            while (r.tell() < childEnd)
            {
                ChunkHeader elementHeader;
                usize elementEnd = 0;
                if (!readChunkHeader(r, childEnd, elementHeader, elementEnd) ||
                    elementHeader.id != M_GEOMETRY_VERTEX_ELEMENT || elementEnd - r.tell() != 10)
                    return false;
                VertexElementDecl e;
                e.source = r.readU16();
                e.type = r.readU16();
                e.semantic = r.readU16();
                e.offset = r.readU16();
                e.index = r.readU16();
                decl.push_back(e);
                if (r.tell() != elementEnd)
                    return false;
            }
        }
        else if (ch.id == M_GEOMETRY_VERTEX_BUFFER)
        {
            if (childEnd - r.tell() < 4)
                return false;
            const u16 bindIndex = r.readU16();
            const u16 vertexSize = r.readU16();
            ChunkHeader dataHeader;
            usize dataEnd = 0;
            if (!readChunkHeader(r, childEnd, dataHeader, dataEnd) ||
                dataHeader.id != M_GEOMETRY_VERTEX_BUFFER_DATA)
                return false;
            if (vertexSize == 0 || static_cast<usize>(vertexCount) >
                                       std::numeric_limits<usize>::max() / vertexSize)
                return false;
            const usize dataLen = dataEnd - r.tell();
            if (dataLen != static_cast<usize>(vertexCount) * vertexSize)
                return false;
            VertexBufferBind bind;
            bind.vertexSize = vertexSize;
            bind.data.resize(dataLen);
            r.readBytes(bind.data.data(), dataLen);
            buffers[bindIndex] = std::move(bind);
            if (r.tell() != childEnd)
                return false;
        }
        else
        {
            r.seek(static_cast<long long>(childEnd)); // unrecognised at this level - skip
        }
    }

    const VertexElementDecl *posEl = nullptr, *normEl = nullptr, *tanEl = nullptr, *colEl = nullptr;
    const VertexElementDecl *uv0El = nullptr, *uv1El = nullptr;
    bool sawExtraUV = false;
    for (const VertexElementDecl& e : decl)
    {
        if (e.semantic == OGRE_VES_POSITION)
            posEl = &e;
        else if (e.semantic == OGRE_VES_NORMAL)
            normEl = &e;
        else if (e.semantic == OGRE_VES_TANGENT)
            tanEl = &e;
        else if (e.semantic == OGRE_VES_DIFFUSE)
            colEl = &e;
        else if (e.semantic == OGRE_VES_TEXTURE_COORDINATES)
        {
            if (e.index == 0)
                uv0El = &e;
            else if (e.index == 1)
                uv1El = &e;
            else
                sawExtraUV = true;
        }
    }
    if (!posEl)
    {
        Log::error("OgreMeshImporter: geometry has no POSITION element");
        return false;
    }

    for (const VertexElementDecl& e : decl)
    {
        const usize elementSize = vertexElementSize(e.type);
        const auto buffer = buffers.find(e.source);
        if (elementSize != 0 && (buffer == buffers.end() || e.offset > buffer->second.vertexSize ||
                                 elementSize > buffer->second.vertexSize - e.offset))
        {
            Log::error("OgreMeshImporter: vertex element exceeds its vertex buffer stride");
            return false;
        }
    }
    if (posEl->type != OGRE_VET_FLOAT3)
    {
        Log::error("OgreMeshImporter: POSITION element is not FLOAT3 (type=%d) - unsupported",
                  posEl->type);
        return false;
    }
    if (normEl && normEl->type != OGRE_VET_FLOAT3)
    {
        Log::error("OgreMeshImporter: NORMAL element is not FLOAT3 (type=%d) - unsupported",
                  normEl->type);
        return false;
    }

    // Optional attributes in an unexpected type are dropped, not fatal: the
    // mesh still renders without a tangent/colour/extra UV set, and refusing
    // the whole asset over one attribute would be worse.
    if (tanEl && tanEl->type != OGRE_VET_FLOAT3 && tanEl->type != OGRE_VET_FLOAT4)
    {
        Log::info("OgreMeshImporter: TANGENT is type=%d (not FLOAT3/FLOAT4) - ignoring it",
                 tanEl->type);
        tanEl = nullptr;
    }
    if (colEl && colEl->type != OGRE_VET_COLOUR && colEl->type != OGRE_VET_COLOUR_ARGB &&
        colEl->type != OGRE_VET_COLOUR_ABGR)
    {
        Log::info("OgreMeshImporter: DIFFUSE is type=%d (not a packed colour) - ignoring it",
                 colEl->type);
        colEl = nullptr;
    }
    if (uv0El && uv0El->type != OGRE_VET_FLOAT2)
    {
        Log::info("OgreMeshImporter: TEXCOORD0 is type=%d (not FLOAT2) - ignoring it",
                 uv0El->type);
        uv0El = nullptr;
    }
    if (uv1El && uv1El->type != OGRE_VET_FLOAT2)
    {
        Log::info("OgreMeshImporter: TEXCOORD1 is type=%d (not FLOAT2) - ignoring it",
                 uv1El->type);
        uv1El = nullptr;
    }

    auto findBuffer = [&](const VertexElementDecl* e) -> const VertexBufferBind*
    {
        if (!e)
            return nullptr;
        auto it = buffers.find(e->source);
        return it == buffers.end() ? nullptr : &it->second;
    };
    const VertexBufferBind* posBuf = findBuffer(posEl);
    if (!posBuf)
    {
        Log::error("OgreMeshImporter: no vertex buffer bound for POSITION's source");
        return false;
    }
    const VertexBufferBind* normBuf = findBuffer(normEl);
    const VertexBufferBind* tanBuf = findBuffer(tanEl);
    const VertexBufferBind* colBuf = findBuffer(colEl);
    const VertexBufferBind* uv0Buf = findBuffer(uv0El);
    const VertexBufferBind* uv1Buf = findBuffer(uv1El);

    outLayout = GeometryLayout();
    outLayout.hasNormal = normBuf != nullptr;
    outLayout.hasTangent = tanBuf != nullptr;
    outLayout.hasColor = colBuf != nullptr;
    outLayout.hasUV0 = uv0Buf != nullptr;
    outLayout.hasUV1 = uv1Buf != nullptr;
    outLayout.droppedExtraUVSets = sawExtraUV;

    outVerts.resize(vertexCount);
    for (u32 i = 0; i < vertexCount; ++i)
    {
        OgreVertex v;
        std::memcpy(&v.x, posBuf->data.data() + static_cast<usize>(i) * posBuf->vertexSize + posEl->offset,
                   sizeof(float) * 3);
        if (normBuf)
            std::memcpy(&v.nx, normBuf->data.data() + static_cast<usize>(i) * normBuf->vertexSize + normEl->offset,
                       sizeof(float) * 3);
        if (tanBuf)
        {
            const usize n = (tanEl->type == OGRE_VET_FLOAT4) ? 4 : 3;
            std::memcpy(&v.tx, tanBuf->data.data() + static_cast<usize>(i) * tanBuf->vertexSize + tanEl->offset,
                       sizeof(float) * n);
        }
        if (colBuf)
        {
            u32 packed = 0;
            std::memcpy(&packed, colBuf->data.data() + static_cast<usize>(i) * colBuf->vertexSize + colEl->offset,
                       sizeof(u32));
            // ARGB/ABGR differ only in whether R or B sits in the low byte;
            // VET_COLOUR is whatever the exporting renderer used, and ABGR
            // (RGBA in memory order on little-endian) is the GL convention.
            if (colEl->type == OGRE_VET_COLOUR_ARGB)
            {
                v.color[0] = static_cast<u8>((packed >> 16) & 0xFF);
                v.color[1] = static_cast<u8>((packed >> 8) & 0xFF);
                v.color[2] = static_cast<u8>(packed & 0xFF);
                v.color[3] = static_cast<u8>((packed >> 24) & 0xFF);
            }
            else
            {
                v.color[0] = static_cast<u8>(packed & 0xFF);
                v.color[1] = static_cast<u8>((packed >> 8) & 0xFF);
                v.color[2] = static_cast<u8>((packed >> 16) & 0xFF);
                v.color[3] = static_cast<u8>((packed >> 24) & 0xFF);
            }
        }
        if (uv0Buf)
            std::memcpy(&v.u, uv0Buf->data.data() + static_cast<usize>(i) * uv0Buf->vertexSize + uv0El->offset,
                       sizeof(float) * 2);
        if (uv1Buf)
            std::memcpy(&v.u1, uv1Buf->data.data() + static_cast<usize>(i) * uv1Buf->vertexSize + uv1El->offset,
                       sizeof(float) * 2);
        outVerts[i] = v;
    }
    return true;
}

// Finds `name` among mesh.materials, appending a new slot if it is not there
// yet. Only the name/nameHash are set here - a .mesh file names its
// materials but does not carry their look, so the rest of each Material
// stays default until MaterialManager::load() replaces the whole vector
// positionally (see the demo that calls it after import()).
u32 materialSlotFor(MeshData& mesh, const std::string& name)
{
    for (usize i = 0; i < mesh.materials.size(); ++i)
        if (mesh.materials[i].name == name)
            return static_cast<u32>(i);
    Material material;
    material.name = name;
    material.nameHash = hashName(name.c_str());
    mesh.materials.push_back(material);
    return static_cast<u32>(mesh.materials.size() - 1);
}

// Appends one submesh's vertices/indices onto MeshData's shared buffers -
// one set of arrays, submeshes as index ranges into it.
u32 appendSubMesh(MeshData& mesh, const std::vector<OgreVertex>& verts,
                  const GeometryLayout& layout, const std::vector<u32>& indices,
                  const std::string& materialName)
{
    const u32 vertexBase = static_cast<u32>(mesh.positions.size());

    mesh.positions.reserve(mesh.positions.size() + verts.size());
    mesh.normals.reserve(mesh.normals.size() + verts.size());
    mesh.tangents.reserve(mesh.tangents.size() + verts.size());
    mesh.uvs.reserve(mesh.uvs.size() + verts.size());
    mesh.uvs2.reserve(mesh.uvs2.size() + verts.size());
    mesh.colors.reserve(mesh.colors.size() + verts.size());

    for (const OgreVertex& v : verts)
    {
        mesh.positions.emplace_back(v.x, v.y, v.z);
        mesh.normals.emplace_back(layout.hasNormal ? Math::vec3(v.nx, v.ny, v.nz)
                                                   : Math::vec3(0.0f, 1.0f, 0.0f));
        mesh.tangents.emplace_back(layout.hasTangent ? Math::vec4(v.tx, v.ty, v.tz, v.tw)
                                                     : Math::vec4(1.0f, 0.0f, 0.0f, 1.0f));
        mesh.uvs.emplace_back(layout.hasUV0 ? Math::vec2(v.u, v.v) : Math::vec2(0.0f));
        mesh.uvs2.emplace_back(layout.hasUV1 ? Math::vec2(v.u1, v.v1) : Math::vec2(0.0f));
        mesh.colors.push_back(layout.hasColor
                                  ? (static_cast<u32>(v.color[0]) | (static_cast<u32>(v.color[1]) << 8) |
                                     (static_cast<u32>(v.color[2]) << 16) | (static_cast<u32>(v.color[3]) << 24))
                                  : 0xFFFFFFFFu);
    }

    SubMesh submesh;
    submesh.indexOffset = static_cast<u32>(mesh.indices.size());
    submesh.indexCount = static_cast<u32>(indices.size());
    submesh.materialSlot = materialSlotFor(mesh, materialName);

    mesh.indices.reserve(mesh.indices.size() + indices.size());
    for (u32 index : indices)
        mesh.indices.push_back(vertexBase + index);

    mesh.submeshes.push_back(submesh);
    return vertexBase;
}

// M_SUBMESH: material name, index buffer, then either a reference to the
// mesh's shared geometry or its own dedicated M_GEOMETRY. Trailing optional
// chunks (operation type, bone assignments, texture aliases) are skipped in
// bulk via the chunk's own declared end.
bool readSubMesh(ByteArray& r, usize endPos, MeshData& mesh, const std::vector<OgreVertex>* sharedVerts,
                 const GeometryLayout& sharedLayout,
                 std::vector<std::pair<u32, usize>>& sharedCopies, usize meshEnd)
{
    std::string materialName;
    if (!readOgreString(r, endPos, materialName) || endPos - r.tell() < 6)
        return false;
    const bool useSharedVertices = r.readBool();
    const u32 indexCount = r.readU32();
    const bool indexes32Bit = r.readBool();

    if (indexCount > MAX_OGRE_INDICES)
        return false;
    const usize indexSize = indexes32Bit ? sizeof(u32) : sizeof(u16);
    if (static_cast<usize>(indexCount) > (endPos - r.tell()) / indexSize)
        return false;

    std::vector<u32> indices(indexCount);
    for (u32 i = 0; i < indexCount; ++i)
        indices[i] = indexes32Bit ? r.readU32() : static_cast<u32>(r.readU16());

    std::vector<OgreVertex> ownVerts;
    std::vector<OgreBoneAssignment> localAssignments;
    const std::vector<OgreVertex>* verts = nullptr;
    GeometryLayout layout;

    if (useSharedVertices)
    {
        if (!sharedVerts)
        {
            Log::error("OgreMeshImporter: submesh '%s' uses shared vertices but the mesh "
                      "declared none",
                      materialName.c_str());
            return false;
        }
        verts = sharedVerts;
        layout = sharedLayout;
    }
    else
    {
        if (r.tell() >= endPos)
        {
            Log::error("OgreMeshImporter: submesh '%s' has no dedicated geometry",
                      materialName.c_str());
            return false;
        }
        ChunkHeader geomHeader;
        usize geomEnd = 0;
        if (!readChunkHeader(r, endPos, geomHeader, geomEnd))
            return false;
        if (geomHeader.id != M_GEOMETRY)
        {
            Log::error("OgreMeshImporter: expected M_GEOMETRY in submesh '%s', got chunk 0x%x",
                      materialName.c_str(), geomHeader.id);
            return false;
        }
        if (!readGeometry(r, geomEnd, ownVerts, layout))
            return false;
        verts = &ownVerts;
    }

    while (r.tell() < meshEnd)
    {
        const usize headerPosition = r.tell();
        ChunkHeader optional;
        usize optionalEnd = 0;
        if (!readChunkHeader(r, meshEnd, optional, optionalEnd))
            return false;
        if (optional.id != M_SUBMESH_BONE_ASSIGNMENT && optional.id != M_SUBMESH_OPERATION &&
            optional.id != M_SUBMESH_TEXTURE_ALIAS)
        {
            r.seek(static_cast<long long>(headerPosition));
            break;
        }
        if (optional.id == M_SUBMESH_BONE_ASSIGNMENT)
        {
            OgreBoneAssignment assignment;
            if (!readBoneAssignment(r, optionalEnd, assignment))
                return false;
            localAssignments.push_back(assignment);
        }
        else if (optional.id == M_SUBMESH_OPERATION)
        {
            if (optionalEnd - r.tell() != sizeof(u16) || r.readU16() != 4) // OT_TRIANGLE_LIST
                return false;
        }
        else if (optional.id == M_SUBMESH_TEXTURE_ALIAS)
        {
            std::string alias;
            std::string texture;
            if (!readOgreString(r, optionalEnd, alias) || !readOgreString(r, optionalEnd, texture))
                return false;
        }
        r.seek(static_cast<long long>(optionalEnd));
    }

    for (u32 index : indices)
    {
        if (index >= verts->size())
        {
            Log::error("OgreMeshImporter: submesh '%s' contains an out-of-range index",
                       materialName.c_str());
            return false;
        }
    }

    if (layout.droppedExtraUVSets)
        Log::info("OgreMeshImporter: submesh '%s' declares more than two UV sets - only sets "
                 "0/1 are kept",
                 materialName.c_str());

    const u32 vertexBase = appendSubMesh(mesh, *verts, layout, indices, materialName);
    if (useSharedVertices)
        sharedCopies.emplace_back(vertexBase, verts->size());
    else
        applyAssignments(mesh, vertexBase, verts->size(), localAssignments);
    return true;
}

} // namespace

bool OgreMeshImporter::supports(const char* extension) const
{
    return extension && std::strcmp(extension, "mesh") == 0;
}

bool importOgreMesh(const std::string& filename, ByteArray& r, MeshData& mesh,
                    std::string* skeletonLink)
{
    std::string meshSkeletonLink;
    if (!r.canRead(sizeof(u16)))
    {
        Log::error("OgreMeshImporter: '%s' is truncated before its header", filename.c_str());
        return false;
    }
    // M_HEADER: bare u16 id + '\n'-terminated version string, no length field.
    const u16 headerId = r.readU16();
    if (headerId != M_HEADER)
    {
        Log::error("OgreMeshImporter: '%s' is not an Ogre .mesh file (bad header id)",
                  filename.c_str());
        return false;
    }
    std::string version;
    if (!readOgreString(r, r.size(), version))
    {
        Log::error("OgreMeshImporter: '%s' has an invalid or unterminated version string",
                   filename.c_str());
        return false;
    }
    if (version != "[MeshSerializer_v1.100]" && version != "[MeshSerializer_v1.41]")
    {
        Log::error("OgreMeshImporter: '%s' has unsupported version '%s' - supported versions "
                  "are MeshSerializer_v1.100 and MeshSerializer_v1.41",
                  filename.c_str(), version.c_str());
        return false;
    }

    ChunkHeader meshHeader;
    usize meshEnd = 0;
    if (!readChunkHeader(r, r.size(), meshHeader, meshEnd))
    {
        Log::error("OgreMeshImporter: '%s' has a truncated or invalid mesh chunk at offset %zu",
                   filename.c_str(), r.tell());
        return false;
    }
    if (meshHeader.id != M_MESH)
    {
        Log::error("OgreMeshImporter: '%s': expected M_MESH chunk", filename.c_str());
        return false;
    }
    if (r.tell() >= meshEnd)
        return false;
    const bool skeletallyAnimated = r.readBool();
    if (skeletallyAnimated)
        Log::info("OgreMeshImporter: '%s' is marked skeletally animated - skeleton/bone data "
                 "will be ignored (static-only loader)",
                 filename.c_str());

    std::vector<OgreVertex> sharedVertices;
    bool hasShared = false;
    GeometryLayout sharedLayout;
    std::vector<OgreBoneAssignment> sharedAssignments;
    std::vector<std::pair<u32, usize>> sharedCopies;

    while (r.tell() < meshEnd)
    {
        ChunkHeader ch;
        usize childEnd = 0;
        if (!readChunkHeader(r, meshEnd, ch, childEnd))
        {
            Log::error("OgreMeshImporter: '%s' has an invalid child chunk at offset %zu",
                       filename.c_str(), r.tell());
            return false;
        }

        if (ch.id == M_GEOMETRY)
        {
            if (!readGeometry(r, childEnd, sharedVertices, sharedLayout))
                return false;
            hasShared = true;
        }
        else if (ch.id == M_SUBMESH)
        {
            if (!readSubMesh(r, childEnd, mesh, hasShared ? &sharedVertices : nullptr, sharedLayout,
                             sharedCopies, meshEnd))
                return false;
        }
        else if (ch.id == M_MESH_BONE_ASSIGNMENT)
        {
            OgreBoneAssignment assignment;
            if (!readBoneAssignment(r, childEnd, assignment))
                return false;
            sharedAssignments.push_back(assignment);
        }
        else if (ch.id == M_MESH_SKELETON_LINK)
        {
            std::string link;
            if (!readOgreString(r, childEnd, link) || r.tell() != childEnd)
                return false;
            meshSkeletonLink = link;
            if (skeletonLink)
                *skeletonLink = link;
        }
        else
        {
            // Skeleton link, bone assignments, LOD levels, bounds, name
            // table, edge lists, poses, animations - all out of scope for a
            // static mesh loader.
            r.seek(static_cast<long long>(childEnd));
        }
    }

    for (const auto& copy : sharedCopies)
        applyAssignments(mesh, copy.first, copy.second, sharedAssignments);

    if (mesh.submeshes.empty())
    {
        Log::error("OgreMeshImporter: no submeshes found in '%s'", filename.c_str());
        return false;
    }

    if (meshSkeletonLink.empty())
        mesh.skin.clear();

    // Unlike FbxImporter/RadionMeshImporter, this never got a MaterialSkinned
    // pass of its own - MaterialManager::pipelineFor() picks the vertex shader
    // variant off this flag (MATERIAL_SKINNED define), and a skinned mesh
    // whose material lacks it gets the plain-mesh variant instead: the vertex
    // layouts do not match, so nothing renders even though the raw geometry
    // (DebugDraw's outline, drawn straight from the mesh data) is fine.
    if (!mesh.skin.empty())
        for (Material& material : mesh.materials)
            material.flags |= MaterialSkinned;
    return true;
}

bool OgreMeshImporter::import(const std::string& filename, ByteArray& r, FileSystem&, MeshData& mesh)
{
    return importOgreMesh(filename, r, mesh, nullptr);
}

bool loadOgreModel(const std::string& meshFilename, ByteArray& meshBytes, FileSystem& files,
                   OgreModelData& output)
{
    OgreModelData loaded;
    meshBytes.seek(0);
    if (!importOgreMesh(meshFilename, meshBytes, loaded.mesh, &loaded.skeletonLink))
        return false;
    if (loaded.skeletonLink.empty())
    {
        output = std::move(loaded);
        return true;
    }

    std::string normalized = loaded.skeletonLink;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    if (normalized.size() < 9 || normalized.substr(normalized.size() - 9) != ".skeleton")
        normalized += ".skeleton";

    const usize slash = meshFilename.find_last_of("/\\");
    const std::string relative = slash == std::string::npos
                                     ? normalized
                                     : meshFilename.substr(0, slash + 1) + normalized;
    const usize linkSlash = normalized.find_last_of('/');
    const std::string basename = linkSlash == std::string::npos
                                     ? normalized
                                     : normalized.substr(linkSlash + 1);
    const std::string besideMesh = slash == std::string::npos
                                       ? basename
                                       : meshFilename.substr(0, slash + 1) + basename;
    std::string skeletonFilename;
    for (const std::string& candidate : {relative, normalized, besideMesh, basename})
        if (files.exists(candidate))
        {
            skeletonFilename = candidate;
            break;
        }
    if (skeletonFilename.empty())
        skeletonFilename = relative;
    ByteArray skeletonBytes = files.readBinary(skeletonFilename);
    if (skeletonBytes.empty())
    {
        Log::error("OgreModelImporter: '%s' links missing skeleton '%s' (including '%s' and '%s')",
                   meshFilename.c_str(), loaded.skeletonLink.c_str(), relative.c_str(),
                   besideMesh.c_str());
        return false;
    }

    std::unordered_map<u16, u8> handleMap;
    if (!loadOgreSkeletonBytes(skeletonFilename, skeletonBytes, loaded.skeleton,
                               loaded.animations, handleMap))
    {
        Log::error("OgreModelImporter: failed to import skeleton '%s'", skeletonFilename.c_str());
        return false;
    }
    for (MeshSkinVertex& skin : loaded.mesh.skin)
    {
        for (usize influence = 0; influence < 4; ++influence)
        {
            if (skin.weights[static_cast<int>(influence)] <= 0.0f)
                continue;
            const auto mapped = handleMap.find(skin.joints[influence]);
            if (mapped == handleMap.end())
            {
                Log::error("OgreModelImporter: mesh references missing bone handle %u",
                           static_cast<unsigned>(skin.joints[influence]));
                return false;
            }
            skin.joints[influence] = mapped->second;
        }
    }
    loaded.hasSkeleton = true;
    output = std::move(loaded);
    return true;
}

} // namespace Radion
