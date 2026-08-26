#include "PCH.h"

#include "B3DImporter.h"

#include "ByteArray.h"
#include "Skeleton.h"
#include "FileSystem.h"

namespace Radion
{

namespace
{

struct ChunkStack
{
    std::vector<u64> ends;

    void push(ByteArray& b, u64 length)
    {
        ends.push_back(b.tell() + length);
    }

    void pop(ByteArray& b)
    {
        if (!ends.empty())
        {
            b.seek(static_cast<long long>(ends.back()));
            ends.pop_back();
        }
    }

    u64 remaining(const ByteArray& b) const
    {
        return ends.empty() ? 0 : (ends.back() > b.tell() ? ends.back() - b.tell() : 0);
    }
};

std::string readTag(ByteArray& b)
{
    char tag[4] = {};
    if (b.canRead(4))
        b.readBytes(tag, 4);
    return std::string(tag, 4);
}

std::string readCString(ByteArray& b)
{
    std::string out;
    while (b.canRead(1))
    {
        const u8 c = b.readU8();
        if (c == 0)
            break;
        out.push_back(static_cast<char>(c));
    }
    return out;
}

Math::quat readQuat(ByteArray& b)
{
    const f32 w = b.readF32();
    const f32 x = b.readF32();
    const f32 y = b.readF32();
    const f32 z = b.readF32();
    Math::quat q(-w, x, y, z);
    const f32 length = Math::length(q);
    return length > 1e-8f ? q / length : Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
}

bool readHeader(ByteArray& data, ChunkStack& stack)
{
    char root[4] = {};
    if (!data.canRead(8))
        return false;
    data.readBytes(root, 4);
    if (std::string(root, 4) != "BB3D")
        return false;
    const s32 length = data.readS32();
    if (length < 4)
        return false;
    stack.push(data, static_cast<u64>(length));
    data.readS32();
    return true;
}

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

struct StoredVertex
{
    Math::vec3 pos = Math::vec3(0.0f);
    Math::vec3 normal = Math::vec3(0.0f, 1.0f, 0.0f);
    Math::vec2 uv = Math::vec2(0.0f);
    Math::mat4 xform = Math::mat4(1.0f);
    Math::mat3 nmat = Math::mat3(1.0f);
};

struct Surface
{
    u32 firstIndex = 0;
    u32 indexCount = 0;
    s32 brushId = -1;
};

struct Brush
{
    std::string name;
    Math::vec3 diffuse = Math::vec3(1.0f);
    s32 tex0 = -1;
};

struct OutMaterial
{
    std::string name;
    Math::vec3 diffuse = Math::vec3(1.0f);
    std::string texFile;
};

struct B3DContext
{
    explicit B3DContext(ByteArray& input) : b(input)
    {
    }

    ByteArray& b;
    ChunkStack stack;
    std::vector<StoredVertex> verts;
    std::vector<u32> indices;
    std::vector<Surface> surfaces;
    std::vector<WeightSlots> weights;
    std::vector<std::string> texNames;
    std::vector<Brush> brushes;
    s32 boneCount = 0;
    bool sawBones = false;

    void readVRTS(const Math::mat4& xform, const Math::mat3& nmat)
    {
        const s32 flags = b.readS32();
        const s32 numUV = b.readS32();
        const s32 uvSize = b.readS32();
        int stride = 12;
        if (flags & 1)
            stride += 12;
        if (flags & 2)
            stride += 16;
        stride += numUV * uvSize * 4;
        const u64 remaining = stack.remaining(b);
        const int count = stride > 0 ? static_cast<int>(remaining / static_cast<u64>(stride)) : 0;
        for (int i = 0; i < count; ++i)
        {
            StoredVertex vertex;
            vertex.pos.x = b.readF32();
            vertex.pos.y = b.readF32();
            vertex.pos.z = b.readF32();
            if (flags & 1)
            {
                vertex.normal.x = b.readF32();
                vertex.normal.y = b.readF32();
                vertex.normal.z = b.readF32();
            }
            if (flags & 2)
            {
                b.readF32();
                b.readF32();
                b.readF32();
                b.readF32();
            }
            for (int t = 0; t < numUV; ++t)
            {
                f32 u = 0.0f;
                f32 v = 0.0f;
                if (uvSize >= 1)
                    u = b.readF32();
                if (uvSize >= 2)
                    v = b.readF32();
                for (int k = 2; k < uvSize; ++k)
                    b.readF32();
                if (t == 0)
                    vertex.uv = Math::vec2(u, v);
            }
            vertex.xform = xform;
            vertex.nmat = nmat;
            verts.push_back(vertex);
            weights.emplace_back();
        }
    }

    void readTRIS(u32 vertexStart)
    {
        const s32 brushId = b.readS32();
        Surface surface;
        surface.brushId = brushId;
        surface.firstIndex = static_cast<u32>(indices.size());
        const int triCount = static_cast<int>(stack.remaining(b) / 12);
        for (int i = 0; i < triCount; ++i)
        {
            const s32 i0 = b.readS32();
            const s32 i1 = b.readS32();
            const s32 i2 = b.readS32();
            indices.push_back(vertexStart + static_cast<u32>(i0));
            indices.push_back(vertexStart + static_cast<u32>(i1));
            indices.push_back(vertexStart + static_cast<u32>(i2));
        }
        surface.indexCount = static_cast<u32>(indices.size()) - surface.firstIndex;
        if (surface.indexCount > 0)
            surfaces.push_back(surface);
    }

    void parseNode(const Math::mat4& parentGlobal)
    {
        readCString(b);
        Math::vec3 pos;
        Math::vec3 scale(1.0f);
        pos.x = b.readF32();
        pos.y = b.readF32();
        pos.z = b.readF32();
        scale.x = b.readF32();
        scale.y = b.readF32();
        scale.z = b.readF32();
        const Math::quat rotation = readQuat(b);
        const s32 myBone = boneCount++;

        const Math::mat4 local = Math::translate(Math::mat4(1.0f), pos) * Math::mat4_cast(rotation) *
                                Math::scale(Math::mat4(1.0f), scale);
        const Math::mat4 global = parentGlobal * local;
        const Math::mat3 normalMatrix = Math::mat3(Math::transpose(Math::inverse(Math::mat3(global))));
        u32 nodeVertexStart = 0;

        while (stack.remaining(b) > 0)
        {
            const std::string tag = readTag(b);
            const s32 length = b.readS32();
            stack.push(b, static_cast<u64>(length));

            if (tag == "MESH")
            {
                nodeVertexStart = static_cast<u32>(verts.size());
                b.readS32();
                while (stack.remaining(b) > 0)
                {
                    const std::string meshTag = readTag(b);
                    const s32 meshLength = b.readS32();
                    stack.push(b, static_cast<u64>(meshLength));
                    if (meshTag == "VRTS")
                        readVRTS(global, normalMatrix);
                    else if (meshTag == "TRIS")
                        readTRIS(nodeVertexStart);
                    stack.pop(b);
                }
            }
            else if (tag == "BONE")
            {
                sawBones = true;
                while (stack.remaining(b) > 0)
                {
                    const s32 localVertex = b.readS32();
                    const f32 weight = b.readF32();
                    const s32 globalVertex = static_cast<s32>(nodeVertexStart) + localVertex;
                    if (globalVertex >= 0 && globalVertex < static_cast<s32>(weights.size()))
                        weights[static_cast<usize>(globalVertex)].add(myBone, weight);
                }
            }
            else if (tag == "NODE")
            {
                parseNode(global);
            }

            stack.pop(b);
        }
    }
};

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

} // namespace

bool B3DImporter::supports(const char* extension) const
{
    return extension && std::strcmp(extension, "b3d") == 0;
}

bool B3DImporter::import(const std::string& filename, ByteArray& data, FileSystem&, MeshData& mesh)
{
    mesh.clear();

    B3DContext context(data);
    if (!readHeader(data, context.stack))
    {
        Log::error("B3DImporter: '%s' is not a Blitz3D file", filename.c_str());
        return false;
    }

    while (context.stack.remaining(data) > 0)
    {
        const std::string tag = readTag(data);
        const s32 length = data.readS32();
        context.stack.push(data, static_cast<u64>(length));

        if (tag == "TEXS")
        {
            while (context.stack.remaining(data) > 0)
            {
                context.texNames.push_back(readCString(data));
                data.readS32();
                data.readS32();
                data.readF32();
                data.readF32();
                data.readF32();
                data.readF32();
                data.readF32();
            }
        }
        else if (tag == "BRUS")
        {
            const s32 textureCount = data.readS32();
            while (context.stack.remaining(data) > 0)
            {
                Brush brush;
                brush.name = readCString(data);
                brush.diffuse.x = data.readF32();
                brush.diffuse.y = data.readF32();
                brush.diffuse.z = data.readF32();
                data.readF32();
                data.readF32();
                data.readS32();
                data.readS32();
                for (s32 i = 0; i < textureCount; ++i)
                {
                    const s32 texId = data.readS32();
                    if (i == 0)
                        brush.tex0 = texId;
                }
                context.brushes.push_back(brush);
            }
        }
        else if (tag == "NODE")
        {
            context.parseNode(Math::mat4(1.0f));
        }

        context.stack.pop(data);
    }

    if (context.verts.empty() || context.indices.empty())
    {
        Log::error("B3DImporter: '%s' has no geometry", filename.c_str());
        mesh.clear();
        return false;
    }

    // TRIS is decoded into the importer context.  Keep the index buffer in
    // MeshData as well; submesh bounds and the upload path both consume this
    // array.  The old importer left it empty and then indexed it below.
    for (const u32 index : context.indices)
    {
        if (index >= context.verts.size())
        {
            Log::error("B3DImporter: '%s' contains an out-of-range vertex index", filename.c_str());
            mesh.clear();
            return false;
        }
    }
    mesh.indices = context.indices;

    const bool skinned = context.sawBones;

    mesh.positions.reserve(context.verts.size());
    mesh.normals.reserve(context.verts.size());
    mesh.uvs.reserve(context.verts.size());
    if (skinned)
        mesh.skin.reserve(context.verts.size());

    for (const StoredVertex& vertex : context.verts)
    {
        if (skinned)
        {
            mesh.positions.push_back(vertex.pos);
            mesh.normals.push_back(vertex.normal);
            mesh.uvs.push_back(vertex.uv);
        }
        else
        {
            const Math::vec3 world = Math::vec3(vertex.xform * Math::vec4(vertex.pos, 1.0f));
            Math::vec3 normal = vertex.nmat * vertex.normal;
            if (Math::dot(normal, normal) > 1e-8f)
                normal = Math::normalize(normal);
            mesh.positions.push_back(world);
            mesh.normals.push_back(normal);
            mesh.uvs.push_back(vertex.uv);
        }
    }

    if (skinned)
    {
        for (const WeightSlots& slots : context.weights)
        {
            MeshSkinVertex skinVertex;
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
                skinVertex.joints[0] = static_cast<u8>(b0 & 0xFF);
                skinVertex.joints[1] = static_cast<u8>(b1 & 0xFF);
                skinVertex.joints[2] = static_cast<u8>(b2 & 0xFF);
                skinVertex.joints[3] = static_cast<u8>(b3 & 0xFF);
                skinVertex.weights = Math::vec4(slots.weights[0] / sum, slots.weights[1] / sum,
                                               slots.weights[2] / sum, slots.weights[3] / sum);
            }
            mesh.skin.push_back(skinVertex);
        }
    }

    std::vector<OutMaterial> outMaterials;
    std::unordered_map<s32, u32> brushSlots;
    u32 defaultSlot = 0xFFFFFFFFu;

    const auto ensureDefault = [&]() -> u32
    {
        if (defaultSlot == 0xFFFFFFFFu)
        {
            defaultSlot = static_cast<u32>(outMaterials.size());
            outMaterials.emplace_back();
        }
        return defaultSlot;
    };

    const auto slotForBrush = [&](s32 brushId) -> u32
    {
        if (brushId < 0 || brushId >= static_cast<s32>(context.brushes.size()))
            return ensureDefault();
        auto found = brushSlots.find(brushId);
        if (found != brushSlots.end())
            return found->second;
        const u32 slot = static_cast<u32>(outMaterials.size());
        const Brush& brush = context.brushes[static_cast<usize>(brushId)];
        OutMaterial out;
        out.name = brush.name;
        out.diffuse = brush.diffuse;
        if (brush.tex0 >= 0 && brush.tex0 < static_cast<s32>(context.texNames.size()))
            out.texFile = context.texNames[static_cast<usize>(brush.tex0)];
        outMaterials.push_back(out);
        brushSlots[brushId] = slot;
        return slot;
    };

    for (const Surface& surface : context.surfaces)
    {
        SubMesh submesh;
        submesh.indexOffset = surface.firstIndex;
        submesh.indexCount = surface.indexCount;
        submesh.materialSlot = slotForBrush(surface.brushId);
        mesh.submeshes.push_back(submesh);
    }

    const std::string directory = directoryOf(filename);
    mesh.materials.resize(outMaterials.size());
    mesh.materialTextureFiles.resize(outMaterials.size());
    mesh.materialNormalFiles.resize(outMaterials.size());
    for (usize i = 0; i < outMaterials.size(); ++i)
    {
        mesh.materials[i].name = outMaterials[i].name;
        mesh.materials[i].nameHash = hashName(outMaterials[i].name);
        mesh.materials[i].params.baseColor = Math::vec4(outMaterials[i].diffuse, 1.0f);
        mesh.materialTextureFiles[i] = joinPath(directory, outMaterials[i].texFile);
    }

    // Same gap FbxImporter/RadionMeshImporter/OgreMeshImporter each already
    // close their own way - MaterialManager::pipelineFor() picks the vertex
    // shader variant (MATERIAL_SKINNED) off this flag, and a skinned mesh
    // whose material lacks it gets the plain-mesh variant instead: no
    // lighting response worth seeing (wrong vertex layout read as the static
    // one), even though the raw geometry (bounds, the editor's outline)
    // looks completely normal.
    if (!mesh.skin.empty())
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

    return true;
}



struct RawKey
{
    f32 time = 0.0f;
    Math::vec3 value = Math::vec3(0.0f);
};

struct RawRotKey
{
    f32 time = 0.0f;
    Math::quat value = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
};

struct JointData
{
    std::string name;
    s32 parent = -1;
    Math::mat4 bindLocal = Math::mat4(1.0f);
    Math::mat4 inverseBind = Math::mat4(1.0f);
    Math::vec3 bindPos = Math::vec3(0.0f);
    Math::quat bindRot = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    Math::vec3 bindScale = Math::vec3(1.0f);
    std::vector<RawKey> posKeys;
    std::vector<RawKey> scaleKeys;
    std::vector<RawRotKey> rotKeys;
};

struct B3DAnimData
{
    bool ok = false;
    std::vector<JointData> joints;
    int animFrames = 0;
    f32 animFps = 25.0f;
};

struct ParseContext
{
    ByteArray& b;
    ChunkStack stack;
    B3DAnimData& out;

    void parseNode(const Math::mat4& parentGlobal, s32 parentBone)
    {
        JointData joint;
        joint.name = readCString(b);
        Math::vec3 pos;
        Math::vec3 scale(1.0f);
        pos.x = b.readF32();
        pos.y = b.readF32();
        pos.z = b.readF32();
        scale.x = b.readF32();
        scale.y = b.readF32();
        scale.z = b.readF32();
        const Math::quat rotation = readQuat(b);
        const s32 myBone = static_cast<s32>(out.joints.size());

        const Math::mat4 local = Math::translate(Math::mat4(1.0f), pos) * Math::mat4_cast(rotation) *
                                Math::scale(Math::mat4(1.0f), scale);
        const Math::mat4 global = parentGlobal * local;

        joint.parent = parentBone;
        joint.bindLocal = local;
        joint.inverseBind = Math::inverse(global);
        joint.bindPos = pos;
        joint.bindRot = rotation;
        joint.bindScale = scale;
        out.joints.push_back(joint);

        while (stack.remaining(b) > 0)
        {
            const std::string tag = readTag(b);
            const s32 length = b.readS32();
            stack.push(b, static_cast<u64>(length));

            if (tag == "ANIM")
            {
                b.readS32();
                const s32 frames = b.readS32();
                f32 fps = b.readF32();
                if (fps <= 0.0f)
                    fps = 25.0f;
                out.animFrames = frames;
                out.animFps = fps;
            }
            else if (tag == "KEYS")
            {
                const s32 flags = b.readS32();
                JointData& target = out.joints[static_cast<usize>(myBone)];
                while (stack.remaining(b) > 0)
                {
                    const s32 frame = b.readS32();
                    const f32 t = static_cast<f32>(frame);
                    if (flags & 1)
                    {
                        RawKey key;
                        key.time = t;
                        key.value.x = b.readF32();
                        key.value.y = b.readF32();
                        key.value.z = b.readF32();
                        target.posKeys.push_back(key);
                    }
                    if (flags & 2)
                    {
                        RawKey key;
                        key.time = t;
                        key.value.x = b.readF32();
                        key.value.y = b.readF32();
                        key.value.z = b.readF32();
                        target.scaleKeys.push_back(key);
                    }
                    if (flags & 4)
                    {
                        RawRotKey key;
                        key.time = t;
                        key.value = readQuat(b);
                        target.rotKeys.push_back(key);
                    }
                }
            }
            else if (tag == "NODE")
            {
                parseNode(global, myBone);
            }

            stack.pop(b);
        }
    }
};

B3DAnimData parseB3D(FileSystem& files, const std::string& filename)
{
    B3DAnimData result;
    ByteArray data = files.readBinary(filename);
    if (data.empty())
        return result;

    ParseContext context{data, {}, result};
    if (!readHeader(data, context.stack))
        return result;

    while (context.stack.remaining(data) > 0)
    {
        const std::string tag = readTag(data);
        const s32 length = data.readS32();
        context.stack.push(data, static_cast<u64>(length));
        if (tag == "NODE")
            context.parseNode(Math::mat4(1.0f), -1);
        context.stack.pop(data);
    }

    result.ok = !result.joints.empty();
    return result;
}

Math::vec3 sampleVec(const std::vector<RawKey>& keys, f32 t, const Math::vec3& fallback)
{
    if (keys.empty())
        return fallback;
    if (keys.size() == 1 || t <= keys.front().time)
        return keys.front().value;
    if (t >= keys.back().time)
        return keys.back().value;
    usize k1 = static_cast<usize>(std::upper_bound(keys.begin(), keys.end(), t,
                                                   [](f32 v, const RawKey& k)
                                                   {
                                                       return v < k.time;
                                                   }) -
                                  keys.begin());
    usize k0 = k1 - 1;
    const f32 span = keys[k1].time - keys[k0].time;
    const f32 f = span > 1e-6f ? (t - keys[k0].time) / span : 0.0f;
    return keys[k0].value + (keys[k1].value - keys[k0].value) * f;
}

Math::quat sampleQuat(const std::vector<RawRotKey>& keys, f32 t, const Math::quat& fallback)
{
    if (keys.empty())
        return fallback;
    if (keys.size() == 1 || t <= keys.front().time)
        return keys.front().value;
    if (t >= keys.back().time)
        return keys.back().value;
    usize k1 = static_cast<usize>(std::upper_bound(keys.begin(), keys.end(), t,
                                                   [](f32 v, const RawRotKey& k)
                                                   {
                                                       return v < k.time;
                                                   }) -
                                  keys.begin());
    usize k0 = k1 - 1;
    const f32 span = keys[k1].time - keys[k0].time;
    const f32 f = span > 1e-6f ? (t - keys[k0].time) / span : 0.0f;
    return Math::slerp(keys[k0].value, keys[k1].value, f);
}

void buildTrack(s32 boneIndex, const JointData& joint, f32 fps, BoneTrack& track)
{
    track.bone = boneIndex;
    if (joint.posKeys.empty() && joint.rotKeys.empty() && joint.scaleKeys.empty())
        return;

    std::vector<f32> times;
    times.reserve(joint.posKeys.size() + joint.scaleKeys.size() + joint.rotKeys.size());
    for (const RawKey& k : joint.posKeys)
        times.push_back(k.time);
    for (const RawKey& k : joint.scaleKeys)
        times.push_back(k.time);
    for (const RawRotKey& k : joint.rotKeys)
        times.push_back(k.time);
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end(),
                            [](f32 a, f32 b)
                            {
                                return std::fabs(a - b) < 1e-4f;
                            }),
                times.end());
    if (times.empty())
        return;

    const f32 ticksPerSecond = fps > 0.0f ? fps : 25.0f;
    track.times.reserve(times.size());
    track.positions.reserve(times.size());
    track.rotations.reserve(times.size());
    track.scales.reserve(times.size());
    for (f32 t : times)
    {
        track.times.push_back(t / ticksPerSecond);
        track.positions.push_back(sampleVec(joint.posKeys, t, joint.bindPos));
        track.rotations.push_back(sampleQuat(joint.rotKeys, t, joint.bindRot));
        track.scales.push_back(sampleVec(joint.scaleKeys, t, joint.bindScale));
    }
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



bool loadB3DSkeleton(const std::string& filename, FileSystem& files, Skeleton& skeleton)
{
    B3DAnimData data = parseB3D(files, filename);
    if (!data.ok)
    {
        Log::error("B3DAnimationLoader: '%s' has no usable skeleton", filename.c_str());
        return false;
    }
    skeleton = Skeleton();
    for (const JointData& joint : data.joints)
    {
        if (!skeleton.addBone(joint.name, joint.parent, joint.bindLocal, joint.inverseBind))
            return false;
    }
    return skeleton.finalize();
}

bool loadB3DAnimation(const std::string& filename, FileSystem& files, const Skeleton& skeleton,
                      AnimationClip& clip)
{
    if (skeleton.empty())
        return false;

    B3DAnimData data = parseB3D(files, filename);
    if (!data.ok || data.animFrames <= 1)
    {
        Log::error("B3DAnimationLoader: '%s' has no animation data", filename.c_str());
        return false;
    }

    clip = AnimationClip();
    clip.setName(stem(filename));
    const f32 ticksPerSecond = data.animFps > 0.0f ? data.animFps : 25.0f;
    clip.setDuration(static_cast<f32>(data.animFrames - 1) / ticksPerSecond);

    for (usize i = 0; i < data.joints.size(); ++i)
    {
        const JointData& joint = data.joints[i];
        if (joint.posKeys.empty() && joint.rotKeys.empty() && joint.scaleKeys.empty())
            continue;
        BoneTrack track;
        buildTrack(static_cast<s32>(i), joint, ticksPerSecond, track);
        if (track.times.empty())
            continue;
        clip.tracks().push_back(std::move(track));
    }
    return !clip.tracks().empty();
}



} // namespace Radion
