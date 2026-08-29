#include "PCH.h"

#include "MS3DImporter.h"

#include "ByteArray.h"
#include "Skeleton.h"
#include "FileSystem.h"

namespace Radion
{

namespace
{

struct MS3DVertex
{
    glm::vec3 pos = glm::vec3(0.0f);
    s8 boneId = -1;
};

struct MS3DTriangle
{
    u16 idx[3] = {0, 0, 0};
    glm::vec3 normal[3] = {glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f)};
    glm::vec2 uv[3] = {glm::vec2(0.0f), glm::vec2(0.0f), glm::vec2(0.0f)};
};

struct MS3DGroup
{
    std::string name;
    std::vector<u16> triIndices;
    s8 materialIndex = -1;
};

struct MS3DMaterial
{
    std::string name;
    glm::vec3 diffuse = glm::vec3(1.0f);
    glm::vec3 specular = glm::vec3(0.0f);
    f32 shininess = 32.0f;
    std::string textureFile;
};

struct MS3DKeyframe
{
    f32 time = 0.0f;
    glm::vec3 param = glm::vec3(0.0f);
};

struct MS3DJoint
{
    std::string name;
    std::string parentName;
    glm::vec3 rotation = glm::vec3(0.0f);
    glm::vec3 translation = glm::vec3(0.0f);
    std::vector<MS3DKeyframe> rotKeys;
    std::vector<MS3DKeyframe> transKeys;
};

struct MS3DFile
{
    bool ok = false;
    std::vector<MS3DVertex> verts;
    std::vector<MS3DTriangle> tris;
    std::vector<MS3DGroup> groups;
    std::vector<MS3DMaterial> materials;
    std::vector<MS3DJoint> joints;
};

std::string readFixedString(ByteArray& in, u32 length)
{
    if (!in.canRead(length))
        return {};
    std::string s = in.readString(length);
    const usize nul = s.find('\0');
    if (nul != std::string::npos)
        s.resize(nul);
    return s;
}

MS3DFile parseMS3D(ByteArray& data)
{
    MS3DFile file;

    if (!data.canRead(14))
        return file;
    char magic[10] = {};
    data.readBytes(magic, 10);
    if (std::memcmp(magic, "MS3D000000", 10) != 0)
        return file;
    const s32 version = data.readS32();
    if (version < 3 || version > 4)
        return file;

    if (!data.canRead(2))
        return file;
    const u16 vertexCount = data.readU16();
    file.verts.resize(vertexCount);
    for (MS3DVertex& v : file.verts)
    {
        if (!data.canRead(1 + 12 + 1 + 1))
            return file;
        data.readU8();
        v.pos.x = data.readF32();
        v.pos.y = data.readF32();
        v.pos.z = data.readF32();
        v.boneId = data.readS8();
        data.readU8();
    }

    if (!data.canRead(2))
        return file;
    const u16 triangleCount = data.readU16();
    file.tris.resize(triangleCount);
    for (MS3DTriangle& t : file.tris)
    {
        if (!data.canRead(2 + 6 + 36 + 1 + 1))
            return file;
        data.readU16();
        for (int i = 0; i < 3; ++i)
            t.idx[i] = data.readU16();
        for (int i = 0; i < 3; ++i)
        {
            t.normal[i].x = data.readF32();
            t.normal[i].y = data.readF32();
            t.normal[i].z = data.readF32();
        }
        // MS3D stores the three s (u) values contiguously, then the three t
        // (v) values contiguously - not interleaved per corner.
        f32 s[3];
        f32 v[3];
        for (int i = 0; i < 3; ++i)
            s[i] = data.readF32();
        for (int i = 0; i < 3; ++i)
            v[i] = data.readF32();
        for (int i = 0; i < 3; ++i)
            t.uv[i] = glm::vec2(s[i], 1.0f - v[i]);
        data.readU8();
        data.readU8();
    }

    if (!data.canRead(2))
        return file;
    const u16 groupCount = data.readU16();
    file.groups.resize(groupCount);
    for (MS3DGroup& g : file.groups)
    {
        if (!data.canRead(1 + 32 + 2))
            return file;
        data.readU8();
        g.name = readFixedString(data, 32);
        const u16 groupTriCount = data.readU16();
        g.triIndices.resize(groupTriCount);
        for (u16& ti : g.triIndices)
        {
            if (!data.canRead(2))
                return file;
            ti = data.readU16();
        }
        if (!data.canRead(1))
            return file;
        g.materialIndex = data.readS8();
    }

    if (!data.canRead(2))
        return file;
    const u16 materialCount = data.readU16();
    file.materials.resize(materialCount);
    for (MS3DMaterial& m : file.materials)
    {
        if (!data.canRead(32 + 64 + 1 + 128 + 128))
            return file;
        m.name = readFixedString(data, 32);
        data.readF32();
        data.readF32();
        data.readF32();
        data.readF32();
        m.diffuse.x = data.readF32();
        m.diffuse.y = data.readF32();
        m.diffuse.z = data.readF32();
        data.readF32();
        m.specular.x = data.readF32();
        m.specular.y = data.readF32();
        m.specular.z = data.readF32();
        data.readF32();
        data.readF32();
        data.readF32();
        data.readF32();
        data.readF32();
        m.shininess = data.readF32();
        data.readF32();
        data.readU8();
        m.textureFile = readFixedString(data, 128);
        readFixedString(data, 128);
    }

    data.readF32();
    data.readF32();
    if (data.canRead(4))
        data.readS32();

    // The joints/keyframes chunk is optional - a purely static export can
    // legitimately end right here. Missing it is not a parse failure, just
    // an empty file.joints.
    if (data.canRead(2))
    {
        const u16 jointCount = data.readU16();
        file.joints.resize(jointCount);
        for (MS3DJoint& j : file.joints)
        {
            if (!data.canRead(1 + 32 + 32 + 24 + 4))
                return file;
            data.readU8();
            j.name = readFixedString(data, 32);
            j.parentName = readFixedString(data, 32);
            j.rotation.x = data.readF32();
            j.rotation.y = data.readF32();
            j.rotation.z = data.readF32();
            j.translation.x = data.readF32();
            j.translation.y = data.readF32();
            j.translation.z = data.readF32();
            const u16 rotKeyCount = data.readU16();
            const u16 transKeyCount = data.readU16();
            j.rotKeys.resize(rotKeyCount);
            for (MS3DKeyframe& k : j.rotKeys)
            {
                if (!data.canRead(16))
                    return file;
                k.time = data.readF32();
                k.param.x = data.readF32();
                k.param.y = data.readF32();
                k.param.z = data.readF32();
            }
            j.transKeys.resize(transKeyCount);
            for (MS3DKeyframe& k : j.transKeys)
            {
                if (!data.canRead(16))
                    return file;
                k.time = data.readF32();
                k.param.x = data.readF32();
                k.param.y = data.readF32();
                k.param.z = data.readF32();
            }
        }
    }

    file.ok = true;
    return file;
}

std::string directoryOf(const std::string& filename)
{
    const usize slash = filename.find_last_of("/\\");
    return slash == std::string::npos ? std::string() : filename.substr(0, slash + 1);
}

// MS3D embeds a Windows path for a material's texture, relative or
// absolute - never trusted directly. Only the filename is kept; the real
// mesh directory (directoryOf(filename) above) supplies the rest.
std::string textureBasename(const std::string& reference)
{
    const usize slash = reference.find_last_of("/\\");
    return slash == std::string::npos ? reference : reference.substr(slash + 1);
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

f32 shininessToRoughness(f32 shininess)
{
    if (shininess <= 0.0f)
        return 1.0f;
    const f32 rough = std::sqrt(2.0f / (shininess + 2.0f));
    return glm::clamp(rough, 0.05f, 1.0f);
}

} // namespace

bool MS3DImporter::supports(const char* extension) const
{
    return extension && std::strcmp(extension, "ms3d") == 0;
}

bool MS3DImporter::import(const std::string& filename, ByteArray& data, FileSystem&, MeshData& mesh)
{
    mesh.clear();

    MS3DFile file = parseMS3D(data);
    if (!file.ok || file.tris.empty() || file.groups.empty())
    {
        Log::error("MS3DImporter: '%s' is not a valid MilkShape3D file", filename.c_str());
        return false;
    }

    for (const MS3DGroup& group : file.groups)
    {
        SubMesh submesh;
        submesh.indexOffset = static_cast<u32>(mesh.indices.size());

        for (u16 triIndex : group.triIndices)
        {
            if (triIndex >= file.tris.size())
                continue;
            const MS3DTriangle& tri = file.tris[triIndex];
            for (int corner = 0; corner < 3; ++corner)
            {
                const u16 vertexIndex = tri.idx[corner];
                const MS3DVertex& source =
                    vertexIndex < file.verts.size() ? file.verts[vertexIndex] : MS3DVertex();
                mesh.positions.push_back(source.pos);
                mesh.normals.push_back(tri.normal[corner]);
                mesh.uvs.push_back(tri.uv[corner]);
                if (!file.joints.empty())
                {
                    MeshSkinVertex skin;
                    const u8 bone = source.boneId >= 0 ? static_cast<u8>(source.boneId) : 0u;
                    skin.joints[0] = bone;
                    skin.joints[1] = bone;
                    skin.joints[2] = bone;
                    skin.joints[3] = bone;
                    skin.weights = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
                    mesh.skin.push_back(skin);
                }
                mesh.indices.push_back(static_cast<u32>(mesh.positions.size()) - 1);
            }
        }

        submesh.indexCount = static_cast<u32>(mesh.indices.size()) - submesh.indexOffset;
        if (submesh.indexCount == 0)
            continue;
        const int materialCount = static_cast<int>(file.materials.size());
        submesh.materialSlot = (group.materialIndex >= 0 && group.materialIndex < materialCount)
                                   ? static_cast<u32>(group.materialIndex)
                                   : 0u;
        mesh.submeshes.push_back(submesh);
    }

    if (mesh.positions.empty() || mesh.indices.empty())
    {
        Log::error("MS3DImporter: '%s' produced no triangles", filename.c_str());
        mesh.clear();
        return false;
    }

    const std::string directory = directoryOf(filename);
    if (file.materials.empty())
    {
        mesh.materials.resize(1);
        mesh.materialTextureFiles.resize(1);
        mesh.materialNormalFiles.resize(1);
        mesh.materials[0].name = "default";
        mesh.materials[0].nameHash = hashName("default");
    }
    else
    {
        const usize materialCount = file.materials.size();
        mesh.materials.resize(materialCount);
        mesh.materialTextureFiles.resize(materialCount);
        mesh.materialNormalFiles.resize(materialCount);
        for (usize i = 0; i < materialCount; ++i)
        {
            const MS3DMaterial& material = file.materials[i];
            mesh.materials[i].name = material.name;
            mesh.materials[i].nameHash = hashName(material.name);
            mesh.materials[i].params.baseColor = glm::vec4(material.diffuse, 1.0f);
            mesh.materials[i].params.surface.x = shininessToRoughness(material.shininess);
            mesh.materialTextureFiles[i] = joinPath(directory, textureBasename(material.textureFile));
        }
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

    return true;
}

namespace
{

struct Key
{
    f32 time = 0.0f;
    glm::vec3 param = glm::vec3(0.0f);
};

struct JointData
{
    std::string name;
    std::string parentName;
    glm::vec3 rotation = glm::vec3(0.0f);
    glm::vec3 translation = glm::vec3(0.0f);
    std::vector<Key> rotKeys;
    std::vector<Key> transKeys;
};

std::string readFixed(ByteArray& in, u32 length)
{
    if (!in.canRead(length))
        return {};
    std::string s = in.readString(length);
    const usize nul = s.find('\0');
    if (nul != std::string::npos)
        s.resize(nul);
    return s;
}

bool skipFixed(ByteArray& in, u32 length)
{
    if (!in.canRead(length))
        return false;
    in.seek(static_cast<long long>(in.tell()) + length);
    return true;
}

bool parseJoints(ByteArray& data, std::vector<JointData>& joints)
{
    if (!data.canRead(14))
        return false;
    char magic[10] = {};
    data.readBytes(magic, 10);
    if (std::memcmp(magic, "MS3D000000", 10) != 0)
        return false;
    const s32 version = data.readS32();
    if (version < 3 || version > 4)
        return false;

    if (!data.canRead(2))
        return false;
    u16 count = data.readU16();
    if (!skipFixed(data, static_cast<u32>(count) * 15))
        return false;

    if (!data.canRead(2))
        return false;
    count = data.readU16();
    // flags(2) + idx[3](6) + normal[3][3](36) + s[3](12) + t[3](12) +
    // smoothingGroup(1) + groupIndex(1) = 70 bytes per triangle.
    if (!skipFixed(data, static_cast<u32>(count) * 70))
        return false;

    if (!data.canRead(2))
        return false;
    count = data.readU16();
    for (u16 i = 0; i < count; ++i)
    {
        if (!skipFixed(data, 1 + 32))
            return false;
        if (!data.canRead(2))
            return false;
        const u16 triCount = data.readU16();
        if (!skipFixed(data, static_cast<u32>(triCount) * 2 + 1))
            return false;
    }

    if (!data.canRead(2))
        return false;
    count = data.readU16();
    if (!skipFixed(data, static_cast<u32>(count) * 361))
        return false;

    if (!skipFixed(data, 4 + 4 + 4))
        return false;

    if (!data.canRead(2))
        return false;
    count = data.readU16();
    joints.clear();
    joints.reserve(count);
    for (u16 i = 0; i < count; ++i)
    {
        if (!data.canRead(1 + 32 + 32 + 24 + 4))
            return false;
        data.readU8();
        JointData joint;
        joint.name = readFixed(data, 32);
        joint.parentName = readFixed(data, 32);
        joint.rotation.x = data.readF32();
        joint.rotation.y = data.readF32();
        joint.rotation.z = data.readF32();
        joint.translation.x = data.readF32();
        joint.translation.y = data.readF32();
        joint.translation.z = data.readF32();
        const u16 rotKeyCount = data.readU16();
        const u16 transKeyCount = data.readU16();
        joint.rotKeys.resize(rotKeyCount);
        for (Key& k : joint.rotKeys)
        {
            if (!data.canRead(16))
                return false;
            k.time = data.readF32();
            k.param.x = data.readF32();
            k.param.y = data.readF32();
            k.param.z = data.readF32();
        }
        joint.transKeys.resize(transKeyCount);
        for (Key& k : joint.transKeys)
        {
            if (!data.canRead(16))
                return false;
            k.time = data.readF32();
            k.param.x = data.readF32();
            k.param.y = data.readF32();
            k.param.z = data.readF32();
        }
        joints.push_back(std::move(joint));
    }
    return !joints.empty();
}

glm::quat eulerToQuat(const glm::vec3& e)
{
    const glm::quat qx = glm::angleAxis(e.x, glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::quat qy = glm::angleAxis(e.y, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::quat qz = glm::angleAxis(e.z, glm::vec3(0.0f, 0.0f, 1.0f));
    return qz * qy * qx;
}

bool buildSkeleton(const std::vector<JointData>& joints, Skeleton& skeleton)
{
    std::vector<s32> parent(joints.size(), -1);
    for (usize i = 0; i < joints.size(); ++i)
    {
        if (joints[i].parentName.empty())
            continue;
        for (usize j = 0; j < joints.size(); ++j)
        {
            if (joints[j].name == joints[i].parentName)
            {
                parent[i] = static_cast<s32>(j);
                break;
            }
        }
    }

    std::vector<glm::mat4> world(joints.size());
    for (usize i = 0; i < joints.size(); ++i)
    {
        const glm::mat4 local = glm::translate(glm::mat4(1.0f), joints[i].translation) *
                                glm::mat4_cast(eulerToQuat(joints[i].rotation));
        world[i] = parent[i] >= 0 ? world[static_cast<usize>(parent[i])] * local : local;
        skeleton.addBone(joints[i].name, parent[i], local, glm::inverse(world[i]));
    }
    return skeleton.finalize();
}

LocalPose decompose(const glm::mat4& m)
{
    LocalPose pose;
    pose.position = glm::vec3(m[3]);
    const glm::vec3 sx(m[0].x, m[0].y, m[0].z);
    const glm::vec3 sy(m[1].x, m[1].y, m[1].z);
    const glm::vec3 sz(m[2].x, m[2].y, m[2].z);
    pose.scale = glm::vec3(glm::length(sx), glm::length(sy), glm::length(sz));
    glm::mat3 rot;
    rot[0] = pose.scale.x > 1e-8f ? sx / pose.scale.x : sx;
    rot[1] = pose.scale.y > 1e-8f ? sy / pose.scale.y : sy;
    rot[2] = pose.scale.z > 1e-8f ? sz / pose.scale.z : sz;
    pose.rotation = glm::quat_cast(rot);
    return pose;
}

glm::quat sampleRotation(const std::vector<Key>& keys, f32 t)
{
    if (keys.size() == 1 || t <= keys.front().time)
        return eulerToQuat(keys.front().param);
    if (t >= keys.back().time)
        return eulerToQuat(keys.back().param);
    usize k1 = static_cast<usize>(std::upper_bound(keys.begin(), keys.end(), t,
                                                   [](f32 v, const Key& k)
                                                   {
                                                       return v < k.time;
                                                   }) -
                                  keys.begin());
    usize k0 = k1 - 1;
    const f32 span = keys[k1].time - keys[k0].time;
    const f32 f = span > 1e-6f ? (t - keys[k0].time) / span : 0.0f;
    return glm::slerp(eulerToQuat(keys[k0].param), eulerToQuat(keys[k1].param), f);
}

glm::vec3 sampleTranslation(const std::vector<Key>& keys, f32 t)
{
    if (keys.size() == 1 || t <= keys.front().time)
        return keys.front().param;
    if (t >= keys.back().time)
        return keys.back().param;
    usize k1 = static_cast<usize>(std::upper_bound(keys.begin(), keys.end(), t,
                                                   [](f32 v, const Key& k)
                                                   {
                                                       return v < k.time;
                                                   }) -
                                  keys.begin());
    usize k0 = k1 - 1;
    const f32 span = keys[k1].time - keys[k0].time;
    const f32 f = span > 1e-6f ? (t - keys[k0].time) / span : 0.0f;
    return keys[k0].param + (keys[k1].param - keys[k0].param) * f;
}

void buildTrack(s32 boneIndex, const JointData& joint, const glm::mat4& bindLocal, BoneTrack& track)
{
    track.bone = boneIndex;
    if (joint.rotKeys.empty() && joint.transKeys.empty())
        return;

    std::vector<f32> times;
    times.reserve(joint.rotKeys.size() + joint.transKeys.size());
    for (const Key& k : joint.rotKeys)
        times.push_back(k.time);
    for (const Key& k : joint.transKeys)
        times.push_back(k.time);
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end(),
                            [](f32 a, f32 b)
                            {
                                return std::fabs(a - b) < 1e-5f;
                            }),
                times.end());

    track.times.reserve(times.size());
    track.positions.reserve(times.size());
    track.rotations.reserve(times.size());
    track.scales.reserve(times.size());
    for (f32 t : times)
    {
        const glm::quat deltaRot = joint.rotKeys.empty() ? glm::quat(1.0f, 0.0f, 0.0f, 0.0f)
                                                         : sampleRotation(joint.rotKeys, t);
        const glm::vec3 deltaTrans =
            joint.transKeys.empty() ? glm::vec3(0.0f) : sampleTranslation(joint.transKeys, t);
        const glm::mat4 delta =
            glm::translate(glm::mat4(1.0f), deltaTrans) * glm::mat4_cast(deltaRot);
        const LocalPose pose = decompose(bindLocal * delta);
        track.times.push_back(t);
        track.positions.push_back(pose.position);
        track.rotations.push_back(pose.rotation);
        track.scales.push_back(pose.scale);
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

} // namespace

bool loadMS3DSkeleton(const std::string& filename, FileSystem& files, Skeleton& skeleton)
{
    ByteArray data = files.readBinary(filename);
    if (data.empty())
    {
        Log::error("MS3DAnimationLoader: cannot read '%s'", filename.c_str());
        return false;
    }
    std::vector<JointData> joints;
    if (!parseJoints(data, joints))
    {
        Log::error("MS3DAnimationLoader: '%s' has no usable skeleton", filename.c_str());
        return false;
    }
    if (!buildSkeleton(joints, skeleton))
    {
        Log::error("MS3DAnimationLoader: failed to build skeleton from '%s'", filename.c_str());
        return false;
    }
    return true;
}

bool loadMS3DAnimation(const std::string& filename, FileSystem& files, const Skeleton& skeleton,
                       AnimationClip& clip)
{
    if (skeleton.empty())
        return false;
    ByteArray data = files.readBinary(filename);
    if (data.empty())
    {
        Log::error("MS3DAnimationLoader: cannot read '%s'", filename.c_str());
        return false;
    }
    std::vector<JointData> joints;
    if (!parseJoints(data, joints))
    {
        Log::error("MS3DAnimationLoader: '%s' has no animation data", filename.c_str());
        return false;
    }

    clip.setName(stem(filename));
    f32 duration = 0.0f;
    for (const JointData& joint : joints)
    {
        const s32 boneIndex = skeleton.findBone(joint.name.c_str());
        if (boneIndex < 0)
            continue;
        BoneTrack track;
        buildTrack(boneIndex, joint, skeleton.bone(static_cast<u32>(boneIndex)).bindLocal, track);
        if (track.times.empty())
            continue;
        duration = std::max(duration, track.times.back());
        clip.tracks().push_back(std::move(track));
    }
    clip.setDuration(duration);
    return !clip.tracks().empty();
}



} // namespace Radion
