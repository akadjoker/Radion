#include "PCH.h"

#include "C3DSImporter.h"

#include "ByteArray.h"
#include "FileSystem.h"

namespace Radion
{

namespace
{

enum Chunk3DS : u16
{
    ChunkMain3DS = 0x4D4D,

    ChunkEdit3DS = 0x3D3D,
    ChunkKeyf3DS = 0xB000,
    ChunkVersion = 0x0002,
    ChunkMeshVersion = 0x3D3E,

    ChunkEditMaterial = 0xAFFF,
    ChunkEditObject = 0x4000,

    ChunkMatName = 0xA000,
    ChunkMatDiffuse = 0xA020,
    ChunkMatTexMap = 0xA200,
    ChunkMatMapFile = 0xA300,

    ChunkObjTriMesh = 0x4100,
    ChunkTriVert = 0x4110,
    ChunkPointFlagArray = 0x4111,
    ChunkTriFace = 0x4120,
    ChunkTriFaceMat = 0x4130,
    ChunkTriUv = 0x4140,
    ChunkTriMatrix = 0x4160,
    ChunkMeshColor = 0x4165,
    ChunkTriSmooth = 0x4150,

    ChunkColRgb = 0x0010,
    ChunkColTru = 0x0011,
    ChunkColLin24 = 0x0012,
    ChunkColLinF = 0x0013,
};

struct Face3DS
{
    u16 a, b, c;
};

struct FaceGroup3DS
{
    std::string matName;
    std::vector<u16> faces;
};

struct Object3DS
{
    std::string name;
    std::vector<Math::Vec3> verts;
    std::vector<Math::Vec2> uvs;
    std::vector<Face3DS> faces;
    std::vector<FaceGroup3DS> groups;
};

struct Material3DS
{
    std::string name;
    Math::Vec3 diffuse = Math::Vec3(1.0f);
    std::string texFile;
};

bool readChunkHeader(ByteArray& in, u16& id, u32& end)
{
    u32 len = 0;
    if (!in.canRead(6))
        return false;
    id = in.readU16();
    len = in.readU32();
    if (len < 6)
        return false;
    const u64 cursor = in.tell();
    if (cursor > static_cast<u64>(in.size()) || len - 6 > static_cast<u64>(in.size()) - cursor)
        return false;
    end = static_cast<u32>(cursor + (len - 6));
    return true;
}

bool readCString(ByteArray& in, std::string& out)
{
    out.clear();
    while (in.canRead(1))
    {
        const u8 c = in.readU8();
        if (c == 0)
            return true;
        out.push_back(static_cast<char>(c));
    }
    return false;
}

void readColorChunk(ByteArray& in, u32 end, Math::Vec3& outColor)
{
    while (in.tell() + 6 <= end)
    {
        u16 id = 0;
        u32 subEnd = 0;
        if (!readChunkHeader(in, id, subEnd))
            return;
        if (id == ChunkColRgb || id == ChunkColLinF)
        {
            if (in.canRead(12))
            {
                outColor.x = in.readF32();
                outColor.y = in.readF32();
                outColor.z = in.readF32();
            }
        }
        else if (id == ChunkColTru || id == ChunkColLin24)
        {
            if (in.canRead(3))
            {
                const f32 r = static_cast<f32>(in.readU8()) / 255.0f;
                const f32 g = static_cast<f32>(in.readU8()) / 255.0f;
                const f32 b = static_cast<f32>(in.readU8()) / 255.0f;
                outColor = Math::Vec3(r, g, b);
            }
        }
        in.seek(static_cast<long long>(subEnd));
        return;
    }
}

void readMaterialChunk(ByteArray& in, u32 matEnd, std::vector<Material3DS>& out)
{
    Material3DS mat;
    u16 texSection = 0;

    while (in.tell() + 6 <= matEnd)
    {
        u16 id = 0;
        u32 subEnd = 0;
        if (!readChunkHeader(in, id, subEnd))
            break;

        switch (id)
        {
        case ChunkMatName:
            readCString(in, mat.name);
            in.seek(static_cast<long long>(subEnd));
            break;
        case ChunkMatDiffuse:
            readColorChunk(in, subEnd, mat.diffuse);
            in.seek(static_cast<long long>(subEnd));
            break;
        case ChunkMatTexMap:
            texSection = id;
            break;
        case ChunkMatMapFile:
            if (texSection == ChunkMatTexMap)
                readCString(in, mat.texFile);
            in.seek(static_cast<long long>(subEnd));
            break;
        default:
            in.seek(static_cast<long long>(subEnd));
            break;
        }
    }
    out.push_back(mat);
}

void readTriVert(ByteArray& in, Object3DS& obj)
{
    u16 n = 0;
    if (!in.canRead(2))
        return;
    n = in.readU16();
    obj.verts.resize(n);
    for (u16 i = 0; i < n; ++i)
    {
        if (!in.canRead(12))
            return;
        Math::Vec3 v;
        v.x = in.readF32();
        v.y = in.readF32();
        v.z = in.readF32();
        obj.verts[i] = v;
    }
}

void readTriUv(ByteArray& in, Object3DS& obj)
{
    u16 n = 0;
    if (!in.canRead(2))
        return;
    n = in.readU16();
    obj.uvs.resize(n);
    for (u16 i = 0; i < n; ++i)
    {
        if (!in.canRead(8))
            return;
        const f32 u = in.readF32();
        const f32 v = in.readF32();
        obj.uvs[i] = Math::Vec2(u, 1.0f - v);
    }
}

void readFaceGroup(ByteArray& in, Object3DS& obj)
{
    FaceGroup3DS group;
    if (!readCString(in, group.matName))
        return;
    u16 n = 0;
    if (!in.canRead(2))
        return;
    n = in.readU16();
    group.faces.resize(n);
    for (u16 i = 0; i < n; ++i)
    {
        if (!in.canRead(2))
            return;
        group.faces[i] = in.readU16();
    }
    obj.groups.push_back(std::move(group));
}

void readTriFace(ByteArray& in, u32 end, Object3DS& obj)
{
    u16 n = 0;
    if (!in.canRead(2))
        return;
    n = in.readU16();
    obj.faces.resize(n);
    for (u16 i = 0; i < n; ++i)
    {
        if (!in.canRead(8))
            return;
        u16 a = in.readU16();
        u16 b = in.readU16();
        u16 c = in.readU16();
        in.readU16();
        obj.faces[i] = {a, b, c};
    }

    while (in.tell() + 6 <= end)
    {
        u16 id = 0;
        u32 subEnd = 0;
        if (!readChunkHeader(in, id, subEnd))
            break;
        if (id == ChunkTriFaceMat)
            readFaceGroup(in, obj);
        in.seek(static_cast<long long>(subEnd));
    }
}

void readTriMesh(ByteArray& in, u32 end, Object3DS& obj)
{
    while (in.tell() + 6 <= end)
    {
        u16 id = 0;
        u32 subEnd = 0;
        if (!readChunkHeader(in, id, subEnd))
            break;

        switch (id)
        {
        case ChunkTriVert:
            readTriVert(in, obj);
            break;
        case ChunkTriUv:
            readTriUv(in, obj);
            break;
        case ChunkTriFace:
            readTriFace(in, subEnd, obj);
            break;
        default:
            break;
        }
        in.seek(static_cast<long long>(subEnd));
    }
}

void readEditObject(ByteArray& in, u32 end, std::vector<Object3DS>& objects)
{
    Object3DS obj;
    if (!readCString(in, obj.name))
        return;

    while (in.tell() + 6 <= end)
    {
        u16 id = 0;
        u32 subEnd = 0;
        if (!readChunkHeader(in, id, subEnd))
            break;
        if (id == ChunkObjTriMesh)
            readTriMesh(in, subEnd, obj);
        in.seek(static_cast<long long>(subEnd));
    }

    if (!obj.faces.empty())
        objects.push_back(std::move(obj));
}

void readEdit3DS(ByteArray& in, u32 end, std::vector<Object3DS>& objects,
                 std::vector<Material3DS>& materials)
{
    while (in.tell() + 6 <= end)
    {
        u16 id = 0;
        u32 subEnd = 0;
        if (!readChunkHeader(in, id, subEnd))
            break;

        if (id == ChunkEditMaterial)
            readMaterialChunk(in, subEnd, materials);
        else if (id == ChunkEditObject)
            readEditObject(in, subEnd, objects);
        in.seek(static_cast<long long>(subEnd));
    }
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

struct OutMaterial
{
    std::string name;
    Math::Vec3 diffuse = Math::Vec3(1.0f);
    std::string texFile;
};

} // namespace

bool C3DSImporter::supports(const char* extension) const
{
    return extension && std::strcmp(extension, "3ds") == 0;
}

bool C3DSImporter::import(const std::string& filename, ByteArray& data, FileSystem& files,
                          MeshData& mesh)
{
    (void)files;
    mesh.clear();
    const std::string directory = directoryOf(filename);

    u16 mainId = 0;
    u32 mainEnd = 0;
    if (!readChunkHeader(data, mainId, mainEnd) || mainId != ChunkMain3DS)
    {
        Log::error("C3DSImporter: '%s' is not a 3ds file", filename.c_str());
        return false;
    }

    std::vector<Object3DS> objects;
    std::vector<Material3DS> materials;

    while (data.tell() + 6 <= mainEnd)
    {
        u16 id = 0;
        u32 end = 0;
        if (!readChunkHeader(data, id, end))
            break;
        if (id == ChunkEdit3DS)
            readEdit3DS(data, end, objects, materials);
        data.seek(static_cast<long long>(end));
    }

    if (objects.empty())
    {
        Log::error("C3DSImporter: '%s' has no geometry", filename.c_str());
        return false;
    }

    std::vector<OutMaterial> outMaterials;
    std::unordered_map<std::string, u32> materialSlots;
    u32 defaultSlot = 0xFFFFFFFFu;

    const auto ensureDefaultSlot = [&]() -> u32
    {
        if (defaultSlot == 0xFFFFFFFFu)
        {
            defaultSlot = static_cast<u32>(outMaterials.size());
            outMaterials.push_back(OutMaterial());
        }
        return defaultSlot;
    };

    const auto slotFor = [&](const std::string& matName) -> u32
    {
        if (matName.empty())
            return ensureDefaultSlot();
        auto found = materialSlots.find(matName);
        if (found != materialSlots.end())
            return found->second;
        const u32 slot = static_cast<u32>(outMaterials.size());
        materialSlots[matName] = slot;
        OutMaterial out;
        out.name = matName;
        for (const Material3DS& m : materials)
        {
            if (m.name == matName)
            {
                out.diffuse = m.diffuse;
                out.texFile = m.texFile;
                break;
            }
        }
        outMaterials.push_back(out);
        return slot;
    };

    for (const Object3DS& obj : objects)
    {
        std::vector<const FaceGroup3DS*> groups;
        FaceGroup3DS fallback;
        if (obj.groups.empty())
        {
            fallback.matName.clear();
            fallback.faces.resize(obj.faces.size());
            for (u16 i = 0; i < static_cast<u16>(obj.faces.size()); ++i)
                fallback.faces[i] = i;
            groups.push_back(&fallback);
        }
        else
        {
            for (const FaceGroup3DS& g : obj.groups)
                groups.push_back(&g);
        }

        for (const FaceGroup3DS* g : groups)
        {
            if (g->faces.empty())
                continue;
            const u32 slot = slotFor(g->matName);

            std::unordered_map<u16, u32> remap;
            SubMesh submesh;
            submesh.indexOffset = static_cast<u32>(mesh.indices.size());
            submesh.materialSlot = slot;

            for (u16 faceIdx : g->faces)
            {
                if (faceIdx >= obj.faces.size())
                    continue;
                const Face3DS& face = obj.faces[faceIdx];
                const u16 corners[3] = {face.a, face.b, face.c};
                for (u16 vi : corners)
                {
                    auto found = remap.find(vi);
                    u32 global;
                    if (found != remap.end())
                    {
                        global = found->second;
                    }
                    else
                    {
                        global = static_cast<u32>(mesh.positions.size());
                        remap[vi] = global;
                        mesh.positions.push_back(vi < obj.verts.size() ? obj.verts[vi]
                                                                       : Math::Vec3(0.0f));
                        mesh.normals.push_back(Math::Vec3(0.0f));
                        mesh.uvs.push_back(vi < obj.uvs.size() ? obj.uvs[vi] : Math::Vec2(0.0f));
                    }
                    mesh.indices.push_back(global);
                }
            }
            submesh.indexCount = static_cast<u32>(mesh.indices.size()) - submesh.indexOffset;
            if (submesh.indexCount > 0)
                mesh.submeshes.push_back(submesh);
        }
    }

    if (mesh.positions.empty() || mesh.indices.empty())
    {
        Log::error("C3DSImporter: '%s' produced no triangles", filename.c_str());
        mesh.clear();
        return false;
    }

    mesh.materials.resize(outMaterials.size());
    mesh.materialTextureFiles.resize(outMaterials.size());
    mesh.materialNormalFiles.resize(outMaterials.size());
    for (usize i = 0; i < outMaterials.size(); ++i)
    {
        mesh.materials[i].name = outMaterials[i].name;
        mesh.materials[i].nameHash = hashName(outMaterials[i].name);
        mesh.materials[i].params.baseColor = Math::Vec4(outMaterials[i].diffuse, 1.0f);
        mesh.materialTextureFiles[i] = joinPath(directory, outMaterials[i].texFile);
    }

    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 a = mesh.indices[i];
        const u32 b = mesh.indices[i + 1];
        const u32 c = mesh.indices[i + 2];
        const Math::Vec3 faceNormal = glm::cross(mesh.positions[b] - mesh.positions[a],
                                                mesh.positions[c] - mesh.positions[a]);
        mesh.normals[a] += faceNormal;
        mesh.normals[b] += faceNormal;
        mesh.normals[c] += faceNormal;
    }
    for (Math::Vec3& normal : mesh.normals)
    {
        if (glm::dot(normal, normal) > 0.0f)
            normal = glm::normalize(normal);
        else
            normal = Math::Vec3(0.0f, 1.0f, 0.0f);
    }

    mesh.bounds = AABB();
    for (const Math::Vec3& position : mesh.positions)
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

} // namespace Radion
