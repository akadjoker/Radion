#include "PCH.h"

#include "ObjImporter.h"

#include "ByteArray.h"
#include "FileSystem.h"
#include "Log.h"

#include <cmath>
#include <cstring>

namespace Radion
{

namespace
{

bool isSpace(char c)
{
    return c == ' ' || c == '\t' || c == '\r';
}

const char* skipSpaces(const char* p, const char* end)
{
    while (p < end && isSpace(*p))
        ++p;
    return p;
}

const char* lineEnd(const char* p, const char* end)
{
    while (p < end && *p != '\n' && *p != '\r')
        ++p;
    return p;
}

const char* nextLine(const char* p, const char* end)
{
    while (p < end && *p != '\n')
        ++p;
    return p < end ? p + 1 : p;
}

bool startsWith(const char* p, const char* end, const char* word)
{
    while (*word)
    {
        if (p >= end || *p++ != *word++)
            return false;
    }
    return true;
}

f32 parseFloat(const char*& p, const char* end)
{
    p = skipSpaces(p, end);
    bool negative = false;
    if (p < end && (*p == '-' || *p == '+'))
    {
        negative = *p == '-';
        ++p;
    }

    f64 value = 0.0;
    while (p < end && *p >= '0' && *p <= '9')
        value = value * 10.0 + static_cast<f64>(*p++ - '0');
    if (p < end && *p == '.')
    {
        ++p;
        f64 fraction = 0.1;
        while (p < end && *p >= '0' && *p <= '9')
        {
            value += static_cast<f64>(*p++ - '0') * fraction;
            fraction *= 0.1;
        }
    }
    if (p < end && (*p == 'e' || *p == 'E'))
    {
        ++p;
        bool exponentNegative = false;
        if (p < end && (*p == '-' || *p == '+'))
        {
            exponentNegative = *p == '-';
            ++p;
        }
        int exponent = 0;
        while (p < end && *p >= '0' && *p <= '9')
            exponent = exponent * 10 + (*p++ - '0');
        value *= std::pow(10.0, exponentNegative ? -exponent : exponent);
    }
    return static_cast<f32>(negative ? -value : value);
}

int parseInt(const char*& p, const char* end)
{
    p = skipSpaces(p, end);
    bool negative = false;
    if (p < end && (*p == '-' || *p == '+'))
    {
        negative = *p == '-';
        ++p;
    }
    int value = 0;
    while (p < end && *p >= '0' && *p <= '9')
        value = value * 10 + (*p++ - '0');
    return negative ? -value : value;
}

std::string lineValue(const char* p, const char* end)
{
    p = skipSpaces(p, end);
    const char* last = lineEnd(p, end);
    while (last > p && isSpace(last[-1]))
        --last;
    return std::string(p, static_cast<usize>(last - p));
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

struct RawMaterial
{
    std::string name;
    Math::Vec3 diffuse = Math::Vec3(0.75f);
    std::string albedo;
    std::string normalMap;

    // An OBJ says "this material is a cutout" with map_d: an alpha mask, used
    // for foliage, chains, fences - anything whose silhouette is in the
    // texture rather than in the triangles.
    bool cutout = false;
};

void parseMtl(const std::string& filename, FileSystem& files, std::vector<RawMaterial>& materials)
{
    ByteArray bytes = files.readBinary(filename);
    if (bytes.empty())
    {
        Log::warning("ObjImporter: material file '%s' was not found", filename.c_str());
        return;
    }

    const char* p = reinterpret_cast<const char*>(bytes.data());
    const char* end = p + bytes.size();
    RawMaterial* current = nullptr;
    while (p < end)
    {
        p = skipSpaces(p, end);
        if (startsWith(p, end, "newmtl") && p + 6 < end && isSpace(p[6]))
        {
            materials.push_back(RawMaterial());
            current = &materials.back();
            current->name = lineValue(p + 6, end);
        }
        else if (current && startsWith(p, end, "Kd") && p + 2 < end && isSpace(p[2]))
        {
            p += 2;
            current->diffuse.x = parseFloat(p, end);
            current->diffuse.y = parseFloat(p, end);
            current->diffuse.z = parseFloat(p, end);
        }
        else if (current && startsWith(p, end, "map_Kd") && p + 6 < end && isSpace(p[6]))
            current->albedo = lineValue(p + 6, end);
        // Both spellings are in the wild for the same thing, and "map_bump"
        // has to be tested first or "bump" would match its tail.
        else if (current && startsWith(p, end, "map_bump") && p + 8 < end && isSpace(p[8]))
            current->normalMap = lineValue(p + 8, end);
        else if (current && startsWith(p, end, "bump") && p + 4 < end && isSpace(p[4]))
            current->normalMap = lineValue(p + 4, end);
        else if (current && startsWith(p, end, "map_d") && p + 5 < end && isSpace(p[5]))
        {
            // The mask FILE is not loaded: in practice the diffuse texture of
            // a cutout material already carries the same alpha (both of
            // Sponza's plants do), and there is no material slot for a
            // standalone mask. What matters is the flag - without it the
            // material lands in the Opaque category, goes through the depth
            // prepass, and writes depth for its transparent half. The
            // prepass has no alpha test (depth.frag is empty by design), so
            // everything behind the invisible part of a leaf gets occluded by
            // nothing.
            current->cutout = true;
        }
        p = nextLine(p, end);
    }
}

struct VertexKey
{
    int position;
    int uv;
    int normal;

    bool operator==(const VertexKey& other) const
    {
        return position == other.position && uv == other.uv && normal == other.normal;
    }
};

struct VertexKeyHash
{
    usize operator()(const VertexKey& key) const
    {
        return static_cast<usize>(key.position + 1) * 73856093u ^
               static_cast<usize>(key.uv + 1) * 19349663u ^
               static_cast<usize>(key.normal + 1) * 83492791u;
    }
};

struct Part
{
    u32 material = 0;
    std::vector<u32> indices;
};

int resolveIndex(int index, usize count)
{
    if (index > 0)
        return index - 1;
    if (index < 0)
        return static_cast<int>(count) + index;
    return -1;
}

} // namespace

ObjImporter::ObjImporter(bool mergeSameMaterial) : mMergeSameMaterial(mergeSameMaterial)
{
}

bool ObjImporter::supports(const char* extension) const
{
    return extension && std::strcmp(extension, "obj") == 0;
}

bool ObjImporter::import(const std::string& filename, ByteArray& data, FileSystem& files,
                         MeshData& mesh)
{
    mesh.clear();
    const std::string directory = directoryOf(filename);
    std::vector<Math::Vec3> sourcePositions;
    std::vector<Math::Vec3> sourceNormals;
    std::vector<Math::Vec2> sourceUvs;
    std::vector<RawMaterial> rawMaterials;
    HashMap<std::string, u32> materialIndices;
    HashMap<VertexKey, u32, VertexKeyHash> vertices;
    std::vector<u8> missingNormals;
    std::vector<Part> parts;
    HashMap<u32, usize> mergedParts;
    u32 currentMaterial = 0;
    bool newPart = true;

    RawMaterial fallback;
    fallback.name = "default";
    rawMaterials.push_back(fallback);
    materialIndices[fallback.name] = 0;

    const char* p = reinterpret_cast<const char*>(data.data());
    const char* end = p + data.size();
    while (p < end)
    {
        p = skipSpaces(p, end);
        if (p >= end)
            break;

        if (*p == 'v' && p + 1 < end && isSpace(p[1]))
        {
            ++p;
            const f32 x = parseFloat(p, end);
            const f32 y = parseFloat(p, end);
            const f32 z = parseFloat(p, end);
            sourcePositions.emplace_back(x, y, z);
        }
        else if (startsWith(p, end, "vn") && p + 2 < end && isSpace(p[2]))
        {
            p += 2;
            const f32 x = parseFloat(p, end);
            const f32 y = parseFloat(p, end);
            const f32 z = parseFloat(p, end);
            sourceNormals.emplace_back(x, y, z);
        }
        else if (startsWith(p, end, "vt") && p + 2 < end && isSpace(p[2]))
        {
            p += 2;
            const f32 x = parseFloat(p, end);
            const f32 y = parseFloat(p, end);
            // V flipped, which is the engine's convention and not a fix for
            // this one model: the asset exporter runs assimp with
            // aiProcess_FlipUVs (tools/exporter/src/AssimpLoader.cpp:30), so
            // every .rmesh already carries flipped V. Reading an OBJ's vt
            // verbatim made this importer the only one disagreeing, and every
            // OBJ texture came out upside down.
            //
            // Corrected HERE rather than by telling stb to flip on load: the
            // mismatch is in the model's coordinate convention, and flipping
            // at load would turn over every texture in the engine, including
            // the ones that are already right.
            sourceUvs.emplace_back(x, 1.0f - y);
        }
        else if (startsWith(p, end, "mtllib") && p + 6 < end && isSpace(p[6]))
        {
            const std::string mtl = joinPath(directory, lineValue(p + 6, end));
            const usize first = rawMaterials.size();
            parseMtl(mtl, files, rawMaterials);
            for (usize i = first; i < rawMaterials.size(); ++i)
            {
                rawMaterials[i].albedo = joinPath(directory, rawMaterials[i].albedo);
                materialIndices[rawMaterials[i].name] = static_cast<u32>(i);
            }
        }
        else if (startsWith(p, end, "usemtl") && p + 6 < end && isSpace(p[6]))
        {
            const std::string name = lineValue(p + 6, end);
            auto found = materialIndices.find(name);
            if (found == materialIndices.end())
            {
                RawMaterial material;
                material.name = name;
                currentMaterial = static_cast<u32>(rawMaterials.size());
                rawMaterials.push_back(material);
                materialIndices[name] = currentMaterial;
            }
            else
                currentMaterial = found->second;
            newPart = true;
        }
        else if (*p == 'f' && p + 1 < end && isSpace(p[1]))
        {
            usize partIndex;
            auto merged = mergedParts.find(currentMaterial);
            if (mMergeSameMaterial && merged != mergedParts.end())
                partIndex = merged->second;
            else if (!mMergeSameMaterial && !newPart && !parts.empty())
                partIndex = parts.size() - 1;
            else
            {
                partIndex = parts.size();
                Part part;
                part.material = currentMaterial;
                parts.push_back(part);
                if (mMergeSameMaterial)
                    mergedParts[currentMaterial] = partIndex;
            }
            newPart = false;

            Part& part = parts[partIndex];
            const char* cursor = p + 1;
            const char* faceEnd = lineEnd(cursor, end);
            u32 first = 0;
            u32 previous = 0;
            u32 corners = 0;
            while ((cursor = skipSpaces(cursor, faceEnd)) < faceEnd)
            {
                const int position =
                    resolveIndex(parseInt(cursor, faceEnd), sourcePositions.size());
                int uv = -1;
                int normal = -1;
                if (cursor < faceEnd && *cursor == '/')
                {
                    ++cursor;
                    if (cursor < faceEnd && *cursor != '/')
                        uv = resolveIndex(parseInt(cursor, faceEnd), sourceUvs.size());
                    if (cursor < faceEnd && *cursor == '/')
                    {
                        ++cursor;
                        normal = resolveIndex(parseInt(cursor, faceEnd), sourceNormals.size());
                    }
                }
                if (position < 0 || position >= static_cast<int>(sourcePositions.size()))
                    return false;

                const VertexKey key{position, uv, normal};
                auto found = vertices.find(key);
                u32 vertex;
                if (found != vertices.end())
                    vertex = found->second;
                else
                {
                    vertex = static_cast<u32>(mesh.positions.size());
                    vertices[key] = vertex;
                    mesh.positions.push_back(sourcePositions[position]);
                    mesh.normals.push_back(normal >= 0 &&
                                                   normal < static_cast<int>(sourceNormals.size())
                                               ? sourceNormals[normal]
                                               : Math::Vec3(0.0f));
                    missingNormals.push_back(normal < 0 ? 1 : 0);
                    mesh.uvs.push_back(uv >= 0 && uv < static_cast<int>(sourceUvs.size())
                                           ? sourceUvs[uv]
                                           : Math::Vec2(0.0f));
                }

                if (corners == 0)
                    first = vertex;
                else if (corners >= 2)
                {
                    part.indices.push_back(first);
                    part.indices.push_back(previous);
                    part.indices.push_back(vertex);
                }
                previous = vertex;
                ++corners;
            }
        }
        p = nextLine(p, end);
    }

    mesh.materials.resize(rawMaterials.size());
    mesh.materialTextureFiles.resize(rawMaterials.size());
    mesh.materialNormalFiles.resize(rawMaterials.size());
    for (usize i = 0; i < rawMaterials.size(); ++i)
    {
        mesh.materials[i].name = rawMaterials[i].name;
        mesh.materials[i].nameHash = hashName(rawMaterials[i].name);
        mesh.materials[i].params.baseColor = Math::Vec4(rawMaterials[i].diffuse, 1.0f);
        if (rawMaterials[i].cutout)
        {
            // AlphaTest is a category, not just a shader switch: it is what
            // keeps this material out of the depth prepass. Two-sided as well,
            // because a leaf card is one layer of triangles meant to be seen
            // from both sides - back-face culling would delete half of every
            // plant.
            mesh.materials[i].flags |= MaterialAlphaTest | MaterialTwoSided;
        }
        mesh.materialTextureFiles[i] = rawMaterials[i].albedo;
        mesh.materialNormalFiles[i] = rawMaterials[i].normalMap;
    }

    for (const Part& part : parts)
    {
        if (part.indices.empty())
            continue;
        SubMesh submesh;
        submesh.indexOffset = static_cast<u32>(mesh.indices.size());
        submesh.indexCount = static_cast<u32>(part.indices.size());
        submesh.materialSlot = part.material;
        mesh.indices.insert(mesh.indices.end(), part.indices.begin(), part.indices.end());
        mesh.submeshes.push_back(submesh);
    }

    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 a = mesh.indices[i];
        const u32 b = mesh.indices[i + 1];
        const u32 c = mesh.indices[i + 2];
        const Math::Vec3 face = glm::cross(mesh.positions[b] - mesh.positions[a],
                                          mesh.positions[c] - mesh.positions[a]);
        if (missingNormals[a])
            mesh.normals[a] += face;
        if (missingNormals[b])
            mesh.normals[b] += face;
        if (missingNormals[c])
            mesh.normals[c] += face;
    }
    for (usize i = 0; i < mesh.normals.size(); ++i)
    {
        if (missingNormals[i] && glm::dot(mesh.normals[i], mesh.normals[i]) > 0.0f)
            mesh.normals[i] = glm::normalize(mesh.normals[i]);
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

    return !mesh.positions.empty() && !mesh.indices.empty();
}

} // namespace Radion
