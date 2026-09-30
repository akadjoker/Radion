#include "PCH.h"
#include "GltfExporter.h"

#include "FileSystem.h"
#include "Mesh.h"

#include <nlohmann/json.hpp>

#include <cstring>
#include <fstream>

using namespace Radion;

namespace
{
using Json = nlohmann::json;

// glTF constants.
constexpr int kFloat = 5126;
constexpr int kUnsignedInt = 5125;
constexpr int kArrayBuffer = 34962;
constexpr int kElementArrayBuffer = 34963;
constexpr u32 kGlbMagic = 0x46546C67; // "glTF"
constexpr u32 kChunkJson = 0x4E4F534A;
constexpr u32 kChunkBin = 0x004E4942;

bool fail(std::string* error, const std::string& message)
{
    if (error)
        *error = message;
    return false;
}

template <typename T> void append(std::vector<unsigned char>& bytes, const T* data, usize count)
{
    const usize size = sizeof(T) * count;
    const usize at = bytes.size();
    bytes.resize(at + size);
    if (size)
        std::memcpy(bytes.data() + at, data, size);
}

void pad(std::vector<unsigned char>& bytes, unsigned char fill)
{
    while (bytes.size() % 4 != 0)
        bytes.push_back(fill);
}

void appendU32(std::vector<unsigned char>& bytes, u32 value)
{
    // glTF is little-endian on every platform.
    for (int shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<unsigned char>((value >> shift) & 0xFF));
}

// A buffer view onto the binary chunk, returning its index.
int addView(Json& views, std::vector<unsigned char>& bin, const void* data, usize size, int target)
{
    pad(bin, 0);
    Json view = {{"buffer", 0}, {"byteOffset", bin.size()}, {"byteLength", size}};
    if (target)
        view["target"] = target;
    append(bin, static_cast<const unsigned char*>(data), size);
    views.push_back(view);
    return static_cast<int>(views.size()) - 1;
}

int addAccessor(Json& accessors, int view, usize byteOffset, int componentType, usize count,
                const char* type)
{
    Json accessor = {{"bufferView", view},
                     {"byteOffset", byteOffset},
                     {"componentType", componentType},
                     {"count", count},
                     {"type", type}};
    accessors.push_back(accessor);
    return static_cast<int>(accessors.size()) - 1;
}

Json materialJson(const Material& material, usize index)
{
    Json json;
    json["name"] = material.name.empty() ? "material" + std::to_string(index) : material.name;
    json["pbrMetallicRoughness"] = {
        {"baseColorFactor", {material.params.baseColor.r, material.params.baseColor.g,
                             material.params.baseColor.b, material.params.baseColor.a}},
        {"metallicFactor", glm::clamp(material.params.surface.y, 0.0f, 1.0f)},
        {"roughnessFactor", glm::clamp(material.params.surface.x, 0.0f, 1.0f)}};
    if (material.flags & MaterialTwoSided)
        json["doubleSided"] = true;
    if (material.params.baseColor.a < 1.0f || material.blend != BlendMode::Opaque)
        json["alphaMode"] = "BLEND";
    if (material.params.emissive.r > 0.0f || material.params.emissive.g > 0.0f ||
        material.params.emissive.b > 0.0f)
        json["emissiveFactor"] = {material.params.emissive.r, material.params.emissive.g,
                                  material.params.emissive.b};
    return json;
}
} // namespace

bool GltfExporter::build(const MeshData& mesh, const std::string& name,
                         std::vector<unsigned char>& glb, std::string* error)
{
    const usize vertexCount = mesh.positions.size();
    if (vertexCount == 0 || mesh.indices.size() < 3)
        return fail(error, "the mesh is empty");

    for (const u32 index : mesh.indices)
    {
        if (index >= vertexCount)
            return fail(error, "a triangle refers to a vertex that does not exist");
    }

    // A mesh with no submeshes is one primitive over every index.
    std::vector<SubMesh> submeshes = mesh.submeshes;
    if (submeshes.empty())
    {
        SubMesh whole;
        whole.indexCount = static_cast<u32>(mesh.indices.size());
        submeshes.push_back(whole);
    }
    for (SubMesh& submesh : submeshes)
    {
        if (static_cast<u64>(submesh.indexOffset) + submesh.indexCount > mesh.indices.size())
            return fail(error, "a submesh reaches past the end of the index list");
        // glTF triangles come in threes.
        submesh.indexCount -= submesh.indexCount % 3;
    }

    Json views = Json::array();
    Json accessors = Json::array();
    std::vector<unsigned char> bin;

    // Positions need their extent declared.
    glm::vec3 low(3.402823466e+38f);
    glm::vec3 high(-3.402823466e+38f);
    for (const glm::vec3& position : mesh.positions)
    {
        low = glm::min(low, position);
        high = glm::max(high, position);
    }
    const int positionView = addView(views, bin, mesh.positions.data(),
                                     vertexCount * sizeof(glm::vec3), kArrayBuffer);
    const int positionAccessor = addAccessor(accessors, positionView, 0, kFloat, vertexCount, "VEC3");
    accessors[positionAccessor]["min"] = {low.x, low.y, low.z};
    accessors[positionAccessor]["max"] = {high.x, high.y, high.z};

    int normalAccessor = -1;
    if (mesh.normals.size() == vertexCount)
    {
        const int view = addView(views, bin, mesh.normals.data(), vertexCount * sizeof(glm::vec3),
                                 kArrayBuffer);
        normalAccessor = addAccessor(accessors, view, 0, kFloat, vertexCount, "VEC3");
    }

    int uvAccessor = -1;
    if (mesh.uvs.size() == vertexCount)
    {
        const int view = addView(views, bin, mesh.uvs.data(), vertexCount * sizeof(glm::vec2),
                                 kArrayBuffer);
        uvAccessor = addAccessor(accessors, view, 0, kFloat, vertexCount, "VEC2");
    }

    const int indexView = addView(views, bin, mesh.indices.data(), mesh.indices.size() * sizeof(u32),
                                  kElementArrayBuffer);

    // Only the materials some submesh uses, renumbered, so the file carries no
    // dead entries.
    Json materials = Json::array();
    std::vector<int> materialOf(mesh.materials.size(), -1);

    Json primitives = Json::array();
    for (const SubMesh& submesh : submeshes)
    {
        if (submesh.indexCount == 0)
            continue;

        Json attributes = {{"POSITION", positionAccessor}};
        if (normalAccessor >= 0)
            attributes["NORMAL"] = normalAccessor;
        if (uvAccessor >= 0)
            attributes["TEXCOORD_0"] = uvAccessor;

        Json primitive = {
            {"attributes", attributes},
            {"indices", addAccessor(accessors, indexView,
                                    static_cast<usize>(submesh.indexOffset) * sizeof(u32),
                                    kUnsignedInt, submesh.indexCount, "SCALAR")},
            {"mode", 4}};

        if (submesh.materialSlot < mesh.materials.size())
        {
            int& slot = materialOf[submesh.materialSlot];
            if (slot < 0)
            {
                slot = static_cast<int>(materials.size());
                materials.push_back(materialJson(mesh.materials[submesh.materialSlot],
                                                 submesh.materialSlot));
            }
            primitive["material"] = slot;
        }
        primitives.push_back(primitive);
    }
    if (primitives.empty())
        return fail(error, "no submesh has any triangles");

    pad(bin, 0);

    const std::string label = name.empty() ? "model" : name;
    Json root = {{"asset", {{"version", "2.0"}, {"generator", "Radion Blender"}}},
                 {"scene", 0},
                 {"scenes", Json::array({{{"name", label}, {"nodes", Json::array({0})}}})},
                 {"nodes", Json::array({{{"name", label}, {"mesh", 0}}})},
                 {"meshes", Json::array({{{"name", label}, {"primitives", primitives}}})},
                 {"accessors", accessors},
                 {"bufferViews", views},
                 {"buffers", Json::array({{{"byteLength", bin.size()}}})}};
    if (!materials.empty())
        root["materials"] = materials;

    std::string jsonText = root.dump(-1, ' ', false, Json::error_handler_t::replace);
    while (jsonText.size() % 4 != 0)
        jsonText.push_back(' ');

    glb.clear();
    const u32 total = static_cast<u32>(12 + 8 + jsonText.size() + 8 + bin.size());
    appendU32(glb, kGlbMagic);
    appendU32(glb, 2);
    appendU32(glb, total);
    appendU32(glb, static_cast<u32>(jsonText.size()));
    appendU32(glb, kChunkJson);
    glb.insert(glb.end(), jsonText.begin(), jsonText.end());
    appendU32(glb, static_cast<u32>(bin.size()));
    appendU32(glb, kChunkBin);
    glb.insert(glb.end(), bin.begin(), bin.end());
    return true;
}

bool GltfExporter::save(const MeshData& mesh, const std::string& path, std::string* error)
{
    std::vector<unsigned char> glb;
    if (!build(mesh, FileSystem::baseName(path), glb, error))
        return false;

    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file)
        return fail(error, "could not open '" + path + "' for writing");
    file.write(reinterpret_cast<const char*>(glb.data()), static_cast<std::streamsize>(glb.size()));
    file.close();
    if (!file)
        return fail(error, "writing '" + path + "' failed");
    return true;
}
