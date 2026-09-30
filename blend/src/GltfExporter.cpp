#include "PCH.h"
#include "GltfExporter.h"

#include "FileSystem.h"
#include "Mesh.h"

#include <nlohmann/json.hpp>

#include <cstring>
#include <fstream>
#include <map>

using namespace Radion;

namespace
{
using Json = nlohmann::json;

// glTF constants.
constexpr int kFloat = 5126;
constexpr int kUnsignedInt = 5125;
constexpr int kUnsignedByte = 5121;
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

// Hands out glTF texture indices for image files: an image that two materials
// share is embedded once. Anything it cannot embed becomes a warning and the
// material keeps only its factors.
class TextureTable
{
public:
    TextureTable(Json& views, std::vector<unsigned char>& bin, std::vector<std::string>* warnings) :
        mViews(views),
        mBin(bin),
        mWarnings(warnings)
    {
    }

    // -1 when there is no usable image for `file`.
    int textureFor(const std::string& file)
    {
        if (file.empty())
            return -1;
        const auto known = mTextureOfFile.find(file);
        if (known != mTextureOfFile.end())
            return known->second;

        int texture = -1;
        const ByteArray bytes = FileSystem::getSingleton().readBinary(file);
        const char* mime = bytes.empty() ? nullptr : mimeOf(bytes);
        if (bytes.empty())
            warn("texture '" + file + "' could not be read; left out of the file");
        else if (!mime)
            warn("texture '" + file + "' is not PNG or JPEG; left out of the file");
        else
        {
            const int view = addView(mViews, mBin, bytes.data(), bytes.size(), 0);
            mImages.push_back({{"name", FileSystem::baseName(file)}, {"mimeType", mime}, {"bufferView", view}});
            if (mSamplers.empty())
            {
                // Repeat/linear: what the editor's own sampler does with a freshly assigned map.
                mSamplers.push_back({{"magFilter", 9729}, {"minFilter", 9987}, {"wrapS", 10497}, {"wrapT", 10497}});
            }
            mTextures.push_back({{"sampler", 0}, {"source", static_cast<int>(mImages.size()) - 1}});
            texture = static_cast<int>(mTextures.size()) - 1;
        }
        mTextureOfFile[file] = texture;
        return texture;
    }

    bool empty() const
    {
        return mTextures.empty();
    }
    const Json& images() const
    {
        return mImages;
    }
    const Json& textures() const
    {
        return mTextures;
    }
    const Json& samplers() const
    {
        return mSamplers;
    }

private:
    static const char* mimeOf(const ByteArray& bytes)
    {
        const unsigned char* d = bytes.data();
        if (bytes.size() >= 8 && d[0] == 0x89 && d[1] == 'P' && d[2] == 'N' && d[3] == 'G')
            return "image/png";
        if (bytes.size() >= 3 && d[0] == 0xFF && d[1] == 0xD8 && d[2] == 0xFF)
            return "image/jpeg";
        return nullptr;
    }

    void warn(const std::string& message)
    {
        if (mWarnings)
            mWarnings->push_back(message);
    }

    Json& mViews;
    std::vector<unsigned char>& mBin;
    std::vector<std::string>* mWarnings;
    Json mImages = Json::array();
    Json mTextures = Json::array();
    Json mSamplers = Json::array();
    std::map<std::string, int> mTextureOfFile;
};

Json materialJson(const Material& material, usize index, TextureTable& textures)
{
    Json json;
    json["name"] = material.name.empty() ? "material" + std::to_string(index) : material.name;
    json["pbrMetallicRoughness"] = {
        {"baseColorFactor", {material.params.baseColor.x, material.params.baseColor.y,
                             material.params.baseColor.z, material.params.baseColor.w}},
        {"metallicFactor", Math::clamp(material.params.surface.y, 0.0f, 1.0f)},
        {"roughnessFactor", Math::clamp(material.params.surface.x, 0.0f, 1.0f)}};
    const int albedo = textures.textureFor(material.textures[SlotAlbedo].file);
    if (albedo >= 0)
        json["pbrMetallicRoughness"]["baseColorTexture"] = {{"index", albedo}};
    const int surface = textures.textureFor(material.textures[SlotSurface].file);
    if (surface >= 0)
        json["pbrMetallicRoughness"]["metallicRoughnessTexture"] = {{"index", surface}};
    const int normal = textures.textureFor(material.textures[SlotNormal].file);
    if (normal >= 0)
        json["normalTexture"] = {{"index", normal}};
    const int emissive = textures.textureFor(material.textures[SlotEmissive].file);
    if (emissive >= 0)
    {
        json["emissiveTexture"] = {{"index", emissive}};
        // glTF multiplies the map by the factor, and the factor defaults to black.
        if (!json.contains("emissiveFactor"))
            json["emissiveFactor"] = {1.0, 1.0, 1.0};
    }
    if (material.flags & MaterialTwoSided)
        json["doubleSided"] = true;
    if (material.params.baseColor.w < 1.0f || material.blend != BlendMode::Opaque)
        json["alphaMode"] = "BLEND";
    if (material.params.emissive.x > 0.0f || material.params.emissive.y > 0.0f ||
        material.params.emissive.z > 0.0f)
        json["emissiveFactor"] = {material.params.emissive.x, material.params.emissive.y,
                                  material.params.emissive.z};
    return json;
}
} // namespace

bool GltfExporter::build(const MeshData& mesh, const std::string& name,
                         std::vector<unsigned char>& glb, std::string* error,
                         std::vector<std::string>* warnings)
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
    Math::vec3 low(3.402823466e+38f);
    Math::vec3 high(-3.402823466e+38f);
    for (const Math::vec3& position : mesh.positions)
    {
        low = Math::min(low, position);
        high = Math::max(high, position);
    }
    const int positionView = addView(views, bin, mesh.positions.data(),
                                     vertexCount * sizeof(Math::vec3), kArrayBuffer);
    const int positionAccessor = addAccessor(accessors, positionView, 0, kFloat, vertexCount, "VEC3");
    accessors[positionAccessor]["min"] = {low.x, low.y, low.z};
    accessors[positionAccessor]["max"] = {high.x, high.y, high.z};

    int normalAccessor = -1;
    if (mesh.normals.size() == vertexCount)
    {
        const int view = addView(views, bin, mesh.normals.data(), vertexCount * sizeof(Math::vec3),
                                 kArrayBuffer);
        normalAccessor = addAccessor(accessors, view, 0, kFloat, vertexCount, "VEC3");
    }

    int uvAccessor = -1;
    if (mesh.uvs.size() == vertexCount)
    {
        const int view = addView(views, bin, mesh.uvs.data(), vertexCount * sizeof(Math::vec2),
                                 kArrayBuffer);
        uvAccessor = addAccessor(accessors, view, 0, kFloat, vertexCount, "VEC2");
    }

    int colorAccessor = -1;
    if (mesh.colors.size() == vertexCount)
    {
        // MeshAttribs::color is four bytes r,g,b,a in memory: exactly COLOR_0 as normalised bytes.
        const int view = addView(views, bin, mesh.colors.data(), vertexCount * sizeof(u32), kArrayBuffer);
        colorAccessor = addAccessor(accessors, view, 0, kUnsignedByte, vertexCount, "VEC4");
        accessors[colorAccessor]["normalized"] = true;
    }

    const int indexView = addView(views, bin, mesh.indices.data(), mesh.indices.size() * sizeof(u32),
                                  kElementArrayBuffer);

    // Only the materials some submesh uses, renumbered, so the file carries no
    // dead entries.
    Json materials = Json::array();
    TextureTable textureTable(views, bin, warnings);
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
        if (colorAccessor >= 0)
            attributes["COLOR_0"] = colorAccessor;

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
                                                 submesh.materialSlot, textureTable));
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
    if (!textureTable.empty())
    {
        root["images"] = textureTable.images();
        root["textures"] = textureTable.textures();
        root["samplers"] = textureTable.samplers();
    }

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

bool GltfExporter::save(const MeshData& mesh, const std::string& path, std::string* error,
                        std::vector<std::string>* warnings)
{
    std::vector<unsigned char> glb;
    if (!build(mesh, FileSystem::baseName(path), glb, error, warnings))
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
