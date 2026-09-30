#include "PCH.h"

#include "BSPImporter.h"

#include "ByteArray.h"
#include "FileSystem.h"
#include "Log.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Radion
{

namespace
{

constexpr u32 kQ3Ident = 0x50534249u; // "IBSP", little-endian
constexpr s32 kQ3Version = 46;
constexpr usize kLumpCount = 17;
constexpr usize kHeaderSize = 8 + kLumpCount * 8;

enum LumpIndex : usize
{
    LumpTextures = 1,
    LumpModels = 7,
    LumpVertices = 10,
    LumpMeshVertices = 11,
    LumpFaces = 13,
    LumpLightmaps = 14
};

constexpr usize kTextureSize = 72;
constexpr usize kModelSize = 40;
constexpr usize kVertexSize = 44;
constexpr usize kFaceSize = 104;
constexpr usize kLightmapWidth = 128;
constexpr usize kLightmapHeight = 128;
constexpr usize kLightmapSize = kLightmapWidth * kLightmapHeight * 3;

constexpr s32 kFacePolygon = 1;
constexpr s32 kFacePatch = 2;
constexpr s32 kFaceMesh = 3;

struct Lump
{
    usize offset = 0;
    usize length = 0;
};

struct SourceVertex
{
    Math::vec3 position = Math::vec3(0.0f);
    Math::vec3 normal = Math::vec3(0.0f, 1.0f, 0.0f);
    Math::vec2 uv = Math::vec2(0.0f);
    Math::vec2 lightmapUV = Math::vec2(0.0f);
    Math::vec4 color = Math::vec4(1.0f);
};

struct Face
{
    s32 texture = -1;
    s32 type = 0;
    s32 firstVertex = 0;
    s32 vertexCount = 0;
    s32 firstMeshVertex = 0;
    s32 meshVertexCount = 0;
    s32 lightmap = -1;
    s32 patchWidth = 0;
    s32 patchHeight = 0;
};

struct FaceGroup
{
    s32 texture = -1;
    s32 lightmap = -1;
    std::vector<usize> faces;
};

struct ShaderInfo
{
    std::string image;
    std::vector<std::string> animationFrames;
    f32 animationFPS = 0.0f;
    Math::vec2 scroll = Math::vec2(0.0f);
    f32 rotate = 0.0f;
    bool sky = false;
    bool fog = false;
    bool noDraw = false;
    bool transparent = false;
    bool additive = false;
    bool alphaTest = false;
    bool twoSided = false;
};

struct ShaderToken
{
    std::string text;
    bool newline = false;
};

struct GeneratedMaterialSource
{
    std::vector<std::string> albedoFrames;
    f32 animationFPS = 0.0f;
};

bool rangeFits(usize offset, usize length, usize size)
{
    return offset <= size && length <= size - offset;
}

u32 readU32(const u8* bytes, usize offset)
{
    return static_cast<u32>(bytes[offset + 0]) | (static_cast<u32>(bytes[offset + 1]) << 8) |
           (static_cast<u32>(bytes[offset + 2]) << 16) |
           (static_cast<u32>(bytes[offset + 3]) << 24);
}

s32 readI32(const u8* bytes, usize offset)
{
    return static_cast<s32>(readU32(bytes, offset));
}

f32 readF32(const u8* bytes, usize offset)
{
    const u32 bits = readU32(bytes, offset);
    f32 value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
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

std::string fixedString(const u8* bytes, usize length)
{
    usize count = 0;
    while (count < length && bytes[count] != 0)
        ++count;
    return std::string(reinterpret_cast<const char*>(bytes), count);
}

std::string lower(std::string value)
{
    for (char& c : value)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return value;
}

std::vector<ShaderToken> tokenizeShader(const std::string& text)
{
    std::vector<ShaderToken> tokens;
    usize cursor = 0;
    while (cursor < text.size())
    {
        const char c = text[cursor];
        if (c == '\n')
        {
            tokens.push_back({std::string(), true});
            ++cursor;
            continue;
        }
        if (c == '\r' || c == ' ' || c == '\t')
        {
            ++cursor;
            continue;
        }
        if (c == '/' && cursor + 1 < text.size() && text[cursor + 1] == '/')
        {
            cursor += 2;
            while (cursor < text.size() && text[cursor] != '\n')
                ++cursor;
            continue;
        }
        if (c == '{' || c == '}')
        {
            tokens.push_back({std::string(1, c), false});
            ++cursor;
            continue;
        }

        const usize start = cursor;
        while (cursor < text.size())
        {
            const char value = text[cursor];
            if (std::isspace(static_cast<unsigned char>(value)) || value == '{' || value == '}' ||
                (value == '/' && cursor + 1 < text.size() && text[cursor + 1] == '/'))
                break;
            ++cursor;
        }
        if (cursor > start)
            tokens.push_back({text.substr(start, cursor - start), false});
    }
    return tokens;
}

void parseShaderText(const std::string& text, std::unordered_map<std::string, ShaderInfo>& shaders)
{
    const std::vector<ShaderToken> tokens = tokenizeShader(text);
    usize cursor = 0;
    while (cursor < tokens.size())
    {
        while (cursor < tokens.size() && tokens[cursor].newline)
            ++cursor;
        if (cursor >= tokens.size())
            break;

        const std::string shaderName = tokens[cursor++].text;
        while (cursor < tokens.size() && tokens[cursor].newline)
            ++cursor;
        if (cursor >= tokens.size() || tokens[cursor].text != "{")
            continue;
        ++cursor;

        ShaderInfo info;
        std::string firstMap;
        std::string editorImage;
        s32 depth = 1;
        while (cursor < tokens.size() && depth > 0)
        {
            if (tokens[cursor].newline)
            {
                ++cursor;
                continue;
            }
            if (tokens[cursor].text == "{")
            {
                ++depth;
                ++cursor;
                continue;
            }
            if (tokens[cursor].text == "}")
            {
                --depth;
                ++cursor;
                continue;
            }

            const s32 directiveDepth = depth;
            const std::string keyword = lower(tokens[cursor++].text);
            std::vector<std::string> arguments;
            while (cursor < tokens.size() && !tokens[cursor].newline &&
                   tokens[cursor].text != "{" && tokens[cursor].text != "}")
                arguments.push_back(tokens[cursor++].text);

            if (directiveDepth == 1 && keyword == "surfaceparm" && !arguments.empty())
            {
                const std::string value = lower(arguments[0]);
                info.sky |= value == "sky";
                info.fog |= value == "fog";
                info.noDraw |= value == "nodraw";
                info.transparent |= value == "trans";
            }
            else if (directiveDepth == 1 && keyword == "qer_editorimage" && !arguments.empty())
            {
                editorImage = arguments[0];
            }
            else if (directiveDepth == 1 && keyword == "cull" && !arguments.empty())
            {
                const std::string value = lower(arguments[0]);
                info.twoSided |= value == "none" || value == "disable" || value == "twosided";
            }
            else if (directiveDepth >= 2 && (keyword == "map" || keyword == "clampmap") &&
                     !arguments.empty())
            {
                if (firstMap.empty() && !arguments[0].empty() && arguments[0][0] != '$')
                    firstMap = arguments[0];
            }
            else if (directiveDepth >= 2 && keyword == "animmap" && arguments.size() >= 2)
            {
                if (info.animationFrames.empty())
                {
                    info.animationFPS = std::strtof(arguments[0].c_str(), nullptr);
                    info.animationFrames.assign(arguments.begin() + 1, arguments.end());
                }
            }
            else if (directiveDepth >= 2 && keyword == "blendfunc" && !arguments.empty())
            {
                const std::string source = lower(arguments[0]);
                const std::string destination =
                    arguments.size() > 1 ? lower(arguments[1]) : std::string();
                if (source == "add" || (source == "gl_one" && destination == "gl_one"))
                {
                    info.transparent = true;
                    info.additive = true;
                }
                else if (source != "filter" &&
                         !(source == "gl_dst_color" && destination == "gl_zero") &&
                         !(source == "gl_one" && destination == "gl_zero"))
                {
                    info.transparent = true;
                }
            }
            else if (directiveDepth >= 2 && keyword == "alphafunc")
            {
                info.alphaTest = true;
            }
            else if (directiveDepth >= 2 && keyword == "tcmod" && arguments.size() >= 2)
            {
                const std::string operation = lower(arguments[0]);
                if (operation == "scroll" && arguments.size() >= 3)
                {
                    info.scroll.x = std::strtof(arguments[1].c_str(), nullptr);
                    info.scroll.y = std::strtof(arguments[2].c_str(), nullptr);
                }
                else if (operation == "rotate")
                {
                    info.rotate = std::strtof(arguments[1].c_str(), nullptr);
                }
            }
        }

        info.image = !firstMap.empty()
                         ? firstMap
                         : (!editorImage.empty()
                                ? editorImage
                                : (info.animationFrames.empty() ? std::string()
                                                                : info.animationFrames.front()));
        if (!shaderName.empty())
            shaders[lower(shaderName)] = std::move(info);
    }
}

std::unordered_map<std::string, ShaderInfo> loadShaderInfo(FileSystem& files,
                                                           const std::string& bspFilename)
{
    std::unordered_map<std::string, ShaderInfo> shaders;
    std::unordered_set<std::string> scriptFiles;

    const auto collectDirectory = [&files, &scriptFiles](const std::string& directory)
    {
        if (!files.isDirectory(directory))
            return;
        for (const FileSystem::DirEntry& entry : files.listDirectory(directory))
        {
            if (!entry.isDirectory && lower(FileSystem::extensionOf(entry.name)) == ".shader")
                scriptFiles.insert(FileSystem::join(directory, entry.name));
        }
    };

    for (const std::string& path : files.getSearchPaths())
        collectDirectory(FileSystem::join(path, "scripts"));
    collectDirectory(FileSystem::join(FileSystem::directoryOf(bspFilename), "scripts"));

    // Archives cannot be listed through listDirectory(), but Q3 packs expose
    // their script names through scripts/shaderlist.txt.
    const std::string shaderList = files.readText("scripts/shaderlist.txt");
    for (const ShaderToken& token : tokenizeShader(shaderList))
    {
        if (token.newline || token.text.empty() || token.text == "{" || token.text == "}")
            continue;
        std::string name = token.text;
        if (lower(FileSystem::extensionOf(name)) != ".shader")
            name += ".shader";
        scriptFiles.insert(FileSystem::join("scripts", name));
    }

    for (const std::string& script : scriptFiles)
    {
        const std::string source = files.readText(script);
        if (!source.empty())
            parseShaderText(source, shaders);
    }
    return shaders;
}

bool hasExtension(const std::string& path)
{
    const usize slash = path.find_last_of("/\\");
    const usize dot = path.find_last_of('.');
    return dot != std::string::npos && (slash == std::string::npos || dot > slash);
}

std::string resolveTexture(FileSystem& files, const std::string& bspFilename,
                           const std::string& texture)
{
    if (texture.empty())
        return std::string();

    static const char* extensions[] = {".tga", ".png", ".jpg", ".jpeg", ".dds", ".bmp"};
    const std::string directory = FileSystem::directoryOf(bspFilename);
    const std::string besideBsp = FileSystem::join(directory, texture);

    const auto resolveCandidate = [&files](const std::string& candidate) -> std::string
    {
        return files.exists(candidate) ? candidate : std::string();
    };

    if (hasExtension(texture))
    {
        std::string found = resolveCandidate(texture);
        if (!found.empty())
            return found;
        found = resolveCandidate(besideBsp);
        if (!found.empty())
            return found;
    }

    const std::string stem =
        hasExtension(texture) ? FileSystem::withoutExtension(texture) : texture;
    const std::string besideStem = FileSystem::join(directory, stem);
    for (const char* extension : extensions)
    {
        std::string found = resolveCandidate(stem + extension);
        if (!found.empty())
            return found;
        found = resolveCandidate(besideStem + extension);
        if (!found.empty())
            return found;
    }

    // Keep the shader/texture name even when it cannot be resolved yet.
    // AssetManager may have more search paths mounted by upload time.
    return texture;
}

std::string portablePath(std::string path)
{
    std::replace(path.begin(), path.end(), '\\', '/');
    return path;
}

std::string quoted(const std::string& value)
{
    std::string result = "\"";
    for (char c : value)
    {
        if (c == '\\' || c == '"')
            result.push_back('\\');
        result.push_back(c);
    }
    result.push_back('"');
    return result;
}

bool writeLightmapTGA(FileSystem& files, const std::string& filename,
                      const std::vector<u8>& rgba)
{
    const usize pixelCount = kLightmapWidth * kLightmapHeight;
    if (rgba.size() != pixelCount * 4)
        return false;

    ByteArray tga(18 + pixelCount * 4);
    if (!tga.data())
        return false;
    std::memset(tga.data(), 0, tga.size());
    tga[2] = 2; // uncompressed true-colour
    tga[12] = static_cast<u8>(kLightmapWidth & 0xFFu);
    tga[13] = static_cast<u8>((kLightmapWidth >> 8) & 0xFFu);
    tga[14] = static_cast<u8>(kLightmapHeight & 0xFFu);
    tga[15] = static_cast<u8>((kLightmapHeight >> 8) & 0xFFu);
    tga[16] = 32;
    tga[17] = 0x28; // 8 alpha bits, top-left origin
    for (usize pixel = 0; pixel < pixelCount; ++pixel)
    {
        tga[18 + pixel * 4 + 0] = rgba[pixel * 4 + 2];
        tga[18 + pixel * 4 + 1] = rgba[pixel * 4 + 1];
        tga[18 + pixel * 4 + 2] = rgba[pixel * 4 + 0];
        tga[18 + pixel * 4 + 3] = rgba[pixel * 4 + 3];
    }
    return files.writeBinary(filename, tga);
}

const char* blendName(BlendMode blend)
{
    switch (blend)
    {
    case BlendMode::Alpha:
        return "Alpha";
    case BlendMode::Additive:
        return "Additive";
    case BlendMode::Multiply:
        return "Multiply";
    case BlendMode::PremultipliedAlpha:
        return "PremultipliedAlpha";
    case BlendMode::AddColors:
        return "AddColors";
    case BlendMode::SubtractColors:
        return "SubtractColors";
    default:
        return "Opaque";
    }
}

const char* cullName(CullMode cull)
{
    if (cull == CullMode::None)
        return "None";
    if (cull == CullMode::Front)
        return "Front";
    return "Back";
}

void writeFlags(std::ostringstream& output, u32 flags)
{
    struct FlagName
    {
        u32 flag;
        const char* name;
    };
    static const FlagName names[] = {{MaterialCastShadow, "CastShadow"},
                                     {MaterialReceiveShadow, "ReceiveShadow"},
                                     {MaterialTwoSided, "TwoSided"},
                                     {MaterialAlphaTest, "AlphaTest"},
                                     {MaterialNoDepthWrite, "NoDepthWrite"},
                                     {MaterialAnimated, "Animated"},
                                     {MaterialLit, "Lit"}};
    output << '[';
    bool comma = false;
    for (const FlagName& entry : names)
    {
        if (!(flags & entry.flag))
            continue;
        if (comma)
            output << ", ";
        output << entry.name;
        comma = true;
    }
    output << ']';
}

bool writeMaterialFile(FileSystem& files, const std::string& filename, const MeshData& mesh,
                       const std::vector<GeneratedMaterialSource>& sources)
{
    if (sources.size() != mesh.materials.size())
        return false;

    std::ostringstream output;
    for (usize i = 0; i < mesh.materials.size(); ++i)
    {
        const Material& material = mesh.materials[i];
        const GeneratedMaterialSource& source = sources[i];
        output << "material " << quoted(material.name) << "\n{\n    flags ";
        writeFlags(output, material.flags);
        output << "\n    blendMode " << blendName(material.blend) << "\n    cullMode "
               << cullName(material.cull) << "\n\n    properties\n    {\n"
               << "        baseColor (" << material.params.baseColor.x << ", "
               << material.params.baseColor.y << ", " << material.params.baseColor.z << ", "
               << material.params.baseColor.w << ")\n"
               << "        surface (" << material.params.surface.x << ", "
               << material.params.surface.y << ", " << material.params.surface.z << ", "
               << material.params.surface.w << ")\n"
               << "        uvTransform (" << material.params.uvTransform.x << ", "
               << material.params.uvTransform.y << ", " << material.params.uvTransform.z << ", "
               << material.params.uvTransform.w << ")\n"
               << "        uvAnim (" << material.params.uvAnim.x << ", "
               << material.params.uvAnim.y << ", " << material.params.uvAnim.z << ", "
               << material.params.uvAnim.w << ")\n"
               << "        sequence (" << material.params.sequence.x << ", "
               << material.params.sequence.y << ", " << material.params.sequence.z << ", "
               << material.params.sequence.w << ")\n    }\n\n    textures\n    {\n";

        if (!source.albedoFrames.empty())
        {
            output << "        texture \"Albedo\"\n        {\n            slot Albedo\n";
            if (source.albedoFrames.size() > 1 && source.animationFPS > 0.0f)
            {
                output << "            type Sequence\n            frames [";
                for (usize frame = 0; frame < source.albedoFrames.size(); ++frame)
                {
                    if (frame)
                        output << ", ";
                    output << quoted(portablePath(source.albedoFrames[frame]));
                }
                output << "]\n            fps " << source.animationFPS
                       << "\n            loop true\n";
            }
            else
            {
                output << "            type Static\n            file "
                       << quoted(portablePath(source.albedoFrames.front())) << "\n";
            }
            if (material.params.uvAnim != Math::vec4(0.0f))
                output << "            uvAnimation\n            {\n"
                       << "                scrollSpeed (" << material.params.uvAnim.x << ", "
                       << material.params.uvAnim.y << ")\n"
                       << "                rotateSpeed " << material.params.uvAnim.z
                       << "\n            }\n";
            output << "            filter Anisotropic\n            wrap Repeat\n"
                   << "            srgb true\n            generateMips true\n        }\n";
        }

        const MaterialTexture& lightmap = material.textures[SlotLightmap];
        if (!lightmap.file.empty())
            output << "        texture \"Lightmap\"\n        {\n"
                   << "            slot Lightmap\n            type Static\n            file "
                   << quoted(portablePath(lightmap.file))
                   << "\n            filter Linear\n            wrap Clamp\n"
                   << "            srgb false\n            generateMips true\n        }\n";
        output << "    }\n}\n\n";
    }
    return files.writeText(filename, output.str());
}

void patchBasis(f32 t, f32& b0, f32& b1, f32& b2)
{
    const f32 inverse = 1.0f - t;
    b0 = inverse * inverse;
    b1 = 2.0f * t * inverse;
    b2 = t * t;
}

template <typename T> T evaluatePatchValue(const T values[9], f32 u, f32 v)
{
    f32 u0, u1, u2, v0, v1, v2;
    patchBasis(u, u0, u1, u2);
    patchBasis(v, v0, v1, v2);
    const T row0 = values[0] * u0 + values[1] * u1 + values[2] * u2;
    const T row1 = values[3] * u0 + values[4] * u1 + values[5] * u2;
    const T row2 = values[6] * u0 + values[7] * u1 + values[8] * u2;
    return row0 * v0 + row1 * v1 + row2 * v2;
}

SourceVertex evaluatePatch(const SourceVertex control[9], f32 u, f32 v)
{
    Math::vec3 positions[9];
    Math::vec3 normals[9];
    Math::vec2 uvs[9];
    Math::vec2 lightmapUVs[9];
    Math::vec4 colors[9];
    for (usize i = 0; i < 9; ++i)
    {
        positions[i] = control[i].position;
        normals[i] = control[i].normal;
        uvs[i] = control[i].uv;
        lightmapUVs[i] = control[i].lightmapUV;
        colors[i] = control[i].color;
    }

    SourceVertex result;
    result.position = evaluatePatchValue(positions, u, v);
    result.normal = evaluatePatchValue(normals, u, v);
    const f32 normalLength = Math::length(result.normal);
    result.normal =
        normalLength > 1e-6f ? result.normal / normalLength : Math::vec3(0.0f, 1.0f, 0.0f);
    result.uv = evaluatePatchValue(uvs, u, v);
    result.lightmapUV = evaluatePatchValue(lightmapUVs, u, v);
    result.color = evaluatePatchValue(colors, u, v);
    return result;
}

u32 packColor(const Math::vec4& color)
{
    const Math::vec4 clamped = Math::clamp(color, Math::vec4(0.0f), Math::vec4(1.0f));
    const u32 r = static_cast<u32>(clamped.r * 255.0f + 0.5f);
    const u32 g = static_cast<u32>(clamped.g * 255.0f + 0.5f);
    const u32 b = static_cast<u32>(clamped.b * 255.0f + 0.5f);
    const u32 a = static_cast<u32>(clamped.a * 255.0f + 0.5f);
    return (a << 24) | (r << 16) | (g << 8) | b;
}

u32 appendVertex(MeshData& mesh, const SourceVertex& vertex)
{
    const u32 index = static_cast<u32>(mesh.positions.size());
    mesh.positions.push_back(vertex.position);
    mesh.normals.push_back(vertex.normal);
    // Keep the UV transform used by the reference loader.
    mesh.uvs.emplace_back(1.0f - vertex.uv.x, vertex.uv.y);
    mesh.uvs2.push_back(vertex.lightmapUV);
    mesh.colors.push_back(packColor(vertex.color));
    return index;
}

bool appendIndexedFace(const Face& face, const std::vector<SourceVertex>& vertices,
                       const std::vector<s32>& meshVertices, MeshData& mesh)
{
    if (face.firstMeshVertex < 0 || face.meshVertexCount < 3 || face.firstVertex < 0)
        return false;

    const usize first = static_cast<usize>(face.firstMeshVertex);
    const usize count = static_cast<usize>(face.meshVertexCount);
    if (first > meshVertices.size() || count > meshVertices.size() - first)
        return false;

    bool appended = false;
    for (usize corner = 0; corner + 2 < count; corner += 3)
    {
        s64 indices[3];
        bool valid = true;
        for (usize i = 0; i < 3; ++i)
        {
            indices[i] = static_cast<s64>(face.firstVertex) + meshVertices[first + corner + i];
            if (indices[i] < 0 || static_cast<u64>(indices[i]) >= vertices.size())
                valid = false;
        }
        if (!valid)
            continue;

        for (usize i = 0; i < 3; ++i)
            mesh.indices.push_back(appendVertex(mesh, vertices[static_cast<usize>(indices[i])]));
        appended = true;
    }
    return appended;
}

bool appendPatch(const Face& face, const std::vector<SourceVertex>& vertices, u32 tessellation,
                 MeshData& mesh)
{
    if (face.firstVertex < 0 || face.patchWidth < 3 || face.patchHeight < 3 ||
        (face.patchWidth & 1) == 0 || (face.patchHeight & 1) == 0)
        return false;

    const u64 controlCount = static_cast<u64>(face.patchWidth) * face.patchHeight;
    const u64 first = static_cast<u64>(face.firstVertex);
    if (first > vertices.size() || controlCount > vertices.size() - first)
        return false;

    bool appended = false;
    const u32 rowWidth = tessellation + 1;
    for (s32 by = 0; by + 2 < face.patchHeight; by += 2)
    {
        for (s32 bx = 0; bx + 2 < face.patchWidth; bx += 2)
        {
            SourceVertex control[9];
            for (s32 y = 0; y < 3; ++y)
                for (s32 x = 0; x < 3; ++x)
                {
                    const usize source = static_cast<usize>(face.firstVertex) +
                                         static_cast<usize>(by + y) * face.patchWidth + bx + x;
                    control[y * 3 + x] = vertices[source];
                }

            const u32 baseVertex = static_cast<u32>(mesh.positions.size());
            for (u32 y = 0; y <= tessellation; ++y)
                for (u32 x = 0; x <= tessellation; ++x)
                {
                    const f32 u = static_cast<f32>(x) / static_cast<f32>(tessellation);
                    const f32 v = static_cast<f32>(y) / static_cast<f32>(tessellation);
                    appendVertex(mesh, evaluatePatch(control, u, v));
                }

            for (u32 y = 0; y < tessellation; ++y)
                for (u32 x = 0; x < tessellation; ++x)
                {
                    const u32 i0 = baseVertex + y * rowWidth + x;
                    const u32 i1 = i0 + 1;
                    const u32 i2 = i0 + rowWidth;
                    const u32 i3 = i2 + 1;
                    mesh.indices.push_back(i0);
                    mesh.indices.push_back(i2);
                    mesh.indices.push_back(i1);
                    mesh.indices.push_back(i1);
                    mesh.indices.push_back(i2);
                    mesh.indices.push_back(i3);
                }
            appended = true;
        }
    }
    return appended;
}

void computeBounds(MeshData& mesh)
{
    mesh.bounds = AABB();
    for (const Math::vec3& position : mesh.positions)
        mesh.bounds.expand(position);

    for (SubMesh& submesh : mesh.submeshes)
    {
        submesh.bounds = AABB();
        const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
        for (u64 i = submesh.indexOffset; i < end && i < mesh.indices.size(); ++i)
        {
            const u32 vertex = mesh.indices[static_cast<usize>(i)];
            if (vertex < mesh.positions.size())
                submesh.bounds.expand(mesh.positions[vertex]);
        }
    }
}

} // namespace

BSPImporter::BSPImporter(u32 patchTessellation, bool worldspawnOnly)
    : mPatchTessellation(std::max(1u, std::min(patchTessellation, 32u))),
      mWorldspawnOnly(worldspawnOnly)
{
}

bool BSPImporter::supports(const char* extension) const
{
    return extension && std::strcmp(extension, "bsp") == 0;
}

bool BSPImporter::import(const std::string& filename, ByteArray& data, FileSystem& files,
                         MeshData& mesh)
{
    BSPMapData map;
    if (!importMap(filename, data, files, map))
        return false;
    mesh = std::move(map.mesh);
    return true;
}

bool BSPImporter::importMap(const std::string& filename, ByteArray& data, FileSystem& files,
                            BSPMapData& result)
{
    const u8* bytes = data.data();
    const usize size = data.size();
    if (!bytes || size < kHeaderSize)
    {
        Log::error("BSPImporter: '%s' has a truncated header", filename.c_str());
        return false;
    }
    if (readU32(bytes, 0) != kQ3Ident || readI32(bytes, 4) != kQ3Version)
    {
        Log::error("BSPImporter: '%s' is not a Quake 3 IBSP version 46 file", filename.c_str());
        return false;
    }

    Lump lumps[kLumpCount];
    for (usize i = 0; i < kLumpCount; ++i)
    {
        const s32 offset = readI32(bytes, 8 + i * 8);
        const s32 length = readI32(bytes, 12 + i * 8);
        if (offset < 0 || length < 0 ||
            !rangeFits(static_cast<usize>(offset), static_cast<usize>(length), size))
        {
            Log::error("BSPImporter: '%s' has an invalid lump %zu", filename.c_str(), i);
            return false;
        }
        lumps[i].offset = static_cast<usize>(offset);
        lumps[i].length = static_cast<usize>(length);
    }

    const Lump& textureLump = lumps[LumpTextures];
    const Lump& vertexLump = lumps[LumpVertices];
    const Lump& meshVertexLump = lumps[LumpMeshVertices];
    const Lump& faceLump = lumps[LumpFaces];
    const Lump& lightmapLump = lumps[LumpLightmaps];
    if (textureLump.length % kTextureSize != 0 || vertexLump.length % kVertexSize != 0 ||
        meshVertexLump.length % sizeof(s32) != 0 || faceLump.length % kFaceSize != 0 ||
        lightmapLump.length % kLightmapSize != 0)
    {
        Log::error("BSPImporter: '%s' has a misaligned geometry lump", filename.c_str());
        return false;
    }

    std::vector<std::string> textureNames(textureLump.length / kTextureSize);
    for (usize i = 0; i < textureNames.size(); ++i)
        textureNames[i] = fixedString(bytes + textureLump.offset + i * kTextureSize, 64);

    BSPMapData importedMap;
    importedMap.lightmaps.resize(lightmapLump.length / kLightmapSize);
    constexpr f32 lightmapGamma = 0.3f;
    u8 gammaLUT[256];
    for (u32 value = 0; value < 256; ++value)
    {
        const f32 corrected = std::pow(static_cast<f32>(value) / 255.0f, lightmapGamma) * 255.0f;
        gammaLUT[value] = static_cast<u8>(std::min(255.0f, std::max(0.0f, corrected)));
    }
    for (usize pageIndex = 0; pageIndex < importedMap.lightmaps.size(); ++pageIndex)
    {
        BSPLightmapPage& page = importedMap.lightmaps[pageIndex];
        page.rgba.resize(kLightmapWidth * kLightmapHeight * 4);
        const u8* source = bytes + lightmapLump.offset + pageIndex * kLightmapSize;
        for (usize pixel = 0; pixel < kLightmapWidth * kLightmapHeight; ++pixel)
        {
            page.rgba[pixel * 4 + 0] = gammaLUT[source[pixel * 3 + 0]];
            page.rgba[pixel * 4 + 1] = gammaLUT[source[pixel * 3 + 1]];
            page.rgba[pixel * 4 + 2] = gammaLUT[source[pixel * 3 + 2]];
            page.rgba[pixel * 4 + 3] = 255;
        }
    }

    const std::unordered_map<std::string, ShaderInfo> shaders = loadShaderInfo(files, filename);

    std::vector<SourceVertex> vertices(vertexLump.length / kVertexSize);
    for (usize i = 0; i < vertices.size(); ++i)
    {
        const usize offset = vertexLump.offset + i * kVertexSize;
        SourceVertex& vertex = vertices[i];
        // Quake 3 is Z-up. Swapping Y/Z matches the reference loader's Y-up
        // conversion and its established winding convention.
        vertex.position = Math::vec3(readF32(bytes, offset + 0), readF32(bytes, offset + 8),
                                    readF32(bytes, offset + 4));
        vertex.uv = Math::vec2(readF32(bytes, offset + 12), readF32(bytes, offset + 16));
        vertex.lightmapUV = Math::vec2(readF32(bytes, offset + 20), readF32(bytes, offset + 24));
        vertex.normal = Math::vec3(readF32(bytes, offset + 28), readF32(bytes, offset + 36),
                                  readF32(bytes, offset + 32));
        const f32 normalLength = Math::length(vertex.normal);
        vertex.normal =
            normalLength > 1e-6f ? vertex.normal / normalLength : Math::vec3(0.0f, 1.0f, 0.0f);
        vertex.color = Math::vec4(bytes[offset + 40], bytes[offset + 41], bytes[offset + 42],
                                 bytes[offset + 43]) /
                       255.0f;
        if (!std::isfinite(vertex.position.x) || !std::isfinite(vertex.position.y) ||
            !std::isfinite(vertex.position.z))
        {
            Log::error("BSPImporter: '%s' contains a non-finite vertex", filename.c_str());
            return false;
        }
    }

    std::vector<s32> meshVertices(meshVertexLump.length / sizeof(s32));
    for (usize i = 0; i < meshVertices.size(); ++i)
        meshVertices[i] = readI32(bytes, meshVertexLump.offset + i * sizeof(s32));

    std::vector<Face> faces(faceLump.length / kFaceSize);
    for (usize i = 0; i < faces.size(); ++i)
    {
        const usize offset = faceLump.offset + i * kFaceSize;
        Face& face = faces[i];
        face.texture = readI32(bytes, offset + 0);
        face.type = readI32(bytes, offset + 8);
        face.firstVertex = readI32(bytes, offset + 12);
        face.vertexCount = readI32(bytes, offset + 16);
        face.firstMeshVertex = readI32(bytes, offset + 20);
        face.meshVertexCount = readI32(bytes, offset + 24);
        face.lightmap = readI32(bytes, offset + 28);
        face.patchWidth = readI32(bytes, offset + 96);
        face.patchHeight = readI32(bytes, offset + 100);
    }

    usize firstFace = 0;
    usize faceCount = faces.size();
    const Lump& modelLump = lumps[LumpModels];
    if (mWorldspawnOnly && modelLump.length >= kModelSize)
    {
        if (modelLump.length % kModelSize != 0)
        {
            Log::error("BSPImporter: '%s' has a misaligned model lump", filename.c_str());
            return false;
        }
        const s32 modelFirstFace = readI32(bytes, modelLump.offset + 24);
        const s32 modelFaceCount = readI32(bytes, modelLump.offset + 28);
        if (modelFirstFace < 0 || modelFaceCount < 0 ||
            static_cast<u64>(modelFirstFace) + modelFaceCount > faces.size())
        {
            Log::error("BSPImporter: '%s' has an invalid worldspawn model", filename.c_str());
            return false;
        }
        firstFace = static_cast<usize>(modelFirstFace);
        faceCount = static_cast<usize>(modelFaceCount);
    }

    std::vector<FaceGroup> groups;
    std::unordered_map<u64, usize> groupLookup;
    for (usize i = firstFace; i < firstFace + faceCount; ++i)
    {
        const Face& face = faces[i];
        if (face.type != kFacePolygon && face.type != kFacePatch && face.type != kFaceMesh)
            continue;
        const u64 textureKey = static_cast<u32>(face.texture);
        const u64 lightmapKey = static_cast<u32>(face.lightmap);
        const u64 key = textureKey | (lightmapKey << 32);
        auto found = groupLookup.find(key);
        if (found == groupLookup.end())
        {
            const usize group = groups.size();
            groupLookup[key] = group;
            groups.push_back(FaceGroup());
            groups.back().texture = face.texture;
            groups.back().lightmap = face.lightmap;
            found = groupLookup.find(key);
        }
        groups[found->second].faces.push_back(i);
    }

    MeshData& imported = importedMap.mesh;
    for (const FaceGroup& group : groups)
    {
        const s32 lightmapPage =
            group.lightmap >= 0 && static_cast<usize>(group.lightmap) < importedMap.lightmaps.size()
                ? group.lightmap
                : -1;
        const u32 materialSlot = static_cast<u32>(imported.materials.size());
        const std::string shaderName =
            group.texture >= 0 && static_cast<usize>(group.texture) < textureNames.size() &&
                    !textureNames[static_cast<usize>(group.texture)].empty()
                ? textureNames[static_cast<usize>(group.texture)]
                : "bsp_surface_" + std::to_string(materialSlot);
        const auto shaderFound = shaders.find(lower(shaderName));
        const ShaderInfo* shader = shaderFound != shaders.end() ? &shaderFound->second : nullptr;
        if (shader && (shader->sky || shader->fog || shader->noDraw))
            continue;

        std::vector<std::string> albedoFrames;
        if (shader && !shader->animationFrames.empty())
        {
            albedoFrames.reserve(shader->animationFrames.size());
            for (const std::string& frame : shader->animationFrames)
                albedoFrames.push_back(resolveTexture(files, filename, frame));
        }
        else
        {
            const std::string image = shader && !shader->image.empty() ? shader->image : shaderName;
            albedoFrames.push_back(resolveTexture(files, filename, image));
        }

        std::string materialName = shaderName;
        if (lightmapPage >= 0)
            materialName += "#lm" + std::to_string(lightmapPage);
        Material material;
        material.name = materialName;
        material.nameHash = hashName(materialName);
        material.flags |= MaterialLit;
        if (lightmapPage >= 0)
            material.flags &= ~MaterialReceiveShadow;
        if (shader)
        {
            if (shader->twoSided || shader->transparent)
            {
                material.flags |= MaterialTwoSided;
                material.cull = CullMode::None;
            }
            if (shader->alphaTest)
                material.flags |= MaterialAlphaTest;
            if (shader->transparent)
            {
                material.flags |= MaterialNoDepthWrite;
                material.blend = shader->additive ? BlendMode::Additive : BlendMode::Alpha;
            }
            if (shader->scroll != Math::vec2(0.0f) || shader->rotate != 0.0f)
            {
                material.flags |= MaterialAnimated;
                material.params.uvAnim =
                    Math::vec4(shader->scroll, Math::radians(shader->rotate), 0.0f);
            }
            if (albedoFrames.size() > 1 && shader->animationFPS > 0.0f)
            {
                material.flags |= MaterialAnimated;
                material.params.sequence = Math::vec4(static_cast<f32>(albedoFrames.size()),
                                                     shader->animationFPS, 1.0f, 0.0f);
                material.textures[SlotAlbedo].source = TextureSource::Sequence;
                material.textures[SlotAlbedo].layers = static_cast<u16>(
                    std::min<usize>(albedoFrames.size(), static_cast<usize>(0xFFFFu)));
            }
        }
        imported.materials.push_back(material);
        imported.materialTextureFiles.push_back(albedoFrames.empty() ? std::string()
                                                                     : albedoFrames.front());

        BSPMaterialSource source;
        source.materialSlot = materialSlot;
        source.lightmapPage = lightmapPage;
        source.shaderName = shaderName;
        source.albedoFrames = std::move(albedoFrames);
        source.animationFPS = shader ? shader->animationFPS : 0.0f;
        importedMap.materialSources.push_back(std::move(source));

        SubMesh submesh;
        submesh.indexOffset = static_cast<u32>(imported.indices.size());
        submesh.materialSlot = materialSlot;
        submesh.lightmapPage = lightmapPage >= 0 ? static_cast<u32>(lightmapPage) : 0;

        for (usize faceIndex : group.faces)
        {
            const Face& face = faces[faceIndex];
            if (face.type == kFacePatch)
                appendPatch(face, vertices, mPatchTessellation, imported);
            else
                appendIndexedFace(face, vertices, meshVertices, imported);
        }

        submesh.indexCount = static_cast<u32>(imported.indices.size()) - submesh.indexOffset;
        if (submesh.indexCount > 0)
            imported.submeshes.push_back(submesh);
        else
        {
            imported.materials.pop_back();
            imported.materialTextureFiles.pop_back();
            importedMap.materialSources.pop_back();
        }
    }

    if (imported.positions.empty() || imported.indices.empty() || imported.submeshes.empty())
    {
        Log::error("BSPImporter: '%s' contains no supported world geometry", filename.c_str());
        return false;
    }

    computeBounds(imported);
    Log::info("BSPImporter: '%s' imported %zu vertices, %zu triangles and %zu surfaces",
              filename.c_str(), imported.positions.size(), imported.indices.size() / 3,
              imported.submeshes.size());
    result = std::move(importedMap);
    return true;
}

} // namespace Radion
