#include "PCH.h"

#include "RadionMeshImporter.h"

#include "ByteArray.h"
#include "FileSystem.h"
#include "Hash.h"
#include "RadionFormat.h"
#include "miniz.h"

namespace Radion
{

namespace
{

bool readVec2(AssetFormat::Reader& reader, Math::vec2& value)
{
    return reader.readF32(value.x) && reader.readF32(value.y);
}

bool readVec3(AssetFormat::Reader& reader, Math::vec3& value)
{
    return reader.readF32(value.x) && reader.readF32(value.y) && reader.readF32(value.z);
}

bool readVec4(AssetFormat::Reader& reader, Math::vec4& value)
{
    return reader.readF32(value.x) && reader.readF32(value.y) && reader.readF32(value.z) &&
           reader.readF32(value.w);
}

void writeVec2(AssetFormat::Writer& writer, const Math::vec2& value)
{
    writer.writeF32(value.x);
    writer.writeF32(value.y);
}

void writeVec3(AssetFormat::Writer& writer, const Math::vec3& value)
{
    writer.writeF32(value.x);
    writer.writeF32(value.y);
    writer.writeF32(value.z);
}

void writeVec4(AssetFormat::Writer& writer, const Math::vec4& value)
{
    writer.writeF32(value.x);
    writer.writeF32(value.y);
    writer.writeF32(value.z);
    writer.writeF32(value.w);
}

// True only if `count` records of `recordSize` bytes each actually fit in
// what is left of the current chunk - checked, not `count * recordSize <=
// remaining()` directly, since that multiplication can itself overflow for a
// large enough declared count and wrap around to something that looks small.
// A small file can otherwise claim a count in the hundreds of millions and
// have every vector resize() to match before the first read ever fails,
// which is gigabytes of allocation on nothing but a corrupted or hostile
// four-byte count.
bool fitsChunk(const AssetFormat::Reader& reader, u32 count, u64 recordSize)
{
    const u64 remaining = reader.remaining();
    if (recordSize != 0 && count > remaining / recordSize)
        return false;
    return true;
}

} // namespace

bool RadionMeshImporter::supports(const char* extension) const
{
    return extension && (std::strcmp(extension, "rmesh") == 0 || std::strcmp(extension, "rstm") == 0);
}

static bool readStaticMeshHeader(AssetFormat::Reader& reader,
                                 u32& vertexCount, u32& indexCount, u32& subMeshCount,
                                 Math::vec3& boundsMin, Math::vec3& boundsMax,
                                 u32& uncompressedSize, u32& compressedSize)
{
    if (!reader.readU32(vertexCount) ||
        !reader.readU32(indexCount) ||
        !reader.readU32(subMeshCount) ||
        !readVec3(reader, boundsMin) ||
        !readVec3(reader, boundsMax) ||
        !reader.readU32(uncompressedSize) ||
        !reader.readU32(compressedSize))
        return false;
    return true;
}

static bool readStaticMeshPayload(AssetFormat::Reader& reader, MeshData& mesh,
                                  u32 vertexCount, u32 indexCount, u32 subMeshCount,
                                  u32 flags)
{
    const bool index16 = (flags & AssetFormat::StaticMeshIndex16) != 0;
    const u32 attribFlags = flags >> AssetFormat::StaticMeshAttribShift;
    const bool hasNormal = (attribFlags & AssetFormat::HasNormal) != 0;
    const bool hasTangent = (attribFlags & AssetFormat::HasTangent) != 0;
    const bool hasUV = (attribFlags & AssetFormat::HasUV) != 0;

    // Submesh table: 3*u32 + 2*vec3 = 36 bytes
    if (subMeshCount > 65536 ||
        !fitsChunk(reader, subMeshCount, 36))
        return false;
    mesh.submeshes.resize(subMeshCount);
    for (SubMesh& submesh : mesh.submeshes)
    {
        if (!reader.readU32(submesh.indexOffset) ||
            !reader.readU32(submesh.indexCount) ||
            !reader.readU32(submesh.materialSlot) ||
            !readVec3(reader, submesh.bounds.min) ||
            !readVec3(reader, submesh.bounds.max))
            return false;
    }

    // Vertex buffer - position always, the rest only if attribFlags says so.
    // Empty MeshData array means "this mesh does not have it", same
    // convention the rest of the engine already uses for uvs2.
    const u32 stride = AssetFormat::vertexAttribStride(attribFlags);
    if (vertexCount == 0 || vertexCount > 100000000 ||
        !fitsChunk(reader, vertexCount, stride))
        return false;
    mesh.positions.resize(vertexCount);
    if (hasNormal)
        mesh.normals.resize(vertexCount);
    if (hasTangent)
        mesh.tangents.resize(vertexCount);
    if (hasUV)
        mesh.uvs.resize(vertexCount);
    for (u32 i = 0; i < vertexCount; ++i)
    {
        if (!readVec3(reader, mesh.positions[i]))
            return false;
        if (hasNormal && !readVec3(reader, mesh.normals[i]))
            return false;
        if (hasTangent && !readVec4(reader, mesh.tangents[i]))
            return false;
        if (hasUV && !readVec2(reader, mesh.uvs[i]))
            return false;
    }

    // Index buffer
    if (indexCount == 0 || indexCount > 300000000 ||
        !fitsChunk(reader, indexCount, index16 ? 2 : 4))
        return false;
    mesh.indices.resize(indexCount);
    for (u32& index : mesh.indices)
    {
        if (index16)
        {
            u16 value = 0;
            if (!reader.readU16(value))
                return false;
            index = value;
        }
        else
        {
            if (!reader.readU32(index))
                return false;
        }
    }

    return true;
}

static bool importStaticMesh(const std::string& filename, ByteArray& data, MeshData& mesh)
{
    AssetFormat::Reader reader(data);

    u16 versionMajor = 0;
    u16 versionMinor = 0;
    u32 flags = 0;
    if (!reader.readU16(versionMajor) || !reader.readU16(versionMinor) || !reader.readU32(flags))
    {
        Log::error("RadionMeshImporter: '%s': failed to read static mesh header", filename.c_str());
        return false;
    }

    if (versionMajor != AssetFormat::VersionMajor)
    {
        Log::error("RadionMeshImporter: '%s': unsupported static mesh version", filename.c_str());
        return false;
    }
    if ((flags & AssetFormat::LittleEndian) == 0)
    {
        Log::error("RadionMeshImporter: '%s': big-endian static meshes are not supported", filename.c_str());
        return false;
    }

    u32 vertexCount = 0;
    u32 indexCount = 0;
    u32 subMeshCount = 0;
    Math::vec3 boundsMin, boundsMax;
    u32 uncompressedSize = 0;
    u32 compressedSize = 0;
    if (!readStaticMeshHeader(reader, vertexCount, indexCount, subMeshCount,
                              boundsMin, boundsMax, uncompressedSize, compressedSize))
    {
        Log::error("RadionMeshImporter: '%s': failed to read static mesh header", filename.c_str());
        return false;
    }

    mesh.bounds.min = boundsMin;
    mesh.bounds.max = boundsMax;

    const bool compressed = (flags & AssetFormat::StaticMeshCompressed) != 0;
    bool payloadOk = false;

    if (compressed)
    {
        if (compressedSize == 0 || uncompressedSize == 0 || compressedSize > reader.remaining())
        {
            Log::error("RadionMeshImporter: '%s': invalid compressed static mesh sizes", filename.c_str());
            return false;
        }

        std::vector<u8> compressed(compressedSize);
        if (!reader.bytes(compressed.data(), compressedSize))
        {
            Log::error("RadionMeshImporter: '%s': failed to read compressed payload", filename.c_str());
            return false;
        }

        std::vector<u8> uncompressed(uncompressedSize);
        unsigned long destLen = uncompressedSize;
        int status = mz_uncompress(uncompressed.data(), &destLen,
                                   compressed.data(), compressedSize);
        if (status != MZ_OK || destLen != uncompressedSize)
        {
            Log::error("RadionMeshImporter: '%s': failed to decompress static mesh (miniz %d)",
                       filename.c_str(), status);
            return false;
        }

        ByteArray payload(uncompressedSize);
        std::memcpy(payload.data(), uncompressed.data(), uncompressedSize);
        AssetFormat::Reader payloadReader(payload);
        payloadOk = readStaticMeshPayload(payloadReader, mesh, vertexCount, indexCount, subMeshCount, flags);
    }
    else
    {
        payloadOk = readStaticMeshPayload(reader, mesh, vertexCount, indexCount, subMeshCount, flags);
    }

    if (!payloadOk)
    {
        Log::error("RadionMeshImporter: '%s': failed to read static mesh payload", filename.c_str());
        return false;
    }

    u32 materialCount = 0;
    for (const SubMesh& submesh : mesh.submeshes)
        materialCount = Math::max(materialCount, submesh.materialSlot + 1);

    mesh.materials.resize(materialCount);
    for (u32 i = 0; i < materialCount; ++i)
    {
        Material& material = mesh.materials[i];
        material.flags = MaterialCastShadow | MaterialReceiveShadow;
        material.params.baseColor = Math::vec4(0.75f, 0.78f, 0.82f, 1.0f);
        material.name = "Material_" + std::to_string(i);
        material.nameHash = hashName(material.name.c_str());
    }

    return true;
}

bool RadionMeshImporter::import(const std::string& filename, ByteArray& data, FileSystem&,
                                MeshData& mesh)
{
    AssetFormat::Reader reader(data);

    u32 magic = 0;
    if (!reader.readU32(magic))
    {
        Log::error("RadionMeshImporter: '%s': file is smaller than the Radion header", filename.c_str());
        return false;
    }

    if (magic == AssetFormat::StaticMeshMagic)
    {
        return importStaticMesh(filename, data, mesh);
    }

    if (magic != AssetFormat::MeshMagic)
    {
        Log::error("RadionMeshImporter: '%s': wrong Radion asset magic", filename.c_str());
        return false;
    }

    // Continua como RMSH chunkado
    u32 version = 0;
    u32 flags = 0;
    if (!reader.readU32(version) || !reader.readU32(flags))
    {
        Log::error("RadionMeshImporter: '%s': file is smaller than the Radion header", filename.c_str());
        return false;
    }
    if ((version >> 16) != AssetFormat::VersionMajor)
    {
        Log::error("RadionMeshImporter: '%s': unsupported Radion asset major version", filename.c_str());
        return false;
    }
    if ((flags & AssetFormat::LittleEndian) == 0)
    {
        Log::error("RadionMeshImporter: '%s': big-endian Radion assets are not supported", filename.c_str());
        return false;
    }

    std::vector<std::string> materialSlots;
    bool vertices = false;
    bool indices = false;
    AssetFormat::ChunkHeader chunk;
    constexpr size_t chunkHeaderSize = sizeof(u32) + sizeof(u64);
    while (reader.remaining() >= chunkHeaderSize && reader.next(chunk))
    {
        if (!reader.enter(chunk))
            return false;
        if (chunk.id == AssetFormat::MaterialSlots)
        {
            u32 count = 0;
            if (!reader.readU32(count) || count > 65536)
                return false;
            materialSlots.resize(count);
            for (std::string& slot : materialSlots)
                if (!reader.string(slot))
                    return false;
        }
        else if (chunk.id == AssetFormat::VertexBuffer)
        {
            u32 attribFlags = 0;
            u32 count = 0;
            if (!reader.readU32(attribFlags) || !reader.readU32(count) || count == 0 ||
                count > 100000000)
                return false;
            const bool hasNormal = (attribFlags & AssetFormat::HasNormal) != 0;
            const bool hasTangent = (attribFlags & AssetFormat::HasTangent) != 0;
            const bool hasUV = (attribFlags & AssetFormat::HasUV) != 0;
            const bool hasColor = (attribFlags & AssetFormat::HasColor) != 0;
            // The nominal 100M cap alone is still a multi-gigabyte allocation
            // attempt on a file that need not contain a single byte of it -
            // fitsChunk() against the real per-vertex stride is what actually
            // rejects a corrupt/hostile count.
            if (!fitsChunk(reader, count, AssetFormat::vertexAttribStride(attribFlags)))
                return false;
            mesh.positions.resize(count);
            if (hasNormal)
                mesh.normals.resize(count);
            if (hasTangent)
                mesh.tangents.resize(count);
            if (hasUV)
                mesh.uvs.resize(count);
            if (hasColor)
                mesh.colors.resize(count);
            for (u32 i = 0; i < count; ++i)
            {
                if (!readVec3(reader, mesh.positions[i]))
                    return false;
                if (hasNormal && !readVec3(reader, mesh.normals[i]))
                    return false;
                if (hasTangent && !readVec4(reader, mesh.tangents[i]))
                    return false;
                if (hasUV && !readVec2(reader, mesh.uvs[i]))
                    return false;
                if (hasColor && !reader.readU32(mesh.colors[i]))
                    return false;
            }
            vertices = true;
        }
        else if (chunk.id == AssetFormat::IndexBuffer)
        {
            u32 count = 0;
            if (!reader.readU32(count) || count == 0 || count > 300000000 ||
                !fitsChunk(reader, count, sizeof(u32)))
                return false;
            mesh.indices.resize(count);
            for (u32& index : mesh.indices)
                if (!reader.readU32(index))
                    return false;
            indices = true;
        }
        else if (chunk.id == AssetFormat::SubMeshes)
        {
            u32 count = 0;
            // 4*u32 + 2*vec3. The fourth integer is the lightmap page.
            if (!reader.readU32(count) || count > 65536 || !fitsChunk(reader, count, 40))
                return false;
            mesh.submeshes.resize(count);
            for (SubMesh& submesh : mesh.submeshes)
            {
                if (!reader.readU32(submesh.indexOffset) || !reader.readU32(submesh.indexCount) ||
                    !reader.readU32(submesh.materialSlot) || !reader.readU32(submesh.lightmapPage) ||
                    !readVec3(reader, submesh.bounds.min) || !readVec3(reader, submesh.bounds.max))
                    return false;
            }
        }
        else if (chunk.id == AssetFormat::LightmapUV)
        {
            u32 count = 0;
            if (!reader.readU32(count) || count > 100000000 || !fitsChunk(reader, count, 8))
                return false;
            mesh.uvs2.resize(count);
            for (Math::vec2& uv : mesh.uvs2)
                if (!readVec2(reader, uv))
                    return false;
        }
        else if (chunk.id == AssetFormat::Skin)
        {
            u32 count = 0;
            // 4 joint bytes + vec4 weights.
            if (!reader.readU32(count) || count > 100000000 || !fitsChunk(reader, count, 20))
                return false;
            mesh.skin.resize(count);
            for (MeshSkinVertex& skin : mesh.skin)
                if (!reader.bytes(skin.joints, 4) || !readVec4(reader, skin.weights))
                    return false;
        }
        reader.leave();
    }

    if (!vertices || !indices || (!mesh.skin.empty() && mesh.skin.size() != mesh.positions.size()))
        return false;
    for (u32 index : mesh.indices)
        if (index >= mesh.positions.size())
            return false;
    u32 materialCount = static_cast<u32>(materialSlots.size());
    for (const SubMesh& submesh : mesh.submeshes)
        materialCount = Math::max(materialCount, submesh.materialSlot + 1);

    mesh.materials.resize(materialCount);
    for (u32 i = 0; i < materialCount; ++i)
    {
        Material& material = mesh.materials[i];
        material.name = i < materialSlots.size() ? materialSlots[i] : "FallbackMaterial";
        material.flags = MaterialCastShadow | MaterialReceiveShadow;
        if (!mesh.skin.empty())
            material.flags |= MaterialSkinned;
        material.params.baseColor = Math::vec4(0.75f, 0.78f, 0.82f, 1.0f);
        material.nameHash = hashName(material.name.c_str());
    }
    return true;
}

bool saveRadionMesh(const std::string& filename, const MeshData& mesh,
                    const std::string& skeletonFile)
{
    ByteArray data;
    AssetFormat::Writer writer(data);
    writer.header(AssetFormat::MeshMagic);

    u64 chunk = writer.beginChunk(AssetFormat::MaterialSlots);
    writer.writeU32(static_cast<u32>(mesh.materials.size()));
    for (const Material& material : mesh.materials)
        writer.string(material.name);
    writer.endChunk(chunk);

    {
        const bool hasNormal = mesh.normals.size() == mesh.positions.size();
        const bool hasTangent = mesh.tangents.size() == mesh.positions.size();
        const bool hasUV = mesh.uvs.size() == mesh.positions.size();
        // Unlike the offline exporters (which never had a real source of
        // per-vertex colour), this path can round-trip through the GPU -
        // exportMesh() reads back whatever MeshAttribs::color actually
        // holds, which is real data if anything ever painted it. Dropping
        // it unconditionally here the way the exporters do would lose it
        // silently.
        const bool hasColor = mesh.colors.size() == mesh.positions.size();
        u32 attribFlags = 0;
        if (hasNormal)
            attribFlags |= AssetFormat::HasNormal;
        if (hasTangent)
            attribFlags |= AssetFormat::HasTangent;
        if (hasUV)
            attribFlags |= AssetFormat::HasUV;
        if (hasColor)
            attribFlags |= AssetFormat::HasColor;

        chunk = writer.beginChunk(AssetFormat::VertexBuffer);
        writer.writeU32(attribFlags);
        writer.writeU32(static_cast<u32>(mesh.positions.size()));
        for (usize i = 0; i < mesh.positions.size(); ++i)
        {
            writeVec3(writer, mesh.positions[i]);
            if (hasNormal)
                writeVec3(writer, mesh.normals[i]);
            if (hasTangent)
                writeVec4(writer, mesh.tangents[i]);
            if (hasUV)
                writeVec2(writer, mesh.uvs[i]);
            if (hasColor)
                writer.writeU32(mesh.colors[i]);
        }
        writer.endChunk(chunk);
    }

    chunk = writer.beginChunk(AssetFormat::IndexBuffer);
    writer.writeU32(static_cast<u32>(mesh.indices.size()));
    for (u32 index : mesh.indices)
        writer.writeU32(index);
    writer.endChunk(chunk);

    chunk = writer.beginChunk(AssetFormat::SubMeshes);
    writer.writeU32(static_cast<u32>(mesh.submeshes.size()));
    for (const SubMesh& submesh : mesh.submeshes)
    {
        writer.writeU32(submesh.indexOffset);
        writer.writeU32(submesh.indexCount);
        writer.writeU32(submesh.materialSlot);
        writer.writeU32(submesh.lightmapPage);
        writeVec3(writer, submesh.bounds.min);
        writeVec3(writer, submesh.bounds.max);
    }
    writer.endChunk(chunk);

    if (mesh.uvs2.size() == mesh.positions.size())
    {
        chunk = writer.beginChunk(AssetFormat::LightmapUV);
        writer.writeU32(static_cast<u32>(mesh.uvs2.size()));
        for (const Math::vec2& uv : mesh.uvs2)
            writeVec2(writer, uv);
        writer.endChunk(chunk);
    }

    if (!mesh.skin.empty())
    {
        chunk = writer.beginChunk(AssetFormat::Skin);
        writer.writeU32(static_cast<u32>(mesh.skin.size()));
        for (const MeshSkinVertex& skin : mesh.skin)
        {
            writer.bytes(skin.joints, 4);
            writeVec4(writer, skin.weights);
        }
        writer.endChunk(chunk);
    }
    if (!skeletonFile.empty())
    {
        chunk = writer.beginChunk(AssetFormat::SkeletonLink);
        writer.string(skeletonFile);
        writer.endChunk(chunk);
    }
    return FileSystem::getSingleton().writeBinary(filename, data);
}

} // namespace Radion
