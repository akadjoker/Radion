#include "PCH.h"
#include "ObjExporter.h"

#include "FileSystem.h"
#include "Log.h"
#include "Material.h"
#include "Mesh.h"

#include <sstream>

using namespace Radion;

bool ObjExporter::save(const MeshData& mesh, const std::string& path)
{
    if (mesh.positions.empty() || mesh.indices.empty())
        return false;

    const std::string baseName = FileSystem::withoutExtension(FileSystem::fileName(path));
    const std::string mtlName = baseName + ".mtl";
    const std::string mtlPath = FileSystem::directoryOf(path).empty()
                                    ? mtlName
                                    : FileSystem::join(FileSystem::directoryOf(path), mtlName);

    std::ostringstream obj;
    obj << "mtllib " << mtlName << "\n";

    for (const glm::vec3& position : mesh.positions)
        obj << "v " << position.x << " " << position.y << " " << position.z << "\n";

    const bool hasUVs = mesh.uvs.size() == mesh.positions.size();
    if (hasUVs)
    {
        for (const glm::vec2& uv : mesh.uvs)
            obj << "vt " << uv.x << " " << uv.y << "\n";
    }

    const bool hasNormals = mesh.normals.size() == mesh.positions.size();
    if (hasNormals)
    {
        for (const glm::vec3& normal : mesh.normals)
            obj << "vn " << normal.x << " " << normal.y << " " << normal.z << "\n";
    }

    auto writeFaceIndex = [&](std::ostringstream& out, u32 index)
    {
        const u32 vertex = index + 1;
        out << vertex;
        if (hasUVs || hasNormals)
        {
            out << "/" << (hasUVs ? std::to_string(vertex) : std::string());
            if (hasNormals)
                out << "/" << vertex;
        }
    };

    if (mesh.submeshes.empty())
    {
        obj << "g mesh\n";
        for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
        {
            obj << "f ";
            writeFaceIndex(obj, mesh.indices[i]);
            obj << " ";
            writeFaceIndex(obj, mesh.indices[i + 1]);
            obj << " ";
            writeFaceIndex(obj, mesh.indices[i + 2]);
            obj << "\n";
        }
    }
    else
    {
        for (usize s = 0; s < mesh.submeshes.size(); ++s)
        {
            const SubMesh& sub = mesh.submeshes[s];
            obj << "g submesh" << s << "\n";
            const std::string materialName = sub.materialSlot < mesh.materials.size() &&
                                                     !mesh.materials[sub.materialSlot].name.empty()
                                                 ? mesh.materials[sub.materialSlot].name
                                                 : "material" + std::to_string(sub.materialSlot);
            obj << "usemtl " << materialName << "\n";

            const u32 end = sub.indexOffset + sub.indexCount;
            for (u32 i = sub.indexOffset; i + 2 < end; i += 3)
            {
                obj << "f ";
                writeFaceIndex(obj, mesh.indices[i]);
                obj << " ";
                writeFaceIndex(obj, mesh.indices[i + 1]);
                obj << " ";
                writeFaceIndex(obj, mesh.indices[i + 2]);
                obj << "\n";
            }
        }
    }

    FileSystem& files = FileSystem::getSingleton();
    if (!files.writeText(path, obj.str()))
    {
        Log::error("ObjExporter: failed to write '%s'", path.c_str());
        return false;
    }

    std::ostringstream mtl;
    if (mesh.materials.empty())
    {
        mtl << "newmtl material0\nKd 1 1 1\n";
    }
    else
    {
        for (usize i = 0; i < mesh.materials.size(); ++i)
        {
            const Material& material = mesh.materials[i];
            const std::string materialName =
                material.name.empty() ? "material" + std::to_string(i) : material.name;
            mtl << "newmtl " << materialName << "\n";
            mtl << "Kd " << material.params.baseColor.r << " " << material.params.baseColor.g
                << " " << material.params.baseColor.b << "\n";
            if (material.params.baseColor.a < 1.0f)
                mtl << "d " << material.params.baseColor.a << "\n";
            const std::string& albedoFile = material.textures[SlotAlbedo].file;
            if (!albedoFile.empty())
                mtl << "map_Kd " << albedoFile << "\n";
            mtl << "\n";
        }
    }

    if (!files.writeText(mtlPath, mtl.str()))
    {
        Log::error("ObjExporter: failed to write '%s'", mtlPath.c_str());
        return false;
    }

    return true;
}
