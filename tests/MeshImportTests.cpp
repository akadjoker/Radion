#include "PCH.h"

#include "AssetManager.h"
#include "DDSImage.h"
#include "FileSystem.h"
#include "GltfImporter.h"
#include "Material.h"
#include "Mesh.h"
#include "MeshLoader.h"

#include <cstdio>
#include <filesystem>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "MeshImportTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

void importPath(MeshLoader& loader, const std::string& path, const char* label)
{
    FileSystem& files = FileSystem::getSingleton();
    const usize slash = path.find_last_of('/');
    if (slash != std::string::npos)
        files.addSearchPath(path.substr(0, slash));

    MeshData mesh;
    if (!loader.load(path, mesh))
    {
        std::fprintf(stderr, "MeshImportTests: '%s' did not import\n", label);
        ++gFailures;
        return;
    }

    std::fprintf(stderr,
                 "  %s: %zu vertices, %zu indices, %zu submeshes, %zu materials, bounds "
                 "(%.2f %.2f %.2f)-(%.2f %.2f %.2f)\n",
                 label, mesh.positions.size(), mesh.indices.size(), mesh.submeshes.size(),
                 mesh.materials.size(), mesh.bounds.min.x, mesh.bounds.min.y, mesh.bounds.min.z,
                 mesh.bounds.max.x, mesh.bounds.max.y, mesh.bounds.max.z);

    CHECK(!mesh.positions.empty());
    CHECK(!mesh.indices.empty());
    CHECK(mesh.indices.size() % 3 == 0);
    CHECK(!mesh.submeshes.empty());

    for (u32 index : mesh.indices)
        if (index >= mesh.positions.size())
        {
            std::fprintf(stderr, "MeshImportTests: '%s' index %u out of %zu vertices\n", label,
                         index, mesh.positions.size());
            ++gFailures;
            break;
        }

    if (!mesh.normals.empty())
        CHECK(mesh.normals.size() == mesh.positions.size());
    if (!mesh.uvs.empty())
        CHECK(mesh.uvs.size() == mesh.positions.size());

    u32 covered = 0;
    for (const SubMesh& submesh : mesh.submeshes)
    {
        CHECK(submesh.indexOffset + submesh.indexCount <= mesh.indices.size());
        CHECK(submesh.materialSlot < mesh.materials.size() || mesh.materials.empty());
        covered += submesh.indexCount;
    }
    CHECK(covered == mesh.indices.size());

    CHECK(mesh.materialTextureFiles.size() == mesh.materials.size());

    u32 withAlbedo = 0;
    u32 missing = 0;
    for (const std::string& file : mesh.materialTextureFiles)
    {
        if (file.empty())
            continue;
        ++withAlbedo;
        if (!files.exists(file))
        {
            if (missing == 0)
                std::fprintf(stderr, "    first missing albedo: %s\n", file.c_str());
            ++missing;
        }
    }
    std::fprintf(stderr, "    albedo: %u of %zu materials, %u naming a file that is not there\n",
                 withAlbedo, mesh.materials.size(), missing);

    // Total surface area decides how many lightmap pages a fixed texel
    // density needs, which is the only thing that makes "how big an atlas"
    // answerable without guessing.
    f64 area = 0.0;
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[i + 0]];
        const Math::vec3& b = mesh.positions[mesh.indices[i + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[i + 2]];
        area += 0.5 * Math::length(Math::cross(b - a, c - a));
    }
    std::fprintf(stderr, "    surface area %.0f square units\n", area);
    for (u32 density = 2; density <= 8; density *= 2)
    {
        const f64 texels = area * density * density;
        std::fprintf(stderr, "      %u texels/unit -> %.1f M texels = %.0f pages of 2048\n",
                     density, texels / 1e6, texels / (2048.0 * 2048.0));
    }

    // The sidecar is what a reopened scene reads, so the paths have to survive
    // the round trip through it - not merely exist on the MeshData.
    const std::vector<Material> sidecar = AssetManager::getSingleton().materialsForSidecar(mesh);
    u32 bound = 0;
    for (const Material& material : sidecar)
        if (!material.textures[SlotAlbedo].file.empty())
            ++bound;
    std::fprintf(stderr, "    sidecar: %u of %zu materials name an albedo texture\n", bound,
                 sidecar.size());
    CHECK(bound == withAlbedo);

    // The three readings of SlotSurface are different packings of one slot -
    // a material claiming two of them renders as whichever the shader tests
    // for first, silently.
    u32 surfaceMapped = 0;
    for (const Material& material : sidecar)
    {
        const bool metallicRoughness = (material.flags & MaterialMetallicRoughnessMap) != 0;
        const bool specularGlossiness = (material.flags & MaterialSpecularGlossinessMap) != 0;
        CHECK(!(metallicRoughness && specularGlossiness));
        if (metallicRoughness || specularGlossiness)
        {
            ++surfaceMapped;
            CHECK(!material.textures[SlotSurface].file.empty());
        }
    }
    std::fprintf(stderr, "    surface: %u of %zu materials bind a packed surface map\n",
                 surfaceMapped, sidecar.size());
}

void importOne(MeshLoader& loader, const char* relative)
{
    const std::string path = std::string(RADION_TEST_ASSET_DIR) + "/" + relative;
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error))
    {
        std::fprintf(stderr, "  skipping optional import fixture: %s\n", relative);
        return;
    }
    importPath(loader, path, relative);
}

void testGltf()
{
    MeshLoader loader;
    loader.addImporter(new GltfImporter());

    importOne(loader, "models/DamagedHelmet/glTF/DamagedHelmet.gltf");
    importOne(loader, "models/lowpoly_bonfire/scene.gltf");
    importOne(loader, "models/Scene/scene.gltf");
    importOne(loader, "models/flightHelmet/flightHelmet.glb");

    // glTF is Y-up and this model is authored standing on the ground plane.
    // Its node rotations are what places the parts, so a mis-decoded
    // quaternion shows up here as a model lying over or sunk through Y=0
    // long before anyone opens the editor to look at it.
    MeshData helmet;
    const std::string helmetPath =
        std::string(RADION_TEST_ASSET_DIR) + "/models/flightHelmet/flightHelmet.glb";
    std::error_code error;
    if (!std::filesystem::is_regular_file(helmetPath, error))
        return;
    CHECK(loader.load(helmetPath, helmet));
    if (helmet.positions.empty())
        return;
    const Math::vec3 size = helmet.bounds.max - helmet.bounds.min;
    CHECK(std::abs(helmet.bounds.min.y) < size.y * 0.05f);
    CHECK(size.y > size.x && size.y > size.z);
}

} // namespace

// Any path given on the command line is imported and reported instead of the
// built-in set - for checking an asset that lives outside the repository.
int main(int argc, char** argv)
{
    if (argc > 1)
    {
        MeshLoader loader;
        loader.addImporter(new GltfImporter());
        for (int i = 1; i < argc; ++i)
        {
            const std::string path = argv[i];
            if (path.size() > 4 && path.compare(path.size() - 4, 4, ".dds") == 0)
            {
                ByteArray bytes = FileSystem::getSingleton().readBinary(path);
                DDSImage dds;
                if (!dds.loadFromMemory(bytes.data(), bytes.size()) || !dds.isValid())
                {
                    std::fprintf(stderr, "  %s: did not load\n", path.c_str());
                    ++gFailures;
                    continue;
                }
                std::fprintf(stderr, "  %s: %ux%u, %u mips, format %d\n", path.c_str(),
                             dds.width(), dds.height(), dds.mipCount(),
                             static_cast<int>(dds.format()));
                continue;
            }
            importPath(loader, path, argv[i]);
        }
        return gFailures == 0 ? 0 : 1;
    }

    testGltf();
    if (gFailures)
        std::fprintf(stderr, "%d mesh import test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
