#include "PCH.h"

#include "GltfExporter.h"
#include "Mesh.h"

#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

#include "FileSystem.h"

#include <cmath>
#include <cstdio>
#include <fstream>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "GltfExportTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

// Two quads (four triangles) as two submeshes with different materials: the
// smallest mesh that exercises primitives, materials and shared vertex data.
MeshData twoParts()
{
    MeshData mesh;
    mesh.positions = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                      {2, 0, 0}, {3, 0, 0}, {3, 1, 0}, {2, 1, 0}};
    mesh.normals.assign(8, Math::vec3(0, 0, 1));
    mesh.uvs = {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}, {1, 0}, {1, 1}, {0, 1}};
    mesh.indices = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};

    Material red;
    red.name = "red";
    red.params.baseColor = Math::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    red.params.surface.x = 0.25f;
    red.params.surface.y = 0.75f;
    Material glass;
    glass.name = "glass";
    glass.params.baseColor = Math::vec4(0.5f, 0.7f, 0.9f, 0.4f);
    glass.flags |= MaterialTwoSided;
    // A third material no submesh uses must not reach the file.
    Material unused;
    unused.name = "unused";
    mesh.materials = {red, glass, unused};

    SubMesh first;
    first.indexOffset = 0;
    first.indexCount = 6;
    first.materialSlot = 0;
    SubMesh second;
    second.indexOffset = 6;
    second.indexCount = 6;
    second.materialSlot = 1;
    mesh.submeshes = {first, second};
    return mesh;
}

// Parses the bytes with an independent reader and validates the result.
cgltf_data* parse(const std::vector<unsigned char>& glb)
{
    cgltf_options options = {};
    cgltf_data* data = nullptr;
    if (cgltf_parse(&options, glb.data(), glb.size(), &data) != cgltf_result_success)
        return nullptr;
    if (cgltf_load_buffers(&options, data, nullptr) != cgltf_result_success ||
        cgltf_validate(data) != cgltf_result_success)
    {
        cgltf_free(data);
        return nullptr;
    }
    return data;
}

void testStructure()
{
    const MeshData mesh = twoParts();
    std::vector<unsigned char> glb;
    std::string error;
    CHECK(GltfExporter::build(mesh, "parts", glb, &error));
    CHECK(glb.size() % 4 == 0);

    cgltf_data* data = parse(glb);
    CHECK(data != nullptr);
    if (!data)
        return;

    CHECK(data->meshes_count == 1);
    CHECK(data->nodes_count == 1);
    CHECK(data->meshes[0].primitives_count == 2);
    CHECK(std::string(data->meshes[0].name) == "parts");

    // Only the two materials in use, renumbered in order of first use.
    CHECK(data->materials_count == 2);
    const cgltf_primitive& a = data->meshes[0].primitives[0];
    const cgltf_primitive& b = data->meshes[0].primitives[1];
    CHECK(a.indices->count == 6 && b.indices->count == 6);
    CHECK(a.material && std::string(a.material->name) == "red");
    CHECK(b.material && std::string(b.material->name) == "glass");

    const cgltf_pbr_metallic_roughness& pbr = a.material->pbr_metallic_roughness;
    CHECK(pbr.base_color_factor[0] == 1.0f && pbr.base_color_factor[1] == 0.0f);
    CHECK(pbr.roughness_factor == 0.25f);
    CHECK(pbr.metallic_factor == 0.75f);
    CHECK(b.material->double_sided);
    CHECK(b.material->alpha_mode == cgltf_alpha_mode_blend);
    CHECK(!a.material->double_sided);
    CHECK(a.material->alpha_mode == cgltf_alpha_mode_opaque);

    // The second primitive reads the second half of the shared index buffer.
    cgltf_size index = cgltf_accessor_read_index(b.indices, 0);
    CHECK(index == 4);

    // Attributes are there and shared.
    const cgltf_accessor* position = nullptr;
    const cgltf_accessor* normal = nullptr;
    const cgltf_accessor* uv = nullptr;
    for (cgltf_size i = 0; i < a.attributes_count; ++i)
    {
        if (a.attributes[i].type == cgltf_attribute_type_position)
            position = a.attributes[i].data;
        if (a.attributes[i].type == cgltf_attribute_type_normal)
            normal = a.attributes[i].data;
        if (a.attributes[i].type == cgltf_attribute_type_texcoord)
            uv = a.attributes[i].data;
    }
    CHECK(position && position->count == 8);
    CHECK(normal && normal->count == 8);
    CHECK(uv && uv->count == 8);
    CHECK(position && position->has_min && position->has_max);
    CHECK(position && position->max[0] == 3.0f && position->min[0] == 0.0f);

    float p[3] = {0, 0, 0};
    CHECK(cgltf_accessor_read_float(position, 6, p, 3));
    CHECK(p[0] == 3.0f && p[1] == 1.0f && p[2] == 0.0f);

    cgltf_free(data);
}

void testNoSubmeshesIsOnePrimitive()
{
    MeshData mesh = twoParts();
    mesh.submeshes.clear();
    mesh.materials.clear();
    mesh.normals.clear();
    mesh.uvs.clear();

    std::vector<unsigned char> glb;
    CHECK(GltfExporter::build(mesh, "", glb));
    cgltf_data* data = parse(glb);
    CHECK(data != nullptr);
    if (!data)
        return;
    CHECK(data->meshes[0].primitives_count == 1);
    CHECK(data->meshes[0].primitives[0].indices->count == 12);
    CHECK(data->materials_count == 0);
    CHECK(data->meshes[0].primitives[0].material == nullptr);
    cgltf_free(data);
}

void testRejectsBadMeshes()
{
    std::vector<unsigned char> glb;
    std::string error;

    MeshData empty;
    CHECK(!GltfExporter::build(empty, "x", glb, &error));
    CHECK(!error.empty());

    MeshData outOfRange = twoParts();
    outOfRange.indices[3] = 99;
    CHECK(!GltfExporter::build(outOfRange, "x", glb, &error));

    MeshData reaches = twoParts();
    reaches.submeshes[1].indexCount = 60;
    CHECK(!GltfExporter::build(reaches, "x", glb, &error));
}

void testTrailingIndicesAreDropped()
{
    // A submesh whose count is not a multiple of three would make an invalid
    // file; the loose indices are dropped.
    MeshData mesh = twoParts();
    mesh.submeshes[0].indexCount = 5;

    std::vector<unsigned char> glb;
    CHECK(GltfExporter::build(mesh, "x", glb));
    cgltf_data* data = parse(glb);
    CHECK(data != nullptr);
    if (data)
    {
        CHECK(data->meshes[0].primitives[0].indices->count == 3);
        cgltf_free(data);
    }
}

// A valid 1x1 PNG (an opaque red pixel).
const unsigned char kPng[] = {
    0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x01, 0x08, 0x02, 0x00, 0x00, 0x00, 0x90, 0x77, 0x53, 0xDE, 0x00, 0x00, 0x00, 0x0C, 0x49,
    0x44, 0x41, 0x54, 0x08, 0xD7, 0x63, 0xF8, 0xCF, 0xC0, 0x00, 0x00, 0x03, 0x01, 0x01, 0x00, 0x18, 0xDD, 0x8D, 0xB0,
    0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82};

std::string writeTempFile(const char* name, const unsigned char* bytes, size_t size)
{
    const std::string path = std::string("/tmp/") + name;
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file.write(reinterpret_cast<const char*>(bytes), static_cast<std::streamsize>(size));
    return path;
}

void testTexturesAndVertexColors()
{
    const std::string png = writeTempFile("radion_gltf_test.png", kPng, sizeof(kPng));
    const unsigned char notAnImage[] = {'h', 'e', 'l', 'l', 'o', ' ', 'w', 'o', 'r', 'l', 'd'};
    const std::string junk = writeTempFile("radion_gltf_test.bin", notAnImage, sizeof(notAnImage));

    MeshData mesh = twoParts();
    // Both parts point at the same image: it must be embedded once.
    for (usize i = 0; i < 2; ++i)
        mesh.materials[i].textures[SlotAlbedo].file = png;
    mesh.materials[0].textures[SlotNormal].file = png;
    mesh.materials[1].textures[SlotEmissive].file = junk;
    mesh.materials[1].textures[SlotSurface].file = "/tmp/radion_no_such_texture.png";
    mesh.colors.assign(8, 0xFF0000FFu); // bytes r=255 g=0 b=0 a=255
    mesh.colors[3] = 0x80402010u;

    std::vector<unsigned char> glb;
    std::vector<std::string> warnings;
    std::string error;
    CHECK(GltfExporter::build(mesh, "tex", glb, &error, &warnings));
    CHECK(warnings.size() == 2);

    cgltf_data* data = parse(glb);
    CHECK(data != nullptr);
    if (!data)
        return;
    CHECK(data->images_count == 1);
    CHECK(data->textures_count == 1);
    CHECK(data->images_count == 1 && std::string(data->images[0].mime_type) == "image/png");
    CHECK(data->images_count == 1 && data->images[0].buffer_view != nullptr &&
          data->images[0].buffer_view->size == sizeof(kPng));

    const cgltf_primitive& a = data->meshes[0].primitives[0];
    const cgltf_primitive& b = data->meshes[0].primitives[1];
    CHECK(a.material->pbr_metallic_roughness.base_color_texture.texture != nullptr);
    CHECK(a.material->normal_texture.texture != nullptr);
    CHECK(b.material->pbr_metallic_roughness.base_color_texture.texture ==
          a.material->pbr_metallic_roughness.base_color_texture.texture);
    // The unreadable and the non-image files left the material without those maps.
    CHECK(b.material->emissive_texture.texture == nullptr);
    CHECK(b.material->pbr_metallic_roughness.metallic_roughness_texture.texture == nullptr);

    const cgltf_accessor* color = nullptr;
    for (cgltf_size i = 0; i < a.attributes_count; ++i)
    {
        if (a.attributes[i].type == cgltf_attribute_type_color)
            color = a.attributes[i].data;
    }
    CHECK(color && color->count == 8 && color->normalized);
    float rgba[4] = {0, 0, 0, 0};
    CHECK(color && cgltf_accessor_read_float(color, 0, rgba, 4));
    CHECK(rgba[0] == 1.0f && rgba[1] == 0.0f && rgba[2] == 0.0f && rgba[3] == 1.0f);
    CHECK(color && cgltf_accessor_read_float(color, 3, rgba, 4));
    CHECK(std::fabs(rgba[0] - 16.0f / 255.0f) < 1e-4f && std::fabs(rgba[3] - 128.0f / 255.0f) < 1e-4f);
    cgltf_free(data);

    std::remove(png.c_str());
    std::remove(junk.c_str());
}

} // namespace

int main()
{
    testStructure();
    testNoSubmeshesIsOnePrimitive();
    testRejectsBadMeshes();
    testTrailingIndicesAreDropped();
    testTexturesAndVertexColors();

    if (gFailures)
        std::fprintf(stderr, "%d gltf export test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
