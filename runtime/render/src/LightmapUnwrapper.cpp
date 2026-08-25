#include "PCH.h"

#include "LightmapUnwrapper.h"

#include "Log.h"
#include "xatlas.h"

#include <limits>

namespace Radion
{

namespace
{

struct ProgressContext
{
    LightmapUnwrapSettings::ProgressCallback callback = nullptr;
    void* userData = nullptr;
    bool cancelled = false;
};

bool xatlasProgress(xatlas::ProgressCategory category, int progress, void* userData)
{
    ProgressContext* context = static_cast<ProgressContext*>(userData);
    if (!context || !context->callback)
        return true;
    const bool keepGoing = context->callback(xatlas::StringForEnum(category),
                                             static_cast<u32>(glm::clamp(progress, 0, 100)),
                                             context->userData);
    if (!keepGoing)
        context->cancelled = true;
    return keepGoing;
}

} // namespace

bool LightmapUnwrapper::unwrap(const MeshData& input, MeshData& output,
                               const LightmapUnwrapSettings& settings,
                               LightmapUnwrapResult* atlasResult) const
{
    output.clear();
    if (atlasResult)
        *atlasResult = LightmapUnwrapResult();

    if (input.positions.empty() || input.indices.empty() || input.submeshes.empty())
    {
        Log::error("LightmapUnwrapper: input mesh is empty");
        return false;
    }
    if (!input.skin.empty())
    {
        Log::error("LightmapUnwrapper: skinned meshes are not supported");
        return false;
    }
    if (input.indices.size() % 3 != 0)
    {
        Log::error("LightmapUnwrapper: index count is not a multiple of three");
        return false;
    }

    const bool hasNormals = input.normals.size() == input.positions.size();
    const bool hasUvs = input.uvs.size() == input.positions.size();

    xatlas::Atlas* atlas = xatlas::Create();
    if (!atlas)
    {
        Log::error("LightmapUnwrapper: xatlas::Create failed");
        return false;
    }

    ProgressContext progress{settings.progress, settings.progressUserData, false};
    if (settings.progress)
        xatlas::SetProgressCallback(atlas, xatlasProgress, &progress);

    // The whole mesh goes in as ONE xatlas mesh, submeshes and all. Feeding
    // it one mesh per submesh stops charts ever crossing a submesh boundary,
    // which packs far more loose pieces and spills the atlas over several
    // pages for geometry that fits in one.
    xatlas::MeshDecl decl;
    decl.vertexCount = static_cast<u32>(input.positions.size());
    decl.vertexPositionData = input.positions.data();
    decl.vertexPositionStride = sizeof(glm::vec3);
    if (hasNormals)
    {
        decl.vertexNormalData = input.normals.data();
        decl.vertexNormalStride = sizeof(glm::vec3);
    }
    if (hasUvs)
    {
        decl.vertexUvData = input.uvs.data();
        decl.vertexUvStride = sizeof(glm::vec2);
    }
    decl.indexCount = static_cast<u32>(input.indices.size());
    decl.indexData = input.indices.data();
    decl.indexFormat = xatlas::IndexFormat::UInt32;

    const xatlas::AddMeshError error = xatlas::AddMesh(atlas, decl);
    if (error != xatlas::AddMeshError::Success)
    {
        Log::error("LightmapUnwrapper: xatlas AddMesh failed (%s)", xatlas::StringForEnum(error));
        xatlas::Destroy(atlas);
        return false;
    }

    xatlas::ChartOptions chartOptions;
    chartOptions.useInputMeshUvs = hasUvs;
    chartOptions.fixWinding = true;

    xatlas::PackOptions packOptions;
    packOptions.resolution = settings.resolution;
    packOptions.padding = settings.padding;
    packOptions.texelsPerUnit = settings.texelsPerUnit;
    packOptions.bilinear = true;
    packOptions.blockAlign = true;

    xatlas::Generate(atlas, chartOptions, packOptions);

    if (progress.cancelled)
    {
        Log::info("LightmapUnwrapper: cancelled");
        xatlas::Destroy(atlas);
        return false;
    }
    if (atlas->meshCount != 1 || atlas->width == 0 || atlas->height == 0 || atlas->atlasCount == 0)
    {
        Log::error("LightmapUnwrapper: xatlas produced an unusable atlas");
        xatlas::Destroy(atlas);
        return false;
    }

    const xatlas::Mesh& result = atlas->meshes[0];

    // Every consumer of this binds one lightmap texture. Refusing here beats
    // handing back a mesh whose later pages sample a texture that was never
    // written, which shows up as whole sections of the model going black.
    if (atlas->atlasCount != 1)
    {
        Log::error("LightmapUnwrapper: atlas needs %u pages - lower texelsPerUnit or raise "
                   "resolution until it fits one",
                   atlas->atlasCount);
        xatlas::Destroy(atlas);
        return false;
    }

    // xatlas renumbers and duplicates vertices but keeps the triangles in the
    // order they were given, so index j still belongs to triangle j and every
    // submesh's indexOffset/indexCount stays valid untouched. That only holds
    // while the counts match; if a future xatlas drops degenerate triangles
    // the submesh table would silently point at the wrong geometry, so it is
    // checked rather than assumed.
    if (result.indexCount != input.indices.size())
    {
        Log::error("LightmapUnwrapper: xatlas returned %u indices for %zu given - the submesh "
                   "table can no longer be trusted",
                   result.indexCount, input.indices.size());
        xatlas::Destroy(atlas);
        return false;
    }
    if (result.vertexCount > std::numeric_limits<u32>::max())
    {
        Log::error("LightmapUnwrapper: output mesh is too large");
        xatlas::Destroy(atlas);
        return false;
    }

    output.materials = input.materials;
    output.materialTextureFiles = input.materialTextureFiles;
    output.materialNormalFiles = input.materialNormalFiles;
    output.materialSurfaceFiles = input.materialSurfaceFiles;
    output.materialEmissiveFiles = input.materialEmissiveFiles;
    output.submeshes = input.submeshes;

    const usize vertexCount = result.vertexCount;
    output.positions.resize(vertexCount);
    output.uvs2.resize(vertexCount);
    if (hasNormals)
        output.normals.resize(vertexCount);
    if (hasUvs)
        output.uvs.resize(vertexCount);
    if (input.tangents.size() == input.positions.size())
        output.tangents.resize(vertexCount);
    if (input.colors.size() == input.positions.size())
        output.colors.resize(vertexCount);
    output.indices.resize(result.indexCount);

    const f32 width = static_cast<f32>(atlas->width);
    const f32 height = static_cast<f32>(atlas->height);

    for (u32 i = 0; i < result.indexCount; ++i)
    {
        const u32 index = result.indexArray[i];
        if (index >= result.vertexCount)
        {
            Log::error("LightmapUnwrapper: xatlas returned an out-of-range index");
            xatlas::Destroy(atlas);
            output.clear();
            return false;
        }
        const xatlas::Vertex& vertex = result.vertexArray[index];
        if (vertex.xref >= input.positions.size())
        {
            Log::error("LightmapUnwrapper: xatlas returned an out-of-range vertex reference");
            xatlas::Destroy(atlas);
            output.clear();
            return false;
        }

        output.indices[i] = index;
        output.positions[index] = input.positions[vertex.xref];
        output.uvs2[index] = glm::vec2(vertex.uv[0] / width, vertex.uv[1] / height);
        if (!output.normals.empty())
            output.normals[index] = input.normals[vertex.xref];
        if (!output.uvs.empty())
            output.uvs[index] = input.uvs[vertex.xref];
        if (!output.tangents.empty())
            output.tangents[index] = input.tangents[vertex.xref];
        if (!output.colors.empty())
            output.colors[index] = input.colors[vertex.xref];
    }

    Log::info("LightmapUnwrapper: atlas %ux%u, %u charts, %zu vertices from %zu, %zu triangles",
              atlas->width, atlas->height, atlas->chartCount, vertexCount, input.positions.size(),
              input.indices.size() / 3);
    if (atlasResult)
    {
        atlasResult->width = atlas->width;
        atlasResult->height = atlas->height;
        atlasResult->chartCount = atlas->chartCount;
    }
    xatlas::Destroy(atlas);
    return true;
}

} // namespace Radion
