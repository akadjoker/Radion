#ifndef RADION_MATERIAL_MANAGER_H
#define RADION_MATERIAL_MANAGER_H

#include "Containers.h"
#include "Material.h"

#include <string>
#include <vector>

namespace Radion
{

// Pipeline cache key. A 64-bit hash alone as the map key could hand a material a pipeline built for a different key/layout on collision; the hash only picks the bucket, operator== confirms identity.
struct PipelineCacheKey
{
    PipelineKey key;
    VertexLayout layout;

    bool operator==(const PipelineCacheKey& other) const;
};

struct PipelineCacheKeyHash
{
    usize operator()(const PipelineCacheKey& key) const;
};

// Every operation on a material lives here. Materials belong to their mesh; this manager owns only the pipeline cache, keyed by PipelineKey.
class MaterialManager
{
public:
    static MaterialManager& getSingleton();

    MaterialManager(const MaterialManager&) = delete;
    MaterialManager& operator=(const MaterialManager&) = delete;

    static const MaterialFlagName* flagNames(u32& count);
    static u32 flagBit(const char* name);

    // Which list the material belongs in. Refraction wins over transparency
    // because it needs the scene colour already captured.
    RenderCategory categoryOf(const Material& material) const;

    PipelineKey pipelineKeyOf(const Material& material, u8 pass) const;

    // Loads every material declared in a text file, in declaration order.
    bool load(const std::string& filename, std::vector<Material>& materials) const;

    // Saves materials loaded by this manager; source-only data (texture filenames, sequence frames) is preserved by nameHash.
    bool save(const std::string& filename, const std::vector<Material>& materials) const;

    // Runs every curve and raises paramsDirty if anything moved. Once per
    // frame per material, never per packet: the sort key has to stay put.
    void animate(Material& material, f32 time) const;

    void sync(Material& material) const;

    // Compiles or reuses the fixed pipeline for the render category; materials never name shaders.
    PipelineHandle resolvePipeline(Material& material, const VertexLayout& layout, u8 pass = 0);

    // Pipelines are cached and shared, so they outlive any one material and are not released here.
    void release(Material& material) const;

    // Releases every cached pipeline (Engine::shutdown()); the cache is the one thing MaterialManager owns.
    void destroyAllPipelines();

private:
    MaterialManager();

    HashMap<PipelineCacheKey, PipelineHandle, PipelineCacheKeyHash> mPipelines;
};

} // namespace Radion

#endif // RADION_MATERIAL_MANAGER_H
