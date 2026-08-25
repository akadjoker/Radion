#ifndef RADION_MATERIAL_MANAGER_H
#define RADION_MATERIAL_MANAGER_H

#include "Containers.h"
#include "Material.h"

#include <string>
#include <vector>

namespace Radion
{

// What the pipeline cache is actually keyed by. Hashing PipelineKey+layout
// into one u64 and using that alone as the map key used to mean a genuine
// hash collision - rare, but not impossible with a 64-bit combine - handed a
// material a pipeline built for a different key/layout entirely, wrong
// vertex attributes and all. The hash still only picks the bucket; this
// struct's operator== is what confirms identity.
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

// Every operation on a material lives here. Materials themselves belong to the
// mesh that uses them, so this manager owns no material - what it owns is the
// pipeline cache, keyed by PipelineKey so materials that only differ in
// params share one.
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

    // Loads every material declared in a text file. Parsing details and
    // temporary source descriptions stay private; callers only receive the
    // runtime materials, in declaration order.
    bool load(const std::string& filename, std::vector<Material>& materials) const;

    // Saves materials previously loaded by this manager. Source-only data such
    // as texture filenames and sequence frames is preserved by nameHash.
    bool save(const std::string& filename, const std::vector<Material>& materials) const;

    // Runs every curve and raises paramsDirty if anything moved. Once per
    // frame per material, never per packet: the sort key has to stay put.
    void animate(Material& material, f32 time) const;

    // Uploads params when paramsDirty, and creates the buffer on first use.
    void sync(Material& material) const;

    // Compiles or reuses the fixed pipeline selected by the render category.
    // Materials carry data and state only; they never name shaders.
    PipelineHandle resolvePipeline(Material& material, const VertexLayout& layout, u8 pass = 0);

    // Releases what the material owns on the GPU. Pipelines are cached and
    // shared, so they outlive any one material and are not released here.
    void release(Material& material) const;

    // Releases every cached pipeline. What Engine::shutdown() calls - the
    // cache is the one thing here MaterialManager itself owns, everything
    // else belongs to whoever holds the Material.
    void destroyAllPipelines();

private:
    MaterialManager();

    HashMap<PipelineCacheKey, PipelineHandle, PipelineCacheKeyHash> mPipelines;
};

} // namespace Radion

#endif // RADION_MATERIAL_MANAGER_H
