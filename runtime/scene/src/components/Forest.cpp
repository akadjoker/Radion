#include "PCH.h"

#include "Forest.h"

#include "AssetManager.h"
#include "GameObject.h"
#include "MaterialManager.h"
#include "RenderList.h"
#include "TreeRender.h"
#include "VegetationGrid.h"

namespace Radion
{

Forest::Forest() : Component(Type)
{
}

void Forest::onDestroy()
{
    MaterialManager& materials = MaterialManager::getSingleton();
    for (Species& species : mSpecies)
    {
        Assets().destroyMesh(species.mesh);
        // MaterialManager::sync() gives a species that casts shadows its own
        // paramsBuffer UBO for the override materials below. Only the mesh
        // used to be freed here - the buffer stayed allocated on the GPU
        // until device shutdown, one leak per Forest removed.
        for (Material& material : species.materials)
            materials.release(material);
    }
    mSpecies.clear();
    mInstances.clear();
    mTotalWeight = 0.0f;
}

f32 Forest::random()
{
    mRandomState = mRandomState * 1664525u + 1013904223u;
    return static_cast<f32>(mRandomState >> 8) / 16777216.0f;
}

bool Forest::buildSpecies(Species& species, const TreeParams& params, f32 height)
{
    MeshData data;
    Assets().buildTree(data, params);
    if (data.positions.empty() || data.submeshes.size() != 2)
        return false;

    // The generator's height comes out of trunkLength, climbRate and the
    // levels together, so it is whatever those happen to give. Scaling the
    // mesh once here is what lets an instance's own scale mean variation
    // rather than correction.
    const f32 grown = data.bounds.max.y - data.bounds.min.y;
    if (grown > 0.0001f && height > 0.0f)
    {
        Assets().scale(data, Math::vec3(height / grown));
        Assets().computeBounds(data);
        Assets().computeSubMeshBounds(data);
    }

    const MeshHandle mesh = Assets().createMesh(data);
    if (!mesh.valid())
        return false;

    const bool existing = species.mesh.valid();

    Assets().destroyMesh(species.mesh);
    species.params = params;
    species.mesh = mesh;
    species.height = height;
    species.radius = data.bounds.radius();
    ++species.impostorRevision;

    if (!existing)
    {
        species.materials[0] = Material();
        species.materials[0].flags |= MaterialLit;
        species.materials[1] = Material();
        species.materials[1].flags |= MaterialAlphaTest | MaterialTwoSided | MaterialLit;
        species.materials[1].params.surface.z = 0.4f;
    }

    AssetManager& assets = Assets();
    if (!mBarkAlbedoPath.empty())
    {
        species.materials[0].textures[SlotAlbedo].file = mBarkAlbedoPath;
        species.materials[0].textures[SlotAlbedo].source = TextureSource::Static;
        species.materials[0].textures[SlotAlbedo].texture =
            assets.loadTexture(mBarkAlbedoPath, Material::colorSpaceFor(SlotAlbedo));
    }
    if (!mBarkNormalPath.empty())
    {
        species.materials[0].textures[SlotNormal].file = mBarkNormalPath;
        species.materials[0].textures[SlotNormal].source = TextureSource::Static;
        species.materials[0].textures[SlotNormal].texture =
            assets.loadTexture(mBarkNormalPath, Material::colorSpaceFor(SlotNormal));
    }

    const u32 twig = species.twigTexture < mTwigAlbedoPaths.size() ? species.twigTexture : 0;
    if (twig < mTwigAlbedoPaths.size())
    {
        // Four mip levels, not the whole chain. A leaf card's texture is mostly
        // transparent, and the box filter has no idea: past a few levels it has
        // averaged the pale colour sitting in those transparent texels into the
        // leaf itself, which is the white halo around every leaf at distance.
        species.materials[1].textures[SlotAlbedo].file = mTwigAlbedoPaths[twig];
        species.materials[1].textures[SlotAlbedo].source = TextureSource::Static;
        species.materials[1].textures[SlotAlbedo].texture = assets.loadTexture(
            mTwigAlbedoPaths[twig], Material::colorSpaceFor(SlotAlbedo), true, 4);
    }

    if (!mCastShadows)
        for (Material& material : species.materials)
            material.flags &= ~MaterialCastShadow;

    // submitPackets() (below) resolves each species' pipeline lazily, just
    // before the frame actually draws it - a planted tree in the scene hits
    // that path and looks fine. MeshPreview::render() does not: it skips any
    // submesh whose material has no pipeline yet (see its own guard), and
    // nothing else ever resolves one before the editor's preview reads it
    // straight off `species.materials`. Resolved here too so the preview has
    // one the first time it draws, not just the trees that get instanced.
    if (const Mesh* built = Assets().getMesh(mesh))
    {
        MaterialManager& materialManager = MaterialManager::getSingleton();
        for (Material& material : species.materials)
            materialManager.resolvePipeline(material, built->colorLayout);
    }

    return true;
}

s32 Forest::addSpecies(const TreeParams& params, f32 height, f32 weight, u32 twigTexture)
{
    Species species;
    species.twigTexture = twigTexture;
    if (!buildSpecies(species, params, height))
        return -1;
    species.weight = weight > 0.0f ? weight : 0.0f;
    mTotalWeight += species.weight;
    mSpecies.push_back(species);
    return static_cast<s32>(mSpecies.size() - 1);
}

bool Forest::rebuildSpecies(u32 species, const TreeParams& params, f32 height)
{
    if (species >= mSpecies.size())
        return false;
    return buildSpecies(mSpecies[species], params, height);
}

u32 Forest::speciesCount() const
{
    return static_cast<u32>(mSpecies.size());
}

const TreeParams& Forest::speciesParams(u32 species) const
{
    return mSpecies[species].params;
}

f32 Forest::speciesHeight(u32 species) const
{
    return species < mSpecies.size() ? mSpecies[species].height : 0.0f;
}

f32 Forest::speciesWeight(u32 species) const
{
    return species < mSpecies.size() ? mSpecies[species].weight : 0.0f;
}

bool Forest::setSpeciesWeight(u32 species, f32 weight)
{
    if (species >= mSpecies.size())
        return false;
    mTotalWeight -= mSpecies[species].weight;
    mSpecies[species].weight = Math::max(0.0f, weight);
    mTotalWeight += mSpecies[species].weight;
    return true;
}

u32 Forest::speciesTwigTexture(u32 species) const
{
    return species < mSpecies.size() ? mSpecies[species].twigTexture : 0;
}

bool Forest::setSpeciesTwigTexture(u32 species, u32 twigTexture)
{
    if (species >= mSpecies.size())
        return false;
    mSpecies[species].twigTexture = twigTexture;
    MaterialTexture& texture = mSpecies[species].materials[1].textures[SlotAlbedo];
    const u32 selected = twigTexture < mTwigAlbedoPaths.size() ? twigTexture : 0;
    if (selected < mTwigAlbedoPaths.size())
    {
        texture.file = mTwigAlbedoPaths[selected];
        texture.source = TextureSource::Static;
        texture.texture = Assets().loadTexture(texture.file, Material::colorSpaceFor(SlotAlbedo), true, 4);
    }
    else
    {
        texture.file.clear();
        texture.texture = TextureHandle();
    }
    return true;
}

MeshHandle Forest::speciesMesh(u32 species) const
{
    return species < mSpecies.size() ? mSpecies[species].mesh : MeshHandle();
}

Material& Forest::material(u32 species, u32 slot)
{
    return mSpecies[species].materials[slot < 2 ? slot : 1];
}

const Material& Forest::material(u32 species, u32 slot) const
{
    return mSpecies[species].materials[slot < 2 ? slot : 1];
}

const std::string& Forest::barkAlbedoPath() const { return mBarkAlbedoPath; }
const std::string& Forest::barkNormalPath() const { return mBarkNormalPath; }
const std::vector<std::string>& Forest::twigTexturePaths() const { return mTwigAlbedoPaths; }

void Forest::setBarkTexture(const std::string& albedo, const std::string& normalMap)
{
    mBarkAlbedoPath = albedo;
    mBarkNormalPath = normalMap;
    for (Species& species : mSpecies)
    {
        MaterialTexture& bark = species.materials[0].textures[SlotAlbedo];
        bark.file = albedo;
        bark.source = TextureSource::Static;
        bark.texture = albedo.empty() ? TextureHandle() :
            Assets().loadTexture(albedo, Material::colorSpaceFor(SlotAlbedo));
        MaterialTexture& normal = species.materials[0].textures[SlotNormal];
        normal.file = normalMap;
        normal.source = TextureSource::Static;
        normal.texture = normalMap.empty() ? TextureHandle() :
            Assets().loadTexture(normalMap, Material::colorSpaceFor(SlotNormal));
    }
}

void Forest::setTwigTextures(const std::vector<std::string>& albedoPaths)
{
    mTwigAlbedoPaths = albedoPaths;
    for (u32 i = 0; i < mSpecies.size(); ++i)
        setSpeciesTwigTexture(i, mSpecies[i].twigTexture);
}

void Forest::addTwigTexture(const std::string& albedoPath)
{
    if (!albedoPath.empty())
        mTwigAlbedoPaths.push_back(albedoPath);
}

bool Forest::removeTwigTexture(u32 index)
{
    if (index >= mTwigAlbedoPaths.size())
        return false;
    mTwigAlbedoPaths.erase(mTwigAlbedoPaths.begin() + index);
    for (u32 i = 0; i < mSpecies.size(); ++i)
    {
        if (mSpecies[i].twigTexture > index)
            --mSpecies[i].twigTexture;
        else if (mSpecies[i].twigTexture == index)
            mSpecies[i].twigTexture = 0;
        setSpeciesTwigTexture(i, mSpecies[i].twigTexture);
    }
    return true;
}

void Forest::setCastShadows(bool enabled)
{
    mCastShadows = enabled;
    for (Species& species : mSpecies)
        for (Material& material : species.materials)
        {
            if (enabled)
                material.flags |= MaterialCastShadow;
            else
                material.flags &= ~MaterialCastShadow;
        }
}

bool Forest::castsShadows() const
{
    return mCastShadows;
}

void Forest::setWind(f32 strength)
{
    mWind = strength > 0.0f ? strength : 0.0f;
}

f32 Forest::wind() const
{
    return mWind;
}

void Forest::setBarkBumpForce(f32 force)
{
    mBarkBumpForce = force > 0.0f ? force : 0.0f;
}

f32 Forest::barkBumpForce() const
{
    return mBarkBumpForce;
}

void Forest::setImpostorsEnabled(bool enabled)
{
    mImpostorsEnabled = enabled;
}

bool Forest::impostorsEnabled() const
{
    return mImpostorsEnabled;
}

void Forest::setSwapDistance(f32 metres)
{
    mSwapDistance = metres > 0.0f ? metres : 0.0f;
}

f32 Forest::swapDistance() const
{
    return mSwapDistance;
}

void Forest::setSwapBand(f32 metres)
{
    mSwapBand = metres > 0.0f ? metres : 0.0f;
}

f32 Forest::swapBand() const
{
    return mSwapBand;
}

void Forest::setImpostorWidth(f32 ratio)
{
    mImpostorWidth = ratio > 0.0f ? ratio : 0.0f;
}

f32 Forest::impostorWidth() const
{
    return mImpostorWidth;
}

u32 Forest::impostorsVisible() const
{
    return mImpostorsVisible;
}

void Forest::setAlphaCut(f32 cut)
{
    mAlphaCut = Math::clamp(cut, 0.05f, 0.95f);
}

f32 Forest::alphaCut() const
{
    return mAlphaCut;
}

u32 Forest::pickSpecies()
{
    if (mTotalWeight <= 0.0f)
        return 0;
    f32 pick = random() * mTotalWeight;
    for (usize i = 0; i < mSpecies.size(); ++i)
    {
        pick -= mSpecies[i].weight;
        if (pick <= 0.0f)
            return static_cast<u32>(i);
    }
    return static_cast<u32>(mSpecies.size() - 1);
}

bool Forest::plant(const Math::vec3& position, u32 species, f32 scale, f32 yawDegrees)
{
    if (species >= mSpecies.size() || scale <= 0.0f)
        return false;
    Instance instance;
    instance.position = position;
    instance.scale = scale;
    instance.yaw = Math::radians(yawDegrees);
    instance.species = species;
    mInstances.push_back(instance);
    return true;
}

u32 Forest::paint(const Math::vec3& centre, f32 radius, u32 count)
{
    if (mSpecies.empty() || radius <= 0.0f)
        return 0;

    u32 planted = 0;
    for (u32 i = 0; i < count; ++i)
    {
        // Square root of the random radius, or every tree crowds the centre:
        // area grows with r², so a uniform r does not give a uniform scatter.
        const f32 angle = random() * 2.0f * Math::pi<f32>();
        const f32 distance = std::sqrt(random()) * radius;
        const Math::vec3 position = centre + Math::vec3(std::cos(angle) * distance, 0.0f,
                                                       std::sin(angle) * distance);

        const u32 species = pickSpecies();
        const f32 scale = mScaleMinimum + random() * (mScaleMaximum - mScaleMinimum);
        if (plant(position, species, scale, random() * 360.0f))
            ++planted;
    }
    return planted;
}

void Forest::clear()
{
    mInstances.clear();
    mVisible = 0;
}

u32 Forest::count() const
{
    return static_cast<u32>(mInstances.size());
}

u32 Forest::instanceCount() const { return static_cast<u32>(mInstances.size()); }
Math::vec3 Forest::instancePosition(u32 index) const
{
    return index < mInstances.size() ? mInstances[index].position : Math::vec3(0.0f);
}
f32 Forest::instanceScale(u32 index) const
{
    return index < mInstances.size() ? mInstances[index].scale : 0.0f;
}
f32 Forest::instanceYaw(u32 index) const
{
    return index < mInstances.size() ? mInstances[index].yaw : 0.0f;
}
u32 Forest::instanceSpecies(u32 index) const
{
    return index < mInstances.size() ? mInstances[index].species : 0;
}

u32 Forest::visibleCount() const
{
    return mVisible;
}

void Forest::setDrawDistance(f32 metres)
{
    mDrawDistance = metres > 0.0f ? metres : 0.0f;
}

f32 Forest::drawDistance() const
{
    return mDrawDistance;
}

void Forest::setScaleRange(f32 minimum, f32 maximum)
{
    mScaleMinimum = minimum > 0.0f ? minimum : 0.0f;
    mScaleMaximum = maximum > mScaleMinimum ? maximum : mScaleMinimum;
}

f32 Forest::scaleMinimum() const { return mScaleMinimum; }
f32 Forest::scaleMaximum() const { return mScaleMaximum; }

void Forest::setSeed(u32 seed)
{
    mRandomState = seed;
}

u32 Forest::seed() const { return mRandomState; }

void Forest::setGrid(VegetationGrid* grid)
{
    mGrid = grid;
}

const VegetationGrid* Forest::grid() const
{
    return mGrid;
}

u32 Forest::paintFromGrid()
{
    if (!mGrid || mGrid->cells().empty() || mSpecies.empty())
        return 0;

    u32 planted = 0;
    for (const VegetationGrid::Cell& cell : mGrid->cells())
    {
        if (cell.occupant != VegetationGrid::Occupant::Tree)
            continue;

        const u32 species = cell.species < mSpecies.size() ? cell.species : 0;
        if (plant(cell.position, species, cell.scale, Math::degrees(cell.yaw)))
            ++planted;
    }
    return planted;
}

void Forest::submit(RenderList& list, const Math::mat4& transform, const Math::vec3& cameraPosition)
{
    mVisible = 0;
    mImpostorsVisible = 0;
    if (mSpecies.empty() || mInstances.empty())
        return;

    // A shadow view takes the generic path; the camera view takes the tree
    // pass. The split is here because only the camera view needs what the tree
    // pipeline adds - the wind, and the leaves' own lighting. A shadow view
    // needs depth and nothing else, and the depth pass already knows how to
    // draw a mesh with a model matrix.
    if ((list.filter() & MaterialCastShadow) != 0)
    {
        submitShadow(list, transform, cameraPosition);
        return;
    }

    submitCamera(transform, cameraPosition);
}

void Forest::submitCamera(const Math::mat4& transform, const Math::vec3& cameraPosition)
{
    AssetManager& assets = Assets();
    const f32 cutoff = mDrawDistance * mDrawDistance;

    for (u32 s = 0; s < mSpecies.size(); ++s)
    {
        Species& species = mSpecies[s];
        const Mesh* mesh = assets.getMesh(species.mesh);
        if (!mesh)
            continue;

        // Instances are baked to world here, because tree.vert adds the
        // instance position straight to the vertex and never sees a model
        // matrix - the same trade Grass::rebuildWorld() makes.
        //
        // Split in two at the swap distance, with the band counted on BOTH
        // sides: a tree inside the band goes in both lists, draws as geometry,
        // and has the impostor fade in over it. That overlap is the crossfade -
        // without it the swap is a pop.
        species.batch.clear();
        species.impostorBatch.clear();
        const f32 meshLimit = mImpostorsEnabled ? mSwapDistance + mSwapBand : mDrawDistance;
        const f32 meshCutoff = Math::min(meshLimit, mDrawDistance);
        const f32 impostorStart = mSwapDistance - mSwapBand;

        for (const Instance& instance : mInstances)
        {
            if (instance.species != s)
                continue;

            const Math::vec3 world = Math::vec3(transform * Math::vec4(instance.position, 1.0f));
            const f32 distanceSquared = Math::dot(world - cameraPosition, world - cameraPosition);
            if (distanceSquared > cutoff)
                continue;

            TreeInstanceData data;
            data.position = world;
            data.scale = instance.scale;
            data.normal = Math::vec3(0.0f, 1.0f, 0.0f);
            data.rotation = instance.yaw;

            if (distanceSquared <= meshCutoff * meshCutoff)
                species.batch.push_back(data);
            if (mImpostorsEnabled && distanceSquared >= impostorStart * impostorStart)
                species.impostorBatch.push_back(data);
        }

        if (species.batch.empty() && species.impostorBatch.empty())
            continue;

        TreeDrawCommand command;
        command.mesh = species.mesh;
        command.instances = species.batch.data();
        command.instanceCount = static_cast<u32>(species.batch.size());
        command.bark = species.materials[0].textures[SlotAlbedo].texture;
        command.barkNormal = species.materials[0].textures[SlotNormal].texture;
        command.twigTexture = species.materials[1].textures[SlotAlbedo].texture;
        command.modelHeight = species.height;
        command.wind = mWind;
        command.alphaCut = mAlphaCut;
        command.bumpForce = mBarkBumpForce;
        command.castShadow = mCastShadows;
        command.impostorInstances = species.impostorBatch.data();
        command.impostorInstanceCount = static_cast<u32>(species.impostorBatch.size());
        command.impostorsEnabled = mImpostorsEnabled;
        command.swapDistance = mSwapDistance;
        command.swapBand = mSwapBand;
        command.impostorWidth = mImpostorWidth;
        // Keyed by the species' slot, and the revision bumps on every rebuild -
        // the mesh changed, so the photographs of it have to change too.
        command.impostorKey = s;
        command.impostorRevision = species.impostorRevision;
        TreeDraws().submit(command);

        mVisible += command.instanceCount;
        mImpostorsVisible += command.impostorInstanceCount;
    }
}

void Forest::submitShadow(RenderList& list, const Math::mat4& transform,
                          const Math::vec3& cameraPosition)
{
    AssetManager& assets = Assets();
    MaterialManager& materials = MaterialManager::getSingleton();
    const f32 cutoff = mDrawDistance * mDrawDistance;

    // Outer loop over species: same mesh, same materials, so their packets go
    // in together and the sort has nothing to untangle.
    for (u32 s = 0; s < mSpecies.size(); ++s)
    {
        Species& species = mSpecies[s];
        Mesh* mesh = assets.getMesh(species.mesh);
        if (!mesh)
            continue;

        for (Material& material : species.materials)
        {
            materials.resolvePipeline(material, mesh->colorLayout);
            materials.sync(material);
        }

        for (const Instance& instance : mInstances)
        {
            if (instance.species != s)
                continue;

            const Math::vec3 world = Math::vec3(transform * Math::vec4(instance.position, 1.0f));
            if (Math::dot(world - cameraPosition, world - cameraPosition) > cutoff)
                continue;

            Math::mat4 model = Math::translate(transform, instance.position);
            model = Math::rotate(model, instance.yaw, Math::vec3(0.0f, 1.0f, 0.0f));
            model = Math::scale(model, Math::vec3(instance.scale));

            // The list runs the frustum test itself, per submesh, so a tree
            // that is behind the camera costs one box transform and no packet.
            list.submit(species.mesh, *mesh, model, species.materials, 2);
        }
    }
}

} // namespace Radion
