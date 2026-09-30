#include "PCH.h"

#include "panels/LightmapPanel.h"

#include "AssetManager.h"
#include "EditorApplication.h"
#include "FileSystem.h"
#include "GameObject.h"
#include "Light.h"
#include "Log.h"
#include "MaterialManager.h"
#include "MeshRenderer.h"
#include "Scene.h"

#include <IconsMaterialDesignIcons.h>
#include <imgui.h>

namespace Radion
{

LightmapPanel::LightmapPanel(EditorApplication& app) : EditorPanel("Lightmap", app)
{
}

namespace
{

// World-space surface area of the mesh. The atlas has to hold every triangle
// at `texelsPerUnit` texels per world unit, and area scales with the square
// of that, so this is what turns "I want a 2048 atlas" into a number.
f32 worldSurfaceArea(const MeshData& mesh, const Math::mat4& transform)
{
    f32 area = 0.0f;
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 a = mesh.indices[i];
        const u32 b = mesh.indices[i + 1];
        const u32 c = mesh.indices[i + 2];
        if (a >= mesh.positions.size() || b >= mesh.positions.size() || c >= mesh.positions.size())
            continue;
        const Math::vec3 p0 = Math::vec3(transform * Math::vec4(mesh.positions[a], 1.0f));
        const Math::vec3 p1 = Math::vec3(transform * Math::vec4(mesh.positions[b], 1.0f));
        const Math::vec3 p2 = Math::vec3(transform * Math::vec4(mesh.positions[c], 1.0f));
        area += Math::length(Math::cross(p1 - p0, p2 - p0)) * 0.5f;
    }
    return area;
}

std::string meshOutputBase(const MeshRenderer& renderer)
{
    const MeshDesc& desc = Assets().meshDesc(renderer.mesh());
    if (desc.source != MeshSource::File || desc.file.empty())
        return std::string();
    const std::string resolved = FileSystem::getSingleton().resolve(desc.file);
    return FileSystem::withoutExtension(resolved.empty() ? desc.file : resolved);
}

// One name, whichever technique produced it: the material points at a single
// lightmap file, so a second bake has to replace the first rather than leave
// a file nothing references.
std::string lightmapPathFor(const MeshRenderer& renderer)
{
    const std::string base = meshOutputBase(renderer);
    if (base.empty())
        return std::string();
    const std::string stem = FileSystem::fileName(base);
    const std::string folder =
        FileSystem::join(FileSystem::directoryOf(base), stem + "_lightmap");
    FileSystem& files = FileSystem::getSingleton();
    if (!files.isDirectory(folder))
        files.createDirectory(folder);
    return FileSystem::join(folder, stem + "_lightmap.png");
}

} // namespace

bool LightmapPanel::sceneSun(Math::vec3& direction, Math::vec3& color)
{
    DirectionalLight* sun = app().scene().electedSunLight();
    if (!sun || !sun->owner())
        return false;
    direction = sun->owner()->forward();
    color = sun->color() * sun->intensity();
    return true;
}

void LightmapPanel::applyBakedTexture(MeshRenderer& renderer, MeshData& data,
                                      const std::string& file)
{
    const TextureHandle lightmap = Assets().reloadTexture(file, ColorSpace::Linear, true);
    if (!lightmap.valid())
    {
        Log::error("LightmapPanel: could not load the baked texture back from '%s'", file.c_str());
        return;
    }
    const Mesh* mesh = Assets().getMesh(renderer.mesh());
    if (!mesh)
        return;
    LightmapBakePass::applyToMaterials(data.materials, mesh->colorLayout, lightmap, file);
    for (u32 slot = 0; slot < data.materials.size(); ++slot)
        renderer.setMaterialOverride(slot, data.materials[slot]);
    app().markDirty();
}

void LightmapPanel::drawUnwrapSection(MeshRenderer& renderer, MeshData& data)
{
    const bool hasUv2 = data.uvs2.size() == data.positions.size();
    if (hasUv2)
        ImGui::TextDisabled("UV2 present - %zu vertices", data.uvs2.size());
    else
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f),
                          "No UV2. Both bakers need one before they can write anything.");

    int resolution = static_cast<int>(mUnwrapSettings.resolution);
    if (ImGui::DragInt("Atlas resolution", &resolution, 16.0f, 0, 8192))
        mUnwrapSettings.resolution = static_cast<u32>(Math::max(resolution, 0));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Zero is not 'no preference' - it is the setting that guarantees a "
                         "single page, sized to whatever texels per unit needs. Any other value "
                         "pins the page size and lets the atlas spill over several pages, which "
                         "the unwrap then refuses.");

    int padding = static_cast<int>(mUnwrapSettings.padding);
    if (ImGui::DragInt("Padding", &padding, 1.0f, 0, 32))
        mUnwrapSettings.padding = static_cast<u32>(Math::max(padding, 0));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Texels of empty space around each chart, so bilinear filtering at a "
                         "chart edge cannot pick up its neighbour.");

    ImGui::DragFloat("Texels per unit", &mUnwrapSettings.texelsPerUnit, 0.01f, 0.0f, 64.0f);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How big a chart gets for a given world size. The atlas area grows with "
                         "the SQUARE of this, so it is the knob that decides whether one page is "
                         "enough. Zero lets the unwrapper estimate one.");

    ImGui::DragInt("Fit to atlas", &mFitResolution, 16.0f, 128, 8192);
    ImGui::SameLine();
    if (ImGui::Button("Fit"))
    {
        // Every triangle needs area * texelsPerUnit^2 texels, and the atlas
        // has resolution^2 to give, so texelsPerUnit = resolution / sqrt(area)
        // is the largest value that still fits one page. Charts never tile
        // perfectly, hence the margin.
        const f32 area = worldSurfaceArea(data, renderer.owner()->globalTransform());
        if (area > 0.0f)
        {
            const f32 fitted = static_cast<f32>(mFitResolution) / std::sqrt(area);
            mUnwrapSettings.texelsPerUnit = fitted * 0.8f;
            mUnwrapSettings.resolution = 0;
            Log::info("LightmapPanel: %.1f world units of surface, texels per unit set to %.3f for "
                      "a %d atlas",
                      static_cast<double>(area), static_cast<double>(mUnwrapSettings.texelsPerUnit),
                      mFitResolution);
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Measures the mesh's world surface area and picks the texels per unit "
                         "that fills an atlas of this size without needing a second page. Sets "
                         "the atlas resolution back to zero, which is what keeps it one page.");

    if (mUnwrapJob.running())
    {
        ImGui::ProgressBar(mUnwrapJob.percent() / 100.0f, ImVec2(-1.0f, 0.0f));
        ImGui::TextDisabled("%s", mUnwrapJob.stage().c_str());
        if (ImGui::Button(ICON_MDI_CLOSE " Cancel Unwrap"))
            mUnwrapJob.cancel();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Stops at the next progress report, which xatlas only sends between "
                              "stages - it is not instant.");
    }
    else if (ImGui::Button(ICON_MDI_VECTOR_SQUARE " Unwrap UV2"))
    {
        mUnwrapJob.start(data, mUnwrapSettings);
        mUnwrapObjectId = renderer.owner() ? renderer.owner()->id() : 0;
    }

    MeshData unwrapped;
    bool unwrapSucceeded = false;
    if (mUnwrapJob.collect(unwrapped, unwrapSucceeded))
    {
        if (!unwrapSucceeded)
        {
            Log::error("LightmapPanel: unwrap failed - see the console for what xatlas said");
            app().toasts().error("UV2 unwrap failed");
        }
        // The selection can move while a large unwrap runs. Applying the
        // result to whatever happens to be selected now would overwrite an
        // unrelated mesh with geometry that is not its own.
        else if (renderer.owner() && renderer.owner()->id() != mUnwrapObjectId)
        {
            Log::error("LightmapPanel: selection changed during the unwrap - result discarded");
            app().toasts().warning("Selection changed during the unwrap - result discarded");
        }
        else
        {
            Assets().computeBounds(unwrapped);
            Assets().computeSubMeshBounds(unwrapped);
            data = std::move(unwrapped);
            app().applyMeshEdit(renderer.mesh());

            // The unwrap splits vertices, so the mesh in memory is no longer
            // the file it came from. Written out and re-registered here or
            // the scene keeps naming the old file, reopens without UV2, and
            // the bake then refuses to run for a reason nothing explains.
            const std::string base = meshOutputBase(renderer);
            const std::string meshOutput = base + ".rmesh";
            const std::string materialOutput = base + ".material";
            if (base.empty() || !Assets().saveMesh(data, meshOutput))
                Log::error("LightmapPanel: UV2 generated but '%s' could not be written - saving "
                           "the scene now would lose it",
                           meshOutput.c_str());
            else
            {
                if (!data.materials.empty())
                    MaterialManager::getSingleton().save(materialOutput, data.materials);
                Assets().registerMeshDesc(renderer.mesh(), MeshDesc::fromFile(meshOutput));
                Log::info("LightmapPanel: UV2 generated, mesh written to '%s'",
                          meshOutput.c_str());
                app().toasts().success("UV2 generated - mesh saved as " + meshOutput);
            }
            app().markDirty();
        }
    }

    const std::string base = meshOutputBase(renderer);
    if (!base.empty())
        ImGui::TextWrapped("Writes %s.rmesh and .material - the unwrapped mesh is what the scene "
                          "will refer to from then on.",
                          base.c_str());
    if (!mUnwrapJob.running())
    {
        const std::string last = mUnwrapJob.stage();
        if (!last.empty())
            ImGui::TextDisabled("last unwrap: %s (%u%%)", last.c_str(), mUnwrapJob.percent());
    }

    const LightmapUnwrapResult& atlas = mUnwrapJob.result();
    if (atlas.width == 0)
        return;

    ImGui::TextDisabled("atlas %ux%u, %u charts", atlas.width, atlas.height, atlas.chartCount);
    // The UV2 is normalized against the atlas xatlas chose. Baking into a
    // smaller texture rescales every chart AND the padding between them, so
    // charts that were properly separated end up bleeding into each other -
    // silently, and looking like a bad bake rather than a mismatch.
    const u32 largest = Math::max(atlas.width, atlas.height);
    if (largest > mBakeResolution)
    {
        const f32 factor = static_cast<f32>(largest) / static_cast<f32>(mBakeResolution);
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.55f, 0.25f, 1.0f));
        ImGui::TextWrapped("Baking at %u shrinks this atlas %.1fx: %u texels of padding become "
                           "%.1f, so charts bleed into each other. Either raise the bake "
                           "resolution or lower Texels per unit and unwrap again.",
                           mBakeResolution, static_cast<double>(factor), mUnwrapSettings.padding,
                           static_cast<double>(mUnwrapSettings.padding) / factor);
        ImGui::PopStyleColor();
    }
}

void LightmapPanel::applyPreset(bool draft, const MeshData& data, const Math::mat4& transform)
{
    // The Final numbers are the ones tools/lightmapbake settled on for the
    // Bistro, not a guess: 8192 shadow against a 4096 map, 16 samples over a
    // 2 degree sun. Draft is the same shape at a quarter of everything, for
    // iterating on the sun angle without waiting.
    if (draft)
    {
        mBakeResolution = 1024;
        mBakeSettings.shadowResolution = 2048;
        mBakeSettings.sampleCount = 4;
        mBakeSettings.filterRadius = 1.0f;
    }
    else
    {
        mBakeResolution = 4096;
        mBakeSettings.shadowResolution = 8192;
        mBakeSettings.sampleCount = 16;
        mBakeSettings.filterRadius = 2.0f;
    }
    mBakeSettings.sunAngularRadius = 2.0f;
    mBakeSettings.ambient = Math::vec3(0.12f, 0.16f, 0.24f);
    mBakeSettings.ambientGround = 0.35f;
    mBakeSettings.biasTexels = 3.0f;
    mBakeSettings.bias = 0.0f;

    // The UV2 half comes with it. A preset that set only the bake left the
    // atlas at whatever it was - packing the charts for a 1024 page and then
    // baking them at 4096 stretches a low-density unwrap over four times the
    // texels, which adds no detail at all, and is exactly the state this
    // panel was found in.
    mUnwrapSettings.padding = 8;
    mFitResolution = static_cast<int>(mBakeResolution);
    mUnwrapSettings.resolution = 0;
    const f32 area = worldSurfaceArea(data, transform);
    if (area > 0.0f)
        mUnwrapSettings.texelsPerUnit =
            (static_cast<f32>(mFitResolution) / std::sqrt(area)) * 0.8f;
}

void LightmapPanel::drawBakeSection(GameObject& object, MeshRenderer& renderer, MeshData& data)
{
    ImGui::TextDisabled("Renders the sun's shadow map and reads it back. Fast, one shot, with "
                       "real penumbra from the sun's angular size.");

    const Math::mat4 transform = object.globalTransform();
    if (ImGui::Button(ICON_MDI_FLASH " Draft"))
        applyPreset(true, data, transform);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("1024 map, 2048 shadow, 4 samples. Seconds instead of minutes - for "
                         "finding the sun angle you want. Sets the UV2 density to match.");
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_STAR " Final"))
        applyPreset(false, data, transform);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("4096 map, 8192 shadow, 16 samples over a 2 degree sun - the settings "
                         "tools/lightmapbake settled on for the Bistro. Sets the UV2 density and "
                         "padding to match, so the atlas and the bake cannot disagree.");
    ImGui::Separator();

    int resolution = static_cast<int>(mBakeResolution);
    if (ImGui::DragInt("Resolution##gpu", &resolution, 8.0f, 16, 8192))
        mBakeResolution = static_cast<u32>(Math::max(resolution, 16));

    int shadowResolution = static_cast<int>(mBakeSettings.shadowResolution);
    if (ImGui::DragInt("Shadow resolution", &shadowResolution, 64.0f, 256, 16384))
        mBakeSettings.shadowResolution = static_cast<u32>(Math::max(shadowResolution, 256));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("A different question from the resolution above: this is how finely the "
                         "sun's shadow was computed, and it is what actually limits how sharp a "
                         "shadow edge can be.");

    ImGui::DragFloat("Sun angular radius", &mBakeSettings.sunAngularRadius, 0.05f, 0.0f, 10.0f,
                    "%.2f deg");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How wide the sun is in the sky. Zero gives hard shadows; the real sun "
                         "is about 0.26 degrees.");

    int sampleCount = static_cast<int>(mBakeSettings.sampleCount);
    if (ImGui::DragInt("Samples##gpu", &sampleCount, 1.0f, 1, 128))
        mBakeSettings.sampleCount = static_cast<u32>(Math::max(sampleCount, 1));
    ImGui::DragFloat("Filter radius", &mBakeSettings.filterRadius, 0.1f, 0.0f, 16.0f);
    ImGui::DragFloat("Bias texels", &mBakeSettings.biasTexels, 0.1f, 0.0f, 32.0f);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Bias measured in shadow-map texels, so it means the same thing in a "
                         "room and across a city. Too low gives moire on flat surfaces, too high "
                         "slides the shadow off its caster.");

    const bool hasUv2 = data.uvs2.size() == data.positions.size();
    ImGui::BeginDisabled(!hasUv2);
    if (ImGui::Button(ICON_MDI_PLAY " Bake (GPU)"))
        mBakeRequested = true;
    ImGui::EndDisabled();

    if (!mBakeRequested)
        return;

    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.35f, 1.0f), ICON_MDI_PROGRESS_CLOCK
                       " Baking %u samples at %u - the window will not answer until it is done",
                       mBakeSettings.sampleCount, mBakeSettings.shadowResolution);
    ImGui::ProgressBar(-0.4f * static_cast<f32>(ImGui::GetTime()), ImVec2(-FLT_MIN, 0.0f),
                      "working");

    if (mBakeFramesShown++ < 1)
        return;
    mBakeRequested = false;
    mBakeFramesShown = 0;

    {
        Math::vec3 sunDirection(0.0f, -1.0f, 0.0f);
        Math::vec3 sunColor(1.0f);
        if (!sceneSun(sunDirection, sunColor))
            Log::warning("LightmapPanel: no directional light in the scene, baking straight down");

        if (!mBakePass.bake(renderer.mesh(), object.globalTransform(), data.bounds, sunDirection,
                          sunColor, mBakeResolution, mBakeSettings))
            Log::error("LightmapPanel: GPU bake failed");
        else
        {
            const std::string file = lightmapPathFor(renderer);
            if (file.empty() || !mBakePass.save(file))
                Log::error("LightmapPanel: GPU bake finished but could not be written");
            else
            {
                Log::info("LightmapPanel: GPU bake written to '%s'", file.c_str());
                app().toasts().success("GPU bake saved to " + file);
                applyBakedTexture(renderer, data, file);
            }
        }
    }
}

void LightmapPanel::onImGui()
{
    if (!active())
        return;

    ImGui::Begin(title().c_str());

    GameObject* object = app().selection().resolve(app().scene());
    MeshRenderer* renderer = object ? object->getComponent<MeshRenderer>() : nullptr;
    MeshData* data = renderer ? app().importedMeshData(renderer->mesh()) : nullptr;
    if (!object || !renderer || !renderer->mesh().valid())
    {
        ImGui::TextDisabled("Select an object with a MeshRenderer.");
        ImGui::End();
        return;
    }
    if (!data)
    {
        ImGui::TextDisabled("This mesh was not imported this session, so there is no CPU-side copy "
                           "to unwrap or bake. Re-import it.");
        ImGui::End();
        return;
    }

    ImGui::Text("%s", object->name().c_str());
    ImGui::Separator();

    if (ImGui::CollapsingHeader("UV2 (xatlas)", ImGuiTreeNodeFlags_DefaultOpen))
        drawUnwrapSection(*renderer, *data);

    ImGui::Separator();
    if (ImGui::CollapsingHeader("Bake", ImGuiTreeNodeFlags_DefaultOpen))
        drawBakeSection(*object, *renderer, *data);

    ImGui::End();
}

} // namespace Radion
