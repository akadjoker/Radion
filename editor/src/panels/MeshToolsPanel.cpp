#include "PCH.h"

#include "panels/MeshToolsPanel.h"

#include "Animation.h"
#include "AssetManager.h"
#include "EditorApplication.h"
#include "GameObject.h"
#include "Log.h"
#include "MaterialManager.h"
#include "MeshClipper.h"
#include "MeshRenderer.h"
#include "Scene.h"

#include <IconsMaterialDesignIcons.h>
#include <filesystem>
#include <glm/gtc/quaternion.hpp>
#include <imgui.h>
#include <limits>

namespace Radion
{

MeshToolsPanel::MeshToolsPanel(EditorApplication& app) : EditorPanel("Mesh Tools", app)
{
}

namespace
{
// Every Save dialog here starts wherever the last one (in any of this
// panel's own kinds - mesh/materials/skeleton/animation) left off, not back
// at RADION_ASSET_DIR every time - a project with its own asset layout means
// re-navigating the same subfolder on every single save otherwise.
std::string saveStartDirectory(EditorApplication& app)
{
    const std::string& last = app.settings().lastSaveDirectory;
    return last.empty() ? std::string(RADION_ASSET_DIR) : last;
}
} // namespace

void MeshToolsPanel::onImGui()
{
    GameObject* object = app().selection().resolve(app().scene());
    MeshRenderer* renderer = object ? object->getComponent<MeshRenderer>() : nullptr;
    if (!renderer || !renderer->mesh().valid())
    {
        ImGui::TextDisabled("Select an object with a MeshRenderer.");
        return;
    }

    AssetManager& assets = AssetManager::getSingleton();
    MeshData* data = app().importedMeshData(renderer->mesh());
    if (!data)
    {
        // Not cached from an Import/Load this exact session - but if the
        // mesh has a real file behind it (this session's own import, or one
        // loaded fresh off a saved scene), there is no reason these tools
        // can't work on it too: decode it once now and cache it the same
        // way, rather than only ever working right after Import.
        const MeshDesc& desc = assets.meshDesc(renderer->mesh());
        if (desc.source == MeshSource::File)
        {
            MeshData loaded;
            if (assets.importMeshFileData(desc.file, loaded))
            {
                app().registerImportedMesh(renderer->mesh(), std::move(loaded));
                data = app().importedMeshData(renderer->mesh());
            }
        }
    }

    const MeshDesc& selectedDesc = assets.meshDesc(renderer->mesh());
    const bool selectedIsPlane = selectedDesc.source == MeshSource::Plane;
    GameObject* cutter = mCutterPlaneId != 0 ? app().scene().findGameObject(mCutterPlaneId) : nullptr;
    MeshRenderer* cutterRenderer = cutter ? cutter->getComponent<MeshRenderer>() : nullptr;
    if (!cutterRenderer || assets.meshDesc(cutterRenderer->mesh()).source != MeshSource::Plane)
    {
        cutter = nullptr;
        mCutterPlaneId = 0;
    }

    ImGui::TextUnformatted("Open plane cut");
    if (selectedIsPlane)
    {
        if (ImGui::Button(cutter == object ? "Cutter plane selected" : "Use as cutter plane"))
        {
            mCutterPlaneId = object->id();
            cutter = object;
            mCutStatus = "Plane armed. Select the mesh to cut.";
        }
        ImGui::TextDisabled("The plane's local +Y normal defines the cut direction.");
    }
    else if (!cutter)
    {
        ImGui::TextDisabled("Select a Plane object and choose 'Use as cutter plane'.");
    }
    else
    {
        ImGui::Text("Cutter: %s", cutter->name().c_str());
        ImGui::Checkbox("Keep normal side", &mKeepPlaneNormalSide);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Off removes the side pointed to by the Plane's +Y normal. "
                              "On removes the opposite side.");

        const bool skinned = data && !data->skin.empty();
        ImGui::BeginDisabled(!data || skinned);
        if (ImGui::Button("Apply open cut"))
        {
            const glm::vec3 worldNormal = glm::normalize(cutter->up());
            const glm::vec3 worldPoint = cutter->globalPosition();
            const glm::mat4 targetTransform = object->globalTransform();
            const glm::vec3 localNormal =
                glm::transpose(glm::mat3(targetTransform)) * worldNormal;
            const f32 normalLength = glm::length(localNormal);
            if (normalLength <= 1e-6f)
            {
                mCutStatus = "Cut failed: target transform has a zero scale axis.";
            }
            else
            {
                const glm::vec3 targetOrigin = glm::vec3(targetTransform[3]);
                const f32 localOffset =
                    glm::dot(worldNormal, targetOrigin - worldPoint) / normalLength;
                MeshData clipped;
                if (!clipMeshByPlane(*data, localNormal / normalLength, localOffset,
                                     mKeepPlaneNormalSide, clipped))
                    mCutStatus = "Cut failed: invalid or unsupported mesh data.";
                else if (clipped.indices.empty())
                    mCutStatus = "Cut would remove the complete mesh; nothing was changed.";
                else
                {
                    const u32 oldTriangles = static_cast<u32>(data->indices.size() / 3);
                    const u32 newTriangles = static_cast<u32>(clipped.indices.size() / 3);
                    *data = std::move(clipped);
                    if (app().applyMeshEdit(renderer->mesh()))
                    {
                        mCutStatus = "Cut applied: " + std::to_string(oldTriangles) + " -> " +
                                     std::to_string(newTriangles) + " triangles.";
                    }
                    else
                        mCutStatus = "Cut failed while uploading the edited mesh.";
                }
            }
        }
        ImGui::EndDisabled();
        if (skinned)
            ImGui::TextDisabled("Skinned meshes are not supported by the first version.");
        else if (!data)
            ImGui::TextDisabled("This mesh has no editable CPU data.");
    }
    if (!mCutStatus.empty())
        ImGui::TextWrapped("%s", mCutStatus.c_str());
    ImGui::Separator();

    if (!data)
    {
        ImGui::TextDisabled("'%s' has no mesh file behind it (a primitive, most likely) - "
                            "nothing here to run these against.",
                            object->name().c_str());
        return;
    }

    const Mesh* mesh = assets.getMesh(renderer->mesh());
    ImGui::Text("%s", object->name().c_str());
    if (mesh)
        ImGui::TextDisabled("%u verts, %u tris, %zu submeshes", mesh->vertexCount,
                            mesh->indexCount / 3, mesh->submeshes.size());
    ImGui::Separator();

    if (mesh && ImGui::TreeNode("Submeshes", "Submeshes (%zu)", mesh->submeshes.size()))
    {
        for (usize i = 0; i < mesh->submeshes.size(); ++i)
        {
            const SubMesh& submesh = mesh->submeshes[i];
            const char* materialName = submesh.materialSlot < mesh->materials.size()
                                           ? mesh->materials[submesh.materialSlot].name.c_str()
                                           : "(no material)";
            ImGui::BulletText("#%zu - %u tris - slot %u: %s", i, submesh.indexCount / 3,
                              submesh.materialSlot, materialName);
        }
        ImGui::TreePop();
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Normals / Tangents");
    static bool smoothNormals = true;
    static bool angleWeighted = false;
    ImGui::Checkbox("Smooth", &smoothNormals);
    ImGui::SameLine();
    ImGui::Checkbox("Angle-weighted", &angleWeighted);
    if (ImGui::Button("Generate Normals"))
    {
        assets.recalculateNormals(*data, smoothNormals, angleWeighted);
        app().applyMeshEdit(renderer->mesh());
    }
    ImGui::SameLine();
    if (ImGui::Button("Generate Tangents"))
    {
        // Needs the mesh's own UVs - a tangent has no meaning without them,
        // so this only makes sense after Generate Planar UV on a mesh that
        // had none.
        assets.recalculateTangents(*data);
        app().applyMeshEdit(renderer->mesh());
    }

    ImGui::Separator();
    ImGui::TextUnformatted("UV");
    static int uvMode = 0; // 0 planar, 1 cylindrical, 2 spherical
    static f32 uvResolutionU = 1.0f;
    static f32 uvResolutionV = 1.0f;
    ImGui::SetNextItemWidth(140.0f);
    ImGui::Combo("##uvMode", &uvMode, "Planar\0Cylindrical\0Spherical\0");
    if (uvMode == 0)
    {
        ImGui::DragFloat("Resolution", &uvResolutionU, 0.01f, 0.001f, 100.0f);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("World units per UV tile. Picks, per triangle, whichever axis its "
                              "normal points along most and projects from there, duplicating a "
                              "vertex only where two faces meeting there disagree on the axis (a "
                              "box's own corners) - the fix for a mesh like an old .3ds room "
                              "export with no UVs of its own.");
    }
    else
    {
        ImGui::DragFloat("U tiles", &uvResolutionU, 0.01f, 0.001f, 100.0f);
        ImGui::DragFloat("V tiles", &uvResolutionV, 0.01f, 0.001f, 100.0f);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(uvMode == 1
                                  ? "Wraps once around Y (a barrel/column shape)."
                                  : "Wraps once around Y and once pole-to-pole (a ball shape).");
    }
    if (ImGui::Button("Generate UV"))
    {
        if (uvMode == 0)
            assets.makePlanarUV(*data, uvResolutionU);
        else if (uvMode == 1)
            assets.makeCylindricalUV(*data, uvResolutionU, uvResolutionV);
        else
            assets.makeSphericalUV(*data, uvResolutionU, uvResolutionV);
        app().applyMeshEdit(renderer->mesh());
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Winding");
    static int windingTarget = 0; // 0 whole mesh, 1+ = submesh index + 1
    ImGui::SetNextItemWidth(160.0f);
    if (ImGui::BeginCombo("Target", windingTarget == 0
                                        ? "Whole Mesh"
                                        : ("Submesh #" + std::to_string(windingTarget - 1)).c_str()))
    {
        if (ImGui::Selectable("Whole Mesh", windingTarget == 0))
            windingTarget = 0;
        if (mesh)
            for (usize i = 0; i < mesh->submeshes.size(); ++i)
            {
                const std::string label = "Submesh #" + std::to_string(i);
                if (ImGui::Selectable(label.c_str(), windingTarget == static_cast<int>(i) + 1))
                    windingTarget = static_cast<int>(i) + 1;
            }
        ImGui::EndCombo();
    }
    if (ImGui::Button("Flip Triangles"))
    {
        // Turns front faces to back and back to front - a room built to be
        // seen from inside reads as inside-out from outside (or the other
        // way round) until this runs, and it is common enough for just one
        // submesh/material to be the one that came in backwards that the
        // whole-mesh case alone was not enough.
        if (windingTarget == 0)
            assets.flipWinding(*data);
        else
            assets.flipWinding(*data, static_cast<u32>(windingTarget - 1));
        app().applyMeshEdit(renderer->mesh());
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Submesh structure");
    static int targetTriangles = 4000;
    ImGui::DragInt("Split target (tris)", &targetTriangles, 50.0f, 100, 100000);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Breaks any submesh larger than this into several spatially local "
                          "ones - a culling test gets boxes worth rejecting instead of one box "
                          "the size of the whole mesh. Purely a culling-granularity split: it "
                          "does not change which material draws which triangle.");
    if (ImGui::Button("Split Submeshes"))
    {
        assets.splitSubMeshes(*data, static_cast<u32>(targetTriangles));
        app().applyMeshEdit(renderer->mesh());
    }
    ImGui::SameLine();
    if (ImGui::Button("Join Submeshes"))
    {
        // TEMP diagnostic: which slot each submesh points at, before and
        // after - tells apart "these already share a slot and still did not
        // merge" from "they never shared a slot to begin with".
        for (usize i = 0; i < data->submeshes.size(); ++i)
            Log::info("MeshToolsPanel: before join, submesh #%zu -> material slot %u", i,
                      data->submeshes[i].materialSlot);
        Log::info("MeshToolsPanel: %zu materials total", data->materials.size());

        // Undoes a Split (or an import that came in over-fragmented) - one
        // draw call per material slot instead of one per spatial chunk.
        assets.mergeSubmeshes(*data, false);
        app().applyMeshEdit(renderer->mesh());

        for (usize i = 0; i < data->submeshes.size(); ++i)
            Log::info("MeshToolsPanel: after join, submesh #%zu -> material slot %u", i,
                      data->submeshes[i].materialSlot);
    }
    ImGui::SameLine();
    if (ImGui::Button("Compact Materials"))
    {
        // The renderer's own overrides are indexed by material slot too, so
        // they have to travel through the same remap - left alone, slot 5's
        // override lands on whatever material ends up at 5 afterwards, which
        // reads as the compaction having eaten materials still in use.
        std::vector<Material> previousOverrides(
            renderer->materialOverrides(),
            renderer->materialOverrides() + renderer->materialOverrideCount());

        std::vector<u32> remap;
        const u32 removed = assets.compactMaterials(*data, &remap);
        if (removed > 0)
        {
            renderer->clearMaterialOverrides();
            for (usize slot = 0; slot < previousOverrides.size(); ++slot)
                if (slot < remap.size() && remap[slot] != AssetManager::kInvalidMaterialSlot)
                    renderer->setMaterialOverride(remap[slot], previousOverrides[slot]);

            app().applyMeshEdit(renderer->mesh());
            Log::info("MeshToolsPanel: compacted materials, %u orphaned slot(s) dropped", removed);
            app().toasts().success("Dropped " + std::to_string(removed) +
                                   " orphaned material slot(s)");
        }
        else
            app().toasts().success("No orphaned material slots");
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Drops any material slot no submesh points at any more - what deleting "
                          "submeshes (a building's roof, say, after Ctrl-drag/Shift-click "
                          "picking its pieces in the Viewport) leaves behind. Only changes the "
                          "in-memory mesh; use Save Materials... below afterward to write a "
                          "new, compact .material file rather than overwrite the original.");
    ImGui::SameLine();
    if (ImGui::Button("Compact Geometry"))
    {
        const usize verticesBefore = data->positions.size();
        const usize trianglesBefore = data->indices.size() / 3;
        const u32 dropped = assets.compactGeometry(*data);
        if (dropped > 0)
        {
            app().applyMeshEdit(renderer->mesh());
            Log::info("MeshToolsPanel: compacted geometry, %zu -> %zu verts, %zu -> %zu tris",
                      verticesBefore, data->positions.size(), trianglesBefore,
                      data->indices.size() / 3);
            app().toasts().success("Dropped " + std::to_string(dropped) + " unused vertices");
        }
        else
            app().toasts().success("No unused geometry");
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Deleting a submesh only removes its entry, never its triangles - so a "
                          "mesh stripped to a few pieces still holds, and still SAVES, every "
                          "vertex it started with. This throws those away for real, which is what "
                          "actually makes the saved file (and its load time) smaller. Destructive: "
                          "undo cannot bring the geometry back, so save under a new name.");

    ImGui::Separator();
    ImGui::TextUnformatted("Strip by height");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Deletes whole submeshes by how high off the ground they sit - the "
                          "bulk way to reduce a building to just the floor a navmesh needs, "
                          "without picking every roof piece by hand in the Viewport.");
    static f32 stripHeight = 10.0f;
    ImGui::DragFloat("Height##strip", &stripHeight, 0.1f);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("World-space Y, measured through this object's own transform - the "
                          "same height the Inspector shows for its position.");
    if (ImGui::Button("Remove Submeshes Above"))
    {
        // The submesh's LOWEST point decides: a piece is kept when any part
        // of it still reaches below the line. Judging by centre or by the
        // top instead would throw away a wall that rises past the line
        // while standing on the very floor being kept.
        //
        // Two passes, cheap then exact: the bounding box is only used to
        // skip pieces obviously below the line, and anything it says is
        // above is confirmed against the real vertices. A rotated or
        // diagonal piece has box corners well below any vertex it actually
        // owns, which alone would keep roofs that should have gone.
        const glm::mat4 transform = object->globalTransform();
        std::vector<u32> doomed;
        for (u32 i = 0; i < static_cast<u32>(data->submeshes.size()); ++i)
        {
            const SubMesh& submesh = data->submeshes[i];
            const AABB& bounds = submesh.bounds;
            f32 lowestCorner = std::numeric_limits<f32>::max();
            for (u32 corner = 0; corner < 8; ++corner)
            {
                const glm::vec3 local((corner & 1) ? bounds.max.x : bounds.min.x,
                                      (corner & 2) ? bounds.max.y : bounds.min.y,
                                      (corner & 4) ? bounds.max.z : bounds.min.z);
                lowestCorner = glm::min(lowestCorner, (transform * glm::vec4(local, 1.0f)).y);
            }
            if (lowestCorner <= stripHeight)
                continue;

            f32 lowestVertex = std::numeric_limits<f32>::max();
            const usize end =
                glm::min<usize>(submesh.indexOffset + submesh.indexCount, data->indices.size());
            for (usize index = submesh.indexOffset; index < end; ++index)
            {
                const u32 vertex = data->indices[index];
                if (vertex >= data->positions.size())
                    continue;
                lowestVertex =
                    glm::min(lowestVertex, (transform * glm::vec4(data->positions[vertex], 1.0f)).y);
                if (lowestVertex <= stripHeight)
                    break; // reaches the ground after all - keep it, stop looking
            }
            if (lowestVertex > stripHeight)
                doomed.push_back(i);
        }

        if (doomed.empty())
            app().toasts().success("Nothing sits entirely above that height");
        else
        {
            app().recordMeshUndo(renderer->mesh());
            // Highest index first - each erase shifts everything after it
            // down by one, which a forward loop would then read wrong.
            for (usize i = doomed.size(); i-- > 0;)
                assets.removeSubmesh(*data, doomed[i]);
            renderer->setHiddenSubmeshes({});
            app().applyMeshEdit(renderer->mesh());
            app().markDirty();
            Log::info("MeshToolsPanel: removed %zu submesh(es) sitting above y=%.2f", doomed.size(),
                      static_cast<f64>(stripHeight));
            app().toasts().success("Removed " + std::to_string(doomed.size()) + " submesh(es)");
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Deletes every submesh whose lowest corner is still above the height "
                          "above - roofs, upper floors, anything floating. A piece that reaches "
                          "down past the line is kept whole, however tall it is.");

    ImGui::Separator();
    ImGui::TextUnformatted("Bake into geometry");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Moves/rotates the vertices themselves, shared by every object using "
                          "this mesh - not this one object's own Transform in the Inspector. "
                          "Useful for an import that landed off-centre or on its side.");
    static glm::vec3 bakeTranslate(0.0f);
    ImGui::DragFloat3("Translate##bake", &bakeTranslate.x, 0.01f);
    ImGui::SameLine();
    if (ImGui::Button("Apply##bakeTranslate"))
    {
        assets.translate(*data, bakeTranslate);
        app().applyMeshEdit(renderer->mesh());
        bakeTranslate = glm::vec3(0.0f);
    }
    static glm::vec3 bakeRotateDegrees(0.0f);
    ImGui::DragFloat3("Rotate##bake", &bakeRotateDegrees.x, 0.5f);
    ImGui::SameLine();
    if (ImGui::Button("Apply##bakeRotate"))
    {
        const glm::mat4 rotation = glm::mat4_cast(glm::quat(glm::radians(bakeRotateDegrees)));
        assets.transform(*data, rotation);
        app().applyMeshEdit(renderer->mesh());
        bakeRotateDegrees = glm::vec3(0.0f);
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Optimize");
    static f32 overdrawThreshold = 1.05f;
    static f32 simplifyRatio = 0.5f;
    static f32 simplifyError = 0.01f;

    const bool hasData = data && !data->indices.empty();
    ImGui::BeginDisabled(!hasData);
    if (ImGui::Button("Weld Vertices"))
    {
        const usize before = data->positions.size();
        const u32 removed = assets.weldVertices(*data);
        app().applyMeshEdit(renderer->mesh());
        mOptimizeStatus = "Weld: " + std::to_string(before) + " -> " +
                          std::to_string(data->positions.size()) + " verts (" +
                          std::to_string(removed) + " removed).";
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Merges vertices that are identical across every attribute and "
                          "reindexes. Importers that emit one vertex per face corner (OBJ, 3DS) "
                          "leave meshes several times larger than needed.");
    ImGui::SameLine();
    if (ImGui::Button("Vertex Cache"))
    {
        assets.optimizeVertexCache(*data);
        app().applyMeshEdit(renderer->mesh());
        mOptimizeStatus = "Vertex cache: triangles reordered per submesh.";
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reorders each submesh's triangles so successive ones reuse recently "
                          "transformed vertices. Index order only - nothing moves.");
    ImGui::SameLine();
    if (ImGui::Button("Overdraw"))
    {
        assets.optimizeOverdraw(*data, overdrawThreshold);
        app().applyMeshEdit(renderer->mesh());
        mOptimizeStatus = "Overdraw: triangles reordered per submesh.";
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reorders triangles to draw roughly front-to-back within local "
                          "clusters. Run after Vertex Cache.");
    ImGui::SameLine();
    if (ImGui::Button("Vertex Fetch"))
    {
        const usize before = data->positions.size();
        assets.optimizeVertexFetch(*data);
        app().applyMeshEdit(renderer->mesh());
        mOptimizeStatus = "Vertex fetch: streams reordered, " + std::to_string(before) + " -> " +
                          std::to_string(data->positions.size()) + " verts.";
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reorders the vertex streams into first-use order and drops vertices "
                          "nothing references. Run last.");

    ImGui::SetNextItemWidth(140.0f);
    ImGui::DragFloat("Overdraw threshold", &overdrawThreshold, 0.005f, 1.0f, 3.0f, "%.3f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How much vertex cache efficiency Overdraw may trade away - 1.05 "
                          "allows 5%%.");
    if (ImGui::Button("Optimize All"))
    {
        const usize beforeVerts = data->positions.size();
        const u32 removed = assets.weldVertices(*data);
        assets.optimizeVertexCache(*data);
        assets.optimizeOverdraw(*data, overdrawThreshold);
        assets.optimizeVertexFetch(*data);
        app().applyMeshEdit(renderer->mesh());
        mOptimizeStatus = "Optimized: " + std::to_string(beforeVerts) + " -> " +
                          std::to_string(data->positions.size()) + " verts (" +
                          std::to_string(removed) + " welded), indices reordered.";
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Weld, then Vertex Cache, Overdraw and Vertex Fetch - the full "
                          "pipeline in the right order.");

    ImGui::SetNextItemWidth(140.0f);
    ImGui::SliderFloat("Target ratio", &simplifyRatio, 0.05f, 1.0f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Fraction of the triangles to keep. 0.5 tries to halve the mesh; the "
                          "error limit below may stop it earlier.");
    ImGui::SetNextItemWidth(140.0f);
    ImGui::DragFloat("Error limit", &simplifyError, 0.001f, 0.0001f, 0.5f, "%.4f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Maximum geometric deviation as a fraction of the mesh extent - "
                          "simplification stops before crossing it.");
    if (ImGui::Button("Simplify"))
    {
        const usize beforeTris = data->indices.size() / 3;
        f32 reachedError = 0.0f;
        if (assets.simplifyMesh(*data, simplifyRatio, simplifyError, &reachedError))
        {
            assets.optimizeVertexFetch(*data);
            app().applyMeshEdit(renderer->mesh());
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Simplify: %zu -> %zu tris, error %.4f.",
                     beforeTris, data->indices.size() / 3, reachedError);
            mOptimizeStatus = buffer;
        }
        else
            mOptimizeStatus = "Simplify failed: mesh has no editable geometry.";
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Collapses edges per submesh until the target ratio or the error "
                          "limit is hit. Submesh borders and material seams hold; unreferenced "
                          "vertices are compacted afterwards.");
    ImGui::EndDisabled();
    if (!mOptimizeStatus.empty())
        ImGui::TextWrapped("%s", mOptimizeStatus.c_str());

    ImGui::Separator();
    if (ImGui::Button(ICON_MDI_CONTENT_SAVE " Save As..."))
    {
        mSaveHandle = renderer->mesh();
        mSaveKind = SaveKind::Mesh;
        mSaveDialog.Open(ImGuiFileDialog::Mode::SaveFile, saveStartDirectory(app()),
                         object->name() + ".rmesh");
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Writes the edits above out as a new .rmesh and points this object's "
                          "mesh at it - the original file is untouched. Without this, every edit "
                          "on this panel only lives in this session's memory: reopening the scene "
                          "fresh would read the unedited original again.");
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_CONTENT_SAVE " Save Materials..."))
    {
        mSaveKind = SaveKind::Materials;
        mSaveDialog.Open(ImGuiFileDialog::Mode::SaveFile, saveStartDirectory(app()),
                         object->name() + ".material");
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Writes this object's current materials (its overrides, or the mesh's "
                          "own where there is none) out as a .material sidecar - a mesh imported "
                          "with no sidecar of its own (Andromeda, the ninja, ...) only ever gets "
                          "AssetMesh.cpp's flat fallback; this is how it gets a real one, same "
                          "format sinbad.material already is. Save it next to the mesh file and "
                          "the next import of it picks it up automatically.");

    // Skeleton/clips: only for an object whose Animator is actually bound -
    // nothing here to write out for a static mesh, or one whose Animator
    // failed to bind.
    if (Animator* animator = object->getComponent<Animator>())
    {
        if (animator->bound())
        {
            const AnimationSet* set = Animations().get(animator->animationSet());
            ImGui::Separator();
            ImGui::TextUnformatted("Skeleton / Animation");
            if (ImGui::Button(ICON_MDI_CONTENT_SAVE " Save Skeleton..."))
            {
                mSaveKind = SaveKind::Skeleton;
                mSaveDialog.Open(ImGuiFileDialog::Mode::SaveFile, saveStartDirectory(app()),
                                 object->name() + ".rskel");
            }
            if (set)
                for (const AnimationClip& clip : set->clips)
                {
                    ImGui::PushID(&clip);
                    if (ImGui::Button(ICON_MDI_CONTENT_SAVE))
                    {
                        mSaveKind = SaveKind::Animation;
                        mSaveClipName = clip.name();
                        mSaveDialog.Open(ImGuiFileDialog::Mode::SaveFile, saveStartDirectory(app()),
                                         clip.name() + ".ranim");
                    }
                    ImGui::SameLine();
                    ImGui::TextUnformatted(clip.name().c_str());
                    ImGui::PopID();
                }
        }
    }

    if (mSaveKind != SaveKind::None &&
        mSaveDialog.Render(RADION_ASSET_DIR, RADION_ASSET_DIR, RADION_ASSET_DIR))
    {
        const ImGuiFileDialog::Result result = mSaveDialog.ConsumeResult();
        const SaveKind kind = mSaveKind;
        mSaveKind = SaveKind::None;
        if (!result.accepted)
        {
            mSaveClipName.clear();
            return;
        }
        app().settings().lastSaveDirectory = result.path.parent_path().string();

        switch (kind)
        {
        case SaveKind::Mesh:
        {
            MeshData* pending = app().importedMeshData(mSaveHandle);
            if (pending && assets.saveMesh(*pending, result.path.string()))
            {
                std::error_code error;
                const std::filesystem::path relative = std::filesystem::relative(
                    result.path, std::filesystem::path(RADION_ASSET_DIR), error);
                const std::string relativePath =
                    !error && !relative.empty() ? relative.generic_string() : result.path.string();
                assets.registerMeshDesc(mSaveHandle, MeshDesc::fromFile(relativePath));
                Log::info("MeshToolsPanel: saved '%s'", relativePath.c_str());
            }
            else
                Log::error("MeshToolsPanel: failed to save '%s'", result.path.string().c_str());
            break;
        }
        case SaveKind::Materials:
        {
            std::vector<Material> materials;
            if (renderer->materialOverrideCount() > 0)
                materials.assign(renderer->materialOverrides(),
                                 renderer->materialOverrides() + renderer->materialOverrideCount());
            else if (mesh)
                materials = mesh->materials;
            if (MaterialManager::getSingleton().save(result.path.string(), materials))
                Log::info("MeshToolsPanel: saved '%s'", result.path.string().c_str());
            else
                Log::error("MeshToolsPanel: failed to save '%s'", result.path.string().c_str());
            break;
        }
        case SaveKind::Skeleton:
        {
            if (Animator* animator = object->getComponent<Animator>();
                animator && animator->skeleton() &&
                RadionSkeletonIO::saveSkeleton(result.path.string(), *animator->skeleton()))
                Log::info("MeshToolsPanel: saved '%s'", result.path.string().c_str());
            else
                Log::error("MeshToolsPanel: failed to save '%s'", result.path.string().c_str());
            break;
        }
        case SaveKind::Animation:
        {
            Animator* animator = object->getComponent<Animator>();
            const AnimationSet* set = animator ? Animations().get(animator->animationSet()) : nullptr;
            const AnimationClip* clip = nullptr;
            if (set)
                for (const AnimationClip& candidate : set->clips)
                    if (candidate.name() == mSaveClipName)
                    {
                        clip = &candidate;
                        break;
                    }
            if (animator && animator->skeleton() && clip &&
                RadionSkeletonIO::saveAnimation(result.path.string(), *animator->skeleton(), *clip))
                Log::info("MeshToolsPanel: saved '%s'", result.path.string().c_str());
            else
                Log::error("MeshToolsPanel: failed to save '%s'", result.path.string().c_str());
            mSaveClipName.clear();
            break;
        }
        case SaveKind::None:
            break;
        }
    }
}

} // namespace Radion
