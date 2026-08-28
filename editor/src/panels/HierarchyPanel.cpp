#include "PCH.h"

#include "panels/HierarchyPanel.h"

#include "AssetManager.h"
#include "Log.h"
#include "panels/AssetsPanel.h"
#include "Billboard.h"
#include "Camera.h"
#include "CameraControllers.h"
#include "EditorApplication.h"
#include "FileSystem.h"
#include "Forest.h"
#include "GameObject.h"
#include "Grass.h"
#include "Landscape.h"
#include "Light.h"
#include "MeshRenderer.h"
#include "Ocean.h"
#include "ParticleEffect.h"
#include "ParticleEmitter.h"
#include "Prefab.h"
#include "ReflectionProbe.h"
#include "Road.h"
#include "Scene.h"
#include "Terrain.h"
#include "Text3D.h"

#include <IconsMaterialDesignIcons.h>
#include <imgui.h>

namespace Radion
{

namespace
{
GameObject* beginCreateObject(EditorApplication& app, const char* name, GameObject* parent)
{
    app.recordUndo();
    GameObject* object = app.scene().createGameObject(name, parent);
    if (object)
        object->setPosition(app.cursor3D());
    return object;
}

// A freshly built primitive has no material of its own - without one its
// submesh has nothing to draw with and RenderList::emitSubmesh() silently
// drops it, so the shape never actually appears in the scene despite the
// mesh upload having succeeded.
Material defaultPrimitiveMaterial()
{
    Material material;
    material.flags |= MaterialLit;
    material.params.baseColor = glm::vec4(0.7f, 0.7f, 0.7f, 1.0f);
    material.params.surface.x = 0.7f; // roughness
    material.params.surface.y = 0.0f; // metal
    material.paramsDirty = true;
    return material;
}

void finishCreateObject(EditorApplication& app, GameObject* object)
{
    if (!object)
        return;

    app.scene().update(0.0f);
    app.selection().select(object->id());
    app.markDirty();
}

template <typename ComponentType>
void createComponentObject(EditorApplication& app, GameObject* parent, const char* name)
{
    GameObject* object = beginCreateObject(app, name, parent);
    if (object)
        object->addComponent<ComponentType>();
    finishCreateObject(app, object);
}

// A flat mirror: a Plane whose material carries MaterialMirror, which
// lit.frag samples as a screen-space planar reflection instead of the
// environment cube (see Material.h's own doc on the flag). Renderer::
// executeReflection() picks the first MaterialMirror surface it finds in
// the frame's opaque list as the plane to render that reflection from, so
// creating this is the whole setup - rotate the GameObject afterward to
// aim it anywhere, the plane's local +Y is its facing.
void createMirrorObject(EditorApplication& app, GameObject* parent)
{
    MeshHandle mesh = Assets().createPlane(2.0f, 2.0f);
    if (!mesh.valid())
        return;
    GameObject* object = beginCreateObject(app, "Mirror", parent);
    if (object)
    {
        MeshRenderer* renderer = object->addComponent<MeshRenderer>();
        renderer->setMesh(mesh);
        Material material;
        material.flags |= MaterialLit | MaterialMirror;
        material.params.baseColor = glm::vec4(0.9f, 0.9f, 0.9f, 1.0f);
        material.params.surface.x = 0.15f; // roughness
        material.params.surface.y = 0.0f;  // metal - the mirror mix is its own path, not this
        material.params.custom0.x = 1.0f;  // mirror strength, see lit.frag's HAS_MIRROR block
        material.params.custom0.y = 0.5f;  // reflection quality (Renderer::executeReflection)
        material.params.custom0.z = 0.0f;  // bump strength - flat by default
        material.params.custom0.w = 1.0f;  // FOV zoom - the fitted frustum's own extent by default
        material.params.custom1.x = 0.0f;  // far plane - auto (Renderer::executeReflection)
        material.paramsDirty = true;
        renderer->setMaterialOverride(0, material);
        // The reflected camera's frustum is fitted exactly to this plane's own
        // rectangle (Renderer::executeReflection()), so the plane itself sits
        // right on that frustum's near clip - left visible, its own back edge
        // flickers into its own reflection at a grazing angle instead of the
        // clean recursion-free capture a mirror is supposed to show.
        renderer->setVisibleInReflections(false);
    }
    finishCreateObject(app, object);
}

// A ReflectionProbe needs create() called before it holds a usable cubemap -
// unlike every other component here, addComponent() alone leaves it inert.
// Zero extents is the shader's no-parallax path (a plain mirror of the
// surroundings); influenceRadius is what makes this the probe a nearby
// object picks over the scene's global one (Scene::resolveNearestProbe()).
void createReflectionProbeObject(EditorApplication& app, GameObject* parent)
{
    GameObject* object = beginCreateObject(app, "Reflection Probe", parent);
    if (object)
    {
        ReflectionProbe* probeComponent = object->addComponent<ReflectionProbe>();
        probeComponent->create(128);
        EnvironmentProbe& env = probeComponent->probe();
        env.extents = glm::vec3(0.0f);
        env.influenceRadius = 5.0f;
        env.content = EnvironmentProbe::Content::SkyAndWorld;
        env.refresh = EnvironmentProbe::Refresh::Automatic;
        env.invalidate();
    }
    finishCreateObject(app, object);
}

} // namespace

HierarchyPanel::HierarchyPanel(EditorApplication& app) : EditorPanel("Hierarchy", app)
{
}

void HierarchyPanel::onImGui()
{
    drawPendingPrimitivePopup();
    drawPendingTerrainPopup();
    drawPendingOceanPopup();
    drawPendingGridDuplicatePopup();
    drawSavePrefabPopup();

    if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_D))
        if (GameObject* selected = app().selection().resolve(app().scene()))
            app().duplicateObject(*selected);

    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        mDragSelecting = false;
    if (mPendingSelect != 0)
    {
        if (ImGui::GetDragDropPayload() != nullptr)
            mPendingSelect = 0;
        else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
        {
            // Ctrl/Shift adds to the set instead of replacing it - what a
            // batch delete needs, and the only place the editor builds a
            // multi-selection at all.
            const ImGuiIO& io = ImGui::GetIO();
            if (io.KeyCtrl || io.KeyShift)
                app().selection().toggle(mPendingSelect);
            else
                app().selection().select(mPendingSelect);
            mPendingSelect = 0;
        }
    }

    drawObjectActions();
    drawSearchField();
    ImGui::Separator();

    GameObject& root = app().scene().root();
    if (!mSearch.empty())
    {
        u32 shown = 0;
        for (usize i = 0; i < root.childCount(); ++i)
            drawFilteredList(*root.child(i), shown);
        if (shown == 0)
            ImGui::TextDisabled("Nothing matches '%s'", mSearch.c_str());
        return;
    }

    ImGui::TextDisabled("Drag here or onto the empty space below to unparent");
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kGameObjectDragPayload))
        {
            const u64 id = *static_cast<const u64*>(payload->Data);
            if (GameObject* object = app().scene().findGameObject(id))
            {
                app().recordUndo();
                if (app().scene().reparent(object))
                    app().markDirty();
            }
        }
        ImGui::EndDragDropTarget();
    }
    for (usize i = 0; i < root.childCount(); ++i)
        drawNode(*root.child(i));

    // The whole empty area under the tree unparents, not just the one-line
    // hint above it: dragging out of a parent is the only way back to root
    // and a thin text target is not something a drag can reliably land on.
    const ImVec2 remaining = ImGui::GetContentRegionAvail();
    if (remaining.y > 1.0f)
    {
        ImGui::Dummy(remaining);
        if (ImGui::BeginDragDropTarget())
        {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kGameObjectDragPayload))
            {
                const u64 id = *static_cast<const u64*>(payload->Data);
                if (GameObject* object = app().scene().findGameObject(id))
                {
                    app().recordUndo();
                    if (app().scene().reparent(object))
                        app().markDirty();
                }
            }
            ImGui::EndDragDropTarget();
        }
        if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            app().selection().clear();
    }

    // Clicking empty space below the tree clears selection - the same
    // convention PLANO_EDITOR.md calls for.
    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
        !ImGui::IsAnyItemHovered())
        app().selection().clear();

    if (ImGui::BeginPopupContextWindow("HierarchyContext", ImGuiPopupFlags_MouseButtonRight |
                                                               ImGuiPopupFlags_NoOpenOverItems))
    {
        if (ImGui::BeginMenu("Create"))
        {
            drawCreateMenu(nullptr);
            ImGui::EndMenu();
        }
        ImGui::EndPopup();
    }
}

void HierarchyPanel::drawCreateMenu(GameObject* parent)
{
    if (ImGui::MenuItem("Empty Node3D"))
        finishCreateObject(app(), beginCreateObject(app(), "Node3D", parent));
    if (ImGui::MenuItem("Camera"))
        createComponentObject<Camera>(app(), parent, "Camera");
    if (ImGui::BeginMenu("Camera + Controller"))
    {
        if (ImGui::MenuItem("Fly (FreeFly)"))
        {
            GameObject* object = beginCreateObject(app(), "Fly Camera", parent);
            if (object)
            {
                object->addComponent<Camera>();
                object->addComponent<FreeFly>();
            }
            finishCreateObject(app(), object);
        }
        if (ImGui::MenuItem("FPS"))
        {
            GameObject* object = beginCreateObject(app(), "FPS Camera", parent);
            if (object)
            {
                object->addComponent<Camera>();
                object->addComponent<FPS>();
            }
            finishCreateObject(app(), object);
        }
        if (ImGui::MenuItem("Orbit"))
        {
            GameObject* object = beginCreateObject(app(), "Orbit Camera", parent);
            if (object)
            {
                object->addComponent<Camera>();
                object->addComponent<Orbit>();
            }
            finishCreateObject(app(), object);
        }
        if (ImGui::MenuItem("Maya"))
        {
            GameObject* object = beginCreateObject(app(), "Maya Camera", parent);
            if (object)
            {
                object->addComponent<Camera>();
                object->addComponent<Maya>();
            }
            finishCreateObject(app(), object);
        }
        ImGui::EndMenu();
    }

    ImGui::Separator();

    if (ImGui::BeginMenu("Lights"))
    {
        if (ImGui::MenuItem("Directional Light"))
        {
            // setSunLight() needs the light already flushed into Scene's own
            // mLights (Scene::setSunLight()'s own "not registered" check) -
            // finishCreateObject()'s update(0.0f) is what does that, so this
            // has to run after it, not right after addComponent().
            GameObject* object = beginCreateObject(app(), "Directional Light", parent);
            DirectionalLight* light = object ? object->addComponent<DirectionalLight>() : nullptr;
            finishCreateObject(app(), object);
            if (light && !app().scene().sunLight())
                app().scene().setSunLight(light);
        }
        if (ImGui::MenuItem("Point Light"))
            createComponentObject<PointLight>(app(), parent, "Point Light");
        if (ImGui::MenuItem("Spot Light"))
            createComponentObject<SpotLight>(app(), parent, "Spot Light");
        if (ImGui::MenuItem("Rectangle Light"))
            createComponentObject<RectangleLight>(app(), parent, "Rectangle Light");
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Primitives"))
    {
        if (ImGui::MenuItem("Cube"))
            queuePendingPrimitive(PrimitiveKind::Cube, parent);
        if (ImGui::MenuItem("Sphere"))
            queuePendingPrimitive(PrimitiveKind::Sphere, parent);
        if (ImGui::MenuItem("Plane"))
            queuePendingPrimitive(PrimitiveKind::Plane, parent);
        if (ImGui::MenuItem("Cylinder"))
            queuePendingPrimitive(PrimitiveKind::Cylinder, parent);
        if (ImGui::MenuItem("Cone"))
            queuePendingPrimitive(PrimitiveKind::Cone, parent);
        if (ImGui::MenuItem("Capsule"))
            queuePendingPrimitive(PrimitiveKind::Capsule, parent);
        if (ImGui::MenuItem("Torus"))
            queuePendingPrimitive(PrimitiveKind::Torus, parent);
        if (ImGui::MenuItem("Hills"))
            queuePendingPrimitive(PrimitiveKind::Hills, parent);
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Mesh"))
    {
        if (ImGui::MenuItem("Mesh Instance"))
            createComponentObject<MeshRenderer>(app(), parent, "Mesh Instance");
        if (ImGui::MenuItem("Billboard"))
            createComponentObject<Billboard>(app(), parent, "Billboard");
        if (ImGui::MenuItem("Text 3D"))
            createComponentObject<Text3D>(app(), parent, "Text 3D");
        if (ImGui::MenuItem("Particle System"))
            createComponentObject<ParticleEffect>(app(), parent, "Particle System");
        if (ImGui::MenuItem("Particle Emitter"))
            createComponentObject<ParticleEmitter>(app(), parent, "Particle Emitter");
        ImGui::Separator();
        ImGui::BeginDisabled();
        ImGui::MenuItem("Choose from Assets...");
        ImGui::EndDisabled();
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Terrain"))
    {
        if (ImGui::MenuItem("Terrain"))
        {
            mPendingTerrain.open = true;
            mPendingTerrain.parent = parent;
            mPendingTerrain.heightmapFile.clear();
        }
        ImGui::BeginDisabled();
        ImGui::MenuItem("Landscape (serialization pending)");
        ImGui::EndDisabled();
        if (ImGui::MenuItem("Road"))
            createComponentObject<Road>(app(), parent, "Road");
        if (ImGui::MenuItem("Grass"))
            createComponentObject<Grass>(app(), parent, "Grass");
        if (ImGui::MenuItem("Tree System"))
            createComponentObject<Forest>(app(), parent, "Tree System");
        if (ImGui::MenuItem("Ocean"))
        {
            mPendingOcean.open = true;
            mPendingOcean.parent = parent;
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Special"))
    {
        if (ImGui::MenuItem("Reflection Probe"))
            createReflectionProbeObject(app(), parent);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Its own capture point, at the 3D cursor - drop it where a room "
                              "needs a reflection the scene's single global probe cannot give "
                              "it, parent it under an object or leave it standalone.");
        if (ImGui::MenuItem("Mirror"))
            createMirrorObject(app(), parent);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("A flat plane that reflects the scene through itself, exactly - "
                              "rotate it to aim it anywhere. Only one reflective plane per frame "
                              "(the first found wins); for more than one, most scenes want "
                              "Reflection Probe instead.");
        ImGui::EndMenu();
    }
}

void HierarchyPanel::drawPendingTerrainPopup()
{
    if (!mPendingTerrain.open)
        return;
    ImGui::OpenPopup("Create Terrain");
    ImGui::SetNextWindowSize(ImVec2(360.0f, 0.0f), ImGuiCond_Appearing);
    if (!ImGui::BeginPopupModal("Create Terrain", &mPendingTerrain.open,
                                ImGuiWindowFlags_AlwaysAutoResize))
        return;
    ImGui::TextUnformatted("Terrain");
    ImGui::Separator();
    ImGui::TextUnformatted("Heightmap");
    ImGui::Button(mPendingTerrain.heightmapFile.empty() ? "Drop image asset here"
                                                        : mPendingTerrain.heightmapFile.c_str(),
                   ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
            mPendingTerrain.heightmapFile =
                std::string(static_cast<const char*>(payload->Data), payload->DataSize);
        ImGui::EndDragDropTarget();
    }
    ImGui::DragFloat("Cell size", &mPendingTerrain.cellSize, 0.01f, 0.001f, 10000.0f);
    ImGui::DragFloat("Height scale", &mPendingTerrain.heightScale, 0.1f, 0.0f, 100000.0f);
    ImGui::DragFloat("UV tiles", &mPendingTerrain.uvTiles, 0.1f, 0.01f, 10000.0f);
    ImGui::DragInt("Max LOD", &mPendingTerrain.maxLod, 0.2f, 1, 32);
    ImGui::Separator();
    ImGui::BeginDisabled(mPendingTerrain.heightmapFile.empty());
    if (ImGui::Button("Create", ImVec2(120.0f, 0.0f)))
    {
        GameObject* object = beginCreateObject(app(), "Terrain", mPendingTerrain.parent);
        if (object)
        {
            Terrain* terrain = object->addComponent<Terrain>();
            if (!terrain->loadFile(mPendingTerrain.heightmapFile.c_str(), mPendingTerrain.cellSize,
                                   mPendingTerrain.heightScale,
                                   static_cast<u32>(mPendingTerrain.maxLod),
                                   mPendingTerrain.uvTiles))
            {
                app().scene().destroy(object);
                object = nullptr;
            }
        }
        finishCreateObject(app(), object);
        mPendingTerrain.open = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
    {
        mPendingTerrain.open = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

void HierarchyPanel::drawPendingOceanPopup()
{
    if (!mPendingOcean.open)
        return;
    ImGui::OpenPopup("Create Ocean");
    ImGui::SetNextWindowSize(ImVec2(360.0f, 0.0f), ImGuiCond_Appearing);
    if (!ImGui::BeginPopupModal("Create Ocean", &mPendingOcean.open,
                                ImGuiWindowFlags_AlwaysAutoResize))
        return;
    ImGui::TextUnformatted("Ocean");
    ImGui::Separator();
    ImGui::DragFloat("Size", &mPendingOcean.size, 0.5f, 1.0f, 100000.0f);
    ImGui::DragInt("Segments", &mPendingOcean.segments, 1.0f, 1, 2048);
    ImGui::DragFloat("Level", &mPendingOcean.level, 0.01f);
    const char* qualityNames[] = {"Sky only", "Reflection", "Reflection + refraction"};
    ImGui::Combo("Quality", &mPendingOcean.quality, qualityNames, IM_ARRAYSIZE(qualityNames));
    ImGui::Separator();
    if (ImGui::Button("Create", ImVec2(120.0f, 0.0f)))
    {
        GameObject* object = beginCreateObject(app(), "Ocean", mPendingOcean.parent);
        if (object)
        {
            Ocean* ocean = object->addComponent<Ocean>();
            if (!ocean->build(mPendingOcean.size, static_cast<u32>(mPendingOcean.segments)))
            {
                app().scene().destroy(object);
                object = nullptr;
            }
            else
            {
                ocean->setLevel(mPendingOcean.level);
                ocean->setQuality(static_cast<OceanQuality>(glm::clamp(mPendingOcean.quality, 0, 2)));
            }
        }
        finishCreateObject(app(), object);
        mPendingOcean.open = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
    {
        mPendingOcean.open = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

void HierarchyPanel::drawPendingGridDuplicatePopup()
{
    if (!mPendingGridDuplicate.open)
        return;
    GameObject* source = app().scene().findGameObject(mPendingGridDuplicate.source);
    if (!source)
    {
        mPendingGridDuplicate.open = false;
        return;
    }
    ImGui::OpenPopup("Duplicates");
    ImGui::SetNextWindowSize(ImVec2(360.0f, 0.0f), ImGuiCond_Appearing);
    if (!ImGui::BeginPopupModal("Duplicates", &mPendingGridDuplicate.open,
                                ImGuiWindowFlags_AlwaysAutoResize))
        return;
    ImGui::Text("Duplicate '%s' in a layout",
                source->name().empty() ? "(unnamed)" : source->name().c_str());
    ImGui::Separator();
    const char* modeNames[] = {"Grid X/Z", "Line X", "Line Z"};
    ImGui::Combo("Layout", &mPendingGridDuplicate.mode, modeNames, IM_ARRAYSIZE(modeNames));
    const bool alongX = mPendingGridDuplicate.mode != 2;
    const bool alongZ = mPendingGridDuplicate.mode != 1;
    if (alongX)
        ImGui::DragInt("Count X", &mPendingGridDuplicate.countX, 1.0f, 1, 512);
    if (alongZ)
        ImGui::DragInt("Count Z", &mPendingGridDuplicate.countZ, 1.0f, 1, 512);
    ImGui::DragFloat("Spacing", &mPendingGridDuplicate.spacing, 0.05f, 0.01f, 10000.0f);
    const u32 countX = alongX ? static_cast<u32>(glm::max(mPendingGridDuplicate.countX, 1)) : 1u;
    const u32 countZ = alongZ ? static_cast<u32>(glm::max(mPendingGridDuplicate.countZ, 1)) : 1u;
    // The source itself keeps the first cell, so one less clone than cells.
    ImGui::TextDisabled("%u objects total (%u new)", countX * countZ, countX * countZ - 1);
    ImGui::Separator();
    if (ImGui::Button("Create", ImVec2(120.0f, 0.0f)))
    {
        app().duplicateObjectGrid(*source, alongX, alongZ, countX, countZ,
                                  mPendingGridDuplicate.spacing);
        mPendingGridDuplicate.open = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
    {
        mPendingGridDuplicate.open = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

void HierarchyPanel::drawSavePrefabPopup()
{
    const std::string root = app().assetBrowserRoot();
    if (!mSavePrefabDialog.Render(root, root, root))
        return;

    const ImGuiFileDialog::Result result = mSavePrefabDialog.ConsumeResult();
    if (!result.accepted)
        return;
    app().settings().lastSaveDirectory = result.path.parent_path().string();

    GameObject* source = app().scene().findGameObject(mSavePrefabSource);
    if (!source)
    {
        app().toasts().error("The object no longer exists");
        return;
    }

    const std::string path = result.path.string();
    Prefab prefab;
    if (prefab.saveToFile(path, *source))
    {
        Log::info("HierarchyPanel: saved prefab '%s'", path.c_str());
        app().toasts().success("Saved " + FileSystem::fileName(path));
    }
    else
    {
        Log::error("HierarchyPanel: could not save prefab '%s'", path.c_str());
        app().toasts().error("Could not save " + FileSystem::fileName(path));
    }
}

void HierarchyPanel::drawObjectActions()
{
    Scene& scene = app().scene();
    GameObject* selected = app().selection().resolve(scene);

    if (ImGui::Button(ICON_MDI_PLUS))
        ImGui::OpenPopup("Add Object");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Add object");
    if (ImGui::BeginPopup("Add Object"))
    {
        if (ImGui::BeginMenu("Create"))
        {
            drawCreateMenu(selected);
            ImGui::EndMenu();
        }
        ImGui::EndPopup();
    }

    ImGui::SameLine();
    ImGui::BeginDisabled(!selected || !selected->parent() ||
                         selected->parent()->childIndex(selected) == 0);
    if (ImGui::Button(ICON_MDI_ARROW_UP))
    {
        app().recordUndo();
        selected->parent()->moveChildUp(selected);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Move up");
    ImGui::EndDisabled();

    ImGui::SameLine();
    const bool canMoveDown =
        selected && selected->parent() &&
        selected->parent()->childIndex(selected) + 1 < selected->parent()->childCount();
    ImGui::BeginDisabled(!canMoveDown);
    if (ImGui::Button(ICON_MDI_ARROW_DOWN))
    {
        app().recordUndo();
        selected->parent()->moveChildDown(selected);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Move down");
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::BeginDisabled(!selected);
    if (ImGui::Button(ICON_MDI_CONTENT_COPY))
        app().duplicateObject(*selected);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Duplicate (Ctrl+D)");
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::BeginDisabled(!selected);
    if (ImGui::Button(ICON_MDI_DELETE))
    {
        app().recordUndo();
        // Resolved into pointers up front: Scene::destroy() is deferred, but
        // the ids are read from a selection this loop is also clearing.
        std::vector<GameObject*> doomed;
        doomed.reserve(app().selection().count());
        for (u64 id : app().selection().selectedIds())
            if (GameObject* object = scene.findGameObject(id))
                doomed.push_back(object);

        bool destroyedAny = false;
        for (GameObject* object : doomed)
            destroyedAny = scene.destroy(object) || destroyedAny;
        if (destroyedAny)
        {
            // Scene::destroy() is deliberately deferred. runFrame() flushes
            // pending scene changes at the start of the next frame; doing it
            // here can invalidate objects while this frame's editor panels
            // are still traversing the hierarchy.
            app().selection().clear();
            app().markDirty();
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(app().selection().count() > 1
                             ? "Delete the selected objects and their children"
                             : "Delete object and its children (Ctrl/Shift-click to select more)");
    ImGui::EndDisabled();
}

void HierarchyPanel::drawSearchField()
{
    char buffer[128];
    std::strncpy(buffer, mSearch.c_str(), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    ImGui::SetNextItemWidth(-ImGui::GetFrameHeight() - ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::InputTextWithHint("##search", ICON_MDI_MAGNIFY " Search", buffer, sizeof(buffer)))
        mSearch = buffer;
    ImGui::SameLine();
    ImGui::BeginDisabled(mSearch.empty());
    if (ImGui::Button(ICON_MDI_CLOSE "##clearsearch"))
        mSearch.clear();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Clear the filter and go back to the tree");
}

bool HierarchyPanel::matchesSearch(const GameObject& object) const
{
    if (mSearch.empty())
        return true;
    const std::string& name = object.name();
    if (name.size() < mSearch.size())
        return false;
    for (usize start = 0; start + mSearch.size() <= name.size(); ++start)
    {
        usize i = 0;
        while (i < mSearch.size() &&
               std::tolower(static_cast<unsigned char>(name[start + i])) ==
                   std::tolower(static_cast<unsigned char>(mSearch[i])))
            ++i;
        if (i == mSearch.size())
            return true;
    }
    return false;
}

void HierarchyPanel::drawFilteredList(GameObject& object, u32& shown)
{
    if (matchesSearch(object))
    {
        ++shown;
        ImGui::PushID(static_cast<int>(object.id()));

        const char* icon = ICON_MDI_CUBE_OUTLINE;
        if (object.getComponent<Camera>())
            icon = ICON_MDI_CAMERA;
        else if (object.getComponent<Light>())
            icon = ICON_MDI_LIGHTBULB;
        else if (object.getComponent<MeshRenderer>())
            icon = ICON_MDI_CUBE;

        const std::string label =
            std::string(icon) + " " + (object.name().empty() ? "(unnamed)" : object.name());
        if (ImGui::Selectable(label.c_str(), app().selection().selectedId() == object.id()))
            app().selection().select(object.id());
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            app().requestFocusObject(object.id());
        if (ImGui::BeginDragDropSource())
        {
            const u64 id = object.id();
            ImGui::SetDragDropPayload(kGameObjectDragPayload, &id, sizeof(id));
            ImGui::TextUnformatted(object.name().c_str());
            ImGui::EndDragDropSource();
        }

        // The flat list hides where a match actually sits, so the one thing
        // the tree gave for free - its position - is spelled out here.
        if (const GameObject* parent = object.parent())
            if (parent->parent())
            {
                ImGui::SameLine();
                ImGui::TextDisabled("in %s", parent->name().c_str());
            }

        ImGui::PopID();
    }

    for (usize i = 0; i < object.childCount(); ++i)
        drawFilteredList(*object.child(i), shown);
}

void HierarchyPanel::drawNode(GameObject& object)
{
    // The id, not the name, as ImGui id: two siblings can share a name, and
    // a rename must not reshuffle which node ImGui thinks is expanded.
    ImGui::PushID(static_cast<int>(object.id()));

    // Decided once and reused by the TreePop() guard below: the drag-drop
    // target on this very node reparents immediately, so childCount() can
    // change between TreeNodeEx() (which pushes only for non-leaves) and the
    // end of this call - re-reading it there popped a push that never was.
    const bool hadChildren = object.childCount() > 0;

    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                               ImGuiTreeNodeFlags_OpenOnDoubleClick |
                               ImGuiTreeNodeFlags_SpanAvailWidth;
    if (!hadChildren)
        flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
    if (app().selection().isSelected(object.id()))
        flags |= ImGuiTreeNodeFlags_Selected;

    const char* icon = ICON_MDI_CUBE_OUTLINE;
    if (object.getComponent<Camera>())
        icon = ICON_MDI_CAMERA;
    else if (object.getComponent<Light>())
        icon = ICON_MDI_LIGHTBULB;
    else if (object.getComponent<MeshRenderer>())
        icon = ICON_MDI_CUBE;

    const std::string label =
        std::string(icon) + " " + (object.name().empty() ? "(unnamed)" : object.name());
    const bool open = ImGui::TreeNodeEx(label.c_str(), flags);
    if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
    {
        mPendingSelect = object.id();
        mDragSelecting = true;
    }
    // Every row the pointer crosses while the button is held joins the set.
    // Additive on purpose: a drag that had to replace the selection each
    // frame would only ever keep the last row it touched.
    if (mDragSelecting && ImGui::IsMouseDragging(ImGuiMouseButton_Left) &&
        ImGui::IsItemHovered() && !app().selection().isSelected(object.id()))
    {
        app().selection().toggle(object.id());
        mPendingSelect = 0;
    }
    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
        app().requestFocusObject(object.id());

    if (ImGui::BeginPopupContextItem("NodeContext"))
    {
        if (ImGui::BeginMenu("Create"))
        {
            drawCreateMenu(&object);
            ImGui::EndMenu();
        }
        if (ImGui::MenuItem(ICON_MDI_CONTENT_COPY " Duplicate", "Ctrl+D"))
            app().duplicateObject(object);
        if (ImGui::MenuItem(ICON_MDI_CONTENT_DUPLICATE " Duplicates..."))
        {
            mPendingGridDuplicate.open = true;
            mPendingGridDuplicate.source = object.id();
        }
        ImGui::Separator();
        if (ImGui::MenuItem(ICON_MDI_PACKAGE_VARIANT_CLOSED " Save as Prefab..."))
        {
            mSavePrefabSource = object.id();
            const std::string suggested =
                (object.name().empty() ? std::string("Prefab") : object.name()) + ".rprefab";
            const std::string& last = app().settings().lastSaveDirectory;
            mSavePrefabDialog.Open(ImGuiFileDialog::Mode::SaveFile,
                                   last.empty() ? app().assetBrowserRoot() : last, suggested);
        }
        ImGui::Separator();
        if (ImGui::MenuItem(ICON_MDI_DELETE " Delete"))
        {
            app().recordUndo();
            // Right-clicking inside a multi-selection deletes the whole set,
            // not just the row under the cursor - the toolbar button does the
            // same, and having the two disagree is how a batch delete
            // silently removes one object.
            bool destroyedAny = false;
            if (app().selection().count() > 1 && app().selection().isSelected(object.id()))
            {
                std::vector<GameObject*> doomed;
                doomed.reserve(app().selection().count());
                for (u64 id : app().selection().selectedIds())
                    if (GameObject* target = app().scene().findGameObject(id))
                        doomed.push_back(target);
                for (GameObject* target : doomed)
                    destroyedAny = app().scene().destroy(target) || destroyedAny;
            }
            else
                destroyedAny = app().scene().destroy(&object);
            if (destroyedAny)
            {
                // Do not flush here: drawNode() still uses `object` below
                // (drag/drop and child traversal). Immediate destruction was
                // a use-after-free and crashed the editor. The normal scene
                // update at the beginning of the next frame performs the
                // queued destruction safely, before any panel starts drawing.
                app().selection().clear();
                app().markDirty();
            }
        }
        ImGui::EndPopup();
    }

    if (ImGui::BeginDragDropSource())
    {
        const u64 id = object.id();
        ImGui::SetDragDropPayload(kGameObjectDragPayload, &id, sizeof(id));
        ImGui::TextUnformatted(object.name().c_str());
        ImGui::EndDragDropSource();
    }
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kGameObjectDragPayload))
        {
            const u64 id = *static_cast<const u64*>(payload->Data);
            if (GameObject* dragged = app().scene().findGameObject(id))
            {
                app().recordUndo();
                if (app().scene().reparent(dragged, &object))
                    app().markDirty();
            }
        }
        ImGui::EndDragDropTarget();
    }

    if (open && hadChildren)
    {
        for (usize i = 0; i < object.childCount(); ++i)
            drawNode(*object.child(i));
        ImGui::TreePop();
    }

    ImGui::PopID();
}

void HierarchyPanel::queuePendingPrimitive(PrimitiveKind kind, GameObject* parent)
{
    mPendingPrimitive.open = true;
    mPendingPrimitive.kind = kind;
    mPendingPrimitive.parent = parent;
    mPendingPrimitive.dimensions = glm::vec3(1.0f);
    mPendingPrimitive.uvTiles = 1.0f;
    mPendingPrimitive.segmentsA = 0;
    mPendingPrimitive.segmentsB = 0;
    mPendingPrimitive.heightmapFile.clear();
    mPendingPrimitive.heightScale = 5.0f;

    switch (kind)
    {
    case PrimitiveKind::Sphere:
        mPendingPrimitive.dimensions.x = 0.5f;
        mPendingPrimitive.segmentsA = 16;
        mPendingPrimitive.segmentsB = 24;
        break;
    case PrimitiveKind::Plane:
        mPendingPrimitive.dimensions.x = 2.0f;
        mPendingPrimitive.dimensions.y = 2.0f;
        mPendingPrimitive.segmentsA = 1;
        mPendingPrimitive.segmentsB = 1;
        break;
    case PrimitiveKind::Cylinder:
    case PrimitiveKind::Cone:
        mPendingPrimitive.dimensions.x = 0.5f;
        mPendingPrimitive.dimensions.y = 1.0f;
        mPendingPrimitive.segmentsB = 24;
        break;
    case PrimitiveKind::Capsule:
        mPendingPrimitive.dimensions.x = 0.5f;
        mPendingPrimitive.dimensions.y = 1.0f;
        mPendingPrimitive.segmentsA = 8;
        mPendingPrimitive.segmentsB = 24;
        break;
    case PrimitiveKind::Torus:
        mPendingPrimitive.dimensions.x = 1.0f;
        mPendingPrimitive.dimensions.y = 0.25f;
        mPendingPrimitive.segmentsA = 24;
        mPendingPrimitive.segmentsB = 12;
        break;
    case PrimitiveKind::Hills:
        mPendingPrimitive.dimensions.x = 20.0f;
        mPendingPrimitive.dimensions.y = 20.0f;
        mPendingPrimitive.segmentsA = 64;
        mPendingPrimitive.segmentsB = 64;
        break;
    case PrimitiveKind::Cube:
        break;
    }
}

// The queued shape stays off the scene tree until this popup's OK button
// fires: only then does the mesh get built and the GameObject actually
// created. Cancel (or dismissing the popup) leaves the scene untouched.
void HierarchyPanel::drawPendingPrimitivePopup()
{
    static const char* kPrimitiveNames[] = {"Cube",    "Sphere", "Plane", "Cylinder",
                                            "Cone",    "Capsule", "Torus", "Hills"};

    if (mPendingPrimitive.open)
        ImGui::OpenPopup("Create Primitive##HierarchyPendingPrimitive");

    ImGui::SetNextWindowSize(ImVec2(280.0f, 0.0f), ImGuiCond_Appearing);
    if (!ImGui::BeginPopupModal("Create Primitive##HierarchyPendingPrimitive", &mPendingPrimitive.open,
                                ImGuiWindowFlags_NoResize))
        return;

    ImGui::TextUnformatted(kPrimitiveNames[static_cast<int>(mPendingPrimitive.kind)]);
    ImGui::Separator();

    switch (mPendingPrimitive.kind)
    {
    case PrimitiveKind::Cube:
        ImGui::DragFloat("Width", &mPendingPrimitive.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Height", &mPendingPrimitive.dimensions.y, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Depth", &mPendingPrimitive.dimensions.z, 0.05f, 0.01f, 10000.0f);
        break;
    case PrimitiveKind::Sphere:
        ImGui::DragFloat("Radius", &mPendingPrimitive.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragInt("Rings", &mPendingPrimitive.segmentsA, 0.2f, 3, 128);
        ImGui::DragInt("Slices", &mPendingPrimitive.segmentsB, 0.2f, 3, 128);
        break;
    case PrimitiveKind::Plane:
        ImGui::DragFloat("Width", &mPendingPrimitive.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Depth", &mPendingPrimitive.dimensions.y, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("UV Tiles", &mPendingPrimitive.uvTiles, 0.1f, 0.01f, 10000.0f);
        ImGui::DragInt("Segments X", &mPendingPrimitive.segmentsA, 0.2f, 1, 256);
        ImGui::DragInt("Segments Z", &mPendingPrimitive.segmentsB, 0.2f, 1, 256);
        break;
    case PrimitiveKind::Cylinder:
    case PrimitiveKind::Cone:
        ImGui::DragFloat("Radius", &mPendingPrimitive.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Height", &mPendingPrimitive.dimensions.y, 0.05f, 0.01f, 10000.0f);
        ImGui::DragInt("Slices", &mPendingPrimitive.segmentsB, 0.2f, 3, 128);
        break;
    case PrimitiveKind::Capsule:
        ImGui::DragFloat("Radius", &mPendingPrimitive.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Height", &mPendingPrimitive.dimensions.y, 0.05f, 0.01f, 10000.0f);
        ImGui::DragInt("Rings", &mPendingPrimitive.segmentsA, 0.2f, 2, 128);
        ImGui::DragInt("Slices", &mPendingPrimitive.segmentsB, 0.2f, 3, 128);
        break;
    case PrimitiveKind::Torus:
        ImGui::DragFloat("Major Radius", &mPendingPrimitive.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Minor Radius", &mPendingPrimitive.dimensions.y, 0.05f, 0.01f, 10000.0f);
        ImGui::DragInt("Major Segments", &mPendingPrimitive.segmentsA, 0.2f, 3, 128);
        ImGui::DragInt("Minor Segments", &mPendingPrimitive.segmentsB, 0.2f, 3, 128);
        break;
    case PrimitiveKind::Hills:
        ImGui::DragFloat("Width", &mPendingPrimitive.dimensions.x, 0.1f, 0.01f, 100000.0f);
        ImGui::DragFloat("Depth", &mPendingPrimitive.dimensions.y, 0.1f, 0.01f, 100000.0f);
        ImGui::DragInt("Segments X", &mPendingPrimitive.segmentsA, 0.5f, 2, 512);
        ImGui::DragInt("Segments Z", &mPendingPrimitive.segmentsB, 0.5f, 2, 512);
        ImGui::DragFloat("Height Scale", &mPendingPrimitive.heightScale, 0.05f, 0.0f, 10000.0f);
        ImGui::DragFloat("UV Tiles", &mPendingPrimitive.uvTiles, 0.1f, 0.01f, 10000.0f);

        ImGui::TextUnformatted("Heightmap");
        ImGui::SameLine();
        ImGui::Button(mPendingPrimitive.heightmapFile.empty() ? "(drop image asset here)"
                                                               : mPendingPrimitive.heightmapFile.c_str(),
                     ImVec2(-FLT_MIN, 0.0f));
        if (ImGui::BeginDragDropTarget())
        {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
                mPendingPrimitive.heightmapFile =
                    std::string(static_cast<const char*>(payload->Data), payload->DataSize);
            ImGui::EndDragDropTarget();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Drag an image from Assets - its red channel becomes displacement, "
                              "0..1 scaled by Height Scale.");
        break;
    }

    ImGui::Separator();
    if (ImGui::Button("OK", ImVec2(120.0f, 0.0f)))
    {
        AssetManager& assets = Assets();
        MeshHandle handle;
        switch (mPendingPrimitive.kind)
        {
        case PrimitiveKind::Cube:
            handle = assets.createBox(mPendingPrimitive.dimensions);
            break;
        case PrimitiveKind::Sphere:
            handle = assets.createSphere(mPendingPrimitive.dimensions.x,
                                         static_cast<u32>(mPendingPrimitive.segmentsA),
                                         static_cast<u32>(mPendingPrimitive.segmentsB));
            break;
        case PrimitiveKind::Plane:
            handle = assets.createPlane(
                mPendingPrimitive.dimensions.x, mPendingPrimitive.dimensions.y,
                static_cast<u32>(mPendingPrimitive.segmentsA),
                static_cast<u32>(mPendingPrimitive.segmentsB), mPendingPrimitive.uvTiles);
            break;
        case PrimitiveKind::Cylinder:
            handle = assets.createCylinder(mPendingPrimitive.dimensions.x,
                                           mPendingPrimitive.dimensions.y,
                                           static_cast<u32>(mPendingPrimitive.segmentsB));
            break;
        case PrimitiveKind::Cone:
            handle = assets.createCone(mPendingPrimitive.dimensions.x,
                                       mPendingPrimitive.dimensions.y,
                                       static_cast<u32>(mPendingPrimitive.segmentsB));
            break;
        case PrimitiveKind::Capsule:
            handle = assets.createCapsule(mPendingPrimitive.dimensions.x,
                                          mPendingPrimitive.dimensions.y,
                                          static_cast<u32>(mPendingPrimitive.segmentsA),
                                          static_cast<u32>(mPendingPrimitive.segmentsB));
            break;
        case PrimitiveKind::Torus:
            handle = assets.createTorus(mPendingPrimitive.dimensions.x,
                                        mPendingPrimitive.dimensions.y,
                                        static_cast<u32>(mPendingPrimitive.segmentsA),
                                        static_cast<u32>(mPendingPrimitive.segmentsB));
            break;
        case PrimitiveKind::Hills:
            if (!mPendingPrimitive.heightmapFile.empty())
                handle = assets.createHillsPlane(
                    mPendingPrimitive.dimensions.x, mPendingPrimitive.dimensions.y,
                    static_cast<u32>(mPendingPrimitive.segmentsA),
                    static_cast<u32>(mPendingPrimitive.segmentsB), mPendingPrimitive.heightmapFile,
                    mPendingPrimitive.heightScale, mPendingPrimitive.uvTiles);
            else
                Log::error("HierarchyPanel: Hills needs a heightmap - drag one from Assets first");
            break;
        }

        if (handle.valid())
        {
            const char* name = kPrimitiveNames[static_cast<int>(mPendingPrimitive.kind)];
            GameObject* object = beginCreateObject(app(), name, mPendingPrimitive.parent);
            if (object)
            {
                MeshRenderer* renderer = object->addComponent<MeshRenderer>();
                renderer->setMesh(handle);
                renderer->setMaterialOverride(0, defaultPrimitiveMaterial());
            }
            finishCreateObject(app(), object);
        }

        mPendingPrimitive.open = false;
        ImGui::CloseCurrentPopup();
    }

    ImGui::SetItemDefaultFocus();
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
    {
        mPendingPrimitive.open = false;
        ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
}

} // namespace Radion
