#include "PCH.h"

#include "panels/InspectorPanel.h"

#include "Animation.h"
#include "AssetManager.h"
#include "AudioPlayer.h"
#include "Billboard.h"
#include "BoneAttachment.h"
#include "Camera.h"
#include "CameraControllers.h"
#include "Collider.h"
#include "CollisionWorld.h"
#include "DebugDraw3D.h"
#include "EditorApplication.h"
#include "FileSystem.h"
#include "GameObject.h"
#include "Light.h"
#include "Landscape.h"
#include "Log.h"
#include "MeshRenderer.h"
#include "Ocean.h"
#include "ParticleEffect.h"
#include "ParticleEmitter.h"
#include "ReflectionProbe.h"
#include "Road.h"
#include "Scene.h"
#include "Skeleton.h"
#include "NavMeshSurface.h"
#include "dynamics/JointMatch.h"
#include "dynamics/RigidBody.h"
#include "SelfDestroy.h"
#include "Waypoints.h"
#include "ZenBehaviour.h"
#include "Text3D.h"
#include "Terrain.h"
#include "TiledTerrain.h"
#include "VoxelWorldComponent.h"
#include "Forest.h"
#include "Grass.h"
#include "Hair.h"
#include "collision/CollisionShape.h"
#include "panels/AssetsPanel.h"
#include "panels/HierarchyPanel.h"

#include <IconsMaterialDesignIcons.h>
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <glm/gtc/quaternion.hpp>
#include <imgui.h>

namespace Radion
{

namespace
{

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

// The extension after the last '.', lower-cased - empty for an extension-less
// path.
std::string lowerExtension(const std::string& file)
{
    const usize dot = file.find_last_of('.');
    if (dot == std::string::npos)
        return std::string();
    std::string extension = file.substr(dot + 1);
    for (char& c : extension)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return extension;
}

// Already .rskel: loads it as-is (still needed in-memory, to decode any .fbx
// clip against it below). Anything else (.fbx today): decodes through
// AssetManager::importSkeleton() and writes a .rskel beside it - the format
// Animator::bind()/SceneSerializer actually understand. Empty on failure.
// The .rskel/.ranim this writes always lands next to the SOURCE file on
// disk - `file` itself is often a bare logical path (a MeshDesc's own .file,
// pre-filled by drawAddComponentSection() straight from an already-imported
// mesh, e.g. "models/ninja/ninja.b3d" with no RADION_ASSET_DIR prefix at
// all). FileSystem::writeBinary() (which RadionSkeletonIO::save*() goes
// through) does no search-path resolution the way reading does - it needs a
// real path, so one has to be resolved here before deriving the sibling
// output name, or the write fails against whatever the process's own
// working directory happens to be instead of the asset.
std::string ensureSkeletonFile(const std::string& file, Skeleton& skeleton)
{
    if (lowerExtension(file) == "rskel")
        return RadionSkeletonIO::loadSkeleton(file, skeleton) ? file : std::string();
    if (!Assets().importSkeleton(file, skeleton))
        return std::string();
    const std::string resolved = FileSystem::getSingleton().resolve(file);
    const std::string base = resolved.empty() ? file : resolved;
    const usize dot = base.find_last_of('.');
    const std::string out = (dot == std::string::npos ? base : base.substr(0, dot)) + ".rskel";
    if (RadionSkeletonIO::saveSkeleton(out, skeleton))
        return out;
    Log::error("InspectorPanel: skeleton read from '%s' fine, but could not cache it to '%s'",
              file.c_str(), out.c_str());
    return std::string();
}

// Same idea for one clip, against the skeleton ensureSkeletonFile() already
// resolved - a .fbx clip's bone tracks are indices into that skeleton, so it
// has to exist first.
std::string ensureAnimationFile(const std::string& file, const Skeleton& skeleton)
{
    if (lowerExtension(file) == "ranim")
        return file;
    AnimationClip clip;
    if (!Assets().importAnimation(file, skeleton, clip))
        return std::string();
    const std::string resolved = FileSystem::getSingleton().resolve(file);
    const std::string base = resolved.empty() ? file : resolved;
    const usize dot = base.find_last_of('.');
    const std::string out = (dot == std::string::npos ? base : base.substr(0, dot)) + ".ranim";
    if (RadionSkeletonIO::saveAnimation(out, skeleton, clip))
        return out;
    Log::error("InspectorPanel: animation read from '%s' fine, but could not cache it to '%s'",
              file.c_str(), out.c_str());
    return std::string();
}

// The "Create" button's whole job: get every file to the format loadFromFiles()
// understands, bind an Animator to the result, and leave the object untouched
// on any failure along the way (a half-bound Animator would be worse than
// none - Inspector would show it as "not bound" with no obvious next step).
// `error` is always left with a human-readable reason on false - a caller
// that only logs (the old behaviour here) is indistinguishable from Create
// having silently done nothing at all.
bool createAnimator(GameObject& object, const std::string& skeletonFile,
                    const std::vector<std::string>& clipFiles, std::string& error)
{
    Skeleton skeleton;
    const std::string skeletonPath = ensureSkeletonFile(skeletonFile, skeleton);
    if (skeletonPath.empty())
    {
        error = "Could not prepare a skeleton from '" + skeletonFile +
               "' - either it has no bones (LIMB_NODE/skin), or it read fine but the .rskel "
               "cache next to it failed to write. See the log for the exact reason.";
        Log::error("InspectorPanel: %s", error.c_str());
        return false;
    }
    std::vector<std::string> animationPaths;
    animationPaths.reserve(clipFiles.size());
    for (const std::string& clipFile : clipFiles)
    {
        const std::string animationPath = ensureAnimationFile(clipFile, skeleton);
        if (animationPath.empty())
        {
            error = "Could not prepare an animation from '" + clipFile +
                   "' - either it has no animation stack matching the skeleton above, or it "
                   "read fine but the .ranim cache next to it failed to write. See the log for "
                   "the exact reason.";
            Log::error("InspectorPanel: %s", error.c_str());
            return false;
        }
        animationPaths.push_back(animationPath);
    }
    const AnimationSetHandle handle = Animations().loadFromFiles(skeletonPath, animationPaths);
    if (!handle.valid())
    {
        error = "Skeleton and clips converted to '" + skeletonPath +
               "' and its .ranim files, but AnimationManager::loadFromFiles() still could not "
               "bind them - see the log.";
        Log::error("InspectorPanel: %s", error.c_str());
        return false;
    }
    Animator* animator = object.addComponent<Animator>();
    if (!animator)
    {
        error = "'" + object.name() + "' already has an Animator component.";
        return false;
    }
    animator->bind(handle);
    return true;
}

// One component's row: bullet + name, with a small remove button
// right-aligned - the editor had no way at all to detach a component once
// added (Lumix's property_grid.cpp draws the same idea as a "..." context
// menu with "Remove component"; a direct button reads faster for the one
// action we offer). Returns true the frame the button is clicked - the
// caller must stop touching that component's pointer immediately (it may
// already be gone by the time drawComponentList() processes the removal,
// once every component this frame has had its turn) and must not call this
// twice for the same slot in one frame.
bool drawComponentHeader(EditorApplication& app, const char* label, Component& component)
{
    ImGui::PushID(label);
    ImGui::BulletText("%s", label);
    ImGui::SameLine();
    bool active = component.active();
    if (ImGui::Checkbox("Enabled", &active))
    {
        component.setActive(active);
        app.markDirty();
    }
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - 22.0f);
    const bool clicked = ImGui::SmallButton(ICON_MDI_DELETE);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Remove this component");
    ImGui::PopID();
    return clicked;
}

// GameObject::removeComponent(ComponentType) is private (Component.h keeps
// the type-erased overload internal, addComponent<T>()'s own counterpart) -
// only the templated removeComponent<T>() is public, so drawComponentList's
// single deferred removal (picked at runtime, one frame after the button
// that requested it) needs this to get from the enum back to a concrete T.
void removeComponentByType(GameObject& object, ComponentType type)
{
    switch (type)
    {
    case ComponentType::Camera: object.removeComponent<Camera>(); break;
    case ComponentType::FreeFly: object.removeComponent<FreeFly>(); break;
    case ComponentType::FPS: object.removeComponent<FPS>(); break;
    case ComponentType::Light: object.removeComponent<Light>(); break;
    case ComponentType::MeshRenderer: object.removeComponent<MeshRenderer>(); break;
    case ComponentType::ReflectionProbe: object.removeComponent<ReflectionProbe>(); break;
    case ComponentType::Text3D: object.removeComponent<Text3D>(); break;
    case ComponentType::Billboard: object.removeComponent<Billboard>(); break;
    case ComponentType::BoneAttachment: object.removeComponent<BoneAttachment>(); break;
    case ComponentType::Orbit: object.removeComponent<Orbit>(); break;
    case ComponentType::Maya: object.removeComponent<Maya>(); break;
    case ComponentType::SelfDestroy: object.removeComponent<SelfDestroy>(); break;
    case ComponentType::Waypoints: object.removeComponent<Waypoints>(); break;
    case ComponentType::NavMeshSurface: object.removeComponent<NavMeshSurface>(); break;
    case ComponentType::ThirdPerson: object.removeComponent<ThirdPerson>(); break;
    case ComponentType::Animator: object.removeComponent<Animator>(); break;
    case ComponentType::ParticleEffect: object.removeComponent<ParticleEffect>(); break;
    case ComponentType::Terrain: object.removeComponent<Terrain>(); break;
    case ComponentType::Landscape: object.removeComponent<Landscape>(); break;
    case ComponentType::Road: object.removeComponent<Road>(); break;
    case ComponentType::Grass: object.removeComponent<Grass>(); break;
    case ComponentType::Hair: object.removeComponent<Hair>(); break;
    case ComponentType::TiledTerrain: object.removeComponent<TiledTerrain>(); break;
    case ComponentType::Forest: object.removeComponent<Forest>(); break;
    case ComponentType::Ocean: object.removeComponent<Ocean>(); break;
    case ComponentType::VoxelWorld: object.removeComponent<VoxelWorldComponent>(); break;
    case ComponentType::Script: object.removeComponent<ScriptComponent>(); break;
    case ComponentType::Collider: object.removeComponent<Collider>(); break;
    case ComponentType::RigidBody: object.removeComponent<Physics::RigidBody>(); break;
    case ComponentType::Joint: object.removeComponent<Physics::Joint>(); break;
    case ComponentType::AudioPlayer:
        object.removeComponent<AudioPlayer>();
        break;
    default: break;
    }
}

// A curated subset, not every KeyCode - the realistic choices for a camera
// movement binding (letters near WASD, arrows, the usual modifiers), not a
// combo a user has to hunt through 100+ entries of function/numpad/media
// keys to find "E" in.
constexpr KeyCode kBindableKeys[] = {
    KEY_W,       KEY_A,          KEY_S,        KEY_D,          KEY_Q,
    KEY_E,       KEY_R,          KEY_F,        KEY_C,          KEY_SPACE,
    KEY_LEFT_SHIFT, KEY_LEFT_CONTROL, KEY_LEFT_ALT, KEY_UP,     KEY_DOWN,
    KEY_LEFT,    KEY_RIGHT};

const char* keyLabel(KeyCode key)
{
    switch (key)
    {
    case KEY_W:
        return "W";
    case KEY_A:
        return "A";
    case KEY_S:
        return "S";
    case KEY_D:
        return "D";
    case KEY_Q:
        return "Q";
    case KEY_E:
        return "E";
    case KEY_R:
        return "R";
    case KEY_F:
        return "F";
    case KEY_C:
        return "C";
    case KEY_SPACE:
        return "Space";
    case KEY_LEFT_SHIFT:
        return "Left Shift";
    case KEY_LEFT_CONTROL:
        return "Left Ctrl";
    case KEY_LEFT_ALT:
        return "Left Alt";
    case KEY_UP:
        return "Up Arrow";
    case KEY_DOWN:
        return "Down Arrow";
    case KEY_LEFT:
        return "Left Arrow";
    case KEY_RIGHT:
        return "Right Arrow";
    default:
        return "(other)";
    }
}

// True if changed. `current` falls back to the bound key even when it is
// not one of kBindableKeys (an old scene, or one hand-edited outside this
// combo) - the combo just shows it as "(other)" rather than silently
// snapping to something else the moment the Inspector draws it.
bool drawKeyCombo(const char* label, KeyCode& current)
{
    bool changed = false;
    if (ImGui::BeginCombo(label, keyLabel(current)))
    {
        for (KeyCode key : kBindableKeys)
        {
            const bool selected = key == current;
            if (ImGui::Selectable(keyLabel(key), selected))
            {
                current = key;
                changed = true;
            }
            if (selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    return changed;
}

const char* mouseButtonLabel(MouseButton button)
{
    switch (button)
    {
    case LEFT:
        return "Left";
    case RIGHT:
        return "Right";
    case MIDDLE:
        return "Middle";
    }
    return "Right";
}

bool drawMouseButtonCombo(const char* label, MouseButton& current)
{
    bool changed = false;
    if (ImGui::BeginCombo(label, mouseButtonLabel(current)))
    {
        for (MouseButton button : {LEFT, RIGHT, MIDDLE})
        {
            const bool selected = button == current;
            if (ImGui::Selectable(mouseButtonLabel(button), selected))
            {
                current = button;
                changed = true;
            }
            if (selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    return changed;
}

// Target slot shared by Orbit and Maya - both orbit a GameObject (falling
// back to a fixed world-space point when none is set). Returns true if the
// target changed.
bool drawOrbitTargetSlot(EditorApplication& app, GameObject*& target, glm::vec3& targetPoint)
{
    bool changed = false;
    ImGui::TextUnformatted("Target");
    ImGui::SameLine();
    ImGui::Button(target ? target->name().c_str() : "(drag an object here, or set a point below)",
                 ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kGameObjectDragPayload))
        {
            const u64 id = *static_cast<const u64*>(payload->Data);
            if (GameObject* dragged = app.scene().findGameObject(id))
            {
                target = dragged;
                changed = true;
            }
        }
        ImGui::EndDragDropTarget();
    }
    if (target && ImGui::Button("Clear Target"))
    {
        target = nullptr;
        changed = true;
    }
    if (!target && ImGui::DragFloat3("Target Point", &targetPoint.x, 0.05f))
        changed = true;
    return changed;
}

// Common to FreeFly and FPS - same Action enum shape (Forward/Back/Left/
// Right/Up/Down/Sprint), same speed/look/pitch fields. Returns true if
// anything changed, so the caller knows whether to markDirty().
template <class ControllerType>
bool drawFreeLookController(ControllerType& controller)
{
    bool changed = false;
    ImGui::Indent(14.0f);

    f32 moveSpeed = controller.moveSpeed();
    if (ImGui::DragFloat("Move Speed", &moveSpeed, 0.1f, 0.01f, 10000.0f))
    {
        controller.setMoveSpeed(moveSpeed);
        changed = true;
    }
    f32 sprintMultiplier = controller.sprintMultiplier();
    if (ImGui::DragFloat("Sprint Multiplier", &sprintMultiplier, 0.05f, 1.0f, 100.0f, "%.2fx"))
    {
        controller.setSprintMultiplier(sprintMultiplier);
        changed = true;
    }
    f32 lookSpeed = controller.lookSpeed();
    if (ImGui::DragFloat("Look Speed", &lookSpeed, 0.005f, 0.001f, 5.0f))
    {
        controller.setLookSpeed(lookSpeed);
        changed = true;
    }
    f32 pitchLimit = controller.pitchLimit();
    if (ImGui::DragFloat("Pitch Limit", &pitchLimit, 0.5f, 1.0f, 89.9f, "%.1f°"))
    {
        controller.setPitchLimit(pitchLimit);
        changed = true;
    }
    bool invertY = controller.invertY();
    if (ImGui::Checkbox("Invert Y", &invertY))
    {
        controller.setInvertY(invertY);
        changed = true;
    }
    bool requireLookButton = controller.requiresLookButton();
    if (ImGui::Checkbox("Require Look Button Held", &requireLookButton))
    {
        controller.setRequireLookButton(requireLookButton);
        changed = true;
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Keys");
    static const char* kActionLabels[] = {"Forward", "Back", "Left", "Right",
                                          "Up",      "Down", "Sprint"};
    for (u8 i = 0; i < static_cast<u8>(ControllerType::Action::Count); ++i)
    {
        const auto action = static_cast<typename ControllerType::Action>(i);
        KeyCode key = controller.key(action);
        ImGui::PushID(i);
        if (drawKeyCombo(kActionLabels[i], key))
        {
            controller.setKey(action, key);
            changed = true;
        }
        ImGui::PopID();
    }

    ImGui::Unindent(14.0f);
    return changed;
}

// Same true-if-changed convention as drawTextureSlot below.
bool drawFlagCheckbox(const char* label, MaterialFlags flag, Material& material)
{
    bool value = (material.flags & flag) != 0;
    if (!ImGui::Checkbox(label, &value))
        return false;
    material.flags =
        value ? (material.flags | flag) : (material.flags & ~static_cast<u32>(flag));
    return true;
}

// True if `material` changed - the caller is the one holding the working
// copy, so it decides whether that means writing it back to the renderer.
bool drawTextureSlot(EditorApplication& app, const char* label, MaterialSlot slot,
                     Material& material)
{
    ImGui::PushID(label);
    MaterialTexture& texture = material.textures[slot];
 
    const bool open = ImGui::TreeNodeEx(
        label, ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_AllowOverlap);
 
    ImGui::SameLine();

 
    const ImVec2 iconButtonSize(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
    const f32 iconsWidth =
        iconButtonSize.x * 2.0f + ImGui::GetStyle().ItemSpacing.x * 3.0f;
    const std::string buttonLabel = texture.file.empty() ? "(drop asset here)" : texture.file;
    ImGui::Button(buttonLabel.c_str(), ImVec2(-iconsWidth, 0.0f));

    bool changed = false;
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            const std::string path(static_cast<const char*>(payload->Data), payload->DataSize);
            texture.texture = Assets().loadTexture(
                path, Material::colorSpaceFor(slot, material.flags));
            // Match MaterialParserInternal's defaults exactly. Otherwise a
            // newly dropped map uses sampler 0 (or the previous slot's
            // sampler) until save/reload, then suddenly becomes anisotropic
            // repeat filtering when the sidecar is parsed.
            SamplerDesc sampler;
            sampler.filter = Filter::Anisotropic;
            sampler.wrapU = Wrap::Repeat;
            sampler.wrapV = Wrap::Repeat;
            sampler.wrapW = Wrap::Repeat;
            sampler.anisotropy = 8.0f;
            texture.sampler = Assets().getSampler(sampler);
            texture.source = TextureSource::Static;
            texture.file = path;
            changed = true;
        }
        ImGui::EndDragDropTarget();
    }

    // A regular Button, not SmallButton - SmallButton zeroes FramePadding
    // for a tight inline-with-text look, which leaves no vertical room for
    // an icon glyph taller than a text line and clips its top edge.
    ImGui::SameLine();
    ImGui::BeginDisabled(texture.file.empty());
    if (ImGui::Button(ICON_MDI_CROSSHAIRS_GPS, iconButtonSize))
        app.requestRevealAsset(texture.file);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reveal in Assets");
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(texture.file.empty());
    if (ImGui::Button(ICON_MDI_DELETE, iconButtonSize))
    {
        texture = MaterialTexture();
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Clear");
    ImGui::EndDisabled();

    if (open)
    {
        ImGui::Spacing();
  
        if (texture.texture.valid())
        {
            const u32 nativeId = app.engine().getGPU().nativeTextureId(texture.texture);
            ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(nativeId)),
                        ImVec2(64.0f, 64.0f));
        }
        else
            ImGui::TextDisabled("No texture assigned");
        ImGui::TreePop();
    }

    ImGui::PopID();
    return changed;
}

} // namespace

InspectorPanel::InspectorPanel(EditorApplication& app) : EditorPanel("Inspector", app)
{
}

InspectorPanel::~InspectorPanel()
{
    mTreePreview.destroy();
}

void InspectorPanel::onImGui()
{
    GameObject* object = app().selection().resolve(app().scene());
    if (!object)
    {
        ImGui::TextDisabled("No object selected");
        return;
    }
    drawHeader(*object);
    ImGui::Separator();
    drawTransform(*object);
    ImGui::Separator();
    drawComponentList(*object);
}

void InspectorPanel::drawHeader(GameObject& object)
{
    ImGui::TextDisabled("ID: %llu", static_cast<unsigned long long>(object.id()));

    bool active = object.active();
    if (ImGui::Checkbox("Active", &active))
    {
        object.setActive(active);
        app().markDirty();
    }
    ImGui::SameLine();
    bool visible = object.visible();
    if (ImGui::Checkbox("Visible", &visible))
    {
        object.setVisible(visible);
        app().markDirty();
    }
    ImGui::SameLine();
    bool isStatic = object.isStatic();
    if (ImGui::Checkbox("Static", &isStatic))
    {
        object.setStatic(isStatic);
        app().scene().rebuildStaticIndex();
        app().scene().rebuildDynamicIndex();
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Static geometry is indexed in the scene BVH, which culls it per camera "
                          "and per shadow cascade. Promise the transform will not move again.");

    char nameBuffer[256];
    std::strncpy(nameBuffer, object.name().c_str(), sizeof(nameBuffer) - 1);
    nameBuffer[sizeof(nameBuffer) - 1] = '\0';
    if (ImGui::InputText("Name", nameBuffer, sizeof(nameBuffer)))
    {
        object.setName(nameBuffer);
        app().markDirty();
    }

    char tagBuffer[256];
    std::strncpy(tagBuffer, object.tag().c_str(), sizeof(tagBuffer) - 1);
    tagBuffer[sizeof(tagBuffer) - 1] = '\0';
    if (ImGui::InputText("Tag", tagBuffer, sizeof(tagBuffer)))
    {
        object.setTag(tagBuffer);
        app().markDirty();
    }
}

namespace
{
// A SmallButton is shorter than the DragFloat3 it shares a line with, and
// being first it is what SameLine() lines the row up on - which left the
// field hanging below it. Square, exactly one frame high, matches.
bool resetButton(const char* id, const char* tooltip)
{
    const f32 size = ImGui::GetFrameHeight();
    const bool pressed = ImGui::Button(id, ImVec2(size, size));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", tooltip);
    ImGui::SameLine();
    return pressed;
}
} // namespace

void InspectorPanel::drawTransform(GameObject& object)
{
    if (!ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen))
        return;

    if (resetButton(ICON_MDI_RESTORE "##resetPosition", "Reset position to 0, 0, 0"))
    {
        app().recordUndo();
        object.setPosition(glm::vec3(0.0f));
        app().markDirty();
    }
    glm::vec3 position = object.position();
    if (ImGui::DragFloat3("Position", &position.x, 0.05f))
    {
        object.setPosition(position);
        app().markDirty();
    }

    // Displayed as Euler degrees, converted from the live quaternion every
    // frame - simplest editing surface for v1, at the usual cost of that
    // conversion: no guarantee of the same Euler triple across a gimbal-lock
    // pose. Revisit if that turns out to matter in practice.
    if (resetButton(ICON_MDI_RESTORE "##resetRotation", "Reset rotation to 0, 0, 0"))
    {
        app().recordUndo();
        object.setRotationDegrees(glm::vec3(0.0f));
        app().markDirty();
    }
    const glm::vec3 eulerRadians = glm::eulerAngles(object.rotation());
    glm::vec3 eulerDegrees = glm::degrees(eulerRadians);
    if (ImGui::DragFloat3("Rotation", &eulerDegrees.x, 0.5f))
    {
        object.setRotationDegrees(eulerDegrees);
        app().markDirty();
    }

    if (resetButton(ICON_MDI_RESTORE "##resetScale", "Reset scale to 1, 1, 1"))
    {
        app().recordUndo();
        object.setScale(glm::vec3(1.0f));
        app().markDirty();
    }
    glm::vec3 scale = object.scale();
    if (ImGui::DragFloat3("Scale", &scale.x, 0.02f))
    {
        object.setScale(scale);
        app().markDirty();
    }
}

void InspectorPanel::drawComponentList(GameObject& object)
{
    if (!ImGui::CollapsingHeader("Components", ImGuiTreeNodeFlags_DefaultOpen))
        return;

    // Removal happens once, after every component below has had its turn -
    // removing mid-loop would leave whichever component's `if` runs next
    // (drawAnimatorComponent, e.g.) holding a pointer into a slot that no
    // longer exists.
    ComponentType toRemove = ComponentType::Count;

    // Every block below is wrapped in PushID(label)/PopID() at the call site,
    // not just inside drawComponentHeader() (which pops its own before this
    // even starts): two components on the same object whose own fields share
    // a bare label - Orbit and Maya both have "Distance"/"Pitch"/"Yaw", say -
    // land in the same window-level ID scope otherwise. CollapsingHeader
    // (drawComponentHeader's own) does not push a scope for what follows it
    // the way TreeNode does, so nothing upstream was covering this.
    if (Camera* camera = object.getComponent<Camera>())
    {
        ImGui::PushID("Camera");
        if (drawComponentHeader(app(), "Camera", *camera))
            toRemove = ComponentType::Camera;
        else
            drawCameraComponent(*camera);
        ImGui::PopID();
    }
    if (FreeFly* freeFly = object.getComponent<FreeFly>())
    {
        ImGui::PushID("FreeFly");
        if (drawComponentHeader(app(), "FreeFly", *freeFly))
            toRemove = ComponentType::FreeFly;
        else if (drawFreeLookController(*freeFly))
            app().markDirty();
        ImGui::PopID();
    }
    if (FPS* fps = object.getComponent<FPS>())
    {
        ImGui::PushID("FPS");
        if (drawComponentHeader(app(), "FPS", *fps))
            toRemove = ComponentType::FPS;
        else if (drawFreeLookController(*fps))
            app().markDirty();
        ImGui::PopID();
    }
    if (Light* light = object.getComponent<Light>())
    {
        const char* label = "Light";
        switch (light->lightType())
        {
        case LightType::Directional: label = "DirectionalLight"; break;
        case LightType::Point: label = "PointLight"; break;
        case LightType::Spot: label = "SpotLight"; break;
        case LightType::Rectangle: label = "RectangleLight"; break;
        }
        ImGui::PushID(label);
        if (drawComponentHeader(app(), label, *light))
            toRemove = ComponentType::Light;
        else
            drawLightComponent(*light);
        ImGui::PopID();
    }
    if (MeshRenderer* renderer = object.getComponent<MeshRenderer>())
    {
        ImGui::PushID("MeshRenderer");
        if (drawComponentHeader(app(), "MeshRenderer", *renderer))
            toRemove = ComponentType::MeshRenderer;
        else
            drawMeshRenderer(object, *renderer);
        ImGui::PopID();
    }
    if (ReflectionProbe* probeComponent = object.getComponent<ReflectionProbe>())
    {
        ImGui::PushID("ReflectionProbe");
        if (drawComponentHeader(app(), "ReflectionProbe", *probeComponent))
            toRemove = ComponentType::ReflectionProbe;
        else
            drawReflectionProbe(*probeComponent);
        ImGui::PopID();
    }
    if (Text3D* text = object.getComponent<Text3D>())
    {
        ImGui::PushID("Text3D");
        if (drawComponentHeader(app(), "Text3D", *text))
            toRemove = ComponentType::Text3D;
        else
            drawText3DComponent(*text);
        ImGui::PopID();
    }
    if (Billboard* billboard = object.getComponent<Billboard>())
    {
        ImGui::PushID("Billboard");
        if (drawComponentHeader(app(), "Billboard", *billboard))
            toRemove = ComponentType::Billboard;
        else
            drawBillboardComponent(*billboard);
        ImGui::PopID();
    }
    if (Waypoints* waypoints = object.getComponent<Waypoints>())
    {
        ImGui::PushID("Waypoints");
        if (drawComponentHeader(app(), "Waypoints", *waypoints))
            toRemove = ComponentType::Waypoints;
        else
            drawWaypointsComponent(object, *waypoints);
        ImGui::PopID();
    }
    if (NavMeshSurface* surface = object.getComponent<NavMeshSurface>())
    {
        ImGui::PushID("NavMeshSurface");
        if (drawComponentHeader(app(), "NavMeshSurface", *surface))
            toRemove = ComponentType::NavMeshSurface;
        else
            drawNavMeshSurfaceComponent(object, *surface);
        ImGui::PopID();
    }
    if (SelfDestroy* selfDestroy = object.getComponent<SelfDestroy>())
    {
        ImGui::PushID("SelfDestroy");
        if (drawComponentHeader(app(), "SelfDestroy", *selfDestroy))
            toRemove = ComponentType::SelfDestroy;
        else
            drawSelfDestroyComponent(object, *selfDestroy);
        ImGui::PopID();
    }
    if (Collider* collider = object.getComponent<Collider>())
    {
        ImGui::PushID("Collider");
        if (drawComponentHeader(app(), "Collider", *collider))
            toRemove = ComponentType::Collider;
        else
            drawColliderComponent(object, *collider);
        ImGui::PopID();
    }
    if (Physics::RigidBody* rigidBody = object.getComponent<Physics::RigidBody>())
    {
        ImGui::PushID("RigidBody");
        if (drawComponentHeader(app(), "RigidBody", *rigidBody))
            toRemove = ComponentType::RigidBody;
        else
            drawRigidBodyComponent(object, *rigidBody);
        ImGui::PopID();
    }
    if (Physics::Joint* joint = object.getComponent<Physics::Joint>())
    {
        ImGui::PushID("Joint");
        if (drawComponentHeader(app(), "Joint", *joint))
            toRemove = ComponentType::Joint;
        else
            drawJointComponent(object, *joint);
        ImGui::PopID();
    }
    if (AudioPlayer* audioPlayer = object.getComponent<AudioPlayer>())
    {
        ImGui::PushID("AudioPlayer");
        if (drawComponentHeader(app(), "AudioPlayer", *audioPlayer))
            toRemove = ComponentType::AudioPlayer;
        else
            drawAudioPlayerComponent(*audioPlayer);
        ImGui::PopID();
    }
    if (ZenBehaviour* behaviour = object.findComponent<ZenBehaviour>())
    {
        ImGui::PushID("ZenBehaviour");
        if (drawComponentHeader(app(), "Zen Behaviour", *behaviour))
            toRemove = ComponentType::Script;
        else
            drawZenBehaviourComponent(object, *behaviour);
        ImGui::PopID();
    }
    if (BoneAttachment* attachment = object.getComponent<BoneAttachment>())
    {
        ImGui::PushID("BoneAttachment");
        if (drawComponentHeader(app(), "BoneAttachment", *attachment))
            toRemove = ComponentType::BoneAttachment;
        else
            drawBoneAttachmentComponent(*attachment);
        ImGui::PopID();
    }
    if (Orbit* orbit = object.getComponent<Orbit>())
    {
        ImGui::PushID("Orbit");
        if (drawComponentHeader(app(), "Orbit", *orbit))
            toRemove = ComponentType::Orbit;
        else
            drawOrbitComponent(object, *orbit);
        ImGui::PopID();
    }
    if (Maya* maya = object.getComponent<Maya>())
    {
        ImGui::PushID("Maya");
        if (drawComponentHeader(app(), "Maya", *maya))
            toRemove = ComponentType::Maya;
        else
            drawMayaComponent(object, *maya);
        ImGui::PopID();
    }
    if (ThirdPerson* thirdPerson = object.getComponent<ThirdPerson>())
    {
        ImGui::PushID("ThirdPerson");
        if (drawComponentHeader(app(), "ThirdPerson", *thirdPerson))
            toRemove = ComponentType::ThirdPerson;
        else
            drawThirdPersonComponent(object, *thirdPerson);
        ImGui::PopID();
    }
    if (Animator* animator = object.getComponent<Animator>())
    {
        ImGui::PushID("Animator");
        if (drawComponentHeader(app(), "Animator", *animator))
            toRemove = ComponentType::Animator;
        else
            drawAnimatorComponent(*animator);
        ImGui::PopID();
    }
    if (ParticleEffect* effect = object.getComponent<ParticleEffect>())
    {
        ImGui::PushID("ParticleEffect");
        if (drawComponentHeader(app(), "ParticleEffect", *effect))
            toRemove = ComponentType::ParticleEffect;
        else
            drawParticleEffectComponent(*effect);
        ImGui::PopID();
    }
    if (ParticleEmitter* emitter = object.getComponent<ParticleEmitter>())
    {
        ImGui::PushID("ParticleEmitter");
        if (drawComponentHeader(app(), "ParticleEmitter", *emitter))
            toRemove = ComponentType::ParticleEmitter;
        else
            drawParticleEmitterComponent(*emitter);
        ImGui::PopID();
    }
    const auto markerWarning = []()
    {
        ImGui::TextDisabled("Properties are not editor-serializable yet; only Enabled is saved.");
    };
    if (Terrain* component = object.getComponent<Terrain>())
    {
        ImGui::PushID("Terrain");
        if (drawComponentHeader(app(), "Terrain", *component))
            toRemove = ComponentType::Terrain;
        else
            drawTerrainComponent(*component);
        ImGui::PopID();
    }
    if (Landscape* component = object.getComponent<Landscape>())
    {
        ImGui::PushID("Landscape");
        if (drawComponentHeader(app(), "Landscape", *component))
            toRemove = ComponentType::Landscape;
        else
            markerWarning();
        ImGui::PopID();
    }
    if (Road* component = object.getComponent<Road>())
    {
        ImGui::PushID("Road");
        if (drawComponentHeader(app(), "Road", *component))
            toRemove = ComponentType::Road;
        else
            drawRoadComponent(object, *component);
        ImGui::PopID();
    }
    if (Grass* component = object.getComponent<Grass>())
    {
        ImGui::PushID("Grass");
        if (drawComponentHeader(app(), "Grass", *component))
            toRemove = ComponentType::Grass;
        else
            drawGrassComponent(*component);
        ImGui::PopID();
    }
    if (TiledTerrain* component = object.getComponent<TiledTerrain>())
    {
        ImGui::PushID("TiledTerrain");
        if (drawComponentHeader(app(), "TiledTerrain", *component))
            toRemove = ComponentType::TiledTerrain;
        else
            drawTiledTerrainComponent(*component);
        ImGui::PopID();
    }
    if (Hair* component = object.getComponent<Hair>())
    {
        ImGui::PushID("Hair");
        if (drawComponentHeader(app(), "Hair", *component))
            toRemove = ComponentType::Hair;
        else
            drawHairComponent(*component);
        ImGui::PopID();
    }
    if (Forest* component = object.getComponent<Forest>())
    {
        ImGui::PushID("Forest");
        if (drawComponentHeader(app(), "Forest", *component))
            toRemove = ComponentType::Forest;
        else
            drawForestComponent(*component);
        ImGui::PopID();
    }
    if (Ocean* component = object.getComponent<Ocean>())
    {
        ImGui::PushID("Ocean");
        if (drawComponentHeader(app(), "Ocean", *component))
            toRemove = ComponentType::Ocean;
        else
            drawOceanComponent(*component);
        ImGui::PopID();
    }
    if (VoxelWorldComponent* component = object.getComponent<VoxelWorldComponent>())
    {
        ImGui::PushID("VoxelWorld");
        if (drawComponentHeader(app(), "Voxel World", *component))
            toRemove = ComponentType::VoxelWorld;
        else
            drawVoxelWorldComponent(*component);
        ImGui::PopID();
    }

    if (toRemove != ComponentType::Count)
    {
        removeComponentByType(object, toRemove);
        app().markDirty();
    }

    ImGui::Separator();
    drawAddComponentSection(object);
}

void InspectorPanel::drawTerrainComponent(Terrain& terrain)
{
    bool changed = false;
    ImGui::Indent(14.0f);
    ImGui::TextUnformatted("Heightmap");
    ImGui::Button(terrain.heightmapFile().empty() ? "Drop image asset here" : terrain.heightmapFile().c_str(),
                  ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            const std::string path(static_cast<const char*>(payload->Data), payload->DataSize);
            if (terrain.loadFile(path.c_str(), terrain.cellSize(), terrain.heightScale(),
                                  glm::max(1u, terrain.maxLod()), terrain.uvTiles()))
                changed = true;
        }
        ImGui::EndDragDropTarget();
    }
    ImGui::Text("Heightmap: %s", terrain.heightmapFile().empty() ? "(memory)" : terrain.heightmapFile().c_str());
    ImGui::Text("Size: %u x %u", terrain.width(), terrain.height());
    ImGui::Text("Patches: %u", terrain.patchCount());
    ImGui::Text("Triangles: %u", terrain.triangleCount());
    ImGui::Text("Cell size: %.3f", terrain.cellSize());
    ImGui::Text("Height scale: %.3f", terrain.heightScale());
    f32 uvTiles = terrain.uvTiles();
    if (ImGui::DragFloat("UV tiles", &uvTiles, 0.1f, 0.01f, 10000.0f))
    {
        terrain.setUvTiles(uvTiles);
        changed = true;
    }
    ImGui::Text("Max LOD: %u", terrain.maxLod());
    bool receiveShadows = terrain.receivesShadows();
    if (ImGui::Checkbox("Receive shadows", &receiveShadows))
    {
        terrain.setReceiveShadows(receiveShadows);
        changed = true;
    }
    Material& material = terrain.material();
    if (ImGui::CollapsingHeader("Terrain surface", ImGuiTreeNodeFlags_DefaultOpen))
    {
        changed |= ImGui::ColorEdit3("Tint", &material.params.baseColor.x);
        changed |= ImGui::SliderFloat("Roughness", &material.params.surface.x, 0.04f, 1.0f);
        changed |= ImGui::SliderFloat("Low layer end", &material.params.custom0.x, 0.01f, 0.5f);
        changed |= ImGui::SliderFloat("High layer start", &material.params.custom0.y, 0.5f, 0.99f);
        changed |= ImGui::SliderFloat("Rock slope start", &material.params.custom0.z, 0.0f, 0.8f);
        changed |= ImGui::SliderFloat("Rock slope end", &material.params.custom0.w,
                                      material.params.custom0.z + 0.001f, 1.0f);
        changed |= ImGui::DragFloat("Macro tiles", &material.params.custom1.x, 0.05f,
                                    0.01f, 1000.0f);
        changed |= ImGui::SliderFloat("Macro strength", &material.params.custom1.y, 0.0f, 1.0f);
        changed |= ImGui::SliderFloat("Splat strength", &material.params.custom1.z, 0.0f, 1.0f);
        changed |= ImGui::DragFloat("Rock world scale", &material.params.custom1.w, 0.005f,
                                    0.001f, 100.0f);
        changed |= drawFlagCheckbox("Cast shadow", MaterialCastShadow, material);
        ImGui::SameLine();
        changed |= drawFlagCheckbox("Receive shadow", MaterialReceiveShadow, material);

        ImGui::TextDisabled("Automatic weights: low / base / cliff / high altitude");
        changed |= drawTextureSlot(app(), "Base / grass", SlotAlbedo, material);
        changed |= drawTextureSlot(app(), "Cliff / rock", SlotNormal, material);
        changed |= drawTextureSlot(app(), "Low / sand", SlotSurface, material);
        changed |= drawTextureSlot(app(), "High / snow", SlotDetail, material);
        changed |= drawTextureSlot(app(), "RGBA splat map", SlotColorMap, material);
        changed |= drawTextureSlot(app(), "Macro colour", SlotHeight, material);
    }

    if (ImGui::CollapsingHeader("Vegetation generation"))
    {
        if (ImGui::CollapsingHeader("Terrain asset maps", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextDisabled("Height, surface splat e vegetacao sao guardados como PNG/RGBA");
            ImGui::Text("Heightmap: %s", terrain.heightmapFile().empty()
                                           ? "(memoria)"
                                           : terrain.heightmapFile().c_str());
            ImGui::Text("Splat: %s", terrain.surfaceSplatFile().empty()
                                        ? (terrain.hasSurfaceSplat() ? "(editada)" : "(none)")
                                        : terrain.surfaceSplatFile().c_str());
            ImGui::Text("Vegetacao: %s", terrain.vegetationMaskFile().empty()
                                            ? (terrain.hasVegetationMask() ? "(editada)" : "(none)")
                                            : terrain.vegetationMaskFile().c_str());

            ImGui::Button("Drop surface splat RGBA", ImVec2(-FLT_MIN, 0.0f));
            if (ImGui::BeginDragDropTarget())
            {
                if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
                {
                    const std::string path(static_cast<const char*>(payload->Data), payload->DataSize);
                    app().recordUndo();
                    changed |= terrain.loadSurfaceSplat(path.c_str());
                }
                ImGui::EndDragDropTarget();
            }
            ImGui::Button("Drop vegetation mask RGBA", ImVec2(-FLT_MIN, 0.0f));
            if (ImGui::BeginDragDropTarget())
            {
                if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
                {
                    const std::string path(static_cast<const char*>(payload->Data), payload->DataSize);
                    app().recordUndo();
                    changed |= terrain.loadVegetationMask(path.c_str());
                }
                ImGui::EndDragDropTarget();
            }
            if (ImGui::Button("Create automatic splat"))
            {
                app().recordUndo();
                changed |= terrain.createSurfaceSplat();
            }
            ImGui::SameLine();
            if (ImGui::Button("Create blank vegetation mask"))
            {
                app().recordUndo();
                changed |= terrain.createVegetationMask();
            }
            if (terrain.owner() && ImGui::Button("Save Terrain Assets", ImVec2(-FLT_MIN, 0.0f)))
            {
                const std::filesystem::path directory =
                    std::filesystem::path(RADION_ASSET_DIR) / "terrain";
                std::error_code error;
                std::filesystem::create_directories(directory, error);
                const std::string stem = "terrain_" + std::to_string(terrain.owner()->id());
                const std::filesystem::path height = directory / (stem + "_height.png");
                const std::filesystem::path splat = directory / (stem + "_splat.png");
                const std::filesystem::path vegetation = directory / (stem + "_vegetation.png");
                bool saved = terrain.saveHeightmap(height.string().c_str());
                if (terrain.hasSurfaceSplat())
                    saved = terrain.saveSurfaceSplat(splat.string().c_str()) && saved;
                if (terrain.hasVegetationMask())
                    saved = terrain.saveVegetationMask(vegetation.string().c_str()) && saved;
                if (saved)
                {
                    app().markDirty();
                    Log::info("Terrain assets saved for object %llu", terrain.owner()->id());
                }
                else
                    Log::error("Could not save Terrain assets for object %llu", terrain.owner()->id());
            }
        }

        ImGui::Separator();
        ImGui::TextUnformatted("RGBA vegetation mask");
        ImGui::TextDisabled("R grass, G flowers, B bushes, A trees");
        ImGui::Button(terrain.vegetationMaskFile().empty()
                          ? (terrain.hasVegetationMask() ? "Unsaved painted mask" : "Drop mask image here")
                          : terrain.vegetationMaskFile().c_str(),
                      ImVec2(-FLT_MIN, 0.0f));
        if (ImGui::BeginDragDropTarget())
        {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
            {
                const std::string path(static_cast<const char*>(payload->Data), payload->DataSize);
                if (terrain.loadVegetationMask(path.c_str()))
                    changed = true;
            }
            ImGui::EndDragDropTarget();
        }
        if (ImGui::Button("New blank mask"))
        {
            app().recordUndo();
            changed |= terrain.createVegetationMask();
        }
        ImGui::BeginDisabled(!terrain.hasVegetationMask());
        ImGui::SameLine();
        if (ImGui::Button("Grass full"))
        {
            app().recordUndo();
            terrain.fillVegetation(Terrain::VegetationChannel::Grass, 1.0f);
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Grass clear"))
        {
            app().recordUndo();
            terrain.fillVegetation(Terrain::VegetationChannel::Grass, 0.0f);
            changed = true;
        }
        if (ImGui::Button("Trees full"))
        {
            app().recordUndo();
            terrain.fillVegetation(Terrain::VegetationChannel::Trees, 1.0f);
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Trees clear"))
        {
            app().recordUndo();
            terrain.fillVegetation(Terrain::VegetationChannel::Trees, 0.0f);
            changed = true;
        }
        ImGui::EndDisabled();
        ImGui::Separator();

        auto drawSettings = [&](const char* label, Terrain::VegetationSettings& settings) {
            bool settingsChanged = false;
            ImGui::PushID(label);
            ImGui::TextUnformatted(label);
            settingsChanged |= ImGui::DragFloat("Spacing", &settings.spacing, 0.1f, 0.05f, 10000.0f);
            settingsChanged |= ImGui::SliderFloat("Density", &settings.density, 0.0f, 1.0f);
            settingsChanged |= ImGui::SliderFloat("Jitter", &settings.jitter, 0.0f, 1.0f);
            settingsChanged |= ImGui::DragFloatRange2("Height range", &settings.minimumHeight,
                                                      &settings.maximumHeight, 0.25f,
                                                      -1000000.0f, 1000000.0f);
            settingsChanged |= ImGui::SliderFloat("Maximum slope", &settings.maximumSlopeDegrees,
                                                  0.0f, 89.0f, "%.1f deg");
            settingsChanged |= ImGui::DragFloatRange2("Scale range", &settings.minimumScale,
                                                      &settings.maximumScale, 0.01f,
                                                      0.01f, 100.0f);
            settingsChanged |= ImGui::InputScalar("Seed", ImGuiDataType_U32, &settings.seed);
            settingsChanged |= ImGui::InputScalar("Maximum instances", ImGuiDataType_U32,
                                                  &settings.maximumInstances);
            ImGui::PopID();
            return settingsChanged;
        };

        changed |= drawSettings("Grass", terrain.grassGenerationSettings());
        Grass* grass = terrain.owner() ? terrain.owner()->getComponent<Grass>() : nullptr;
        if (!grass)
            ImGui::TextDisabled("Add a Grass component to this object and configure its atlas.");
        else if (grass->regionCount() == 0)
            ImGui::TextDisabled("Grass needs at least one atlas region before generation.");
        ImGui::BeginDisabled(!grass || grass->regionCount() == 0);
        if (ImGui::Button("Generate grass", ImVec2(-FLT_MIN, 0.0f)))
        {
            app().recordUndo();
            terrain.generateGrass(*grass, true);
            changed = true;
        }
        ImGui::EndDisabled();

        ImGui::Separator();
        changed |= drawSettings("Trees", terrain.treeGenerationSettings());
        Forest* forest = terrain.owner() ? terrain.owner()->getComponent<Forest>() : nullptr;
        if (!forest)
            ImGui::TextDisabled("Add a Forest component to this object and add tree species.");
        else if (forest->speciesCount() == 0)
            ImGui::TextDisabled("Forest needs at least one species before generation.");
        ImGui::BeginDisabled(!forest || forest->speciesCount() == 0);
        if (ImGui::Button("Generate trees", ImVec2(-FLT_MIN, 0.0f)))
        {
            app().recordUndo();
            terrain.generateTrees(*forest, true);
            changed = true;
        }
        ImGui::EndDisabled();
    }
    if (changed)
    {
        material.flags |= MaterialLit | MaterialTerrain;
        material.paramsDirty = true;
        app().markDirty();
    }
    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawVoxelWorldComponent(VoxelWorldComponent& voxelWorld)
{
    ImGui::Indent(14.0f);
    bool changed = false;

    ImGui::SeparatorText("World");
    int seed = static_cast<int>(voxelWorld.seed());
    if (ImGui::InputInt("Seed", &seed))
    {
        voxelWorld.setSeed(static_cast<u32>(std::max(0, seed)));
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Terrain generator seed. The same seed and settings reproduce the "
                          "same world.");
    int radius = voxelWorld.chunkRadius();
    if (ImGui::DragInt("Chunk radius", &radius, 1.0f, 0, 32))
    {
        voxelWorld.setChunkRadius(radius);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Chunks kept meshed around the origin (VoxelStreamer view radius). "
                          "Generation itself runs one ring wider than this.");
    int originId = static_cast<int>(voxelWorld.originObjectId());
    if (ImGui::InputInt("Origin GameObject ID", &originId))
    {
        voxelWorld.setOriginObjectId(static_cast<u64>(std::max(0, originId)));
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("GameObject the streaming origin follows every frame. Zero falls back "
                          "to this object.");
    char atlas[256];
    std::snprintf(atlas, sizeof(atlas), "%s", voxelWorld.atlasFile().c_str());
    if (ImGui::InputText("Atlas file", atlas, sizeof(atlas)))
    {
        voxelWorld.setAtlasFile(atlas);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Texture atlas the block faces sample from. Changing it rebuilds the "
                          "three voxel materials.");
    char editsFile[256];
    std::snprintf(editsFile, sizeof(editsFile), "%s", voxelWorld.editsFile().c_str());
    if (ImGui::InputText("Edits file", editsFile, sizeof(editsFile)))
    {
        voxelWorld.setEditsFile(editsFile);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Where the edited blocks are written, beside the scene. The terrain "
                          "itself comes back from the seed, so only what somebody changed by "
                          "hand needs a file.");
    if (ImGui::Button("Save edits"))
    {
        if (voxelWorld.saveEdits(voxelWorld.editsFile().c_str()))
            changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Load edits"))
    {
        if (voxelWorld.loadEdits(voxelWorld.editsFile().c_str()))
            changed = true;
    }
    ImGui::SameLine();
    ImGui::Text("%zu blocks", voxelWorld.editedBlocks());

    ImGui::SeparatorText("Block palette");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("What the world is made of. Each block names its tile in the atlas per "
                          "face, so a texture pack is a change of coordinates and not of code. "
                          "Ids are positional: editing a block keeps everything already placed.");
    for (Voxel::BlockId id = 1; id < voxelWorld.blockCount(); ++id)
    {
        const Voxel::BlockDefinition* current = voxelWorld.blockDefinition(id);
        if (!current)
            continue;
        ImGui::PushID(static_cast<int>(id));
        if (ImGui::TreeNode(current->name.c_str()))
        {
            Voxel::BlockDefinition edited = *current;
            bool blockChanged = false;

            char name[64];
            std::snprintf(name, sizeof(name), "%s", edited.name.c_str());
            if (ImGui::InputText("Name", name, sizeof(name)))
            {
                edited.name = name;
                blockChanged = true;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("The terrain generator asks for its blocks by name: grass, "
                                  "dirt, stone, sand, gravel, bedrock, water, snow, log, leaves "
                                  "and the four ores. A name it cannot find is simply left out "
                                  "of the world.");

            const char* renderTypes[] = {"Opaque", "Cutout", "Transparent"};
            int renderType = static_cast<int>(edited.renderType);
            if (ImGui::Combo("Render", &renderType, renderTypes, 3))
            {
                edited.renderType = static_cast<Voxel::BlockRenderType>(renderType);
                edited.transparent = edited.renderType != Voxel::BlockRenderType::Opaque;
                blockChanged = true;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Opaque hides what is behind it and hides its neighbours' "
                                  "faces. Cutout keeps depth and drops the atlas's empty texels, "
                                  "for leaves. Transparent blends and writes no depth, for "
                                  "water.");
            if (ImGui::Checkbox("Solid", &edited.solid))
                blockChanged = true;
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Whether a ray stops on it and whether it will hold a player "
                                  "up. Water is not solid.");

            int tile[2] = {static_cast<int>(edited.faces[0].atlasX),
                           static_cast<int>(edited.faces[0].atlasY)};
            if (ImGui::DragInt2("All faces", tile, 0.1f, 0, 31))
            {
                for (Voxel::BlockFaceMaterial& face : edited.faces)
                {
                    face.atlasX = static_cast<u16>(tile[0]);
                    face.atlasY = static_cast<u16>(tile[1]);
                }
                blockChanged = true;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Atlas column and row for every face at once. terrain.png is "
                                  "16x16 tiles: grass top is 0,0, its side 3,0, dirt 2,0, stone "
                                  "1,0, sand 2,1, log side 4,1, leaves 4,3, water 13,12.");

            const char* faceNames[] = {"-X", "+X", "Bottom", "Top", "-Z", "+Z"};
            for (usize face = 0; face < edited.faces.size(); ++face)
            {
                int faceTile[2] = {static_cast<int>(edited.faces[face].atlasX),
                                   static_cast<int>(edited.faces[face].atlasY)};
                ImGui::PushID(static_cast<int>(face));
                if (ImGui::DragInt2(faceNames[face], faceTile, 0.1f, 0, 31))
                {
                    edited.faces[face].atlasX = static_cast<u16>(faceTile[0]);
                    edited.faces[face].atlasY = static_cast<u16>(faceTile[1]);
                    blockChanged = true;
                }
                ImGui::PopID();
            }

            if (blockChanged)
            {
                voxelWorld.setBlockDefinition(id, edited);
                changed = true;
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
    if (ImGui::Button("Add block"))
    {
        Voxel::BlockDefinition definition;
        definition.name = "block " + std::to_string(voxelWorld.blockCount());
        voxelWorld.addBlock(definition);
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset palette"))
    {
        voxelWorld.resetBlocksToDefault();
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Back to the fourteen blocks the engine ships, on terrain.png's own "
                          "tiles.");
    if (ImGui::Button("Regenerate world"))
    {
        app().recordUndo();
        voxelWorld.regenerate();
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reapplies terrain and streaming settings, which drops every loaded "
                          "chunk and streams it back in around the origin.");

    ImGui::SeparatorText("Terrain");
    bool flat = voxelWorld.flat();
    if (ImGui::Checkbox("Flat world", &flat))
    {
        voxelWorld.setFlat(flat);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("A table at the base surface height, with the noise ignored - somewhere "
                          "to build on. Caves, ores and trees keep their own switches, so a clean "
                          "slab means turning those off too.");
    int minY = voxelWorld.minWorldY();
    if (ImGui::InputInt("Min world Y", &minY))
    {
        voxelWorld.setMinWorldY(minY);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Lowest world Y the vertical chunk band covers. Clamped below max "
                          "world Y.");
    int maxY = voxelWorld.maxWorldY();
    if (ImGui::InputInt("Max world Y", &maxY))
    {
        voxelWorld.setMaxWorldY(maxY);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Highest world Y the vertical chunk band covers. Clamped above min "
                          "world Y.");
    int waterLevel = voxelWorld.waterLevel();
    if (ImGui::InputInt("Water level", &waterLevel))
    {
        voxelWorld.setWaterLevel(waterLevel);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("World Y at and below which air becomes water.");
    f32 baseSurfaceHeight = voxelWorld.baseSurfaceHeight();
    if (ImGui::DragFloat("Base surface height", &baseSurfaceHeight, 0.5f, -1000.0f, 1000.0f))
    {
        voxelWorld.setBaseSurfaceHeight(baseSurfaceHeight);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Surface height with zero contribution from the continental and "
                          "detail noise layers.");
    f32 continentalAmplitude = voxelWorld.continentalAmplitude();
    if (ImGui::DragFloat("Continental amplitude", &continentalAmplitude, 0.5f, 0.0f, 500.0f))
    {
        voxelWorld.setContinentalAmplitude(continentalAmplitude);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Low frequency height variation added on top of the base surface "
                          "height.");
    f32 detailAmplitude = voxelWorld.detailAmplitude();
    if (ImGui::DragFloat("Detail amplitude", &detailAmplitude, 0.1f, 0.0f, 200.0f))
    {
        voxelWorld.setDetailAmplitude(detailAmplitude);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("High frequency height variation layered over the continental shape.");
    int minSurfaceHeight = voxelWorld.minSurfaceHeight();
    if (ImGui::DragInt("Min surface height", &minSurfaceHeight, 1.0f, -1000, 1000))
    {
        voxelWorld.setMinSurfaceHeight(minSurfaceHeight);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Clamp floor for the generated surface height. Clamped below max "
                          "surface height.");
    int maxSurfaceHeight = voxelWorld.maxSurfaceHeight();
    if (ImGui::DragInt("Max surface height", &maxSurfaceHeight, 1.0f, -1000, 1000))
    {
        voxelWorld.setMaxSurfaceHeight(maxSurfaceHeight);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Clamp ceiling for the generated surface height. Clamped above min "
                          "surface height.");
    f32 reliefFrequency = voxelWorld.reliefFrequency();
    if (ImGui::DragFloat("Relief frequency", &reliefFrequency, 0.0001f, 0.0001f, 0.05f, "%.4f"))
    {
        voxelWorld.setReliefFrequency(reliefFrequency);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How fast flat country turns into mountain range. Lower means larger "
                          "regions of the same character; it scales the continental amplitude "
                          "rather than adding height, so no biome border shows a step.");

    ImGui::SeparatorText("Biomes");
    bool biomes = voxelWorld.biomes();
    if (ImGui::Checkbox("Biomes", &biomes))
    {
        voxelWorld.setBiomes(biomes);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Off makes the whole world plains: grass surface, dirt filler. Use it "
                          "to isolate a terrain problem from a biome one.");
    ImGui::BeginDisabled(!biomes);
    f32 biomeFrequency = voxelWorld.biomeFrequency();
    if (ImGui::DragFloat("Biome frequency", &biomeFrequency, 0.0002f, 0.0005f, 0.05f, "%.4f"))
    {
        voxelWorld.setBiomeFrequency(biomeFrequency);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Voronoi cell size for the biome map, in blocks per cell = 1 / this. "
                          "0.0045 gives cells about 220 blocks across.");
    ImGui::EndDisabled();

    ImGui::SeparatorText("Caves");
    bool caves = voxelWorld.caves();
    if (ImGui::Checkbox("Caves", &caves))
    {
        voxelWorld.setCaves(caves);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Carves tunnels out of the generated ground. Off leaves it solid.");
    ImGui::BeginDisabled(!caves);
    f32 caveFrequency = voxelWorld.caveFrequency();
    if (ImGui::DragFloat("Cave frequency", &caveFrequency, 0.001f, 0.005f, 0.2f, "%.3f"))
    {
        voxelWorld.setCaveFrequency(caveFrequency);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Tunnel scale. Higher means tighter, more tangled tunnels.");
    f32 caveThreshold = voxelWorld.caveThreshold();
    if (ImGui::DragFloat("Cave threshold", &caveThreshold, 0.005f, 0.5f, 0.99f, "%.3f"))
    {
        voxelWorld.setCaveThreshold(caveThreshold);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Where the two ridged noise fields have to meet before a tunnel opens. "
                          "Lower carves more; 0.86 removes about 6% of the underground.");
    int caveCeiling = voxelWorld.caveCeiling();
    if (ImGui::DragInt("Cave ceiling", &caveCeiling, 1.0f, 0, 32))
    {
        voxelWorld.setCaveCeiling(caveCeiling);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Blocks of ground kept under the surface. Zero lets tunnels break "
                          "through and open cave mouths.");
    ImGui::EndDisabled();

    ImGui::SeparatorText("Vegetation and ores");
    bool trees = voxelWorld.trees();
    if (ImGui::Checkbox("Trees", &trees))
    {
        voxelWorld.setTrees(trees);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Needs the log and leaves blocks in the registry; without them no tree "
                          "is placed whatever this says.");
    ImGui::BeginDisabled(!trees);
    f32 treeDensity = voxelWorld.treeDensity();
    if (ImGui::DragFloat("Tree density", &treeDensity, 0.05f, 0.0f, 8.0f, "%.2f"))
    {
        voxelWorld.setTreeDensity(treeDensity);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Multiplies each biome's own density: forest 0.045 per column, plains "
                          "0.006, snow 0.018, mountains 0.004, desert none.");
    ImGui::EndDisabled();
    bool ores = voxelWorld.ores();
    if (ImGui::Checkbox("Ores", &ores))
    {
        voxelWorld.setOres(ores);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Replaces stone with coal, iron, gold and diamond in 2x2x2 pockets, "
                          "each with its own depth band.");
    bool ambientOcclusion = voxelWorld.ambientOcclusion();
    if (ImGui::Checkbox("Ambient occlusion", &ambientOcclusion))
    {
        voxelWorld.setAmbientOcclusion(ambientOcclusion);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Per-vertex corner shading baked by the mesher. It also enters the "
                          "greedy merge key, so it costs geometry: measured at +75% vertices "
                          "over a chunk radius of six. Turn it off to see what that buys.");

    ImGui::SeparatorText("Bounds");
    bool bounded = voxelWorld.bounded();
    if (ImGui::Checkbox("Bounded", &bounded))
    {
        voxelWorld.setBounded(bounded);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Restricts streaming to a fixed X/Z chunk box instead of an endless "
                          "world.");
    ImGui::BeginDisabled(!bounded);
    int boundsMinX = voxelWorld.boundsMinX();
    if (ImGui::DragInt("Bounds min X", &boundsMinX, 1.0f, -10000, 10000))
    {
        voxelWorld.setBoundsMinX(boundsMinX);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Lowest chunk X inside the box. Clamped below bounds max X.");
    int boundsMaxX = voxelWorld.boundsMaxX();
    if (ImGui::DragInt("Bounds max X", &boundsMaxX, 1.0f, -10000, 10000))
    {
        voxelWorld.setBoundsMaxX(boundsMaxX);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Highest chunk X inside the box. Clamped above bounds min X.");
    int boundsMinZ = voxelWorld.boundsMinZ();
    if (ImGui::DragInt("Bounds min Z", &boundsMinZ, 1.0f, -10000, 10000))
    {
        voxelWorld.setBoundsMinZ(boundsMinZ);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Lowest chunk Z inside the box. Clamped below bounds max Z.");
    int boundsMaxZ = voxelWorld.boundsMaxZ();
    if (ImGui::DragInt("Bounds max Z", &boundsMaxZ, 1.0f, -10000, 10000))
    {
        voxelWorld.setBoundsMaxZ(boundsMaxZ);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Highest chunk Z inside the box. Clamped above bounds min Z.");
    ImGui::EndDisabled();

    ImGui::SeparatorText("Streaming");
    int maxUploadsPerFrame = static_cast<int>(voxelWorld.maxUploadsPerFrame());
    if (ImGui::SliderInt("Max uploads per frame", &maxUploadsPerFrame, 1, 64))
    {
        voxelWorld.setMaxUploadsPerFrame(static_cast<u32>(maxUploadsPerFrame));
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("GPU mesh uploads accepted per update; caps the per frame streaming "
                          "cost.");
    int maxGenerationJobs = static_cast<int>(voxelWorld.maxGenerationJobs());
    if (ImGui::SliderInt("Max generation jobs", &maxGenerationJobs, 1, 64))
    {
        voxelWorld.setMaxGenerationJobs(static_cast<u32>(maxGenerationJobs));
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Chunk generation jobs allowed in flight at the same time.");
    int maxMeshJobs = static_cast<int>(voxelWorld.maxMeshJobs());
    if (ImGui::SliderInt("Max mesh jobs", &maxMeshJobs, 1, 64))
    {
        voxelWorld.setMaxMeshJobs(static_cast<u32>(maxMeshJobs));
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Chunk meshing jobs allowed in flight at the same time.");

    ImGui::Text("Loaded chunks: %zu", voxelWorld.loadedChunks());
    ImGui::Text("Pending generation: %zu", voxelWorld.pendingGeneration());
    ImGui::Text("Pending meshing: %zu", voxelWorld.pendingMeshing());
    ImGui::Text("Queued meshes: %zu", voxelWorld.queuedMeshes());
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Pending counts that keep climbing instead of settling, or a loaded "
                          "count that drifts while the camera is still, mean streaming is not "
                          "keeping up.");

    if (changed)
        app().markDirty();
    ImGui::Unindent(14.0f);
}

bool drawOceanTextureSlot(EditorApplication& app, const char* label, Ocean& ocean, bool normal)
{
    ImGui::PushID(label);
    const std::string& current = normal ? ocean.normalMapFile() : ocean.foamTextureFile();
    ImGui::Button(current.empty() ? "(drop asset here)" : current.c_str(), ImVec2(-FLT_MIN, 0.0f));
    bool changed = false;
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            const std::string path(static_cast<const char*>(payload->Data), payload->DataSize);
            if (normal)
                ocean.setNormalMapFile(path);
            else
                ocean.setFoamTextureFile(path);
            changed = true;
        }
        ImGui::EndDragDropTarget();
    }
    ImGui::PopID();
    if (changed)
        app.markDirty();
    return changed;
}

void InspectorPanel::drawTiledTerrainComponent(TiledTerrain& terrain)
{
    bool changed = false;
    ImGui::Indent(14.0f);

    int tilesInSide = terrain.tilesInSide();
    if (ImGui::DragInt("Tiles in side", &tilesInSide, 1.0f, 1, 256))
    {
        terrain.setTilesInSide(tilesInSide);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Atlas grid size - the atlas texture is read as a tilesInSide x "
                          "tilesInSide grid, each tile ID indexing into it row-major.");

    int tilesPerPatch = terrain.tilesPerPatch();
    if (ImGui::DragInt("Tiles per patch", &tilesPerPatch, 1.0f, 1, 256))
    {
        terrain.setTilesPerPatch(tilesPerPatch);
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Tiles per submesh. The scene culls one patch at a time, so a smaller "
                          "patch culls more precisely at the cost of more submeshes.");

    f32 patchLength = terrain.patchLength();
    if (ImGui::DragFloat("Patch length", &patchLength, 0.01f, 0.001f, 10000.0f))
    {
        terrain.setPatchLength(patchLength);
        changed = true;
    }

    int defaultTile = terrain.defaultTile();
    if (ImGui::DragInt("Default tile", &defaultTile, 1.0f, 0, 255))
    {
        terrain.setDefaultTile(static_cast<u8>(defaultTile));
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Tile ID a patch overhanging the map edge samples, and what a freshly "
                          "built tilemap starts filled with.");

    ImGui::TextUnformatted("Atlas material");
    ImGui::Button(terrain.atlasMaterial().empty() ? "Drop material asset here"
                                                  : terrain.atlasMaterial().c_str(),
                  ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            const std::string path(static_cast<const char*>(payload->Data), payload->DataSize);
            terrain.setAtlasMaterial(path);
            changed = true;
        }
        ImGui::EndDragDropTarget();
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Map size");
    static int newWidth = 8;
    static int newHeight = 8;
    ImGui::DragInt("Width##TiledTerrainMapWidth", &newWidth, 1.0f, 1, 4096);
    ImGui::DragInt("Height##TiledTerrainMapHeight", &newHeight, 1.0f, 1, 4096);
    if (ImGui::Button("Build Tilemap", ImVec2(-FLT_MIN, 0.0f)))
    {
        app().recordUndo();
        const u32 width = static_cast<u32>(newWidth);
        const u32 height = static_cast<u32>(newHeight);
        std::vector<u8> tiles(static_cast<usize>(width) * height, terrain.defaultTile());
        terrain.loadTilemap(width, height, tiles.data());
        changed = true;
    }
    ImGui::Text("Current: %u x %u", terrain.mapWidth(), terrain.mapHeight());

    if (changed)
        app().markDirty();
    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawRoadComponent(GameObject& object, Road& road)
{
    ImGui::Indent(14.0f);
    bool changed = false;
    int subdivisions = static_cast<int>(road.subdivisions());
    if (ImGui::DragInt("Subdivisions", &subdivisions, 1.0f, 1, 64)) { road.setSubdivisions(static_cast<u32>(subdivisions)); changed = true; }
    f32 value = road.textureRepeat();
    if (ImGui::DragFloat("Texture repeat", &value, 0.01f, 0.1f, 10000.0f)) { road.setTextureRepeat(value); changed = true; }
    value = road.surfaceOffset();
    if (ImGui::DragFloat("Surface offset", &value, 0.001f, -100.0f, 100.0f)) { road.setSurfaceOffset(value); changed = true; }
    bool conform = road.conformTerrain();
    if (ImGui::Checkbox("Conform terrain", &conform)) { road.setConformTerrain(conform); changed = true; }
    ImGui::Text("Terrain: %s", road.terrain() && road.terrain()->owner()
                                  ? road.terrain()->owner()->name().c_str() : "(none)");
    ImGui::Button("Drop terrain object here", ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kGameObjectDragPayload))
        {
            const u64 id = *static_cast<const u64*>(payload->Data);
            GameObject* terrainObject = app().scene().findGameObject(id);
            Terrain* terrain = terrainObject ? terrainObject->getComponent<Terrain>() : nullptr;
            if (terrain)
            {
                app().recordUndo();
                road.setTerrain(terrain);
                changed = true;
            }
        }
        ImGui::EndDragDropTarget();
    }
    if (ImGui::Button("Clear terrain"))
    {
        app().recordUndo();
        road.setTerrain(nullptr);
        changed = true;
    }
    if (ImGui::TreeNode("Material"))
    {
        changed |= drawMaterialFields(road.material());
        changed |= drawTextureSlot(app(), "Albedo", SlotAlbedo, road.material());
        changed |= drawTextureSlot(app(), "Normal", SlotNormal, road.material());
        changed |= drawTextureSlot(app(), "Surface", SlotSurface, road.material());
        ImGui::TreePop();
    }
    ImGui::Text("Points: %u", static_cast<u32>(road.pointCount()));
    ImGui::Button("Drop point object here", ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kGameObjectDragPayload))
        {
            const u64 id = *static_cast<const u64*>(payload->Data);
            GameObject* point = app().scene().findGameObject(id);
            if (point && point != &object)
            {
                app().recordUndo();
                if (road.addPoint(point)) changed = true;
            }
        }
        ImGui::EndDragDropTarget();
    }
    s32 moveFrom = -1;
    s32 moveTo = -1;
    s32 removeIndex = -1;
    for (usize i = 0; i < road.pointCount(); ++i)
    {
        ImGui::PushID(static_cast<int>(i));
        GameObject* point = road.point(i);
        ImGui::Text("%u: %s", static_cast<u32>(i + 1), point ? point->name().c_str() : "(missing)");
        ImGui::SameLine();
        f32 width = road.pointWidth(i);
        if (ImGui::DragFloat("Width", &width, 0.01f, 0.1f, 1000.0f)) { road.setPointWidth(i, width); changed = true; }
        ImGui::SameLine();
        ImGui::BeginDisabled(i == 0);
        if (ImGui::SmallButton("Up")) { moveFrom = static_cast<s32>(i); moveTo = moveFrom - 1; }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(i + 1 >= road.pointCount());
        if (ImGui::SmallButton("Down")) { moveFrom = static_cast<s32>(i); moveTo = moveFrom + 1; }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::SmallButton("Remove")) removeIndex = static_cast<s32>(i);
        ImGui::PopID();
    }
    if (moveFrom >= 0 && moveTo >= 0)
    {
        app().recordUndo();
        GameObject* point = road.point(static_cast<usize>(moveFrom));
        const f32 width = road.pointWidth(static_cast<usize>(moveFrom));
        road.removePoint(point);
        road.insertPoint(static_cast<usize>(moveTo), point, width);
        changed = true;
    }
    if (removeIndex >= 0)
    {
        app().recordUndo();
        road.removePoint(road.point(static_cast<usize>(removeIndex)));
        changed = true;
    }
    if (road.pointCount() > 0 && ImGui::Button("Clear points", ImVec2(-FLT_MIN, 0.0f)))
    {
        app().recordUndo();
        road.clearPoints();
        changed = true;
    }
    if (changed)
    {
        road.rebuild();
        app().markDirty();
    }
    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawGrassComponent(Grass& grass)
{
    ImGui::Indent(14.0f);
    ImGui::Button(grass.atlasFile().empty() ? "Drop atlas image here" : grass.atlasFile().c_str(),
                  ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            if (grass.loadAtlas(std::string(static_cast<const char*>(payload->Data), payload->DataSize)))
            {
                if (grass.regionCount() == 0)
                    grass.addRegion(GrassAtlasRect());
                app().markDirty();
            }
        }
        ImGui::EndDragDropTarget();
    }
    f32 value = grass.height();
    if (ImGui::DragFloat("Height", &value, 0.01f, 0.001f, 1000.0f)) { grass.setHeight(value); app().markDirty(); }
    value = grass.width();
    if (ImGui::DragFloat("Width", &value, 0.01f, 0.0f, 100.0f)) { grass.setWidth(value); app().markDirty(); }
    value = grass.wind();
    if (ImGui::DragFloat("Wind", &value, 0.01f, 0.0f, 100.0f)) { grass.setWind(value); app().markDirty(); }
    value = grass.alphaCut();
    if (ImGui::SliderFloat("Alpha cut", &value, 0.0f, 1.0f)) { grass.setAlphaCut(value); app().markDirty(); }
    value = grass.cameraBend();
    if (ImGui::SliderFloat("Camera bend", &value, 0.0f, 1.0f)) { grass.setCameraBend(value); app().markDirty(); }
    value = grass.drawDistance();
    if (ImGui::DragFloat("Draw distance", &value, 1.0f, 0.0f, 100000.0f)) { grass.setDrawDistance(value); app().markDirty(); }
    value = grass.stiffness();
    if (ImGui::DragFloat("Stiffness", &value, 0.1f, 0.0f, 1000.0f)) { grass.setStiffness(value); app().markDirty(); }
    value = grass.drag();
    if (ImGui::DragFloat("Drag", &value, 0.01f, 0.0f, 1.0f)) { grass.setDrag(value); app().markDirty(); }
    bool enabled = grass.softFringe();
    if (ImGui::Checkbox("Soft fringe", &enabled)) { grass.setSoftFringe(enabled); app().markDirty(); }
    u32 seed = grass.seed();
    if (ImGui::InputScalar("Seed", ImGuiDataType_U32, &seed)) { grass.setSeed(seed); app().markDirty(); }
    ImGui::Text("Regions: %u", grass.regionCount());
    ImGui::Text("Clumps: %u", grass.count());
    if (ImGui::TreeNode("Atlas regions"))
    {
        if (grass.atlasFile().empty())
            ImGui::TextDisabled("Drop an atlas image above first.");
        for (u32 i = 0; i < grass.regionCount(); ++i)
        {
            u32 x = 0, y = 0, width = 0, height = 0;
            f32 size = 1.0f, weight = 1.0f;
            if (!grass.regionPixels(i, x, y, width, height, size, weight))
                continue;
            ImGui::PushID(static_cast<int>(i));
            ImGui::Text("Region %u", i + 1);
            bool regionChanged = ImGui::InputScalar("X", ImGuiDataType_U32, &x);
            regionChanged |= ImGui::InputScalar("Y", ImGuiDataType_U32, &y);
            regionChanged |= ImGui::InputScalar("Width", ImGuiDataType_U32, &width);
            regionChanged |= ImGui::InputScalar("Height", ImGuiDataType_U32, &height);
            regionChanged |= ImGui::DragFloat("Size", &size, 0.01f, 0.001f, 1000.0f);
            regionChanged |= ImGui::DragFloat("Weight", &weight, 0.01f, 0.0f, 1000.0f);
            if (regionChanged && grass.setRegion(i, x, y, width, height, size, weight))
                app().markDirty();
            ImGui::Separator();
            ImGui::PopID();
        }
        if (ImGui::Button("Add region"))
        {
            u32 x = 0, y = 0, width = 0, height = 0;
            f32 size = 1.0f, weight = 1.0f;
            if (grass.regionCount() > 0 && grass.regionPixels(grass.regionCount() - 1,
                                                               x, y, width, height, size, weight))
                grass.addRegion(x, y, width, height, size, weight);
            else
                grass.addRegion(GrassAtlasRect());
            app().markDirty();
        }
        if (grass.regionCount() > 0 && ImGui::Button("Clear regions"))
        {
            app().recordUndo();
            grass.clearRegions();
            app().markDirty();
        }
        ImGui::TreePop();
    }
    static f32 paintRadius = 20.0f;
    static int paintCount = 100;
    ImGui::DragFloat("Paint radius", &paintRadius, 0.5f, 0.1f, 10000.0f);
    ImGui::DragInt("Paint count", &paintCount, 1.0f, 1, 1000000);
    if (ImGui::Button("Plant grass", ImVec2(-FLT_MIN, 0.0f)))
    {
        glm::vec3 centre = app().cursor3D();
        if (grass.owner())
            centre = glm::vec3(glm::inverse(grass.owner()->globalTransform()) * glm::vec4(centre, 1.0f));
        app().recordUndo();
        if (grass.paint(centre, paintRadius, static_cast<u32>(paintCount)) > 0)
            app().markDirty();
    }
    if (ImGui::Button("Plant one here", ImVec2(-FLT_MIN, 0.0f)))
    {
        glm::vec3 centre = app().cursor3D();
        glm::vec3 normal(0.0f, 1.0f, 0.0f);
        if (grass.owner())
        {
            const glm::mat4 inverseTransform = glm::inverse(grass.owner()->globalTransform());
            centre = glm::vec3(inverseTransform * glm::vec4(centre, 1.0f));
            normal = glm::normalize(glm::vec3(inverseTransform * glm::vec4(normal, 0.0f)));
        }
        app().recordUndo();
        if (grass.plant(centre, normal))
            app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Plants exactly one tuft at the 3D cursor, no scatter radius.");
    {
        bool placeOnClick =
            app().vegetationPlacementMode() == EditorApplication::VegetationPlacementMode::Grass;
        if (ImGui::Checkbox("Place grass on click", &placeOnClick))
            app().vegetationPlacementMode() = placeOnClick
                ? EditorApplication::VegetationPlacementMode::Grass
                : EditorApplication::VegetationPlacementMode::None;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("While on, clicking a surface in the viewport plants a tuft there "
                              "instead of selecting it.");
    }
    if (ImGui::Button("Clear grass", ImVec2(-FLT_MIN, 0.0f)) && grass.count() > 0)
    {
        app().recordUndo();
        grass.clear();
        app().markDirty();
    }
    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawHairComponent(Hair& hair)
{
    ImGui::Indent(14.0f);
    if (!hair.owner() || !hair.owner()->getComponent<MeshRenderer>())
        ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.2f, 1.0f),
                           "Requires a MeshRenderer on this object.");
    ImGui::Text("Generated roots: %u", hair.rootCount());
    ImGui::TextDisabled("Vertex alpha controls density and relative length.");

    ImGui::Button(hair.textureFile().empty() ? "Drop strand texture here"
                                             : hair.textureFile().c_str(),
                  ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
            if (hair.loadTexture(std::string(static_cast<const char*>(payload->Data),
                                             payload->DataSize)))
                app().markDirty();
        ImGui::EndDragDropTarget();
    }

    u32 integer = hair.strandCount();
    if (ImGui::InputScalar("Strands", ImGuiDataType_U32, &integer))
    { hair.setStrandCount(integer); app().markDirty(); }
    integer = hair.submesh();
    if (ImGui::InputScalar("Scalp submesh", ImGuiDataType_U32, &integer))
    { hair.setSubmesh(integer); app().markDirty(); }
    integer = hair.seed();
    if (ImGui::InputScalar("Seed", ImGuiDataType_U32, &integer))
    { hair.setSeed(integer); app().markDirty(); }
    f32 growthNormal = hair.minimumGrowthNormalY();
    if (ImGui::SliderFloat("Minimum growth normal Y", &growthNormal, -1.0f, 1.0f))
    { hair.setMinimumGrowthNormalY(growthNormal); app().markDirty(); }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("-1: selected submesh/alpha mask only. 0: upper-facing half only.");
    integer = hair.segments();
    const u32 minimumSlider = 1;
    const u32 maximumSegments = kHairMaxSegments;
    if (ImGui::SliderScalar("Segments", ImGuiDataType_U32, &integer,
                            &minimumSlider, &maximumSegments))
    { hair.setSegments(integer); app().markDirty(); }
    integer = hair.followers();
    const u32 maximumFollowers = kHairMaxFollowers;
    if (ImGui::SliderScalar("Ribbon layers", ImGuiDataType_U32, &integer,
                            &minimumSlider, &maximumFollowers))
    { hair.setFollowers(integer); app().markDirty(); }

    f32 minimum = hair.minimumLength(), maximum = hair.maximumLength();
    if (ImGui::DragFloatRange2("Length", &minimum, &maximum, 0.005f, 0.001f, 100.0f))
    { hair.setLengthRange(minimum, maximum); app().markDirty(); }
    f32 value = hair.width();
    if (ImGui::DragFloat("Width", &value, 0.0002f, 0.0001f, 1.0f, "%.4f"))
    { hair.setWidth(value); app().markDirty(); }
    value = hair.stiffness();
    if (ImGui::DragFloat("Stiffness", &value, 0.1f, 0.0f, 1000.0f))
    { hair.setStiffness(value); app().markDirty(); }
    value = hair.drag();
    if (ImGui::SliderFloat("Drag", &value, 0.0f, 0.999f))
    { hair.setDrag(value); app().markDirty(); }
    value = hair.gravity();
    if (ImGui::DragFloat("Gravity", &value, 0.1f, -100.0f, 100.0f))
    { hair.setGravity(value); app().markDirty(); }
    value = hair.wind();
    if (ImGui::DragFloat("Wind", &value, 0.05f, 0.0f, 100.0f))
    { hair.setWind(value); app().markDirty(); }
    value = hair.drawDistance();
    if (ImGui::DragFloat("Draw distance", &value, 1.0f, 0.0f, 100000.0f))
    { hair.setDrawDistance(value); app().markDirty(); }
    value = hair.alphaCut();
    if (ImGui::SliderFloat("Alpha cut", &value, 0.0f, 1.0f))
    { hair.setAlphaCut(value); app().markDirty(); }
    value = hair.roughness();
    if (ImGui::SliderFloat("Roughness", &value, 0.04f, 1.0f))
    { hair.setRoughness(value); app().markDirty(); }
    value = hair.specularStrength();
    if (ImGui::SliderFloat("Specular strength", &value, 0.0f, 1.0f))
    { hair.setSpecularStrength(value); app().markDirty(); }
    value = hair.specularTint();
    if (ImGui::SliderFloat("Specular tint", &value, 0.0f, 1.0f))
    { hair.setSpecularTint(value); app().markDirty(); }
    value = hair.transmission();
    if (ImGui::SliderFloat("Transmission", &value, 0.0f, 1.0f))
    { hair.setTransmission(value); app().markDirty(); }
    glm::vec3 color = hair.color();
    if (ImGui::ColorEdit3("Colour", &color.x))
    { hair.setColor(color); app().markDirty(); }
    bool fringe = hair.softFringe();
    if (ImGui::Checkbox("Soft fringe", &fringe))
    { hair.setSoftFringe(fringe); app().markDirty(); }

    if (ImGui::Button("Generate / Regenerate", ImVec2(-FLT_MIN, 0.0f)))
    {
        app().recordUndo();
        if (hair.generate())
            app().markDirty();
    }
    if (hair.rootCount() > 0 && ImGui::Button("Clear hair", ImVec2(-FLT_MIN, 0.0f)))
    {
        app().recordUndo();
        hair.clear();
        app().markDirty();
    }
    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawForestComponent(Forest& forest)
{
    ImGui::Indent(14.0f);
    const u64 objectId = forest.owner() ? forest.owner()->id() : 0;
    if (mForestObjectId != objectId)
    {
        mForestObjectId = objectId;
        mForestSpeciesIndex = 0;
    }
    if (forest.speciesCount() == 0)
        mForestSpeciesIndex = 0;
    else
        mForestSpeciesIndex = glm::clamp(mForestSpeciesIndex, 0, static_cast<int>(forest.speciesCount()) - 1);
    if (ImGui::TreeNode("Add species"))
    {
        for (u32 i = 0; i < Assets().treePresetCount(); ++i)
        {
            const TreePreset& preset = Assets().treePreset(i);
            if (ImGui::Button(preset.name))
            {
                const s32 index = forest.addSpecies(preset.params);
                if (index >= 0)
                    mForestSpeciesIndex = index;
                app().markDirty();
            }
        }
        ImGui::TreePop();
    }
    if (forest.speciesCount() == 0)
    {
        ImGui::TextUnformatted("No species");
    }
    else
    {
        if (forest.speciesCount() > 1)
        {
            const std::string selectedName = "Species " + std::to_string(mForestSpeciesIndex + 1);
            if (ImGui::BeginCombo("Species", selectedName.c_str()))
            {
                for (u32 i = 0; i < forest.speciesCount(); ++i)
                {
                    const std::string name = "Species " + std::to_string(i + 1);
                    if (ImGui::Selectable(name.c_str(), mForestSpeciesIndex == static_cast<int>(i)))
                        mForestSpeciesIndex = static_cast<int>(i);
                }
                ImGui::EndCombo();
            }
        }
        const u32 speciesIndex = static_cast<u32>(mForestSpeciesIndex);
        if (!mTreePreview.valid())
            mTreePreview.create(300, 400);
        if (mTreePreview.valid())
        {
            const Material materials[2] = {forest.material(speciesIndex, 0),
                                           forest.material(speciesIndex, 1)};
            mTreePreview.render(forest.speciesMesh(speciesIndex), materials, 2, mTreePreviewYaw);
            const u32 texture = mTreePreview.textureId();
            if (texture != 0)
                ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(texture)),
                             ImVec2(210.0f, 280.0f), ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
            ImGui::SliderFloat("Preview rotation", &mTreePreviewYaw, 0.0f, glm::two_pi<f32>());
        }
        TreeParams params = forest.speciesParams(speciesIndex);
        bool rebuilt = false;
        rebuilt |= ImGui::DragFloat("Clump max", &params.clumpMax, 0.01f);
        rebuilt |= ImGui::DragFloat("Clump min", &params.clumpMin, 0.01f);
        rebuilt |= ImGui::DragFloat("Branch factor", &params.branchFactor, 0.01f);
        rebuilt |= ImGui::DragFloat("Max radius", &params.maxRadius, 0.01f, 0.001f, 100.0f);
        rebuilt |= ImGui::DragFloat("Trunk length", &params.trunkLength, 0.01f, 0.01f, 1000.0f);
        rebuilt |= ImGui::DragFloat("Initial branch length", &params.initialBranchLength, 0.01f);
        rebuilt |= ImGui::DragFloat("Length falloff", &params.lengthFalloffFactor, 0.01f);
        rebuilt |= ImGui::DragFloat("Length power", &params.lengthFalloffPower, 0.01f);
        rebuilt |= ImGui::DragFloat("Radius falloff", &params.radiusFalloffRate, 0.01f);
        rebuilt |= ImGui::DragFloat("Taper", &params.taperRate, 0.01f);
        rebuilt |= ImGui::DragFloat("Climb rate", &params.climbRate, 0.01f);
        rebuilt |= ImGui::DragFloat("Trunk kink", &params.trunkKink, 0.01f);
        rebuilt |= ImGui::DragFloat("Twist rate", &params.twistRate, 0.1f);
        rebuilt |= ImGui::DragFloat("Drop amount", &params.dropAmount, 0.01f);
        rebuilt |= ImGui::DragFloat("Grow amount", &params.growAmount, 0.01f);
        rebuilt |= ImGui::DragFloat("Sweep amount", &params.sweepAmount, 0.01f);
        rebuilt |= ImGui::DragFloat("V multiplier", &params.vMultiplier, 0.01f);
        rebuilt |= ImGui::DragFloat("Twig scale", &params.twigScale, 0.01f);
        int levels = static_cast<int>(params.levels);
        int segments = static_cast<int>(params.segments);
        int trunkSteps = static_cast<int>(params.trunkSteps);
        int treeSeed = params.seed;
        if (ImGui::DragInt("Levels", &levels, 1.0f, 1, 12)) { params.levels = static_cast<u32>(levels); rebuilt = true; }
        if (ImGui::DragInt("Segments", &segments, 1.0f, 4, 64)) { params.segments = static_cast<u32>(segments); rebuilt = true; }
        if (ImGui::DragInt("Trunk steps", &trunkSteps, 1.0f, 0, 12)) { params.trunkSteps = static_cast<u32>(trunkSteps); rebuilt = true; }
        if (ImGui::DragInt("Tree seed", &treeSeed, 1.0f)) { params.seed = treeSeed; rebuilt = true; }
        if (rebuilt)
        {
            forest.rebuildSpecies(speciesIndex, params, forest.speciesHeight(speciesIndex));
            app().markDirty();
        }
        f32 height = forest.speciesHeight(speciesIndex);
        if (ImGui::DragFloat("Height", &height, 0.1f, 0.1f, 1000.0f))
        {
            forest.rebuildSpecies(speciesIndex, params, height);
            app().markDirty();
        }
        f32 weight = forest.speciesWeight(speciesIndex);
        if (ImGui::DragFloat("Species weight", &weight, 0.01f, 0.0f, 1000.0f))
        {
            forest.setSpeciesWeight(speciesIndex, weight);
            app().markDirty();
        }
        const std::vector<std::string>& twigPaths = forest.twigTexturePaths();
        if (!twigPaths.empty())
        {
            int twig = static_cast<int>(forest.speciesTwigTexture(speciesIndex));
            twig = glm::clamp(twig, 0, static_cast<int>(twigPaths.size()) - 1);
            if (ImGui::BeginCombo("Twig texture", twigPaths[twig].c_str()))
            {
                for (u32 i = 0; i < twigPaths.size(); ++i)
                    if (ImGui::Selectable(twigPaths[i].c_str(), twig == static_cast<int>(i)))
                    {
                        forest.setSpeciesTwigTexture(speciesIndex, i);
                        app().markDirty();
                    }
                ImGui::EndCombo();
            }
        }
        if (ImGui::TreeNode("Bark material"))
        {
            bool materialChanged = drawMaterialFields(forest.material(speciesIndex, 0));
            materialChanged |= drawTextureSlot(app(), "Albedo", SlotAlbedo, forest.material(speciesIndex, 0));
            materialChanged |= drawTextureSlot(app(), "Normal", SlotNormal, forest.material(speciesIndex, 0));
            materialChanged |= drawTextureSlot(app(), "Surface", SlotSurface, forest.material(speciesIndex, 0));
            if (materialChanged) app().markDirty();
            ImGui::TreePop();
        }
        if (ImGui::TreeNode("Twig material"))
        {
            bool materialChanged = drawMaterialFields(forest.material(speciesIndex, 1));
            materialChanged |= drawTextureSlot(app(), "Albedo", SlotAlbedo, forest.material(speciesIndex, 1));
            materialChanged |= drawTextureSlot(app(), "Normal", SlotNormal, forest.material(speciesIndex, 1));
            materialChanged |= drawTextureSlot(app(), "Surface", SlotSurface, forest.material(speciesIndex, 1));
            if (materialChanged) app().markDirty();
            ImGui::TreePop();
        }
    }
    ImGui::Button(forest.barkAlbedoPath().empty() ? "Drop bark albedo" : forest.barkAlbedoPath().c_str(), ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            forest.setBarkTexture(std::string(static_cast<const char*>(payload->Data), payload->DataSize), forest.barkNormalPath());
            app().markDirty();
        }
        ImGui::EndDragDropTarget();
    }
    ImGui::Button(forest.barkNormalPath().empty() ? "Drop bark normal" : forest.barkNormalPath().c_str(), ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            forest.setBarkTexture(forest.barkAlbedoPath(), std::string(static_cast<const char*>(payload->Data), payload->DataSize));
            app().markDirty();
        }
        ImGui::EndDragDropTarget();
    }
    if (ImGui::TreeNode("Twig textures"))
    {
        s32 removeTwig = -1;
        const std::vector<std::string>& paths = forest.twigTexturePaths();
        for (u32 i = 0; i < paths.size(); ++i)
        {
            ImGui::PushID(static_cast<int>(i));
            ImGui::TextUnformatted(paths[i].c_str());
            ImGui::SameLine();
            if (ImGui::SmallButton("Remove")) removeTwig = static_cast<s32>(i);
            ImGui::PopID();
        }
        ImGui::Button("Drop twig texture to add", ImVec2(-FLT_MIN, 0.0f));
        if (ImGui::BeginDragDropTarget())
        {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
            {
                forest.addTwigTexture(std::string(static_cast<const char*>(payload->Data), payload->DataSize));
                app().markDirty();
            }
            ImGui::EndDragDropTarget();
        }
        if (removeTwig >= 0 && forest.removeTwigTexture(static_cast<u32>(removeTwig)))
            app().markDirty();
        ImGui::TreePop();
    }
    bool shadows = forest.castsShadows();
    if (ImGui::Checkbox("Cast shadows", &shadows)) { forest.setCastShadows(shadows); app().markDirty(); }
    f32 value = forest.wind();
    if (ImGui::DragFloat("Wind", &value, 0.01f, 0.0f, 100.0f)) { forest.setWind(value); app().markDirty(); }
    value = forest.barkBumpForce();
    if (ImGui::DragFloat("Bark bump", &value, 0.01f, 0.0f, 100.0f)) { forest.setBarkBumpForce(value); app().markDirty(); }
    value = forest.alphaCut();
    if (ImGui::SliderFloat("Alpha cut", &value, 0.05f, 0.95f)) { forest.setAlphaCut(value); app().markDirty(); }
    value = forest.drawDistance();
    if (ImGui::DragFloat("Draw distance", &value, 1.0f, 0.0f, 100000.0f)) { forest.setDrawDistance(value); app().markDirty(); }
    f32 scaleMinimum = forest.scaleMinimum();
    f32 scaleMaximum = forest.scaleMaximum();
    if (ImGui::DragFloatRange2("Scale range", &scaleMinimum, &scaleMaximum, 0.01f, 0.01f, 100.0f)) { forest.setScaleRange(scaleMinimum, scaleMaximum); app().markDirty(); }
    u32 seed = forest.seed();
    if (ImGui::InputScalar("Seed", ImGuiDataType_U32, &seed)) { forest.setSeed(seed); app().markDirty(); }
    bool impostors = forest.impostorsEnabled();
    if (ImGui::Checkbox("Impostors", &impostors)) { forest.setImpostorsEnabled(impostors); app().markDirty(); }
    value = forest.swapDistance();
    if (ImGui::DragFloat("Swap distance", &value, 1.0f, 0.0f, 100000.0f)) { forest.setSwapDistance(value); app().markDirty(); }
    value = forest.swapBand();
    if (ImGui::DragFloat("Swap band", &value, 0.1f, 0.0f, 100000.0f)) { forest.setSwapBand(value); app().markDirty(); }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Overlap around the swap distance where both the mesh and the\n"
                          "impostor draw at once, mesh fading out as the impostor fades in.\n"
                          "Zero makes the handover a pop instead of a cross-fade.");
    value = forest.impostorWidth();
    if (ImGui::DragFloat("Impostor width", &value, 0.01f, 0.05f, 5.0f)) { forest.setImpostorWidth(value); app().markDirty(); }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Impostor quad width over height. A tree is taller than it is\n"
                          "wide - too narrow clips the crown at the handover, too wide\n"
                          "leaves it floating in empty space.");
    static f32 paintRadius = 20.0f;
    static int paintCount = 100;
    ImGui::DragFloat("Paint radius", &paintRadius, 0.5f, 0.1f, 10000.0f);
    ImGui::DragInt("Paint count", &paintCount, 1.0f, 1, 1000000);
    if (ImGui::Button("Plant trees", ImVec2(-FLT_MIN, 0.0f)) && forest.speciesCount() > 0)
    {
        glm::vec3 centre = app().cursor3D();
        if (forest.owner())
            centre = glm::vec3(glm::inverse(forest.owner()->globalTransform()) * glm::vec4(centre, 1.0f));
        app().recordUndo();
        if (forest.paint(centre, paintRadius, static_cast<u32>(paintCount)) > 0)
            app().markDirty();
    }
    if (ImGui::Button("Plant one here", ImVec2(-FLT_MIN, 0.0f)) && forest.speciesCount() > 0)
    {
        // paint() only ever scatters (it rejects radius <= 0 outright) -
        // plant() is the same per-tree call it makes internally, exposed so
        // a single exact placement does not need a fake scatter radius.
        glm::vec3 centre = app().cursor3D();
        if (forest.owner())
            centre = glm::vec3(glm::inverse(forest.owner()->globalTransform()) * glm::vec4(centre, 1.0f));
        const u32 speciesIndex =
            glm::clamp(static_cast<u32>(mForestSpeciesIndex), 0u, forest.speciesCount() - 1);
        app().recordUndo();
        if (forest.plant(centre, speciesIndex))
            app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Plants exactly one tree of the selected species at the 3D cursor, no "
                          "scatter radius.");
    if (forest.speciesCount() > 0)
    {
        app().vegetationPlacementSpecies() = static_cast<u32>(mForestSpeciesIndex);
        bool placeOnClick =
            app().vegetationPlacementMode() == EditorApplication::VegetationPlacementMode::Tree;
        if (ImGui::Checkbox("Place tree on click", &placeOnClick))
            app().vegetationPlacementMode() = placeOnClick
                ? EditorApplication::VegetationPlacementMode::Tree
                : EditorApplication::VegetationPlacementMode::None;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("While on, clicking a surface in the viewport plants the selected "
                              "species there instead of selecting it.");
    }
    if (forest.count() > 0 && ImGui::Button("Clear trees", ImVec2(-FLT_MIN, 0.0f)))
    {
        app().recordUndo();
        forest.clear();
        app().markDirty();
    }
    ImGui::Text("Trees: %u", forest.count());
    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawOceanComponent(Ocean& ocean)
{
    bool changed = false;
    ImGui::Indent(14.0f);

    const char* qualityNames[] = {"Sky only", "Reflection", "Reflection + refraction"};
    int quality = static_cast<int>(ocean.quality());
    if (ImGui::Combo("Quality", &quality, qualityNames, IM_ARRAYSIZE(qualityNames)))
    {
        ocean.setQuality(static_cast<OceanQuality>(glm::clamp(quality, 0, 2)));
        changed = true;
    }
    u32 waveCount = ocean.waveCount();
    int waveCountInt = static_cast<int>(waveCount);
    if (ImGui::SliderInt("Wave count", &waveCountInt, 0, static_cast<int>(kOceanMaxWaves)))
    {
        ocean.setWaveCount(static_cast<u32>(waveCountInt));
        changed = true;
    }
    f32 value = ocean.waveScale();
    if (ImGui::DragFloat("Wave scale", &value, 0.01f, 0.0f, 1000.0f))
    {
        ocean.setWaveScale(value);
        changed = true;
    }
    value = ocean.steepness();
    if (ImGui::SliderFloat("Steepness", &value, 0.0f, 1.0f))
    {
        ocean.setSteepness(value);
        changed = true;
    }
    value = ocean.timeScale();
    if (ImGui::DragFloat("Time scale", &value, 0.01f, -100.0f, 100.0f))
    {
        ocean.setTimeScale(value);
        changed = true;
    }
    value = ocean.level();
    if (ImGui::DragFloat("Level", &value, 0.01f))
    {
        ocean.setLevel(value);
        changed = true;
    }

    if (ImGui::TreeNode("Waves"))
    {
        for (u32 i = 0; i < kOceanMaxWaves; ++i)
        {
            const OceanWave& wave = ocean.wave(i);
            glm::vec2 direction = wave.direction;
            f32 wavelength = wave.wavelength;
            f32 amplitude = wave.amplitude;
            ImGui::PushID(static_cast<int>(i));
            ImGui::Text("Wave %u", i + 1);
            changed |= ImGui::DragFloat2("Direction", &direction.x, 0.01f, -1.0f, 1.0f);
            changed |= ImGui::DragFloat("Wavelength", &wavelength, 0.05f, 0.001f, 100000.0f);
            changed |= ImGui::DragFloat("Amplitude", &amplitude, 0.01f, 0.0f, 100000.0f);
            if (direction != wave.direction || wavelength != wave.wavelength ||
                amplitude != wave.amplitude)
            {
                ocean.setWave(i, direction, wavelength, amplitude);
                changed = true;
            }
            ImGui::PopID();
        }
        ImGui::TreePop();
    }

    ImGui::SeparatorText("Water shading");
    glm::vec3 color = ocean.shallowColor();
    if (ImGui::ColorEdit3("Shallow color", &color.x))
    {
        ocean.setShallowColor(color);
        changed = true;
    }
    color = ocean.deepColor();
    if (ImGui::ColorEdit3("Deep color", &color.x))
    {
        ocean.setDeepColor(color);
        changed = true;
    }
    value = ocean.absorptionDistance();
    if (ImGui::DragFloat("Absorption distance", &value, 0.1f, 0.001f, 100000.0f))
    {
        ocean.setAbsorptionDistance(value);
        changed = true;
    }
    value = ocean.roughness();
    if (ImGui::SliderFloat("Roughness", &value, 0.001f, 1.0f))
    {
        ocean.setRoughness(value);
        changed = true;
    }
    value = ocean.specularStrength();
    if (ImGui::DragFloat("Specular strength", &value, 0.01f, 0.0f, 100.0f))
    {
        ocean.setSpecularStrength(value);
        changed = true;
    }

    ImGui::SeparatorText("Normals and foam");
    changed |= drawOceanTextureSlot(app(), "Ocean normal texture", ocean, true);
    bool enabled = ocean.normalMapEnabled();
    if (ImGui::Checkbox("Normal map", &enabled))
    {
        ocean.setNormalMapEnabled(enabled);
        changed = true;
    }
    int octaves = static_cast<int>(ocean.normalOctaves());
    if (ImGui::SliderInt("Normal octaves", &octaves, 1, 6))
    {
        ocean.setNormalOctaves(static_cast<u32>(octaves));
        changed = true;
    }
    glm::vec2 normalScale(ocean.normalScale1(), ocean.normalScale2());
    if (ImGui::DragFloat2("Normal scale", &normalScale.x, 0.001f, 0.0f, 100.0f))
    {
        ocean.setNormalScale(normalScale.x, normalScale.y);
        changed = true;
    }
    value = ocean.normalStrength();
    if (ImGui::DragFloat("Normal strength", &value, 0.01f, 0.0f, 100.0f))
    {
        ocean.setNormalStrength(value);
        changed = true;
    }
    glm::vec2 normalSpeed(ocean.normalSpeed1(), ocean.normalSpeed2());
    if (ImGui::DragFloat2("Normal speed", &normalSpeed.x, 0.01f, -100.0f, 100.0f))
    {
        ocean.setNormalSpeed(normalSpeed.x, normalSpeed.y);
        changed = true;
    }
    enabled = ocean.foamEnabled();
    changed |= drawOceanTextureSlot(app(), "Ocean foam texture", ocean, false);
    if (ImGui::Checkbox("Foam", &enabled))
    {
        ocean.setFoamEnabled(enabled);
        changed = true;
    }
    value = ocean.foamScale();
    if (ImGui::DragFloat("Foam scale", &value, 0.001f, 0.0f, 100.0f))
    {
        ocean.setFoamScale(value);
        changed = true;
    }
    value = ocean.foamStrength();
    if (ImGui::DragFloat("Foam strength", &value, 0.01f, 0.0f, 100.0f))
    {
        ocean.setFoamStrength(value);
        changed = true;
    }
    value = ocean.foamDepth();
    if (ImGui::DragFloat("Foam depth", &value, 0.01f, 0.001f, 100000.0f))
    {
        ocean.setFoamDepth(value);
        changed = true;
    }
    value = ocean.foamCrest();
    if (ImGui::DragFloat("Foam crest", &value, 0.01f, -100.0f, 100.0f))
    {
        ocean.setFoamCrest(value);
        changed = true;
    }

    ImGui::SeparatorText("Fresnel and screen effects");
    value = ocean.fresnelDetail();
    if (ImGui::SliderFloat("Fresnel detail", &value, 0.0f, 1.0f)) { ocean.setFresnelDetail(value); changed = true; }
    value = ocean.fresnelMax();
    if (ImGui::SliderFloat("Fresnel max", &value, 0.0f, 1.0f)) { ocean.setFresnelMax(value); changed = true; }
    value = ocean.fresnelBias();
    if (ImGui::SliderFloat("Fresnel bias", &value, 0.0f, 1.0f)) { ocean.setFresnelBias(value); changed = true; }
    value = ocean.fresnelScale();
    if (ImGui::DragFloat("Fresnel scale", &value, 0.01f, 0.0f, 100.0f)) { ocean.setFresnelScale(value); changed = true; }
    value = ocean.fresnelPower();
    if (ImGui::DragFloat("Fresnel power", &value, 0.05f, 0.1f, 100.0f)) { ocean.setFresnelPower(value); changed = true; }
    value = ocean.minOpacity();
    if (ImGui::SliderFloat("Minimum opacity", &value, 0.0f, 1.0f)) { ocean.setMinOpacity(value); changed = true; }
    value = ocean.reflectionDistortion();
    if (ImGui::DragFloat("Reflection distortion", &value, 0.001f, -100.0f, 100.0f)) { ocean.setReflectionDistortion(value); changed = true; }
    value = ocean.reflectionStrength();
    if (ImGui::DragFloat("Reflection strength", &value, 0.01f, 0.0f, 100.0f)) { ocean.setReflectionStrength(value); changed = true; }
    value = ocean.refractionStrength();
    if (ImGui::SliderFloat("Refraction strength", &value, 0.0f, 1.0f)) { ocean.setRefractionStrength(value); changed = true; }
    value = ocean.colorStrength();
    if (ImGui::DragFloat("Color strength", &value, 0.01f, 0.0f, 100.0f)) { ocean.setColorStrength(value); changed = true; }
    color = ocean.underwaterColor();
    if (ImGui::ColorEdit3("Underwater color", &color.x)) { ocean.setUnderwaterColor(color); changed = true; }
    int debugMode = ocean.debugMode();
    if (ImGui::DragInt("Debug mode", &debugMode, 1.0f, 0, 16)) { ocean.setDebugMode(debugMode); changed = true; }

    if (changed)
        app().markDirty();
    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawAnimatorComponent(Animator& animator)
{
    ImGui::Indent(14.0f);
    if (!animator.bound())
    {
        ImGui::TextDisabled("Not bound - see Add Component below, or the Animation panel.");
        ImGui::Unindent(14.0f);
        return;
    }
    const Skeleton* skeleton = animator.skeleton();
    if (skeleton)
        ImGui::Text("%u bones, %u layer(s), %u IK chain(s)", skeleton->boneCount(),
                   animator.layerCount(), animator.ikChainCount());

    if (const AnimationSet* set = Animations().get(animator.animationSet()))
    {
        ImGui::TextUnformatted("Clips");
        for (const AnimationClip& clip : set->clips)
            ImGui::BulletText("%s (%.2fs)", clip.name().c_str(),
                             static_cast<double>(clip.duration()));
        if (set->clips.empty())
            ImGui::TextDisabled("No clips bound yet.");
    }

    // Adding a clip to a live Animator: sets are cached by their exact
    // (skeleton, clips) file list, so "add" is really "load the same
    // skeleton with one more clip and rebind" - the old set stays cached
    // for whoever else still holds it. The dropped file goes through the
    // same ensureAnimationFile() the Add Animator popup uses, so a .fbx
    // clip converts to .ranim beside its source on the way in.
    ImGui::Button("(drop a .ranim/.fbx clip here to add it)", ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            const std::string dropped(static_cast<const char*>(payload->Data),
                                      static_cast<usize>(payload->DataSize));
            const std::string extension = lowerExtension(dropped);
            if (extension != "ranim" && extension != "fbx" && extension != "b3d" &&
                extension != "ms3d" && extension != "gltf" && extension != "glb")
            {
                Log::warning("InspectorPanel: '%s' is not an animation clip", dropped.c_str());
                app().toasts().warning(FileSystem::fileName(dropped) +
                                       " is not an animation clip");
            }
            else if (skeleton)
            {
                const std::string skeletonPath =
                    Animations().skeletonSourceFile(animator.animationSet());
                std::vector<std::string> clips =
                    Animations().animationSourceFiles(animator.animationSet());
                const std::string clipPath = ensureAnimationFile(dropped, *skeleton);
                if (clipPath.empty())
                {
                    Log::error("InspectorPanel: could not read a clip from '%s' against this "
                               "skeleton",
                               dropped.c_str());
                    app().toasts().error("Could not read a clip from " +
                                         FileSystem::fileName(dropped));
                }
                else if (std::find(clips.begin(), clips.end(), clipPath) != clips.end())
                    app().toasts().info(FileSystem::fileName(clipPath) + " is already bound");
                else if (skeletonPath.empty())
                {
                    Log::error("InspectorPanel: this Animator's set has no source files - it was "
                               "bound in memory, clips cannot be added by file");
                    app().toasts().error("This Animator was not bound from files - cannot add "
                                         "clips to it");
                }
                else
                {
                    clips.push_back(clipPath);
                    const AnimationSetHandle handle =
                        Animations().loadFromFiles(skeletonPath, clips);
                    if (handle.valid())
                    {
                        animator.bind(handle);
                        app().markDirty();
                        app().toasts().success("Added clip " + FileSystem::fileName(clipPath));
                    }
                    else
                    {
                        Log::error("InspectorPanel: loadFromFiles() failed adding '%s'",
                                   clipPath.c_str());
                        app().toasts().error("Could not rebind with " +
                                             FileSystem::fileName(clipPath));
                    }
                }
            }
        }
        ImGui::EndDragDropTarget();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Drag an animation file from the Assets panel. Test playback in the "
                          "Animation panel.");
    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawParticleEmitterComponent(ParticleEmitter& emitter)
{
    ImGui::Indent(14.0f);
    bool changed = false;
    if (ImGui::Button("Bullet preset")) { ParticleEmitter::presetBulletImpact(emitter); changed = true; }
    ImGui::SameLine();
    if (ImGui::Button("Debris preset")) { ParticleEmitter::presetDebris(emitter); changed = true; }
    if (ImGui::Button("Dust preset")) { ParticleEmitter::presetDust(emitter); changed = true; }
    ImGui::SameLine();
    if (ImGui::Button("Smoke preset")) { ParticleEmitter::presetSmoke(emitter); changed = true; }
    int mode = static_cast<int>(emitter.emissionMode());
    const char* modes[] = {"Continuous", "Burst", "One Shot", "Pulse"};
    if (ImGui::Combo("Emission mode", &mode, modes, IM_ARRAYSIZE(modes)))
    {
        emitter.setEmissionMode(static_cast<ParticleEmissionMode>(glm::clamp(mode, 0, 3)));
        changed = true;
    }
    int maxParticles = static_cast<int>(emitter.maxParticles());
    if (ImGui::DragInt("Max particles", &maxParticles, 1.0f, 1, 1000000)) { emitter.setMaxParticles(static_cast<u32>(maxParticles)); changed = true; }
    f32 value = 0.0f;
    if (mode == static_cast<int>(ParticleEmissionMode::Continuous))
    {
        value = emitter.emissionRate();
        if (ImGui::DragFloat("Emission rate", &value, 0.1f, 0.0f, 100000.0f)) { emitter.setContinuous(value); changed = true; }
    }
    else if (mode == static_cast<int>(ParticleEmissionMode::Burst))
    {
        int count = static_cast<int>(emitter.burstCount());
        f32 interval = emitter.burstInterval();
        bool emissionChanged = ImGui::DragInt("Burst count", &count, 1.0f, 1, 1000000);
        emissionChanged |= ImGui::DragFloat("Burst interval", &interval, 0.01f, 0.001f, 100000.0f);
        if (emissionChanged)
        { emitter.setBurst(static_cast<u32>(count), interval); changed = true; }
    }
    else if (mode == static_cast<int>(ParticleEmissionMode::OneShot))
    {
        int count = static_cast<int>(emitter.oneShotCount());
        if (ImGui::DragInt("One shot count", &count, 1.0f, 1, 1000000)) { emitter.setOneShot(static_cast<u32>(count)); changed = true; }
    }
    else
    {
        f32 rate = emitter.pulseRate();
        int count = static_cast<int>(emitter.particlesPerPulse());
        bool emissionChanged = ImGui::DragFloat("Pulse rate", &rate, 0.1f, 0.0f, 100000.0f);
        emissionChanged |= ImGui::DragInt("Particles per pulse", &count, 1.0f, 1, 1000000);
        if (emissionChanged)
        { emitter.setPulse(rate, static_cast<u32>(count)); changed = true; }
    }

    int shape = static_cast<int>(emitter.shape());
    const char* shapes[] = {"Point", "Sphere", "Box", "Cone", "Circle", "Ring"};
    if (ImGui::Combo("Shape", &shape, shapes, IM_ARRAYSIZE(shapes)))
    {
        if (shape == 0) emitter.setShapePoint();
        else if (shape == 1) emitter.setShapeSphere(emitter.shapeRadius());
        else if (shape == 2) emitter.setShapeBox(emitter.shapeBoxSize());
        else if (shape == 3) emitter.setShapeCone(emitter.shapeConeAngle(), emitter.shapeRadius());
        else if (shape == 4) emitter.setShapeCircle(emitter.shapeRadius());
        else emitter.setShapeRing(emitter.shapeRadius(), emitter.shapeInnerRadius());
        changed = true;
    }
    if (shape == 1 || shape == 4)
    {
        value = emitter.shapeRadius();
        if (ImGui::DragFloat("Shape radius", &value, 0.01f, 0.0f, 100000.0f)) { if (shape == 1) emitter.setShapeSphere(value); else emitter.setShapeCircle(value); changed = true; }
    }
    else if (shape == 2)
    {
        glm::vec3 box = emitter.shapeBoxSize();
        if (ImGui::DragFloat3("Box size", &box.x, 0.01f, 0.0f, 100000.0f)) { emitter.setShapeBox(box); changed = true; }
    }
    else if (shape == 3)
    {
        f32 angle = emitter.shapeConeAngle();
        f32 radius = emitter.shapeRadius();
        bool shapeChanged = ImGui::DragFloat("Cone angle", &angle, 0.1f, 0.0f, 180.0f);
        shapeChanged |= ImGui::DragFloat("Cone radius", &radius, 0.01f, 0.0f, 100000.0f);
        if (shapeChanged)
        { emitter.setShapeCone(angle, radius); changed = true; }
    }
    else if (shape == 5)
    {
        f32 outer = emitter.shapeRadius();
        f32 inner = emitter.shapeInnerRadius();
        bool shapeChanged = ImGui::DragFloat("Outer radius", &outer, 0.01f, 0.0f, 100000.0f);
        shapeChanged |= ImGui::DragFloat("Inner radius", &inner, 0.01f, 0.0f, 100000.0f);
        if (shapeChanged)
        { emitter.setShapeRing(outer, inner); changed = true; }
    }

    glm::vec3 offset = emitter.emissionOffset();
    if (ImGui::DragFloat3("Offset", &offset.x, 0.01f)) { emitter.setEmissionOffset(offset); changed = true; }
    value = emitter.lifetimeMin();
    f32 value2 = emitter.lifetimeMax();
    if (ImGui::DragFloatRange2("Lifetime", &value, &value2, 0.01f, 0.001f, 100000.0f)) { emitter.setLifetime(value, value2); changed = true; }
    value = emitter.speedMin(); value2 = emitter.speedMax();
    if (ImGui::DragFloatRange2("Speed", &value, &value2, 0.01f, 0.0f, 100000.0f)) { emitter.setSpeed(value, value2); changed = true; }
    glm::vec2 size = emitter.sizeStart(); glm::vec2 end = emitter.sizeEnd();
    if (ImGui::DragFloat2("Start size", &size.x, 0.01f, 0.0f, 10000.0f)) { emitter.setSize(size, end); changed = true; }
    if (ImGui::DragFloat2("End size", &end.x, 0.01f, 0.0f, 10000.0f)) { emitter.setSize(size, end); changed = true; }
    float startColor[4] = {emitter.colorStart().red(), emitter.colorStart().green(), emitter.colorStart().blue(), emitter.colorStart().alpha()};
    float endColor[4] = {emitter.colorEnd().red(), emitter.colorEnd().green(), emitter.colorEnd().blue(), emitter.colorEnd().alpha()};
    bool colorChanged = ImGui::ColorEdit4("Start color", startColor);
    colorChanged |= ImGui::ColorEdit4("End color", endColor);
    if (colorChanged)
    {
        emitter.setColor(Color::fromRGBFloat(startColor[0], startColor[1], startColor[2], startColor[3]),
                         Color::fromRGBFloat(endColor[0], endColor[1], endColor[2], endColor[3]));
        changed = true;
    }
    glm::vec3 direction = emitter.emissionDirection();
    if (ImGui::DragFloat3("Direction", &direction.x, 0.01f, -1.0f, 1.0f)) { emitter.setEmissionDirection(direction); changed = true; }
    value = emitter.spreadAngle();
    if (ImGui::SliderFloat("Spread angle", &value, 0.0f, 180.0f)) { emitter.setSpreadAngle(value); changed = true; }
    value = emitter.rotationSpeedMin(); value2 = emitter.rotationSpeedMax();
    if (ImGui::DragFloatRange2("Rotation speed", &value, &value2, 0.01f, -1000.0f, 1000.0f)) { emitter.setRotationSpeed(value, value2); changed = true; }
    glm::vec3 gravity = emitter.gravity();
    if (ImGui::DragFloat3("Gravity", &gravity.x, 0.01f, -1000.0f, 1000.0f)) { emitter.setGravity(gravity); changed = true; }
    value = emitter.drag();
    if (ImGui::DragFloat("Drag", &value, 0.01f, 0.0f, 100.0f)) { emitter.setDrag(value); changed = true; }
    value = emitter.duration();
    if (ImGui::DragFloat("Duration", &value, 0.01f, -1.0f, 100000.0f)) { emitter.setDuration(value); changed = true; }
    bool enabled = emitter.loop();
    if (ImGui::Checkbox("Loop", &enabled)) { emitter.setLoop(enabled); changed = true; }

    bool atlas = emitter.usesAtlas();
    if (ImGui::Checkbox("Atlas", &atlas))
    {
        if (atlas) emitter.setAtlasGrid(emitter.atlasCols(), emitter.atlasRows()); else emitter.clearAtlas();
        changed = true;
    }
    if (atlas)
    {
        int cols = static_cast<int>(emitter.atlasCols());
        int rows = static_cast<int>(emitter.atlasRows());
        bool atlasChanged = ImGui::DragInt("Atlas columns", &cols, 1.0f, 1, 64);
        atlasChanged |= ImGui::DragInt("Atlas rows", &rows, 1.0f, 1, 64);
        if (atlasChanged)
        { emitter.setAtlasGrid(static_cast<u32>(cols), static_cast<u32>(rows)); changed = true; }
    }

    ImGui::Button(emitter.textureFile().empty() ? "Drop texture here" : emitter.textureFile().c_str(), ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            emitter.setTextureFile(std::string(static_cast<const char*>(payload->Data), payload->DataSize));
            changed = true;
        }
        ImGui::EndDragDropTarget();
    }
    if (!emitter.textureFile().empty() && ImGui::Button("Clear texture"))
    {
        emitter.setTextureFile(std::string());
        changed = true;
    }
    enabled = emitter.additive();
    if (ImGui::Checkbox("Additive", &enabled)) { emitter.setAdditive(enabled); changed = true; }
    enabled = emitter.depthTest();
    if (ImGui::Checkbox("Depth test", &enabled)) { emitter.setDepthTest(enabled); changed = true; }
    int billboard = static_cast<int>(emitter.billboardMode());
    const char* billboards[] = {"Free", "Upright", "Fixed"};
    if (ImGui::Combo("Billboard", &billboard, billboards, IM_ARRAYSIZE(billboards))) { emitter.setBillboardMode(static_cast<BillboardMode>(glm::clamp(billboard, 0, 2))); changed = true; }

    if (ImGui::TreeNode("Affectors"))
    {
        s32 removeAffector = -1;
        for (usize i = 0; i < emitter.affectors().size(); ++i)
        {
            ParticleAffector* affector = emitter.affectors()[i];
            ImGui::PushID(static_cast<int>(i));
            changed |= ImGui::Checkbox("Enabled", &affector->enabled);
            if (auto* gravityAffector = dynamic_cast<GravityAffector*>(affector)) changed |= ImGui::DragFloat3("Gravity", &gravityAffector->gravity.x, 0.01f);
            else if (auto* dragAffector = dynamic_cast<DragAffector*>(affector)) changed |= ImGui::DragFloat("Drag", &dragAffector->drag, 0.01f, 0.0f, 100.0f);
            else if (auto* vortex = dynamic_cast<VortexAffector*>(affector)) { changed |= ImGui::DragFloat3("Center", &vortex->center.x, 0.01f); changed |= ImGui::DragFloat("Strength", &vortex->strength, 0.01f); changed |= ImGui::DragFloat("Radius", &vortex->radius, 0.01f, 0.0f, 100000.0f); }
            else if (auto* attractor = dynamic_cast<AttractorAffector*>(affector)) { changed |= ImGui::DragFloat3("Position", &attractor->position.x, 0.01f); changed |= ImGui::DragFloat("Strength", &attractor->strength, 0.01f); changed |= ImGui::DragFloat("Radius", &attractor->radius, 0.01f, 0.0f, 100000.0f); changed |= ImGui::Checkbox("Repulse", &attractor->repulse); }
            else if (auto* turbulence = dynamic_cast<TurbulenceAffector*>(affector)) { changed |= ImGui::DragFloat("Strength", &turbulence->strength, 0.01f); changed |= ImGui::DragFloat("Frequency", &turbulence->frequency, 0.01f); }
            else if (auto* color = dynamic_cast<ColorOverLifetimeAffector*>(affector))
            {
                float start[4] = {color->startColor.red(), color->startColor.green(), color->startColor.blue(), color->startColor.alpha()};
                float end[4] = {color->endColor.red(), color->endColor.green(), color->endColor.blue(), color->endColor.alpha()};
                bool affectorChanged = ImGui::ColorEdit4("Start", start);
                affectorChanged |= ImGui::ColorEdit4("End", end);
                if (affectorChanged)
                {
                    color->startColor = Color::fromRGBFloat(start[0], start[1], start[2], start[3]);
                    color->endColor = Color::fromRGBFloat(end[0], end[1], end[2], end[3]);
                    changed = true;
                }
            }
            else if (auto* sizeAffector = dynamic_cast<SizeOverLifetimeAffector*>(affector))
            {
                changed |= ImGui::DragFloat2("Start", &sizeAffector->startSize.x, 0.01f, 0.0f, 10000.0f);
                changed |= ImGui::DragFloat2("End", &sizeAffector->endSize.x, 0.01f, 0.0f, 10000.0f);
            }
            if (ImGui::SmallButton("Remove")) removeAffector = static_cast<s32>(i);
            ImGui::Separator();
            ImGui::PopID();
        }
        if (removeAffector >= 0 && emitter.removeAffector(static_cast<usize>(removeAffector)))
            changed = true;
        if (ImGui::Button("Add gravity")) { emitter.addAffector(new GravityAffector(glm::vec3(0.0f, -9.8f, 0.0f))); changed = true; }
        ImGui::SameLine();
        if (ImGui::Button("Add drag")) { emitter.addAffector(new DragAffector(0.1f)); changed = true; }
        if (ImGui::Button("Add vortex")) { emitter.addAffector(new VortexAffector(glm::vec3(0.0f), 1.0f, 1.0f)); changed = true; }
        ImGui::SameLine();
        if (ImGui::Button("Add attractor")) { emitter.addAffector(new AttractorAffector(glm::vec3(0.0f), 1.0f, 1.0f)); changed = true; }
        if (ImGui::Button("Add turbulence")) { emitter.addAffector(new TurbulenceAffector(1.0f, 1.0f)); changed = true; }
        ImGui::SameLine();
        if (ImGui::Button("Add color")) { emitter.addAffector(new ColorOverLifetimeAffector(emitter.colorStart(), emitter.colorEnd())); changed = true; }
        if (ImGui::Button("Add size")) { emitter.addAffector(new SizeOverLifetimeAffector(emitter.sizeStart(), emitter.sizeEnd())); changed = true; }
        if (ImGui::Button("Clear affectors")) { emitter.clearAffectors(); changed = true; }
        ImGui::TreePop();
    }

    if (ImGui::Button(emitter.isPlaying() ? "Stop" : "Play"))
    {
        if (emitter.isPlaying()) emitter.stop(); else emitter.play();
        changed = true;
    }
    if (changed)
        app().markDirty();
    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawParticleEffectComponent(ParticleEffect& effect)
{
    bool changed = false;
    int mode = effect.mode() == ParticleEffectMode::OneShot ? 0 : 1;
    const char* modes[] = {"One Shot", "Continuous"};
    if (ImGui::Combo("Mode", &mode, modes, IM_ARRAYSIZE(modes)))
    {
        effect.setMode(mode == 0 ? ParticleEffectMode::OneShot
                                 : ParticleEffectMode::Continuous);
        changed = true;
    }

    int burstCount = static_cast<int>(effect.burstCount());
    if (mode == 0 && ImGui::DragInt("Burst Count", &burstCount, 1.0f, 1, 100000))
    {
        effect.setBurstCount(static_cast<u32>(burstCount));
        changed = true;
    }
    bool autoDestroy = effect.autoDestroy();
    if (ImGui::Checkbox("Auto Destroy", &autoDestroy))
    {
        effect.setAutoDestroy(autoDestroy);
        changed = true;
    }
    bool ownerDirection = effect.useOwnerDirection();
    if (ImGui::Checkbox("Use Owner Direction", &ownerDirection))
    {
        effect.setUseOwnerDirection(ownerDirection);
        changed = true;
    }

    if (effect.isPlaying())
    {
        if (ImGui::Button("Stop", ImVec2(-FLT_MIN, 0.0f)))
        {
            effect.stop();
            changed = true;
        }
    }
    else if (ImGui::Button("Play", ImVec2(-FLT_MIN, 0.0f)))
    {
        effect.play();
        changed = true;
    }

    ParticleSystem::Emitter emitter = effect.emitter();
    ImGui::SeparatorText("Emitter");
    changed |= ImGui::DragFloat("Rate", &emitter.rate, 0.5f, 0.0f, 100000.0f);
    changed |= ImGui::DragFloat3("Direction", &emitter.direction.x, 0.01f, -1.0f, 1.0f);
    f32 spreadDegrees = glm::degrees(emitter.spread);
    if (ImGui::SliderFloat("Spread", &spreadDegrees, 0.0f, 180.0f, "%.1f deg"))
    {
        emitter.spread = glm::radians(spreadDegrees);
        changed = true;
    }
    changed |= ImGui::DragFloatRange2("Speed", &emitter.speedMin, &emitter.speedMax, 0.1f,
                                      0.0f, 100000.0f);
    changed |= ImGui::DragFloatRange2("Lifetime", &emitter.lifeMin, &emitter.lifeMax, 0.05f,
                                      0.01f, 10000.0f);
    changed |= ImGui::DragFloat("Start Size", &emitter.sizeBegin, 0.01f, 0.0f, 10000.0f);
    changed |= ImGui::DragFloat("End Size", &emitter.sizeEnd, 0.01f, 0.0f, 10000.0f);
    changed |= ImGui::ColorEdit4("Start Color", &emitter.colorBegin.x);
    changed |= ImGui::ColorEdit4("End Color", &emitter.colorEnd.x);
    changed |= ImGui::DragFloat("Mass", &emitter.mass, 0.01f, 0.0f, 10000.0f);
    changed |= ImGui::DragFloat("Rotation Velocity", &emitter.rotationVelocity, 0.01f,
                                -1000.0f, 1000.0f);
    changed |= ImGui::DragFloat("Start Radius", &emitter.startRadius, 0.01f, 0.0f, 10000.0f);
    if (changed)
    {
        effect.setEmitter(emitter);
        app().markDirty();
    }
}

void InspectorPanel::drawAddComponentSection(GameObject& object)
{
    // A menu, not a single button, on purpose - HierarchyPanel's own Create
    // menu (drawCreateMenu()) is the model: this list only grows as more
    // component kinds pick up an "attach to an object that already exists"
    // path, same as that one grew its own categories over time. Only a
    // component simple enough to need nothing beyond addComponent<T>() (or
    // a fixed one-time setup call, ReflectionProbe's create()) belongs here
    // directly; anything that needs geometry/data building at creation time
    // (Terrain, Road, ParticleEffect, ...) stays Hierarchy-only until it
    // grows its own "add to existing object" setup to mirror here.
    if (ImGui::Button(ICON_MDI_PLUS " Add Component"))
        ImGui::OpenPopup("AddComponentMenu");

    if (ImGui::BeginPopup("AddComponentMenu"))
    {
        if (!object.getComponent<Camera>() && ImGui::MenuItem("Camera"))
        {
            object.addComponent<Camera>();
            app().markDirty();
        }
        if (object.getComponent<Camera>() && ImGui::BeginMenu("Camera Controller"))
        {
            if (!object.getComponent<FreeFly>() && ImGui::MenuItem("FreeFly"))
            {
                object.addComponent<FreeFly>();
                app().markDirty();
            }
            if (!object.getComponent<FPS>() && ImGui::MenuItem("FPS"))
            {
                object.addComponent<FPS>();
                app().markDirty();
            }
            if (!object.getComponent<Orbit>() && ImGui::MenuItem("Orbit"))
            {
                object.addComponent<Orbit>();
                app().markDirty();
            }
            if (!object.getComponent<Maya>() && ImGui::MenuItem("Maya"))
            {
                object.addComponent<Maya>();
                app().markDirty();
            }
            if (!object.getComponent<ThirdPerson>() && ImGui::MenuItem("ThirdPerson"))
            {
                object.addComponent<ThirdPerson>();
                app().markDirty();
            }
            ImGui::EndMenu();
        }
        if (!object.getComponent<Light>() && ImGui::BeginMenu("Light"))
        {
            if (ImGui::MenuItem("Directional"))
                object.addComponent<DirectionalLight>();
            if (ImGui::MenuItem("Point"))
                object.addComponent<PointLight>();
            if (ImGui::MenuItem("Spot"))
                object.addComponent<SpotLight>();
            if (ImGui::MenuItem("Rectangle"))
                object.addComponent<RectangleLight>();
            if (object.getComponent<Light>())
                app().markDirty();
            ImGui::EndMenu();
        }
        if (!object.getComponent<Text3D>() && ImGui::MenuItem("Text3D"))
        {
            object.addComponent<Text3D>();
            app().markDirty();
        }
        if (!object.getComponent<Billboard>() && ImGui::MenuItem("Billboard"))
        {
            object.addComponent<Billboard>();
            app().markDirty();
        }
        if (!object.getComponent<Waypoints>() && ImGui::MenuItem("Waypoints"))
        {
            app().recordUndo();
            object.addComponent<Waypoints>();
            app().markDirty();
        }
        if (!object.getComponent<NavMeshSurface>() && ImGui::MenuItem("NavMeshSurface"))
        {
            app().recordUndo();
            object.addComponent<NavMeshSurface>();
            app().markDirty();
        }
        if (!object.getComponent<SelfDestroy>() && ImGui::MenuItem("SelfDestroy"))
        {
            object.addComponent<SelfDestroy>();
            app().markDirty();
        }
        if (!object.getComponent<Collider>() && ImGui::MenuItem("Collider"))
        {
            object.addComponent<Collider>();
            app().markDirty();
        }
        if (!object.getComponent<Physics::RigidBody>() && ImGui::MenuItem("RigidBody"))
        {
            object.addComponent<Physics::RigidBody>();
            app().markDirty();
        }
        if (!object.getComponent<Physics::Joint>() && ImGui::MenuItem("Joint"))
        {
            object.addComponent<Physics::HingeJoint>();
            app().markDirty();
        }
        if (!object.getComponent<AudioPlayer>() && ImGui::MenuItem("AudioPlayer"))
        {
            object.addComponent<AudioPlayer>();
            app().markDirty();
        }
        if (!object.getComponent<ScriptComponent>() && ImGui::MenuItem("Zen Behaviour"))
        {
            object.addComponent<ZenBehaviour>();
            app().markDirty();
        }
        if (!object.getComponent<BoneAttachment>() && ImGui::MenuItem("Bone Attachment"))
        {
            object.addComponent<BoneAttachment>();
            app().markDirty();
        }
        if (!object.getComponent<Hair>() && object.getComponent<MeshRenderer>() &&
            ImGui::MenuItem("Hair"))
        {
            Hair* hair = object.addComponent<Hair>();
            hair->generate();
            app().markDirty();
        }
        if (!object.getComponent<ReflectionProbe>() && ImGui::MenuItem("Reflection Probe"))
        {
            // Same one-time setup HierarchyPanel's createReflectionProbeObject()
            // gives a freshly created one - addComponent<T>() alone leaves this
            // one inert (no cubemap yet), unlike everything else in this menu.
            ReflectionProbe* probeComponent = object.addComponent<ReflectionProbe>();
            probeComponent->create(128);
            EnvironmentProbe& env = probeComponent->probe();
            env.extents = glm::vec3(0.0f);
            env.influenceRadius = 5.0f;
            env.content = EnvironmentProbe::Content::SkyAndWorld;
            env.refresh = EnvironmentProbe::Refresh::Automatic;
            env.invalidate();
            app().markDirty();
        }
        if (!object.getComponent<VoxelWorldComponent>() && ImGui::MenuItem("Voxel World"))
        {
            app().recordUndo();
            object.addComponent<VoxelWorldComponent>();
            app().markDirty();
        }
        if (!object.getComponent<TiledTerrain>() && ImGui::MenuItem("TiledTerrain"))
        {
            app().recordUndo();
            object.addComponent<TiledTerrain>();
            app().markDirty();
        }
        if (!object.getComponent<Animator>() && ImGui::MenuItem("Animator..."))
        {
            mNewAnimatorClipFiles.clear();
            mNewAnimatorError.clear();
            // A skinned MeshRenderer's bone indices come from THIS file's own
            // buildBoneMap() call, made once at mesh-import time - loading the
            // skeleton from any other export (an animation-only Idle.fbx, say)
            // would very likely enumerate FBX objects in a different order and
            // silently mismatch every bone index, even though nothing here
            // would fail loudly. Pre-filling the mesh's own source file is
            // what steers away from that trap; the field is still editable
            // for an object with no mesh of its own (an empty rig setup
            // elsewhere).
            MeshRenderer* renderer = object.getComponent<MeshRenderer>();
            const MeshDesc& desc = renderer ? Assets().meshDesc(renderer->mesh()) : MeshDesc();
            mNewAnimatorSkeletonFile = desc.source == MeshSource::File ? desc.file : std::string();
            // Not OpenPopup() here directly - "AddComponentMenu" is still
            // open around this MenuItem and closes on this same click
            // (MenuItem's default behaviour), and opening a second popup
            // while the first is mid-close on the very same frame is exactly
            // the ordering ImGui's own popup stack does not guarantee. The
            // flag defers the actual OpenPopup() to below, once
            // AddComponentMenu is fully done with for this frame.
            mOpenAddAnimatorPopup = true;
        }
        ImGui::EndPopup();
    }

    if (mOpenAddAnimatorPopup)
    {
        ImGui::OpenPopup("Add Animator");
        mOpenAddAnimatorPopup = false;
    }

    ImGui::SetNextWindowSize(ImVec2(420.0f, 0.0f), ImGuiCond_Appearing);
    if (!ImGui::BeginPopupModal("Add Animator", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;

    ImGui::TextWrapped("Skeleton (.fbx/.b3d/.ms3d/.gltf/.glb or .rskel) - must be the SAME file "
                       "this object's mesh was imported from, not a separate animation-only "
                       "export (its bone order would not match the mesh's own skin indices).");
    ImGui::TextUnformatted(mNewAnimatorSkeletonFile.empty() ? "(none)"
                                                            : mNewAnimatorSkeletonFile.c_str());
    if (ImGui::Button("Browse...##skeleton"))
    {
        const std::string& last = app().settings().lastOpenDirectory;
        mAnimatorSkeletonDialog.Open(ImGuiFileDialog::Mode::OpenFile,
                                     last.empty() ? RADION_ASSET_DIR : last);
        mAnimatorSkeletonDialogPending = true;
    }
    if (mAnimatorSkeletonDialogPending &&
        mAnimatorSkeletonDialog.Render(RADION_ASSET_DIR, RADION_ASSET_DIR, RADION_ASSET_DIR))
    {
        const ImGuiFileDialog::Result result = mAnimatorSkeletonDialog.ConsumeResult();
        mAnimatorSkeletonDialogPending = false;
        if (result.accepted)
        {
            mNewAnimatorSkeletonFile = result.path.string();
            app().settings().lastOpenDirectory = result.path.parent_path().string();
        }
    }

    if (!mNewAnimatorSkeletonFile.empty())
    {
        const std::filesystem::path skeletonPath(mNewAnimatorSkeletonFile);
        const std::string directory = skeletonPath.parent_path().string();
        const std::string skeletonExtension = lowerExtension(mNewAnimatorSkeletonFile);
        const bool alreadyHasClip =
            std::find(mNewAnimatorClipFiles.begin(), mNewAnimatorClipFiles.end(),
                     mNewAnimatorSkeletonFile) != mNewAnimatorClipFiles.end();

        // b3d/ms3d/gltf carry the skeleton AND every animation in the one
        // file (importAnimation() reads clips from it same as
        // importSkeleton() reads bones) - unlike Mixamo's fbx convention
        // below, the clip to add here IS the skeleton file itself, not a
        // sibling.
        if ((skeletonExtension == "b3d" || skeletonExtension == "ms3d" ||
            skeletonExtension == "gltf" || skeletonExtension == "glb") &&
            !alreadyHasClip)
        {
            ImGui::TextDisabled("This format carries its animation in the same file:");
            ImGui::PushID("SkeletonAsClip");
            if (ImGui::SmallButton(ICON_MDI_PLUS))
                mNewAnimatorClipFiles.push_back(mNewAnimatorSkeletonFile);
            ImGui::SameLine();
            ImGui::TextUnformatted(skeletonPath.filename().string().c_str());
            ImGui::PopID();
        }

        // Mixamo-style exports are the other shape: one skeleton+mesh file,
        // one extra .fbx per clip, all sitting next to each other - listing
        // them here beats a "Browse..." round trip per clip for that case.
        std::vector<std::string> suggestions;
        for (const FileSystem::DirEntry& entry :
             FileSystem::getSingleton().listDirectory(directory))
        {
            if (entry.isDirectory)
                continue;
            const std::string extension = lowerExtension(entry.name);
            if (extension != "fbx" && extension != "ranim" && extension != "b3d" &&
                extension != "ms3d" && extension != "gltf" && extension != "glb")
                continue;
            const std::string full = directory + "/" + entry.name;
            if (full == mNewAnimatorSkeletonFile)
                continue;
            bool alreadyAdded = false;
            for (const std::string& clip : mNewAnimatorClipFiles)
                if (clip == full)
                {
                    alreadyAdded = true;
                    break;
                }
            if (!alreadyAdded)
                suggestions.push_back(entry.name);
        }
        if (!suggestions.empty())
        {
            ImGui::TextDisabled("Other files next to the skeleton:");
            for (const std::string& name : suggestions)
            {
                ImGui::PushID(name.c_str());
                if (ImGui::SmallButton(ICON_MDI_PLUS))
                    mNewAnimatorClipFiles.push_back(directory + "/" + name);
                ImGui::SameLine();
                ImGui::TextUnformatted(name.c_str());
                ImGui::PopID();
            }
        }
    }

    ImGui::Separator();
    ImGui::TextWrapped("Animation clips (.fbx or .ranim) - one file per clip, Mixamo-style "
                       "exports work as-is.");
    int removeIndex = -1;
    for (int i = 0; i < static_cast<int>(mNewAnimatorClipFiles.size()); ++i)
    {
        ImGui::PushID(i);
        if (ImGui::Button(ICON_MDI_CLOSE))
            removeIndex = i;
        ImGui::SameLine();
        ImGui::TextUnformatted(mNewAnimatorClipFiles[static_cast<usize>(i)].c_str());
        ImGui::PopID();
    }
    if (removeIndex >= 0)
        mNewAnimatorClipFiles.erase(mNewAnimatorClipFiles.begin() + removeIndex);
    if (ImGui::Button(ICON_MDI_PLUS " Add Clip..."))
    {
        const std::string& last = app().settings().lastOpenDirectory;
        mAnimatorClipDialog.Open(ImGuiFileDialog::Mode::OpenFile,
                                 last.empty() ? RADION_ASSET_DIR : last);
        mAnimatorClipDialogPending = true;
    }
    if (mAnimatorClipDialogPending &&
        mAnimatorClipDialog.Render(RADION_ASSET_DIR, RADION_ASSET_DIR, RADION_ASSET_DIR))
    {
        const ImGuiFileDialog::Result result = mAnimatorClipDialog.ConsumeResult();
        mAnimatorClipDialogPending = false;
        if (result.accepted)
        {
            mNewAnimatorClipFiles.push_back(result.path.string());
            app().settings().lastOpenDirectory = result.path.parent_path().string();
        }
    }

    ImGui::Separator();
    ImGui::BeginDisabled(mNewAnimatorSkeletonFile.empty());
    if (ImGui::Button("Create"))
    {
        if (createAnimator(object, mNewAnimatorSkeletonFile, mNewAnimatorClipFiles,
                           mNewAnimatorError))
        {
            app().markDirty();
            ImGui::CloseCurrentPopup();
        }
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
        ImGui::CloseCurrentPopup();
    if (!mNewAnimatorError.empty())
    {
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.4f, 0.4f, 1.0f));
        ImGui::TextWrapped("%s", mNewAnimatorError.c_str());
        ImGui::PopStyleColor();
    }
    ImGui::EndPopup();
}

void InspectorPanel::drawCameraComponent(Camera& camera)
{
    ImGui::Indent(14.0f);

    bool orthographic = camera.projectionMode() == CameraProjection::Orthographic;
    if (ImGui::Checkbox("Orthographic", &orthographic))
    {
        if (orthographic)
            camera.setOrthographic(camera.orthographicSize(), camera.aspect(), camera.nearPlane(),
                                   camera.farPlane());
        else
            camera.setPerspective(camera.fieldOfView(), camera.aspect(), camera.nearPlane(),
                                  camera.farPlane());
        app().markDirty();
    }

    if (orthographic)
    {
        f32 size = camera.orthographicSize();
        if (ImGui::DragFloat("Size", &size, 0.1f, 0.01f, 10000.0f))
        {
            camera.setOrthographic(size, camera.aspect(), camera.nearPlane(), camera.farPlane());
            app().markDirty();
        }
    }
    else
    {
        f32 fov = camera.fieldOfView();
        if (ImGui::DragFloat("Field of View", &fov, 0.5f, 1.0f, 179.0f, "%.1f°"))
        {
            camera.setPerspective(fov, camera.aspect(), camera.nearPlane(), camera.farPlane());
            app().markDirty();
        }
    }

    f32 nearPlane = camera.nearPlane();
    f32 farPlane = camera.farPlane();
    bool clipChanged = false;
    clipChanged |= ImGui::DragFloat("Near Plane", &nearPlane, 0.01f, 0.001f, farPlane - 0.01f);
    clipChanged |= ImGui::DragFloat("Far Plane", &farPlane, 1.0f, nearPlane + 0.01f, 1000000.0f);
    if (clipChanged)
    {
        if (orthographic)
            camera.setOrthographic(camera.orthographicSize(), camera.aspect(), nearPlane, farPlane);
        else
            camera.setPerspective(camera.fieldOfView(), camera.aspect(), nearPlane, farPlane);
        app().markDirty();
    }

    // Read-only: the viewport/game render target's own dimensions drive this
    // every frame (Camera::setAspect(), ViewportPanel/GamePanel) - a value
    // typed in here would just be overwritten on the next resize.
    ImGui::BeginDisabled();
    f32 aspect = camera.aspect();
    ImGui::DragFloat("Aspect", &aspect);
    ImGui::EndDisabled();

    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawLightComponent(Light& light)
{
    ImGui::Indent(14.0f);

    glm::vec3 color = light.color();
    if (ImGui::ColorEdit3("Color", &color.x))
    {
        light.setColor(color);
        app().markDirty();
    }
    f32 intensity = light.intensity();
    if (ImGui::DragFloat("Intensity", &intensity, 0.05f, 0.0f, 10000.0f))
    {
        light.setIntensity(intensity);
        app().markDirty();
    }
    const LightType kind = light.lightType();
    bool systemEnabled = true;
    if (kind == LightType::Directional)
        systemEnabled = app().engine().sunShadows();
    else if (kind == LightType::Point)
        systemEnabled = app().engine().pointShadows();
    else
        systemEnabled = app().engine().spotShadows();

    ImGui::BeginDisabled(!systemEnabled);
    bool castShadows = light.castsShadows();
    if (ImGui::Checkbox("Cast Shadows", &castShadows))
    {
        light.setCastShadows(castShadows);
        app().markDirty();
    }
    ImGui::EndDisabled();
    if (!systemEnabled && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Shadows for this light type are switched off in Settings > Shadows. "
                          "This light keeps its own setting.");
    ImGui::SameLine();
    bool volumetric = light.volumetric();
    if (ImGui::Checkbox("Volumetric", &volumetric))
    {
        light.setVolumetric(volumetric);
        app().markDirty();
    }

    switch (light.lightType())
    {
    case LightType::Point:
    {
        PointLight& point = static_cast<PointLight&>(light);
        f32 range = point.range();
        if (ImGui::DragFloat("Range", &range, 0.1f, 0.01f, 100000.0f))
        {
            point.setRange(range);
            app().markDirty();
        }
        break;
    }
    case LightType::Spot:
    {
        SpotLight& spot = static_cast<SpotLight&>(light);
        f32 range = spot.range();
        if (ImGui::DragFloat("Range", &range, 0.1f, 0.01f, 100000.0f))
        {
            spot.setRange(range);
            app().markDirty();
        }
        f32 inner = spot.innerAngle();
        f32 outer = spot.outerAngle();
        bool anglesChanged = false;
        anglesChanged |= ImGui::DragFloat("Inner Angle", &inner, 0.5f, 0.0f, outer, "%.1f°");
        anglesChanged |= ImGui::DragFloat("Outer Angle", &outer, 0.5f, inner, 89.0f, "%.1f°");
        if (anglesChanged)
        {
            spot.setAngles(inner, outer);
            app().markDirty();
        }
        break;
    }
    case LightType::Rectangle:
    {
        RectangleLight& rectangle = static_cast<RectangleLight&>(light);
        f32 range = rectangle.range();
        if (ImGui::DragFloat("Range", &range, 0.1f, 0.01f, 100000.0f))
        {
            rectangle.setRange(range);
            app().markDirty();
        }
        f32 width = rectangle.width();
        f32 height = rectangle.height();
        bool sizeChanged = false;
        sizeChanged |= ImGui::DragFloat("Width", &width, 0.05f, 0.01f, 10000.0f);
        sizeChanged |= ImGui::DragFloat("Height", &height, 0.05f, 0.01f, 10000.0f);
        if (sizeChanged)
        {
            rectangle.setSize(width, height);
            app().markDirty();
        }
        break;
    }
    case LightType::Directional:
        break; // color/intensity/shadows above are all a directional light has
    }

    ImGui::Unindent(14.0f);
}

namespace
{
const char* billboardModeName(BillboardMode mode)
{
    switch (mode)
    {
    case BillboardMode::Free:
        return "Free";
    case BillboardMode::Upright:
        return "Upright";
    case BillboardMode::Fixed:
        return "Fixed";
    }
    return "Free";
}

// True if changed.
bool drawBillboardModeCombo(BillboardMode& mode)
{
    bool changed = false;
    if (ImGui::BeginCombo("Mode", billboardModeName(mode)))
    {
        for (BillboardMode candidate :
            {BillboardMode::Free, BillboardMode::Upright, BillboardMode::Fixed})
        {
            const bool selected = candidate == mode;
            if (ImGui::Selectable(billboardModeName(candidate), selected))
            {
                mode = candidate;
                changed = true;
            }
            if (selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    return changed;
}

const char* billboardBlendName(BatchRenderer::BlendMode mode)
{
    switch (mode)
    {
    case BatchRenderer::BlendMode::Alpha:
        return "Alpha";
    case BatchRenderer::BlendMode::Additive:
        return "Additive";
    case BatchRenderer::BlendMode::Multiplied:
        return "Multiplied";
    case BatchRenderer::BlendMode::AddColors:
        return "Add Colors";
    case BatchRenderer::BlendMode::SubtractColors:
        return "Subtract Colors";
    }
    return "Alpha";
}

// True if changed.
bool drawBillboardBlendCombo(BatchRenderer::BlendMode& mode)
{
    bool changed = false;
    if (ImGui::BeginCombo("Blend", billboardBlendName(mode)))
    {
        for (BatchRenderer::BlendMode candidate :
            {BatchRenderer::BlendMode::Alpha, BatchRenderer::BlendMode::Additive,
             BatchRenderer::BlendMode::Multiplied, BatchRenderer::BlendMode::AddColors,
             BatchRenderer::BlendMode::SubtractColors})
        {
            const bool selected = candidate == mode;
            if (ImGui::Selectable(billboardBlendName(candidate), selected))
            {
                mode = candidate;
                changed = true;
            }
            if (selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    return changed;
}
} // namespace

void InspectorPanel::drawText3DComponent(Text3D& text)
{
    ImGui::Indent(14.0f);

    char buffer[512];
    std::strncpy(buffer, text.text().c_str(), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    if (ImGui::InputTextMultiline("Text", buffer, sizeof(buffer), ImVec2(-FLT_MIN, 60.0f)))
    {
        text.setText(buffer);
        app().markDirty();
    }

    f32 characterSize = text.characterSize();
    if (ImGui::DragFloat("Character Size", &characterSize, 0.01f, 0.001f, 1000.0f))
    {
        text.setCharacterSize(characterSize);
        app().markDirty();
    }
    f32 spacing = text.spacing();
    if (ImGui::DragFloat("Spacing", &spacing, 0.01f, 0.0f, 10.0f))
    {
        text.setSpacing(spacing);
        app().markDirty();
    }

    float colorValues[4] = {text.color().red(), text.color().green(), text.color().blue(),
                            text.color().alpha()};
    if (ImGui::ColorEdit4("Color", colorValues))
    {
        text.setColor(
            Color::fromRGBFloat(colorValues[0], colorValues[1], colorValues[2], colorValues[3]));
        app().markDirty();
    }

    BillboardMode mode = text.mode();
    if (drawBillboardModeCombo(mode))
    {
        text.setMode(mode);
        app().markDirty();
    }

    static const char* kAlignNames[] = {"Left", "Center", "Right"};
    int alignIndex = static_cast<int>(text.alignment());
    if (ImGui::Combo("Alignment", &alignIndex, kAlignNames, IM_ARRAYSIZE(kAlignNames)))
    {
        text.setAlignment(static_cast<TextAlign>(alignIndex));
        app().markDirty();
    }

    bool additive = text.additive();
    if (ImGui::Checkbox("Additive", &additive))
    {
        text.setAdditive(additive);
        app().markDirty();
    }
    ImGui::SameLine();
    bool depthTest = text.depthTest();
    if (ImGui::Checkbox("Depth Test", &depthTest))
    {
        text.setDepthTest(depthTest);
        app().markDirty();
    }

    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawBillboardComponent(Billboard& billboard)
{
    ImGui::Indent(14.0f);

    glm::vec2 size = billboard.size();
    if (ImGui::DragFloat2("Size", &size.x, 0.01f, 0.001f, 10000.0f))
    {
        billboard.setSize(size);
        app().markDirty();
    }

    float colorValues[4] = {billboard.color().red(), billboard.color().green(),
                            billboard.color().blue(), billboard.color().alpha()};
    if (ImGui::ColorEdit4("Color", colorValues))
    {
        billboard.setColor(Color::fromRGBFloat(colorValues[0], colorValues[1], colorValues[2],
                                               colorValues[3]));
        app().markDirty();
    }

    BillboardMode mode = billboard.mode();
    if (drawBillboardModeCombo(mode))
    {
        billboard.setMode(mode);
        app().markDirty();
    }

    ImGui::TextUnformatted("Texture");
    ImGui::SameLine();
    ImGui::Button(billboard.texture().valid() ? "(assigned)" : "(drop image asset here)",
                 ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            const std::string path(static_cast<const char*>(payload->Data), payload->DataSize);
            billboard.setTextureFile(path);
            app().markDirty();
        }
        ImGui::EndDragDropTarget();
    }

    BatchRenderer::BlendMode blend = billboard.blendMode();
    if (drawBillboardBlendCombo(blend))
    {
        billboard.setBlendMode(blend);
        app().markDirty();
    }
    bool depthTest = billboard.depthTest();
    if (ImGui::Checkbox("Depth Test", &depthTest))
    {
        billboard.setDepthTest(depthTest);
        app().markDirty();
    }

    ImGui::Separator();
    bool animated = billboard.animated();
    ImGui::TextUnformatted(animated ? "Animated Atlas" : "Atlas Cell");
    int cols = static_cast<int>(billboard.atlasCols());
    int rows = static_cast<int>(billboard.atlasRows());
    bool atlasChanged = false;
    atlasChanged |= ImGui::DragInt("Columns", &cols, 0.1f, 1, 64);
    atlasChanged |= ImGui::DragInt("Rows", &rows, 0.1f, 1, 64);
    if (animated)
    {
        f32 fps = billboard.atlasFps();
        if (ImGui::DragFloat("FPS", &fps, 0.1f, 0.1f, 240.0f))
        {
            billboard.setAnimatedAtlas(static_cast<u32>(cols), static_cast<u32>(rows), fps);
            app().markDirty();
        }
        else if (atlasChanged)
        {
            billboard.setAnimatedAtlas(static_cast<u32>(cols), static_cast<u32>(rows),
                                       billboard.atlasFps());
            app().markDirty();
        }
        if (ImGui::Button("Use Fixed Cell Instead"))
        {
            billboard.setAtlasCell(static_cast<u32>(cols), static_cast<u32>(rows), 0, 0);
            app().markDirty();
        }
    }
    else
    {
        static int cellCol = 0;
        static int cellRow = 0;
        atlasChanged |= ImGui::DragInt("Cell Column", &cellCol, 0.1f, 0, 63);
        atlasChanged |= ImGui::DragInt("Cell Row", &cellRow, 0.1f, 0, 63);
        if (atlasChanged)
        {
            billboard.setAtlasCell(static_cast<u32>(cols), static_cast<u32>(rows),
                                   static_cast<u32>(cellCol), static_cast<u32>(cellRow));
            app().markDirty();
        }
        if (ImGui::Button("Animate This Atlas"))
        {
            billboard.setAnimatedAtlas(static_cast<u32>(cols), static_cast<u32>(rows), 12.0f);
            app().markDirty();
        }
    }

    ImGui::Unindent(14.0f);
}

namespace
{
bool containsCaseInsensitive(const std::string& haystack, const char* needle)
{
    if (needle[0] == '\0')
        return true;
    std::string lowered = haystack;
    for (char& c : lowered)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    std::string loweredNeedle(needle);
    for (char& c : loweredNeedle)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return lowered.find(loweredNeedle) != std::string::npos;
}

// The Select Bone popup's body. Filter empty: the skeleton's real hierarchy,
// every branch open (a rig is picked from by reading it top to bottom, so
// collapsed-by-default just adds clicks). Filter set: a flat list of the
// matches - grafting matched nodes onto partial trees reads worse than a
// plain list once the tree is mostly filtered away.
void drawBoneTree(const Skeleton& skeleton, s32 parent, s32 currentBone, const char* filter,
                  bool& picked, s32& pickedBone)
{
    if (filter[0] != '\0')
    {
        if (parent != -1)
            return;
        for (u32 i = 0; i < skeleton.boneCount(); ++i)
        {
            if (!containsCaseInsensitive(skeleton.bone(i).name, filter))
                continue;
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::Selectable(skeleton.bone(i).name.c_str(),
                                  currentBone == static_cast<s32>(i)))
            {
                picked = true;
                pickedBone = static_cast<s32>(i);
            }
            ImGui::PopID();
        }
        return;
    }

    for (u32 i = 0; i < skeleton.boneCount(); ++i)
    {
        if (skeleton.bone(i).parent != parent)
            continue;

        bool hasChildren = false;
        for (u32 j = 0; j < skeleton.boneCount() && !hasChildren; ++j)
            hasChildren = skeleton.bone(j).parent == static_cast<s32>(i);

        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                                   ImGuiTreeNodeFlags_SpanAvailWidth |
                                   ImGuiTreeNodeFlags_DefaultOpen;
        if (!hasChildren)
            flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
        if (currentBone == static_cast<s32>(i))
            flags |= ImGuiTreeNodeFlags_Selected;

        ImGui::PushID(static_cast<int>(i));
        const bool open = ImGui::TreeNodeEx(skeleton.bone(i).name.c_str(), flags);
        if (ImGui::IsItemClicked(ImGuiMouseButton_Left) && !ImGui::IsItemToggledOpen())
        {
            picked = true;
            pickedBone = static_cast<s32>(i);
        }
        if (open && hasChildren)
        {
            drawBoneTree(skeleton, static_cast<s32>(i), currentBone, filter, picked, pickedBone);
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
}
} // namespace

void InspectorPanel::drawBoneAttachmentComponent(BoneAttachment& attachment)
{
    ImGui::Indent(14.0f);

    Animator* animator = attachment.animator();
    GameObject* animatorOwner = animator ? animator->owner() : nullptr;
    ImGui::TextUnformatted("Skeleton");
    ImGui::SameLine();
    ImGui::Button(animatorOwner ? animatorOwner->name().c_str() : "(drag an Animator object here)",
                 ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kGameObjectDragPayload))
        {
            const u64 id = *static_cast<const u64*>(payload->Data);
            if (GameObject* dragged = app().scene().findGameObject(id))
            {
                if (Animator* dragged_animator = dragged->getComponent<Animator>())
                {
                    attachment.bind(dragged_animator, attachment.boneIndex() >= 0
                                                          ? attachment.boneIndex()
                                                          : 0);
                    app().markDirty();
                }
                else
                    Log::warning("InspectorPanel: '%s' has no Animator component",
                                dragged->name().c_str());
            }
        }
        ImGui::EndDragDropTarget();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Drag an object with an Animator component from the Hierarchy.");

    if (!animator || !animator->skeleton())
    {
        ImGui::Unindent(14.0f);
        return;
    }

    const Skeleton* skeleton = animator->skeleton();
    const s32 currentBone = attachment.boneIndex();
    const std::string currentName =
        currentBone >= 0 && static_cast<u32>(currentBone) < skeleton->boneCount()
            ? skeleton->bone(static_cast<u32>(currentBone)).name
            : std::string("(none)");
    ImGui::TextUnformatted("Bone");
    ImGui::SameLine();
    if (ImGui::Button((currentName + "##select_bone").c_str(), ImVec2(-FLT_MIN, 0.0f)))
    {
        mBoneFilter[0] = '\0';
        ImGui::OpenPopup("Select Bone");
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Opens the skeleton's joint tree to pick the bone this object rides.");

    ImGui::SetNextWindowSize(ImVec2(340.0f, 420.0f), ImGuiCond_Appearing);
    if (ImGui::BeginPopup("Select Bone"))
    {
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::IsWindowAppearing())
            ImGui::SetKeyboardFocusHere();
        ImGui::InputTextWithHint("##bone_filter", ICON_MDI_MAGNIFY " Filter", mBoneFilter,
                                 sizeof(mBoneFilter));
        ImGui::Separator();
        ImGui::BeginChild("##bone_tree", ImVec2(0.0f, 0.0f), false,
                          ImGuiWindowFlags_HorizontalScrollbar);
        bool picked = false;
        s32 pickedBone = -1;
        drawBoneTree(*skeleton, -1, currentBone, mBoneFilter, picked, pickedBone);
        ImGui::EndChild();
        if (picked)
        {
            attachment.bind(animator, pickedBone);
            app().markDirty();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawThirdPersonComponent(GameObject&, ThirdPerson& camera)
{
    ImGui::Indent(14.0f);

    GameObject* target = camera.target();
    ImGui::TextUnformatted("Target");
    ImGui::SameLine();
    ImGui::Button(target ? target->name().c_str() : "(drag the player here)",
                 ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The object the camera follows. Without one the component does nothing.");
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kGameObjectDragPayload))
        {
            const u64 id = *static_cast<const u64*>(payload->Data);
            if (GameObject* dragged = app().scene().findGameObject(id))
            {
                camera.setTarget(dragged);
                camera.snap();
                app().markDirty();
            }
        }
        ImGui::EndDragDropTarget();
    }
    if (target && ImGui::Button("Clear Target"))
    {
        camera.setTarget(nullptr);
        app().markDirty();
    }

    f32 distance = camera.distance();
    if (ImGui::DragFloat("Distance", &distance, 0.05f, 0.01f, 1000.0f))
    {
        camera.setDistance(distance);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How far behind the target the camera sits.");

    f32 heightOffset = camera.heightOffset();
    if (ImGui::DragFloat("Height Offset", &heightOffset, 0.05f, -100.0f, 100.0f))
    {
        camera.setHeightOffset(heightOffset);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Raises the point the camera aims at above the target's origin - aim at "
                         "the chest, not the feet.");

    f32 shoulderOffset = camera.shoulderOffset();
    if (ImGui::DragFloat("Shoulder Offset", &shoulderOffset, 0.02f, -10.0f, 10.0f))
    {
        camera.setShoulderOffset(shoulderOffset);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Sideways offset along the camera's right axis. 0 is centred; positive "
                         "moves the camera to the character's right.");

    f32 smoothTime = camera.smoothTime();
    if (ImGui::DragFloat("Smooth Time", &smoothTime, 0.01f, 0.01f, 2.0f, "%.2f s"))
    {
        camera.setSmoothTime(smoothTime);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Spring time constant. Small values weld the camera to the character; "
                         "large values let it trail and settle. Position only - the aim never "
                         "lags.");

    f32 yaw = camera.yaw();
    f32 pitch = camera.pitch();
    bool yawPitchChanged = false;
    yawPitchChanged |= ImGui::DragFloat("Yaw", &yaw, 0.5f, -100000.0f, 100000.0f, "%.1f°");
    yawPitchChanged |= ImGui::DragFloat("Pitch", &pitch, 0.5f, -89.0f, 89.0f, "%.1f°");
    if (yawPitchChanged)
    {
        camera.setYawPitch(yaw, pitch);
        app().markDirty();
    }

    f32 minPitch = camera.minPitch();
    f32 maxPitch = camera.maxPitch();
    bool limitsChanged = false;
    limitsChanged |= ImGui::DragFloat("Min Pitch", &minPitch, 0.5f, -89.0f, 89.0f, "%.1f°");
    limitsChanged |= ImGui::DragFloat("Max Pitch", &maxPitch, 0.5f, -89.0f, 89.0f, "%.1f°");
    if (limitsChanged)
    {
        camera.setPitchLimits(minPitch, maxPitch);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How far the mouse can push the camera down and up. A third-person "
                         "camera wants an asymmetric range, unlike FreeFly's single limit.");

    f32 lookSpeed = camera.lookSpeed();
    if (ImGui::DragFloat("Look Speed", &lookSpeed, 0.01f, 0.001f, 5.0f, "%.3f °/px"))
    {
        camera.setLookSpeed(lookSpeed);
        app().markDirty();
    }

    bool invertY = camera.invertY();
    if (ImGui::Checkbox("Invert Y", &invertY))
    {
        camera.setInvertY(invertY);
        app().markDirty();
    }

    if (ImGui::Button("Snap To Target"))
        camera.snap();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Jump to the orbit point and clear the spring velocity. What a teleport "
                         "or a respawn has to call so the camera does not fly across the level.");

    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawSelfDestroyComponent(GameObject&, SelfDestroy& selfDestroy)
{
    ImGui::Indent(14.0f);

    f32 lifetime = selfDestroy.lifetime();
    if (ImGui::DragFloat("Lifetime", &lifetime, 0.05f, 0.0f, 1000.0f, "%.2f s"))
    {
        selfDestroy.setLifetime(lifetime);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Seconds before this object disposes itself. 0 or below disposes on "
                         "the next update.");

    const f32 total = glm::max(selfDestroy.lifetime(), 0.0001f);
    const f32 fraction = glm::clamp(selfDestroy.elapsed() / total, 0.0f, 1.0f);
    char overlay[64];
    std::snprintf(overlay, sizeof(overlay), "%.2f / %.2f s", selfDestroy.elapsed(),
                 selfDestroy.lifetime());
    ImGui::ProgressBar(fraction, ImVec2(-FLT_MIN, 0.0f), overlay);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Time elapsed against the lifetime, live while the editor runs.");

    if (ImGui::Button("Restart"))
        selfDestroy.restart();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Zero the countdown and re-arm the dispose.");

    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawAudioPlayerComponent(AudioPlayer& player)
{
    ImGui::Indent(14.0f);

    char sourceBuffer[256];
    std::strncpy(sourceBuffer, player.source().c_str(), sizeof(sourceBuffer) - 1);
    sourceBuffer[sizeof(sourceBuffer) - 1] = '\0';
    if (ImGui::InputText("Source", sourceBuffer, sizeof(sourceBuffer)))
    {
        player.setSource(sourceBuffer);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Path to the sound file, resolved through the project's search paths - "
                          "drop an audio asset here instead of typing it.");
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            const std::string path(static_cast<const char*>(payload->Data), payload->DataSize);
            player.setSource(path);
            app().markDirty();
        }
        ImGui::EndDragDropTarget();
    }

    bool music = player.music();
    if (ImGui::Checkbox("Music", &music))
    {
        player.setMusic(music);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Music streams from one voice at a time and ignores Pan; off, this "
                          "decodes up front as a sound effect and overlaps freely with others.");

    bool autoplay = player.autoplay();
    if (ImGui::Checkbox("Autoplay", &autoplay))
    {
        player.setAutoplay(autoplay);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Start playing as soon as this object's Start runs.");

    bool loop = player.loop();
    if (ImGui::Checkbox("Loop", &loop))
    {
        player.setLoop(loop);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Restart from the beginning every time playback reaches the end.");

    f32 volume = player.volume();
    if (ImGui::DragFloat("Volume", &volume, 0.01f, 0.0f, 4.0f, "%.2f"))
    {
        player.setVolume(volume);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Playback gain - 1 is unity, up to 4x boost. Applies live to a voice "
                          "already playing.");

    f32 pitch = player.pitch();
    if (ImGui::DragFloat("Pitch", &pitch, 0.01f, 0.01f, 4.0f, "%.2f"))
    {
        player.setPitch(pitch);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Playback speed and pitch multiplier - 1 is unmodified.");

    f32 pan = player.pan();
    if (ImGui::DragFloat("Pan", &pan, 0.01f, -1.0f, 1.0f, "%.2f"))
    {
        player.setPan(pan);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Left/right balance for a non-spatial sound effect - ignored by Music "
                          "and overridden by Spatial.");

    bool spatial = player.spatial();
    if (ImGui::Checkbox("Spatial", &spatial))
    {
        player.setSpatial(spatial);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Follow this object's world position every frame and attenuate against "
                          "the listener the Scene sets from the active camera.");

    ImGui::BeginDisabled(!spatial);
    f32 minDistance = player.minDistance();
    if (ImGui::DragFloat("Min Distance", &minDistance, 0.1f, 0.0f, 1000000.0f, "%.2f"))
    {
        player.setMinDistance(minDistance);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Below this distance from the listener, a spatial voice is at full "
                          "volume.");

    f32 maxDistance = player.maxDistance();
    if (ImGui::DragFloat("Max Distance", &maxDistance, 0.1f, 0.0f, 1000000.0f, "%.2f"))
    {
        player.setMaxDistance(maxDistance);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Past this distance from the listener, a spatial voice is silent.");

    f32 rolloff = player.rolloff();
    if (ImGui::DragFloat("Rolloff", &rolloff, 0.1f, 0.0f, 100.0f, "%.2f"))
    {
        player.setRolloff(rolloff);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Shapes the attenuation curve between Min and Max Distance - higher "
                          "falls off faster.");
    ImGui::EndDisabled();

    ImGui::Separator();
    if (ImGui::Button("Play"))
        player.play();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Load the source if needed and start (or restart) playback.");
    ImGui::SameLine();
    if (ImGui::Button("Stop"))
        player.stop();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Stop playback and release the voice.");
    ImGui::SameLine();
    if (ImGui::Button("Pause"))
        player.pause();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Pause the voice in place, if one is playing.");
    ImGui::SameLine();
    if (ImGui::Button("Resume"))
        player.resume();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Resume a paused voice.");
    ImGui::SameLine();
    ImGui::TextDisabled("%s", player.playing() ? "Playing" : "Stopped");

    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawColliderComponent(GameObject& object, Collider& collider)
{
    ImGui::Indent(14.0f);

    static const char* kShapeNames[] = {"Sphere", "Box", "Capsule", "Mesh"};
    int shapeIndex = static_cast<int>(collider.shape());
    if (ImGui::Combo("Shape", &shapeIndex, kShapeNames, IM_ARRAYSIZE(kShapeNames)))
    {
        switch (static_cast<ColliderShape>(shapeIndex))
        {
        case ColliderShape::Sphere: collider.setSphere(collider.radius()); break;
        case ColliderShape::Box: collider.setBox(collider.halfExtents()); break;
        case ColliderShape::Capsule: collider.setCapsule(collider.radius(), collider.height()); break;
        case ColliderShape::Mesh: collider.setMesh(nullptr); break;
        }
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The volume this collider tests with. Mesh needs a TriangleOctree "
                         "assigned in code - it does not round-trip through the scene file.");

    switch (collider.shape())
    {
    case ColliderShape::Sphere:
    {
        f32 radius = collider.radius();
        if (ImGui::DragFloat("Radius", &radius, 0.02f, 0.001f, 1000.0f, "%.3f"))
        {
            collider.setSphere(glm::max(radius, 0.001f));
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Sphere radius, in local units before the object's own scale.");
        break;
    }
    case ColliderShape::Box:
    {
        glm::vec3 halfExtents = collider.halfExtents();
        if (ImGui::DragFloat3("Half Extents", &halfExtents.x, 0.02f, 0.001f, 1000.0f, "%.3f"))
        {
            collider.setBox(glm::max(halfExtents, glm::vec3(0.001f)));
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Half the box's size on each axis, before the object's own scale.");
        break;
    }
    case ColliderShape::Capsule:
    {
        f32 radius = collider.radius();
        f32 height = collider.height();
        bool changed = ImGui::DragFloat("Radius", &radius, 0.02f, 0.001f, 1000.0f, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Radius of the round cross-section and its two end caps.");
        changed |= ImGui::DragFloat("Height", &height, 0.02f, 0.001f, 1000.0f, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Total height cap to cap. The straight segment between the caps "
                             "is height - 2*radius, clamped to zero.");
        if (changed)
        {
            collider.setCapsule(glm::max(radius, 0.001f), glm::max(height, 0.001f));
            app().markDirty();
        }
        break;
    }
    case ColliderShape::Mesh:
        ImGui::TextDisabled(collider.mesh() ? "Mesh collider (assigned in code)."
                                            : "Mesh collider - no TriangleOctree assigned yet.");
        break;
    }

    u32 type = collider.type();
    if (ImGui::InputScalar("Type", ImGuiDataType_U32, &type))
    {
        collider.setType(type);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Collision type id. Two types only interact once "
                         "Scene::collisions().enable(typeA, typeB, response) pairs them.");

    static const char* kResponseNames[] = {"None", "Stop", "Slide", "SlideXZ"};
    int responseIndex = static_cast<int>(collider.response());
    if (ImGui::Combo("Response", &responseIndex, kResponseNames, IM_ARRAYSIZE(kResponseNames)))
    {
        collider.setResponse(static_cast<CollisionResponse>(responseIndex));
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How CollisionWorld::moveSphere() resolves a paired mover against this "
                         "collider - Stop rejects the whole step, Slide/SlideXZ deflect along "
                         "the surface, None reports the contact without moving anything.");

    const glm::mat4& transform = object.globalTransform();
    const Color outlineColor = Color::Cyan;
    switch (collider.shape())
    {
    case ColliderShape::Sphere:
        Physics::SphereShape(collider.radius()).debugDraw(transform, outlineColor);
        break;
    case ColliderShape::Box:
        Physics::BoxShape(collider.halfExtents()).debugDraw(transform, outlineColor);
        break;
    case ColliderShape::Capsule:
        Physics::CapsuleShape(collider.radius(), collider.capsuleSegmentHalfHeight())
            .debugDraw(transform, outlineColor);
        break;
    case ColliderShape::Mesh:
        if (collider.mesh())
            DebugDraw().box(collider.worldBounds(), outlineColor);
        break;
    }

    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawRigidBodyComponent(GameObject&, Physics::RigidBody& body)
{
    ImGui::Indent(14.0f);

    static const char* kBodyTypeNames[] = {"Static", "Dynamic", "Kinematic"};
    int bodyTypeIndex = static_cast<int>(body.bodyType());
    if (ImGui::Combo("Body Type", &bodyTypeIndex, kBodyTypeNames, IM_ARRAYSIZE(kBodyTypeNames)))
    {
        body.setBodyType(static_cast<Physics::BodyType>(bodyTypeIndex));
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Static never moves. Dynamic is integrated from forces and falls under "
                         "gravity. Kinematic is driven by this object's own transform and pushes "
                         "anything resting on it without ever being pushed back.");

    static const char* kShapeNames[] = {"Sphere", "Box", "Capsule"};
    int shapeIndex = static_cast<int>(body.shapeKind()) - 1;
    if (ImGui::Combo("Shape", &shapeIndex, kShapeNames, IM_ARRAYSIZE(kShapeNames)))
    {
        switch (static_cast<Physics::RigidBodyShape>(shapeIndex + 1))
        {
        case Physics::RigidBodyShape::Sphere: body.setSphere(body.radius()); break;
        case Physics::RigidBodyShape::Box: body.setBox(body.halfExtents()); break;
        case Physics::RigidBodyShape::Capsule: body.setCapsule(body.radius(), body.height()); break;
        default: break;
        }
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The volume the simulation collides and computes inertia with.");

    switch (body.shapeKind())
    {
    case Physics::RigidBodyShape::Sphere:
    {
        f32 radius = body.radius();
        if (ImGui::DragFloat("Radius", &radius, 0.02f, 0.001f, 1000.0f, "%.3f"))
        {
            body.setSphere(glm::max(radius, 0.001f));
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Sphere radius, in local units before the object's own scale.");
        break;
    }
    case Physics::RigidBodyShape::Box:
    {
        glm::vec3 halfExtents = body.halfExtents();
        if (ImGui::DragFloat3("Half Extents", &halfExtents.x, 0.02f, 0.001f, 1000.0f, "%.3f"))
        {
            body.setBox(glm::max(halfExtents, glm::vec3(0.001f)));
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Half the box's size on each axis, before the object's own scale.");
        break;
    }
    case Physics::RigidBodyShape::Capsule:
    {
        f32 radius = body.radius();
        f32 height = body.height();
        bool changed = ImGui::DragFloat("Radius", &radius, 0.02f, 0.001f, 1000.0f, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Radius of the round cross-section and its two end caps.");
        changed |= ImGui::DragFloat("Height", &height, 0.02f, 0.001f, 1000.0f, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Total height cap to cap. The straight segment between the caps "
                             "is height - 2*radius, clamped to zero.");
        if (changed)
        {
            body.setCapsule(glm::max(radius, 0.001f), glm::max(height, 0.001f));
            app().markDirty();
        }
        break;
    }
    case Physics::RigidBodyShape::None:
        break;
    }

    f32 mass = body.mass();
    if (ImGui::DragFloat("Mass", &mass, 0.05f, 0.001f, 100000.0f, "%.3f"))
    {
        body.setMass(glm::max(mass, 0.001f));
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Only used while Dynamic - Static and Kinematic are infinite mass "
                         "regardless of this value.");

    f32 friction = body.friction();
    if (ImGui::DragFloat("Friction", &friction, 0.01f, 0.0f, 10.0f, "%.3f"))
    {
        body.setFriction(glm::max(friction, 0.0f));
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Coulomb friction coefficient against whatever this body contacts.");

    f32 restitution = body.restitution();
    if (ImGui::DragFloat("Restitution", &restitution, 0.01f, 0.0f, 1.0f, "%.3f"))
    {
        body.setRestitution(glm::clamp(restitution, 0.0f, 1.0f));
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Bounciness of a contact - 0 absorbs all of the closing velocity, 1 "
                         "reflects it back unchanged.");

    u32 collisionGroup = body.collisionGroup();
    if (ImGui::InputScalar("Collision Group", ImGuiDataType_U32, &collisionGroup, nullptr,
                           nullptr, "%08X", ImGuiInputTextFlags_CharsHexadecimal))
    {
        body.setCollisionGroup(collisionGroup);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Bitmask identifying which group(s) this body belongs to.");

    u32 collisionMask = body.collisionMask();
    if (ImGui::InputScalar("Collision Mask", ImGuiDataType_U32, &collisionMask, nullptr,
                           nullptr, "%08X", ImGuiInputTextFlags_CharsHexadecimal))
    {
        body.setCollisionMask(collisionMask);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Bitmask of the group(s) this body is allowed to collide with.");

    bool enabled = body.enabled();
    if (ImGui::Checkbox("Enabled", &enabled))
    {
        body.setEnabled(enabled);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("A disabled body stays registered but is skipped by broadphase, "
                         "contacts, and integration.");

    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawJointComponent(GameObject& object, Physics::Joint& joint)
{
    ImGui::Indent(14.0f);

    // Mouse is left out on purpose: it is the editor's own dragging tool,
    // not something a scene is authored with.
    static const char* kKindNames[] = {"Distance", "Fixed",  "Hinge", "Slider",
                                       "Piston",   "Universal", "Point", "Wheel"};
    int kindIndex = static_cast<int>(joint.kind());
    if (ImGui::Combo("Kind", &kindIndex, kKindNames, IM_ARRAYSIZE(kKindNames)))
    {
        GameObject* connectedBody = joint.connectedBody();
        const bool enabled = joint.enabled();
        object.removeComponent<Physics::Joint>();
        Physics::Joint* replacement = nullptr;
        switch (static_cast<Physics::JointKind>(kindIndex))
        {
        case Physics::JointKind::Distance:
            replacement = object.addComponent<Physics::DistanceJoint>();
            break;
        case Physics::JointKind::Fixed:
            replacement = object.addComponent<Physics::FixedJoint>();
            break;
        case Physics::JointKind::Hinge:
            replacement = object.addComponent<Physics::HingeJoint>();
            break;
        case Physics::JointKind::Slider:
            replacement = object.addComponent<Physics::SliderJoint>();
            break;
        case Physics::JointKind::Piston:
            replacement = object.addComponent<Physics::PistonJoint>();
            break;
        case Physics::JointKind::Universal:
            replacement = object.addComponent<Physics::UniversalJoint>();
            break;
        case Physics::JointKind::Point:
            replacement = object.addComponent<Physics::PointJoint>();
            break;
        case Physics::JointKind::Wheel:
            replacement = object.addComponent<Physics::WheelJoint>();
            break;
        default:
            break;
        }
        if (replacement)
        {
            replacement->setConnectedBody(connectedBody);
            replacement->setEnabled(enabled);
        }
        app().markDirty();
        ImGui::Unindent(14.0f);
        return;
    }

    ImGui::Text("Connected Body: %s",
               joint.connectedBody() ? joint.connectedBody()->name().c_str() : "(none)");
    ImGui::Button("Drop object here", ImVec2(-FLT_MIN, 0.0f));
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kGameObjectDragPayload))
        {
            const u64 id = *static_cast<const u64*>(payload->Data);
            GameObject* connectedObject = app().scene().findGameObject(id);
            if (connectedObject && connectedObject != &object)
            {
                app().recordUndo();
                joint.setConnectedBody(connectedObject);
                app().markDirty();
            }
        }
        ImGui::EndDragDropTarget();
    }
    if (ImGui::Button("Clear"))
    {
        app().recordUndo();
        joint.setConnectedBody(nullptr);
        app().markDirty();
    }

    bool enabled = joint.enabled();
    if (ImGui::Checkbox("Enabled", &enabled))
    {
        joint.setEnabled(enabled);
        app().markDirty();
    }

    switch (joint.kind())
    {
    case Physics::JointKind::Hinge:
    {
        Physics::HingeJoint& hinge = static_cast<Physics::HingeJoint&>(joint);
        glm::vec3 axis = hinge.authoredAxis();
        if (ImGui::DragFloat3("Axis", &axis.x, 0.01f))
        {
            hinge.setAuthoredAxis(axis);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Hinge rotation axis, in this object's own local space.");

        f32 minAngle = glm::degrees(hinge.minAngle());
        f32 maxAngle = glm::degrees(hinge.maxAngle());
        bool limitsChanged = ImGui::DragFloat("Min Angle", &minAngle, 1.0f, -180.0f, 0.0f);
        limitsChanged |= ImGui::DragFloat("Max Angle", &maxAngle, 1.0f, 0.0f, 180.0f);
        if (limitsChanged)
        {
            hinge.setLimits(glm::radians(minAngle), glm::radians(maxAngle));
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("How far the hinge may open to either side of its rest angle. "
                              "-180/180 leaves it unconstrained.");

        f32 motorVelocity = hinge.motorTargetVelocity();
        f32 motorTorque = hinge.motorMaxTorque();
        bool motorChanged = ImGui::DragFloat("Motor Velocity", &motorVelocity, 0.01f);
        motorChanged |= ImGui::DragFloat("Motor Max Torque", &motorTorque, 0.1f, 0.0f, 100000.0f);
        if (motorChanged)
        {
            hinge.setMotor(motorVelocity, motorTorque);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Angular velocity the motor drives the hinge towards, and the "
                              "torque it may spend doing it. Max Torque 0 disables the motor.");
        break;
    }
    case Physics::JointKind::Slider:
    {
        Physics::SliderJoint& slider = static_cast<Physics::SliderJoint&>(joint);
        glm::vec3 axis = slider.authoredAxis();
        if (ImGui::DragFloat3("Axis", &axis.x, 0.01f))
        {
            slider.setAuthoredAxis(axis);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Slider travel axis, in this object's own local space.");

        f32 minDistance = slider.minDistance();
        f32 maxDistance = slider.maxDistance();
        bool limitsChanged = ImGui::DragFloat("Min Distance", &minDistance, 0.01f, -1000.0f, 0.0f);
        limitsChanged |= ImGui::DragFloat("Max Distance", &maxDistance, 0.01f, 0.0f, 1000.0f);
        if (limitsChanged)
        {
            slider.setLimits(minDistance, maxDistance);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("How far the slider may travel along its axis from the anchor.");

        f32 motorVelocity = slider.motorTargetVelocity();
        f32 motorForce = slider.motorMaxForce();
        bool motorChanged = ImGui::DragFloat("Motor Velocity", &motorVelocity, 0.01f);
        motorChanged |= ImGui::DragFloat("Motor Max Force", &motorForce, 0.1f, 0.0f, 100000.0f);
        if (motorChanged)
        {
            slider.setMotor(motorVelocity, motorForce);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Velocity the motor drives the slider towards, and the force it "
                              "may spend doing it - an elevator platform's own lift motor. Max "
                              "Force 0 disables the motor.");
        break;
    }
    case Physics::JointKind::Piston:
    {
        Physics::PistonJoint& piston = static_cast<Physics::PistonJoint&>(joint);
        glm::vec3 axis = piston.authoredAxis();
        if (ImGui::DragFloat3("Axis", &axis.x, 0.01f))
        {
            piston.setAuthoredAxis(axis);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Piston travel and spin axis, in this object's own local space.");

        f32 minDistance = piston.minLinearDistance();
        f32 maxDistance = piston.maxLinearDistance();
        bool linearLimitsChanged =
            ImGui::DragFloat("Min Distance", &minDistance, 0.01f, -1000.0f, 0.0f);
        linearLimitsChanged |= ImGui::DragFloat("Max Distance", &maxDistance, 0.01f, 0.0f, 1000.0f);
        if (linearLimitsChanged)
        {
            piston.setLinearLimits(minDistance, maxDistance);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("How far the piston may travel along its axis from the anchor.");

        f32 minAngle = glm::degrees(piston.minAngularAngle());
        f32 maxAngle = glm::degrees(piston.maxAngularAngle());
        bool angularLimitsChanged = ImGui::DragFloat("Min Angle", &minAngle, 1.0f, -180.0f, 0.0f);
        angularLimitsChanged |= ImGui::DragFloat("Max Angle", &maxAngle, 1.0f, 0.0f, 180.0f);
        if (angularLimitsChanged)
        {
            piston.setAngularLimits(glm::radians(minAngle), glm::radians(maxAngle));
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("How far the piston may spin around its axis from the rest angle.");

        f32 linearMotorVelocity = piston.linearMotorTargetVelocity();
        f32 linearMotorForce = piston.linearMotorMaxForce();
        bool linearMotorChanged = ImGui::DragFloat("Linear Motor Velocity", &linearMotorVelocity, 0.01f);
        linearMotorChanged |=
            ImGui::DragFloat("Linear Motor Max Force", &linearMotorForce, 0.1f, 0.0f, 100000.0f);
        if (linearMotorChanged)
        {
            piston.setLinearMotor(linearMotorVelocity, linearMotorForce);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Velocity the motor drives the piston's travel towards - an "
                              "elevator's lift motor. Max Force 0 disables it.");

        f32 angularMotorVelocity = piston.angularMotorTargetVelocity();
        f32 angularMotorTorque = piston.angularMotorMaxTorque();
        bool angularMotorChanged =
            ImGui::DragFloat("Angular Motor Velocity", &angularMotorVelocity, 0.01f);
        angularMotorChanged |=
            ImGui::DragFloat("Angular Motor Max Torque", &angularMotorTorque, 0.1f, 0.0f, 100000.0f);
        if (angularMotorChanged)
        {
            piston.setAngularMotor(angularMotorVelocity, angularMotorTorque);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Angular velocity the motor drives the piston's spin towards. Max "
                              "Torque 0 disables it.");
        break;
    }
    case Physics::JointKind::Universal:
    {
        Physics::UniversalJoint& universal = static_cast<Physics::UniversalJoint&>(joint);
        glm::vec3 axis = universal.authoredAxis();
        if (ImGui::DragFloat3("Axis", &axis.x, 0.01f))
        {
            universal.setAuthoredAxis(axis);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("First hinge axis, in this object's own local space. The second "
                             "axis is derived perpendicular to it automatically.");
        break;
    }
    case Physics::JointKind::Distance:
    {
        Physics::DistanceJoint& distance = static_cast<Physics::DistanceJoint&>(joint);
        f32 minDistance = distance.authoredMinDistance();
        f32 maxDistance = distance.authoredMaxDistance();
        bool changed = ImGui::DragFloat("Min Distance", &minDistance, 0.01f, 0.0f, 1000.0f);
        changed |= ImGui::DragFloat("Max Distance", &maxDistance, 0.01f, 0.0f, 1000.0f);
        if (changed)
        {
            distance.setAuthoredDistance(minDistance, maxDistance);
            app().markDirty();
        }
        break;
    }
    case Physics::JointKind::Wheel:
    {
        Physics::WheelJoint& wheel = static_cast<Physics::WheelJoint&>(joint);
        ImGui::TextDisabled("Put this on the wheel; connect it to the chassis.");

        glm::vec3 suspensionAxis = wheel.authoredSuspensionAxis();
        if (ImGui::DragFloat3("Suspension Axis", &suspensionAxis.x, 0.01f))
        {
            wheel.setAuthoredSuspensionAxis(suspensionAxis);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Direction the strut travels, in the wheel's own local space - "
                              "down, towards the ground.");

        glm::vec3 spinAxis = wheel.authoredSpinAxis();
        if (ImGui::DragFloat3("Spin Axis", &spinAxis.x, 0.01f))
        {
            wheel.setAuthoredSpinAxis(spinAxis);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The axle the wheel turns on, in the wheel's own local space - "
                              "sideways, across the car.");

        f32 restLength = wheel.suspensionRestLength();
        f32 stiffness = wheel.suspensionStiffness();
        f32 damping = wheel.suspensionDamping();
        bool suspensionChanged = ImGui::DragFloat("Rest Length", &restLength, 0.005f, 0.0f, 5.0f);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Where the strut sits with no weight on it. The wheel hangs this "
                              "far below its mount.");
        suspensionChanged |= ImGui::DragFloat("Stiffness", &stiffness, 100.0f, 0.0f, 2.0e6f,
                                              "%.0f N/m");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Spring rate. A car corner carrying 375 kg and settling 20 cm "
                              "needs about 18000 N/m; stiffer rides harder and rolls less. "
                              "Solved inside the constraint, so any value here is stable.");
        suspensionChanged |= ImGui::DragFloat("Damping", &damping, 10.0f, 0.0f, 2.0e5f,
                                              "%.0f N/(m/s)");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The shock absorber. Around a tenth of the stiffness settles "
                              "without wallowing; too little bounces, too much rides like a "
                              "brick.");
        if (suspensionChanged)
        {
            wheel.setSuspension(restLength, stiffness, damping);
            app().markDirty();
        }

        f32 minSteer = glm::degrees(wheel.minSteeringAngle());
        f32 maxSteer = glm::degrees(wheel.maxSteeringAngle());
        bool steerLimitsChanged = ImGui::DragFloat("Min Steer", &minSteer, 1.0f, -90.0f, 0.0f);
        steerLimitsChanged |= ImGui::DragFloat("Max Steer", &maxSteer, 1.0f, 0.0f, 90.0f);
        if (steerLimitsChanged)
        {
            wheel.setSteeringLimits(glm::radians(minSteer), glm::radians(maxSteer));
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("How far this wheel may turn. Around 35 degrees on the front "
                              "pair; 0/0 locks the rear pair straight.");

        f32 steerVelocity = wheel.steeringMotorTargetVelocity();
        f32 steerTorque = wheel.steeringMotorMaxTorque();
        bool steerMotorChanged = ImGui::DragFloat("Steer Velocity", &steerVelocity, 0.01f);
        steerMotorChanged |= ImGui::DragFloat("Steer Max Torque", &steerTorque, 1.0f, 0.0f, 1.0e5f);
        if (steerMotorChanged)
        {
            wheel.setSteeringMotor(steerVelocity, steerTorque);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The steering rack. Max Torque 0 disables it.");

        f32 spinVelocity = wheel.spinMotorTargetVelocity();
        f32 spinTorque = wheel.spinMotorMaxTorque();
        bool spinMotorChanged = ImGui::DragFloat("Drive Velocity", &spinVelocity, 0.1f);
        spinMotorChanged |= ImGui::DragFloat("Drive Max Torque", &spinTorque, 1.0f, 0.0f, 1.0e5f);
        if (spinMotorChanged)
        {
            wheel.setSpinMotor(spinVelocity, spinTorque);
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Engine torque at this wheel: target wheel speed and the torque "
                              "available to reach it. Set the speed to 0 with torque left on "
                              "and it becomes the brake.");

        ImGui::Separator();
        ImGui::Text("Travel %.3f m   Steer %.1f deg   Wheel %.1f rad/s", wheel.suspensionTravel(),
                    static_cast<double>(glm::degrees(wheel.steeringAngle())),
                    static_cast<double>(wheel.spinAngularVelocity()));
        break;
    }
    case Physics::JointKind::Fixed:
    case Physics::JointKind::Point:
    default:
        break;
    }

    ImGui::Unindent(14.0f);
}

// The literal the script itself wrote, formatted the way it reads in the
// source - what the tooltip shows a value is being overridden away from.
static std::string zenPropertyDefaultText(const ScriptProperty& property)
{
    char buffer[128];
    switch (property.kind)
    {
    case ScriptProperty::Kind::Number:
        if (property.integer)
            std::snprintf(buffer, sizeof(buffer), "%lld", (long long)property.number);
        else
            std::snprintf(buffer, sizeof(buffer), "%g", property.number);
        break;
    case ScriptProperty::Kind::String:
        std::snprintf(buffer, sizeof(buffer), "\"%s\"", property.text.c_str());
        break;
    case ScriptProperty::Kind::Bool:
        std::snprintf(buffer, sizeof(buffer), "%s", property.flag ? "True" : "False");
        break;
    default:
        buffer[0] = '\0';
        break;
    }
    return std::string(buffer);
}

void InspectorPanel::drawZenBehaviourComponent(GameObject& object, ZenBehaviour& behaviour)
{
    ImGui::Indent(14.0f);

    // Refresh the path field from the component whenever the selection
    // switches to a different object - otherwise it would keep showing
    // whichever object's path was last typed into it.
    if (mZenBehaviourObjectId != object.id())
    {
        mZenBehaviourObjectId = object.id();
        std::snprintf(mZenScriptPathBuffer, sizeof(mZenScriptPathBuffer), "%s",
                     behaviour.scriptPath().c_str());
    }

    ImGui::InputText("Script", mZenScriptPathBuffer, sizeof(mZenScriptPathBuffer));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Path to a .py script, relative to the working directory or absolute. "
                         "Load compiles it and exposes this object as self.node.");
    if (ImGui::BeginDragDropTarget())
    {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
        {
            const std::string path(static_cast<const char*>(payload->Data), payload->DataSize);
            if (lowerExtension(path) == "py")
            {
                std::snprintf(mZenScriptPathBuffer, sizeof(mZenScriptPathBuffer), "%s", path.c_str());
                behaviour.loadFile(path);
                app().markDirty();
            }
            else
            {
                app().toasts().error("Drop a .py script here");
            }
        }
        ImGui::EndDragDropTarget();
    }

    if (ImGui::Button("Browse..."))
    {
        mZenScriptDialogTarget = object.id();
        const std::string& last = app().settings().lastOpenDirectory;
        mZenScriptDialog.Open(ImGuiFileDialog::Mode::OpenFile,
                              last.empty() ? app().assetBrowserRoot() : last);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Pick the .py script from disk instead of typing its path.");

    if (mZenScriptDialogTarget == object.id() &&
        mZenScriptDialog.Render(app().assetBrowserRoot(), app().assetBrowserRoot(),
                                app().assetBrowserRoot()))
    {
        const ImGuiFileDialog::Result result = mZenScriptDialog.ConsumeResult();
        mZenScriptDialogTarget = 0;
        if (result.accepted)
        {
            const std::string path = result.path.string();
            app().settings().lastOpenDirectory = result.path.parent_path().string();
            std::snprintf(mZenScriptPathBuffer, sizeof(mZenScriptPathBuffer), "%s", path.c_str());
            // Picking a file is a load: a "Browse..." that left the object
            // still running the old script would be a trap.
            behaviour.loadFile(path);
            app().markDirty();
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("Load"))
    {
        behaviour.loadFile(mZenScriptPathBuffer);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Compile the path above and bind it to this object.");

    ImGui::SameLine();
    ImGui::BeginDisabled(behaviour.scriptPath().empty());
    if (ImGui::Button("Reload"))
        behaviour.reload();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Re-read the loaded file from disk and rebind it.");

    if (behaviour.hasError())
    {
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.4f, 0.4f, 1.0f));
        ImGui::TextWrapped("%s", behaviour.lastError().c_str());
        ImGui::PopStyleColor();
    }

    drawZenBehaviourProperties(behaviour);

    ImGui::Unindent(14.0f);
}

// The values the script declares in its class body (or optionally in
// __init__), each one editable per object. An edited value becomes an
// override stored on this component and is written into the script instance
// after its optional constructor; the rest simply show the script default.
void InspectorPanel::drawZenBehaviourProperties(ZenBehaviour& behaviour)
{
    const usize count = behaviour.declaredPropertyCount();
    if (count == 0)
    {
        if (!behaviour.scriptPath().empty() && !behaviour.hasError())
        {
            ImGui::Spacing();
            ImGui::TextDisabled("No properties: the script declares none.");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("A literal assigned in the class body shows up here, one "
                                 "row each:\n\n    class Rotate:\n        speed = 90.0\n"
                                 "        label = \"spin\"\n\nAssignments to self.<name> "
                                 "inside def __init__(self) count too. A name starting with "
                                 "_ is private and left out, and so is a field only ever "
                                 "written inside a method.");
        }
        return;
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Properties");

    for (usize i = 0; i < count; ++i)
    {
        const ScriptProperty* declared = behaviour.declaredPropertyAt(i);
        if (!declared)
            continue;

        const ScriptProperty* overridden = behaviour.findOverride(declared->name);
        const ScriptProperty& shown = overridden ? *overridden : *declared;

        ImGui::PushID((int)i);

        // An overridden row is tinted, so "this object differs from the
        // script" is visible without opening anything.
        if (overridden)
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.98f, 0.83f, 0.38f, 1.0f));

        bool edited = false;
        switch (shown.kind)
        {
        case ScriptProperty::Kind::Number:
            if (shown.integer)
            {
                int value = (int)shown.number;
                if (ImGui::DragInt(declared->name.c_str(), &value))
                {
                    behaviour.setNumberOverride(declared->name, (f64)value, true);
                    edited = true;
                }
            }
            else
            {
                f32 value = (f32)shown.number;
                if (ImGui::DragFloat(declared->name.c_str(), &value, 0.01f))
                {
                    behaviour.setNumberOverride(declared->name, (f64)value, false);
                    edited = true;
                }
            }
            break;
        case ScriptProperty::Kind::String:
        {
            char buffer[256];
            std::snprintf(buffer, sizeof(buffer), "%s", shown.text.c_str());
            if (ImGui::InputText(declared->name.c_str(), buffer, sizeof(buffer)))
            {
                behaviour.setStringOverride(declared->name, buffer);
                edited = true;
            }
            break;
        }
        case ScriptProperty::Kind::Bool:
        {
            bool value = shown.flag;
            if (ImGui::Checkbox(declared->name.c_str(), &value))
            {
                behaviour.setBoolOverride(declared->name, value);
                edited = true;
            }
            break;
        }
        }

        if (overridden)
            ImGui::PopStyleColor();

        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(overridden
                                  ? "self.%s - overridden on this object.\nThe script's own "
                                    "default is %s.\nRight-click to go back to it."
                                  : "self.%s - the script's own default (%s).\nEditing it here "
                                    "overrides it on this object only; every other object on "
                                    "the same script is untouched.",
                              declared->name.c_str(), zenPropertyDefaultText(*declared).c_str());

        if (overridden && ImGui::IsItemClicked(ImGuiMouseButton_Right))
        {
            behaviour.clearOverride(declared->name);
            edited = true;
        }

        if (edited)
            app().markDirty();

        ImGui::PopID();
    }

    if (behaviour.overrideCount() > 0)
    {
        if (ImGui::Button("Reset All Overrides"))
        {
            behaviour.clearOverrides();
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Drops every override on this object and puts the script's own "
                             "defaults back. Other objects on the same script are untouched.");
    }
}

void InspectorPanel::drawOrbitComponent(GameObject&, Orbit& orbit)
{
    ImGui::Indent(14.0f);

    GameObject* target = orbit.target();
    glm::vec3 targetPoint = orbit.targetPoint();
    if (drawOrbitTargetSlot(app(), target, targetPoint))
    {
        if (target)
            orbit.setTarget(target);
        else
            orbit.setTargetPoint(targetPoint);
        app().markDirty();
    }

    f32 distance = orbit.distance();
    if (ImGui::DragFloat("Distance", &distance, 0.05f, 0.01f, 100000.0f))
    {
        orbit.setDistance(distance);
        app().markDirty();
    }

    f32 yaw = orbit.yaw();
    f32 pitch = orbit.pitch();
    bool yawPitchChanged = false;
    yawPitchChanged |= ImGui::DragFloat("Yaw", &yaw, 0.5f, -100000.0f, 100000.0f, "%.1f°");
    yawPitchChanged |= ImGui::DragFloat("Pitch", &pitch, 0.5f, -89.0f, 89.0f, "%.1f°");
    if (yawPitchChanged)
    {
        orbit.setYawPitch(yaw, pitch);
        app().markDirty();
    }

    f32 pitchLimit = orbit.pitchLimit();
    if (ImGui::DragFloat("Pitch Limit", &pitchLimit, 0.5f, 1.0f, 89.9f, "%.1f°"))
    {
        orbit.setPitchLimit(pitchLimit);
        app().markDirty();
    }
    f32 orbitSpeed = orbit.orbitSpeed();
    if (ImGui::DragFloat("Orbit Speed", &orbitSpeed, 0.01f, 0.001f, 10.0f))
    {
        orbit.setOrbitSpeed(orbitSpeed);
        app().markDirty();
    }
    f32 zoomSpeed = orbit.zoomSpeed();
    if (ImGui::DragFloat("Zoom Speed", &zoomSpeed, 0.05f, 0.001f, 100.0f))
    {
        orbit.setZoomSpeed(zoomSpeed);
        app().markDirty();
    }

    MouseButton orbitButton = orbit.orbitButton();
    if (drawMouseButtonCombo("Orbit Button", orbitButton))
    {
        orbit.setOrbitButton(orbitButton);
        app().markDirty();
    }

    bool requireOrbitButton = orbit.requiresOrbitButton();
    if (ImGui::Checkbox("Require Orbit Button Held", &requireOrbitButton))
    {
        orbit.setRequireOrbitButton(requireOrbitButton);
        app().markDirty();
    }
    ImGui::SameLine();
    bool invertY = orbit.invertY();
    if (ImGui::Checkbox("Invert Y", &invertY))
    {
        orbit.setInvertY(invertY);
        app().markDirty();
    }

    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawMayaComponent(GameObject&, Maya& maya)
{
    ImGui::Indent(14.0f);

    GameObject* target = maya.target();
    glm::vec3 targetPoint = maya.targetPoint();
    if (drawOrbitTargetSlot(app(), target, targetPoint))
    {
        if (target)
            maya.setTarget(target);
        else
            maya.setTargetPoint(targetPoint);
        app().markDirty();
    }

    f32 distance = maya.distance();
    if (ImGui::DragFloat("Distance", &distance, 0.05f, 0.01f, 100000.0f))
    {
        maya.setDistance(distance);
        app().markDirty();
    }

    f32 yaw = maya.yaw();
    f32 pitch = maya.pitch();
    bool yawPitchChanged = false;
    yawPitchChanged |= ImGui::DragFloat("Yaw", &yaw, 0.5f, -100000.0f, 100000.0f, "%.1f°");
    yawPitchChanged |= ImGui::DragFloat("Pitch", &pitch, 0.5f, -89.0f, 89.0f, "%.1f°");
    if (yawPitchChanged)
    {
        maya.setYawPitch(yaw, pitch);
        app().markDirty();
    }

    f32 pitchLimit = maya.pitchLimit();
    if (ImGui::DragFloat("Pitch Limit", &pitchLimit, 0.5f, 1.0f, 89.9f, "%.1f°"))
    {
        maya.setPitchLimit(pitchLimit);
        app().markDirty();
    }
    f32 orbitSpeed = maya.orbitSpeed();
    if (ImGui::DragFloat("Orbit Speed", &orbitSpeed, 0.01f, 0.001f, 10.0f))
    {
        maya.setOrbitSpeed(orbitSpeed);
        app().markDirty();
    }
    f32 panSpeed = maya.panSpeed();
    if (ImGui::DragFloat("Pan Speed", &panSpeed, 0.001f, 0.0001f, 10.0f))
    {
        maya.setPanSpeed(panSpeed);
        app().markDirty();
    }
    f32 zoomSpeed = maya.zoomSpeed();
    if (ImGui::DragFloat("Zoom Speed", &zoomSpeed, 0.05f, 0.001f, 100.0f))
    {
        maya.setZoomSpeed(zoomSpeed);
        app().markDirty();
    }

    KeyCode modifierKey = maya.modifierKey();
    if (drawKeyCombo("Modifier Key", modifierKey))
    {
        maya.setModifierKey(modifierKey);
        app().markDirty();
    }
    MouseButton orbitButton = maya.orbitButton();
    if (drawMouseButtonCombo("Orbit Button", orbitButton))
    {
        maya.setOrbitButton(orbitButton);
        app().markDirty();
    }
    MouseButton panButton = maya.panButton();
    if (drawMouseButtonCombo("Pan Button", panButton))
    {
        maya.setPanButton(panButton);
        app().markDirty();
    }
    MouseButton dollyButton = maya.dollyButton();
    if (drawMouseButtonCombo("Dolly Button", dollyButton))
    {
        maya.setDollyButton(dollyButton);
        app().markDirty();
    }
    bool invertY = maya.invertY();
    if (ImGui::Checkbox("Invert Y", &invertY))
    {
        maya.setInvertY(invertY);
        app().markDirty();
    }

    ImGui::Unindent(14.0f);
}

void InspectorPanel::resetPrimitiveSettings(PrimitiveKind kind)
{
    mPrimitiveKind = kind;
    mPrimitiveSettings = PrimitiveSettings();

    switch (kind)
    {
    case PrimitiveKind::Cube:
        break;
    case PrimitiveKind::Sphere:
        mPrimitiveSettings.dimensions.x = 0.5f;
        mPrimitiveSettings.segmentsA = 16;
        mPrimitiveSettings.segmentsB = 24;
        break;
    case PrimitiveKind::Plane:
        mPrimitiveSettings.dimensions.x = 2.0f;
        mPrimitiveSettings.dimensions.y = 2.0f;
        mPrimitiveSettings.segmentsA = 1;
        mPrimitiveSettings.segmentsB = 1;
        break;
    case PrimitiveKind::Cylinder:
    case PrimitiveKind::Cone:
        mPrimitiveSettings.dimensions.x = 0.5f;
        mPrimitiveSettings.dimensions.y = 1.0f;
        mPrimitiveSettings.segmentsB = 24;
        break;
    case PrimitiveKind::Capsule:
        mPrimitiveSettings.dimensions.x = 0.5f;
        mPrimitiveSettings.dimensions.y = 1.0f;
        mPrimitiveSettings.segmentsA = 8;
        mPrimitiveSettings.segmentsB = 24;
        break;
    case PrimitiveKind::Torus:
        mPrimitiveSettings.dimensions.x = 1.0f;
        mPrimitiveSettings.dimensions.y = 0.25f;
        mPrimitiveSettings.segmentsA = 24;
        mPrimitiveSettings.segmentsB = 12;
        break;
    }
}

void InspectorPanel::drawMeshRenderer(GameObject& object, MeshRenderer& renderer)
{
    static const char* kPrimitiveNames[] = {"Cube", "Sphere",  "Plane", "Cylinder",
                                            "Cone", "Capsule", "Torus"};

    AssetManager& assets = AssetManager::getSingleton();
    const MeshDesc& mesh = assets.meshDesc(renderer.mesh());
    const bool isPrimitive = mesh.source >= MeshSource::Box && mesh.source <= MeshSource::Torus;

    // Once a mesh is assigned - primitive or file - there is no
    // re-parametrize-in-place: delete and recreate the node to change
    // shape/dimensions. The picker below only ever runs for a MeshRenderer
    // still waiting for a mesh (e.g. Hierarchy > Create > Special Nodes >
    // Mesh Instance).
    if (renderer.mesh().valid())
    {
        if (isPrimitive)
            ImGui::TextDisabled("%s", meshSourceName(mesh.source));
        else if (!mesh.file.empty())
            ImGui::TextDisabled("%s", mesh.file.c_str());
        drawMeshMaterial(renderer);
        return;
    }

    if (mPrimitiveObjectId != object.id())
    {
        mPrimitiveObjectId = object.id();

        PrimitiveKind kind = PrimitiveKind::Cube;
        for (int i = 0; i < IM_ARRAYSIZE(kPrimitiveNames); ++i)
            if (object.name() == kPrimitiveNames[i])
            {
                kind = static_cast<PrimitiveKind>(i);
                break;
            }
        resetPrimitiveSettings(kind);
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Create Primitive");

    int kindIndex = static_cast<int>(mPrimitiveKind);
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::Combo("##PrimitiveShape", &kindIndex, kPrimitiveNames,
                     IM_ARRAYSIZE(kPrimitiveNames)))
        resetPrimitiveSettings(static_cast<PrimitiveKind>(kindIndex));

    switch (mPrimitiveKind)
    {
    case PrimitiveKind::Cube:
        ImGui::DragFloat("Width", &mPrimitiveSettings.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Height", &mPrimitiveSettings.dimensions.y, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Depth", &mPrimitiveSettings.dimensions.z, 0.05f, 0.01f, 10000.0f);
        break;
    case PrimitiveKind::Sphere:
        ImGui::DragFloat("Radius", &mPrimitiveSettings.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragInt("Rings", &mPrimitiveSettings.segmentsA, 0.2f, 3, 128);
        ImGui::DragInt("Slices", &mPrimitiveSettings.segmentsB, 0.2f, 3, 128);
        break;
    case PrimitiveKind::Plane:
        ImGui::DragFloat("Width", &mPrimitiveSettings.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Depth", &mPrimitiveSettings.dimensions.y, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("UV Tiles", &mPrimitiveSettings.uvTiles, 0.1f, 0.01f, 10000.0f);
        ImGui::DragInt("Segments X", &mPrimitiveSettings.segmentsA, 0.2f, 1, 256);
        ImGui::DragInt("Segments Z", &mPrimitiveSettings.segmentsB, 0.2f, 1, 256);
        break;
    case PrimitiveKind::Cylinder:
    case PrimitiveKind::Cone:
        ImGui::DragFloat("Radius", &mPrimitiveSettings.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Height", &mPrimitiveSettings.dimensions.y, 0.05f, 0.01f, 10000.0f);
        ImGui::DragInt("Slices", &mPrimitiveSettings.segmentsB, 0.2f, 3, 128);
        break;
    case PrimitiveKind::Capsule:
        ImGui::DragFloat("Radius", &mPrimitiveSettings.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Height", &mPrimitiveSettings.dimensions.y, 0.05f, 0.01f, 10000.0f);
        ImGui::DragInt("Rings", &mPrimitiveSettings.segmentsA, 0.2f, 2, 128);
        ImGui::DragInt("Slices", &mPrimitiveSettings.segmentsB, 0.2f, 3, 128);
        break;
    case PrimitiveKind::Torus:
        ImGui::DragFloat("Major Radius", &mPrimitiveSettings.dimensions.x, 0.05f, 0.01f, 10000.0f);
        ImGui::DragFloat("Minor Radius", &mPrimitiveSettings.dimensions.y, 0.05f, 0.01f, 10000.0f);
        ImGui::DragInt("Major Segments", &mPrimitiveSettings.segmentsA, 0.2f, 3, 128);
        ImGui::DragInt("Minor Segments", &mPrimitiveSettings.segmentsB, 0.2f, 3, 128);
        break;
    }

    if (ImGui::Button("OK", ImVec2(-FLT_MIN, 0.0f)))
    {
        app().recordUndo();
        MeshHandle handle;
        switch (mPrimitiveKind)
        {
        case PrimitiveKind::Cube:
            handle = assets.createBox(mPrimitiveSettings.dimensions);
            break;
        case PrimitiveKind::Sphere:
            handle = assets.createSphere(mPrimitiveSettings.dimensions.x,
                                         static_cast<u32>(mPrimitiveSettings.segmentsA),
                                         static_cast<u32>(mPrimitiveSettings.segmentsB));
            break;
        case PrimitiveKind::Plane:
            handle = assets.createPlane(
                mPrimitiveSettings.dimensions.x, mPrimitiveSettings.dimensions.y,
                static_cast<u32>(mPrimitiveSettings.segmentsA),
                static_cast<u32>(mPrimitiveSettings.segmentsB), mPrimitiveSettings.uvTiles);
            break;
        case PrimitiveKind::Cylinder:
            handle = assets.createCylinder(mPrimitiveSettings.dimensions.x,
                                           mPrimitiveSettings.dimensions.y,
                                           static_cast<u32>(mPrimitiveSettings.segmentsB));
            break;
        case PrimitiveKind::Cone:
            handle =
                assets.createCone(mPrimitiveSettings.dimensions.x, mPrimitiveSettings.dimensions.y,
                                  static_cast<u32>(mPrimitiveSettings.segmentsB));
            break;
        case PrimitiveKind::Capsule:
            handle = assets.createCapsule(mPrimitiveSettings.dimensions.x,
                                          mPrimitiveSettings.dimensions.y,
                                          static_cast<u32>(mPrimitiveSettings.segmentsA),
                                          static_cast<u32>(mPrimitiveSettings.segmentsB));
            break;
        case PrimitiveKind::Torus:
            handle =
                assets.createTorus(mPrimitiveSettings.dimensions.x, mPrimitiveSettings.dimensions.y,
                                   static_cast<u32>(mPrimitiveSettings.segmentsA),
                                   static_cast<u32>(mPrimitiveSettings.segmentsB));
            break;
        }

        if (handle.valid())
        {
            renderer.setMesh(handle);
            if (!renderer.materialOverrides())
                renderer.setMaterialOverride(0, defaultPrimitiveMaterial());
            app().scene().update(0.0f);
            app().scene().rebuildStaticIndex();
            app().scene().rebuildDynamicIndex();
            app().markDirty();
        }
    }
}

// One slot's worth of fields - Base Color through Clear Textures, the same
// set every slot needs. Returns true if `material` changed; the caller
// decides what that means for the slot it came from.
bool InspectorPanel::drawMaterialFields(Material& material)
{
    bool changed = false;
    static const char* kBlendNames[] = {"Opaque", "Alpha", "Additive", "Multiply",
                                        "Premultiplied Alpha", "Add Colors", "Subtract Colors"};
    int blend = static_cast<int>(material.blend);
    if (ImGui::Combo("Blend Mode", &blend, kBlendNames, IM_ARRAYSIZE(kBlendNames)))
    {
        material.blend = static_cast<BlendMode>(blend);
        changed = true;
    }

    static const char* kCullNames[] = {"None", "Back", "Front"};
    int cull = static_cast<int>(material.cull);
    ImGui::BeginDisabled((material.flags & MaterialTwoSided) != 0);
    if (ImGui::Combo("Cull Mode", &cull, kCullNames, IM_ARRAYSIZE(kCullNames)))
    {
        material.cull = static_cast<CullMode>(cull);
        changed = true;
    }
    ImGui::EndDisabled();

    bool depthWrite = (material.flags & MaterialNoDepthWrite) == 0;
    if (ImGui::Checkbox("Depth Write", &depthWrite))
    {
        material.flags = depthWrite ? (material.flags & ~MaterialNoDepthWrite)
                                    : (material.flags | MaterialNoDepthWrite);
        changed = true;
    }

    if (material.textures[SlotLightmap].texture.valid())
    {
        ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.55f, 1.0f), ICON_MDI_LIGHTBULB_ON " Lightmapped");
        ImGui::TextDisabled("Sun and ambient come from the baked texture, through UV2. Point and "
                           "spot lights still light this surface in real time.");
        ImGui::Separator();
    }

    changed |= ImGui::ColorEdit3("Base Color", &material.params.baseColor.x);
    changed |= ImGui::DragFloat("Roughness", &material.params.surface.x, 0.01f, 0.0f, 1.0f);
    changed |= ImGui::DragFloat("Metallic", &material.params.surface.y, 0.01f, 0.0f, 1.0f);
    if (material.params.surface.w <= 0.0f)
        material.params.surface.w = 1.0f;
    changed |= ImGui::DragFloat("Normal Strength", &material.params.surface.w, 0.01f, 0.0f, 4.0f);
    if (material.params.custom1.y <= 0.0f)
        material.params.custom1.y = 0.05f;
    ImGui::BeginDisabled(!(material.flags & MaterialParallax));
    changed |= ImGui::DragFloat("Parallax Scale", &material.params.custom1.y, 0.001f, 0.0f, 0.2f,
                                "%.3f");
    ImGui::EndDisabled();

    // emissive.w, not the flags above, is what makes this glow rather than
    // just tint: lit.frag adds emissive.rgb * emissive.w straight into the
    // HDR colour (lit.frag:927), so a value past 1 is what clears bloom's
    // threshold and actually blooms - a colour alone never will, same as
    // the sponza demo's own Glow panel drives it.
    changed |= ImGui::ColorEdit3("Emissive", &material.params.emissive.x);
    changed |= ImGui::SliderFloat("Glow Strength", &material.params.emissive.w, 0.0f, 12.0f, "%.1f");
    if (material.params.emissive.w > 0.0f)
        ImGui::TextDisabled("Needs Post Process (bloom) on in the Viewport toolbar to bloom");

    changed |= drawFlagCheckbox("Lit", MaterialLit, material);
    ImGui::SameLine();
    changed |= drawFlagCheckbox("Two Sided", MaterialTwoSided, material);
    ImGui::SameLine();
    changed |= drawFlagCheckbox("Alpha Test", MaterialAlphaTest, material);
    if (material.flags & MaterialAlphaTest)
        changed |= ImGui::SliderFloat("Alpha Cutoff", &material.params.surface.z, 0.0f, 1.0f);

    const bool lightmapped = material.textures[SlotLightmap].texture.valid();

    changed |= drawFlagCheckbox("Cast Shadow", MaterialCastShadow, material);
    ImGui::SameLine();
    ImGui::BeginDisabled(lightmapped);
    changed |= drawFlagCheckbox("Receive Shadow", MaterialReceiveShadow, material);
    ImGui::EndDisabled();
    if (lightmapped && ImGui::IsItemHovered())
        ImGui::SetTooltip("Baked into the lightmap. Sampling the cascades as well would light the "
                         "same sun twice, so the shader skips them entirely.");

    // Opt-in, not implied by Metallic: EnvironmentReflection() is only added
    // to the shaded colour #ifdef HAS_REFLECTION (lit.frag) - a probe is
    // captured from one point, so away from it a box projection only
    // approximates, and every surface paying for that unasked is how a flat
    // floor ends up with the sky pasted on it in a hard-edged patch.
    changed |= drawFlagCheckbox("Reflection", MaterialReflection, material);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Metallic alone reflects nothing - Reflection is what samples the "
                          "environment probe");
    ImGui::SameLine();

    // A different technique from Reflection above: this samples the frame's
    // one planar capture (Renderer::executeReflection(), lit.frag's
    // HAS_MIRROR) instead of the environment cube - exact instead of
    // approximate, but only for a flat surface and only one per frame.
    // Hierarchy > Create > Special Nodes > Mirror sets this up already;
    // exposed here too for turning it on/off or tuning strength by hand.
    changed |= drawFlagCheckbox("Mirror", MaterialMirror, material);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("A flat mirror - exact planar reflection instead of Reflection's "
                          "cubemap approximation, but only one MaterialMirror surface reflects "
                          "per frame (the first one found).");
    if (material.flags & MaterialMirror)
    {
        if (ImGui::SliderFloat("Mirror Strength", &material.params.custom0.x, 0.0f, 1.0f))
            changed = true;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("How much of the shaded colour the reflection replaces, weighted "
                              "by a Fresnel curve so a grazing angle reflects more than a "
                              "face-on one.");

        // custom0.y: Renderer::executeReflection() reads this straight off
        // the winning MaterialMirror instance - only one mirror's reflection
        // texture exists per frame (the "first one found" rule), so its
        // Quality is the only one that matters that frame regardless of how
        // many other Mirror surfaces sit unused in the scene.
        static const f32 kQualityScales[] = {0.25f, 0.5f, 1.0f};
        static const char* kQualityNames[] = {"Low", "Medium", "High"};
        int qualityIndex = 1;
        for (int i = 0; i < IM_ARRAYSIZE(kQualityScales); ++i)
            if (glm::abs(material.params.custom0.y - kQualityScales[i]) < 0.001f)
                qualityIndex = i;
        if (ImGui::Combo("Quality", &qualityIndex, kQualityNames, IM_ARRAYSIZE(kQualityNames)))
        {
            material.params.custom0.y = kQualityScales[qualityIndex];
            changed = true;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Resolution of the reflection texture, as a fraction of the "
                              "screen. Only matters when this is the one Mirror the scene "
                              "actually reflects through this frame - give High to whichever "
                              "one the camera is pointed at, Low to a background one.");

        if (ImGui::SliderFloat("Bump Strength", &material.params.custom0.z, 0.0f, 0.05f, "%.3f"))
            changed = true;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Distorts the reflection using this material's own Normal map - "
                              "0 is a perfectly flat mirror, higher looks like slightly uneven "
                              "or rippled glass. Needs a Normal texture assigned below to do "
                              "anything.");

        if (material.params.custom0.w <= 0.0f)
            material.params.custom0.w = 1.0f;
        if (ImGui::SliderFloat("Zoom", &material.params.custom0.w, 0.3f, 3.0f, "%.2f"))
            changed = true;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Widens (>1) or narrows (<1) the reflected camera's own frustum, "
                              "which is already fitted to this Mirror's own rectangle - 1 is "
                              "that exact fit. Above 1 buys a little extra room past the edges "
                              "(useful with Bump); below 1 crops in, more distortion at the "
                              "rim either way.");

        if (ImGui::DragFloat("Far Plane", &material.params.custom1.x, 10.0f, 0.0f, 100000.0f,
                             material.params.custom1.x <= 0.0f ? "Auto" : "%.0f"))
            changed = true;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("How far the reflected camera can see. 0 is automatic - twice the "
                              "distance to the mirror, floored at 5000 - which Renderer:: "
                              "executeReflection() has no better guess than since a scene's "
                              "real far plane is not tracked anywhere past the main camera's "
                              "own projection matrix. Set this by hand if the reflection is "
                              "missing anything that should be far behind the mirror.");
    }

    // No auto-attached probe here any more: a probe is its own placeable
    // object (Hierarchy > Create > Special Nodes > Reflection Probe), the
    // way a Camera or a Light is - drop one where a room needs its own
    // capture, parent it under this object or leave it standalone,
    // Scene::resolveNearestProbe() picks whichever is closest per object.
    if (material.flags & MaterialReflection)
        ImGui::TextDisabled("Reflects the nearest Reflection Probe object, or the scene's "
                            "global one when none is in range.");

    changed |= drawFlagCheckbox("Parallax", MaterialParallax, material);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Offsets the UV by view angle using the Height slot below, before "
                          "Albedo/Normal/Surface sample it - needs a Height texture assigned to "
                          "do anything. Parallax Scale is up with Roughness/Metallic.");
    ImGui::SameLine();
    changed |= drawFlagCheckbox("Metallic-Roughness Map", MaterialMetallicRoughnessMap, material);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reads the Surface slot below as a glTF-style metallic-roughness "
                          "texture (G = roughness, B = metalness) instead of the legacy "
                          "specular map (R channel, roughness = 1 - specular) - a glTF import's "
                          "own metallicRoughnessTexture wants this on.");

    ImGui::Spacing();
    ImGui::TextUnformatted("UV Transform");
    changed |= ImGui::DragFloat2("Tiling", &material.params.uvTransform.x, 0.01f, -1000.0f,
                                 1000.0f, "%.3f");
    changed |= ImGui::DragFloat2("Offset", &material.params.uvTransform.z, 0.01f, -1000.0f,
                                 1000.0f, "%.3f");

    ImGui::Spacing();
    ImGui::TextDisabled("Drag a file from Assets onto a slot");
    changed |= drawTextureSlot(app(), "Albedo", SlotAlbedo, material);
    ImGui::Spacing();
    changed |= drawTextureSlot(app(), "Normal", SlotNormal, material);
    ImGui::Spacing();
    changed |= drawTextureSlot(app(), "Surface", SlotSurface, material);
    ImGui::Spacing();
    changed |= drawTextureSlot(app(), "Emissive", SlotEmissive, material);
    ImGui::Spacing();
    changed |= drawTextureSlot(app(), "Height", SlotHeight, material);
    ImGui::Spacing();
    changed |= drawTextureSlot(app(), "Lightmap", SlotLightmap, material);

    if (ImGui::Button("Clear Textures", ImVec2(-FLT_MIN, 0.0f)))
    {
        for (u32 slot = 0; slot < MaterialSlotCount; ++slot)
            material.textures[slot] = MaterialTexture();
        changed = true;
    }
    // Worth saying plainly, because it is the one thing that makes a metal
    // read as painted stone rather than as metal: F0 is mix(0.04, albedo,
    // metallic) per pixel (lit.frag), so at metallic 1 the albedo texture
    // tints the mirror itself. A flat Base Color is what gives a clean one -
    // the sponza mirror ball carries no albedo map for exactly this reason.
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Empties every texture slot. At high Metallic the albedo texture "
                          "tints the reflection itself, so a flat Base Color is what makes a "
                          "clean mirror.");

    if (changed)
        material.paramsDirty = true;
    return changed;
}

// One CollapsingHeader per material slot - a mesh with several submeshes
// (Sponza's ~25 materials, say) used to only ever expose slot 0 here, the
// rest silently unreachable from the Inspector no matter how many the mesh
// actually had. mesh->materials.size() is the slot count MeshRenderer::
// setMaterialOverride() itself indexes by; a primitive with no materials
// array of its own (built straight from HierarchyPanel, never named any)
// still gets exactly the one slot it always had.
bool InspectorPanel::makeSubmeshMaterialUnique(MeshHandle handle, u32 slot, s32 submeshIndex)
{
    MeshData* data = app().importedMeshData(handle);
    if (!data || submeshIndex < 0 || static_cast<usize>(submeshIndex) >= data->submeshes.size())
        return false;
    if (data->submeshes[static_cast<usize>(submeshIndex)].materialSlot != slot)
        return false; // stale pick - the mesh changed since it was recorded
    if (slot >= data->materials.size())
        return false;

    const u32 newSlot = static_cast<u32>(data->materials.size());
    data->materials.push_back(data->materials[slot]);
    // Every per-material file array grows by one in lockstep, whatever its
    // own length already was - AssetMesh.cpp only ever reads these guarded
    // by "index < array.size()", so a short array already means "no texture
    // past here" for every slot beyond it, old or new alike. Copying the
    // source slot's own entry (or empty, past its length) keeps the new slot
    // textured exactly the way the one it split from was.
    const auto duplicateAux = [slot](std::vector<std::string>& files)
    {
        files.push_back(slot < files.size() ? files[slot] : std::string());
    };
    duplicateAux(data->materialTextureFiles);
    duplicateAux(data->materialNormalFiles);
    duplicateAux(data->materialSurfaceFiles);
    duplicateAux(data->materialEmissiveFiles);
    duplicateAux(data->materialHeightFiles);

    data->submeshes[static_cast<usize>(submeshIndex)].materialSlot = newSlot;
    return app().applyMeshEdit(handle);
}

void InspectorPanel::drawMeshMaterial(MeshRenderer& renderer)
{
    const Mesh* mesh = AssetManager::getSingleton().getMesh(renderer.mesh());
    const usize submeshCount = renderer.submeshCount();

    // The one thing that ties the Viewport and this list together, both
    // ways - see EditorApplication::pickedSubmesh(). Read up here, before
    // the outer header below, so a pick landing on this renderer can force
    // that header open instead of leaving the picked entry folded away.
    EditorApplication::PickedSubmesh& picked = app().pickedSubmesh();
    const u64 ownerId = renderer.owner() ? renderer.owner()->id() : 0;
    const bool pickedHere = picked.object == ownerId && picked.justPicked;

    ImGui::Separator();
    // A mesh with many submeshes (Bistro-scale imports run into the
    // hundreds) used to spell out every one of them right here, pushing
    // whatever came after MeshRenderer in the component list far enough
    // down that reaching it meant scrolling past all of them first. Folded
    // shut by default now - one line to open, not a wall to scroll through.
    bool showSubmeshes = submeshCount <= 1;
    if (submeshCount > 1)
    {
        if (pickedHere)
            ImGui::SetNextItemOpen(true);
        showSubmeshes =
            ImGui::CollapsingHeader(("Submeshes (" + std::to_string(submeshCount) + ")").c_str());
    }
    else
        ImGui::TextUnformatted("Material");

    // The Viewport's Shift-click batch (Pick Surface tool) picks a SET of
    // submeshes on this object - shown here regardless of whether
    // "Submeshes (N)" above is folded, since that set is otherwise invisible
    // until this row tells the user it exists at all.
    EditorApplication::SubmeshSelection& submeshSelection = app().submeshSelection();
    const bool haveSubmeshSelection =
        submeshSelection.object == ownerId && !submeshSelection.indices.empty();
    if (haveSubmeshSelection)
    {
        std::string indexList;
        for (u32 index : submeshSelection.indices)
            indexList += (indexList.empty() ? "" : ", ") + std::to_string(index);
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.35f, 1.0f), "%zu submesh(es) selected: %s",
                           submeshSelection.indices.size(), indexList.c_str());
        ImGui::SameLine();
        ImGui::BeginDisabled(app().importedMeshData(renderer.mesh()) == nullptr);
        if (ImGui::Button("Delete Selected Submeshes"))
            app().deleteSubmeshSelection();
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered() && app().importedMeshData(renderer.mesh()) == nullptr)
            ImGui::SetTooltip("This mesh was not imported this session, so there is no CPU-side "
                              "copy to edit.");
        ImGui::SameLine();
        if (ImGui::Button("Clear Selection"))
            submeshSelection.indices.clear();
    }

    // Once for the whole renderer, not per submesh - keeping something out
    // of a reflection is a property of the object, not of any one material
    // on it.
    bool visibleInReflections = renderer.visibleInReflections();
    if (ImGui::Checkbox("Visible in Reflections", &visibleInReflections))
    {
        renderer.setVisibleInReflections(visibleInReflections);
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Off keeps this object out of every reflection capture - a Reflection "
                          "Probe's cubemap AND a Mirror's planar one, both. The one way to stop "
                          "an object seeing itself in its own reflection, or to keep something "
                          "out of a mirror on purpose. Its shadow is unaffected.");

    // Saving a .material sidecar is a Mesh Tools action (see "Save
    // Materials..." there, next to Save As... for the mesh itself) - the
    // Inspector shows/edits live state, it does not accumulate export
    // buttons for every kind of file that state could turn into.
    if (renderer.materialOverrideCount() > 0)
    {
        if (ImGui::Button("Reset Materials to File"))
        {
            // The only way back from a per-slot override once made - there is
            // no per-slot clear, and there needs to be no other way: an
            // override saved into an object's own scene file (see
            // SceneSerializer) never updates itself when the .material
            // sidecar it was copied from is edited afterward. A mesh whose
            // sidecar changed after this object's last save is exactly what
            // leaves overrides stale like this.
            renderer.clearMaterialOverrides();
            app().markDirty();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Drops every per-slot override on this object and goes back to reading "
                "straight from the mesh's own materials (its .material sidecar, if it "
                "has one) - the fix for an override that was saved before that file was "
                "last edited and never picked up the change.");
    }

    if (submeshCount == 0)
    {
        // No mesh, or an importer that never split into submeshes - one
        // plain material, same as it always was.
        Material material =
            renderer.materialOverrideCount() > 0 ? renderer.materialOverrides()[0]
                                                 : defaultPrimitiveMaterial();
        if (drawMaterialFields(material))
        {
            renderer.setMaterialOverride(0, material);
            app().markDirty();
        }
        return;
    }

    const bool haveImportedData = app().importedMeshData(renderer.mesh()) != nullptr;
    // Acted on after the loop: removing a submesh rebuilds the GPU mesh and
    // shifts every index after it, which a loop walking those indices cannot
    // survive.
    s32 submeshToDelete = -1;

    for (usize i = 0; showSubmeshes && i < submeshCount; ++i)
    {
        // mesh may be a dangling pointer past this point on an iteration
        // that ends up splitting a shared slot below (applyMeshEdit()
        // rebuilds the GPU Mesh) - slot is a plain u32, copied out before
        // that can happen, and everything after only reads the mesh pointer
        // again once it has been re-fetched.
        const u32 slot = mesh->submeshes[i].materialSlot;

        Material material;
        if (slot < renderer.materialOverrideCount())
            material = renderer.materialOverrides()[slot];
        else if (slot < mesh->materials.size())
            material = mesh->materials[slot];
        else
            material = defaultPrimitiveMaterial();

        const bool multiSelected = submeshSelection.object == ownerId &&
                             std::find(submeshSelection.indices.begin(),
                                       submeshSelection.indices.end(),
                                       static_cast<u32>(i)) != submeshSelection.indices.end();
        std::string label = (multiSelected ? "> Submesh " : "Submesh ") + std::to_string(i);
        if (slot < mesh->materials.size() && !mesh->materials[slot].name.empty())
            label += " (" + mesh->materials[slot].name + ")";

        // How many submeshes on THIS mesh still read this slot - a slot used
        // only once is already this submesh's own, nothing to split.
        u32 sharedBy = 0;
        for (const SubMesh& submesh : mesh->submeshes)
            if (submesh.materialSlot == slot)
                ++sharedBy;

        const bool isPicked = picked.object == ownerId && picked.index == static_cast<s32>(i);
        if (isPicked && picked.justPicked)
            ImGui::SetNextItemOpen(true);

        ImGui::PushID(static_cast<int>(i));
        // A single-submesh mesh (the overwhelming majority - most
        // primitives, most single-material imports) skips the header
        // entirely: the fields just sit right under "Material" like they
        // always did, nothing new to click through for the common case.
        // AllowOverlap: without it a CollapsingHeader claims its entire row
        // for its own click, and the focus icon placed on the same line via
        // SameLine() below never receives one of its own - every click just
        // toggled the header open/closed instead of doing anything else.
        const bool open = submeshCount == 1 ||
                          ImGui::CollapsingHeader(label.c_str(), ImGuiTreeNodeFlags_AllowOverlap);
        if (ImGui::IsItemClicked())
        {
            // Selecting an entry here highlights it back in the Viewport
            // (the cyan box) - the other half of the Viewport-click-opens-
            // this-entry direction below.
            picked.index = static_cast<s32>(i);
            picked.object = ownerId;
        }
        if (submeshCount > 1)
        {
            // A regular Button, not SmallButton - SmallButton zeroes
            // FramePadding for a tight inline-with-text look, which clips
            // the top of a glyph this tall (see drawTextureSlot's own note
            // on the same thing) - the crosshair rendered, just invisibly.
            const f32 iconSize = ImGui::GetFrameHeight();
            const f32 spacing = ImGui::GetStyle().ItemSpacing.x;
            ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - iconSize * 3.0f - spacing * 2.0f);
            bool submeshVisible = renderer.submeshVisible(static_cast<u32>(i));
            if (ImGui::Checkbox("##submeshVisible", &submeshVisible))
            {
                renderer.setSubmeshVisible(static_cast<u32>(i), submeshVisible);
                app().markDirty();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Visible - off removes this submesh from rendering and "
                                  "shadows");
            ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - iconSize * 2.0f - spacing);
            if (ImGui::Button(ICON_MDI_CROSSHAIRS_GPS, ImVec2(iconSize, iconSize)))
            {
                picked.index = static_cast<s32>(i);
                picked.object = ownerId;
                app().requestFocusObject(ownerId);
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Move the Viewport camera to frame this submesh");

            ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - iconSize);
            ImGui::BeginDisabled(!haveImportedData);
            if (ImGui::Button(ICON_MDI_DELETE, ImVec2(iconSize, iconSize)))
                submeshToDelete = static_cast<s32>(i);
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(haveImportedData
                                      ? "Delete this submesh from the mesh in memory. The file on "
                                        "disk is untouched until Mesh Tools saves it. To hide it "
                                        "reversibly, use the checkbox instead."
                                      : "This mesh was not imported this session, so there is no "
                                        "CPU-side copy to edit. Hide it with the checkbox "
                                        "instead.");
        }
        if (isPicked && picked.justPicked)
        {
            ImGui::SetScrollHereY(0.1f);
            // Consumed - a second click on the very same submesh (a common
            // "wait, which one was that again" re-check) still re-opens and
            // re-scrolls to it instead of silently doing nothing the second
            // time.
            picked.justPicked = false;
        }

        if (open && sharedBy > 1)
            ImGui::TextDisabled(haveImportedData
                                    ? "  shares its material with %u other submeshes - editing "
                                      "it here splits this one off, the rest are untouched"
                                    : "  shares its material with %u other submeshes - this mesh "
                                      "was not imported this session, so editing here changes "
                                      "all of them (reimport via Assets to split instead)",
                                sharedBy - 1);

        if (open && drawMaterialFields(material))
        {
            u32 targetSlot = slot;
            if (sharedBy > 1 && haveImportedData &&
                makeSubmeshMaterialUnique(renderer.mesh(), slot, static_cast<s32>(i)))
            {
                mesh = AssetManager::getSingleton().getMesh(renderer.mesh());
                targetSlot = mesh && i < mesh->submeshes.size() ? mesh->submeshes[i].materialSlot
                                                                : slot;
            }
            renderer.setMaterialOverride(targetSlot, material);
            app().markDirty();
        }
        ImGui::PopID();
    }

    if (submeshToDelete >= 0)
    {
        if (MeshData* data = app().importedMeshData(renderer.mesh()))
        {
            app().recordMeshUndo(renderer.mesh());
            if (AssetManager::getSingleton().removeSubmesh(*data,
                                                           static_cast<u32>(submeshToDelete)))
            {
                // Hidden submeshes are stored as indices into the submesh
                // table, so every one past the deleted entry now names the
                // wrong piece. Cleared rather than remapped: the list is
                // small and getting this subtly wrong hides the wrong
                // geometry with no way to tell.
                renderer.setHiddenSubmeshes({});
                app().applyMeshEdit(renderer.mesh());
                app().markDirty();
                Log::info("InspectorPanel: deleted submesh #%d, %zu left", submeshToDelete,
                          data->submeshes.size());
            }
        }
    }
}

// A Reflection Probe object (Hierarchy > Create > Special Nodes): its own
// capture point, placed wherever it was dropped rather than tied to one
// mesh - a room-sized probe two mirrors in it can both use, each closer to
// it than to the scene's single global one (Scene::resolveNearestProbe()
// picks whichever wins that comparison).
void InspectorPanel::drawReflectionProbe(ReflectionProbe& probeComponent)
{
    EnvironmentProbe& env = probeComponent.probe();

    ImGui::Separator();

    static const int kResolutions[] = {32, 64, 128, 256, 512};
    static const char* kResolutionNames[] = {"32", "64", "128", "256", "512"};
    int resolutionIndex = 2;
    for (int i = 0; i < IM_ARRAYSIZE(kResolutions); ++i)
        if (kResolutions[i] == static_cast<int>(env.resolution()))
            resolutionIndex = i;
    if (ImGui::Combo("Resolution", &resolutionIndex, kResolutionNames,
                     IM_ARRAYSIZE(kResolutionNames)))
    {
        probeComponent.create(static_cast<u32>(kResolutions[resolutionIndex]));
        env.invalidate();
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Per-face size of the captured cubemap. Something small and far away "
                          "does not need the same resolution as the mirror the camera is "
                          "pointed at - 32 for a background prop, 512 for a hero surface.");

    // The box the reflection is treated as living inside - lit.frag re-aims
    // every reflection ray onto its wall, so this is the single control that
    // decides whether a neighbour comes back at its real size or magnified.
    if (ImGui::DragFloat("Influence Radius", &env.influenceRadius, 0.1f, 0.01f, 10000.0f))
        app().markDirty();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How near an object has to be for this probe to be the one it uses.");

    if (ImGui::DragFloat3("Box Extents", &env.extents.x, 0.1f, 0.0f, 10000.0f))
    {
        env.invalidate();
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Zero is a plain mirror of the surroundings. Non-zero projects every "
                          "reflection onto a box that size - only right when a room really is "
                          "there, or nearby things land on the wrong faces.");

    if (ImGui::DragFloat("Probe Intensity", &env.intensity, 0.01f, 0.0f, 4.0f))
        app().markDirty();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Multiplier on the reflection. Drop it to zero to check whether a "
                          "colour is coming from this probe or from somewhere else.");

    int content = static_cast<int>(env.content);
    const char* kContentNames[] = {"Face Colors (debug)", "Sky only", "Sky and World"};
    if (ImGui::Combo("Content", &content, kContentNames, IM_ARRAYSIZE(kContentNames)))
    {
        env.content = static_cast<EnvironmentProbe::Content>(content);
        env.invalidate();
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Face Colors fills each face with a flat colour keyed to its axis - "
                          "the way to tell a broken cubemap orientation from a broken capture.");

    int refresh = static_cast<int>(env.refresh);
    const char* kRefreshNames[] = {"Manual", "Automatic", "Timed"};
    if (ImGui::Combo("Refresh", &refresh, kRefreshNames, IM_ARRAYSIZE(kRefreshNames)))
    {
        env.refresh = static_cast<EnvironmentProbe::Refresh>(refresh);
        env.invalidate();
        app().markDirty();
    }
    if (env.refresh == EnvironmentProbe::Refresh::Timed)
    {
        if (ImGui::DragFloat("Interval", &env.interval, 0.05f, 0.05f, 10.0f, "%.2f s"))
            app().markDirty();
    }

    if (ImGui::Button("Recapture", ImVec2(-FLT_MIN, 0.0f)))
    {
        env.requestCapture();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Six scene renders - not per frame. Anything that moved since the "
                          "last capture is only in the reflection after this.");
    if (env.captureCount() == 0)
        ImGui::TextDisabled("Not captured yet");
    else
        ImGui::TextDisabled("Last capture: %.2f ms (capture #%llu)",
                            static_cast<double>(env.lastCaptureCostMilliseconds()),
                            static_cast<unsigned long long>(env.captureCount()));
}

} // namespace Radion

namespace Radion
{

void InspectorPanel::drawWaypointsComponent(GameObject& object, Waypoints& waypoints)
{
    ImGui::Indent(14.0f);

    ImGui::TextWrapped("Navigation points authored by hand. Positions are local to this object, "
                      "so moving it carries the whole graph.");
    ImGui::Text("Points: %zu", waypoints.pointCount());

    // Dropped at the object's own origin - the gizmo in the viewport is what
    // places it, not a typed coordinate.
    if (ImGui::Button("Add point"))
    {
        app().recordUndo();
        // Dropped at the 3D cursor, in the object's own space - the same
        // place every other "create here" in the editor uses.
        const glm::vec3 local = glm::vec3(glm::inverse(object.globalTransform()) *
                                         glm::vec4(app().cursor3D(), 1.0f));
        app().setSelectedWaypoint(static_cast<s32>(waypoints.addPoint(local)));
        app().markDirty();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear all"))
    {
        app().recordUndo();
        waypoints.clear();
        app().markDirty();
    }

    static f32 autoLinkRadius = 20.0f;
    ImGui::DragFloat("Auto-link radius", &autoLinkRadius, 0.1f, 0.1f, 500.0f, "%.1f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Links every pair of points closer than this, replacing the current "
                         "links. Hand-editing after it is expected.");
    ImGui::SameLine();
    if (ImGui::Button("Auto-link"))
    {
        app().recordUndo();
        waypoints.autoLink(autoLinkRadius);
        app().markDirty();
    }
    if (ImGui::Button("Clear links"))
    {
        app().recordUndo();
        waypoints.clearLinks();
        app().markDirty();
    }

    s32 removeIndex = -1;
    for (u32 i = 0; i < static_cast<u32>(waypoints.pointCount()); ++i)
    {
        ImGui::PushID(static_cast<int>(i));
        WaypointNode& node = waypoints.point(i);
        char label[64];
        const bool picked = app().selectedWaypoint() == static_cast<s32>(i);
        std::snprintf(label, sizeof(label), "%sPoint %u (%zu links)", picked ? "> " : "  ", i,
                     node.links.size());
        if (ImGui::TreeNode(label))
        {
            // Picking a point is what hands it the viewport's move gizmo.
            if (ImGui::RadioButton("Move with the gizmo", picked))
                app().setSelectedWaypoint(picked ? -1 : static_cast<s32>(i));
            glm::vec3 position = node.position;
            if (ImGui::DragFloat3("Position", &position.x, 0.05f))
            {
                waypoints.setPointPosition(i, position);
                app().markDirty();
            }
            f32 radius = node.radius;
            if (ImGui::DragFloat("Radius", &radius, 0.05f, 0.0f, 100.0f, "%.2f"))
            {
                waypoints.setPointRadius(i, radius);
                app().markDirty();
            }
            if (ImGui::Button("Remove"))
                removeIndex = static_cast<s32>(i);
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
    if (removeIndex >= 0)
    {
        app().recordUndo();
        waypoints.removePoint(static_cast<u32>(removeIndex));
        app().markDirty();
    }

    ImGui::Unindent(14.0f);
}

void InspectorPanel::drawNavMeshSurfaceComponent(GameObject& object, NavMeshSurface& surface)
{
    ImGui::Indent(14.0f);

    ImGui::TextWrapped("Bakes this object's mesh into the walkable surface AI navigates. The "
                      "settings describe the agent the surface is built FOR.");

    if (surface.built())
        ImGui::Text("Baked: %zu surface triangles, %.1f ms",
                   surface.navMesh().debugTriangles().size() / 3,
                   static_cast<f64>(surface.lastBuildMilliseconds()));
    else
        ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.4f, 1.0f), "Not baked yet.");

    AI::NavMeshConfig& config = surface.config();
    bool dirty = false;
    dirty |= ImGui::DragFloat("Cell size", &config.cellSize, 0.01f, 0.05f, 2.0f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Voxel width. Smaller hugs the geometry closer and costs more to bake.");
    dirty |= ImGui::DragFloat("Cell height", &config.cellHeight, 0.01f, 0.05f, 2.0f, "%.2f");
    dirty |= ImGui::DragFloat("Agent height", &config.agentHeight, 0.05f, 0.1f, 10.0f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Headroom the agent needs. Anything lower is not walkable.");
    dirty |= ImGui::DragFloat("Agent radius", &config.agentRadius, 0.05f, 0.0f, 5.0f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The surface is eroded by this, so the agent never clips a wall it "
                         "walks beside.");
    dirty |= ImGui::DragFloat("Max climb", &config.agentMaxClimb, 0.05f, 0.01f, 5.0f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Step height. Two floors only connect when the steps between them are "
                         "under this - otherwise the upper one is an unreachable island.");
    dirty |= ImGui::DragFloat("Max slope", &config.agentMaxSlope, 0.5f, 1.0f, 89.0f, "%.1f deg");
    if (dirty)
        app().markDirty();

    ImGui::Separator();
    glm::vec3 groundSeed = surface.groundSeed();
    ImGui::TextWrapped("Ground seed: only what a walker can actually reach from here survives - "
                      "a flat roof with no stairs down to it passes the same slope test the "
                      "ground does, so without this it stays in the surface as an island.");
    if (ImGui::DragFloat3("##GroundSeed", &groundSeed.x, 0.1f))
    {
        surface.setGroundSeed(groundSeed);
        app().markDirty();
    }
    ImGui::SameLine();
    if (ImGui::Button("Use cursor"))
    {
        surface.setGroundSeed(app().cursor3D());
        app().markDirty();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Sets the seed to the 3D cursor's current position.");

    if (ImGui::Button("Bake navmesh"))
        surface.build();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Explicit on purpose: baking a large level is slow, and doing it "
                         "silently on load would be paid for at the worst moment.");

    // The baked surface can be written out and read back instead of rebuilt:
    // the scene only ever stores the recipe, so without a file beside it
    // every load pays the whole Recast pipeline again.
    ImGui::SameLine();
    ImGui::BeginDisabled(!surface.built());
    if (ImGui::Button("Save Baked..."))
    {
        mNavMeshSaveTarget = object.id();
        mNavMeshDialog.Open(ImGuiFileDialog::Mode::SaveFile,
                            app().settings().lastSaveDirectory.empty()
                                ? app().assetBrowserRoot()
                                : app().settings().lastSaveDirectory,
                            object.name() + ".rnav");
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Writes the built surface itself - the Detour tile's own bytes - so "
                         "loading it back skips the whole voxelise/contour/poly-mesh pipeline. "
                         "Goes stale if the level mesh changes; bake and save again then.");
    ImGui::SameLine();
    if (ImGui::Button("Load Baked..."))
    {
        mNavMeshSaveTarget = 0;
        mNavMeshLoadTarget = object.id();
        mNavMeshDialog.Open(ImGuiFileDialog::Mode::OpenFile,
                            app().settings().lastOpenDirectory.empty()
                                ? app().assetBrowserRoot()
                                : app().settings().lastOpenDirectory,
                            std::string());
    }
    if (!surface.navDataFile().empty())
        ImGui::TextDisabled("baked file: %s",
                            FileSystem::fileName(surface.navDataFile()).c_str());

    if ((mNavMeshSaveTarget == object.id() || mNavMeshLoadTarget == object.id()) &&
        mNavMeshDialog.Render(app().assetBrowserRoot(), app().assetBrowserRoot(),
                              app().assetBrowserRoot()))
    {
        const ImGuiFileDialog::Result result = mNavMeshDialog.ConsumeResult();
        const bool saving = mNavMeshSaveTarget == object.id();
        mNavMeshSaveTarget = 0;
        mNavMeshLoadTarget = 0;
        if (result.accepted)
        {
            const std::string path = result.path.string();
            if (saving)
            {
                app().settings().lastSaveDirectory = result.path.parent_path().string();
                if (surface.saveNavData(path))
                {
                    Log::info("InspectorPanel: saved navmesh '%s'", path.c_str());
                    app().toasts().success("Saved " + FileSystem::fileName(path));
                    app().markDirty();
                }
                else
                    app().toasts().error("Could not write " + FileSystem::fileName(path));
            }
            else
            {
                app().settings().lastOpenDirectory = result.path.parent_path().string();
                if (surface.loadNavData(path))
                {
                    Log::info("InspectorPanel: loaded navmesh '%s' (%zu surface triangles)",
                              path.c_str(), surface.navMesh().debugTriangles().size() / 3);
                    app().toasts().success("Loaded " + FileSystem::fileName(path));
                    app().markDirty();
                }
                else
                    app().toasts().error("Could not read " + FileSystem::fileName(path));
            }
        }
    }

    ImGui::Unindent(14.0f);
}

} // namespace Radion
