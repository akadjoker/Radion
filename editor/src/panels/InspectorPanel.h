#ifndef RADION_INSPECTOR_PANEL_H
#define RADION_INSPECTOR_PANEL_H

#include "EditorPanel.h"
#include "ImGuiFileDialog.h"
#include "Mesh.h" // MeshHandle
#include "MeshPreview.h"

#include <glm/vec3.hpp>
#include <string>
#include <vector>

namespace Radion::Physics
{
class RigidBody;
class Joint;
}

namespace Radion
{
class GameObject;
class Animator;

 
class InspectorPanel final : public EditorPanel
{
public:
    explicit InspectorPanel(EditorApplication& app);
    ~InspectorPanel() override;

    void onImGui() override;

private:
    enum class PrimitiveKind : u8
    {
        Cube,
        Sphere,
        Plane,
        Cylinder,
        Cone,
        Capsule,
        Torus
    };

    struct PrimitiveSettings
    {
        glm::vec3 dimensions = glm::vec3(1.0f);
        f32 uvTiles = 1.0f;
        int segmentsA = 0;
        int segmentsB = 0;
    };

    void drawHeader(GameObject& object);
    void drawTransform(GameObject& object);
    void drawComponentList(GameObject& object);
    void drawMeshRenderer(GameObject& object, class MeshRenderer& renderer);
    void drawMeshMaterial(class MeshRenderer& renderer);
    bool drawMaterialFields(struct Material& material);
    // Gives one submesh its own material slot, copied from whatever it
    // shares today, so editing it stops reaching every other submesh still
    // on the old slot - see the slot-sharing note on drawMeshMaterial().
    bool makeSubmeshMaterialUnique(MeshHandle handle, u32 slot, s32 submeshIndex);
    void drawReflectionProbe(class ReflectionProbe& probeComponent);
    void drawCameraComponent(class Camera& camera);
    void drawLightComponent(class Light& light);
    void drawText3DComponent(class Text3D& text);
    void drawBillboardComponent(class Billboard& billboard);
    void drawAudioPlayerComponent(class AudioPlayer& player);
    void drawBoneAttachmentComponent(class BoneAttachment& attachment);
    void drawOrbitComponent(GameObject& object, class Orbit& orbit);
    void drawMayaComponent(GameObject& object, class Maya& maya);
    void drawThirdPersonComponent(GameObject& object, class ThirdPerson& camera);
    void drawSelfDestroyComponent(GameObject& object, class SelfDestroy& selfDestroy);
    void drawColliderComponent(GameObject& object, class Collider& collider);
    void drawRigidBodyComponent(GameObject& object, Physics::RigidBody& body);
    void drawJointComponent(GameObject& object, Physics::Joint& joint);
    void drawWaypointsComponent(GameObject& object, class Waypoints& waypoints);
    void drawNavMeshSurfaceComponent(GameObject& object, class NavMeshSurface& surface);
    void drawZenBehaviourComponent(GameObject& object, class ZenBehaviour& behaviour);
    void drawZenBehaviourProperties(class ZenBehaviour& behaviour);
    void resetPrimitiveSettings(PrimitiveKind kind);

    void drawAnimatorComponent(Animator& animator);
    void drawParticleEffectComponent(class ParticleEffect& effect);
    void drawParticleEmitterComponent(class ParticleEmitter& emitter);
    void drawOceanComponent(class Ocean& ocean);
    void drawVoxelWorldComponent(class VoxelWorldComponent& voxelWorld);
    void drawGrassComponent(class Grass& grass);
    void drawHairComponent(class Hair& hair);
    void drawForestComponent(class Forest& forest);
    void drawTerrainComponent(class Terrain& terrain);
    void drawRoadComponent(GameObject& object, class Road& road);
    // "Add Component" - today just Animator, the one this editor otherwise
    // has no way at all to attach to an already-existing object (import only
    // ever adds MeshRenderer). Converts any .fbx picked here to .rskel/
    // .ranim next to itself through AssetManager::importSkeleton()/
    // importAnimation() - Animator::bind()/SceneSerializer both require the
    // Radion format, never a raw FBX.
    void drawAddComponentSection(GameObject& object);

    PrimitiveSettings mPrimitiveSettings;
    PrimitiveKind mPrimitiveKind = PrimitiveKind::Cube;
    u64 mPrimitiveObjectId = 0;
    u64 mForestObjectId = 0;
    int mForestSpeciesIndex = 0;
    MeshPreview mTreePreview;
    f32 mTreePreviewYaw = 0.0f;

    bool mOpenAddAnimatorPopup = false;
    std::string mNewAnimatorSkeletonFile;
    std::vector<std::string> mNewAnimatorClipFiles;
    ImGuiFileDialog mAnimatorSkeletonDialog;
    ImGuiFileDialog mAnimatorClipDialog;
    // One dialog for both directions - only one of the two target ids is
    // ever set, which is what says whether the result is a save or a load.
    ImGuiFileDialog mNavMeshDialog;
    u64 mNavMeshSaveTarget = 0;
    u64 mNavMeshLoadTarget = 0;
    bool mAnimatorSkeletonDialogPending = false;
    bool mAnimatorClipDialogPending = false;
    char mBoneFilter[64] = ""; // the Select Bone popup's search box
    // The path field's own edit buffer, refreshed from scriptPath() whenever
    // the selected object's ZenBehaviour changes - otherwise every keystroke
    // fights the field back to whatever loadFile() last reported.
    u64 mZenBehaviourObjectId = 0;
    char mZenScriptPathBuffer[256] = "";
    ImGuiFileDialog mZenScriptDialog;
    u64 mZenScriptDialogTarget = 0;
    // Set by "Create" on failure and shown inline - createAnimator() only
    // ever logged before, so a failure looked exactly like nothing having
    // happened at all.
    std::string mNewAnimatorError;
};

} // namespace Radion

#endif // RADION_INSPECTOR_PANEL_H
