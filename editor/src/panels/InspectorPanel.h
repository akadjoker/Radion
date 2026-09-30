#ifndef RADION_INSPECTOR_PANEL_H
#define RADION_INSPECTOR_PANEL_H

#include "EditorPanel.h"
#include "ImGuiFileDialog.h"
#include "Mesh.h" // MeshHandle
#include "MeshPreview.h"

#include "Math.h"
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
        Math::vec3 dimensions = Math::vec3(1.0f);
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
    // Gives one submesh its own material slot copied from its shared one, so edits stop reaching the others (see drawMeshMaterial()).
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
    void drawTiledTerrainComponent(class TiledTerrain& terrain);
    void drawRoadComponent(GameObject& object, class Road& road);
    // "Add Component": Animator only. Converts a picked .fbx to .rskel/.ranim via AssetManager::importSkeleton()/importAnimation(); Animator needs the Radion format.
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
    // One dialog for both directions; which of the two target ids is set says save or load.
    ImGuiFileDialog mNavMeshDialog;
    u64 mNavMeshSaveTarget = 0;
    u64 mNavMeshLoadTarget = 0;
    bool mAnimatorSkeletonDialogPending = false;
    bool mAnimatorClipDialogPending = false;
    char mBoneFilter[64] = ""; // the Select Bone popup's search box
    char mAnimationEventName[64] = "";
    f32 mAnimationEventTime = 0.0f;
    // Edit buffer for the path field, refreshed when the selected ZenBehaviour changes; otherwise keystrokes fight loadFile()'s value.
    u64 mZenBehaviourObjectId = 0;
    char mZenScriptPathBuffer[256] = "";
    ImGuiFileDialog mZenScriptDialog;
    u64 mZenScriptDialogTarget = 0;
    // Set on "Create" failure and shown inline (createAnimator() only logged).
    std::string mNewAnimatorError;
};

} // namespace Radion

#endif // RADION_INSPECTOR_PANEL_H
