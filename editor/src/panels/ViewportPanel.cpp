#include "PCH.h"

#include "panels/ViewportPanel.h"

#include "Animation.h"
#include "AssetManager.h"
#include "Camera.h"
#include "Collider.h"
#include "DebugDraw3D.h"
#include "EditorApplication.h"
#include "Engine.h"
#include "Forest.h"
#include "GameObject.h"
#include "Grass.h"
#include "Light.h"
#include "Log.h"
#include "MeshRenderer.h"
#include "ObstacleComponent.h"
#include "PostProcess.h"
#include "NavMeshSurface.h"
#include "Road.h"
#include "Waypoints.h"
#include "Scene.h"
#include "Terrain.h"

#include "collision/CollisionShape.h"
#include "dynamics/RigidBody.h"

#include <IconsMaterialDesignIcons.h>
#include <ImGuizmo.h>
#include <limits>
#include "Math.h"
#include <imgui.h>

namespace Radion
{

ViewportPanel::ViewportPanel(EditorApplication& app) : EditorPanel("Viewport", app)
{
    // Restore the free-fly camera pose from the last session (EditorSettings::cameraPosition).
    const EditorSettings& settings = app.settings();
    mCameraPosition = settings.cameraPosition;
    mOrbitTarget = settings.cameraOrbitTarget;
    mOrbitDistance = settings.cameraOrbitDistance;
    mOrbitYaw = settings.cameraOrbitYaw;
    mOrbitPitch = settings.cameraOrbitPitch;
    mPerspective = settings.cameraPerspective;
    mGrid = settings.viewportGrid;
    mSnap = settings.viewportSnap;
    mTool = Math::clamp(settings.viewportTool, 0, 5);
    mPointerNavigation = Math::clamp(settings.viewportNavigationTool, 0, 2);
}

void ViewportPanel::focusOnObject(const GameObject& object, s32 submeshIndex)
{
    Math::vec3 center = object.globalPosition();
    f32 radius = 1.0f;
    if (MeshRenderer* renderer = object.getComponent<MeshRenderer>())
    {
        if (const Mesh* mesh = Assets().getMesh(renderer->mesh()))
        {
            const AABB localBounds =
                (submeshIndex >= 0 && static_cast<usize>(submeshIndex) < mesh->submeshes.size())
                    ? mesh->submeshes[static_cast<usize>(submeshIndex)].bounds
                    : mesh->bounds;
            const AABB worldBounds = transformAABB(localBounds, object.globalTransform());
            if (!worldBounds.empty())
            {
                center = worldBounds.center();
                radius = Math::max(worldBounds.radius(), 0.05f);
            }
        }
    }

    Camera* camera = app().scene().activeCamera();
    const f32 fieldOfView = camera ? camera->fieldOfView() : 60.0f;
    const f32 fitDistance = radius / Math::sin(Math::radians(fieldOfView * 0.5f)) * 1.3f;
    mOrbitTarget = center;
    mOrbitDistance = Math::max(fitDistance, 0.5f);
}

void ViewportPanel::updateNavigation()
{
    ImGuiIO& io = ImGui::GetIO();
    const EditorSettings& nav = app().settings();
    const bool hovered = ImGui::IsWindowHovered();
    if (hovered && mPerspective && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && io.KeyAlt)
        mOrbiting = true;
    if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
        mPanning = true;
    if (hovered && mPerspective && mPointerNavigation == 0 &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Right))
        mLooking = true;
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
        mOrbiting = false;
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Middle))
        mPanning = false;
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right))
        mLooking = false;

    if (mOrbiting || mLooking)
    {
        mOrbitYaw -= io.MouseDelta.x * 0.005f;
        mOrbitPitch -= io.MouseDelta.y * 0.005f;
        mOrbitPitch = Math::clamp(mOrbitPitch, -1.5f, 1.5f);
    }
    // Pan runs along the camera's axes: a world up turns into a zoom when looking straight down.
    const Math::vec3 navRight(Math::cos(mOrbitYaw), 0.0f, Math::sin(mOrbitYaw));
    const Math::vec3 navForward(Math::sin(mOrbitYaw) * Math::cos(mOrbitPitch), Math::sin(mOrbitPitch),
                               -Math::cos(mOrbitYaw) * Math::cos(mOrbitPitch));
    const Math::vec3 navUp = Math::normalize(Math::cross(navRight, navForward));
    if (mPanning)
    {
        const f32 scale = mOrbitDistance * 0.001f;
        mOrbitTarget -= navRight * (io.MouseDelta.x * scale);
        mOrbitTarget += navUp * (io.MouseDelta.y * scale);
    }
    if (hovered && io.MouseWheel != 0.0f)
    {
        mOrbitDistance -= io.MouseWheel * mOrbitDistance * 0.15f;
        mOrbitDistance = Math::clamp(mOrbitDistance, nav.cameraMinView, nav.cameraMaxView);
    }

    const ImVec2 mouse = ImGui::GetMousePos();
    const bool overImage = mouse.x >= mImageMin.x && mouse.x <= mImageMax.x &&
                           mouse.y >= mImageMin.y && mouse.y <= mImageMax.y;
    mPointerNavigationActive =
        mPointerNavigation != 0 && overImage && ImGui::IsMouseDown(ImGuiMouseButton_Left);
    if (mPointerNavigationActive)
    {
        if (mPointerNavigation == 1)
        {
            mOrbitDistance += io.MouseDelta.y * mOrbitDistance * 0.01f;
            mOrbitDistance = Math::clamp(mOrbitDistance, nav.cameraMinView, nav.cameraMaxView);
        }
        else
        {
            const f32 panScale = mOrbitDistance * 0.001f;
            mOrbitTarget -= navRight * (io.MouseDelta.x * panScale);
            mOrbitTarget += navUp * (io.MouseDelta.y * panScale);
        }
    }

    const Math::vec3 forward(Math::sin(mOrbitYaw) * Math::cos(mOrbitPitch), Math::sin(mOrbitPitch),
                            -Math::cos(mOrbitYaw) * Math::cos(mOrbitPitch));
    const Math::vec3 right(Math::cos(mOrbitYaw), 0.0f, Math::sin(mOrbitYaw));
    if (mLooking)
    {
        const f32 speed =
            ImGui::IsKeyDown(ImGuiKey_LeftShift) ? nav.cameraFastSpeed : nav.cameraMoveSpeed;
        if (ImGui::IsKeyDown(ImGuiKey_W))
            mOrbitTarget += forward * speed;
        if (ImGui::IsKeyDown(ImGuiKey_S))
            mOrbitTarget -= forward * speed;
        if (ImGui::IsKeyDown(ImGuiKey_A))
            mOrbitTarget -= right * speed;
        if (ImGui::IsKeyDown(ImGuiKey_D))
            mOrbitTarget += right * speed;
        if (ImGui::IsKeyDown(ImGuiKey_E))
            mOrbitTarget.y += speed;
        if (ImGui::IsKeyDown(ImGuiKey_Q))
            mOrbitTarget.y -= speed;
    }
    mCameraPosition = mPerspective ? mOrbitTarget - forward * mOrbitDistance
                                   : mOrbitTarget + Math::vec3(0.0f, mOrbitDistance, 0.0f);
}

void ViewportPanel::drawSceneGizmos(GameObject& object)
{
    const Math::vec3 position = object.globalPosition();
    const Math::vec3 forward = object.forward();
    const Math::vec3 right = object.right();
    const Math::vec3 up = object.up();

    if (Camera* camera = object.getComponent<Camera>())
    {
        const f32 nearDistance = 0.2f;
        const f32 farDistance = Math::min(camera->farPlane(), 3.0f);
        const f32 halfHeight =
            camera->projectionMode() == CameraProjection::Perspective
                ? Math::tan(Math::radians(camera->fieldOfView() * 0.5f)) * farDistance
                : camera->orthographicSize() * 0.5f;
        const f32 halfWidth = halfHeight * camera->aspect();
        const Math::vec3 nearCenter = position + forward * nearDistance;
        const Math::vec3 farCenter = position + forward * farDistance;
        const Math::vec3 nearCorners[] = {
            nearCenter - right * halfWidth * nearDistance / farDistance -
                up * halfHeight * nearDistance / farDistance,
            nearCenter + right * halfWidth * nearDistance / farDistance -
                up * halfHeight * nearDistance / farDistance,
            nearCenter + right * halfWidth * nearDistance / farDistance +
                up * halfHeight * nearDistance / farDistance,
            nearCenter - right * halfWidth * nearDistance / farDistance +
                up * halfHeight * nearDistance / farDistance};
        const Math::vec3 farCorners[] = {farCenter - right * halfWidth - up * halfHeight,
                                        farCenter + right * halfWidth - up * halfHeight,
                                        farCenter + right * halfWidth + up * halfHeight,
                                        farCenter - right * halfWidth + up * halfHeight};
        for (u32 i = 0; i < 4; ++i)
        {
            const u32 next = (i + 1) % 4;
            DebugDraw().line(nearCorners[i], nearCorners[next], Color::Yellow);
            DebugDraw().line(farCorners[i], farCorners[next], Color::Yellow);
            DebugDraw().line(position, farCorners[i], Color::Yellow);
        }
    }

    if (Light* light = object.getComponent<Light>())
    {
        const Math::vec3 color = light->color();
        const Color debugColor = Color::fromRGBFloat(color.x, color.y, color.z);
        switch (light->lightType())
        {
        case LightType::Directional:
            DebugDraw().directionalLightGizmo(position, forward, debugColor);
            break;
        case LightType::Point:
            DebugDraw().pointLightGizmo(position, static_cast<PointLight*>(light)->range(),
                                        debugColor);
            break;
        case LightType::Spot:
        {
            SpotLight* spot = static_cast<SpotLight*>(light);
            DebugDraw().spotLightGizmo(position, forward, spot->range(), spot->innerAngle(),
                                       spot->outerAngle(), debugColor);
            break;
        }
        case LightType::Rectangle:
        {
            RectangleLight* rectangle = static_cast<RectangleLight*>(light);
            DebugDraw().rectangleLightGizmo(position, forward, rectangle->width(),
                                            rectangle->height(), debugColor);
            break;
        }
        }
    }

    if (Road* road = object.getComponent<Road>())
    {
        for (usize i = 0; i < road->pointCount(); ++i)
        {
            GameObject* point = road->point(i);
            if (!point)
                continue;
            const Math::vec3 pointPosition = point->globalPosition();
            DebugDraw().circle(pointPosition, Math::vec3(1.0f, 0.0f, 0.0f),
                               Math::vec3(0.0f, 1.0f, 0.0f), 0.35f, 12, Color::Orange);
            if (i > 0)
            {
                GameObject* previous = road->point(i - 1);
                if (previous)
                    DebugDraw().line(previous->globalPosition(), pointPosition, Color::Orange);
            }
        }
    }

    // An "empty" (no renderer/light) used as a marker would otherwise be invisible and unclickable.
    if (!object.hasAnyComponent())
    {
        const bool selected = app().selection().selectedId() == object.id();
        const Color color = selected ? Color(Color::Yellow) : Color(255, 120, 255, 255);
        DebugDraw().cross(position, 0.35f, color);
        DebugDraw().circle(position, right, up, 0.25f, 16, color);
    }

    if (Waypoints* waypoints = object.getComponent<Waypoints>())
    {
        const u32 count = static_cast<u32>(waypoints->pointCount());
        const bool selected = app().selection().selectedId() == object.id();
        for (u32 i = 0; i < count; ++i)
        {
            const Math::vec3 world = waypoints->worldPosition(i);
            // XZ, so the ring lies flat on the ground.
            DebugDraw().circle(world, Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f),
                               waypoints->point(i).radius, 16,
                               selected ? Color::Yellow : Color::Green);
            // Links are stored only on the lower index, so walking every node's list covers each edge once.
            for (u32 link : waypoints->point(i).links)
                if (link < count)
                    DebugDraw().line(world, waypoints->worldPosition(link), Color(60, 200, 120, 200));
        }
    }

    if (NavMeshSurface* surface = object.getComponent<NavMeshSurface>())
    {
        const std::vector<Math::vec3>& triangles = surface->navMesh().debugTriangles();
        const Math::vec3 lift(0.0f, 0.05f, 0.0f);
        const Color color(60, 170, 220, 130);
        for (usize i = 0; i + 2 < triangles.size(); i += 3)
        {
            DebugDraw().line(triangles[i] + lift, triangles[i + 1] + lift, color);
            DebugDraw().line(triangles[i + 1] + lift, triangles[i + 2] + lift, color);
            DebugDraw().line(triangles[i + 2] + lift, triangles[i] + lift, color);
        }
    }

    for (usize i = 0; i < object.childCount(); ++i)
        drawSceneGizmos(*object.child(i));
}

// mTool 2/3/4 = Move/Rotate/Scale. Nothing is drawn for Select/3D Cursor or no selection.

// Tested on the object's ORIGIN, not its bounds: a level's box contains everything and would be caught every time.
static void collectInRect(GameObject& object, const Math::mat4& viewProjection,
                          const Math::vec2& imageMin, const Math::vec2& imageSize,
                          const Math::vec2& rectMin, const Math::vec2& rectMax,
                          std::vector<u64>& out)
{
    Math::vec4 clip = viewProjection * Math::vec4(object.globalPosition(), 1.0f);
    if (clip.w > 0.0f)
    {
        clip /= clip.w;
        const Math::vec2 screen(imageMin.x + (clip.x * 0.5f + 0.5f) * imageSize.x,
                               imageMin.y + (1.0f - (clip.y * 0.5f + 0.5f)) * imageSize.y);
        if (clip.z >= -1.0f && clip.z <= 1.0f && screen.x >= rectMin.x && screen.x <= rectMax.x &&
            screen.y >= rectMin.y && screen.y <= rectMax.y)
            out.push_back(object.id());
    }
    for (usize i = 0; i < object.childCount(); ++i)
        collectInRect(*object.child(i), viewProjection, imageMin, imageSize, rectMin, rectMax, out);
}

void ViewportPanel::selectInRect(const Math::vec2& min, const Math::vec2& max, bool add)
{
    const Math::vec2 imageSize = mImageMax - mImageMin;
    if (imageSize.x <= 0.0f || imageSize.y <= 0.0f)
        return;

    std::vector<u64> hits;
    const Math::mat4 viewProjection = mEditorProjection * mEditorView;
    for (usize i = 0; i < app().scene().root().childCount(); ++i)
        collectInRect(*app().scene().root().child(i), viewProjection, mImageMin, imageSize, min,
                      max, hits);

    if (!add)
        app().selection().clear();
    for (u64 id : hits)
        if (!app().selection().isSelected(id))
            app().selection().toggle(id);
}

void ViewportPanel::selectSubmeshesInRect(GameObject& object, const Math::vec2& min,
                                          const Math::vec2& max, bool subtract)
{
    const Math::vec2 imageSize = mImageMax - mImageMin;
    if (imageSize.x <= 0.0f || imageSize.y <= 0.0f)
        return;

    MeshRenderer* renderer = object.getComponent<MeshRenderer>();
    const Mesh* mesh = renderer ? AssetManager::getSingleton().getMesh(renderer->mesh()) : nullptr;
    if (!mesh)
        return;

    // The unprojected drag rectangle is a FRUSTUM and must be tested as one: an AABB around it spans near to far and selects everything.
    const Math::mat4 inverseViewProjection = Math::inverse(mEditorProjection * mEditorView);
    const f32 ndcMinX = (min.x - mImageMin.x) / imageSize.x * 2.0f - 1.0f;
    const f32 ndcMaxX = (max.x - mImageMin.x) / imageSize.x * 2.0f - 1.0f;
    // Screen Y grows downward, NDC Y upward: the top edge is the larger NDC value.
    const f32 ndcMaxY = 1.0f - (min.y - mImageMin.y) / imageSize.y * 2.0f;
    const f32 ndcMinY = 1.0f - (max.y - mImageMin.y) / imageSize.y * 2.0f;

    Math::vec3 corner[8];
    u32 written = 0;
    for (u32 i = 0; i < 8; ++i)
    {
        const Math::vec4 ndc((i & 1) ? ndcMaxX : ndcMinX, (i & 2) ? ndcMaxY : ndcMinY,
                            (i & 4) ? 1.0f : -1.0f, 1.0f);
        const Math::vec4 world = inverseViewProjection * ndc;
        if (Math::abs(world.w) < 1e-6f)
            return;
        corner[i] = Math::vec3(world) / world.w;
        ++written;
    }
    if (written != 8)
        return;

    // Inward-facing planes from three corners of one face. Index bits: 1 = maxX, 2 = maxY, 4 = far.
    struct Plane
    {
        Math::vec3 normal;
        f32 distance;
    };
    const auto makePlane = [](const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                              const Math::vec3& inside)
    {
        Math::vec3 normal = Math::cross(b - a, c - a);
        const f32 length = Math::length(normal);
        Plane plane{Math::vec3(0.0f, 1.0f, 0.0f), 0.0f};
        if (length < 1e-8f)
            return plane;
        normal /= length;
        // Orient toward the frustum interior so one sign test means the same for all six.
        if (Math::dot(normal, inside - a) < 0.0f)
            normal = -normal;
        plane.normal = normal;
        plane.distance = -Math::dot(normal, a);
        return plane;
    };

    Math::vec3 centre(0.0f);
    for (const Math::vec3& point : corner)
        centre += point;
    centre /= 8.0f;

    const Plane planes[6] = {
        makePlane(corner[0], corner[1], corner[3], centre), // near
        makePlane(corner[4], corner[5], corner[7], centre), // far
        makePlane(corner[0], corner[2], corner[6], centre), // left
        makePlane(corner[1], corner[3], corner[7], centre), // right
        makePlane(corner[0], corner[1], corner[5], centre), // bottom
        makePlane(corner[2], corner[3], corner[7], centre)  // top
    };

    EditorApplication::SubmeshSelection& selection = app().submeshSelection();
    // A subtract pass over a different object's selection must not adopt this object.
    if (selection.object != object.id())
    {
        if (subtract)
            return;
        selection.object = object.id();
        selection.indices.clear();
    }

    // Use the CPU-side copy (imported this session) for a tight test; the box only rejects cheaply, since it is a poor fit for diagonal or L-shaped pieces.
    const MeshData* meshData = app().importedMeshData(renderer->mesh());
    const Math::mat4 transform = object.globalTransform();
    for (u32 i = 0; i < static_cast<u32>(mesh->submeshes.size()); ++i)
    {
        const AABB bounds = transformAABB(mesh->submeshes[i].bounds, transform);
        // Reject when the box is entirely behind any plane, testing the corner furthest along its normal.
        bool outside = false;
        for (const Plane& plane : planes)
        {
            const Math::vec3 furthest(plane.normal.x >= 0.0f ? bounds.max.x : bounds.min.x,
                                     plane.normal.y >= 0.0f ? bounds.max.y : bounds.min.y,
                                     plane.normal.z >= 0.0f ? bounds.max.z : bounds.min.z);
            if (Math::dot(plane.normal, furthest) + plane.distance < 0.0f)
            {
                outside = true;
                break;
            }
        }
        if (outside)
            continue;

        if (meshData && i < meshData->submeshes.size())
        {
            const SubMesh& submesh = meshData->submeshes[i];
            bool anyVertexInside = false;
            const usize end =
                Math::min<usize>(submesh.indexOffset + submesh.indexCount, meshData->indices.size());
            for (usize index = submesh.indexOffset; index < end && !anyVertexInside; ++index)
            {
                const u32 vertex = meshData->indices[index];
                if (vertex >= meshData->positions.size())
                    continue;
                const Math::vec3 world =
                    Math::vec3(transform * Math::vec4(meshData->positions[vertex], 1.0f));
                bool insideAll = true;
                for (const Plane& plane : planes)
                    if (Math::dot(plane.normal, world) + plane.distance < 0.0f)
                    {
                        insideAll = false;
                        break;
                    }
                anyVertexInside = insideAll;
            }
            if (!anyVertexInside)
                continue;
        }

        const auto found = std::find(selection.indices.begin(), selection.indices.end(), i);
        if (subtract)
        {
            if (found != selection.indices.end())
                selection.indices.erase(found);
        }
        else if (found == selection.indices.end())
            selection.indices.push_back(i);
    }
}

void ViewportPanel::drawTransformGizmo(const Math::vec2& imageMin, const Math::vec2& imageSize)
{
    if (mTool < 2 || mTool > 4)
        return;

    GameObject* selected = app().selection().resolve(app().scene());
    if (!selected)
        return;

    ImGuizmo::SetOrthographic(!mPerspective);
    ImGuizmo::SetDrawlist();
    ImGuizmo::SetRect(imageMin.x, imageMin.y, imageSize.x, imageSize.y);

    const ImGuizmo::OPERATION operation = mTool == 2   ? ImGuizmo::TRANSLATE
                                          : mTool == 3 ? ImGuizmo::ROTATE
                                                       : ImGuizmo::SCALE;

    // A picked waypoint takes the gizmo: it has only a position, so rotate/scale stay on the object.
    Waypoints* waypoints = selected->getComponent<Waypoints>();
    const s32 waypointIndex = app().selectedWaypoint();
    if (waypoints && operation == ImGuizmo::TRANSLATE && waypointIndex >= 0 &&
        waypointIndex < static_cast<s32>(waypoints->pointCount()))
    {
        const u32 index = static_cast<u32>(waypointIndex);
        Math::mat4 pointTransform =
            Math::translate(Math::mat4(1.0f), waypoints->worldPosition(index));
        float pointSnap[3] = {1.0f, 1.0f, 1.0f};
        ImGuizmo::Manipulate(Math::value_ptr(mEditorView), Math::value_ptr(mEditorProjection),
                             operation, ImGuizmo::WORLD, Math::value_ptr(pointTransform), nullptr,
                             mSnap ? pointSnap : nullptr);
        if (!ImGuizmo::IsUsing())
        {
            mGizmoDragging = false;
            return;
        }
        if (!mGizmoDragging)
        {
            mGizmoDragging = true;
            app().recordUndo();
        }
        // The gizmo works in world space; the point is stored local to its owner.
        const Math::vec3 world = Math::vec3(pointTransform[3]);
        const Math::vec3 local =
            Math::vec3(Math::inverse(selected->globalTransform()) * Math::vec4(world, 1.0f));
        waypoints->setPointPosition(index, local);
        app().markDirty();
        return;
    }

    Math::mat4 transform = selected->globalTransform();
    const float snapAmount = mTool == 2 ? 1.0f : mTool == 3 ? 15.0f : 0.1f;
    float snapValues[3] = {snapAmount, snapAmount, snapAmount};

    ImGuizmo::Manipulate(Math::value_ptr(mEditorView), Math::value_ptr(mEditorProjection), operation,
                         ImGuizmo::LOCAL, Math::value_ptr(transform), nullptr,
                         mSnap ? snapValues : nullptr);

    if (!ImGuizmo::IsUsing())
    {
        mGizmoDragging = false;
        return;
    }

    if (!mGizmoDragging)
    {
        mGizmoDragging = true;
        app().recordUndo();
    }

    Math::vec3 translation, scale, skew;
    Math::vec4 perspective;
    Math::quat rotation;
    if (!Math::decompose(transform, scale, rotation, translation, skew, perspective))
        return;

    if (operation == ImGuizmo::TRANSLATE)
        selected->setGlobalPosition(translation);
    else if (operation == ImGuizmo::ROTATE)
    {
        selected->setGlobalRotation(rotation);
        if (selected->getComponent<DirectionalLight>())
            app().engine().sky().sunFromSky = false;
    }
    else
    {
        const Math::vec3 parentScale =
            selected->parent() ? selected->parent()->globalScale() : Math::vec3(1.0f);
        selected->setScale(scale / Math::max(parentScale, Math::vec3(0.0001f)));
    }
    app().markDirty();
}

// FK bone (rotate, local) or IK target (translate, world; Skeleton.h) instead of the object's transform. No-op unless AnimationPanel armed animationPoseTarget() and the object still has a pose-edit Animator.
void ViewportPanel::drawBonePoseGizmo(const Math::vec2& imageMin, const Math::vec2& imageSize)
{
    const EditorApplication::AnimationPoseTarget& target = app().animationPoseTarget();
    GameObject* selected = app().selection().resolve(app().scene());
    if (!selected)
        return;
    Animator* animator = selected->getComponent<Animator>();
    if (!animator || !animator->poseEditMode())
        return;
    const Skeleton* skeleton = animator->skeleton();
    if (!skeleton)
        return;

    ImGuizmo::SetOrthographic(!mPerspective);
    ImGuizmo::SetDrawlist();
    ImGuizmo::SetRect(imageMin.x, imageMin.y, imageSize.x, imageSize.y);
    const Math::mat4 ownerTransform = selected->globalTransform();

    if (target.bone >= 0 && static_cast<u32>(target.bone) < skeleton->boneCount())
    {
        const u32 bone = static_cast<u32>(target.bone);
        Math::mat4 world = ownerTransform * animator->globalPose()[bone];

        ImGuizmo::Manipulate(Math::value_ptr(mEditorView), Math::value_ptr(mEditorProjection),
                             ImGuizmo::ROTATE, ImGuizmo::LOCAL, Math::value_ptr(world));
        if (!ImGuizmo::IsUsing())
            return;

        const s32 parent = skeleton->bone(bone).parent;
        const Math::mat4 parentWorld =
            parent >= 0 ? ownerTransform * animator->globalPose()[static_cast<u32>(parent)]
                        : ownerTransform;
        const Math::mat4 local = Math::inverse(parentWorld) * world;

        Math::vec3 translation, scale, skew;
        Math::vec4 perspective;
        Math::quat rotation;
        if (!Math::decompose(local, scale, rotation, translation, skew, perspective))
            return;

        LocalPose pose = animator->localPose()[bone];
        pose.rotation = Math::normalize(rotation);
        animator->setBoneLocalPose(bone, pose);
    }
    else if (target.ikChain >= 0 && static_cast<u32>(target.ikChain) < animator->ikChainCount())
    {
        IKChain* chain = animator->ikChain(static_cast<u32>(target.ikChain));
        if (!chain)
            return;
        Math::mat4 world = Math::translate(Math::mat4(1.0f), chain->target);

        ImGuizmo::Manipulate(Math::value_ptr(mEditorView), Math::value_ptr(mEditorProjection),
                             ImGuizmo::TRANSLATE, ImGuizmo::WORLD, Math::value_ptr(world));
        if (!ImGuizmo::IsUsing())
            return;

        chain->target = Math::vec3(world[3]);
    }
}

void ViewportPanel::onImGui()
{
    // Required once per frame after ImGui_XXXX_NewFrame() (ImGuizmo.h), or the gizmo's state never resets and it draws misplaced.
    ImGuizmo::BeginFrame();

    const char* tools[] = {ICON_MDI_CURSOR_DEFAULT, ICON_MDI_CROSSHAIRS, ICON_MDI_ARROW_ALL,
                           ICON_MDI_ROTATE_ORBIT, ICON_MDI_ARROW_EXPAND_ALL, ICON_MDI_TARGET};
    const char* tooltips[] = {"Select", "3D Cursor", "Move", "Rotate", "Scale", "Pick Surface"};
    for (int i = 0; i < 6; ++i)
    {
        if (i > 0)
            ImGui::SameLine();
        const bool active = mTool == i;
        if (active)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(tools[i]))
            mTool = i;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", tooltips[i]);
        if (active)
            ImGui::PopStyleColor();
    }
    ImGui::SameLine();
    const bool snapActive = mSnap;
    if (snapActive)
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    if (ImGui::Button(ICON_MDI_MAGNET))
        mSnap = !mSnap;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Snap");
    if (snapActive)
        ImGui::PopStyleColor();
    ImGui::SameLine();
    const bool fastActive = mFastRender;
    if (fastActive)
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    if (ImGui::Button(ICON_MDI_FLASH))
        mFastRender = !mFastRender;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Fast render: shadows, reflections, SSAO, volumetrics, lens flares and "
                         "post-process all off, so a heavy scene stays navigable while editing.");
    if (fastActive)
        ImGui::PopStyleColor();
    ImGui::SameLine();
    const bool gridActive = mGrid;
    if (gridActive)
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    if (ImGui::Button(ICON_MDI_GRID))
        mGrid = !mGrid;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Grid");
    if (gridActive)
        ImGui::PopStyleColor();
    ImGui::SameLine();
    if (ImGui::Button(mPerspective ? "3D" : "2D"))
        mPerspective = !mPerspective;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Toggle 2D/3D view");
    ImGui::SameLine();
    const bool zoomActive = mPointerNavigation == 1;
    if (zoomActive)
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    if (ImGui::Button(ICON_MDI_MAGNIFY_PLUS))
        mPointerNavigation = zoomActive ? 0 : 1;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Left-drag in the viewport to zoom");
    if (zoomActive)
        ImGui::PopStyleColor();
    ImGui::SameLine();
    const bool panActive = mPointerNavigation == 2;
    if (panActive)
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    if (ImGui::Button(ICON_MDI_HAND))
        mPointerNavigation = panActive ? 0 : 2;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Left-drag in the viewport to pan");
    if (panActive)
        ImGui::PopStyleColor();
    ImGui::SameLine();
    // Drives PostProcessStack::enabled directly, the same flag as Settings > Post Process; a second gate let Settings show Enabled with no effect.
    bool& postProcessEnabled = app().engine().postProcess().enabled;
    if (postProcessEnabled)
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    if (ImGui::Button(ICON_MDI_IMAGE_FILTER_HDR))
        postProcessEnabled = !postProcessEnabled;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Post Process (bloom, tone mapping, SSAO, ...) - same switch as "
                          "Settings > Post Process > Enabled.");
    if (postProcessEnabled)
        ImGui::PopStyleColor();
    ImGui::Dummy(ImVec2(0.0f, 4.0f));
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 3.0f));

    GameObject* selectedTerrainObject = app().selection().resolve(app().scene());
    Terrain* selectedTerrain = selectedTerrainObject
                                  ? selectedTerrainObject->getComponent<Terrain>()
                                  : nullptr;
    if (selectedTerrain)
    {
        ImGui::TextUnformatted("Terrain tools");
        if (ImGui::Button(mTerrainPaintActive ? "Stop paint" : "Paint terrain"))
        {
            mTerrainPaintActive = !mTerrainPaintActive;
            mTerrainDeformActive = false;
        }
        ImGui::SameLine();
        if (ImGui::Button(mTerrainDeformActive ? "Stop deform" : "Deform terrain"))
        {
            mTerrainDeformActive = !mTerrainDeformActive;
            mTerrainPaintActive = false;
        }
        if (mTerrainDeformActive)
        {
            const char* modes[] = {"Raise", "Lower", "Smooth", "Flatten 0"};
            ImGui::Combo("Deform mode", &mTerrainDeformMode, modes, 4);
            ImGui::TextDisabled("LMB deforms the heightmap");
        }
        else
        {
            ImGui::Checkbox("Vegetation", &mTerrainPaintVegetation);
            if (mTerrainPaintVegetation)
            {
                const char* channels[] = {"Grass", "Flowers", "Bushes", "Trees"};
                ImGui::Combo("Channel", &mTerrainPaintChannel, channels, 4);
            }
            else
            {
                const char* layers[] = {"Base / grass", "Rock", "Soil", "High / snow"};
                ImGui::Combo("Layer", &mTerrainPaintLayer, layers, 4);
            }
            ImGui::TextDisabled("LMB paints; Shift+LMB restores/erases");
        }
        ImGui::SliderFloat("Brush radius", &mTerrainBrushRadius, 0.5f, 80.0f, "%.1f");
        ImGui::SliderFloat("Brush strength", &mTerrainBrushStrength, 0.05f, 10.0f, "%.2f");
        ImGui::Separator();
    }

    if (const u64 focusId = app().takeFocusObjectRequest())
    {
        if (GameObject* target = app().scene().findGameObject(focusId))
        {
            // The Inspector's per-submesh focus routes through the same request as Hierarchy's, plus the object's pickedSubmesh().
            const EditorApplication::PickedSubmesh& picked = app().pickedSubmesh();
            focusOnObject(*target, picked.object == focusId ? picked.index : -1);
        }
    }

    updateNavigation();
    const bool navigating =
        mOrbiting || mPanning || mLooking || mPointerNavigationActive || mNavigationGizmoActive;
    app().engine().setProbeCaptureDeferred(navigating);

    // Kept live: EditorSettings::save() only runs on a clean exit and a crash would lose the pose.
    EditorSettings& editorSettings = app().settings();
    editorSettings.cameraPosition = mCameraPosition;
    editorSettings.cameraOrbitTarget = mOrbitTarget;
    editorSettings.cameraOrbitDistance = mOrbitDistance;
    editorSettings.cameraOrbitYaw = mOrbitYaw;
    editorSettings.cameraOrbitPitch = mOrbitPitch;
    editorSettings.cameraPerspective = mPerspective;
    editorSettings.viewportGrid = mGrid;
    editorSettings.viewportSnap = mSnap;
    editorSettings.viewportTool = mTool;
    editorSettings.viewportNavigationTool = mPointerNavigation;

    ImVec2 size = ImGui::GetContentRegionAvail();
    size.x = Math::max(size.x, 1.0f);
    size.y = Math::max(size.y, 1.0f);
    const u32 width = static_cast<u32>(size.x);
    const u32 height = static_cast<u32>(size.y);

    // Only the live view submits the scene (EditorApplication::ViewMode); gizmos are gated too since DebugDraw's list is frame-wide and would draw editor overlays over the game picture.
    const bool live = app().viewMode() == EditorApplication::ViewMode::Scene;

    if (live && mGrid)
        DebugDraw().grid(app().cursor3D().y, 40, 1.0f, true);
    if (live)
        drawSceneGizmos(app().scene().root());
    if (live && app().showDynamicIndexDebug())
        app().scene().debugDrawDynamicIndex();
    if (live && app().showOcclusionDebug())
        app().scene().debugDrawOcclusion();
    if (live && app().engine().debugShowPhysicsShapes)
        app().scene().debugDrawPhysicsShapes();
    if (live && app().engine().debugShowPhysicsContacts)
        app().scene().debugDrawPhysicsContacts();
    if (live && app().engine().debugShowPhysicsJoints)
        app().scene().debugDrawPhysicsJoints();
    if (live && app().engine().debugShowAIObstacles)
        app().scene().debugDrawObstacles();
    if (GameObject* selected = live ? app().selection().resolve(app().scene()) : nullptr)
    {
        if (MeshRenderer* renderer = selected->getComponent<MeshRenderer>())
        {
            if (renderer->mesh().valid())
            {
                // The outline hull uses raw un-skinned vertices (DebugDraw3D.cpp's outline shader has no skin palette), wrong for a posed skinned mesh; a world AABB stands in until the outline skins.
                const Mesh* mesh = Assets().getMesh(renderer->mesh());
                constexpr u32 maxOutlineIndices = 300000;
                if (mesh && (mesh->isSkinned() || mesh->indexCount > maxOutlineIndices))
                    DebugDraw().box(transformAABB(mesh->bounds, selected->globalTransform()),
                                    Color::Orange);
                else
                    DebugDraw().outline(renderer->mesh(), selected->globalTransform(),
                                        Color::Orange);

                if (mesh && app().showSubmeshBounds())
                {
                    for (const SubMesh& submesh : mesh->submeshes)
                        DebugDraw().box(transformAABB(submesh.bounds, selected->globalTransform()),
                                        Color::Gray);
                }

                // The last-clicked submesh, cyan (not selection orange) inside the whole-object outline.
                const EditorApplication::PickedSubmesh& picked = app().pickedSubmesh();
                if (mesh && picked.index >= 0 && picked.object == selected->id() &&
                    static_cast<usize>(picked.index) < mesh->submeshes.size())
                {
                    DebugDraw().box(transformAABB(mesh->submeshes[picked.index].bounds,
                                                  selected->globalTransform()),
                                    Color::Cyan);
                }

                // The Shift-click batch, yellow to differ from the cyan last-picked box and orange outline.
                const EditorApplication::SubmeshSelection& multiSelected = app().submeshSelection();
                if (mesh && multiSelected.object == selected->id())
                {
                    for (u32 index : multiSelected.indices)
                        if (static_cast<usize>(index) < mesh->submeshes.size())
                            DebugDraw().box(transformAABB(mesh->submeshes[index].bounds,
                                                          selected->globalTransform()),
                                            Color::Yellow);
                }
            }
        }
        if (Physics::RigidBody* rigidBody = selected->getComponent<Physics::RigidBody>())
        {
            Color color = Color::Gray;
            if (rigidBody->bodyType() == Physics::BodyType::Kinematic)
                color = Color(230, 200, 60, 255);
            else if (rigidBody->bodyType() == Physics::BodyType::Dynamic)
            {
                switch (rigidBody->shapeKind())
                {
                case Physics::RigidBodyShape::Sphere:  color = Color(80, 220, 200, 255); break;
                case Physics::RigidBodyShape::Box:     color = Color(110, 220, 90, 255); break;
                case Physics::RigidBodyShape::Capsule: color = Color(220, 110, 220, 255); break;
                default: break;
                }
            }

            const Math::mat4 bodyTransform = Math::translate(Math::mat4(1.0f), selected->globalPosition()) *
                                            Math::mat4_cast(selected->globalRotation());
            switch (rigidBody->shapeKind())
            {
            case Physics::RigidBodyShape::Sphere:
                Physics::SphereShape(rigidBody->radius()).debugDraw(bodyTransform, color);
                break;
            case Physics::RigidBodyShape::Box:
                Physics::BoxShape(rigidBody->halfExtents()).debugDraw(bodyTransform, color);
                break;
            case Physics::RigidBodyShape::Capsule:
                Physics::CapsuleShape(rigidBody->radius(), rigidBody->capsuleSegmentHalfHeight())
                    .debugDraw(bodyTransform, color);
                break;
            case Physics::RigidBodyShape::None:
                if (rigidBody->shape())
                    rigidBody->shape()->debugDraw(bodyTransform, color);
                break;
            }
        }
        if (Collider* collider = selected->getComponent<Collider>())
        {
            const Color color = Color::Green;
            const Math::mat4 colliderTransform =
                Math::translate(Math::mat4(1.0f), selected->globalPosition()) *
                Math::mat4_cast(selected->globalRotation());
            switch (collider->shape())
            {
            case ColliderShape::Sphere:
                Physics::SphereShape(collider->radius()).debugDraw(colliderTransform, color);
                break;
            case ColliderShape::Box:
                Physics::BoxShape(collider->halfExtents()).debugDraw(colliderTransform, color);
                break;
            case ColliderShape::Capsule:
                Physics::CapsuleShape(collider->radius(), collider->capsuleSegmentHalfHeight())
                    .debugDraw(colliderTransform, color);
                break;
            case ColliderShape::Mesh:
                DebugDraw().box(collider->worldBounds(), color);
                break;
            }
        }
        if (Obstacle* obstacle = selected->getComponent<Obstacle>())
            Scene::debugDrawObstacleShape(*obstacle, Color(200, 80, 220, 255));
    }
    const f32 cursorRadius = Math::max(0.1f, mOrbitDistance * 0.025f);
    const Math::vec3 forward(Math::sin(mOrbitYaw) * Math::cos(mOrbitPitch), Math::sin(mOrbitPitch),
                            -Math::cos(mOrbitYaw) * Math::cos(mOrbitPitch));
    const Math::vec3 cameraRight = Math::normalize(Math::cross(forward, Math::vec3(0.0f, 1.0f, 0.0f)));
    const Math::vec3 cameraUp = Math::normalize(Math::cross(cameraRight, forward));
    if (live)
        DebugDraw().cursor3D(app().cursor3D(), cameraRight, cameraUp, cursorRadius);
    // The observer's own RenderView built from the orbit state, never touching the scene's Camera (see RenderView in Engine.h).
    const f32 aspect = static_cast<f32>(width) / static_cast<f32>(height);
    const EditorSettings& editorCamera = app().settings();
    const f32 nearPlane = editorCamera.cameraNearPlane;
    const f32 farPlane = Math::max(editorCamera.cameraFarPlane, nearPlane + 0.001f);
    RenderView view;
    view.position = mCameraPosition;
    if (mPerspective)
    {
        view.view = Math::lookAt(mCameraPosition, mOrbitTarget, Math::vec3(0.0f, 1.0f, 0.0f));
        view.fieldOfView = 60.0f;
        view.nearPlane = nearPlane;
        view.aspect = aspect;
        view.projection =
            Math::perspective(Math::radians(view.fieldOfView), aspect, view.nearPlane, farPlane);
    }
    else
    {
        view.view = Math::lookAt(mCameraPosition, mOrbitTarget, Math::vec3(0.0f, 0.0f, -1.0f));
        view.nearPlane = nearPlane;
        view.aspect = aspect;
        const f32 halfHeight = mOrbitDistance * 0.5f;
        const f32 halfWidth = halfHeight * aspect;
        view.projection =
            Math::ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, view.nearPlane, farPlane);
    }
    RenderTextureSettings settings;
    settings.outputIndex = 0;
    // The scene editor must stay pixel-stable; the Game panel has its own output stream and keeps TAA.
    settings.temporalAA = false;
    const bool renderPostProcess = app().engine().postProcess().enabled && !mFastRender;
    const EditorSettings& preview = app().settings();
    // Fast render: every optional pass off; the frame cost lives in these passes, not the geometry.
    settings.shadows = preview.previewShadows && !mFastRender;
    settings.planarReflections = preview.previewPlanarReflections && !mFastRender;
    settings.postProcess = renderPostProcess;
    settings.ambientOcclusion = renderPostProcess && preview.previewSSAO;
    settings.volumetrics = renderPostProcess && preview.previewVolumetrics;
    settings.lensFlares = renderPostProcess && preview.previewLensFlares;
    settings.previewOcclusionCulling = preview.previewOcclusionCulling;
    const f32 previewScale = navigating ? preview.previewNavigationScale : 1.0f;
    const u32 previewWidth = Math::max(1u, static_cast<u32>(width * previewScale));
    const u32 previewHeight = Math::max(1u, static_cast<u32>(height * previewScale));
    if (live)
    {
        if (app().engine().renderToTexture(app().scene(), view, previewWidth, previewHeight,
                                           mOutput, settings))
            app().notifySceneRendered();
    }
    mEditorView = view.view;
    mEditorProjection = view.projection;
    if (mOutput.valid())
    {
        const u32 nativeId = app().engine().getGPU().nativeTextureId(mOutput.color);
        const ImVec2 imageMin = ImGui::GetCursorScreenPos();
        ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(nativeId)), size,
                     ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
        mImageMin = Math::vec2(imageMin.x, imageMin.y);
        mImageMax = Math::vec2(imageMin.x + size.x, imageMin.y + size.y);

        // Camera preview reuses GamePanel's already-rendered activeCamera() texture (outputIndex=1); a selected non-active camera shows nothing.
        if (GameObject* selected = app().selection().resolve(app().scene()))
        {
            Camera* previewCamera = selected->getComponent<Camera>();
            TextureHandle gameTexture = app().gameTexture();
            if (previewCamera && previewCamera == app().scene().activeCamera() &&
                gameTexture.valid())
            {
                const f32 margin = 12.0f;
                const f32 previewWidth = 220.0f;
                const f32 previewHeight = previewWidth * 9.0f / 16.0f;
                const ImVec2 previewMin(mImageMax.x - previewWidth - margin, mImageMin.y + margin);
                const ImVec2 previewMax(previewMin.x + previewWidth, previewMin.y + previewHeight);
                const u32 previewNativeId = app().engine().getGPU().nativeTextureId(gameTexture);
                ImDrawList* overlay = ImGui::GetWindowDrawList();
                overlay->AddImage(
                    static_cast<ImTextureID>(static_cast<uintptr_t>(previewNativeId)),
                    previewMin, previewMax, ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
                overlay->AddRect(previewMin, previewMax, IM_COL32(255, 200, 60, 255), 0.0f, 0,
                                 2.0f);
                overlay->AddText(ImVec2(previewMin.x + 4.0f, previewMin.y + 2.0f),
                                 IM_COL32(255, 200, 60, 255), "Camera");
            }
        }

        ImGui::SetCursorScreenPos(imageMin);
        if (app().animationPoseTarget().active)
            drawBonePoseGizmo(Math::vec2(imageMin.x, imageMin.y), Math::vec2(size.x, size.y));
        else
            drawTransformGizmo(Math::vec2(imageMin.x, imageMin.y), Math::vec2(size.x, size.y));

        // ImGuizmo's CanActivate() refuses to drag when IsAnyItemHovered() was true this or the previous frame, so the InvisibleButton must never register as hovered while the gizmo owns the pixel: use Dummy() (same layout, no interaction) when IsOver()/IsUsing().
        const bool gizmoWantsInput = ImGuizmo::IsOver() || ImGuizmo::IsUsing();
        ImGui::SetCursorScreenPos(imageMin);
        ImGui::PushID("viewport.image.input");
        bool imageHovered = false;
        if (gizmoWantsInput)
            ImGui::Dummy(size);
        else
        {
            ImGui::InvisibleButton("input", size);
            imageHovered = ImGui::IsItemHovered();
        }
        ImGui::PopID();
        // InvisibleButton/Dummy already advanced the cursor; repositioning triggers ImGui's "SetCursorScreenPos extends window boundaries".
        const ImVec2 navigationCenter(imageMin.x + size.x - 55.0f, imageMin.y + 55.0f);
        const ImVec2 mouse = ImGui::GetMousePos();
        const Math::vec3 navigationForward(Math::sin(mOrbitYaw) * Math::cos(mOrbitPitch),
                                          Math::sin(mOrbitPitch),
                                          -Math::cos(mOrbitYaw) * Math::cos(mOrbitPitch));
        Math::vec3 navigationRight = Math::cross(navigationForward, Math::vec3(0.0f, 1.0f, 0.0f));
        if (Math::dot(navigationRight, navigationRight) < 0.001f)
            navigationRight = Math::vec3(1.0f, 0.0f, 0.0f);
        else
            navigationRight = Math::normalize(navigationRight);
        const Math::vec3 navigationUp =
            Math::normalize(Math::cross(navigationRight, navigationForward));
        constexpr f32 axisLength = 30.0f;
        const Math::vec3 worldX(1.0f, 0.0f, 0.0f);
        const Math::vec3 worldY(0.0f, 1.0f, 0.0f);
        const Math::vec3 worldZ(0.0f, 0.0f, 1.0f);
        const ImVec2 xAxis(navigationCenter.x + Math::dot(worldX, navigationRight) * axisLength,
                           navigationCenter.y - Math::dot(worldX, navigationUp) * axisLength);
        const ImVec2 yAxis(navigationCenter.x + Math::dot(worldY, navigationRight) * axisLength,
                           navigationCenter.y - Math::dot(worldY, navigationUp) * axisLength);
        const ImVec2 zAxis(navigationCenter.x + Math::dot(worldZ, navigationRight) * axisLength,
                           navigationCenter.y - Math::dot(worldZ, navigationUp) * axisLength);
        const ImVec2 xAxisNeg(navigationCenter.x * 2.0f - xAxis.x,
                              navigationCenter.y * 2.0f - xAxis.y);
        const ImVec2 yAxisNeg(navigationCenter.x * 2.0f - yAxis.x,
                              navigationCenter.y * 2.0f - yAxis.y);
        const ImVec2 zAxisNeg(navigationCenter.x * 2.0f - zAxis.x,
                              navigationCenter.y * 2.0f - zAxis.y);
        const f32 navigationDx = mouse.x - navigationCenter.x;
        const f32 navigationDy = mouse.y - navigationCenter.y;
        bool navigationClicked = false;
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            // A ball click snaps to that axis view; pitch stops short of +-90 degrees so lookAt's fixed up vector stays valid.
            const struct
            {
                ImVec2 ball;
                f32 yaw;
                f32 pitch;
            } views[6] = {
                {xAxis, -Math::half_pi<f32>(), 0.0f},
                {xAxisNeg, Math::half_pi<f32>(), 0.0f},
                {yAxis, mOrbitYaw, -1.5f},
                {yAxisNeg, mOrbitYaw, 1.5f},
                {zAxis, 0.0f, 0.0f},
                {zAxisNeg, Math::pi<f32>(), 0.0f},
            };
            bool snapped = false;
            for (const auto& view : views)
            {
                const f32 dx = mouse.x - view.ball.x;
                const f32 dy = mouse.y - view.ball.y;
                if (dx * dx + dy * dy < 100.0f)
                {
                    mOrbitYaw = view.yaw;
                    mOrbitPitch = view.pitch;
                    navigationClicked = true;
                    snapped = true;
                    break;
                }
            }
            if (!snapped && navigationDx * navigationDx + navigationDy * navigationDy < 2500.0f)
            {
                mNavigationGizmoActive = true;
                mNavigationGizmoStartMouse = Math::vec2(mouse.x, mouse.y);
                mNavigationGizmoStartYaw = mOrbitYaw;
                mNavigationGizmoStartPitch = mOrbitPitch;
                navigationClicked = true;
            }
        }
        if (mNavigationGizmoActive && ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            mOrbitYaw =
                mNavigationGizmoStartYaw - (mouse.x - mNavigationGizmoStartMouse.x) * 0.005f;
            mOrbitPitch = Math::clamp(mNavigationGizmoStartPitch -
                                         (mouse.y - mNavigationGizmoStartMouse.y) * 0.005f,
                                     -1.5f, 1.5f);
            navigationClicked = true;
        }
        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
            mNavigationGizmoActive = false;
        bool terrainPainted = false;
        if (live && selectedTerrain && (mTerrainPaintActive || mTerrainDeformActive) && imageHovered &&
            !navigationClicked && !mPointerNavigationActive && !ImGuizmo::IsOver() &&
            !ImGuizmo::IsUsing() && !ImGui::GetIO().WantCaptureMouse)
        {
            const f32 ndcX = (mouse.x - imageMin.x) / size.x * 2.0f - 1.0f;
            const f32 ndcY = 1.0f - (mouse.y - imageMin.y) / size.y * 2.0f;
            const Math::mat4 inverseViewProjection =
                Math::inverse(mEditorProjection * mEditorView);
            Math::vec4 nearPoint = inverseViewProjection * Math::vec4(ndcX, ndcY, -1.0f, 1.0f);
            Math::vec4 farPoint = inverseViewProjection * Math::vec4(ndcX, ndcY, 1.0f, 1.0f);
            nearPoint /= nearPoint.w;
            farPoint /= farPoint.w;
            Ray ray;
            ray.origin = Math::vec3(nearPoint);
            ray.direction = Math::normalize(Math::vec3(farPoint - nearPoint));
            Math::vec3 hit;
            if (selectedTerrain->raycast(ray, hit))
            {
                const bool stroke = ImGui::IsMouseDown(ImGuiMouseButton_Left);
                terrainPainted = stroke;
                if (stroke && !mTerrainStrokeUndo)
                {
                    app().recordUndo();
                    mTerrainStrokeUndo = true;
                }
                if (stroke)
                {
                    if (mTerrainDeformActive)
                    {
                        const f32 amount = mTerrainBrushStrength *
                                           app().engine().getWindow().getDeltaTime();
                        switch (mTerrainDeformMode)
                        {
                        case 0:
                            terrainPainted = selectedTerrain->raise(hit, mTerrainBrushRadius, amount);
                            break;
                        case 1:
                            terrainPainted = selectedTerrain->lower(hit, mTerrainBrushRadius, amount);
                            break;
                        case 2:
                            terrainPainted = selectedTerrain->smooth(
                                hit, mTerrainBrushRadius, Math::min(1.0f, amount));
                            break;
                        default:
                            terrainPainted = selectedTerrain->flatten(
                                hit, mTerrainBrushRadius, 0.0f, Math::min(1.0f, amount));
                            break;
                        }
                    }
                    else if (mTerrainPaintVegetation)
                    {
                        if (!selectedTerrain->hasVegetationMask())
                            selectedTerrain->createVegetationMask();
                        terrainPainted = selectedTerrain->paintVegetation(
                            hit, mTerrainBrushRadius,
                            static_cast<Terrain::VegetationChannel>(mTerrainPaintChannel),
                            mTerrainBrushStrength * app().engine().getWindow().getDeltaTime(),
                            ImGui::GetIO().KeyShift);
                    }
                    else
                    {
                        if (!selectedTerrain->hasSurfaceSplat())
                            selectedTerrain->createSurfaceSplat();
                        terrainPainted = selectedTerrain->paintSurface(
                            hit, mTerrainBrushRadius,
                            static_cast<Terrain::SurfaceLayer>(mTerrainPaintLayer),
                            mTerrainBrushStrength * app().engine().getWindow().getDeltaTime(),
                            ImGui::GetIO().KeyShift);
                    }
                    if (terrainPainted)
                        app().markDirty();
                }
            }
        }
        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
            mTerrainStrokeUndo = false;
        // Shift is the accumulate modifier with Pick Surface, not place-cursor; both claiming it blocked Shift-drag rectangles.
        const bool clickToPlaceCursor = mTool == 1 || (ImGui::GetIO().KeyShift && mTool != 5);
        // Select is the plain click case: place-cursor and gizmo dragging already claim left-click (same guard as clickToPlaceCursor).
        const bool clickToSelect =
            (mTool == 0 || mTool == 5) && !ImGuizmo::IsOver() && !ImGuizmo::IsUsing();
        // Rubber band starts on a plain left-press with Select and becomes a box only after the mouse travels, so a click still picks one object. Under Pick Surface, Shift-click adds one submesh and Shift-drag adds all covered.
        GameObject* submeshRectTarget = app().selection().resolve(app().scene());
        if (submeshRectTarget && !submeshRectTarget->getComponent<MeshRenderer>())
            submeshRectTarget = nullptr;
        if (imageHovered && !navigationClicked && !mPointerNavigationActive && !terrainPainted &&
            clickToSelect && !clickToPlaceCursor && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            mSubmeshRectSelecting =
                mTool == 5 && ImGui::GetIO().KeyShift && submeshRectTarget != nullptr;
            mRectSelecting = !mSubmeshRectSelecting;
            mRectStart = Math::vec2(mouse.x, mouse.y);
        }
        if (mSubmeshRectSelecting)
        {
            const Math::vec2 current(mouse.x, mouse.y);
            const Math::vec2 rectMin(Math::min(mRectStart.x, current.x),
                                    Math::min(mRectStart.y, current.y));
            const Math::vec2 rectMax(Math::max(mRectStart.x, current.x),
                                    Math::max(mRectStart.y, current.y));
            const bool dragged = (rectMax.x - rectMin.x) > 4.0f || (rectMax.y - rectMin.y) > 4.0f;
            // Ctrl with Shift makes a subtract, drawn red instead of orange.
            const bool subtract = ImGui::GetIO().KeyCtrl;
            if (dragged)
            {
                ImDrawList* drawList = ImGui::GetWindowDrawList();
                const ImU32 fill = subtract ? IM_COL32(255, 70, 70, 40) : IM_COL32(255, 160, 60, 40);
                const ImU32 border =
                    subtract ? IM_COL32(255, 110, 110, 220) : IM_COL32(255, 190, 90, 220);
                drawList->AddRectFilled(ImVec2(rectMin.x, rectMin.y), ImVec2(rectMax.x, rectMax.y),
                                        fill);
                drawList->AddRect(ImVec2(rectMin.x, rectMin.y), ImVec2(rectMax.x, rectMax.y),
                                  border);
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
            {
                mSubmeshRectSelecting = false;
                if (dragged && submeshRectTarget)
                {
                    selectSubmeshesInRect(*submeshRectTarget, rectMin, rectMax, subtract);
                    return;
                }
            }
            // Only a real drag takes the click: a Shift-press that never travels must reach the pick below.
            if (dragged)
                return;
        }
        if (mRectSelecting)
        {
            const Math::vec2 current(mouse.x, mouse.y);
            const Math::vec2 rectMin(Math::min(mRectStart.x, current.x),
                                    Math::min(mRectStart.y, current.y));
            const Math::vec2 rectMax(Math::max(mRectStart.x, current.x),
                                    Math::max(mRectStart.y, current.y));
            const bool dragged = (rectMax.x - rectMin.x) > 4.0f || (rectMax.y - rectMin.y) > 4.0f;
            if (dragged)
            {
                ImDrawList* drawList = ImGui::GetWindowDrawList();
                drawList->AddRectFilled(ImVec2(rectMin.x, rectMin.y), ImVec2(rectMax.x, rectMax.y),
                                        IM_COL32(90, 160, 255, 40));
                drawList->AddRect(ImVec2(rectMin.x, rectMin.y), ImVec2(rectMax.x, rectMax.y),
                                  IM_COL32(120, 190, 255, 200));
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
            {
                mRectSelecting = false;
                if (dragged)
                {
                    selectInRect(rectMin, rectMax, ImGui::GetIO().KeyShift);
                    return;
                }
            }
        }

        if (imageHovered && !navigationClicked && !mPointerNavigationActive &&
            !terrainPainted && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
            (clickToPlaceCursor || clickToSelect))
        {
            const f32 x = (mouse.x - imageMin.x) / size.x * 2.0f - 1.0f;
            const f32 y = 1.0f - (mouse.y - imageMin.y) / size.y * 2.0f;
            const Math::mat4 inverseViewProjection = Math::inverse(mEditorProjection * mEditorView);
            Math::vec4 nearPoint = inverseViewProjection * Math::vec4(x, y, -1.0f, 1.0f);
            Math::vec4 farPoint = inverseViewProjection * Math::vec4(x, y, 1.0f, 1.0f);
            nearPoint /= nearPoint.w;
            farPoint /= farPoint.w;
            const Math::vec3 direction = Math::normalize(Math::vec3(farPoint - nearPoint));

            if (clickToSelect)
            {
                // Read back the depth buffer to find the object under the cursor; a ray-vs-AABB test can pick an object through a poorly fitting box. Fall back to the ray only with no surface (sky).
                GameObject* hit = nullptr;
                Math::vec3 surfacePosition, surfaceNormal;
                const bool hasSurfacePosition =
                    mOutput.valid() &&
                    Scene::pickSurface(mOutput.depth, mOutput.width, mOutput.height,
                                       mouse.x - imageMin.x, mouse.y - imageMin.y, size.x, size.y,
                                       Math::inverse(mEditorProjection), Math::inverse(mEditorView),
                                       surfacePosition, surfaceNormal);
                if (hasSurfacePosition)
                    hit = app().scene().pickObjectAtPoint(surfacePosition);

                GameObject* selectedObject = app().selection().resolve(app().scene());
                Road* selectedRoad = selectedObject ? selectedObject->getComponent<Road>() : nullptr;
                GameObject* roadObject = selectedObject;
                if (!selectedRoad && selectedObject && selectedObject->parent())
                {
                    roadObject = selectedObject->parent();
                    selectedRoad = roadObject->getComponent<Road>();
                }
                if (selectedRoad && ImGui::GetIO().KeyCtrl)
                {
                    Math::vec3 pointPosition = hasSurfacePosition ? surfacePosition : app().cursor3D();
                    if (!hasSurfacePosition && Math::abs(direction.y) > 0.0001f)
                    {
                        const f32 t = -nearPoint.y / direction.y;
                        if (t >= 0.0f)
                            pointPosition = Math::vec3(nearPoint) + direction * t;
                    }
                    app().recordUndo();
                    GameObject* point = app().scene().createGameObject("Road Point", roadObject);
                    if (point)
                    {
                        point->setGlobalPosition(pointPosition);
                        if (!selectedRoad->addPoint(point))
                            app().scene().destroy(point);
                        else
                        {
                            selectedRoad->rebuild();
                            app().selection().select(point->id());
                            app().markDirty();
                        }
                    }
                    return;
                }
                const EditorApplication::VegetationPlacementMode vegetationMode =
                    app().vegetationPlacementMode();
                if (vegetationMode != EditorApplication::VegetationPlacementMode::None && selectedObject)
                {
                    Math::vec3 point = hasSurfacePosition ? surfacePosition : app().cursor3D();
                    Math::vec3 normal(0.0f, 1.0f, 0.0f);
                    if (hasSurfacePosition)
                        normal = surfaceNormal;
                    else if (Math::abs(direction.y) > 0.0001f)
                    {
                        const f32 t = -nearPoint.y / direction.y;
                        if (t >= 0.0f)
                            point = Math::vec3(nearPoint) + direction * t;
                    }
                    const Math::mat4 inverseTransform = Math::inverse(selectedObject->globalTransform());
                    const Math::vec3 local = Math::vec3(inverseTransform * Math::vec4(point, 1.0f));
                    const Math::vec3 localNormal =
                        Math::normalize(Math::vec3(inverseTransform * Math::vec4(normal, 0.0f)));
                    if (vegetationMode == EditorApplication::VegetationPlacementMode::Tree)
                    {
                        Forest* forest = selectedObject->getComponent<Forest>();
                        if (forest && forest->speciesCount() > 0)
                        {
                            const u32 species = Math::min(app().vegetationPlacementSpecies(),
                                                         forest->speciesCount() - 1);
                            app().recordUndo();
                            if (forest->plant(local, species))
                                app().markDirty();
                        }
                    }
                    else if (Grass* grass = selectedObject->getComponent<Grass>())
                    {
                        app().recordUndo();
                        if (grass->plant(local, localNormal))
                            app().markDirty();
                    }
                    return;
                }
                if (!hit)
                {
                    Ray ray;
                    ray.origin = Math::vec3(nearPoint);
                    ray.direction = direction;
                    hit = app().scene().pickObject(ray);
                }
                EditorApplication::PickedSubmesh& picked = app().pickedSubmesh();
                if (hit)
                {
                    // A plain click restarts the batch, even on its object; only Shift adds. The submesh hit is re-added after the pick so Delete has a target.
                    if (!ImGui::GetIO().KeyShift)
                    {
                        app().submeshSelection().object = 0;
                        app().submeshSelection().indices.clear();
                    }
                    app().selection().select(hit->id());
                    // Only meaningful for the depth-buffer surface point; the ray fallback has no exact position.
                    picked.index = -1;
                    s32 pickedSlot = -1;
                    if (hasSurfacePosition)
                        pickedSlot = Scene::pickSubmeshAtPoint(*hit, surfacePosition, &picked.index);
                    picked.object = picked.index >= 0 ? hit->id() : 0;
                    picked.justPicked = picked.index >= 0;

                    // With Pick Surface the batch IS the selection: a plain click makes it the submesh under the cursor, Shift toggles one.
                    if (picked.index >= 0 && mTool == 5)
                    {
                        EditorApplication::SubmeshSelection& multi = app().submeshSelection();
                        if (multi.object != picked.object)
                        {
                            multi.object = picked.object;
                            multi.indices.clear();
                        }
                        const u32 index = static_cast<u32>(picked.index);
                        const auto found = std::find(multi.indices.begin(), multi.indices.end(), index);
                        if (!ImGui::GetIO().KeyShift)
                        {
                            multi.indices.clear();
                            multi.indices.push_back(index);
                        }
                        else if (found != multi.indices.end())
                            multi.indices.erase(found);
                        else
                            multi.indices.push_back(index);
                    }
                    if ((app().surfaceProbe() || mTool == 5) && hasSurfacePosition)
                    {
                        EditorApplication::PickedSurface& probe = app().pickedSurface();
                        probe.valid = true;
                        probe.position = surfacePosition;
                        probe.normal = surfaceNormal;
                        probe.object = hit->id();
                        probe.submesh = picked.index;
                        probe.materialSlot = pickedSlot;
                        probe.viewDepth =
                            -(mEditorView * Math::vec4(surfacePosition, 1.0f)).z;
                    }
                }
                else
                {
                    app().selection().clear();
                    picked.index = -1;
                    picked.object = 0;
                    picked.justPicked = false;
                }
            }
            else if (Math::abs(direction.y) > 0.0001f)
            {
                const f32 t = -nearPoint.y / direction.y;
                if (t >= 0.0f)
                    app().setCursor3D(Math::vec3(nearPoint) + direction * t);
            }
        }
        // Delete removes the whole Shift-click batch (see EditorApplication::deleteSubmeshSelection()).
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
            ImGui::IsKeyPressed(ImGuiKey_Delete) && !app().submeshSelection().indices.empty())
            app().deleteSubmeshSelection();
        // Escape drops the whole batch.
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
            ImGui::IsKeyPressed(ImGuiKey_Escape) && !app().submeshSelection().indices.empty())
            app().submeshSelection().indices.clear();
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddCircleFilled(navigationCenter, 7.0f, IM_COL32(75, 125, 220, 255));
        draw->AddLine(navigationCenter, zAxis, IM_COL32(80, 220, 100, 255), 3.0f);
        draw->AddLine(navigationCenter, xAxis, IM_COL32(220, 70, 70, 255), 4.0f);
        draw->AddLine(navigationCenter, yAxis, IM_COL32(70, 100, 220, 255), 4.0f);
        // Negative ends hollow, positive ends filled; both click to snap the view onto that axis.
        draw->AddCircleFilled(xAxisNeg, 8.0f, IM_COL32(220, 70, 70, 90));
        draw->AddCircle(xAxisNeg, 8.0f, IM_COL32(220, 70, 70, 255), 0, 2.0f);
        draw->AddCircleFilled(yAxisNeg, 8.0f, IM_COL32(70, 100, 220, 90));
        draw->AddCircle(yAxisNeg, 8.0f, IM_COL32(70, 100, 220, 255), 0, 2.0f);
        draw->AddCircleFilled(zAxisNeg, 8.0f, IM_COL32(80, 220, 100, 90));
        draw->AddCircle(zAxisNeg, 8.0f, IM_COL32(80, 220, 100, 255), 0, 2.0f);
        draw->AddCircleFilled(xAxis, 10.0f, IM_COL32(220, 70, 70, 255));
        draw->AddCircleFilled(yAxis, 10.0f, IM_COL32(70, 100, 220, 255));
        draw->AddCircleFilled(zAxis, 10.0f, IM_COL32(80, 220, 100, 255));
        draw->AddText(ImVec2(xAxis.x - 3.0f, xAxis.y - 7.0f), IM_COL32(20, 20, 20, 255), "X");
        draw->AddText(ImVec2(yAxis.x - 3.0f, yAxis.y - 7.0f), IM_COL32(20, 20, 20, 255), "Y");
        draw->AddText(ImVec2(zAxis.x - 3.0f, zAxis.y - 7.0f), IM_COL32(20, 20, 20, 255), "Z");
    }
    else
        ImGui::TextDisabled("Viewport unavailable");
}

} // namespace Radion
