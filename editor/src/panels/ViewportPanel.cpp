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
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_decompose.hpp>
#include <imgui.h>

namespace Radion
{

ViewportPanel::ViewportPanel(EditorApplication& app) : EditorPanel("Viewport", app)
{
    // Picks up wherever the free-fly camera was left last session, instead
    // of always starting from the same hardcoded pose - see
    // EditorSettings::cameraPosition's own comment.
    const EditorSettings& settings = app.settings();
    mCameraPosition = settings.cameraPosition;
    mOrbitTarget = settings.cameraOrbitTarget;
    mOrbitDistance = settings.cameraOrbitDistance;
    mOrbitYaw = settings.cameraOrbitYaw;
    mOrbitPitch = settings.cameraOrbitPitch;
    mPerspective = settings.cameraPerspective;
    mGrid = settings.viewportGrid;
    mSnap = settings.viewportSnap;
    mTool = glm::clamp(settings.viewportTool, 0, 5);
    mPointerNavigation = glm::clamp(settings.viewportNavigationTool, 0, 2);
}

void ViewportPanel::focusOnObject(const GameObject& object, s32 submeshIndex)
{
    glm::vec3 center = object.globalPosition();
    f32 radius = 1.0f;
    if (MeshRenderer* renderer = object.getComponent<MeshRenderer>())
    {
        if (const Mesh* mesh = Assets().getMesh(renderer->mesh()))
        {
            // The one submesh the Inspector's own focus icon asked for,
            // rather than the whole mesh - same idea as the cyan pick box,
            // just aimed at the camera instead of drawn.
            const AABB localBounds =
                (submeshIndex >= 0 && static_cast<usize>(submeshIndex) < mesh->submeshes.size())
                    ? mesh->submeshes[static_cast<usize>(submeshIndex)].bounds
                    : mesh->bounds;
            const AABB worldBounds = transformAABB(localBounds, object.globalTransform());
            if (!worldBounds.empty())
            {
                center = worldBounds.center();
                radius = glm::max(worldBounds.radius(), 0.05f);
            }
        }
    }

    Camera* camera = app().scene().activeCamera();
    const f32 fieldOfView = camera ? camera->fieldOfView() : 60.0f;
    const f32 fitDistance = radius / glm::sin(glm::radians(fieldOfView * 0.5f)) * 1.3f;
    mOrbitTarget = center;
    mOrbitDistance = glm::max(fitDistance, 0.5f);
}

void ViewportPanel::updateNavigation()
{
    ImGuiIO& io = ImGui::GetIO();
    // Navigation feel lives in EditorSettings ("View > Camera Settings...")
    // and is edited there; ViewportPanel only consumes it here.
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
        mOrbitPitch = glm::clamp(mOrbitPitch, -1.5f, 1.5f);
    }
    // Pan runs along the camera's own axes, not the world's. A hardcoded
    // world up reads as a zoom the moment the view looks straight down:
    // moving the target along world Y is then moving it toward the camera.
    const glm::vec3 navRight(glm::cos(mOrbitYaw), 0.0f, glm::sin(mOrbitYaw));
    const glm::vec3 navForward(glm::sin(mOrbitYaw) * glm::cos(mOrbitPitch), glm::sin(mOrbitPitch),
                               -glm::cos(mOrbitYaw) * glm::cos(mOrbitPitch));
    const glm::vec3 navUp = glm::normalize(glm::cross(navRight, navForward));
    if (mPanning)
    {
        const f32 scale = mOrbitDistance * 0.001f;
        mOrbitTarget -= navRight * (io.MouseDelta.x * scale);
        mOrbitTarget += navUp * (io.MouseDelta.y * scale);
    }
    if (hovered && io.MouseWheel != 0.0f)
    {
        mOrbitDistance -= io.MouseWheel * mOrbitDistance * 0.15f;
        mOrbitDistance = glm::clamp(mOrbitDistance, nav.cameraMinView, nav.cameraMaxView);
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
            mOrbitDistance = glm::clamp(mOrbitDistance, nav.cameraMinView, nav.cameraMaxView);
        }
        else
        {
            const f32 panScale = mOrbitDistance * 0.001f;
            mOrbitTarget -= navRight * (io.MouseDelta.x * panScale);
            mOrbitTarget += navUp * (io.MouseDelta.y * panScale);
        }
    }

    const glm::vec3 forward(glm::sin(mOrbitYaw) * glm::cos(mOrbitPitch), glm::sin(mOrbitPitch),
                            -glm::cos(mOrbitYaw) * glm::cos(mOrbitPitch));
    const glm::vec3 right(glm::cos(mOrbitYaw), 0.0f, glm::sin(mOrbitYaw));
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
                                   : mOrbitTarget + glm::vec3(0.0f, mOrbitDistance, 0.0f);
}

void ViewportPanel::drawSceneGizmos(GameObject& object)
{
    const glm::vec3 position = object.globalPosition();
    const glm::vec3 forward = object.forward();
    const glm::vec3 right = object.right();
    const glm::vec3 up = object.up();

    if (Camera* camera = object.getComponent<Camera>())
    {
        const f32 nearDistance = 0.2f;
        const f32 farDistance = glm::min(camera->farPlane(), 3.0f);
        const f32 halfHeight =
            camera->projectionMode() == CameraProjection::Perspective
                ? glm::tan(glm::radians(camera->fieldOfView() * 0.5f)) * farDistance
                : camera->orthographicSize() * 0.5f;
        const f32 halfWidth = halfHeight * camera->aspect();
        const glm::vec3 nearCenter = position + forward * nearDistance;
        const glm::vec3 farCenter = position + forward * farDistance;
        const glm::vec3 nearCorners[] = {
            nearCenter - right * halfWidth * nearDistance / farDistance -
                up * halfHeight * nearDistance / farDistance,
            nearCenter + right * halfWidth * nearDistance / farDistance -
                up * halfHeight * nearDistance / farDistance,
            nearCenter + right * halfWidth * nearDistance / farDistance +
                up * halfHeight * nearDistance / farDistance,
            nearCenter - right * halfWidth * nearDistance / farDistance +
                up * halfHeight * nearDistance / farDistance};
        const glm::vec3 farCorners[] = {farCenter - right * halfWidth - up * halfHeight,
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
        const glm::vec3 color = light->color();
        const Color debugColor = Color::fromRGBFloat(color.r, color.g, color.b);
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
            const glm::vec3 pointPosition = point->globalPosition();
            DebugDraw().circle(pointPosition, glm::vec3(1.0f, 0.0f, 0.0f),
                               glm::vec3(0.0f, 1.0f, 0.0f), 0.35f, 12, Color::Orange);
            if (i > 0)
            {
                GameObject* previous = road->point(i - 1);
                if (previous)
                    DebugDraw().line(previous->globalPosition(), pointPosition, Color::Orange);
            }
        }
    }

    // An "empty" - no MeshRenderer, no Light, nothing else drawing anything
    // of its own - used purely as a marker (a spawn point, a rope anchor)
    // was otherwise invisible in the viewport: nothing to click, nothing to
    // see, no way to tell it apart from an object misplaced at the origin.
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
            const glm::vec3 world = waypoints->worldPosition(i);
            // XZ, so the ring lies flat on the ground the point marks - the
            // XY plane would stand it up on edge, facing sideways.
            DebugDraw().circle(world, glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f),
                               waypoints->point(i).radius, 16,
                               selected ? Color::Yellow : Color::Green);
            // The links, drawn once per pair: they are stored only on the
            // lower index, so walking every node's own list covers each edge
            // exactly once.
            for (u32 link : waypoints->point(i).links)
                if (link < count)
                    DebugDraw().line(world, waypoints->worldPosition(link), Color(60, 200, 120, 200));
        }
    }

    if (NavMeshSurface* surface = object.getComponent<NavMeshSurface>())
    {
        const std::vector<glm::vec3>& triangles = surface->navMesh().debugTriangles();
        const glm::vec3 lift(0.0f, 0.05f, 0.0f);
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

// mTool 2/3/4 = Move/Rotate/Scale (see the toolbar in onImGui). Nothing is
// drawn for Select/3D Cursor or when nothing is selected.

// Walks the whole scene and keeps whatever projects inside the box. Tested
// on the object's ORIGIN, not its bounds: a rubber band over a level would
// otherwise catch the level itself every time, since its box contains
// everything else.
static void collectInRect(GameObject& object, const glm::mat4& viewProjection,
                          const glm::vec2& imageMin, const glm::vec2& imageSize,
                          const glm::vec2& rectMin, const glm::vec2& rectMax,
                          std::vector<u64>& out)
{
    glm::vec4 clip = viewProjection * glm::vec4(object.globalPosition(), 1.0f);
    if (clip.w > 0.0f)
    {
        clip /= clip.w;
        const glm::vec2 screen(imageMin.x + (clip.x * 0.5f + 0.5f) * imageSize.x,
                               imageMin.y + (1.0f - (clip.y * 0.5f + 0.5f)) * imageSize.y);
        if (clip.z >= -1.0f && clip.z <= 1.0f && screen.x >= rectMin.x && screen.x <= rectMax.x &&
            screen.y >= rectMin.y && screen.y <= rectMax.y)
            out.push_back(object.id());
    }
    for (usize i = 0; i < object.childCount(); ++i)
        collectInRect(*object.child(i), viewProjection, imageMin, imageSize, rectMin, rectMax, out);
}

void ViewportPanel::selectInRect(const glm::vec2& min, const glm::vec2& max, bool add)
{
    const glm::vec2 imageSize = mImageMax - mImageMin;
    if (imageSize.x <= 0.0f || imageSize.y <= 0.0f)
        return;

    std::vector<u64> hits;
    const glm::mat4 viewProjection = mEditorProjection * mEditorView;
    for (usize i = 0; i < app().scene().root().childCount(); ++i)
        collectInRect(*app().scene().root().child(i), viewProjection, mImageMin, imageSize, min,
                      max, hits);

    if (!add)
        app().selection().clear();
    for (u64 id : hits)
        if (!app().selection().isSelected(id))
            app().selection().toggle(id);
}

void ViewportPanel::selectSubmeshesInRect(GameObject& object, const glm::vec2& min,
                                          const glm::vec2& max, bool subtract)
{
    const glm::vec2 imageSize = mImageMax - mImageMin;
    if (imageSize.x <= 0.0f || imageSize.y <= 0.0f)
        return;

    MeshRenderer* renderer = object.getComponent<MeshRenderer>();
    const Mesh* mesh = renderer ? AssetManager::getSingleton().getMesh(renderer->mesh()) : nullptr;
    if (!mesh)
        return;

    // The drag rectangle unprojected into the world is a FRUSTUM, and it has
    // to be tested as one: an AABB drawn around that frustum spans from the
    // near plane to the far plane in every direction, so it swallows the
    // whole level and every submesh in it comes back selected.
    const glm::mat4 inverseViewProjection = glm::inverse(mEditorProjection * mEditorView);
    const f32 ndcMinX = (min.x - mImageMin.x) / imageSize.x * 2.0f - 1.0f;
    const f32 ndcMaxX = (max.x - mImageMin.x) / imageSize.x * 2.0f - 1.0f;
    // Screen Y grows downward, NDC Y grows upward: the rectangle's top edge
    // is the larger NDC value.
    const f32 ndcMaxY = 1.0f - (min.y - mImageMin.y) / imageSize.y * 2.0f;
    const f32 ndcMinY = 1.0f - (max.y - mImageMin.y) / imageSize.y * 2.0f;

    glm::vec3 corner[8];
    u32 written = 0;
    for (u32 i = 0; i < 8; ++i)
    {
        const glm::vec4 ndc((i & 1) ? ndcMaxX : ndcMinX, (i & 2) ? ndcMaxY : ndcMinY,
                            (i & 4) ? 1.0f : -1.0f, 1.0f);
        const glm::vec4 world = inverseViewProjection * ndc;
        if (glm::abs(world.w) < 1e-6f)
            return;
        corner[i] = glm::vec3(world) / world.w;
        ++written;
    }
    if (written != 8)
        return;

    // Inward-facing planes, each built from three corners of one face.
    // Bit layout of the index above: 1 = maxX, 2 = maxY, 4 = far.
    struct Plane
    {
        glm::vec3 normal;
        f32 distance;
    };
    const auto makePlane = [](const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                              const glm::vec3& inside)
    {
        glm::vec3 normal = glm::cross(b - a, c - a);
        const f32 length = glm::length(normal);
        Plane plane{glm::vec3(0.0f, 1.0f, 0.0f), 0.0f};
        if (length < 1e-8f)
            return plane;
        normal /= length;
        // Point it at the side the frustum's own interior is on, so one sign
        // test means the same thing for all six.
        if (glm::dot(normal, inside - a) < 0.0f)
            normal = -normal;
        plane.normal = normal;
        plane.distance = -glm::dot(normal, a);
        return plane;
    };

    glm::vec3 centre(0.0f);
    for (const glm::vec3& point : corner)
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
    // A subtract pass over a different object's selection has nothing to
    // take away, and must not silently adopt this object either.
    if (selection.object != object.id())
    {
        if (subtract)
            return;
        selection.object = object.id();
        selection.indices.clear();
    }

    // The CPU-side copy, when the mesh was imported this session: its actual
    // vertices are what makes the test tight. A bounding box is a poor stand-
    // in for a long diagonal or an L-shaped piece - the box overlaps the
    // rectangle from far outside it - so the box is only used to reject
    // cheaply, and anything it lets through is confirmed vertex by vertex.
    const MeshData* meshData = app().importedMeshData(renderer->mesh());
    const glm::mat4 transform = object.globalTransform();
    for (u32 i = 0; i < static_cast<u32>(mesh->submeshes.size()); ++i)
    {
        const AABB bounds = transformAABB(mesh->submeshes[i].bounds, transform);
        // Rejected as soon as the box sits entirely behind any one plane -
        // tested against the corner furthest along that plane's normal, the
        // last one to cross it.
        bool outside = false;
        for (const Plane& plane : planes)
        {
            const glm::vec3 furthest(plane.normal.x >= 0.0f ? bounds.max.x : bounds.min.x,
                                     plane.normal.y >= 0.0f ? bounds.max.y : bounds.min.y,
                                     plane.normal.z >= 0.0f ? bounds.max.z : bounds.min.z);
            if (glm::dot(plane.normal, furthest) + plane.distance < 0.0f)
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
                glm::min<usize>(submesh.indexOffset + submesh.indexCount, meshData->indices.size());
            for (usize index = submesh.indexOffset; index < end && !anyVertexInside; ++index)
            {
                const u32 vertex = meshData->indices[index];
                if (vertex >= meshData->positions.size())
                    continue;
                const glm::vec3 world =
                    glm::vec3(transform * glm::vec4(meshData->positions[vertex], 1.0f));
                bool insideAll = true;
                for (const Plane& plane : planes)
                    if (glm::dot(plane.normal, world) + plane.distance < 0.0f)
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

void ViewportPanel::drawTransformGizmo(const glm::vec2& imageMin, const glm::vec2& imageSize)
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

    // A picked waypoint takes the gizmo over: the point is not a GameObject,
    // so it has only a position to move - rotate/scale stay on the object.
    Waypoints* waypoints = selected->getComponent<Waypoints>();
    const s32 waypointIndex = app().selectedWaypoint();
    if (waypoints && operation == ImGuizmo::TRANSLATE && waypointIndex >= 0 &&
        waypointIndex < static_cast<s32>(waypoints->pointCount()))
    {
        const u32 index = static_cast<u32>(waypointIndex);
        glm::mat4 pointTransform =
            glm::translate(glm::mat4(1.0f), waypoints->worldPosition(index));
        float pointSnap[3] = {1.0f, 1.0f, 1.0f};
        ImGuizmo::Manipulate(glm::value_ptr(mEditorView), glm::value_ptr(mEditorProjection),
                             operation, ImGuizmo::WORLD, glm::value_ptr(pointTransform), nullptr,
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
        // The gizmo works in world space; the point is stored local to its
        // owner, so it goes back through the inverse of that transform.
        const glm::vec3 world = glm::vec3(pointTransform[3]);
        const glm::vec3 local =
            glm::vec3(glm::inverse(selected->globalTransform()) * glm::vec4(world, 1.0f));
        waypoints->setPointPosition(index, local);
        app().markDirty();
        return;
    }

    glm::mat4 transform = selected->globalTransform();
    const float snapAmount = mTool == 2 ? 1.0f : mTool == 3 ? 15.0f : 0.1f;
    float snapValues[3] = {snapAmount, snapAmount, snapAmount};

    ImGuizmo::Manipulate(glm::value_ptr(mEditorView), glm::value_ptr(mEditorProjection), operation,
                         ImGuizmo::LOCAL, glm::value_ptr(transform), nullptr,
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

    glm::vec3 translation, scale, skew;
    glm::vec4 perspective;
    glm::quat rotation;
    if (!glm::decompose(transform, scale, rotation, translation, skew, perspective))
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
        const glm::vec3 parentScale =
            selected->parent() ? selected->parent()->globalScale() : glm::vec3(1.0f);
        selected->setScale(scale / glm::max(parentScale, glm::vec3(0.0001f)));
    }
    app().markDirty();
}

// FK bone (rotate, local to its parent) or IK chain target (translate,
// world - IKChain::target is already world space, see Skeleton.h) instead of
// the selected object's own transform. No-op unless AnimationPanel armed
// EditorApplication::animationPoseTarget() and the selected object still has
// a bound, pose-edit-mode Animator - a stale target from a since-deselected
// object draws nothing rather than a gizmo pointed at whatever is selected
// now.
void ViewportPanel::drawBonePoseGizmo(const glm::vec2& imageMin, const glm::vec2& imageSize)
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
    const glm::mat4 ownerTransform = selected->globalTransform();

    if (target.bone >= 0 && static_cast<u32>(target.bone) < skeleton->boneCount())
    {
        const u32 bone = static_cast<u32>(target.bone);
        glm::mat4 world = ownerTransform * animator->globalPose()[bone];

        ImGuizmo::Manipulate(glm::value_ptr(mEditorView), glm::value_ptr(mEditorProjection),
                             ImGuizmo::ROTATE, ImGuizmo::LOCAL, glm::value_ptr(world));
        if (!ImGuizmo::IsUsing())
            return;

        const s32 parent = skeleton->bone(bone).parent;
        const glm::mat4 parentWorld =
            parent >= 0 ? ownerTransform * animator->globalPose()[static_cast<u32>(parent)]
                        : ownerTransform;
        const glm::mat4 local = glm::inverse(parentWorld) * world;

        glm::vec3 translation, scale, skew;
        glm::vec4 perspective;
        glm::quat rotation;
        if (!glm::decompose(local, scale, rotation, translation, skew, perspective))
            return;

        LocalPose pose = animator->localPose()[bone];
        pose.rotation = glm::normalize(rotation);
        animator->setBoneLocalPose(bone, pose);
    }
    else if (target.ikChain >= 0 && static_cast<u32>(target.ikChain) < animator->ikChainCount())
    {
        IKChain* chain = animator->ikChain(static_cast<u32>(target.ikChain));
        if (!chain)
            return;
        glm::mat4 world = glm::translate(glm::mat4(1.0f), chain->target);

        ImGuizmo::Manipulate(glm::value_ptr(mEditorView), glm::value_ptr(mEditorProjection),
                             ImGuizmo::TRANSLATE, ImGuizmo::WORLD, glm::value_ptr(world));
        if (!ImGuizmo::IsUsing())
            return;

        chain->target = glm::vec3(world[3]);
    }
}

void ViewportPanel::onImGui()
{
    // Required once per frame (ImGuizmo.h's own comment: "call BeginFrame
    // right after ImGui_XXXX_NewFrame()") - without it the gizmo's internal
    // per-frame state never resets, which is why it drew in the wrong place
    // and ate no input.
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
    // Drives PostProcessStack::enabled directly - the exact same flag
    // Settings > Post Process's own "Enabled" checkbox edits, not a second,
    // separate switch that also had to be on. Two independent gates on one
    // feature (this button used to be its own bool, defaulting off, while
    // Settings' defaulted on) meant Settings could show Enabled checked and
    // nothing would happen until this button was also found and clicked.
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
            // The Inspector's per-submesh focus icon routes through the same
            // request as Hierarchy's whole-object one, plus whichever
            // submesh pickedSubmesh() already names for this same object -
            // set together, right before the request, by whoever asked.
            const EditorApplication::PickedSubmesh& picked = app().pickedSubmesh();
            focusOnObject(*target, picked.object == focusId ? picked.index : -1);
        }
    }

    updateNavigation();
    const bool navigating =
        mOrbiting || mPanning || mLooking || mPointerNavigationActive || mNavigationGizmoActive;
    app().engine().setProbeCaptureDeferred(navigating);

    // Kept live rather than only on close: EditorSettings::save() only runs
    // from the destructor (a clean exit), and a pose that only updates then
    // is a pose a crash throws away. Plain floats, copied every frame - cheap
    // next to everything else this panel already does per frame.
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
    size.x = glm::max(size.x, 1.0f);
    size.y = glm::max(size.y, 1.0f);
    const u32 width = static_cast<u32>(size.x);
    const u32 height = static_cast<u32>(size.y);

    // Only the live view submits the scene (EditorApplication::ViewMode) -
    // while Game is live this panel keeps showing the last frame it drew
    // rather than paying for a second full render of everything. The gizmos
    // are gated on the same flag and not merely the render: DebugDraw's list
    // is frame-wide, so emitting into it while the Game view is the one that
    // consumes it would draw the editor's grid and selection outlines over
    // the game picture.
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
    if (GameObject* selected = live ? app().selection().resolve(app().scene()) : nullptr)
    {
        if (MeshRenderer* renderer = selected->getComponent<MeshRenderer>())
        {
            if (renderer->mesh().valid())
            {
                // The outline hull is built from the mesh's raw, un-skinned
                // vertices (DebugDraw3D.cpp's outline shader has no skin
                // palette at all) - fine for a static mesh, but a skinned one
                // posed away from bind pose leaves the hull tracing a shape
                // that no longer matches what is actually on screen, cutting
                // across the real silhouette instead of framing it. A world
                // AABB has no pose to get wrong, so it is what stands in
                // until the outline itself learns to skin (the proper fix -
                // Lumix's own selection highlight sidesteps this a different
                // way, a screen-space pass over the already-posed render
                // instead of a second raw-geometry draw).
                const Mesh* mesh = Assets().getMesh(renderer->mesh());
                constexpr u32 maxOutlineIndices = 300000;
                if (mesh && (mesh->isSkinned() || mesh->indexCount > maxOutlineIndices))
                    DebugDraw().box(transformAABB(mesh->bounds, selected->globalTransform()),
                                    Color::Orange);
                else
                    DebugDraw().outline(renderer->mesh(), selected->globalTransform(),
                                        Color::Orange);

                // Every submesh box at once, for reading how the mesh is
                // split - see EditorApplication::showSubmeshBounds().
                if (mesh && app().showSubmeshBounds())
                {
                    for (const SubMesh& submesh : mesh->submeshes)
                        DebugDraw().box(transformAABB(submesh.bounds, selected->globalTransform()),
                                        Color::Gray);
                }

                // The one submesh the last click landed on, boxed inside the
                // whole-object outline - on a mesh with many pieces (Sponza's
                // ~30) the object outline alone says nothing about which
                // material slot the Inspector just jumped to. Cyan, not the
                // selection's orange, so the two never read as one shape.
                const EditorApplication::PickedSubmesh& picked = app().pickedSubmesh();
                if (mesh && picked.index >= 0 && picked.object == selected->id() &&
                    static_cast<usize>(picked.index) < mesh->submeshes.size())
                {
                    DebugDraw().box(transformAABB(mesh->submeshes[picked.index].bounds,
                                                  selected->globalTransform()),
                                    Color::Cyan);
                }

                // Every submesh Shift-click has accumulated into the batch -
                // yellow, so a set of several reads as distinct from both the
                // single cyan "last pointed at" one above and the whole-object
                // orange outline.
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

            const glm::mat4 bodyTransform = glm::translate(glm::mat4(1.0f), selected->globalPosition()) *
                                            glm::mat4_cast(selected->globalRotation());
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
            const glm::mat4 colliderTransform =
                glm::translate(glm::mat4(1.0f), selected->globalPosition()) *
                glm::mat4_cast(selected->globalRotation());
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
    }
    const f32 cursorRadius = glm::max(0.1f, mOrbitDistance * 0.025f);
    const glm::vec3 forward(glm::sin(mOrbitYaw) * glm::cos(mOrbitPitch), glm::sin(mOrbitPitch),
                            -glm::cos(mOrbitYaw) * glm::cos(mOrbitPitch));
    const glm::vec3 cameraRight = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
    const glm::vec3 cameraUp = glm::normalize(glm::cross(cameraRight, forward));
    if (live)
        DebugDraw().cursor3D(app().cursor3D(), cameraRight, cameraUp, cursorRadius);
    // The observer's own view: a RenderView built straight from the orbit
    // state, never touching the scene's real Camera/GameObject at all - see
    // RenderView's own comment (Engine.h) for why this replaced borrowing
    // and restoring the game camera's every field around each render.
    const f32 aspect = static_cast<f32>(width) / static_cast<f32>(height);
    const EditorSettings& editorCamera = app().settings();
    const f32 nearPlane = editorCamera.cameraNearPlane;
    const f32 farPlane = glm::max(editorCamera.cameraFarPlane, nearPlane + 0.001f);
    RenderView view;
    view.position = mCameraPosition;
    if (mPerspective)
    {
        view.view = glm::lookAt(mCameraPosition, mOrbitTarget, glm::vec3(0.0f, 1.0f, 0.0f));
        view.fieldOfView = 60.0f;
        view.nearPlane = nearPlane;
        view.aspect = aspect;
        view.projection =
            glm::perspective(glm::radians(view.fieldOfView), aspect, view.nearPlane, farPlane);
    }
    else
    {
        view.view = glm::lookAt(mCameraPosition, mOrbitTarget, glm::vec3(0.0f, 0.0f, -1.0f));
        view.nearPlane = nearPlane;
        view.aspect = aspect;
        const f32 halfHeight = mOrbitDistance * 0.5f;
        const f32 halfWidth = halfHeight * aspect;
        view.projection =
            glm::ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, view.nearPlane, farPlane);
    }
    RenderTextureSettings settings;
    settings.outputIndex = 0;
    // The scene editor must remain pixel-stable while orbiting, selecting,
    // and drawing gizmos. The Game panel uses its separate output stream and
    // keeps TAA enabled, so this does not change game rendering quality.
    settings.temporalAA = false;
    const bool renderPostProcess = app().engine().postProcess().enabled && !mFastRender;
    const EditorSettings& preview = app().settings();
    // Fast render: every optional pass off at once. What makes a heavy scene
    // (Bistro) navigable in the editor - the frame cost lives in these
    // passes, not in the geometry itself, so the shapes stay readable while
    // the per-frame work collapses.
    settings.shadows = preview.previewShadows && !mFastRender;
    settings.planarReflections = preview.previewPlanarReflections && !mFastRender;
    settings.postProcess = renderPostProcess;
    settings.ambientOcclusion = renderPostProcess && preview.previewSSAO;
    settings.volumetrics = renderPostProcess && preview.previewVolumetrics;
    settings.lensFlares = renderPostProcess && preview.previewLensFlares;
    settings.previewOcclusionCulling = preview.previewOcclusionCulling;
    const f32 previewScale = navigating ? preview.previewNavigationScale : 1.0f;
    const u32 previewWidth = glm::max(1u, static_cast<u32>(width * previewScale));
    const u32 previewHeight = glm::max(1u, static_cast<u32>(height * previewScale));
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
        mImageMin = glm::vec2(imageMin.x, imageMin.y);
        mImageMax = glm::vec2(imageMin.x + size.x, imageMin.y + size.y);

        // Picture-in-picture camera preview, without rendering anything new
        // for it: GamePanel already renders scene.activeCamera() every frame
        // it draws (its own outputIndex=1 slot) - only while the selection is
        // that same camera does the already-published texture mean
        // anything, so a selected-but-not-active camera shows nothing here
        // rather than some other camera's view under its label.
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
            drawBonePoseGizmo(glm::vec2(imageMin.x, imageMin.y), glm::vec2(size.x, size.y));
        else
            drawTransformGizmo(glm::vec2(imageMin.x, imageMin.y), glm::vec2(size.x, size.y));

        // ImGuizmo's own CanActivate() (ImGuizmo.cpp) refuses to start a
        // drag whenever ImGui::IsAnyItemHovered() is true - and that check
        // looks at *this frame or the previous one*
        // (g.HoveredId || g.HoveredIdPreviousFrame), so merely submitting
        // the InvisibleButton after the gizmo was not enough: hovering the
        // viewport for even one frame before the click already leaves
        // HoveredIdPreviousFrame pointing at it next frame. The only fix is
        // to never let this InvisibleButton register as hovered at all
        // while the gizmo wants that same pixel - skip it entirely
        // (Dummy() reserves the identical layout space with no interaction)
        // whenever IsOver()/IsUsing() says the gizmo owns this hover.
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
        // InvisibleButton/Dummy already advanced the layout cursor to the
        // image end. Repositioning it there again triggers Dear ImGui's
        // "SetCursorScreenPos extends window boundaries" diagnostic.
        const ImVec2 navigationCenter(imageMin.x + size.x - 55.0f, imageMin.y + 55.0f);
        const ImVec2 mouse = ImGui::GetMousePos();
        const glm::vec3 navigationForward(glm::sin(mOrbitYaw) * glm::cos(mOrbitPitch),
                                          glm::sin(mOrbitPitch),
                                          -glm::cos(mOrbitYaw) * glm::cos(mOrbitPitch));
        glm::vec3 navigationRight = glm::cross(navigationForward, glm::vec3(0.0f, 1.0f, 0.0f));
        if (glm::dot(navigationRight, navigationRight) < 0.001f)
            navigationRight = glm::vec3(1.0f, 0.0f, 0.0f);
        else
            navigationRight = glm::normalize(navigationRight);
        const glm::vec3 navigationUp =
            glm::normalize(glm::cross(navigationRight, navigationForward));
        constexpr f32 axisLength = 30.0f;
        const glm::vec3 worldX(1.0f, 0.0f, 0.0f);
        const glm::vec3 worldY(0.0f, 1.0f, 0.0f);
        const glm::vec3 worldZ(0.0f, 0.0f, 1.0f);
        const ImVec2 xAxis(navigationCenter.x + glm::dot(worldX, navigationRight) * axisLength,
                           navigationCenter.y - glm::dot(worldX, navigationUp) * axisLength);
        const ImVec2 yAxis(navigationCenter.x + glm::dot(worldY, navigationRight) * axisLength,
                           navigationCenter.y - glm::dot(worldY, navigationUp) * axisLength);
        const ImVec2 zAxis(navigationCenter.x + glm::dot(worldZ, navigationRight) * axisLength,
                           navigationCenter.y - glm::dot(worldZ, navigationUp) * axisLength);
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
            // A ball click snaps to that axis view (camera moved onto the
            // axis, looking back at the orbit target); the pitch stops just
            // short of +-90 degrees so lookAt's fixed up vector stays sound.
            const struct
            {
                ImVec2 ball;
                f32 yaw;
                f32 pitch;
            } views[6] = {
                {xAxis, -glm::half_pi<f32>(), 0.0f},
                {xAxisNeg, glm::half_pi<f32>(), 0.0f},
                {yAxis, mOrbitYaw, -1.5f},
                {yAxisNeg, mOrbitYaw, 1.5f},
                {zAxis, 0.0f, 0.0f},
                {zAxisNeg, glm::pi<f32>(), 0.0f},
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
                mNavigationGizmoStartMouse = glm::vec2(mouse.x, mouse.y);
                mNavigationGizmoStartYaw = mOrbitYaw;
                mNavigationGizmoStartPitch = mOrbitPitch;
                navigationClicked = true;
            }
        }
        if (mNavigationGizmoActive && ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            mOrbitYaw =
                mNavigationGizmoStartYaw - (mouse.x - mNavigationGizmoStartMouse.x) * 0.005f;
            mOrbitPitch = glm::clamp(mNavigationGizmoStartPitch -
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
            const glm::mat4 inverseViewProjection =
                glm::inverse(mEditorProjection * mEditorView);
            glm::vec4 nearPoint = inverseViewProjection * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
            glm::vec4 farPoint = inverseViewProjection * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
            nearPoint /= nearPoint.w;
            farPoint /= farPoint.w;
            Ray ray;
            ray.origin = glm::vec3(nearPoint);
            ray.direction = glm::normalize(glm::vec3(farPoint - nearPoint));
            glm::vec3 hit;
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
                                hit, mTerrainBrushRadius, glm::min(1.0f, amount));
                            break;
                        default:
                            terrainPainted = selectedTerrain->flatten(
                                hit, mTerrainBrushRadius, 0.0f, glm::min(1.0f, amount));
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
        // Shift normally means "place the 3D cursor", but with the Pick
        // Surface tool it is the accumulate modifier for submesh selection -
        // both claiming it is what stopped a Shift-drag from ever starting a
        // rectangle there.
        const bool clickToPlaceCursor = mTool == 1 || (ImGui::GetIO().KeyShift && mTool != 5);
        // Select is the plain click case: place-cursor (Shift or the 3D
        // Cursor tool) and the gizmo dragging the object both already claim
        // left-click for their own thing, so this only fires when neither
        // does - same guard clickToPlaceCursor's own block already used.
        const bool clickToSelect =
            (mTool == 0 || mTool == 5) && !ImGuizmo::IsOver() && !ImGuizmo::IsUsing();
        // Rubber band: starts on a plain left-press with the Select tool and
        // only becomes a box once the mouse has actually travelled, so an
        // ordinary click still picks a single object through the path below.
        // Shift is the accumulate modifier throughout the Pick Surface tool:
        // Shift-click adds one submesh, Shift-drag adds every submesh the
        // rectangle covers. Picking a roof's hundreds of pieces one click at
        // a time is what the rectangle replaces.
        GameObject* submeshRectTarget = app().selection().resolve(app().scene());
        if (submeshRectTarget && !submeshRectTarget->getComponent<MeshRenderer>())
            submeshRectTarget = nullptr;
        if (imageHovered && !navigationClicked && !mPointerNavigationActive && !terrainPainted &&
            clickToSelect && !clickToPlaceCursor && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            mSubmeshRectSelecting =
                mTool == 5 && ImGui::GetIO().KeyShift && submeshRectTarget != nullptr;
            mRectSelecting = !mSubmeshRectSelecting;
            mRectStart = glm::vec2(mouse.x, mouse.y);
        }
        if (mSubmeshRectSelecting)
        {
            const glm::vec2 current(mouse.x, mouse.y);
            const glm::vec2 rectMin(glm::min(mRectStart.x, current.x),
                                    glm::min(mRectStart.y, current.y));
            const glm::vec2 rectMax(glm::max(mRectStart.x, current.x),
                                    glm::max(mRectStart.y, current.y));
            const bool dragged = (rectMax.x - rectMin.x) > 4.0f || (rectMax.y - rectMin.y) > 4.0f;
            // Ctrl alongside Shift turns the rectangle into a subtract - red
            // rather than orange, so which one it is is visible while still
            // dragging instead of only in the result.
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
            // Only a real drag takes over the click: a Shift-press that never
            // travels has to reach the pick below, which is what adds one
            // submesh at a time. Returning unconditionally here is what would
            // silently break Shift-click while Shift-drag worked.
            if (dragged)
                return;
        }
        if (mRectSelecting)
        {
            const glm::vec2 current(mouse.x, mouse.y);
            const glm::vec2 rectMin(glm::min(mRectStart.x, current.x),
                                    glm::min(mRectStart.y, current.y));
            const glm::vec2 rectMax(glm::max(mRectStart.x, current.x),
                                    glm::max(mRectStart.y, current.y));
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
            const glm::mat4 inverseViewProjection = glm::inverse(mEditorProjection * mEditorView);
            glm::vec4 nearPoint = inverseViewProjection * glm::vec4(x, y, -1.0f, 1.0f);
            glm::vec4 farPoint = inverseViewProjection * glm::vec4(x, y, 1.0f, 1.0f);
            nearPoint /= nearPoint.w;
            farPoint /= farPoint.w;
            const glm::vec3 direction = glm::normalize(glm::vec3(farPoint - nearPoint));

            if (clickToSelect)
            {
                // The depth buffer already knows exactly what's under the
                // cursor - reading it back and looking up which object's box
                // contains that point beats a ray-vs-AABB test on meshes
                // whose box is a poor fit for their actual shape (a rock,
                // say): the ray can cross the box from an angle no triangle
                // there faces, picking the object even though nothing visible
                // was actually clicked. Only fall back to the ray when there
                // is no surface under the cursor at all (sky, empty space).
                GameObject* hit = nullptr;
                glm::vec3 surfacePosition, surfaceNormal;
                const bool hasSurfacePosition =
                    mOutput.valid() &&
                    Scene::pickSurface(mOutput.depth, mOutput.width, mOutput.height,
                                       mouse.x - imageMin.x, mouse.y - imageMin.y, size.x, size.y,
                                       glm::inverse(mEditorProjection), glm::inverse(mEditorView),
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
                    glm::vec3 pointPosition = hasSurfacePosition ? surfacePosition : app().cursor3D();
                    if (!hasSurfacePosition && glm::abs(direction.y) > 0.0001f)
                    {
                        const f32 t = -nearPoint.y / direction.y;
                        if (t >= 0.0f)
                            pointPosition = glm::vec3(nearPoint) + direction * t;
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
                    glm::vec3 point = hasSurfacePosition ? surfacePosition : app().cursor3D();
                    glm::vec3 normal(0.0f, 1.0f, 0.0f);
                    if (hasSurfacePosition)
                        normal = surfaceNormal;
                    else if (glm::abs(direction.y) > 0.0001f)
                    {
                        const f32 t = -nearPoint.y / direction.y;
                        if (t >= 0.0f)
                            point = glm::vec3(nearPoint) + direction * t;
                    }
                    const glm::mat4 inverseTransform = glm::inverse(selectedObject->globalTransform());
                    const glm::vec3 local = glm::vec3(inverseTransform * glm::vec4(point, 1.0f));
                    const glm::vec3 localNormal =
                        glm::normalize(glm::vec3(inverseTransform * glm::vec4(normal, 0.0f)));
                    if (vegetationMode == EditorApplication::VegetationPlacementMode::Tree)
                    {
                        Forest* forest = selectedObject->getComponent<Forest>();
                        if (forest && forest->speciesCount() > 0)
                        {
                            const u32 species = glm::min(app().vegetationPlacementSpecies(),
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
                    ray.origin = glm::vec3(nearPoint);
                    ray.direction = direction;
                    hit = app().scene().pickObject(ray);
                }
                EditorApplication::PickedSubmesh& picked = app().pickedSubmesh();
                if (hit)
                {
                    // A plain click is a fresh start, even on the object the
                    // batch already belongs to - only Shift adds to it. The
                    // single submesh it lands on is put back into the batch
                    // right after the pick below, so Delete always has the
                    // thing under the cursor to act on.
                    if (!ImGui::GetIO().KeyShift)
                    {
                        app().submeshSelection().object = 0;
                        app().submeshSelection().indices.clear();
                    }
                    app().selection().select(hit->id());
                    // Only meaningful off the depth-buffer surface point -
                    // the ray-vs-AABB fallback has no exact surface position
                    // to test a submesh's own (tighter) box against.
                    picked.index = -1;
                    s32 pickedSlot = -1;
                    if (hasSurfacePosition)
                        pickedSlot = Scene::pickSubmeshAtPoint(*hit, surfacePosition, &picked.index);
                    picked.object = picked.index >= 0 ? hit->id() : 0;
                    picked.justPicked = picked.index >= 0;

                    // With the Pick Surface tool the batch IS the selection:
                    // a plain click makes it exactly the submesh under the
                    // cursor, Shift toggles that one in or out of what is
                    // already there. Keeping a plain click out of the batch
                    // left Delete with nothing to act on right after the
                    // user had visibly selected something.
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
                            -(mEditorView * glm::vec4(surfacePosition, 1.0f)).z;
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
            else if (glm::abs(direction.y) > 0.0001f)
            {
                const f32 t = -nearPoint.y / direction.y;
                if (t >= 0.0f)
                    app().setCursor3D(glm::vec3(nearPoint) + direction * t);
            }
        }
        // Delete removes every submesh the Shift-click batch above has
        // accumulated, not just the single last-picked one - the Inspector's
        // own "Delete Selected Submeshes" button is the only other way to
        // reach the same EditorApplication::deleteSubmeshSelection().
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
            ImGui::IsKeyPressed(ImGuiKey_Delete) && !app().submeshSelection().indices.empty())
            app().deleteSubmeshSelection();
        // Escape drops the whole batch - the way back from a rectangle that
        // caught more than it should, without hunting each wrong piece down
        // to Shift-click it off again.
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
            ImGui::IsKeyPressed(ImGuiKey_Escape) && !app().submeshSelection().indices.empty())
            app().submeshSelection().indices.clear();
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddCircleFilled(navigationCenter, 7.0f, IM_COL32(75, 125, 220, 255));
        draw->AddLine(navigationCenter, zAxis, IM_COL32(80, 220, 100, 255), 3.0f);
        draw->AddLine(navigationCenter, xAxis, IM_COL32(220, 70, 70, 255), 4.0f);
        draw->AddLine(navigationCenter, yAxis, IM_COL32(70, 100, 220, 255), 4.0f);
        // Negative ends hollow, positive ends filled - both click to snap the
        // view onto that axis.
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
