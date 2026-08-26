#include "PCH.h"

#include "Renderer.h"

#include "AssetManager.h"
#include "DebugDraw3D.h"
#include "DepthPass.h"
#include "ForwardPass.h"
#include "GPUProfiler.h"
#include "GrassRender.h"
#include "HairRender.h"
#include "Log.h"
#include "OceanRender.h"
#include "ParticlePass.h"
#include "Profiler.h"
#include "RenderList.h"
#include "ShadowDebugView.h"
#include "ShadowPass.h"
#include "Sky.h"
#include "TrailRender.h"
#include "TreeRender.h"
#include "WaterPass.h"

#include <glm/gtc/matrix_transform.hpp>

namespace Radion
{

bool Renderer::addDefaultPasses()
{
    mDepth = new DepthPass();
    if (!mDepth->setup())
    {
        delete mDepth;
        mDepth = nullptr;
        return false;
    }

    mShadows = new ShadowPass();
    if (!mShadows->setup())
    {
        delete mShadows;
        mShadows = nullptr;
        mDepth->shutdown();
        delete mDepth;
        mDepth = nullptr;
        return false;
    }

    if (!mLighting.setup())
    {
        mLighting.shutdown();
        mShadows->shutdown();
        delete mShadows;
        mShadows = nullptr;
        mDepth->shutdown();
        delete mDepth;
        mDepth = nullptr;
        return false;
    }

    // Not part of the failure cascade below: a decal array that fails to
    // allocate leaves mDecals with no valid textures, and ForwardPass falls
    // back to its neutral placeholder the same as if none had been added.
    if (!mDecals.create())
        Log::warning("Renderer: decal texture arrays unavailable, decals will not render");

    // Same reasoning: a debug overlay nobody may ever turn on is not worth
    // failing the whole frame over.
    mShadowDebugView = new ShadowDebugView();
    if (!mShadowDebugView->setup())
        Log::warning("Renderer: shadow debug view unavailable");

    if (!mVolumetric.setup())
    {
        mVolumetric.shutdown();
        mLighting.shutdown();
        mShadows->shutdown();
        delete mShadows;
        mShadows = nullptr;
        mDepth->shutdown();
        delete mDepth;
        mDepth = nullptr;
        return false;
    }

    if (!mLensFlare.setup())
    {
        mLensFlare.shutdown();
        mVolumetric.shutdown();
        mLighting.shutdown();
        mShadows->shutdown();
        delete mShadows;
        mShadows = nullptr;
        mDepth->shutdown();
        delete mDepth;
        mDepth = nullptr;
        return false;
    }

    ForwardPass* forward = new ForwardPass();
    if (!add(forward))
    {
        delete forward;
        mDepth->shutdown();
        delete mDepth;
        mDepth = nullptr;
        return false;
    }

    mOwned.push_back(forward);

    // Registration order here only decides shutdown order and what find()/
    // at() enumerate; it is not the order the frame draws in. Renderer::
    // execute() runs Forward's opaque half, then these two, then Sky, then
    // Water/Ocean, then Forward's transparent half - grass/trees are
    // alpha-tested opaque, so they have to be resolved into the depth/colour
    // buffer before Sky fills whatever they did not cover, and long before
    // anything blended reads that colour back for refraction.
    RenderTechnique* grass = createGrassPass();
    if (!add(grass))
    {
        delete grass;
        shutdown();
        return false;
    }
    mOwned.push_back(grass);

    RenderTechnique* hair = createHairPass();
    if (!add(hair))
    {
        delete hair;
        shutdown();
        return false;
    }
    mOwned.push_back(hair);

    RenderTechnique* trees = createTreePass();
    if (!add(trees))
    {
        delete trees;
        shutdown();
        return false;
    }
    mOwned.push_back(trees);

    // The scene colour behind transparent surfaces must already contain the
    // sky: Water/Ocean's own scene-colour copy is taken right after this
    // runs, and the sky has to be in it already or refraction shows the clear
    // colour instead. Opaque depth from the prepass still keeps the sky out
    // of terrain/grass/tree pixels.
    RenderTechnique* sky = createSkyPass();
    if (!add(sky))
    {
        delete sky;
        shutdown();
        return false;
    }
    mOwned.push_back(sky);

    WaterPass* water = new WaterPass();
    if (!add(water))
    {
        delete water;
        shutdown();
        return false;
    }

    mOwned.push_back(water);

    // Same slot as water, right after it: a Gerstner surface is also
    // alpha-blended geometry that reads the opaque depth and the sky colour
    // already drawn behind it.
    RenderTechnique* ocean = createOceanPass();
    if (!add(ocean))
    {
        delete ocean;
        shutdown();
        return false;
    }
    mOwned.push_back(ocean);

    // Transparent particles: after water/ocean (which read scene colour), but
    // before trails/debug so later overlays draw on top.
    RenderTechnique* particles = createParticlePass();
    if (!add(particles))
    {
        delete particles;
        shutdown();
        return false;
    }
    mOwned.push_back(particles);

    RenderTechnique* trails = createTrailPass();
    if (!add(trails))
    {
        delete trails;
        shutdown();
        return false;
    }
    mOwned.push_back(trails);

    RenderTechnique* debug = createDebugPass();
    if (!add(debug))
    {
        delete debug;
        shutdown();
        return false;
    }
    mOwned.push_back(debug);
    return true;
}

CascadeShadowSettings* Renderer::cascadeSettings()
{
    return mShadows ? &mShadows->cascadeSettings() : nullptr;
}

f32 Renderer::cascadeHalfExtent(u32 cascade) const
{
    return mShadows ? mShadows->halfExtent(cascade) : 0.0f;
}

f32 Renderer::cascadeSplit(u32 cascade) const
{
    return mShadows ? mShadows->split(cascade) : 0.0f;
}

const RenderListStats* Renderer::shadowListStats() const
{
    return mShadows ? &mShadows->lastCascadeStats() : nullptr;
}

const char* Renderer::sunShadowStatus() const
{
    return mShadows ? ShadowPass::skipReasonText(mShadows->skipReason()) : "no shadow pass";
}

void Renderer::debugDrawShadows(bool showCascades, bool showAtlas, u32 windowWidth,
                                u32 windowHeight)
{
    if (!mShadowDebugView)
        return;
    if (showCascades && mShadows)
        mShadowDebugView->drawCascades(mShadows->texture(), mShadows->cascadeSettings().count,
                                       windowWidth, windowHeight);
    if (showAtlas)
        mShadowDebugView->drawAtlas(mLighting.atlasTexture(), windowWidth, windowHeight);
}

void Renderer::debugDrawTexture(TextureHandle texture, bool isArray, u32 layer, TargetHandle target,
                                u32 width, u32 height, const glm::vec4& sourceRect)
{
    if (mShadowDebugView)
        mShadowDebugView->drawTexture(texture, isArray, layer, target, width, height, sourceRect);
}

void Renderer::debugDrawCubemap(TextureHandle texture, u32 face, u32 mip, TargetHandle target,
                                u32 width, u32 height)
{
    if (mShadowDebugView)
        mShadowDebugView->drawCubemap(texture, face, mip, target, width, height);
}

TextureHandle Renderer::directionalShadowTexture() const
{
    return mShadows ? mShadows->texture() : TextureHandle();
}

void Renderer::executeShadows(ShadowCasterSource& casters, FrameContext& frame)
{
    if (mShadows && mDepth)
        mShadows->execute(casters, frame, *mDepth);
}

void Renderer::executeDepth(const FrameContext& frame)
{
    if (!mDepth)
        return;
    ProfileScope scope(mDepth->name());
    mDepth->execute(frame);
}

void Renderer::executeLightingPrepare(ShadowCasterSource& casters, FrameContext& frame,
                                      bool renderShadows)
{
    if (mDepth)
        mLighting.prepare(casters, frame, *mDepth, renderShadows);
}

void Renderer::executeLightingCull(FrameContext& frame, TextureHandle sceneDepth, u32 screenWidth,
                                   u32 screenHeight)
{
    mLighting.cull(frame, sceneDepth, screenWidth, screenHeight);
}

void Renderer::submitDecals(FrameContext& frame)
{
    mLighting.submitDecals(mDecals);
    frame.decalAlbedo = mDecals.albedoArray();
    frame.decalNormal = mDecals.normalArray();
    frame.decalSurface = mDecals.surfaceArray();
}

void Renderer::executeVolumetric(FrameContext& frame, PostProcessStack& post)
{
    mVolumetric.execute(frame, post, mLighting);
}

void Renderer::executeLensFlare(const FrameContext& frame, PostProcessStack& post)
{
    mLensFlare.execute(frame, post);
}

const char* Renderer::reflectionSource() const
{
    return mReflectionSource;
}

bool Renderer::executeReflection(ShadowCasterSource& casters, FrameContext& frame)
{
    mReflectionSource = "none";
    // The plane to mirror through comes from whatever asked for a
    // reflection, tried in this order: a MaterialMirror surface first -
    // arbitrary orientation, deliberately placed - then an Ocean off its own
    // queue, then a Water plane off the frame's refraction packets, both of
    // which have only ever been horizontal. Reading it here rather than
    // taking it as an argument is what keeps this out of the caller's hands.
    //
    // One plane, not one per surface: a second mirrored pass costs the scene
    // again, and two reflective surfaces that disagree in one shot is not
    // what this is for. The first one found wins, and a scene that needs
    // otherwise wants a probe, not this.
    glm::vec3 planePoint(0.0f);
    glm::vec3 planeNormal(0.0f, 1.0f, 0.0f);
    // custom0.y - the Inspector's Quality combo, 0 meaning "not set, use the
    // 0.5 default" so a mirror saved before this field existed still renders
    // exactly as it always has.
    f32 resolutionScale = 0.5f;
    // custom0.w - the Inspector's Zoom slider, now a creative multiplier on
    // top of the fitted frustum below rather than the only thing standing
    // between a mirror and running off its own edges. 0 or 1 means "leave
    // the fitted frustum alone".
    f32 fovZoom = 1.0f;
    // Set only for a MaterialMirror plane (never for Water/Ocean, which have
    // no fixed rectangle to fit): world-space corners of the mirror's own
    // quad, bottom-left/bottom-right/top-left, used below to build a
    // frustum cropped to exactly this rectangle (Kooima's generalized
    // perspective projection - the "portal camera" technique) instead of
    // reusing the main camera's own FOV, which only ever covered a mirror
    // as large as what that camera happened to see from here. Without this
    // a mirror ran off its own edges when large/close, and panned/smeared
    // as the main camera turned, since the reflected view direction was
    // borrowed from it instead of aimed at the mirror itself.
    bool haveMirrorRect = false;
    glm::vec3 mirrorCornerBL(0.0f), mirrorCornerBR(0.0f), mirrorCornerTL(0.0f);
    // custom1.x - the Inspector's Far Plane field. 0 means "not set", auto
    // below (twice the distance to the mirror, floored at 5000).
    f32 farPlaneOverride = 0.0f;
    bool wanted = false;

    if (frame.list)
    {
        for (RenderCategory category : {RenderCategory::Opaque, RenderCategory::AlphaTest})
        {
            for (const RenderPacket& packet : frame.list->packets(category))
            {
                const RenderInstance& instance = frame.list->instance(packet.instance);
                if (!instance.material || !(instance.material->flags & MaterialMirror))
                    continue;

                // The whole scene is about to be rendered a second time for
                // this one surface, so it has to be worth a second pass.
                // Frustum culling already kept the list to what is in view;
                // what it cannot say is that the mirror is a few pixels
                // across, and a reflection nobody can make out costs exactly
                // as much as one filling the screen.
                //
                // Safe to decide per frame here, unlike the occlusion
                // threshold this deliberately does NOT copy: nothing is
                // cached, so a mirror that grows on approach starts
                // reflecting again on the very frame it crosses back.
                {
                    constexpr f32 kMinMirrorApparentSize = 0.02f;
                    const glm::mat4& model = frame.list->models()[packet.instance];
                    if (const Mesh* mesh = Assets().getMesh(instance.mesh))
                    {
                        const AABB world = transformAABB(mesh->bounds, model);
                        const f32 distance = glm::length(world.center() - frame.cameraPosition);
                        if (distance > world.radius() &&
                            world.radius() / distance < kMinMirrorApparentSize)
                            continue;
                    }
                }

                // Translation and local +Y of the mirror's own model matrix -
                // the same normal convention buildPlane() gives an ordinary
                // Plane mesh, so rotating the GameObject in the editor is
                // all it takes to aim the mirror anywhere.
                const glm::mat4& model = frame.list->models()[packet.instance];
                planePoint = glm::vec3(model[3]);
                planeNormal = glm::normalize(glm::vec3(model[1]));
                if (instance.material->params.custom0.y > 0.0f)
                    resolutionScale = instance.material->params.custom0.y;
                if (instance.material->params.custom0.w > 0.0f)
                    fovZoom = instance.material->params.custom0.w;
                if (instance.material->params.custom1.x > 0.0f)
                    farPlaneOverride = instance.material->params.custom1.x;
                if (const Mesh* mirrorMesh = Assets().getMesh(instance.mesh))
                {
                    // buildPlane()'s own layout: XZ at y=0, so the AABB's
                    // min/max X/Z corners are exactly the quad's corners.
                    const glm::vec3& min = mirrorMesh->bounds.min;
                    const glm::vec3& max = mirrorMesh->bounds.max;
                    mirrorCornerBL = glm::vec3(model * glm::vec4(min.x, 0.0f, min.z, 1.0f));
                    mirrorCornerBR = glm::vec3(model * glm::vec4(max.x, 0.0f, min.z, 1.0f));
                    mirrorCornerTL = glm::vec3(model * glm::vec4(min.x, 0.0f, max.z, 1.0f));
                    haveMirrorRect = true;
                }
                wanted = true;
                mReflectionSource = "mirror";
                mReflectionMaterialName = instance.material->name;
                mReflectionMaterialFlags = instance.material->flags;
                break;
            }
            if (wanted)
                break;
        }
    }

    if (!wanted)
    {
        for (const OceanDrawCommand& command : OceanDraws().commands())
        {
            if (command.quality == OceanQuality::SkyOnly)
                continue;
            planePoint = glm::vec3(0.0f, command.model[3].y, 0.0f);
            wanted = true;
            mReflectionSource = "ocean";
            break;
        }
    }
    if (!wanted && frame.list)
    {
        const std::vector<RenderPacket>& water = frame.list->packets(RenderCategory::Refraction);
        if (!water.empty())
        {
            planePoint = glm::vec3(0.0f, frame.list->models()[water[0].instance][3].y, 0.0f);
            wanted = true;
            mReflectionSource = "water";
        }
    }
    if (mReflectionSource != mLoggedReflectionSource)
    {
        mLoggedReflectionSource = mReflectionSource;
        Log::info("Renderer: planar reflection source is '%s' (material '%s', flags 0x%X)",
                  mReflectionSource, mReflectionMaterialName.c_str(), mReflectionMaterialFlags);
    }

    if (!wanted)
        return false;

    RenderTechnique* forward = find("Forward");
    RenderTechnique* sky = find("Sky");
    if (!forward && !sky)
        return false;

    // Half resolution by default: the surface distorts the lookup by its own
    // normal and never shows the result sharp, so the other three quarters of
    // the pixels would be paid for and thrown away. A Mirror can ask for a
    // different scale (its Quality combo, resolutionScale above) when it is
    // what the camera is actually looking at and worth the extra cost, or
    // less when it is a background prop.
    const u32 width = glm::max(1u, static_cast<u32>(static_cast<f32>(frame.width) * resolutionScale));
    const u32 height =
        glm::max(1u, static_cast<u32>(static_cast<f32>(frame.height) * resolutionScale));
    if (mReflection.width != width || mReflection.height != height)
    {
        mReflection.destroy();
        // Reflection is rendered by the same HDR forward/sky shaders as the
        // main scene. RGBA8 clipped every component above 1 before the water
        // shader could tone or attenuate it, turning bright clouds into a
        // flat white texture. Mips on: a mirror reads this back with
        // textureLod, one level per unit of Roughness, for a frosted/rough
        // mirror instead of every one being polished glass.
        if (!mReflection.create(width, height, Format::RGBA16F, Format::Depth24,
                                "renderer.reflection", /*mips=*/true))
        {
            Log::error("Renderer: reflection target failed; water falls back to the sky");
            return false;
        }
    }

    RADION_PROFILE_SCOPE("Reflection");
    RADION_GPU_PROFILE_SCOPE("Reflection");

    // Householder reflection through (planePoint, planeNormal):
    // reflect(x) = x - 2*dot(x - planePoint, planeNormal)*planeNormal. The
    // linear part is symmetric (I - 2*n*n^T), the translation is
    // 2*dot(planePoint,planeNormal)*planeNormal - for planeNormal (0,1,0)
    // this collapses to exactly the diag(1,-1,1) + translate(0,2*level,0)
    // this used to hardcode, so Water/Ocean reflect exactly as before.
    const f32 nx = planeNormal.x, ny = planeNormal.y, nz = planeNormal.z;
    glm::mat4 mirror(1.0f);
    mirror[0] = glm::vec4(1.0f - 2.0f * nx * nx, -2.0f * nx * ny, -2.0f * nx * nz, 0.0f);
    mirror[1] = glm::vec4(-2.0f * nx * ny, 1.0f - 2.0f * ny * ny, -2.0f * ny * nz, 0.0f);
    mirror[2] = glm::vec4(-2.0f * nx * nz, -2.0f * ny * nz, 1.0f - 2.0f * nz * nz, 0.0f);
    mirror[3] = glm::vec4(2.0f * glm::dot(planePoint, planeNormal) * planeNormal, 1.0f);

    // Everything that does not depend on where the camera is stays: the
    // lights, the shadow atlas, the sky. Only the camera and the target move.
    FrameContext reflectFrame = frame;
    const glm::vec3 reflectedEye = glm::vec3(mirror * glm::vec4(frame.cameraPosition, 1.0f));
    reflectFrame.cameraPosition = reflectedEye;

    if (haveMirrorRect)
    {
        // Kooima's generalized perspective projection ("Generalized
        // Perspective Projection", MSU, 2009) - the standard technique
        // behind any correct planar mirror or portal: a frustum cropped to
        // exactly the rectangle (mirrorCornerBL/BR/TL) as seen from the eye,
        // rather than reusing the main camera's own FOV in some borrowed
        // direction. The three corners are ON the mirror plane, which
        // reflection through that same plane leaves fixed - they need no
        // mirroring of their own, only the eye does.
        const glm::vec3& pa = mirrorCornerBL;
        const glm::vec3& pb = mirrorCornerBR;
        const glm::vec3& pc = mirrorCornerTL;
        glm::vec3 vr = glm::normalize(pb - pa);
        glm::vec3 vu = glm::normalize(pc - pa);
        glm::vec3 vn = glm::normalize(glm::cross(vr, vu));
        // vn has to point back toward the eye for `d` below to come out
        // positive - cross(vr, vu) has no reason to already agree with
        // that for an arbitrarily rotated mirror.
        if (glm::dot(vn, reflectedEye - pa) < 0.0f)
            vn = -vn;

        const glm::vec3 va = pa - reflectedEye;
        const glm::vec3 vb = pb - reflectedEye;
        const glm::vec3 vc = pc - reflectedEye;
        // Perpendicular distance from the eye to the window's own plane -
        // va points FROM the eye TO the corner and vn was just oriented
        // toward the eye, so va and vn point roughly opposite ways and
        // their dot product comes out negative; this is the actual
        // distance, not that raw (negative) dot product.
        // The near plane sits exactly there: nothing between the eye and
        // the mirror surface is ever meant to render (the clip plane below
        // enforces the same boundary from the other side), so there is no
        // depth range being wasted or geometry being clipped away by it.
        const f32 d = glm::max(-glm::dot(va, vn), 0.01f);
        const f32 nearPlane = d;
        // Not tracked anywhere on FrameContext (only the finished projection
        // matrix is), so 5000 is a guess - the Inspector's Far Plane field
        // (custom1.x) overrides it for a scene where that guess is wrong.
        // Kept comfortably past nearPlane regardless of source: near >= far
        // makes glm::frustum() build a degenerate matrix that draws
        // nothing, and a mirror far from the camera (or scaled to a large
        // world) can push nearPlane (== the distance to it) past either on
        // its own.
        const f32 farPlane = glm::max(farPlaneOverride > 0.0f ? farPlaneOverride : 5000.0f,
                                      nearPlane * 2.0f);

        const f32 l = glm::dot(vr, va) * nearPlane / d;
        const f32 r = glm::dot(vr, vb) * nearPlane / d;
        const f32 b = glm::dot(vu, va) * nearPlane / d;
        const f32 t = glm::dot(vu, vc) * nearPlane / d;
        // fovZoom > 1 grows the window past its own edges either side -
        // useful room for Bump/roughness sampling near the rim, or simply
        // an artistic wider reflection than the literal rectangle.
        const f32 halfWidth = (r - l) * 0.5f * fovZoom;
        const f32 halfHeight = (t - b) * 0.5f * fovZoom;
        const f32 midX = (r + l) * 0.5f;
        const f32 midY = (t + b) * 0.5f;
        reflectFrame.projection = glm::frustum(midX - halfWidth, midX + halfWidth,
                                               midY - halfHeight, midY + halfHeight, nearPlane,
                                               farPlane);

        // Rotates world axes onto (vr, vu, vn). Kooima's view basis uses +vn
        // for the third row here; negating it makes the camera look behind
        // the mirror and leaves the reflection frustum with nothing to draw.
        glm::mat4 basis(1.0f);
        basis[0] = glm::vec4(vr, 0.0f);
        basis[1] = glm::vec4(vu, 0.0f);
        basis[2] = glm::vec4(vn, 0.0f);
        reflectFrame.view = glm::transpose(basis) *
                            glm::translate(glm::mat4(1.0f), -reflectedEye);
    }
    else
    {
        // Water/Ocean: no fixed rectangle to fit a frustum to (the surface
        // can be effectively unbounded), so this keeps exactly the
        // behaviour reflections have always had - the main camera's own
        // FOV, mirrored, Zoom still able to widen it by hand.
        reflectFrame.view = frame.view * mirror;
        reflectFrame.projection = frame.projection;
        if (fovZoom != 1.0f)
        {
            reflectFrame.projection[0][0] /= fovZoom;
            reflectFrame.projection[1][1] /= fovZoom;
        }
    }
    reflectFrame.viewProjection = reflectFrame.projection * reflectFrame.view;
    // Everything on the far side of the plane is cut away: it is not visible
    // in the mirror, and the ocean pass reads this same plane to know it
    // must sit this one out rather than reflect itself.
    reflectFrame.clipPlane =
        glm::vec4(planeNormal, -glm::dot(planePoint, planeNormal));
    reflectFrame.target = mReflection.target;
    reflectFrame.viewport = Viewport{0.0f, 0.0f, static_cast<f32>(width), static_cast<f32>(height)};
    reflectFrame.width = width;
    reflectFrame.height = height;
    // The occlusion belongs to the main camera's depth, which this view does
    // not share - applying it here darkens the wrong pixels.
    reflectFrame.ambientOcclusion = TextureHandle();

    // frame.list was culled against the main camera's frustum: an object
    // outside it but visible in the mirror would never appear, and one this
    // view no longer needs would stay in it. A dedicated list, culled against
    // the mirrored view-projection, is what the shadow views already do
    // through the same interface - this storage is kept and reused frame to
    // frame, only the culling result is rebuilt.
    if (!mReflectionList)
        mReflectionList = new RenderList();
    // `reflectionCapture=true`: a planar mirror is exactly the other place
    // MeshRenderer::visibleInReflections has to be honoured, the same reason
    // captureEnvironment() passes it for the probe below - a shared name for
    // "does not appear in a reflection", not two different flags for two
    // different reflection techniques.
    if (!casters.buildShadowList(*mReflectionList, reflectFrame.viewProjection, 0, nullptr,
                                 MeshHandle(), 0, true))
        return false;
    reflectFrame.list = mReflectionList;

    // Alpha 0, so what the mirrored view never drew stays transparent: the
    // surface shader mixes on that alpha to decide between the reflection and
    // the sky, and a clear of 1 would tell it every pixel is geometry.
    ClearValue clear;
    clear.bits = ClearColor | ClearDepth;
    clear.color[0] = clear.color[1] = clear.color[2] = clear.color[3] = 0.0f;
    clear.depth = 1.0f;
    GPU::getSingleton().setTarget(mReflection.target, clear);

    // Only Forward/Sky draw here, and both write gl_ClipDistance[0] - lit,
    // unlit, landscape and the sky shader all do. Off again immediately after:
    // nothing else in the frame may run with an unwritten clip output.
    GPU& gpu = GPU::getSingleton();
    gpu.setClipDistanceEnabled(true);
    if (forward)
        forward->execute(reflectFrame);
    if (sky)
        sky->execute(reflectFrame);
    gpu.setClipDistanceEnabled(false);

    // Fills the chain mReflection.create(mips=true) reserved - a mirror
    // samples from it with textureLod for Roughness-driven blur (lit.frag's
    // HAS_MIRROR block); water/ocean keep sampling level 0 the same as
    // before, untouched by this existing.
    gpu.generateMips(mReflection.color);

    Assets().publishRenderTarget(kReflectionTargetName, mReflection.color);
    frame.reflectionViewProj = reflectFrame.viewProjection;
    return true;
}

bool Renderer::executeRefraction(ShadowCasterSource& casters, FrameContext& frame)
{
    // Same plane the reflection found, by the same two routes - an Ocean off
    // its own queue, a Water plane off the frame's refraction packets.
    f32 level = 0.0f;
    bool wanted = false;
    for (const OceanDrawCommand& command : OceanDraws().commands())
    {
        if (command.quality == OceanQuality::SkyOnly)
            continue;
        level = command.model[3].y;
        wanted = true;
        break;
    }
    if (!wanted && frame.list)
    {
        const std::vector<RenderPacket>& water = frame.list->packets(RenderCategory::Refraction);
        if (!water.empty())
        {
            level = frame.list->models()[water[0].instance][3].y;
            wanted = true;
        }
    }
    if (!wanted)
        return false;

    RenderTechnique* forwardTechnique = find("Forward");
    if (!forwardTechnique || !forwardTechnique->isEnabled())
        return false;

    const u32 width = glm::max(1u, frame.width);
    const u32 height = glm::max(1u, frame.height);
    if (mRefraction.width != width || mRefraction.height != height)
    {
        mRefraction.destroy();
        if (!mRefraction.create(width, height, Format::RGBA16F, Format::Depth24,
                                "renderer.refraction"))
        {
            Log::error("Renderer: refraction target failed; water falls back to flat colour");
            return false;
        }
    }

    RADION_PROFILE_SCOPE("Refraction");
    RADION_GPU_PROFILE_SCOPE("Refraction");

    // The ordinary camera, only clipped: dot(worldPos, plane) = level - y, so
    // everything at or below the surface survives and everything standing out
    // of the water is cut. No Sky here - open sky belongs above the surface,
    // and letting it in is what puts clouds inside the pool.
    FrameContext refractFrame = frame;
    refractFrame.clipPlane = glm::vec4(0.0f, -1.0f, 0.0f, level);
    refractFrame.target = mRefraction.target;
    refractFrame.viewport = Viewport{0.0f, 0.0f, static_cast<f32>(width), static_cast<f32>(height)};
    refractFrame.width = width;
    refractFrame.height = height;
    refractFrame.ambientOcclusion = TextureHandle();

    if (!mRefractionList)
        mRefractionList = new RenderList();
    if (!casters.buildShadowList(*mRefractionList, refractFrame.viewProjection, 0))
        return false;
    refractFrame.list = mRefractionList;

    ClearValue clear;
    clear.bits = ClearColor | ClearDepth;
    clear.color[0] = clear.color[1] = clear.color[2] = clear.color[3] = 0.0f;
    clear.depth = 1.0f;
    GPU& gpu = GPU::getSingleton();
    gpu.setTarget(mRefraction.target, clear);

    gpu.setClipDistanceEnabled(true);
    forwardTechnique->execute(refractFrame);
    gpu.setClipDistanceEnabled(false);

    Assets().publishRenderTarget(kRefractionTargetName, mRefraction.color);
    Assets().publishRenderTarget(kRefractionDepthTargetName, mRefraction.depth);
    Assets().publishRenderTarget(kWaterRefractionDebugTargetName, mRefraction.color);
    return true;
}

void Renderer::captureEnvironment(EnvironmentProbe& probe, ShadowCasterSource& casters,
                                  const FrameContext& frame)
{
    if (!probe.ready())
        return;

    RenderTechnique* forward = find("Forward");
    RenderTechnique* sky = find("Sky");
    if (!forward && !sky)
        return;

    const bool wantsWorld = probe.content == EnvironmentProbe::Content::SkyAndWorld;
    if (wantsWorld && !mCaptureList)
        mCaptureList = new RenderList();

    glm::mat4 faceViewProjection[EnvironmentProbe::FaceCount];
    probe.faceViewProjections(faceViewProjection);

    GPU& gpu = GPU::getSingleton();
    const f32 resolution = static_cast<f32>(probe.resolution());

    for (u32 face = 0; face < EnvironmentProbe::FaceCount; ++face)
    {
        const TargetHandle target = probe.faceTarget(face);
        if (!target.valid())
            continue;

        // Everything the frame carries that does not depend on where the
        // camera points is inherited: the lights, their shadow atlas, the
        // decals, the sky. Only the camera and the target are this face's own.
        FrameContext faceFrame = frame;
        faceFrame.target = target;
        faceFrame.viewProjection = faceViewProjection[face];
        faceFrame.projection = glm::mat4(1.0f); // unused by Forward/Sky
        faceFrame.view = glm::mat4(1.0f);
        faceFrame.cameraPosition = probe.position;
        faceFrame.viewport = Viewport{0.0f, 0.0f, resolution, resolution};
        faceFrame.width = probe.resolution();
        faceFrame.height = probe.resolution();
        faceFrame.clipPlane = glm::vec4(0.0f);
        // No reflection inside the reflection: sampling the cubemap while
        // rendering into it is undefined, and one bounce is all this buys
        // anyway.
        faceFrame.environmentCube = TextureHandle();
        faceFrame.environmentProbeIntensity = 0.0f;
        // The capture has no depth buffer and no prepass, so there is no AO
        // for it to read - and reflections do not need it.
        faceFrame.ambientOcclusion = TextureHandle();

        // Depth cleared with the colour: the six faces share one depth
        // buffer, so each has to start from a clean one or it would test
        // against the previous face's geometry.
        ClearValue clear;
        clear.bits = ClearColor | ClearDepth;
        clear.depth = 1.0f;
        gpu.setTarget(target, clear);

        if (wantsWorld && mCaptureList)
        {
            // buildShadowList is the only way in to "build a list for a
            // camera that is not the scene's own". Filter 0 lets everything
            // through - a reflection shows what is there, not only what casts
            // shadows. It adds no lights to the list, so the sun direction
            // that reaches the shading comes from the sky rather than from the
            // scene's directional light: keep the two in sync (every demo
            // does) and they agree.
            if (casters.buildShadowList(*mCaptureList, faceViewProjection[face], 0, nullptr,
                                        probe.excludeMesh, probe.excludeObjectId, true))
            {
                faceFrame.list = mCaptureList;
                if (forward)
                    forward->execute(faceFrame);
            }
        }

        // Sky last: it only writes where nothing else did, and with no depth
        // buffer here that has to come from its own pipeline state rather than
        // from a depth test.
        if (sky)
        {
            faceFrame.list = nullptr;
            sky->execute(faceFrame);
        }
    }

    gpu.setTarget(TargetHandle());
    // Mips 1..n are the roughness axis; a box filter is a placeholder for the
    // GGX prefilter, enough to see roughness respond before it exists.
    probe.generateMips();
}

bool Renderer::add(RenderTechnique* technique)
{
    if (!technique)
        return false;

    if (!technique->setup())
    {
        // setup() can fail after creating some of its buffers/textures/
        // pipelines - Water's third buffer failing after its first two
        // succeeded, for example. shutdown() is required to be safe to call
        // on a partially-set-up technique (GPU::destroy() on an invalid or
        // already-destroyed handle is a no-op), so the caller never has to
        // know how far setup() got.
        technique->shutdown();
        Log::error("Renderer: technique '%s' failed to set up and was dropped", technique->name());
        return false;
    }

    mTechniques.push_back(technique);
    return true;
}

void Renderer::execute(const FrameContext& frame)
{
    // Registration order (mTechniques, still walked by find()/at()/the
    // technique panel) is not draw order any more: Forward's opaque and
    // transparent halves bracket everything else, and grass/trees have to be
    // opaque-composited before Sky fills in what they did not cover, not
    // after Water/Ocean have already blended over that gap. See findings 1-4
    // and 38 in docs/review.md.
    RenderTechnique* forwardTechnique = find("Forward");
    ForwardPass* forward = (forwardTechnique && forwardTechnique->isEnabled())
                               ? static_cast<ForwardPass*>(forwardTechnique)
                               : nullptr;

    auto run = [&frame](RenderTechnique* technique)
    {
        if (technique && technique->isEnabled())
        {
            ProfileScope scope(technique->name());
            GPUProfileScope gpuScope(technique->name());
            technique->execute(frame);
        }
    };

    if (forward)
    {
        ProfileScope scope("Forward");
        forward->executeOpaque(frame);
    }
    run(find("Grass"));
    run(find("Hair"));
    run(find("Trees"));
    run(find("Sky"));
    run(find("Water"));
    run(find("Ocean"));
    run(find("Particles"));
    if (forward)
    {
        ProfileScope scope("Forward");
        forward->executeTransparent(frame);
    }
    run(find("Trails"));
    run(find("Debug"));
}

void Renderer::shutdown()
{
    delete mCaptureList;
    mCaptureList = nullptr;
    delete mReflectionList;
    mReflectionList = nullptr;
    mReflection.destroy();
    mLensFlare.shutdown();
    mVolumetric.shutdown();
    mDecals.shutdown();
    mLighting.shutdown();
    if (mShadowDebugView)
    {
        mShadowDebugView->shutdown();
        delete mShadowDebugView;
        mShadowDebugView = nullptr;
    }
    if (mShadows)
    {
        mShadows->shutdown();
        delete mShadows;
        mShadows = nullptr;
    }
    if (mDepth)
    {
        mDepth->shutdown();
        delete mDepth;
        mDepth = nullptr;
    }
    for (usize i = mTechniques.size(); i > 0; --i)
        mTechniques[i - 1]->shutdown();
    mTechniques.clear();

    for (RenderTechnique* technique : mOwned)
        delete technique;
    mOwned.clear();
}

RenderTechnique* Renderer::find(const char* name) const
{
    if (!name)
        return nullptr;

    for (RenderTechnique* technique : mTechniques)
    {
        if (std::strcmp(technique->name(), name) == 0)
            return technique;
    }
    return nullptr;
}

bool Renderer::setEnabled(const char* name, bool enabled)
{
    RenderTechnique* technique = find(name);
    if (!technique)
        return false;

    technique->setEnabled(enabled);
    return true;
}

} // namespace Radion
