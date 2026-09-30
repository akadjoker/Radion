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

#include "Math.h"

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

    // Not part of the failure cascade: a failed decal array makes ForwardPass use its neutral placeholder.
    if (!mDecals.create())
        Log::warning("Renderer: decal texture arrays unavailable, decals will not render");

    // Same: a debug overlay nobody may turn on is not worth failing the frame.
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

    // Registration order only decides shutdown order and find()/at() enumeration, not draw order (see Renderer::execute()).
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

    // The sky must be in the scene colour before Water/Ocean copy it, else refraction shows the clear colour.
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

    // Same slot as water: alpha-blended, reads opaque depth and the sky colour.
    RenderTechnique* ocean = createOceanPass();
    if (!add(ocean))
    {
        delete ocean;
        shutdown();
        return false;
    }
    mOwned.push_back(ocean);

    // Transparent particles: after water/ocean, before trails/debug overlays.
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
                                u32 width, u32 height, const Math::vec4& sourceRect)
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
    // Mirror plane source, in order: MaterialMirror surface, Ocean queue, Water plane from refraction packets (both horizontal). One plane only: the first found wins; other scenes want a probe.
    Math::vec3 planePoint(0.0f);
    Math::vec3 planeNormal(0.0f, 1.0f, 0.0f);
    // custom0.y: Quality combo; 0 means unset, use 0.5.
    f32 resolutionScale = 0.5f;
    // custom0.w: Zoom, a multiplier on the fitted frustum; 0 or 1 leaves it alone.
    f32 fovZoom = 1.0f;
    // MaterialMirror only: world-space corners (BL/BR/TL) of the quad, used to build a cropped frustum (Kooima portal camera) instead of the main camera's FOV.
    bool haveMirrorRect = false;
    Math::vec3 mirrorCornerBL(0.0f), mirrorCornerBR(0.0f), mirrorCornerTL(0.0f);
    // custom1.x: Far Plane; 0 means auto (twice the mirror distance, floored at 5000).
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

                // Rendering the scene a second time must be worth it: skip a mirror only a few pixels across. Decided per frame (nothing cached), so it resumes the frame it grows back.
                {
                    constexpr f32 kMinMirrorApparentSize = 0.02f;
                    const Math::mat4& model = frame.list->models()[packet.instance];
                    if (const Mesh* mesh = Assets().getMesh(instance.mesh))
                    {
                        const AABB world = transformAABB(mesh->bounds, model);
                        const f32 distance = Math::length(world.center() - frame.cameraPosition);
                        if (distance > world.radius() &&
                            world.radius() / distance < kMinMirrorApparentSize)
                            continue;
                    }
                }

                // Translation and local +Y of the mirror's model matrix, same normal convention as buildPlane().
                const Math::mat4& model = frame.list->models()[packet.instance];
                planePoint = Math::vec3(model[3]);
                planeNormal = Math::normalize(Math::vec3(model[1]));
                if (instance.material->params.custom0.y > 0.0f)
                    resolutionScale = instance.material->params.custom0.y;
                if (instance.material->params.custom0.w > 0.0f)
                    fovZoom = instance.material->params.custom0.w;
                if (instance.material->params.custom1.x > 0.0f)
                    farPlaneOverride = instance.material->params.custom1.x;
                if (const Mesh* mirrorMesh = Assets().getMesh(instance.mesh))
                {
                    // buildPlane() layout: XZ at y=0, so AABB X/Z corners are the quad's corners.
                    const Math::vec3& min = mirrorMesh->bounds.min;
                    const Math::vec3& max = mirrorMesh->bounds.max;
                    mirrorCornerBL = Math::vec3(model * Math::vec4(min.x, 0.0f, min.z, 1.0f));
                    mirrorCornerBR = Math::vec3(model * Math::vec4(max.x, 0.0f, min.z, 1.0f));
                    mirrorCornerTL = Math::vec3(model * Math::vec4(min.x, 0.0f, max.z, 1.0f));
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
            planePoint = Math::vec3(0.0f, command.model[3].y, 0.0f);
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
            planePoint = Math::vec3(0.0f, frame.list->models()[water[0].instance][3].y, 0.0f);
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

    // Half resolution by default: the lookup is distorted and never shown sharp. A Mirror can ask for another scale (resolutionScale).
    const u32 width = Math::max(1u, static_cast<u32>(static_cast<f32>(frame.width) * resolutionScale));
    const u32 height =
        Math::max(1u, static_cast<u32>(static_cast<f32>(frame.height) * resolutionScale));
    if (mReflection.width != width || mReflection.height != height)
    {
        mReflection.destroy();
        // HDR like the main scene: RGBA8 would clip bright clouds to flat white. Mips on: a mirror samples with textureLod per unit of Roughness.
        if (!mReflection.create(width, height, Format::RGBA16F, Format::Depth24,
                                "renderer.reflection", /*mips=*/true))
        {
            Log::error("Renderer: reflection target failed; water falls back to the sky");
            return false;
        }
    }

    RADION_PROFILE_SCOPE("Reflection");
    RADION_GPU_PROFILE_SCOPE("Reflection");

    // Householder reflection: reflect(x) = x - 2*dot(x - planePoint, n)*n; for n=(0,1,0) this is diag(1,-1,1) + translate(0,2*level,0).
    const f32 nx = planeNormal.x, ny = planeNormal.y, nz = planeNormal.z;
    Math::mat4 mirror(1.0f);
    mirror[0] = Math::vec4(1.0f - 2.0f * nx * nx, -2.0f * nx * ny, -2.0f * nx * nz, 0.0f);
    mirror[1] = Math::vec4(-2.0f * nx * ny, 1.0f - 2.0f * ny * ny, -2.0f * ny * nz, 0.0f);
    mirror[2] = Math::vec4(-2.0f * nx * nz, -2.0f * ny * nz, 1.0f - 2.0f * nz * nz, 0.0f);
    mirror[3] = Math::vec4(2.0f * Math::dot(planePoint, planeNormal) * planeNormal, 1.0f);

    // Only camera and target move; lights, shadow atlas and sky are inherited.
    FrameContext reflectFrame = frame;
    const Math::vec3 reflectedEye = Math::vec3(mirror * Math::vec4(frame.cameraPosition, 1.0f));
    reflectFrame.cameraPosition = reflectedEye;

    if (haveMirrorRect)
    {
        // Kooima's generalized perspective projection (MSU, 2009): a frustum cropped to the mirror rectangle seen from the eye. The corners lie on the plane, so only the eye is mirrored.
        const Math::vec3& pa = mirrorCornerBL;
        const Math::vec3& pb = mirrorCornerBR;
        const Math::vec3& pc = mirrorCornerTL;
        Math::vec3 vr = Math::normalize(pb - pa);
        Math::vec3 vu = Math::normalize(pc - pa);
        Math::vec3 vn = Math::normalize(Math::cross(vr, vu));
        // vn must point back toward the eye for `d` to be positive.
        if (Math::dot(vn, reflectedEye - pa) < 0.0f)
            vn = -vn;

        const Math::vec3 va = pa - reflectedEye;
        const Math::vec3 vb = pb - reflectedEye;
        const Math::vec3 vc = pc - reflectedEye;
        // Perpendicular eye-to-plane distance (negated dot, since va and vn point opposite ways). The near plane sits there: nothing between eye and mirror should render.
        const f32 d = Math::max(-Math::dot(va, vn), 0.01f);
        const f32 nearPlane = d;
        // FrameContext does not track far, so 5000 is a guess (custom1.x overrides). Kept past nearPlane: near >= far makes Math::frustum() degenerate.
        const f32 farPlane = Math::max(farPlaneOverride > 0.0f ? farPlaneOverride : 5000.0f,
                                      nearPlane * 2.0f);

        const f32 l = Math::dot(vr, va) * nearPlane / d;
        const f32 r = Math::dot(vr, vb) * nearPlane / d;
        const f32 b = Math::dot(vu, va) * nearPlane / d;
        const f32 t = Math::dot(vu, vc) * nearPlane / d;
        // fovZoom > 1 grows the window past its edges (room for Bump/roughness sampling).
        const f32 halfWidth = (r - l) * 0.5f * fovZoom;
        const f32 halfHeight = (t - b) * 0.5f * fovZoom;
        const f32 midX = (r + l) * 0.5f;
        const f32 midY = (t + b) * 0.5f;
        reflectFrame.projection = Math::frustum(midX - halfWidth, midX + halfWidth,
                                               midY - halfHeight, midY + halfHeight, nearPlane,
                                               farPlane);

        // Rotates world axes onto (vr, vu, vn); the third row uses +vn, negating it looks behind the mirror.
        Math::mat4 basis(1.0f);
        basis[0] = Math::vec4(vr, 0.0f);
        basis[1] = Math::vec4(vu, 0.0f);
        basis[2] = Math::vec4(vn, 0.0f);
        reflectFrame.view = Math::transpose(basis) *
                            Math::translate(Math::mat4(1.0f), -reflectedEye);
    }
    else
    {
        // Water/Ocean: unbounded surface, so keep the main camera's FOV, mirrored; Zoom can still widen it.
        reflectFrame.view = frame.view * mirror;
        reflectFrame.projection = frame.projection;
        if (fovZoom != 1.0f)
        {
            reflectFrame.projection[0][0] /= fovZoom;
            reflectFrame.projection[1][1] /= fovZoom;
        }
    }
    reflectFrame.viewProjection = reflectFrame.projection * reflectFrame.view;
    // Cut everything behind the plane; the ocean pass reads this plane to skip itself.
    reflectFrame.clipPlane =
        Math::vec4(planeNormal, -Math::dot(planePoint, planeNormal));
    reflectFrame.target = mReflection.target;
    reflectFrame.viewport = Viewport{0.0f, 0.0f, static_cast<f32>(width), static_cast<f32>(height)};
    reflectFrame.width = width;
    reflectFrame.height = height;
    // Occlusion belongs to the main camera's depth, not this view.
    reflectFrame.ambientOcclusion = TextureHandle();

    // frame.list was culled against the main camera; a dedicated list culled against the mirrored view-projection is rebuilt each frame, storage reused.
    if (!mReflectionList)
        mReflectionList = new RenderList();
    // reflectionCapture=true: MeshRenderer::visibleInReflections applies to planar mirrors as to captureEnvironment().
    if (!casters.buildShadowList(*mReflectionList, reflectFrame.viewProjection, 0, nullptr,
                                 MeshHandle(), 0, true))
        return false;
    reflectFrame.list = mReflectionList;

    // Alpha 0: what the mirrored view never drew stays transparent; the surface shader mixes on it between reflection and sky.
    ClearValue clear;
    clear.bits = ClearColor | ClearDepth;
    clear.color[0] = clear.color[1] = clear.color[2] = clear.color[3] = 0.0f;
    clear.depth = 1.0f;
    GPU::getSingleton().setTarget(mReflection.target, clear);

    // Only Forward/Sky draw here, both write gl_ClipDistance[0]; off again right after.
    GPU& gpu = GPU::getSingleton();
    gpu.setClipDistanceEnabled(true);
    if (forward)
        forward->execute(reflectFrame);
    if (sky)
        sky->execute(reflectFrame);
    gpu.setClipDistanceEnabled(false);

    // Fills the mip chain a mirror samples with textureLod for Roughness blur (HAS_MIRROR in lit.frag).
    gpu.generateMips(mReflection.color);

    Assets().publishRenderTarget(kReflectionTargetName, mReflection.color);
    frame.reflectionViewProj = reflectFrame.viewProjection;
    return true;
}

bool Renderer::executeRefraction(ShadowCasterSource& casters, FrameContext& frame)
{
    // Same plane the reflection found, by the same two routes.
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

    const u32 width = Math::max(1u, frame.width);
    const u32 height = Math::max(1u, frame.height);
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

    // Ordinary camera, clipped: dot(worldPos, plane) = level - y keeps everything at or below the surface. No Sky: it would put clouds inside the pool.
    FrameContext refractFrame = frame;
    refractFrame.clipPlane = Math::vec4(0.0f, -1.0f, 0.0f, level);
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

    Math::mat4 faceViewProjection[EnvironmentProbe::FaceCount];
    probe.faceViewProjections(faceViewProjection);

    GPU& gpu = GPU::getSingleton();
    const f32 resolution = static_cast<f32>(probe.resolution());

    for (u32 face = 0; face < EnvironmentProbe::FaceCount; ++face)
    {
        const TargetHandle target = probe.faceTarget(face);
        if (!target.valid())
            continue;

        // Everything not camera-dependent is inherited; only camera and target are per face.
        FrameContext faceFrame = frame;
        faceFrame.target = target;
        faceFrame.viewProjection = faceViewProjection[face];
        faceFrame.projection = Math::mat4(1.0f);
        faceFrame.view = Math::mat4(1.0f);
        faceFrame.cameraPosition = probe.position;
        faceFrame.viewport = Viewport{0.0f, 0.0f, resolution, resolution};
        faceFrame.width = probe.resolution();
        faceFrame.height = probe.resolution();
        faceFrame.clipPlane = Math::vec4(0.0f);
        // No reflection inside the reflection: sampling the cubemap while rendering into it is undefined.
        faceFrame.environmentCube = TextureHandle();
        faceFrame.environmentProbeIntensity = 0.0f;
        // No depth buffer or prepass in the capture, so no AO.
        faceFrame.ambientOcclusion = TextureHandle();

        // Depth cleared with colour: the six faces share one depth buffer.
        ClearValue clear;
        clear.bits = ClearColor | ClearDepth;
        clear.depth = 1.0f;
        gpu.setTarget(target, clear);

        if (wantsWorld && mCaptureList)
        {
            // buildShadowList is the only way to build a list for a foreign camera. Filter 0 lets everything through; no lights are added, so the sun direction comes from the sky, not the scene's directional light (keep them in sync).
            if (casters.buildShadowList(*mCaptureList, faceViewProjection[face], 0, nullptr,
                                        probe.excludeMesh, probe.excludeObjectId, true))
            {
                faceFrame.list = mCaptureList;
                if (forward)
                    forward->execute(faceFrame);
            }
        }

        // Sky last: it writes only where nothing else did, via pipeline state since there is no depth buffer.
        if (sky)
        {
            faceFrame.list = nullptr;
            sky->execute(faceFrame);
        }
    }

    gpu.setTarget(TargetHandle());
    // Mips 1..n are the roughness axis; a box filter stands in for the GGX prefilter.
    probe.generateMips();
}

bool Renderer::add(RenderTechnique* technique)
{
    if (!technique)
        return false;

    if (!technique->setup())
    {
        // setup() can fail partway; shutdown() must be safe on a partially set-up technique (destroy on an invalid handle is a no-op).
        technique->shutdown();
        Log::error("Renderer: technique '%s' failed to set up and was dropped", technique->name());
        return false;
    }

    mTechniques.push_back(technique);
    return true;
}

void Renderer::execute(const FrameContext& frame)
{
    // Registration order (walked by find()/at()) is not draw order: Forward's opaque/transparent halves bracket the rest, and grass/trees composite before Sky. See docs/review.md findings 1-4, 38.
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
