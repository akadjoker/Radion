#include "PCH.h"

#include "VolumetricPass.h"

#include "AssetManager.h"
#include "EnvironmentBlock.h"
#include "Lighting.h"
#include "Log.h"
#include "Material.h"
#include "PostProcess.h"

namespace Radion
{

namespace
{

// Matches VolumetricSettings in every volumetric_*.comp/.frag shader.
struct alignas(16) VolumetricSettingsBlock
{
    Math::Mat4 inverseViewProjection = Math::Mat4(1.0f);
    Math::Vec4 cameraPosAndMaxDistance = Math::Vec4(0.0f);
    Math::Vec4 destSizeAndSamples = Math::Vec4(0.0f); // x,y size, z samples, w scattering
};

struct alignas(16) VolumetricSunBlock
{
    Math::Vec4 sunColorAndDensity = Math::Vec4(0.0f);
    Math::Vec4 debugFallback = Math::Vec4(0.0f);
};

struct alignas(16) VolumetricSpotBlock
{
    Math::Vec4 densityAndCount = Math::Vec4(0.0f); // x density, y entity count
};

struct alignas(16) VolumetricAddBlock
{
    Math::Vec4 destSizeAndStrength = Math::Vec4(0.0f);
};

struct alignas(16) VolumetricProxyBlock
{
    Math::Mat4 viewProjection = Math::Mat4(1.0f);
    Math::Vec4 lightPosAndScale = Math::Vec4(0.0f);
};

struct alignas(16) VolumetricPointBlock
{
    Math::Vec4 positionAndRange = Math::Vec4(0.0f);
    Math::Vec4 colorAndDensity = Math::Vec4(0.0f);
    Math::Vec4 shadowAtlasMulAdd = Math::Vec4(0.0f);
    Math::Vec4 hasShadow = Math::Vec4(0.0f);
};

struct alignas(16) VolumetricRectBlock
{
    Math::Vec4 positionAndRange = Math::Vec4(0.0f);
    Math::Vec4 colorAndDensity = Math::Vec4(0.0f);
    Math::Vec4 rightAndWidth = Math::Vec4(0.0f);
    Math::Vec4 upAndHeight = Math::Vec4(0.0f);
    Math::Mat4 shadowVP = Math::Mat4(1.0f);
    Math::Vec4 shadowAtlasMulAdd = Math::Vec4(0.0f);
    Math::Vec4 hasShadowAndTwoSided = Math::Vec4(0.0f);
};

} // namespace

bool VolumetricPass::setup()
{
    GPU& gpu = GPU::getSingleton();

    const auto makeUniform = [&gpu](u64 size, const char* name)
    {
        BufferDesc desc;
        desc.size = size;
        desc.usage = BufferUniform;
        desc.residency = Residency::Stream;
        desc.debugName = name;
        return gpu.createBuffer(desc);
    };
    mSettingsBlock = makeUniform(sizeof(VolumetricSettingsBlock), "volumetric.settings");
    mSunBlock = makeUniform(sizeof(VolumetricSunBlock), "volumetric.sun");
    mSpotBlock = makeUniform(sizeof(VolumetricSpotBlock), "volumetric.spot");
    mAddBlock = makeUniform(sizeof(VolumetricAddBlock), "volumetric.add");
    mProxyBlock = makeUniform(sizeof(VolumetricProxyBlock), "volumetric.proxy");
    mPointBlock = makeUniform(sizeof(VolumetricPointBlock), "volumetric.point");
    mRectBlock = makeUniform(sizeof(VolumetricRectBlock), "volumetric.rect");

    SamplerDesc samplerDesc;
    samplerDesc.filter = Filter::Linear;
    samplerDesc.wrapU = Wrap::Clamp;
    samplerDesc.wrapV = Wrap::Clamp;
    mSampler = gpu.createSampler(samplerDesc);

    AssetManager& assets = Assets();
    mSphereProxy = assets.createSphere(1.0f, 12, 16);
    mCubeProxy = assets.createBox(Math::Vec3(2.0f));

    // Pipelines load their shaders lazily in ensurePipelines(), same reason
    // DepthPass/Lighting defer theirs: setup() runs before a demo has added
    // its asset search paths.
    return mSettingsBlock.valid() && mSunBlock.valid() && mSpotBlock.valid() &&
           mAddBlock.valid() && mProxyBlock.valid() && mPointBlock.valid() &&
           mRectBlock.valid() && mSampler.valid() && mSphereProxy.valid() && mCubeProxy.valid();
}

bool VolumetricPass::ensurePipelines()
{
    if (mPipelinesReady)
        return true;
    if (mPipelinesFailed)
        return false;

    AssetManager& assets = Assets();
    const auto loadCompute = [&assets](const char* file, const char* name)
    {
        const std::string& source = assets.loadShader(file);
        PipelineHandle pipeline;
        if (source.empty())
        {
            Log::error("VolumetricPass: %s is missing", file);
            return pipeline;
        }
        PipelineDesc desc;
        desc.cs = {source.c_str(), 0, file};
        desc.debugName = name;
        return GPU::getSingleton().createPipeline(desc);
    };

    mSunPipeline = loadCompute("volumetric_sun.comp", "volumetric.sun");
    mSpotPipeline = loadCompute("volumetric_spot.comp", "volumetric.spot");
    mAddPipeline = loadCompute("volumetric_add.comp", "volumetric.add");

    const std::string& proxyVertex = assets.loadShader("volumetric_proxy.vert");
    const std::string& pointFragment = assets.loadShader("volumetric_point.frag");
    const std::string& rectFragment = assets.loadShader("volumetric_rect.frag");
    if (!proxyVertex.empty() && !pointFragment.empty())
    {
        PipelineDesc desc;
        desc.vs = {proxyVertex.c_str(), 0, "volumetric_proxy.vert"};
        desc.fs = {pointFragment.c_str(), 0, "volumetric_point.frag"};
        desc.layout = Assets().getMesh(mSphereProxy)->depthLayout;
        desc.blend.mode = BlendMode::Additive;
        desc.depth.test = false;
        desc.depth.write = false;
        // Draw the proxy's back faces: the camera can end up inside a light's
        // volume, and front-face culling would then discard everything.
        desc.raster.cull = CullMode::Front;
        desc.debugName = "volumetric.point";
        mPointPipeline = GPU::getSingleton().createPipeline(desc);
    }
    if (!proxyVertex.empty() && !rectFragment.empty())
    {
        PipelineDesc desc;
        desc.vs = {proxyVertex.c_str(), 0, "volumetric_proxy.vert"};
        desc.fs = {rectFragment.c_str(), 0, "volumetric_rect.frag"};
        desc.layout = Assets().getMesh(mCubeProxy)->depthLayout;
        desc.blend.mode = BlendMode::Additive;
        desc.depth.test = false;
        desc.depth.write = false;
        desc.raster.cull = CullMode::Front;
        desc.debugName = "volumetric.rect";
        mRectPipeline = GPU::getSingleton().createPipeline(desc);
    }

    mPipelinesReady = mSunPipeline.valid() && mSpotPipeline.valid() && mAddPipeline.valid() &&
                      mPointPipeline.valid() && mRectPipeline.valid();
    mPipelinesFailed = !mPipelinesReady;
    return mPipelinesReady;
}

bool VolumetricPass::resize(u32 halfWidth, u32 halfHeight)
{
    // Both members must agree, not just mAccumA: a previous call that
    // resized mAccumA and then failed on mAccumB left them at different
    // sizes (or mAccumB invalid), and comparing mAccumA alone would call
    // that "already done" forever after.
    if (mAccumA.width == halfWidth && mAccumA.height == halfHeight && mAccumA.valid() &&
        mAccumB.width == halfWidth && mAccumB.height == halfHeight && mAccumB.valid())
        return true;

    bool ok = true;
    if (mAccumA.width != halfWidth || mAccumA.height != halfHeight || !mAccumA.valid())
        ok &= mAccumA.create(halfWidth, halfHeight, Format::RGBA16F, Format::Unknown,
                             "volumetric.accumA");
    if (mAccumB.width != halfWidth || mAccumB.height != halfHeight || !mAccumB.valid())
        ok &= mAccumB.create(halfWidth, halfHeight, Format::RGBA16F, Format::Unknown,
                             "volumetric.accumB");
    return ok;
}

void VolumetricPass::execute(FrameContext& frame, PostProcessStack& post, Lighting& lighting)
{
    if (!sunEnabled && !spotEnabled && !pointEnabled && !rectEnabled)
        return;
    if (!ensurePipelines())
        return;

    const u32 halfWidth = glm::max(1u, post.sceneWidth() / 2);
    const u32 halfHeight = glm::max(1u, post.sceneHeight() / 2);
    if (!resize(halfWidth, halfHeight))
    {
        Log::warning("VolumetricPass: could not (re)size the half-resolution "
                     "accumulation targets, skipping this frame");
        return;
    }

    GPU& gpu = GPU::getSingleton();

    VolumetricSettingsBlock settings;
    settings.inverseViewProjection = glm::inverse(frame.viewProjection);
    settings.cameraPosAndMaxDistance = Math::Vec4(frame.cameraPosition, maxDistance);
    settings.destSizeAndSamples = Math::Vec4(static_cast<f32>(halfWidth),
                                            static_cast<f32>(halfHeight),
                                            static_cast<f32>(samples), scattering);
    gpu.updateBuffer(mSettingsBlock, 0, sizeof(settings), &settings);

    // Cleared first, always: whichever source runs first overwrites every
    // pixel, but if the sun is the only one disabled this frame, spot's
    // read-write add still needs a defined zero to start from rather than
    // whatever was left from a previous frame.
    ClearValue clear;
    clear.bits = ClearColor;
    gpu.setTarget(mAccumA.target, clear);

    if (sunEnabled)
        runSun(frame, post);
    if (spotEnabled && lighting.entityCount() > 0)
        runSpot(frame, post, lighting);
    if (rectEnabled)
        runRects(frame, post, lighting);
    if (pointEnabled)
        runPoints(frame, post, lighting);

    resolve(frame, post);
}

void VolumetricPass::runSun(const FrameContext& frame, PostProcessStack& post)
{
    if (!frame.directionalShadow.valid() || !frame.directionalShadowBlock.valid())
        return;

    GPU& gpu = GPU::getSingleton();

    VolumetricSunBlock sun;
    const EnvironmentBlock environment = environmentForFrame(frame);
    sun.sunColorAndDensity = Math::Vec4(Math::Vec3(environment.sunColor), sunDensity);
    sun.debugFallback = Math::Vec4(debugFallback ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f);
    gpu.updateBuffer(mSunBlock, 0, sizeof(sun), &sun);

    gpu.bindTexture(0, post.sceneDepth());
    gpu.bindTexture(1, frame.directionalShadow, frame.directionalShadowSampler);
    gpu.bindImage(2, mAccumA.color, 0, true);
    gpu.bindUniform(0, frame.directionalShadowBlock);
    gpu.bindUniform(1, mSettingsBlock);
    gpu.bindUniform(2, mSunBlock);

    gpu.setPipeline(mSunPipeline);
    gpu.dispatch((mAccumA.width + 7) / 8, (mAccumA.height + 7) / 8, 1);
    gpu.barrier(BarrierImageWrite | BarrierTexture);
}

void VolumetricPass::runSpot(const FrameContext& frame, PostProcessStack& post, Lighting& lighting)
{
    if (!frame.entityBuffer.valid() || !frame.shadowAtlas.valid())
    {
        static bool warned = false;
        if (!warned)
        {
            Log::warning("VolumetricPass: spot beam skipped, entity buffer or "
                         "shadow atlas is not ready yet");
            warned = true;
        }
        return;
    }

    GPU& gpu = GPU::getSingleton();

    ClearValue clear;
    clear.bits = ClearColor;
    gpu.setTarget(mAccumB.target, clear);

    VolumetricSpotBlock spot;
    spot.densityAndCount = Math::Vec4(spotDensity, static_cast<f32>(lighting.entityCount()), 0.0f,
                                     0.0f);
    gpu.updateBuffer(mSpotBlock, 0, sizeof(spot), &spot);

    gpu.bindStorage(0, frame.entityBuffer);
    gpu.bindStorage(1, frame.entityMatrixBuffer);
    gpu.bindTexture(0, post.sceneDepth());
    gpu.bindTexture(1, frame.shadowAtlas, frame.shadowAtlasSampler);
    gpu.bindImage(2, mAccumB.color, 0, true);
    gpu.bindUniform(1, mSettingsBlock);
    gpu.bindUniform(2, mSpotBlock);

    gpu.setPipeline(mSpotPipeline);
    gpu.dispatch((mAccumB.width + 7) / 8, (mAccumB.height + 7) / 8, 1);
    gpu.barrier(BarrierImageWrite | BarrierTexture);

    // Add spot's buffer onto the sun's - same shader as the final composite,
    // just at half resolution and between two half-resolution textures.
    VolumetricAddBlock add;
    add.destSizeAndStrength = Math::Vec4(static_cast<f32>(mAccumA.width),
                                        static_cast<f32>(mAccumA.height), spotStrength, 0.0f);
    gpu.updateBuffer(mAddBlock, 0, sizeof(add), &add);
    gpu.bindTexture(0, mAccumB.color, mSampler);
    gpu.bindImage(1, mAccumA.color, 0, true);
    gpu.bindUniform(0, mAddBlock);
    gpu.setPipeline(mAddPipeline);
    gpu.dispatch((mAccumA.width + 7) / 8, (mAccumA.height + 7) / 8, 1);
    gpu.barrier(BarrierImageWrite | BarrierTexture);
}

void VolumetricPass::runRects(const FrameContext& frame, PostProcessStack& post, Lighting& lighting)
{
    if (!frame.shadowAtlas.valid())
        return;
    const Mesh* mesh = Assets().getMesh(mCubeProxy);
    if (!mesh || mesh->submeshes.empty())
        return;

    GPU& gpu = GPU::getSingleton();
    gpu.setTarget(mAccumA.target);
    gpu.setViewport({0.0f, 0.0f, static_cast<f32>(mAccumA.width), static_cast<f32>(mAccumA.height)});
    gpu.setPipeline(mRectPipeline);

    VolumetricProxyBlock proxy;
    proxy.viewProjection = frame.viewProjection;
    VolumetricRectBlock rect;
    const SubMesh& submesh = mesh->submeshes[0];
    bool drewAny = false;

    for (const RenderLight& light : lighting.entities())
    {
        if (light.type != RenderLightType::Rectangle) continue;
        if ((light.flags & RenderLightVolumetric) == 0) continue;

        proxy.lightPosAndScale = Math::Vec4(light.position, light.range);
        gpu.updateBuffer(mProxyBlock, 0, sizeof(proxy), &proxy);

        rect.positionAndRange = Math::Vec4(light.position, light.range);
        rect.colorAndDensity = Math::Vec4(light.color, rectDensity * rectStrength);
        rect.rightAndWidth = Math::Vec4(light.rectangleRight, light.rectangleWidth);
        rect.upAndHeight = Math::Vec4(light.rectangleUp, light.rectangleHeight);
        const bool hasShadow = light.matrixIndex >= 0 && light.shadowFade > 0.0f &&
                               static_cast<usize>(light.matrixIndex) < lighting.matrices().size();
        rect.shadowVP = hasShadow ? lighting.matrices()[static_cast<usize>(light.matrixIndex)]
                                  : Math::Mat4(1.0f);
        rect.shadowAtlasMulAdd = light.shadowAtlasMulAdd;
        rect.hasShadowAndTwoSided = Math::Vec4(hasShadow ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f);
        gpu.updateBuffer(mRectBlock, 0, sizeof(rect), &rect);

        gpu.bindUniform(0, mProxyBlock);
        gpu.bindUniform(1, mSettingsBlock);
        gpu.bindUniform(2, mRectBlock);
        gpu.bindTexture(0, post.sceneDepth());
        gpu.bindTexture(2, frame.shadowAtlas, frame.shadowAtlasSampler);

        DrawDesc draw;
        draw.vertexBuffers[0] = mesh->positionBuffer;
        draw.vertexBufferCount = 1;
        draw.indexBuffer = mesh->indexBuffer;
        draw.indexType = mesh->indexType;
        draw.first = submesh.indexOffset;
        draw.count = submesh.indexCount;
        gpu.draw(draw);
        drewAny = true;
    }

    if (drewAny)
        gpu.barrier(BarrierImageWrite | BarrierTexture);
}

void VolumetricPass::runPoints(const FrameContext& frame, PostProcessStack& post, Lighting& lighting)
{
    if (!frame.shadowAtlas.valid())
        return;
    const MeshHandle proxyHandle = pointProxyIsCube ? mCubeProxy : mSphereProxy;
    const Mesh* mesh = Assets().getMesh(proxyHandle);
    if (!mesh || mesh->submeshes.empty())
        return;

    GPU& gpu = GPU::getSingleton();
    gpu.setTarget(mAccumA.target);
    gpu.setViewport({0.0f, 0.0f, static_cast<f32>(mAccumA.width), static_cast<f32>(mAccumA.height)});
    gpu.setPipeline(mPointPipeline);

    VolumetricProxyBlock proxy;
    proxy.viewProjection = frame.viewProjection;
    VolumetricPointBlock point;
    const SubMesh& submesh = mesh->submeshes[0];
    bool drewAny = false;

    for (const RenderLight& light : lighting.entities())
    {
        if (light.type != RenderLightType::Point) continue;
        if ((light.flags & RenderLightVolumetric) == 0) continue;

        // The sphere proxy is exact and needs a small margin so the range's
        // edge is not clipped by the proxy itself at grazing angles; the cube
        // already covers ~1.9x the sphere's volume and needs none.
        const f32 scale = light.range * (pointProxyIsCube ? 1.0f : 1.05f);
        proxy.lightPosAndScale = Math::Vec4(light.position, scale);
        gpu.updateBuffer(mProxyBlock, 0, sizeof(proxy), &proxy);

        point.positionAndRange = Math::Vec4(light.position, light.range);
        point.colorAndDensity = Math::Vec4(light.color, pointDensity * pointStrength);
        point.shadowAtlasMulAdd = light.shadowAtlasMulAdd;
        const bool hasShadow = light.matrixIndex >= 0 && light.shadowFade > 0.0f;
        point.hasShadow = Math::Vec4(hasShadow ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f);
        gpu.updateBuffer(mPointBlock, 0, sizeof(point), &point);

        gpu.bindUniform(0, mProxyBlock);
        gpu.bindUniform(1, mSettingsBlock);
        gpu.bindUniform(2, mPointBlock);
        gpu.bindTexture(0, post.sceneDepth());
        gpu.bindTexture(2, frame.shadowAtlas, frame.shadowAtlasSampler);

        DrawDesc draw;
        draw.vertexBuffers[0] = mesh->positionBuffer;
        draw.vertexBufferCount = 1;
        draw.indexBuffer = mesh->indexBuffer;
        draw.indexType = mesh->indexType;
        draw.first = submesh.indexOffset;
        draw.count = submesh.indexCount;
        gpu.draw(draw);
        drewAny = true;
    }

    if (drewAny)
        gpu.barrier(BarrierImageWrite | BarrierTexture);
}

void VolumetricPass::resolve(const FrameContext& frame, PostProcessStack& post)
{
    GPU& gpu = GPU::getSingleton();

    if (blurEnabled)
    {
        const Viewport half{0.0f, 0.0f, static_cast<f32>(mAccumA.width),
                            static_cast<f32>(mAccumA.height)};
        post.draw(mAccumA.color, TextureHandle(), mAccumB.target, half, 4, Math::Vec4(0.0f));
        post.draw(mAccumB.color, TextureHandle(), mAccumA.target, half, 5, Math::Vec4(0.0f));
    }

    VolumetricAddBlock add;
    add.destSizeAndStrength = Math::Vec4(static_cast<f32>(post.sceneWidth()),
                                        static_cast<f32>(post.sceneHeight()), strength, 0.0f);
    gpu.updateBuffer(mAddBlock, 0, sizeof(add), &add);
    gpu.bindTexture(0, mAccumA.color, mSampler);
    gpu.bindImage(1, post.sceneColor(), 0, true);
    gpu.bindUniform(0, mAddBlock);
    gpu.setPipeline(mAddPipeline);
    gpu.dispatch((post.sceneWidth() + 7) / 8, (post.sceneHeight() + 7) / 8, 1);
    gpu.barrier(BarrierImageWrite | BarrierTexture);

    (void)frame;
}

void VolumetricPass::shutdown()
{
    GPU& gpu = GPU::getSingleton();
    mAccumA.destroy();
    mAccumB.destroy();
    gpu.destroy(mSunPipeline);
    gpu.destroy(mSpotPipeline);
    gpu.destroy(mAddPipeline);
    gpu.destroy(mPointPipeline);
    gpu.destroy(mRectPipeline);
    gpu.destroy(mSettingsBlock);
    gpu.destroy(mSunBlock);
    gpu.destroy(mSpotBlock);
    gpu.destroy(mAddBlock);
    gpu.destroy(mProxyBlock);
    gpu.destroy(mPointBlock);
    gpu.destroy(mRectBlock);
    gpu.destroy(mSampler);
}

} // namespace Radion
