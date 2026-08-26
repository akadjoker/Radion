#include "PCH.h"

#include "LensFlarePass.h"

#include "AssetManager.h"
#include "GPU.h"
#include "PostProcess.h"
#include "Sky.h"

namespace Radion
{

namespace
{

struct LensFlareBlock
{
    Math::vec4 lightScreenPosAndOcclusion;
    Math::vec4 offsetAndSize;
    Math::vec4 screenSizeRcp;
};

 
constexpr f32 kElementOffsets[LensFlarePass::kElementCount] = {1.0f,  0.55f, 0.4f, 0.1f,
                                                               -0.1f, -0.3f, -0.5f};
constexpr const char* kElementFiles[LensFlarePass::kElementCount] = {
    "lensflare/flare_0_halo.png",   "lensflare/flare_1_disco.png",
    "lensflare/flare_2_anel.png",   "lensflare/flare_3_hexa.png",
    "lensflare/flare_4_disco.png",  "lensflare/flare_5_hexa.png",
    "lensflare/flare_6_estria.png",
};

} // namespace

bool LensFlarePass::setup()
{
    return true;
}

bool LensFlarePass::ensurePipeline()
{
    if (mPipelineReady)
        return true;
    if (mPipelineFailed)
        return false;

    GPU& gpu = GPU::getSingleton();
    AssetManager& assets = Assets();

    // Deferred to first use, not setup(): setup() runs inside
    // Engine::initialize(), before a demo has registered its own asset
    // search paths - loading here the way GrassRender/TreeRender defer their
    // own shaders to ensurePipelines() rather than setup().
    SamplerDesc depthDesc;
    depthDesc.filter = Filter::Point;
    depthDesc.wrapU = Wrap::Clamp;
    depthDesc.wrapV = Wrap::Clamp;
    mDepthSampler = assets.getSampler(depthDesc);

    SamplerDesc flareDesc;
    flareDesc.filter = Filter::Linear;
    flareDesc.wrapU = Wrap::Clamp;
    flareDesc.wrapV = Wrap::Clamp;
    mFlareSampler = assets.getSampler(flareDesc);

    for (u32 i = 0; i < kElementCount; ++i)
    {
        mElementTextures[i] = assets.loadTexture(kElementFiles[i], ColorSpace::sRGB, false);
        if (!mElementTextures[i].valid())
            Log::warning("LensFlarePass: missing %s", kElementFiles[i]);
    }

    const std::string& vertex = assets.loadShader("shaders/lensflare.vert");
    const std::string& fragment = assets.loadShader("shaders/lensflare.frag");
    if (vertex.empty() || fragment.empty())
    {
        Log::error("LensFlarePass: shaders are missing; the flare will not draw");
        mPipelineFailed = true;
        return false;
    }

    PipelineDesc desc;
    desc.vs = {vertex.c_str(), 0, "lensflare.vert"};
    desc.fs = {fragment.c_str(), 0, "lensflare.frag"};
    desc.topology = Topology::TriangleStrip;
    desc.raster.cull = CullMode::None;
    desc.depth.test = false;
    desc.depth.write = false;
    desc.blend.mode = BlendMode::Additive;
    desc.debugName = "lensflare";
    mPipeline = gpu.createPipeline(desc);
    if (!mPipeline.valid())
    {
        mPipelineFailed = true;
        return false;
    }

    BufferDesc bufferDesc;
    bufferDesc.size = sizeof(LensFlareBlock);
    bufferDesc.usage = BufferUniform;
    bufferDesc.residency = Residency::Dynamic;
    bufferDesc.debugName = "lensflare.block";
    mBlock = gpu.createBuffer(bufferDesc);

    mPipelineReady = mBlock.valid();
    mPipelineFailed = !mPipelineReady;
    return mPipelineReady;
}

bool LensFlarePass::ensureTarget(PostProcessStack& post)
{
    const TextureHandle color = post.sceneColor();
    if (mColorOnlyTarget.valid() && mAliasedColor == color)
        return true;

    GPU& gpu = GPU::getSingleton();
    if (mColorOnlyTarget.valid())
        gpu.destroy(mColorOnlyTarget);

    TargetDesc desc;
    desc.colors[0].texture = color;
    desc.colorCount = 1;
    desc.debugName = "lensflare.colorOnly";
    mColorOnlyTarget = gpu.createTarget(desc);
    mAliasedColor = color;
    return mColorOnlyTarget.valid();
}

void LensFlarePass::execute(const FrameContext& frame, PostProcessStack& post)
{
    if (!enabled || !frame.sky)
        return;
    if (!ensurePipeline() || !ensureTarget(post))
        return;

    // Directional light, sun at infinity: place it far along the sun
    // direction from the camera, the way Wicked does (POS = eye + D * -zFar).
    const Math::vec3 toSunDir = Math::normalize(frame.sky->sunDirection);
    const Math::vec3 sunWorld = frame.cameraPosition + toSunDir * sunDistance;
    const Math::vec4 clip = frame.viewProjection * Math::vec4(sunWorld, 1.0f);
    if (clip.w <= 0.0f)
        return;

    const Math::vec3 cameraForward =
        Math::normalize(Math::vec3(Math::inverse(frame.view) * Math::vec4(0.0f, 0.0f, -1.0f, 0.0f)));
    if (Math::dot(toSunDir, cameraForward) <= 0.0f)
        return;

    const Math::vec3 ndc = Math::vec3(clip) / clip.w;
    const Math::vec3 screenPos(ndc.x * 0.5f + 0.5f, ndc.y * 0.5f + 0.5f, ndc.z * 0.5f + 0.5f);

    GPU& gpu = GPU::getSingleton();
    gpu.setTarget(mColorOnlyTarget);
    gpu.setViewport(Viewport{0.0f, 0.0f, static_cast<f32>(post.sceneWidth()),
                             static_cast<f32>(post.sceneHeight())});
    gpu.setPipeline(mPipeline);
    gpu.bindTexture(0, post.sceneDepth(), mDepthSampler);
    gpu.bindUniform(0, mBlock);

    LensFlareBlock block;
    block.lightScreenPosAndOcclusion = Math::vec4(screenPos, occlusionRadius);
    block.screenSizeRcp = Math::vec4(1.0f / static_cast<f32>(post.sceneWidth()),
                                    1.0f / static_cast<f32>(post.sceneHeight()),
                                    debugOcclusion ? 1.0f : 0.0f, 0.0f);

    DrawDesc command;
    command.count = 4;

    for (u32 i = 0; i < kElementCount; ++i)
    {
        if (!mElementTextures[i].valid())
            continue;

        TextureDesc info;
        const f32 width = gpu.textureInfo(mElementTextures[i], info) ? static_cast<f32>(info.width) : 64.0f;
        const f32 height = info.height > 0 ? static_cast<f32>(info.height) : 64.0f;

        block.offsetAndSize = Math::vec4(kElementOffsets[i], width, height, 0.0f);
        gpu.updateBuffer(mBlock, 0, sizeof(block), &block);
        gpu.bindTexture(1, mElementTextures[i], mFlareSampler);
        gpu.draw(command);
    }

    gpu.setTarget(TargetHandle());
}

void LensFlarePass::shutdown()
{
    GPU& gpu = GPU::getSingleton();
    if (mColorOnlyTarget.valid())
        gpu.destroy(mColorOnlyTarget);
    if (mPipeline.valid())
        gpu.destroy(mPipeline);
    if (mBlock.valid())
        gpu.destroy(mBlock);
    mColorOnlyTarget = TargetHandle();
    mPipeline = PipelineHandle();
    mBlock = BufferHandle();
    mPipelineReady = false;
    mPipelineFailed = false;
}

} // namespace Radion
