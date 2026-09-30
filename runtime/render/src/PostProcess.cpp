#include "PCH.h"

#include "PostProcess.h"

#include "AssetManager.h"
#include "Log.h"

namespace Radion
{
namespace
{

constexpr char kFullscreenVertex[] = R"GLSL(#version 450 core
layout(location = 0) out vec2 uv;
void main()
{
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    uv = p;
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

constexpr char kPostFragment[] = R"GLSL(#version 450 core
layout(binding = 0) uniform sampler2D sourceTexture;
layout(binding = 1) uniform sampler2D secondaryTexture;
layout(std140, binding = 0) uniform PostBlock
{
    mat4 projection;
    mat4 inverseProjection;
    vec4 sizeMode;
    vec4 options;
};
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;

vec3 aces(vec3 x)
{
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}

float fxaaLuma(vec3 color)
{
    return dot(color, vec3(0.299, 0.587, 0.114));
}

const float fxaaQuality[12] = float[12](
    1.0, 1.0, 1.0, 1.0, 1.0, 1.5, 2.0, 2.0, 2.0, 2.0, 4.0, 8.0
);

vec3 viewPosition(vec2 coord, float depth)
{
    vec4 clip = vec4(coord * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec4 view = inverseProjection * clip;
    return view.xyz / view.w;
}

float hashNoise(vec2 p)
{
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123);
}

void main()
{
    int mode = int(sizeMode.z + 0.5);
    vec4 color = texture(sourceTexture, uv);
    if (mode == 1)
    {
        color.rgb *= options.x;
        int curve = int(options.y + 0.5);
        if (curve == 1)
            color.rgb = color.rgb / (color.rgb + vec3(1.0));
        else if (curve == 2)
            color.rgb = aces(color.rgb);
        color.rgb = pow(clamp(color.rgb, 0.0, 1.0), vec3(1.0 / 2.2));
    }
    else if (mode == 2)
    {
        vec2 texel = sizeMode.xy;
        vec3 rgbM = color.rgb;
        float lumaM = fxaaLuma(rgbM);
        float lumaN = fxaaLuma(textureOffset(sourceTexture, uv, ivec2( 0, -1)).rgb);
        float lumaS = fxaaLuma(textureOffset(sourceTexture, uv, ivec2( 0,  1)).rgb);
        float lumaW = fxaaLuma(textureOffset(sourceTexture, uv, ivec2(-1,  0)).rgb);
        float lumaE = fxaaLuma(textureOffset(sourceTexture, uv, ivec2( 1,  0)).rgb);
        float lumaMin = min(lumaM, min(min(lumaN, lumaS), min(lumaW, lumaE)));
        float lumaMax = max(lumaM, max(max(lumaN, lumaS), max(lumaW, lumaE)));
        float lumaRange = lumaMax - lumaMin;
        if (lumaRange >= max(options.z, lumaMax * options.y))
        {
            float lumaNW = fxaaLuma(textureOffset(sourceTexture, uv, ivec2(-1, -1)).rgb);
            float lumaNE = fxaaLuma(textureOffset(sourceTexture, uv, ivec2( 1, -1)).rgb);
            float lumaSW = fxaaLuma(textureOffset(sourceTexture, uv, ivec2(-1,  1)).rgb);
            float lumaSE = fxaaLuma(textureOffset(sourceTexture, uv, ivec2( 1,  1)).rgb);
            float lumaNS = lumaN + lumaS;
            float lumaWE = lumaW + lumaE;
            float edgeH = abs(-2.0 * lumaW + lumaNW + lumaSW) +
                          abs(-2.0 * lumaM + lumaNS) * 2.0 +
                          abs(-2.0 * lumaE + lumaNE + lumaSE);
            float edgeV = abs(-2.0 * lumaN + lumaNW + lumaNE) +
                          abs(-2.0 * lumaM + lumaWE) * 2.0 +
                          abs(-2.0 * lumaS + lumaSW + lumaSE);
            bool horizontal = edgeH >= edgeV;
            float luma1 = horizontal ? lumaN : lumaW;
            float luma2 = horizontal ? lumaS : lumaE;
            float gradient1 = luma1 - lumaM;
            float gradient2 = luma2 - lumaM;
            bool firstIsSteepest = abs(gradient1) >= abs(gradient2);
            float scaledGradient = 0.25 * max(abs(gradient1), abs(gradient2));
            float perpendicularStep = horizontal ? texel.y : texel.x;
            if (!firstIsSteepest)
                perpendicularStep = -perpendicularStep;
            float localLuma = 0.5 * (firstIsSteepest ? luma1 + lumaM : luma2 + lumaM);
            vec2 edgeUV = uv;
            if (horizontal)
                edgeUV.y += perpendicularStep * 0.5;
            else
                edgeUV.x += perpendicularStep * 0.5;
            vec2 edgeDirection = horizontal ? vec2(texel.x, 0.0) : vec2(0.0, texel.y);
            vec2 uv1 = edgeUV - edgeDirection * fxaaQuality[0];
            vec2 uv2 = edgeUV + edgeDirection * fxaaQuality[0];
            float endLuma1 = fxaaLuma(texture(sourceTexture, uv1).rgb) - localLuma;
            float endLuma2 = fxaaLuma(texture(sourceTexture, uv2).rgb) - localLuma;
            bool end1 = abs(endLuma1) >= scaledGradient;
            bool end2 = abs(endLuma2) >= scaledGradient;
            if (!end1)
                uv1 -= edgeDirection * fxaaQuality[1];
            if (!end2)
                uv2 += edgeDirection * fxaaQuality[1];
            for (int i = 2; i < 12 && (!end1 || !end2); ++i)
            {
                if (!end1)
                    endLuma1 = fxaaLuma(texture(sourceTexture, uv1).rgb) - localLuma;
                if (!end2)
                    endLuma2 = fxaaLuma(texture(sourceTexture, uv2).rgb) - localLuma;
                end1 = abs(endLuma1) >= scaledGradient;
                end2 = abs(endLuma2) >= scaledGradient;
                if (!end1)
                    uv1 -= edgeDirection * fxaaQuality[i];
                if (!end2)
                    uv2 += edgeDirection * fxaaQuality[i];
            }
            float distance1 = horizontal ? uv.x - uv1.x : uv.y - uv1.y;
            float distance2 = horizontal ? uv2.x - uv.x : uv2.y - uv.y;
            bool closerToFirst = distance1 < distance2;
            float span = max(distance1 + distance2, 0.000001);
            float edgeOffset = -min(distance1, distance2) / span + 0.5;
            bool centerIsLower = lumaM < localLuma;
            bool correctSign = ((closerToFirst ? endLuma1 : endLuma2) < 0.0) != centerIsLower;
            float finalOffset = correctSign ? edgeOffset : 0.0;
            float averageLuma = (2.0 * (lumaNS + lumaWE) + lumaNW + lumaSW + lumaNE + lumaSE) /
                                12.0;
            float subpixel = clamp(abs(averageLuma - lumaM) / lumaRange, 0.0, 1.0);
            subpixel = (-2.0 * subpixel + 3.0) * subpixel * subpixel;
            finalOffset = max(finalOffset, subpixel * subpixel * options.x);
            vec2 finalUV = uv;
            if (horizontal)
                finalUV.y += finalOffset * perpendicularStep;
            else
                finalUV.x += finalOffset * perpendicularStep;
            color.rgb = texture(sourceTexture, finalUV).rgb;
        }
    }
    else if (mode == 3)
    {
        // Four taps averaged by 1/(1+luma) instead of one plain sample. A
        // specular highlight on a smooth sphere is smaller than a pixel and
        // enormously bright, so as the camera moves it lands on one pixel and
        // then the next: a straight average lets that single value dominate
        // its whole neighbourhood and the bloom pops on and off. Weighting by
        // brightness lets the neighbours hold it down, and the highlight fades
        // instead of blinking. The soft knee below cannot help with this - it
        // smooths a value crossing the threshold, not one jumping between
        // pixels.
        // sizeMode.xy is the destination texel; half of it is the source
        // texel, which is what puts these four taps on the four full-res
        // pixels this half-res one covers.
        vec2 texel = sizeMode.xy * 0.5;
        vec3 taps[4];
        taps[0] = texture(sourceTexture, uv + vec2(-texel.x, -texel.y)).rgb;
        taps[1] = texture(sourceTexture, uv + vec2( texel.x, -texel.y)).rgb;
        taps[2] = texture(sourceTexture, uv + vec2(-texel.x,  texel.y)).rgb;
        taps[3] = texture(sourceTexture, uv + vec2( texel.x,  texel.y)).rgb;

        vec3 sum = vec3(0.0);
        float weightSum = 0.0;
        for (int i = 0; i < 4; ++i)
        {
            float tapLum = dot(taps[i], vec3(0.2126, 0.7152, 0.0722));
            float weight = 1.0 / (1.0 + tapLum);
            sum += taps[i] * weight;
            weightSum += weight;
        }
        color.rgb = sum / max(weightSum, 0.0001);

        float lum = dot(color.rgb, vec3(0.2126, 0.7152, 0.0722));
        float knee = max(0.0001, options.x * options.y);
        float soft = clamp(lum - options.x + knee, 0.0, 2.0 * knee);
        soft = soft * soft / (4.0 * knee);
        float contribution = max(soft, lum - options.x) / max(lum, 0.0001);
        color.rgb *= contribution;
    }
    else if (mode == 4 || mode == 5)
    {
        vec2 stepUV = mode == 4 ? vec2(sizeMode.x, 0.0) : vec2(0.0, sizeMode.y);
        color.rgb = texture(sourceTexture, uv).rgb * 0.227027;
        color.rgb += texture(sourceTexture, uv + stepUV).rgb * 0.194594;
        color.rgb += texture(sourceTexture, uv - stepUV).rgb * 0.194594;
        color.rgb += texture(sourceTexture, uv + stepUV * 2.0).rgb * 0.121621;
        color.rgb += texture(sourceTexture, uv - stepUV * 2.0).rgb * 0.121621;
        color.rgb += texture(sourceTexture, uv + stepUV * 3.0).rgb * 0.054054;
        color.rgb += texture(sourceTexture, uv - stepUV * 3.0).rgb * 0.054054;
        color.rgb += texture(sourceTexture, uv + stepUV * 4.0).rgb * 0.016216;
        color.rgb += texture(sourceTexture, uv - stepUV * 4.0).rgb * 0.016216;
    }
    else if (mode == 6)
    {
        color.rgb += texture(secondaryTexture, uv).rgb * options.x;
    }
    else if (mode == 7)
    {
        float depth = texture(sourceTexture, uv).r;
        if (depth >= 1.0)
            color = vec4(1.0);
        else
        {
            vec3 p = viewPosition(uv, depth);
            vec2 texel = sizeMode.xy;
            float dx = texture(sourceTexture, uv + vec2(texel.x, 0.0)).r;
            float dy = texture(sourceTexture, uv + vec2(0.0, texel.y)).r;
            vec3 px = viewPosition(uv + vec2(texel.x, 0.0), dx);
            vec3 py = viewPosition(uv + vec2(0.0, texel.y), dy);
            vec3 normal = normalize(cross(px - p, py - p));
            float angle = hashNoise(gl_FragCoord.xy) * 6.2831853;
            vec3 randomVector = vec3(cos(angle), sin(angle), 0.0);
            vec3 tangent = normalize(randomVector - normal * dot(randomVector, normal));
            mat3 basis = mat3(tangent, cross(normal, tangent), normal);
            float occlusion = 0.0;
            float taken = 0.0;
            int sampleCount = clamp(int(options.w + 0.5), 4, 64);
            for (int i = 0; i < sampleCount; ++i)
            {
                float fi = float(i);
                float a = hashNoise(gl_FragCoord.xy + vec2(fi * 1.37, 0.0)) * 6.2831853;
                float r = hashNoise(gl_FragCoord.xy + vec2(0.0, fi * 2.71));
                float z = hashNoise(gl_FragCoord.xy + vec2(fi * 3.14, fi * 1.61));
                vec3 sampleVector = vec3(cos(a) * sqrt(1.0 - z * z),
                                         sin(a) * sqrt(1.0 - z * z), z);
                float scale = fi / float(sampleCount);
                sampleVector *= r * mix(0.1, 1.0, scale * scale);
                vec3 samplePosition = p + basis * sampleVector * options.x;
                vec4 projected = projection * vec4(samplePosition, 1.0);
                vec2 sampleUV = projected.xy / projected.w * 0.5 + 0.5;
                if (any(lessThan(sampleUV, vec2(0.0))) || any(greaterThan(sampleUV, vec2(1.0))))
                    continue;
                vec3 realPosition = viewPosition(sampleUV, texture(sourceTexture, sampleUV).r);
                float rangeCheck = smoothstep(0.0, 1.0,
                    options.x / max(0.0001, abs(p.z - realPosition.z)));
                occlusion += (realPosition.z >= samplePosition.z + options.y ? 1.0 : 0.0) * rangeCheck;
                taken += 1.0;
            }
            color = vec4(vec3(clamp(1.0 - occlusion / max(taken, 1.0) * options.z, 0.0, 1.0)), 1.0);
        }
    }
    else if (mode == 8 || mode == 9)
    {
        vec2 direction = mode == 8 ? vec2(sizeMode.x, 0.0) : vec2(0.0, sizeMode.y);
        float centerDepth = texture(secondaryTexture, uv).r;
        float sum = texture(sourceTexture, uv).r * 0.227027;
        float weightSum = 0.227027;
        const float weights[4] = float[4](0.194594, 0.121621, 0.054054, 0.016216);
        for (int i = 1; i <= 4; ++i)
        {
            for (int side = -1; side <= 1; side += 2)
            {
                vec2 sampleUV = uv + direction * float(i * side);
                float sampleDepth = texture(secondaryTexture, sampleUV).r;
                float bilateral = exp(-abs(sampleDepth - centerDepth) * options.x);
                float weight = weights[i - 1] * bilateral;
                sum += texture(sourceTexture, sampleUV).r * weight;
                weightSum += weight;
            }
        }
        color = vec4(vec3(sum / max(weightSum, 0.0001)), 1.0);
    }
    outColor = color;
}
)GLSL";

struct PostBlock
{
    Math::mat4 projection;
    Math::mat4 inverseProjection;
    Math::vec4 sizeMode;
    Math::vec4 options;
};

struct TAABlock
{
    Math::vec4 sizeAndFeedback;
    Math::vec4 clipAndSharpen;
};

} // namespace

bool PostProcessStack::initialize()
{
    GPU& gpu = GPU::getSingleton();
    PipelineDesc pipeline;
    pipeline.vs = {kFullscreenVertex, 0, "post.fullscreen.vert"};
    pipeline.fs = {kPostFragment, 0, "post.stack.frag"};
    pipeline.depth.test = false;
    pipeline.depth.write = false;
    pipeline.raster.cull = CullMode::None;
    pipeline.debugName = "post.stack";
    mPipeline = gpu.createPipeline(pipeline);

    BufferDesc uniform;
    uniform.size = sizeof(PostBlock);
    uniform.usage = BufferUniform;
    uniform.residency = Residency::Stream;
    uniform.debugName = "post.uniform";
    mUniform = gpu.createBuffer(uniform);

    uniform.size = sizeof(TAABlock);
    uniform.debugName = "taa.uniform";
    mTAAUniform = gpu.createBuffer(uniform);

    SamplerDesc sampler;
    sampler.filter = Filter::Linear;
    sampler.wrapU = Wrap::Clamp;
    sampler.wrapV = Wrap::Clamp;
    mSampler = gpu.createSampler(sampler);
    return mPipeline.valid() && mUniform.valid() && mTAAUniform.valid() && mSampler.valid();
}

void PostProcessStack::shutdown()
{
    mScene.destroy();
    mPing[0].destroy();
    mPing[1].destroy();
    mBloom[0].destroy();
    mBloom[1].destroy();
    mAO[0].destroy();
    mAO[1].destroy();
    mResolved[0].destroy();
    mResolved[1].destroy();
    for (u32 stream = 0; stream < 3; ++stream)
        for (u32 buffer = 0; buffer < 2; ++buffer)
            mTAAHistory[stream][buffer].destroy();
    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mPipeline);
    gpu.destroy(mTAAPipeline);
    gpu.destroy(mUniform);
    gpu.destroy(mTAAUniform);
    gpu.destroy(mSampler);
    mPipeline = PipelineHandle();
    mTAAPipeline = PipelineHandle();
    mUniform = BufferHandle();
    mTAAUniform = BufferHandle();
    mSampler = SamplerHandle();
}

namespace
{
// The fixed relative order this effect set's maths requires: Bloom sums
// additively in the HDR linear buffer, ToneMap converts that to display
// space exactly once, and FXAA needs the display-encoded result ToneMap just
// produced. Nothing here makes the layers arbitrarily interchangeable despite
// sharing one PostLayer type - see finding 24 in docs/review.md.
u8 domainOrder(PostEffect effect)
{
    switch (effect)
    {
    case PostEffect::Bloom:
        return 0;
    case PostEffect::ToneMap:
        return 1;
    case PostEffect::FXAA:
        return 2;
    }
    return 0;
}

const char* effectName(PostEffect effect)
{
    switch (effect)
    {
    case PostEffect::Bloom:
        return "Bloom";
    case PostEffect::ToneMap:
        return "ToneMap";
    case PostEffect::FXAA:
        return "FXAA";
    }
    return "?";
}
} // namespace

PostLayer& PostProcessStack::add(PostEffect effect)
{
    // A duplicate does not just double the work - two ToneMaps means gamma
    // applied twice, and a repeated Bloom convolves an already-tone-mapped
    // buffer instead of the HDR one it expects.
    for (PostLayer& layer : mLayers)
        if (layer.effect == effect)
        {
            Log::warning("PostProcess: '%s' is already in the stack; add() ignored",
                         effectName(effect));
            return layer;
        }

    // Inserted by domainOrder, not appended - a caller adding these out of
    // order (a preset file, a UI list, anything not literally Bloom then
    // ToneMap then FXAA) would otherwise corrupt the stack the same way
    // move() already refuses to: no crash, no warning, just FXAA smoothing
    // linear HDR values or Bloom convolving an already-encoded buffer.
    auto it = std::upper_bound(mLayers.begin(), mLayers.end(), effect,
                               [](PostEffect e, const PostLayer& layer)
                               {
                                   return domainOrder(e) < domainOrder(layer.effect);
                               });
    return *mLayers.insert(it, {effect, true});
}

bool PostProcessStack::remove(PostEffect effect)
{
    for (auto it = mLayers.begin(); it != mLayers.end(); ++it)
    {
        if (it->effect == effect)
        {
            mLayers.erase(it);
            return true;
        }
    }
    return false;
}

bool PostProcessStack::setEnabled(PostEffect effect, bool enabled)
{
    for (PostLayer& layer : mLayers)
    {
        if (layer.effect == effect)
        {
            layer.enabled = enabled;
            return true;
        }
    }
    return false;
}

bool PostProcessStack::isEnabled(PostEffect effect) const
{
    for (const PostLayer& layer : mLayers)
        if (layer.effect == effect)
            return layer.enabled;
    return false;
}

bool PostProcessStack::move(usize from, usize to)
{
    if (from >= mLayers.size() || to >= mLayers.size() || from == to)
        return false;

    std::vector<PostLayer> next = mLayers;
    const PostLayer layer = next[from];
    next.erase(next.begin() + from);
    next.insert(next.begin() + to, layer);

    // Reject only if the result would put a layer before one whose output it
    // depends on - not just "any" reorder: two effects with the same
    // domainOrder (none currently) would still be free to swap.
    for (usize i = 1; i < next.size(); ++i)
    {
        if (domainOrder(next[i - 1].effect) > domainOrder(next[i].effect))
        {
            Log::warning("PostProcess: move() rejected - would put %s before %s, which needs a "
                         "colour domain the stack has not produced yet",
                         effectName(next[i - 1].effect), effectName(next[i].effect));
            return false;
        }
    }

    mLayers = std::move(next);
    return true;
}

void PostProcessStack::clear()
{
    mLayers.clear();
}

bool PostProcessStack::resize(u32 width, u32 height)
{
    const u32 halfWidth = Math::max(1u, width / 2);
    const u32 halfHeight = Math::max(1u, height / 2);

    // Checking only mScene's dimensions used to be enough to call the whole
    // stack "already the right size" - until a previous resize() got mScene
    // recreated and then failed on mPing[0]. The next frame's mScene check
    // agreed nothing had changed and never retried it, leaving mPing/mBloom/
    // mAO at the old size or invalid indefinitely. Every member must both
    // match and be valid before this is allowed to skip.
    if (mScene.width == width && mScene.height == height && mScene.valid() &&
        mScene.velocity.valid() && mScene.reactive.valid() &&
        mPing[0].width == width && mPing[0].height == height && mPing[0].valid() &&
        mPing[1].width == width && mPing[1].height == height && mPing[1].valid() &&
        mBloom[0].width == halfWidth && mBloom[0].height == halfHeight && mBloom[0].valid() &&
        mBloom[1].width == halfWidth && mBloom[1].height == halfHeight && mBloom[1].valid() &&
        mAO[0].width == halfWidth && mAO[0].height == halfHeight && mAO[0].valid() &&
        mAO[1].width == halfWidth && mAO[1].height == halfHeight && mAO[1].valid())
        return true;

    // Each create() is itself transactional (see OffscreenTarget::create()):
    // a member that already has the right size and is valid is left alone,
    // and one that failed a previous attempt gets retried here instead of
    // silently staying broken.
    bool ok = true;
    if (mScene.width != width || mScene.height != height || !mScene.valid())
        ok &= mScene.create(width, height, Format::RGBA16F, Format::Depth24Stencil8, "post.scene",
                            false, Format::RG16F, Format::R8);
    if (mPing[0].width != width || mPing[0].height != height || !mPing[0].valid())
        ok &= mPing[0].create(width, height, Format::RGBA16F, Format::Unknown, "post.ping0");
    if (mPing[1].width != width || mPing[1].height != height || !mPing[1].valid())
        ok &= mPing[1].create(width, height, Format::RGBA16F, Format::Unknown, "post.ping1");
    if (mBloom[0].width != halfWidth || mBloom[0].height != halfHeight || !mBloom[0].valid())
        ok &= mBloom[0].create(halfWidth, halfHeight, Format::RGBA16F, Format::Unknown,
                               "post.bloom0");
    if (mBloom[1].width != halfWidth || mBloom[1].height != halfHeight || !mBloom[1].valid())
        ok &= mBloom[1].create(halfWidth, halfHeight, Format::RGBA16F, Format::Unknown,
                               "post.bloom1");
    if (mAO[0].width != halfWidth || mAO[0].height != halfHeight || !mAO[0].valid())
        ok &= mAO[0].create(halfWidth, halfHeight, Format::R8, Format::Unknown, "post.ao0");
    if (mAO[1].width != halfWidth || mAO[1].height != halfHeight || !mAO[1].valid())
        ok &= mAO[1].create(halfWidth, halfHeight, Format::R8, Format::Unknown, "post.ao1");
    return ok;
}

bool PostProcessStack::begin(u32 width, u32 height, FrameContext& frame, u32 temporalIndex,
                             bool resetTemporalHistory)
{
    if (!resize(width, height))
    {
        Log::error("PostProcess: failed to create %ux%u targets", width, height);
        return false;
    }
    frame.target = mScene.target;
    frame.sceneColor = mScene.color;
    frame.sceneDepth = mScene.depth;
    frame.viewport = {0.0f, 0.0f, static_cast<f32>(width), static_cast<f32>(height)};
    mTemporalIndex = Math::min(temporalIndex, 2u);
    mTemporalAAAllowed = frame.temporalAA;
    // Each stream keeps its history at its own resolution. Streams render
    // interleaved at different sizes (editor viewport and game panel), so a
    // stream's history may only be dropped when that stream itself changes
    // size - resize() touching every stream would invalidate the others each
    // frame and TAA would never accumulate.
    OffscreenTarget* history = mTAAHistory[mTemporalIndex];
    if (history[0].width != width || history[0].height != height ||
        history[1].width != width || history[1].height != height)
    {
        history[0].destroy();
        history[1].destroy();
        mTAAHistoryValid[mTemporalIndex] = false;
    }
    if (resetTemporalHistory)
        mTAAHistoryValid[mTemporalIndex] = false;
    return true;
}

bool PostProcessStack::ensureTAAPipeline()
{
    if (mTAAPipeline.valid())
        return true;
    if (mTAAPipelineFailed)
        return false;
    const std::string& source = Assets().loadShader("taa.comp");
    if (source.empty())
    {
        Log::error("PostProcess: taa.comp is missing");
        mTAAPipelineFailed = true;
        return false;
    }
    PipelineDesc desc;
    desc.cs = {source.c_str(), 0, "taa.comp"};
    desc.debugName = "post.taa";
    mTAAPipeline = GPU::getSingleton().createPipeline(desc);
    mTAAPipelineFailed = !mTAAPipeline.valid();
    return mTAAPipeline.valid();
}

TextureHandle PostProcessStack::resolveTAA(TextureHandle source)
{
    if (!mTemporalAAAllowed || !taaEnabled || !source.valid() || !mScene.velocity.valid() ||
        !mScene.reactive.valid() || !ensureTAAPipeline())
        return source;

    OffscreenTarget& readTarget = mTAAHistory[mTemporalIndex][mTAARead[mTemporalIndex]];
    OffscreenTarget& writeTarget = mTAAHistory[mTemporalIndex][mTAARead[mTemporalIndex] ^ 1u];
    if (!readTarget.valid() || !writeTarget.valid())
    {
        if (!readTarget.create(mScene.width, mScene.height, Format::RGBA16F, Format::Unknown,
                               "post.taa.history", false, Format::Unknown, Format::Unknown, true) ||
            !writeTarget.create(mScene.width, mScene.height, Format::RGBA16F, Format::Unknown,
                                "post.taa.history", false, Format::Unknown, Format::Unknown, true))
        {
            mTAAHistoryValid[mTemporalIndex] = false;
            return source;
        }
    }

    const u32 write = mTAARead[mTemporalIndex] ^ 1u;
    if (!mTAAHistoryValid[mTemporalIndex])
    {
        const Viewport viewport{0.0f, 0.0f, static_cast<f32>(mScene.width),
                                static_cast<f32>(mScene.height)};
        draw(source, TextureHandle(), mTAAHistory[mTemporalIndex][write].target, viewport, 0,
             Math::vec4(0.0f));
        mTAARead[mTemporalIndex] = write;
        mTAAHistoryValid[mTemporalIndex] = true;
        return mTAAHistory[mTemporalIndex][write].color;
    }

    TAABlock block;
    block.sizeAndFeedback = Math::vec4(static_cast<f32>(mScene.width), static_cast<f32>(mScene.height),
                                      taaFeedback, taaMotionFeedback);
    block.clipAndSharpen = Math::vec4(taaClipWidth, taaSharpness, 0.0f, 0.0f);
    GPU& gpu = GPU::getSingleton();
    gpu.updateBuffer(mTAAUniform, 0, sizeof(block), &block);
    gpu.bindTexture(0, source, mSampler);
    gpu.bindTexture(1, readTarget.color, mSampler);
    gpu.bindTexture(2, mScene.velocity, mSampler);
    gpu.bindTexture(3, mScene.reactive, mSampler);
    gpu.bindTexture(4, mScene.depth, mSampler);
    gpu.bindImage(0, writeTarget.color, 0, true);
    gpu.bindUniform(0, mTAAUniform);
    gpu.setPipeline(mTAAPipeline);
    gpu.dispatch((mScene.width + 7) / 8, (mScene.height + 7) / 8, 1);
    gpu.barrier(BarrierImageWrite | BarrierTexture);
    mTAARead[mTemporalIndex] = write;
    return writeTarget.color;
}

TextureHandle PostProcessStack::computeSSAO(const Math::mat4& projection)
{
    if (!enabled || !ssaoEnabled || !mScene.depth.valid())
        return TextureHandle();
    mProjection = projection;
    mInverseProjection = Math::inverse(projection);
    const Viewport viewport{0.0f, 0.0f, static_cast<f32>(mAO[0].width),
                            static_cast<f32>(mAO[0].height)};
    draw(mScene.depth, TextureHandle(), mAO[0].target, viewport, 7,
         Math::vec4(ssaoRadius, ssaoBias, ssaoIntensity, static_cast<f32>(ssaoSamples)));
    if (!ssaoBlur)
        return mAO[0].color;

    draw(mAO[0].color, mScene.depth, mAO[1].target, viewport, 8,
         Math::vec4(ssaoDepthSigma, 0.0f, 0.0f, 0.0f));
    draw(mAO[1].color, mScene.depth, mAO[0].target, viewport, 9,
         Math::vec4(ssaoDepthSigma, 0.0f, 0.0f, 0.0f));
    return mAO[0].color;
}

void PostProcessStack::draw(TextureHandle source, TextureHandle secondary, TargetHandle destination,
                            const Viewport& viewport, u32 mode, const Math::vec4& options)
{
    PostBlock block;
    block.projection = mProjection;
    block.inverseProjection = mInverseProjection;
    block.sizeMode =
        Math::vec4(1.0f / viewport.width, 1.0f / viewport.height, static_cast<f32>(mode), 0.0f);
    block.options = options;
    GPU& gpu = GPU::getSingleton();
    gpu.setTarget(destination);
    gpu.setViewport(viewport);
    gpu.setPipeline(mPipeline);
    gpu.updateBuffer(mUniform, 0, sizeof(block), &block);
    gpu.bindUniform(0, mUniform);
    gpu.bindTexture(0, source, mSampler);
    gpu.bindTexture(1, secondary.valid() ? secondary : source, mSampler);
    DrawDesc command;
    command.count = 3;
    gpu.draw(command);
}

TextureHandle PostProcessStack::runLayers(bool& displayEncoded)
{
    TextureHandle source = ssaoDebug && ssaoEnabled ? mAO[0].color : mScene.color;
    if (!ssaoDebug)
        source = resolveTAA(source);
    u32 ping = 0;
    displayEncoded = false;
    for (const PostLayer& layer : mLayers)
    {
        if (ssaoDebug || !enabled || !layer.enabled)
            continue;
        Viewport viewport{0.0f, 0.0f, static_cast<f32>(mScene.width),
                          static_cast<f32>(mScene.height)};
        if (layer.effect == PostEffect::Bloom)
        {
            Viewport half{0.0f, 0.0f, static_cast<f32>(mBloom[0].width),
                          static_cast<f32>(mBloom[0].height)};
            draw(source, TextureHandle(), mBloom[0].target, half, 3,
                 Math::vec4(bloomThreshold, bloomSoftKnee, 0.0f, 0.0f));
            draw(mBloom[0].color, TextureHandle(), mBloom[1].target, half, 4, Math::vec4(0.0f));
            draw(mBloom[1].color, TextureHandle(), mBloom[0].target, half, 5, Math::vec4(0.0f));
            draw(source, mBloom[0].color, mPing[ping].target, viewport, 6,
                 Math::vec4(bloomStrength, 0.0f, 0.0f, 0.0f));
        }
        else if (layer.effect == PostEffect::ToneMap)
        {
            draw(source, TextureHandle(), mPing[ping].target, viewport, 1,
                 Math::vec4(exposure, static_cast<f32>(toneMap), 0.0f, 0.0f));
            displayEncoded = true;
        }
        else
            draw(source, TextureHandle(), mPing[ping].target, viewport, 2,
                 Math::vec4(fxaaSubpixel, fxaaEdgeThreshold, fxaaEdgeThresholdMin, 1.0f / 8.0f));
        source = mPing[ping].color;
        ping ^= 1;
    }
    return source;
}

void PostProcessStack::resolve(const Rect& destination, u32 windowWidth, u32 windowHeight)
{
    bool displayEncoded = false;
    const TextureHandle source = runLayers(displayEncoded);

    // Gamma encoding is presentation, not an optional artistic effect. The
    // scene target is linear RGBA16F; copying it verbatim to an ordinary
    // window backbuffer makes mid-tones far too dark. When the post stack or
    // its tone-map layer is disabled, mode 1 with no tone curve and unit
    // exposure is what makes "post off" still mean a correctly encoded
    // image - folded straight into this final draw instead of writing it to
    // the ping target first just to copy it back out again right after: with
    // everything disabled that used to mean two full-resolution fullscreen
    // passes to do what one already does everywhere else in this function.
    ClearValue clear;
    clear.bits = ClearColor;
    Viewport viewport{static_cast<f32>(destination.x), static_cast<f32>(destination.y),
                      static_cast<f32>(destination.width), static_cast<f32>(destination.height)};
    GPU::getSingleton().setTarget(TargetHandle(), clear);
    if (!displayEncoded && !ssaoDebug)
        draw(source, TextureHandle(), TargetHandle(), viewport, 1,
             Math::vec4(1.0f, static_cast<f32>(ToneMapMode::None), 0.0f, 0.0f));
    else
        draw(source, TextureHandle(), TargetHandle(), viewport, 0, Math::vec4(0.0f));
    (void)windowWidth;
    (void)windowHeight;
}

TextureHandle PostProcessStack::resolveToTexture(u32 outputIndex, bool applyPostProcess)
{
    if (outputIndex >= 2)
        return TextureHandle();

    OffscreenTarget& resolved = mResolved[outputIndex];
    // Lazily sized to the scene target - the editor's viewport drives that
    // size through Engine::renderToTexture(), so following it here keeps the
    // resolved image 1:1 with what was rendered, no rescale.
    if (resolved.width != mScene.width || resolved.height != mScene.height)
    {
        resolved.destroy();
        if (!resolved.create(mScene.width, mScene.height, Format::RGBA8, Format::Unknown,
                             outputIndex == 0 ? "post.resolved.viewport" : "post.resolved.game"))
            return TextureHandle();
    }

    bool displayEncoded = false;
    const TextureHandle source = applyPostProcess ? runLayers(displayEncoded) : mScene.color;

    ClearValue clear;
    clear.bits = ClearColor;
    Viewport viewport{0.0f, 0.0f, static_cast<f32>(resolved.width),
                      static_cast<f32>(resolved.height)};
    GPU::getSingleton().setTarget(resolved.target, clear);
    // Same two modes as resolve() above, for the same reason - the only
    // difference is where the draw lands.
    if (!displayEncoded && (!ssaoDebug || !applyPostProcess))
        draw(source, TextureHandle(), resolved.target, viewport, 1,
             Math::vec4(1.0f, static_cast<f32>(ToneMapMode::None), 0.0f, 0.0f));
    else
        draw(source, TextureHandle(), resolved.target, viewport, 0, Math::vec4(0.0f));
    return resolved.color;
}

} // namespace Radion
