#include "PCH.h"

#include "OceanRender.h"

#include "AssetManager.h"
#include "EnvironmentBlock.h"
#include "GPUProfiler.h"
#include "Material.h"
#include "OffscreenTarget.h"
#include "Profiler.h"
#include "Sky.h"

namespace Radion
{

namespace
{

// Named render target the reflection tier and up bind by name, the way
// WaterPass resolves kReflectionTargetName. Not shared with WaterPass: the
// ocean plane is its own draw, but the reflection texture underneath it is
// the very same one - whoever renders the mirrored sub-pass this frame
// publishes once and both passes can read it.
constexpr const char* kOceanReflectionTarget = kReflectionTargetName;

// Mirrors OceanUniforms in ocean_uniforms.glsl, field for field. std140 pairs
// each vec3 with the scalar that follows it - see GrassUniforms for the same
// convention.
struct alignas(16) OceanUniforms
{
    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 viewProjection = glm::mat4(1.0f);
    glm::mat4 reflectionViewProjection = glm::mat4(1.0f);
    glm::mat4 invViewProjection = glm::mat4(1.0f);

    glm::vec3 viewPosition = glm::vec3(0.0f);
    f32 time = 0.0f;

    glm::vec3 lightDirection = glm::vec3(0.0f, -1.0f, 0.0f);
    f32 timeScale = 1.0f;

    glm::vec3 lightColor = glm::vec3(1.0f);
    f32 steepness = 0.55f;

    glm::vec3 ambient = glm::vec3(0.2f);
    f32 skyIntensity = 1.0f;

    glm::vec4 waves[kOceanMaxWaves];
    s32 waveCount = 0;
    f32 pad0 = 0.0f, pad1 = 0.0f, pad2 = 0.0f;

    glm::vec3 shallowColor = glm::vec3(0.28f, 0.55f, 0.55f);
    f32 absorptionDistance = 28.0f;

    glm::vec3 deepColor = glm::vec3(0.02f, 0.11f, 0.20f);
    f32 roughness = 0.06f;

    f32 fresnelDetail = 0.25f;
    f32 fresnelMax = 1.0f;
    f32 minOpacity = 0.45f;
    f32 specularStrength = 0.6f;

    f32 normalScale1 = 0.038f;
    f32 normalScale2 = 0.0067f;
    f32 normalStrength = 0.1f;
    f32 normalSpeed1 = 0.35f;

    f32 normalSpeed2 = 0.55f;
    s32 normalOctaves = 4;
    s32 hasNormalMap = 1;
    s32 hasFoam = 1;

    f32 foamScale = 0.035f;
    f32 foamStrength = 1.0f;
    f32 foamDepth = 6.0f;
    f32 foamCrest = 0.0f;

    f32 reflectionDistortion = 0.035f;
    s32 debugMode = 0;
    glm::vec2 screenSize = glm::vec2(1.0f);

    glm::vec3 underwaterColor = glm::vec3(0.06f, 0.22f, 0.30f);
    s32 underwaterCamera = 0;

    s32 hasSkyCube = 0;
    f32 fresnelBias = 0.10f;
    f32 fresnelScale = 0.90f;
    f32 fresnelPower = 4.0f;

    glm::vec4 opticalStrengths = glm::vec4(1.0f);
};

// GLSL requires #version to stay the file's first line, so a variant #define
// cannot simply be prepended - it goes right after it, the same trick
// MaterialManager::withVariantDefines uses for materials.
std::string withOceanDefines(const std::string& source, OceanQuality quality)
{
    const usize versionEnd = source.find('\n');
    std::string result = source.substr(0, versionEnd + 1);
    if (quality == OceanQuality::Reflection || quality == OceanQuality::ReflectionRefraction)
        result += "#define OCEAN_HAS_REFLECTION 1\n";
    if (quality == OceanQuality::ReflectionRefraction)
        result += "#define OCEAN_HAS_DEPTH 1\n";
    result += source.substr(versionEnd + 1);
    return result;
}

class OceanPass final : public RenderTechnique
{
public:
    const char* name() const override
    {
        return "Ocean";
    }

    bool setup() override
    {
        GPU& gpu = GPU::getSingleton();

        BufferDesc uniformBuffer;
        uniformBuffer.size = sizeof(OceanUniforms);
        uniformBuffer.usage = BufferUniform;
        uniformBuffer.residency = Residency::Stream;
        uniformBuffer.debugName = "ocean.uniforms";
        mUniforms = gpu.createBuffer(uniformBuffer);

        SamplerDesc detailSampler;
        detailSampler.filter = Filter::Trilinear;
        detailSampler.wrapU = Wrap::Repeat;
        detailSampler.wrapV = Wrap::Repeat;
        mDetailSampler = gpu.createSampler(detailSampler);

        SamplerDesc targetSampler;
        targetSampler.filter = Filter::Linear;
        targetSampler.wrapU = Wrap::Clamp;
        targetSampler.wrapV = Wrap::Clamp;
        mTargetSampler = gpu.createSampler(targetSampler);

        SamplerDesc cubeSampler;
        cubeSampler.filter = Filter::Linear;
        cubeSampler.wrapU = Wrap::Clamp;
        cubeSampler.wrapV = Wrap::Clamp;
        cubeSampler.wrapW = Wrap::Clamp;
        mCubeSampler = gpu.createSampler(cubeSampler);

        // Bound whenever the frame has no sky cube of its own: GL counts a
        // samplerCube as used the moment the shader names it, so the draw
        // needs something valid on that unit even down the branch that never
        // reads it. Six pixels and not one - a cube upload reads all six faces
        // out of a single buffer.
        const u8 black[4 * 6] = {0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 255,
                                 0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 255};
        TextureDesc placeholder;
        placeholder.type = TextureType::TexCube;
        placeholder.format = Format::RGBA8;
        placeholder.width = 1;
        placeholder.height = 1;
        placeholder.depth = 6;
        placeholder.usage = TextureSampled;
        placeholder.data = black;
        placeholder.debugName = "ocean.placeholder_cube";
        mPlaceholderCube = gpu.createTexture(placeholder);

        return mUniforms.valid() && mPlaceholderCube.valid();
    }

    void execute(const FrameContext& frame) override
    {
        const std::vector<OceanDrawCommand>& commands = OceanDraws().commands();
        if (commands.empty())
            return;

        // A clip plane means this is a mirrored sub-pass, and the surface must
        // not appear in its own reflection. The queue is global rather than
        // per-list, so this is the only place that can tell the two apart.
        if (frame.clipPlane != glm::vec4(0.0f))
            return;

        if (!ensurePipelines())
            return;

        RADION_PROFILE_SCOPE("Ocean");
        RADION_GPU_PROFILE_SCOPE("Ocean");

        GPU& gpu = GPU::getSingleton();

        bool depthReady = false;
        for (const OceanDrawCommand& command : commands)
            depthReady |= command.quality == OceanQuality::ReflectionRefraction;
        if (depthReady)
            depthReady = resolveSceneCopy(gpu, frame);

        gpu.setTarget(frame.target);
        gpu.setViewport(frame.viewport);

        for (const OceanDrawCommand& command : commands)
            draw(gpu, frame, command, depthReady);
    }

    void shutdown() override
    {
        GPU& gpu = GPU::getSingleton();
        gpu.destroy(mPipelines[0]);
        gpu.destroy(mPipelines[1]);
        gpu.destroy(mPipelines[2]);
        gpu.destroy(mUniforms);
        gpu.destroy(mPlaceholderCube);
        mSceneCopy.destroy();
        mPipelinesReady = false;
        mPipelinesFailed = false;
        OceanDraws().clear();
    }

private:
 
    bool ensurePipelines()
    {
        if (mPipelinesReady)
            return true;
        if (mPipelinesFailed)
            return false;

        GPU& gpu = GPU::getSingleton();
        AssetManager& assets = Assets();

        const std::string& vertexSource = assets.loadShader("shaders/ocean.vert");
        const std::string& fragmentSource = assets.loadShader("shaders/ocean.frag");
        if (vertexSource.empty() || fragmentSource.empty())
        {
            Log::error("Ocean: shaders are missing; the surface will not draw");
            mPipelinesFailed = true;
            return false;
        }

        // Positions only (Mesh::depthLayout): the wave sum reads nothing else
        // from the vertex, the same stream Terrain's depth/shadow draws use.
        VertexLayout layout;
        layout.streamCount = 1;
        layout.streams[0].stride = sizeof(glm::vec3);
        layout.attribCount = 1;
        layout.attribs[0] = {0, 0, 0, AttribFormat::Float3};

        const OceanQuality tiers[3] = {OceanQuality::SkyOnly, OceanQuality::Reflection,
                                       OceanQuality::ReflectionRefraction};
        for (u32 i = 0; i < 3; ++i)
        {
            const std::string vs = withOceanDefines(vertexSource, tiers[i]);
            const std::string fs = withOceanDefines(fragmentSource, tiers[i]);

            PipelineDesc desc;
            desc.vs = {vs.c_str(), 0, "ocean.vert"};
            desc.fs = {fs.c_str(), 0, "ocean.frag"};
            desc.layout = layout;
            // Seen from both sides: a mirrored reflection sub-pass flips
            // winding, and the ocean plane itself is drawn once for every
            // camera, never culled from below when the demo dips under it.
            desc.raster.cull = CullMode::None;
            desc.blend.mode = BlendMode::Alpha;
            desc.debugName = "ocean.draw";
            mPipelines[i] = gpu.createPipeline(desc);
            if (!mPipelines[i].valid())
            {
                Log::error("Ocean: pipeline failed to compile for quality tier %u", i);
                mPipelinesFailed = true;
                return false;
            }
        }

        mPipelinesReady = true;
        return true;
    }

 
    bool resolveSceneCopy(GPU& gpu, const FrameContext& frame)
    {
        const u32 width = frame.width ? frame.width : static_cast<u32>(frame.viewport.width);
        const u32 height = frame.height ? frame.height : static_cast<u32>(frame.viewport.height);
        if (width == 0 || height == 0)
            return false;

 
        GPU& device = GPU::getSingleton();
        TextureDesc colorInfo;
        TextureDesc depthInfo;
        if (!frame.sceneColor.valid() || !frame.sceneDepth.valid() ||
            !device.textureInfo(frame.sceneColor, colorInfo) ||
            !device.textureInfo(frame.sceneDepth, depthInfo))
            return false;

        if (mSceneCopy.width != width || mSceneCopy.height != height ||
            mCopyColorFormat != colorInfo.format || mCopyDepthFormat != depthInfo.format)
        {
            mSceneCopy.destroy();
            if (!mSceneCopy.create(width, height, colorInfo.format, depthInfo.format,
                                   "ocean.scene_copy"))
            {
                Log::error("Ocean: scene copy failed; refraction is off this frame");
                return false;
            }
            mCopyColorFormat = colorInfo.format;
            mCopyDepthFormat = depthInfo.format;
        }

        const Rect rect{0, 0, static_cast<s32>(width), static_cast<s32>(height)};
        gpu.blitTarget(mSceneCopy.target, frame.target, rect, rect, true);
        gpu.blitTarget(mSceneCopy.target, frame.target, rect, rect, false);
        Assets().publishRenderTarget(kOceanSceneColorDebugTargetName, mSceneCopy.color);
        Assets().publishRenderTarget(kOceanSceneDepthDebugTargetName, mSceneCopy.depth);
        return true;
    }

    static u32 tierIndex(OceanQuality quality)
    {
        return static_cast<u32>(quality) < 3 ? static_cast<u32>(quality) : 0;
    }

    void draw(GPU& gpu, const FrameContext& frame, const OceanDrawCommand& command,
              bool depthReady)
    {
        const Mesh* mesh = Assets().getMesh(command.mesh);
        if (!mesh || !mesh->positionBuffer.valid())
            return;

        OceanUniforms uniforms;
        uniforms.model = command.model;
        uniforms.viewProjection = frame.viewProjection;
        uniforms.reflectionViewProjection = frame.reflectionViewProj;
        uniforms.invViewProjection = glm::inverse(frame.viewProjection);
        uniforms.viewPosition = frame.cameraPosition;
        uniforms.time = frame.time;
        uniforms.timeScale = command.timeScale;
        uniforms.steepness = command.steepness;

        const EnvironmentBlock environment = environmentForFrame(frame);
        uniforms.lightDirection = glm::vec3(environment.sunDirection);
        uniforms.lightColor = glm::vec3(environment.sunColor);
        uniforms.ambient = glm::vec3(environment.ambient);
        uniforms.skyIntensity = (frame.sky && frame.sky->enabled) ? frame.sky->intensity : 1.0f;

        const u32 waveCount = glm::min(command.waveCount, kOceanMaxWaves);
        uniforms.waveCount = static_cast<s32>(waveCount);
        for (u32 i = 0; i < waveCount; ++i)
        {
            const OceanWave& wave = command.waves[i];
            uniforms.waves[i] = glm::vec4(wave.direction, wave.wavelength, wave.amplitude);
        }

        uniforms.shallowColor = command.shallowColor;
        uniforms.deepColor = command.deepColor;
        uniforms.absorptionDistance = command.absorptionDistance;
        uniforms.roughness = command.roughness;
        uniforms.specularStrength = command.specularStrength;

        uniforms.hasNormalMap = command.hasNormalMap ? 1 : 0;
        uniforms.normalOctaves = static_cast<s32>(command.normalOctaves);
        uniforms.normalScale1 = command.normalScale1;
        uniforms.normalScale2 = command.normalScale2;
        uniforms.normalStrength = command.normalStrength;
        uniforms.normalSpeed1 = command.normalSpeed1;
        uniforms.normalSpeed2 = command.normalSpeed2;

        uniforms.hasFoam = command.hasFoam ? 1 : 0;
        uniforms.foamScale = command.foamScale;
        uniforms.foamStrength = command.foamStrength;
        uniforms.foamDepth = command.foamDepth;
        uniforms.foamCrest = command.foamCrest;

        uniforms.fresnelDetail = command.fresnelDetail;
        uniforms.fresnelMax = command.fresnelMax;
        uniforms.fresnelBias = command.fresnelBias;
        uniforms.fresnelScale = command.fresnelScale;
        uniforms.fresnelPower = command.fresnelPower;
        uniforms.minOpacity = command.minOpacity;
        uniforms.reflectionDistortion = command.reflectionDistortion;
        uniforms.opticalStrengths = glm::vec4(command.reflectionStrength,
                                              command.refractionStrength,
                                              command.colorStrength, 0.0f);
        uniforms.debugMode = command.debugMode;
        uniforms.screenSize = glm::vec2(frame.viewport.width, frame.viewport.height);

        // Below the surface the shader takes the Snell's-window path instead.
        // Measured against the plane's own height rather than against anything
        // the caller passes: the surface knows where it is, and a demo that
        // had to tell it would be one more thing to get out of step. The wave
        // amplitude is not subtracted - a crest passing over the camera is a
        // frame of the wrong path, against a hard switch every frame the
        // camera sits near the surface.
        const f32 waterLevel = command.model[3].y;
        uniforms.underwaterCamera = frame.cameraPosition.y < waterLevel ? 1 : 0;
        uniforms.underwaterColor = command.underwaterColor;

        // Whichever cube the frame is actually showing: an environment probe
        // if one was captured, otherwise the sky's own cubemap. Anything else
        // and the water reflects a sky nobody can see.
        TextureHandle skyCube = frame.environmentCube;
        if (!skyCube.valid() && frame.sky && frame.sky->enabled &&
            frame.sky->mode == SkyMode::Cubemap)
            skyCube = frame.sky->cubemap;
        uniforms.hasSkyCube = skyCube.valid() ? 1 : 0;

        gpu.updateBuffer(mUniforms, 0, sizeof(OceanUniforms), &uniforms);
        gpu.bindUniform(0, mUniforms);

        AssetManager& assets = Assets();
        if (command.normalMap.valid())
            gpu.bindTexture(0, command.normalMap, mDetailSampler);
        if (command.foamMap.valid())
            gpu.bindTexture(1, command.foamMap, mDetailSampler);

        if (command.quality == OceanQuality::Reflection ||
            command.quality == OceanQuality::ReflectionRefraction)
        {
            const TextureHandle reflection =
                assets.resolveRenderTarget(hashName(kOceanReflectionTarget));
            if (reflection.valid())
                gpu.bindTexture(2, reflection, mTargetSampler);
        }

        if (command.quality == OceanQuality::ReflectionRefraction && depthReady)
        {
            gpu.bindTexture(3, mSceneCopy.depth, mTargetSampler);
            gpu.bindTexture(5, mSceneCopy.color, mTargetSampler);
        }

        gpu.bindTexture(4, skyCube.valid() ? skyCube : mPlaceholderCube, mCubeSampler);

        gpu.setPipeline(mPipelines[tierIndex(command.quality)]);

        DrawDesc draw;
        draw.vertexBuffers[0] = mesh->positionBuffer;
        draw.vertexBufferCount = 1;
        draw.indexBuffer = mesh->indexBuffer;
        draw.indexType = mesh->indexType;
        draw.count = mesh->indexCount;
        gpu.draw(draw);
    }

    BufferHandle mUniforms;
    SamplerHandle mDetailSampler;
    SamplerHandle mTargetSampler;
    SamplerHandle mCubeSampler;
    OffscreenTarget mSceneCopy;
    Format mCopyColorFormat = Format::Unknown;
    Format mCopyDepthFormat = Format::Unknown;
    TextureHandle mPlaceholderCube;
    PipelineHandle mPipelines[3];
    bool mPipelinesReady = false;
    bool mPipelinesFailed = false;
};

} // namespace

OceanRenderQueue& OceanRenderQueue::getSingleton()
{
    static OceanRenderQueue queue;
    return queue;
}

void OceanRenderQueue::clear()
{
    mCommands.clear();
}

void OceanRenderQueue::submit(const OceanDrawCommand& command)
{
    mCommands.push_back(command);
}

const std::vector<OceanDrawCommand>& OceanRenderQueue::commands() const
{
    return mCommands;
}

OceanRenderQueue& OceanDraws()
{
    return OceanRenderQueue::getSingleton();
}

RenderTechnique* createOceanPass()
{
    return new OceanPass();
}

} // namespace Radion
