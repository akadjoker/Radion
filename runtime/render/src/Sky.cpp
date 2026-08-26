#include "PCH.h"

#include "Sky.h"

#include "AssetManager.h"
#include "EnvironmentBlock.h"

namespace Radion
{
namespace
{

constexpr char kSkyVertex[] = R"GLSL(#version 450 core
layout(std140, binding = 0) uniform SkyBlock
{
    mat4 invViewProjection;
    vec4 cameraPosition;
    vec4 sunDirection;
    vec4 sunTransmittance;
    vec4 ambientIntensity;
    vec4 atmosphere;
    vec4 options;
    vec4 cloudGeometry;     // height, scale, coverage, density
    vec4 cloudMotion;       // direction.xy, speed, time
    vec4 cloudColorEnabled; // color.rgb, enabled (0/1)
};
layout(location = 0) out vec3 rayDirection;
out float gl_ClipDistance[1];
void main()
{
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vec2 ndc = p * 2.0 - 1.0;
    vec4 farPoint = invViewProjection * vec4(ndc, 1.0, 1.0);
    rayDirection = farPoint.xyz / farPoint.w - cameraPosition.xyz;
    // GL_CLIP_DISTANCE0 is enabled globally because ordinary geometry uses
    // it in reflection passes. The sky has no world-space clip plane and must
    // explicitly opt out; leaving this built-in unwritten is undefined and
    // clipped large regions of the reflected fullscreen triangle to black.
    gl_ClipDistance[0] = 0.0;
    gl_Position = vec4(ndc, 1.0, 1.0);
}
)GLSL";

constexpr char kSkyFragment[] = R"GLSL(#version 450 core
layout(std140, binding = 0) uniform SkyBlock
{
    mat4 invViewProjection;
    vec4 cameraPosition;
    vec4 sunDirection;
    vec4 sunTransmittance;
    vec4 ambientIntensity;
    vec4 atmosphere;
    vec4 options;
    vec4 cloudGeometry;     // height, scale, coverage, density
    vec4 cloudMotion;       // direction.xy, speed, time
    vec4 cloudColorEnabled; // color.rgb, enabled (0/1)
};
layout(binding = 0) uniform samplerCube uSkyCube;
layout(location = 0) in vec3 rayDirection;
layout(location = 0) out vec4 outColor;
const float PI = 3.14159265359;

vec3 gradientSky(vec3 dir, vec3 sunDir)
{
    float day = smoothstep(-0.08, 0.25, sunDir.y);
    float dusk = 1.0 - smoothstep(0.08, 0.45, abs(sunDir.y));
    vec3 zenith = mix(ambientIntensity.rgb * 0.25, vec3(0.22, 0.45, 0.85), day);
    vec3 horizon = mix(ambientIntensity.rgb * 0.5, vec3(0.72, 0.82, 0.92), day);
    vec3 flatDir = normalize(vec3(dir.x, 0.0, dir.z + 0.0001));
    vec3 flatSun = normalize(vec3(sunDir.x, 0.0, sunDir.z + 0.0001));
    horizon = mix(horizon, vec3(0.98, 0.48, 0.20),
                  dusk * pow(max(dot(flatDir, flatSun), 0.0), 3.0) * 0.85);
    float h = clamp(dir.y, 0.0, 1.0);
    vec3 color = mix(horizon, zenith, pow(h, 0.6));
    if (dir.y < 0.0)
        color = mix(horizon, horizon * 0.55, clamp(-dir.y * 3.0, 0.0, 1.0));
    float d = dot(dir, sunDir);
    float disc = smoothstep(0.9993, 0.9998, d);
    float halo = pow(max(d, 0.0), 400.0) * 0.5 + pow(max(d, 0.0), 32.0) * 0.12;
    vec3 sunColor = mix(vec3(1.0, 0.45, 0.15), vec3(1.0, 0.98, 0.92),
                        smoothstep(0.0, 0.35, sunDir.y));
    return (color + sunColor * (disc * 1.2 + halo) * smoothstep(-0.12, -0.02, sunDir.y)) *
           ambientIntensity.a;
}

bool raySphere(vec3 origin, vec3 direction, float radius, out float nearHit, out float farHit)
{
    float b = dot(origin, direction);
    float c = dot(origin, origin) - radius * radius;
    float discriminant = b * b - c;
    if (discriminant < 0.0)
        return false;
    discriminant = sqrt(discriminant);
    nearHit = -b - discriminant;
    farHit = -b + discriminant;
    return true;
}

vec3 atmosphereSky(vec3 direction, vec3 sunDir)
{
    const float planetRadius = 6360000.0;
    const float atmosphereRadius = 6420000.0;
    const float rayleighHeight = 8000.0;
    const float mieHeight = 1200.0;
    const vec3 betaRayleigh = vec3(5.8e-6, 13.5e-6, 33.1e-6);
    const float betaMie = 21e-6;
    vec3 origin = vec3(0.0, planetRadius + 1000.0, 0.0);
    float nearHit;
    float farHit;
    if (!raySphere(origin, direction, atmosphereRadius, nearHit, farHit))
        return vec3(0.0);
    float maxDistance = farHit;
    float groundNear;
    float groundFar;
    if (raySphere(origin, direction, planetRadius, groundNear, groundFar) && groundNear > 0.0)
        maxDistance = min(maxDistance, groundNear);

    int viewCount = clamp(int(options.z + 0.5), 4, 48);
    int lightCount = clamp(int(options.w + 0.5), 2, 16);
    float viewStep = maxDistance / float(viewCount);
    float distance = viewStep * 0.5;
    float opticalRayleigh = 0.0;
    float opticalMie = 0.0;
    vec3 sumRayleigh = vec3(0.0);
    vec3 sumMie = vec3(0.0);
    for (int i = 0; i < viewCount; ++i)
    {
        vec3 point = origin + direction * distance;
        float height = length(point) - planetRadius;
        float densityRayleigh = exp(-height / rayleighHeight) * viewStep;
        float densityMie = exp(-height / mieHeight) * viewStep;
        opticalRayleigh += densityRayleigh;
        opticalMie += densityMie;

        float lightNear;
        float lightFar;
        raySphere(point, sunDir, atmosphereRadius, lightNear, lightFar);
        float lightStep = lightFar / float(lightCount);
        float lightDistance = lightStep * 0.5;
        float lightRayleigh = 0.0;
        float lightMie = 0.0;
        bool blocked = false;
        for (int j = 0; j < lightCount; ++j)
        {
            vec3 lightPoint = point + sunDir * lightDistance;
            float lightHeight = length(lightPoint) - planetRadius;
            if (lightHeight < 0.0)
            {
                blocked = true;
                break;
            }
            lightRayleigh += exp(-lightHeight / rayleighHeight) * lightStep;
            lightMie += exp(-lightHeight / mieHeight) * lightStep;
            lightDistance += lightStep;
        }
        if (!blocked)
        {
            vec3 extinction = betaRayleigh * atmosphere.x * (opticalRayleigh + lightRayleigh) +
                              betaMie * atmosphere.y * 1.1 * (opticalMie + lightMie);
            vec3 attenuation = exp(-extinction);
            sumRayleigh += densityRayleigh * attenuation;
            sumMie += densityMie * attenuation;
        }
        distance += viewStep;
    }

    float mu = dot(direction, sunDir);
    float rayleighPhase = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
    float g = atmosphere.z;
    float miePhase = (1.0 - g * g) /
        (4.0 * PI * pow(max(1.0 + g * g - 2.0 * g * mu, 0.0001), 1.5));
    vec3 color = options.x * (sumRayleigh * betaRayleigh * atmosphere.x * rayleighPhase +
                              sumMie * betaMie * atmosphere.y * miePhase);
    color += smoothstep(0.9995, 0.9999, mu) * sunTransmittance.rgb * options.x * 0.03;
    return color * atmosphere.w;
}

float cloudHash(vec2 p)
{
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

float cloudValueNoise(vec2 p)
{
    vec2 i = floor(p);
    vec2 f = fract(p);
    float a = cloudHash(i);
    float b = cloudHash(i + vec2(1.0, 0.0));
    float c = cloudHash(i + vec2(0.0, 1.0));
    float d = cloudHash(i + vec2(1.0, 1.0));
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

float cloudFBM(vec2 p)
{
    float value = 0.0;
    float amplitude = 0.5;
    for (int i = 0; i < 4; ++i)
    {
        value += amplitude * cloudValueNoise(p);
        p *= 2.02;
        amplitude *= 0.5;
    }
    return value;
}

vec3 applyClouds(vec3 color, vec3 dir, vec3 sunDir)
{
    if (dir.y <= 0.0 || cloudColorEnabled.a < 0.5)
        return color;

    float t = cloudGeometry.x / dir.y;
    vec2 point = (cameraPosition.xz + dir.xz * t) * cloudGeometry.y +
                 cloudMotion.xy * cloudMotion.z * cloudMotion.w;
    float noise = cloudFBM(point);

    float coverage = clamp(cloudGeometry.z, 0.0, 1.0);
    float density = max(cloudGeometry.w, 0.001);
    float alpha = smoothstep(1.0 - coverage, 1.0 - coverage + density, noise);

    // The sample plane is infinite, so grazing-angle rays near the horizon
    // would otherwise stretch it into a hard band right at dir.y == 0 -
    // fade the layer out before it gets there.
    float fade = smoothstep(0.0, 0.2, dir.y);

    float sunFacing = clamp(dot(dir, sunDir) * 0.5 + 0.5, 0.0, 1.0);
    vec3 lighting = mix(cloudColorEnabled.rgb * 0.6, cloudColorEnabled.rgb, sunFacing);

    return mix(color, lighting, alpha * fade);
}

void main()
{
    vec3 dir = normalize(rayDirection);
    vec3 sunDir = normalize(sunDirection.xyz);
    vec3 color;
    if (options.y < 0.5)
        color = gradientSky(dir, sunDir);
    else if (options.y < 1.5)
        color = atmosphereSky(dir, sunDir);
    else
        color = texture(uSkyCube, dir).rgb;
    color = applyClouds(color, dir, sunDir);
    outColor = vec4(color, 1.0);
}
)GLSL";

struct SkyBlock
{
    Math::Mat4 invViewProjection;
    Math::Vec4 cameraPosition;
    Math::Vec4 sunDirection;
    Math::Vec4 sunTransmittance;
    Math::Vec4 ambientIntensity;
    Math::Vec4 atmosphere;
    Math::Vec4 options;
    Math::Vec4 cloudGeometry;
    Math::Vec4 cloudMotion;
    Math::Vec4 cloudColorEnabled;
};

class SkyPass final : public RenderTechnique
{
public:
    const char* name() const override
    {
        return "Sky";
    }

    bool setup() override
    {
        GPU& gpu = GPU::getSingleton();
        PipelineDesc pipeline;
        pipeline.vs = {kSkyVertex, 0, "sky.vert"};
        pipeline.fs = {kSkyFragment, 0, "sky.frag"};
        pipeline.depth.test = true;
        pipeline.depth.write = false;
        pipeline.depth.func = Compare::LessEqual;
        pipeline.raster.cull = CullMode::None;
        pipeline.debugName = "sky";
        mPipeline = gpu.createPipeline(pipeline);
        BufferDesc uniform;
        uniform.size = sizeof(SkyBlock);
        uniform.usage = BufferUniform;
        uniform.residency = Residency::Stream;
        uniform.debugName = "sky.uniform";
        mUniform = gpu.createBuffer(uniform);

        SamplerDesc samplerDesc;
        samplerDesc.filter = Filter::Linear;
        samplerDesc.wrapU = Wrap::Clamp;
        samplerDesc.wrapV = Wrap::Clamp;
        samplerDesc.wrapW = Wrap::Clamp;
        mSampler = Assets().getSampler(samplerDesc);

        // Bound whenever the mode isn't Cubemap (or the handle isn't ready
        // yet): GL considers a sampler "used" as soon as the shader
        // references it, so the draw needs something valid on unit 0
        // regardless of which branch main() actually takes. One pixel
        // repeated six times, not one - a cube upload reads all six faces
        // from a single buffer, so a single-pixel source is short five faces.
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
        placeholder.debugName = "sky.placeholder_cube";
        mPlaceholderCube = gpu.createTexture(placeholder);

        return mPipeline.valid() && mUniform.valid() && mPlaceholderCube.valid();
    }

    void execute(const FrameContext& frame) override
    {
        if (!frame.sky || !frame.sky->enabled)
            return;
        const SkySettings& sky = *frame.sky;
        const bool cubemapReady = sky.mode == SkyMode::Cubemap && sky.cubemap.valid();
        // Falls back to the gradient sky rather than drawing black when the
        // mode says Cubemap but nothing valid was ever loaded into it.
        const f32 modeOption = cubemapReady ? 2.0f
                              : sky.mode == SkyMode::Atmosphere ? 1.0f
                                                                : 0.0f;
        SkyBlock block;
        block.invViewProjection = glm::inverse(frame.viewProjection);
        block.cameraPosition = Math::Vec4(frame.cameraPosition, 1.0f);
        block.sunDirection = Math::Vec4(glm::normalize(sky.sunDirection), 0.0f);
        block.sunTransmittance = Math::Vec4(sky.sunTransmittance, 1.0f);
        block.ambientIntensity = Math::Vec4(sky.ambient, sky.intensity);
        block.atmosphere = Math::Vec4(sky.rayleigh, sky.mie, sky.mieG, sky.atmosphereExposure);
        block.options = Math::Vec4(sky.sunIntensity, modeOption, static_cast<f32>(sky.viewSteps),
                                  static_cast<f32>(sky.lightSteps));
        block.cloudGeometry =
            Math::Vec4(sky.cloudHeight, sky.cloudScale, sky.cloudCoverage, sky.cloudDensity);
        block.cloudMotion = Math::Vec4(sky.cloudDirection, sky.cloudSpeed, frame.time);
        block.cloudColorEnabled = Math::Vec4(sky.cloudColor, sky.cloudsEnabled ? 1.0f : 0.0f);
        GPU& gpu = GPU::getSingleton();
        gpu.setTarget(frame.target);
        gpu.setViewport(frame.viewport);
        gpu.setPipeline(mPipeline);
        gpu.updateBuffer(mUniform, 0, sizeof(block), &block);
        gpu.bindUniform(0, mUniform);
        gpu.bindTexture(0, cubemapReady ? sky.cubemap : mPlaceholderCube, mSampler);
        DrawDesc draw;
        draw.count = 3;
        gpu.draw(draw);
    }

    void shutdown() override
    {
        GPU& gpu = GPU::getSingleton();
        gpu.destroy(mPipeline);
        gpu.destroy(mUniform);
        gpu.destroy(mPlaceholderCube);
        mPipeline = PipelineHandle();
        mUniform = BufferHandle();
        mPlaceholderCube = TextureHandle();
    }

private:
    PipelineHandle mPipeline;
    BufferHandle mUniform;
    SamplerHandle mSampler;
    TextureHandle mPlaceholderCube;
};

} // namespace

EnvironmentBlock environmentFromSky(const SkySettings* sky)
{
    EnvironmentBlock environment;
    if (!sky || !sky->enabled)
        return environment;

    environment.sunDirection = Math::Vec4(-sky->sunDirection, 0.0f);
    environment.sunColor = Math::Vec4(sky->sunTransmittance, 1.0f);
    environment.ambient = Math::Vec4(sky->ambient * sky->ambientStrength * sky->intensity,
                                    sky->lightmapIntensity);
    environment.timeAndUnused.y = sky->lightmapShadowLift;
    return environment;
}

EnvironmentBlock environmentForFrame(const FrameContext& frame)
{
    EnvironmentBlock environment = environmentFromSky(frame.sky);

    // The probe travels in the frame, not in the sky: it is captured by a
    // pass, and its placement has nothing to do with the time of day.
    environment.probePositionAndMips =
        Math::Vec4(frame.environmentProbePosition,
                  static_cast<f32>(glm::max(frame.environmentProbeMips, 1u)));
    environment.probeExtentsAndIntensity =
        Math::Vec4(frame.environmentProbeExtents,
                  frame.environmentCube.valid() ? frame.environmentProbeIntensity : 0.0f);
    environment.timeAndUnused = Math::Vec4(frame.time, environment.timeAndUnused.y, 0.0f, 0.0f);

    if (!frame.list)
        return environment;

    if (const RenderLight* sun = frame.list->sun())
    {
        environment.sunDirection = Math::Vec4(glm::normalize(sun->direction), 0.0f);
        environment.sunColor = Math::Vec4(sun->color, 1.0f);
    }
    return environment;
}

bool loadSkyCubemap(SkySettings& sky, const std::string& baseName)
{
    if (baseName.empty())
    {
        sky.cubemap = TextureHandle();
        sky.cubemapName.clear();
        return true;
    }

    const TextureHandle handle = Assets().loadCubemap(baseName, ColorSpace::sRGB);
    if (!handle.valid())
        return false;

    sky.cubemap = handle;
    sky.cubemapName = baseName;
    return true;
}

RenderTechnique* createSkyPass()
{
    return new SkyPass();
}

void SkySettings::updateSun()
{
    if (automaticSun)
    {
        timeOfDay = glm::mod(timeOfDay, 24.0f);
        if (timeOfDay < 0.0f)
            timeOfDay += 24.0f;
        const f32 phase = glm::two_pi<f32>() * (timeOfDay - 6.0f) / 24.0f;
        sunElevation = glm::sin(phase) * maximumElevation;
        sunAzimuth = glm::mod(90.0f + (timeOfDay - 6.0f) * 15.0f + northOffset, 360.0f);
    }
    const f32 azimuth = glm::radians(sunAzimuth);
    const f32 elevation = glm::radians(sunElevation);
    const f32 horizontal = glm::cos(elevation);
    sunDirection = glm::normalize(Math::Vec3(glm::sin(azimuth) * horizontal,
                                            glm::sin(elevation),
                                            glm::cos(azimuth) * horizontal));

    constexpr f32 planetRadius = 6360000.0f;
    constexpr f32 atmosphereRadius = 6420000.0f;
    constexpr f32 rayleighHeight = 8000.0f;
    constexpr f32 mieHeight = 1200.0f;
    constexpr f32 betaMie = 21e-6f;
    const Math::Vec3 betaRayleigh(5.8e-6f, 13.5e-6f, 33.1e-6f);
    if (sunDirection.y < -0.05f)
    {
        sunTransmittance = Math::Vec3(0.0f);
        return;
    }
    const Math::Vec3 origin(0.0f, planetRadius + 1000.0f, 0.0f);
    const f32 b = glm::dot(origin, sunDirection);
    const f32 c = glm::dot(origin, origin) - atmosphereRadius * atmosphereRadius;
    const f32 discriminant = b * b - c;
    if (discriminant < 0.0f)
    {
        sunTransmittance = Math::Vec3(0.0f);
        return;
    }
    constexpr u32 samples = 32;
    const f32 top = -b + glm::sqrt(discriminant);
    const f32 step = top / samples;
    f32 opticalRayleigh = 0.0f;
    f32 opticalMie = 0.0f;
    for (u32 i = 0; i < samples; ++i)
    {
        const Math::Vec3 point = origin + sunDirection * (step * (static_cast<f32>(i) + 0.5f));
        const f32 height = glm::length(point) - planetRadius;
        if (height < 0.0f)
        {
            sunTransmittance = Math::Vec3(0.0f);
            return;
        }
        opticalRayleigh += glm::exp(-height / rayleighHeight) * step;
        opticalMie += glm::exp(-height / mieHeight) * step;
    }
    const Math::Vec3 extinction = betaRayleigh * rayleigh * opticalRayleigh +
                                 Math::Vec3(betaMie * mie * opticalMie);
    sunTransmittance = glm::exp(-extinction);
}

} // namespace Radion
