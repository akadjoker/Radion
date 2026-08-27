#include "PCH.h"

#include "EnvironmentProbe.h"

#include "AssetManager.h"
#include "Log.h"

#include "Math.h"

namespace Radion
{

namespace
{

// GL's cube face order is +X, -X, +Y, -Y, +Z, -Z, and its cube texture space
// has Y running DOWN - which is why five of these six up vectors are negative
// Y or its equivalent. These are the conventional capture matrices; whether
// the result lands mirrored is not something to reason about, it is what
// Content::FaceColors is for.
// Plain floats rather than Math::vec3: this math routine does not need vectors.
// constexpr, so a constexpr table has to be built out of scalars.
struct FaceBasis
{
    f32 forward[3];
    f32 up[3];
};

constexpr FaceBasis kFaces[EnvironmentProbe::FaceCount] = {
    {{1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}},  // +X
    {{-1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}}, // -X
    {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},   // +Y
    {{0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, -1.0f}}, // -Y
    {{0.0f, 0.0f, 1.0f}, {0.0f, -1.0f, 0.0f}},  // +Z
    {{0.0f, 0.0f, -1.0f}, {0.0f, -1.0f, 0.0f}}, // -Z
};

Math::vec3 toVector(const f32 (&values)[3])
{
    return Math::vec3(values[0], values[1], values[2]);
}

// +X/+Y/+Z red/green/blue, and their opposites the complementary colours, so
// a glance at a mirror sphere says which way each face went.
constexpr f32 kFaceColors[EnvironmentProbe::FaceCount][3] = {
    {1.0f, 0.1f, 0.1f}, // +X red
    {0.1f, 1.0f, 1.0f}, // -X cyan
    {0.1f, 1.0f, 0.1f}, // +Y green
    {1.0f, 0.1f, 1.0f}, // -Y magenta
    {0.1f, 0.1f, 1.0f}, // +Z blue
    {1.0f, 1.0f, 0.1f}, // -Z yellow
};

} // namespace

bool EnvironmentProbe::create(u32 resolution)
{
    shutdown();
    if (resolution == 0)
        return false;

    GPU& gpu = GPU::getSingleton();

    TextureDesc desc;
    desc.type = TextureType::TexCube;
    desc.width = resolution;
    desc.height = resolution;
    desc.depth = FaceCount;
    // HDR, like the reference's own probes: a sky carries values well past 1
    // and a reflection that clipped them would lose the sun entirely. It uses
    // BC6H (compressed HDR); RGBA16F is the uncompressed equivalent and needs
    // no compression pass to exist first.
    desc.format = Format::RGBA16F;
    // 0 asks for the full chain, per TextureDesc's own convention - and the
    // chain is not decoration here, it is the roughness axis.
    desc.mips = 0;
    desc.usage = TextureSampled | TextureTarget;
    desc.debugName = "probe.cubemap";
    mCubemap = gpu.createTexture(desc);
    if (!mCubemap.valid())
    {
        Log::error("EnvironmentProbe: could not create the %ux%u cubemap", resolution, resolution);
        return false;
    }

    TextureDesc info;
    mResolution = resolution;
    mMipCount = gpu.textureInfo(mCubemap, info) ? info.mips : 1;

    TextureDesc depthDesc;
    depthDesc.type = TextureType::Tex2D;
    depthDesc.format = Format::Depth24Stencil8;
    depthDesc.width = resolution;
    depthDesc.height = resolution;
    depthDesc.mips = 1;
    depthDesc.usage = TextureTarget;
    depthDesc.debugName = "probe.depth";
    mDepth = gpu.createTexture(depthDesc);
    if (!mDepth.valid())
    {
        Log::error("EnvironmentProbe: could not create the capture depth buffer");
        shutdown();
        return false;
    }

    for (u32 face = 0; face < FaceCount; ++face)
    {
        TargetDesc target;
        target.colors[0].texture = mCubemap;
        target.colors[0].mip = 0;
        target.colors[0].slice = face;
        target.colorCount = 1;
        target.depth.texture = mDepth;
        target.debugName = "probe.face";
        mFaceTargets[face] = gpu.createTarget(target);
        if (!mFaceTargets[face].valid())
        {
            Log::error("EnvironmentProbe: could not attach cube face %u", face);
            shutdown();
            return false;
        }
    }

    SamplerDesc samplerDesc;
    // Trilinear: the roughness lookup reads BETWEEN mips, so a sampler that
    // snapped to the nearest level would step visibly as roughness changes.
    samplerDesc.filter = Filter::Trilinear;
    samplerDesc.wrapU = Wrap::Clamp;
    samplerDesc.wrapV = Wrap::Clamp;
    samplerDesc.wrapW = Wrap::Clamp;
    mSampler = Assets().getSampler(samplerDesc);

    mDirty = true;
    mCaptureRequested = false;
    mAccumulator = 0.0f;
    mCaptureCount = 0;
    mLastCaptureCostMilliseconds = 0.0f;
    mLastCaptureTimeSeconds = -1.0f;
    Log::info("EnvironmentProbe: %ux%u cubemap, %u mip(s)", resolution, resolution, mMipCount);
    return true;
}

void EnvironmentProbe::shutdown()
{
    // tryGet(), not getSingleton(): Engine's own probe is shut down at the
    // right point in Engine::shutdown(), before the GPU device goes away,
    // but a ReflectionProbe (scene/) is owned by a GameObject, and a Scene
    // outlives engine.shutdown() in every demo's main() - its destructor,
    // and every component destructor under it, runs after the device is
    // already gone. Nothing left to destroy at that point anyway.
    GPU* gpu = GPU::tryGet();
    if (!gpu)
    {
        mFaceTargets[0] = mFaceTargets[1] = mFaceTargets[2] = TargetHandle();
        mFaceTargets[3] = mFaceTargets[4] = mFaceTargets[5] = TargetHandle();
        mCubemap = TextureHandle();
        mDepth = TextureHandle();
        mResolution = 0;
        mMipCount = 0;
        mDirty = true;
        mAccumulator = 0.0f;
        return;
    }
    for (u32 face = 0; face < FaceCount; ++face)
    {
        if (mFaceTargets[face].valid())
            gpu->destroy(mFaceTargets[face]);
        mFaceTargets[face] = TargetHandle();
    }
    if (mCubemap.valid())
        gpu->destroy(mCubemap);
    mCubemap = TextureHandle();
    if (mDepth.valid())
        gpu->destroy(mDepth);
    mDepth = TextureHandle();
    mResolution = 0;
    mMipCount = 0;
    mDirty = true;
    mAccumulator = 0.0f;
}

bool EnvironmentProbe::ready() const
{
    return mCubemap.valid();
}

void EnvironmentProbe::invalidate()
{
    mDirty = true;
}

void EnvironmentProbe::requestCapture()
{
    mCaptureRequested = true;
}

bool EnvironmentProbe::consumeCapture(f32 deltaTime, bool deferred)
{
    if (!ready() || !enabled)
        return false;

    bool timedDue = false;
    if (refresh == Refresh::Timed)
    {
        mAccumulator += Math::max(deltaTime, 0.0f);
        if (mAccumulator >= Math::max(interval, 0.0f))
        {
            mAccumulator = 0.0f;
            timedDue = true;
        }
    }

    const bool due = mCaptureRequested || (refresh == Refresh::Automatic && mDirty) || timedDue;
    if (!due || deferred)
    {
        // A timed deadline also has to survive deferral. The same one-bit
        // request coalesces every deadline that passes while navigation is
        // active into one capture when interaction stops.
        if (timedDue)
            mCaptureRequested = true;
        return false;
    }

    mDirty = false;
    mCaptureRequested = false;
    return true;
}

void EnvironmentProbe::recordCapture(f32 elapsedMilliseconds, f32 timeSeconds)
{
    ++mCaptureCount;
    mLastCaptureCostMilliseconds = elapsedMilliseconds;
    mLastCaptureTimeSeconds = timeSeconds;
}

TextureHandle EnvironmentProbe::cubemap() const
{
    return mCubemap;
}

SamplerHandle EnvironmentProbe::sampler() const
{
    return mSampler;
}

TargetHandle EnvironmentProbe::faceTarget(u32 face) const
{
    return face < FaceCount ? mFaceTargets[face] : TargetHandle();
}

u32 EnvironmentProbe::resolution() const
{
    return mResolution;
}

u32 EnvironmentProbe::mipCount() const
{
    return mMipCount;
}

void EnvironmentProbe::faceViewProjections(Math::mat4 out[6]) const
{
    // 90 degrees, square aspect: six of them tile the whole sphere of
    // directions exactly, which is the entire point of a cube.
    const Math::mat4 projection =
        Math::perspective(Math::half_pi<f32>(), 1.0f, Math::max(nearPlane, 0.0001f),
                         Math::max(farPlane, nearPlane + 0.001f));
    for (u32 face = 0; face < FaceCount; ++face)
    {
        const Math::mat4 view = Math::lookAt(position, position + toVector(kFaces[face].forward),
                                           toVector(kFaces[face].up));
        out[face] = projection * view;
    }
}

void EnvironmentProbe::captureFaceColors()
{
    if (!ready())
        return;
    GPU& gpu = GPU::getSingleton();
    for (u32 face = 0; face < FaceCount; ++face)
    {
        ClearValue clear;
        clear.bits = ClearColor;
        clear.color[0] = kFaceColors[face][0];
        clear.color[1] = kFaceColors[face][1];
        clear.color[2] = kFaceColors[face][2];
        clear.color[3] = 1.0f;
        gpu.setTarget(mFaceTargets[face], clear);
    }
    gpu.setTarget(TargetHandle());
    generateMips();
}

void EnvironmentProbe::generateMips()
{
    if (!ready() || mMipCount <= 1)
        return;
    GPU::getSingleton().generateMips(mCubemap);
}

} // namespace Radion
