#ifndef RADION_ENVIRONMENT_PROBE_H
#define RADION_ENVIRONMENT_PROBE_H

#include "GPU.h"
#include "Mesh.h"

#include "Math.h"

namespace Radion
{

// Cubemap of the surroundings for image-based reflections.
// The mip chain is the roughness axis (mip 0 mirror, last mip nearly diffuse). Capture is gated, not per-frame, since it is six scene renders.
// Deliberately one probe, not a set: several would need blending and per-tile culling. A volume (`extents`) covers local and global cases.
class EnvironmentProbe final
{
public:
    enum class Refresh : u8
    {
        Manual,    // only when requestCapture() is called
        Automatic, // after create() and whenever invalidate() marks it stale
        Timed      // every `interval` seconds, regardless of invalidation
    };

    // FaceColors shows which face points where and whether the image is mirrored.
    enum class Content : u8
    {
        FaceColors,
        Sky,
        SkyAndWorld
    };

    bool create(u32 resolution = 128);
    void shutdown();
    bool ready() const;

    // World space. `extents` are HALF-extents of the box the reflection lives in (parallax correction); all zero = infinitely distant.
    Math::vec3 position = Math::vec3(0.0f);
    Math::vec3 extents = Math::vec3(0.0f);

    // Radius around `position` within which this probe serves objects. Zero falls back to the extents box.
    f32 influenceRadius = 0.0f;

    // Multiplier on the reflection; zero is a quick check of whether a colour comes from the probe.
    f32 intensity = 1.0f;

    Content content = Content::SkyAndWorld;
    Refresh refresh = Refresh::Automatic;
    f32 interval = 0.5f;
    bool enabled = true;

    // Kept out of the capture so a reflective object at the probe's position does not reflect its own inside.
    MeshHandle excludeMesh;

    // Same as excludeMesh for one object (GameObject::id(), 0 = none); a MeshHandle cannot, since identical meshes share one handle.
    // ReflectionProbe sets this to its owner (see ReflectionProbe::onLateUpdate). Opaque to render/.
    u64 excludeObjectId = 0;

    f32 nearPlane = 0.1f;
    f32 farPlane = 1000.0f;

    // Marks the cubemap stale; call after moving something the reflection shows.
    void invalidate();

    // Explicit capture request (editor Recapture); separate from invalidate() so scene edits do not make a Manual probe automatic.
    void requestCapture();

    // True at most once per due capture, clearing the request; advances the Timed accumulator.
    // `deferred` keeps a due request pending (editor viewport drag).
    bool consumeCapture(f32 deltaTime, bool deferred = false);

    // Filled by Engine around the six-face capture, for editor diagnostics.
    void recordCapture(f32 elapsedMilliseconds, f32 timeSeconds);
    u64 captureCount() const { return mCaptureCount; }
    f32 lastCaptureCostMilliseconds() const { return mLastCaptureCostMilliseconds; }
    f32 lastCaptureTimeSeconds() const { return mLastCaptureTimeSeconds; }

    TextureHandle cubemap() const;
    SamplerHandle sampler() const;

    TargetHandle faceTarget(u32 face) const;

    u32 resolution() const;
    u32 mipCount() const;

    // View-projection per face, in GL face order (+X, -X, +Y, -Y, +Z, -Z).
    void faceViewProjections(Math::mat4 out[6]) const;

    // Fills every face with a flat colour keyed to its axis: +X/+Y/+Z are
    // red/green/blue, and their opposites cyan/magenta/yellow.
    void captureFaceColors();

    // Rebuilds mips 1..n from mip 0 with a box filter (stand-in for a GGX prefilter).
    void generateMips();

    static constexpr u32 FaceCount = 6;

private:
    TextureHandle mCubemap;

    // One 2D depth buffer shared by all six faces (each clears it); without it the sky would paint over the world.
    TextureHandle mDepth;
    SamplerHandle mSampler;
    TargetHandle mFaceTargets[FaceCount];
    u32 mResolution = 0;
    u32 mMipCount = 0;
    bool mDirty = true;
    bool mCaptureRequested = false;
    f32 mAccumulator = 0.0f;
    u64 mCaptureCount = 0;
    f32 mLastCaptureCostMilliseconds = 0.0f;
    f32 mLastCaptureTimeSeconds = -1.0f;
};

} // namespace Radion

#endif // RADION_ENVIRONMENT_PROBE_H
