#ifndef RADION_ENVIRONMENT_PROBE_H
#define RADION_ENVIRONMENT_PROBE_H

#include "GPU.h"
#include "Mesh.h"

#include <glm/glm.hpp>

namespace Radion
{

// A cubemap of the surroundings, for image-based reflections - what makes a
// car body or a plane's fuselage reflect the world instead of just catching a
// specular highlight from the sun.
//
// Ported from the reference's EnvironmentProbeComponent (wiScene_Components
// .cpp:2149) and RefreshEnvProbes (wiRenderer.cpp:9313). Two things it keeps
// from there:
//
//   - The mip chain IS the roughness axis: mip 0 is a mirror, the last mip is
//     nearly diffuse, and the shader picks a level from the material's
//     roughness. That is what a prefilter pass fills in.
//   - Capture is gated, never per-frame by default. The reference has
//     render_dirty plus a realtime_time_accumulator for exactly this: six
//     scene renders is not something to pay every frame when the world is
//     standing still.
//
// Deliberately ONE probe, not a set: several probes need blending between
// them and per-tile culling (the reference does it by hand in the shader,
// ordered bottom to top), which is the expensive half. A single probe with a
// volume already covers both of the reference's two shader paths - see
// `extents`.
class EnvironmentProbe final
{
public:
    enum class Refresh : u8
    {
        Manual,    // only when requestCapture() is called
        Automatic, // after create() and whenever invalidate() marks it stale
        Timed      // every `interval` seconds, regardless of invalidation
    };

    // What the cubemap is filled with. FaceColors exists to check the thing
    // every cubemap gets wrong: which face points where, and whether the
    // image lands mirrored. Six flat colours make that answerable at a
    // glance, which reasoning about GL's cube conventions does not.
    enum class Content : u8
    {
        FaceColors,
        Sky,
        SkyAndWorld
    };

    bool create(u32 resolution = 128);
    void shutdown();
    bool ready() const;

    // World space. `extents` are HALF-extents of the box the reflection is
    // treated as living inside: the shader intersects the reflection ray
    // against it and corrects the sample direction, so a wall reflects at the
    // distance it actually is. All zero means infinitely distant - no
    // correction, the reference's global-probe path.
    Math::Vec3 position = Math::Vec3(0.0f);
    Math::Vec3 extents = Math::Vec3(0.0f);

    // Which objects this probe serves, as a radius around `position` - a
    // job `extents` used to do as a side effect of being non-zero, and could
    // not do without also switching the parallax correction on. They are not
    // the same question: "is this probe the right one for that object" is
    // about placement, "how far away is the world it captured" is about the
    // geometry, and a probe meant as a plain mirror of its surroundings needs
    // the first with the second left off (extents zero). Zero here keeps the
    // historical behaviour - selection falls back to the extents box - so a
    // probe that only sets extents still works exactly as it did.
    f32 influenceRadius = 0.0f;

    // A plain multiplier on the reflection every surface gets. Its use is not
    // only artistic: dropping it to zero is the fastest way to answer "is that
    // colour coming from the probe or from somewhere else", which reasoning
    // about a cubemap cannot.
    f32 intensity = 1.0f;

    Content content = Content::SkyAndWorld;
    Refresh refresh = Refresh::Automatic;
    f32 interval = 0.5f;
    bool enabled = true;

    // Kept out of the capture. A mirrored surface sitting at the probe's own
    // position would otherwise fill the cubemap with the inside of itself and
    // then reflect that back - which is what a reflective object placed at
    // its own probe always does, so the probe has to be told.
    MeshHandle excludeMesh;

    // The same, said as the one object rather than as a mesh - GameObject::id(),
    // 0 for none. A MeshHandle cannot express this: AssetManager caches meshes
    // by description, so two spheres of equal radius ARE one handle, and
    // excluding "the sphere mesh" drops every other identical sphere from the
    // capture along with this one. A ReflectionProbe sets this to its own
    // owner automatically (see ReflectionProbe::onLateUpdate), so a probe
    // attached to an object never needs telling what to leave out. Opaque to
    // render/, which only forwards it to whoever builds the caster list.
    u64 excludeObjectId = 0;

    // How far the capture cameras see. The reference falls back to the main
    // camera's far plane when a probe does not set its own.
    f32 nearPlane = 0.1f;
    f32 farPlane = 1000.0f;

    // Marks the cubemap stale. What to call after moving something the
    // reflection should show.
    void invalidate();

    // Explicit capture request, used by the editor's Recapture button. This
    // is deliberately separate from invalidate(): scene edits must not make
    // a Manual probe behave like an Automatic one.
    void requestCapture();

    // True at most once per due capture, and clears the request - so the
    // caller can just ask every frame. Advances the Timed accumulator.
    // `deferred` keeps a due request pending (the editor uses it while the
    // viewport camera is being dragged), instead of losing it for the sake
    // of interaction latency.
    bool consumeCapture(f32 deltaTime, bool deferred = false);

    // Filled by Engine around the complete six-face capture and exposed for
    // editor diagnostics.
    void recordCapture(f32 elapsedMilliseconds, f32 timeSeconds);
    u64 captureCount() const { return mCaptureCount; }
    f32 lastCaptureCostMilliseconds() const { return mLastCaptureCostMilliseconds; }
    f32 lastCaptureTimeSeconds() const { return mLastCaptureTimeSeconds; }

    TextureHandle cubemap() const;
    SamplerHandle sampler() const;

    // One render target per cube face, at mip 0.
    TargetHandle faceTarget(u32 face) const;

    u32 resolution() const;
    u32 mipCount() const;

    // View-projection for each of the 6 faces, in GL's own face order
    // (+X, -X, +Y, -Y, +Z, -Z - the order glNamedFramebufferTextureLayer
    // takes as layer 0..5).
    void faceViewProjections(Math::Mat4 out[6]) const;

    // Fills every face with a flat colour keyed to its axis: +X/+Y/+Z are
    // red/green/blue, and their opposites cyan/magenta/yellow.
    void captureFaceColors();

    // Rebuilds mips 1..n from mip 0. A placeholder for the GGX prefilter:
    // a plain box filter blurs by mip level, which is enough to see the
    // roughness axis working before the importance sampling exists.
    void generateMips();

    static constexpr u32 FaceCount = 6;

private:
    TextureHandle mCubemap;

    // One plain 2D depth buffer, shared by all six face targets: each face
    // clears it before drawing, so they never need one each. Without it the
    // capture has no depth test at all - and the sky, drawn last, would paint
    // over the world instead of only filling in behind it.
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
