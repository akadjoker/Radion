#ifndef RADION_TRAIL_RENDER_H
#define RADION_TRAIL_RENDER_H

#include "Batch.h"
#include "Color.h"
#include "GPU.h"
#include "RenderTechnique.h"

#include "Math.h"
#include <vector>

namespace Radion
{

struct TrailVertex
{
    Math::vec3 position;
    Math::vec2 uv;
    Color color;
};

struct TrailDrawCommand
{
    const TrailVertex* vertices = nullptr;
    u32 count = 0;
    TextureHandle texture;
    BatchRenderer::BlendMode blend = BatchRenderer::BlendMode::Additive;
    bool depthTest = true;
};

enum class BillboardMode : u8
{
    Free,    // fully camera-facing, rotates on every axis toward the viewer
    Upright, // faces the camera's yaw only, world up stays fixed
    Fixed    // no auto-facing, uses the instance's own stored right/up axes
};

// Right/up axes for one facing mode. Free and Upright derive from the
// frame's camera basis; Fixed ignores it and returns fixedRight/fixedUp
// unchanged. Shared by Billboard and Text3D so both facing rules stay in
// exactly one place.
void resolveBillboardAxes(BillboardMode mode, const Math::vec3& cameraRight,
                          const Math::vec3& cameraUp, const Math::vec3& cameraForward,
                          const Math::vec3& fixedRight, const Math::vec3& fixedUp,
                          Math::vec3& outRight, Math::vec3& outUp);

// One camera-facing quad. Submitted with the instance's own transform only -
// the camera isn't known at the component's onLateUpdate() - and expanded
// into world-space triangles by TrailPass::execute() once the frame's
// camera axes are.
struct BillboardInstance
{
    Math::vec3 position = Math::vec3(0.0f);
    Math::vec2 size = Math::vec2(1.0f);
    f32 rotation = 0.0f; // radians, spins the quad in its own facing plane
    Math::vec4 uvRect = Math::vec4(0.0f, 0.0f, 1.0f, 1.0f);
    Color color;
    BillboardMode mode = BillboardMode::Free;
    Math::vec3 fixedRight = Math::vec3(1.0f, 0.0f, 0.0f);
    Math::vec3 fixedUp = Math::vec3(0.0f, 1.0f, 0.0f);
    TextureHandle texture;
    BatchRenderer::BlendMode blend = BatchRenderer::BlendMode::Additive;
    bool depthTest = true;
};

// One glyph within a MeshTextInstance: offset from the string's base
// position, in world units along the string's own right/up axes (not
// pixels), plus that glyph's cell in the font atlas.
struct MeshGlyph
{
    Math::vec2 offset = Math::vec2(0.0f);
    Math::vec4 uvRect = Math::vec4(0.0f);
};

// A whole string of glyphs sharing one base position, facing mode and
// texture - one draw command instead of one BillboardInstance per
// character. `glyphs` points into the owning Text3D component's own
// buffer, which outlives the frame this instance is submitted on.
struct MeshTextInstance
{
    Math::vec3 position = Math::vec3(0.0f);
    f32 glyphSize = 1.0f; // world-space width and height of one glyph quad
    Color color;
    BillboardMode mode = BillboardMode::Free;
    Math::vec3 fixedRight = Math::vec3(1.0f, 0.0f, 0.0f);
    Math::vec3 fixedUp = Math::vec3(0.0f, 1.0f, 0.0f);
    TextureHandle texture;
    BatchRenderer::BlendMode blend = BatchRenderer::BlendMode::Alpha;
    bool depthTest = true;
    const MeshGlyph* glyphs = nullptr;
    u32 glyphCount = 0;
};

class TrailRenderQueue
{
public:
    static TrailRenderQueue& getSingleton();

    void clear();
    void submit(const TrailDrawCommand& command);
    void submit(const BillboardInstance& instance);
    void submit(const MeshTextInstance& instance);
    const std::vector<TrailDrawCommand>& commands() const;
    const std::vector<BillboardInstance>& billboards() const;
    const std::vector<MeshTextInstance>& texts() const;

    // The embedded 8x8 font atlas TrailPass renders MeshTextInstances with.
    // Set once by the pass in its setup(); Text3D reads it back to fill
    // MeshTextInstance::texture, since a component has no other way to
    // reach the pass that will draw it.
    void setFontTexture(TextureHandle texture);
    TextureHandle fontTexture() const;

private:
    std::vector<TrailDrawCommand> mCommands;
    std::vector<BillboardInstance> mBillboards;
    std::vector<MeshTextInstance> mTexts;
    TextureHandle mFontTexture;
};

TrailRenderQueue& TrailDraws();
RenderTechnique* createTrailPass();

} // namespace Radion

#endif // RADION_TRAIL_RENDER_H
