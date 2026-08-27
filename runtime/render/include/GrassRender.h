#ifndef RADION_GRASS_RENDER_H
#define RADION_GRASS_RENDER_H

#include "GPU.h"
#include "RenderTechnique.h"

#include <vector>

namespace Radion
{

// A tuft. Three vec4 in std430, matching GrassClump in grass_common.glsl.
struct GrassClump
{
    Math::vec4 positionScale = Math::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    Math::vec4 normalRotation = Math::vec4(0.0f, 1.0f, 0.0f, 0.0f);
    Math::vec4 rect = Math::vec4(0.0f);
};

// A region of the vegetation atlas. `size` multiplies the tuft's height, so a
// tall plant in the atlas gives a tall quad without tuning each one by hand;
// `aspect` is the region's width over its height, without which a thin stalk
// and a wide bush would both come out square.
struct GrassAtlasRect
{
    Math::vec4 texMulAdd = Math::vec4(1.0f, 1.0f, 0.0f, 0.0f);
    Math::vec4 sizeAspect = Math::vec4(1.0f, 1.0f, 0.0f, 0.0f);
};

// A sphere that pushes the grass out of its way. Cleared and re-added each
// frame by whoever moves through the field.
struct GrassInfluencer
{
    Math::vec3 centre = Math::vec3(0.0f);
    f32 radius = 3.0f;
    f32 force = 30.0f;
};

// What a Grass component hands over for the frame. The pass reads the arrays
// straight out of the component, so nothing is copied on the way.
struct GrassDrawCommand
{
    const GrassClump* clumps = nullptr;
    u32 clumpCount = 0;
    const GrassAtlasRect* rects = nullptr;
    u32 rectCount = 0;
    u64 revision = 0; // changes when the clumps do, and only then

    TextureHandle atlas;
    f32 height = 1.2f;
    f32 width = 0.7f;
    f32 wind = 1.0f;
    f32 alphaCut = 0.35f;
    f32 cameraBend = 0.55f;
    f32 drawDistance = 120.0f;

    // Verlet: stiffness pulls the tip back to rest, drag is how much of the
    // inertia is lost per step. Without state the grass would snap back the
    // instant a push ended.
    f32 stiffness = 12.0f;
    f32 drag = 0.12f;
    f32 deltaTime = 0.0f;
    bool reset = false;

    // Second pass that blends the leaf outline. Off shows the binary step a
    // plain alpha test leaves behind.
    bool softFringe = true;

    // Defaults match the fake sun in unlit.frag, so grass and the ground it
    // stands on are lit by the same light.
    Math::vec3 lightDirection = -Math::normalize(Math::vec3(0.4f, 0.8f, 0.3f));
    Math::vec3 lightColor = Math::vec3(1.0f);
    Math::vec3 ambient = Math::vec3(0.35f);

    const GrassInfluencer* influencers = nullptr;
    u32 influencerCount = 0;
};

// As many as the block holds; the shader agrees on this number.
constexpr u32 kGrassMaxInfluencers = 8;

class GrassRenderQueue
{
public:
    static GrassRenderQueue& getSingleton();

    void clear();
    void submit(const GrassDrawCommand& command);
    const std::vector<GrassDrawCommand>& commands() const;

private:
    std::vector<GrassDrawCommand> mCommands;
};

GrassRenderQueue& GrassDraws();

// Culls in compute and draws indirect: the instance count is written by the
// GPU and never read back, so the cost on the CPU is one dispatch and one draw
// whatever the field holds.
RenderTechnique* createGrassPass();

} // namespace Radion

#endif // RADION_GRASS_RENDER_H
