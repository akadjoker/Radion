#include "PCH.h"

#include "TrailRender.h"

#include "Batch.h"

namespace Radion
{

void resolveBillboardAxes(BillboardMode mode, const Math::vec3& cameraRight,
                          const Math::vec3& cameraUp, const Math::vec3& cameraForward,
                          const Math::vec3& fixedRight, const Math::vec3& fixedUp,
                          Math::vec3& outRight, Math::vec3& outUp)
{
    if (mode == BillboardMode::Free)
    {
        outRight = cameraRight;
        outUp = cameraUp;
        return;
    }
    if (mode == BillboardMode::Upright)
    {
        // Yaw-only facing: flatten the camera's forward onto the world XZ
        // plane, keep world up fixed - doesn't tilt as the camera looks
        // up/down, same as a tree/grass billboard.
        const Math::vec3 worldUp(0.0f, 1.0f, 0.0f);
        Math::vec3 flatForward(cameraForward.x, 0.0f, cameraForward.z);
        f32 len = Math::length(flatForward);
        flatForward = len > 1e-5f ? flatForward * (1.0f / len) : Math::vec3(0.0f, 0.0f, 1.0f);
        Math::vec3 right = Math::cross(flatForward, worldUp);
        len = Math::length(right);
        outRight = len > 1e-5f ? right * (1.0f / len) : Math::vec3(1.0f, 0.0f, 0.0f);
        outUp = worldUp;
        return;
    }
    // Fixed: the instance's own orientation, no camera-facing at all.
    outRight = fixedRight;
    outUp = fixedUp;
}

namespace
{

// Color::value() packs 0xAARRGGBB (red in bits 16-23) - BatchRenderer's own
// packColor()/unpackColor() pack red in bits 0-7 instead (see Batch.cpp).
// Handing Color::value() straight to a BatchRenderer draw call, as this file
// used to, put red where blue was expected and blue where red was: a Color
// built as a dark red painted as a dark blue quad. Re-packed through
// Color's own r()/g()/b()/a() accessors instead of its raw value(), this
// reads correctly regardless of which of the two conventions either side
// changes to later.
u32 packBatchColor(Color color)
{
    return BatchRenderer::packColor(color.r(), color.g(), color.b(), color.a());
}

void quadCorners(const Math::vec3& center, const Math::vec3& right, const Math::vec3& up,
                 const Math::vec2& size, f32 rotation, Math::vec3 outPosition[4])
{
    Math::vec3 axisRight = right, axisUp = up;
    if (rotation != 0.0f)
    {
        // Spins the quad in its own plane - without this, many aligned
        // quads read as a grid (see particle.vert, same reasoning for the
        // GPU-driven particles).
        const f32 s = Math::sin(rotation), c = Math::cos(rotation);
        axisRight = right * c + up * s;
        axisUp = up * c - right * s;
    }
    const Math::vec3 hr = axisRight * (size.x * 0.5f);
    const Math::vec3 hu = axisUp * (size.y * 0.5f);
    outPosition[0] = center - hr - hu;
    outPosition[1] = center + hr - hu;
    outPosition[2] = center + hr + hu;
    outPosition[3] = center - hr + hu;
}

class TrailPass final : public RenderTechnique
{
public:
    const char* name() const override
    {
        return "Trails";
    }
    bool setup() override
    {
        BatchRenderer::Config config;
        config.maxVertices = 4096;
        config.maxDrawCalls = 1;
        config.enableProfiling = false;
        mReady = mBatch.init(config);
        if (mReady)
            TrailDraws().setFontTexture(mBatch.fontTexture());
        return mReady;
    }
    void execute(const FrameContext& frame) override
    {
        const std::vector<TrailDrawCommand>& commands = TrailDraws().commands();
        const std::vector<BillboardInstance>& billboards = TrailDraws().billboards();
        const std::vector<MeshTextInstance>& texts = TrailDraws().texts();
        if (commands.empty() && billboards.empty() && texts.empty())
            return;

        GPU& gpu = GPU::getSingleton();
        gpu.setTarget(frame.target);
        gpu.setViewport(frame.viewport);
        mBatch.update();
        mBatch.setProjection(frame.viewProjection);
        mBatch.loadIdentity();
        mBatch.setDepthWrite(false);
        mBatch.setCullFace(false);
        mBatch.setBlend(true);

        for (const TrailDrawCommand& command : commands)
        {
            if (!command.vertices || command.count < 3)
                continue;
            mBatch.setTexture(command.texture);
            mBatch.setDepthTest(command.depthTest);
            mBatch.setBlendMode(command.blend);
            for (u32 i = 0; i + 2 < command.count; i += 3)
            {
                const TrailVertex& a = command.vertices[i];
                const TrailVertex& b = command.vertices[i + 1];
                const TrailVertex& c = command.vertices[i + 2];
                mBatch.drawTriangle3D(a.position, a.uv, packBatchColor(a.color), b.position, b.uv,
                                      packBatchColor(b.color), c.position, c.uv,
                                      packBatchColor(c.color));
            }
        }

        if (!billboards.empty() || !texts.empty())
            drawBillboards(frame, billboards, texts);

        mBatch.drawRenderBatch();
    }
    void shutdown() override
    {
        if (mReady)
            mBatch.shutdown();
        mReady = false;
        TrailDraws().clear();
    }

private:
    void drawBillboards(const FrameContext& frame, const std::vector<BillboardInstance>& billboards,
                        const std::vector<MeshTextInstance>& texts)
    {
        // Same extraction ParticlePass uses for GPU billboarding: the
        // columns of an orthonormal view matrix are the camera axes in
        // world space.
        const Math::mat3 viewRotation(frame.view);
        const Math::vec3 cameraRight = Math::normalize(
            Math::vec3(viewRotation[0][0], viewRotation[1][0], viewRotation[2][0]));
        const Math::vec3 cameraUp = Math::normalize(
            Math::vec3(viewRotation[0][1], viewRotation[1][1], viewRotation[2][1]));
        const Math::vec3 cameraForward = -Math::normalize(
            Math::vec3(viewRotation[0][2], viewRotation[1][2], viewRotation[2][2]));

        for (const BillboardInstance& instance : billboards)
        {
            Math::vec3 right, up;
            resolveBillboardAxes(instance.mode, cameraRight, cameraUp, cameraForward,
                                 instance.fixedRight, instance.fixedUp, right, up);

            Math::vec3 corner[4];
            quadCorners(instance.position, right, up, instance.size, instance.rotation, corner);

            const f32 u0 = instance.uvRect.x, v0 = instance.uvRect.y;
            const f32 u1 = instance.uvRect.x + instance.uvRect.z;
            const f32 v1 = instance.uvRect.y + instance.uvRect.w;

            mBatch.setTexture(instance.texture);
            mBatch.setDepthTest(instance.depthTest);
            mBatch.setBlendMode(instance.blend);
            const u32 packedColor = packBatchColor(instance.color);
            mBatch.drawTriangle3D(corner[0], Math::vec2(u0, v1), packedColor, corner[1],
                                  Math::vec2(u1, v1), packedColor, corner[2], Math::vec2(u1, v0),
                                  packedColor);
            mBatch.drawTriangle3D(corner[0], Math::vec2(u0, v1), packedColor, corner[2],
                                  Math::vec2(u1, v0), packedColor, corner[3], Math::vec2(u0, v0),
                                  packedColor);
        }

        for (const MeshTextInstance& instance : texts)
        {
            if (!instance.glyphs || instance.glyphCount == 0)
                continue;

            Math::vec3 right, up;
            resolveBillboardAxes(instance.mode, cameraRight, cameraUp, cameraForward,
                                 instance.fixedRight, instance.fixedUp, right, up);

            mBatch.setTexture(instance.texture);
            mBatch.setDepthTest(instance.depthTest);
            mBatch.setBlendMode(instance.blend);
            const u32 packedColor = packBatchColor(instance.color);

            for (u32 i = 0; i < instance.glyphCount; ++i)
            {
                const MeshGlyph& glyph = instance.glyphs[i];
                if (glyph.uvRect.z <= 0.0f)
                    continue; // space or a glyph outside the atlas - nothing to draw

                // glyph.offset places this glyph's bottom-left corner in the
                // string's own right/up plane; the quad extends one
                // glyphSize further along each axis from there.
                const Math::vec3 bottomLeft =
                    instance.position + right * glyph.offset.x + up * glyph.offset.y;
                const Math::vec3 bottomRight = bottomLeft + right * instance.glyphSize;
                const Math::vec3 topRight = bottomRight + up * instance.glyphSize;
                const Math::vec3 topLeft = bottomLeft + up * instance.glyphSize;

                const f32 u0 = glyph.uvRect.x, v0 = glyph.uvRect.y;
                const f32 u1 = glyph.uvRect.x + glyph.uvRect.z;
                const f32 v1 = glyph.uvRect.y + glyph.uvRect.w;

                mBatch.drawTriangle3D(bottomLeft, Math::vec2(u0, v1), packedColor, bottomRight,
                                      Math::vec2(u1, v1), packedColor, topRight, Math::vec2(u1, v0),
                                      packedColor);
                mBatch.drawTriangle3D(bottomLeft, Math::vec2(u0, v1), packedColor, topRight,
                                      Math::vec2(u1, v0), packedColor, topLeft, Math::vec2(u0, v0),
                                      packedColor);
            }
        }
    }

    BatchRenderer mBatch;
    bool mReady = false;
};
} // namespace

TrailRenderQueue& TrailRenderQueue::getSingleton()
{
    static TrailRenderQueue queue;
    return queue;
}
void TrailRenderQueue::clear()
{
    mCommands.clear();
    mBillboards.clear();
    mTexts.clear();
}
void TrailRenderQueue::submit(const TrailDrawCommand& command)
{
    if (command.vertices && command.count >= 3)
        mCommands.push_back(command);
}
void TrailRenderQueue::submit(const BillboardInstance& instance)
{
    if (instance.size.x > 0.0f && instance.size.y > 0.0f)
        mBillboards.push_back(instance);
}
void TrailRenderQueue::submit(const MeshTextInstance& instance)
{
    if (instance.glyphs && instance.glyphCount > 0)
        mTexts.push_back(instance);
}
const std::vector<TrailDrawCommand>& TrailRenderQueue::commands() const
{
    return mCommands;
}
const std::vector<BillboardInstance>& TrailRenderQueue::billboards() const
{
    return mBillboards;
}
const std::vector<MeshTextInstance>& TrailRenderQueue::texts() const
{
    return mTexts;
}
void TrailRenderQueue::setFontTexture(TextureHandle texture)
{
    mFontTexture = texture;
}
TextureHandle TrailRenderQueue::fontTexture() const
{
    return mFontTexture;
}
TrailRenderQueue& TrailDraws()
{
    return TrailRenderQueue::getSingleton();
}
RenderTechnique* createTrailPass()
{
    return new TrailPass();
}

} // namespace Radion
