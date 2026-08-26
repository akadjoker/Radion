#include "PCH.h"

#include "DebugDraw3D.h"

#include "AssetManager.h"
#include "Batch.h"
#include "RenderTechnique.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_inverse.hpp>

namespace Radion
{

namespace
{

const char* kVectorVertexShader = R"GLSL(
#version 450 core
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec4 inTangent;
layout(std140, binding = 0) uniform DebugVectorBlock
{
    mat4 viewProjection;
    mat4 model;
    mat4 normalMatrix;
    vec4 options;
};
out VertexData { vec3 position; vec3 normal; vec3 tangent; } vertex;
void main()
{
    vertex.position = vec3(model * vec4(inPosition, 1.0));
    vertex.normal = normalize(mat3(normalMatrix) * inNormal);
    vertex.tangent = normalize(mat3(model) * inTangent.xyz);
    gl_Position = vec4(vertex.position, 1.0);
}
)GLSL";

const char* kVectorGeometryShader = R"GLSL(
#version 450 core
layout(points) in;
layout(line_strip, max_vertices = 4) out;
layout(std140, binding = 0) uniform DebugVectorBlock
{
    mat4 viewProjection;
    mat4 model;
    mat4 normalMatrix;
    vec4 options;
};
in VertexData { vec3 position; vec3 normal; vec3 tangent; } vertices[];
out vec4 lineColor;
void emitLine(vec3 from, vec3 to, vec4 color)
{
    lineColor = color;
    gl_Position = viewProjection * vec4(from, 1.0);
    EmitVertex();
    gl_Position = viewProjection * vec4(to, 1.0);
    EmitVertex();
    EndPrimitive();
}
void main()
{
    int flags = int(options.y + 0.5);
    if ((flags & 1) != 0)
        emitLine(vertices[0].position, vertices[0].position + vertices[0].normal * options.x,
                 vec4(0.2, 1.0, 0.35, 1.0));
    if ((flags & 2) != 0)
        emitLine(vertices[0].position, vertices[0].position + vertices[0].tangent * options.x,
                 vec4(1.0, 0.25, 0.85, 1.0));
}
)GLSL";

const char* kVectorFragmentShader = R"GLSL(
#version 450 core
in vec4 lineColor;
layout(location = 0) out vec4 outColor;
void main() { outColor = lineColor; }
)GLSL";

const char* kOutlineVertexShader = R"GLSL(
#version 450 core
layout(location = 0) in vec3 inPosition;
layout(std140, binding = 0) uniform DebugOutlineBlock
{
    mat4 viewProjection;
    mat4 model;
    vec4 color;
    vec4 options;
};
void main()
{
    // Per-vertex normal offset in screen space (the classic silhouette
    // rendering trick) needs a normal that actually points away from the
    // mesh's own silhouette everywhere - true on a cube or a sphere, false
    // on a lumpy rock where local curvature sends plenty of normals back
    // toward the outline. Scaling the whole hull outward from its own
    // local pivot instead (options.x, 1.0 for the mask draw, a bit above
    // 1.0 for the outline draw) makes the second pass strictly bigger than
    // the first everywhere, independent of concavity, at the cost of an
    // outline that thins out unevenly if the mesh's pivot sits far off its
    // visual center.
    gl_Position = viewProjection * model * vec4(inPosition * options.x, 1.0);
}
)GLSL";

const char* kOutlineFragmentShader = R"GLSL(
#version 450 core
layout(std140, binding = 0) uniform DebugOutlineBlock
{
    mat4 viewProjection;
    mat4 model;
    vec4 color;
    vec4 options;
};
layout(location = 0) out vec4 outColor;
void main() { outColor = color; }
)GLSL";

class DebugPass final : public RenderTechnique
{
public:
    const char* name() const override
    {
        return "Debug";
    }
    bool setup() override
    {
        return true;
    }
    void execute(const FrameContext& frame) override;
    void shutdown() override;

private:
    struct VectorBlock
    {
        glm::mat4 viewProjection;
        glm::mat4 model;
        glm::mat4 normalMatrix;
        glm::vec4 options;
    };

    struct OutlineBlock
    {
        glm::mat4 viewProjection;
        glm::mat4 model;
        glm::vec4 color;
        glm::vec4 options;
    };

    bool initializeVectors(const VertexLayout& layout);
    void drawMeshVectors(const FrameContext& frame, const DebugDraw3D& debug);
    bool initializeOutline(const VertexLayout& layout);
    void drawOutlines(const FrameContext& frame, const DebugDraw3D& debug);

    BatchRenderer mBatch;
    PipelineHandle mVectorPipeline;
    BufferHandle mVectorUniform;
    PipelineHandle mOutlineMaskPipeline;
    PipelineHandle mOutlinePipeline;
    BufferHandle mOutlineUniform;
    bool mBatchInitialized = false;
};

void DebugPass::execute(const FrameContext& frame)
{
    DebugDraw3D& debug = DebugDraw();
    if (debug.empty())
        return;

    GPU& gpu = GPU::getSingleton();
    if (!debug.outlineCommands().empty())
    {
        ClearValue clear;
        clear.bits = ClearStencil;
        gpu.setTarget(frame.target, clear);
    }
    else
    {
        gpu.setTarget(frame.target);
    }
    gpu.setViewport(frame.viewport);

    if ((!debug.lines().empty() || !debug.triangles().empty()) && !mBatchInitialized)
    {
        BatchRenderer::Config config;
        config.maxVertices = 65536;
        config.maxDrawCalls = 64;
        config.enableProfiling = false;
        if (!mBatch.init(config))
        {
            debug.clear();
            return;
        }
        mBatchInitialized = true;
    }

    if (!debug.lines().empty() || !debug.triangles().empty())
    {
        mBatch.update();
        mBatch.setProjection(frame.viewProjection);
        mBatch.loadIdentity();
        mBatch.setTexture(TextureHandle());
        mBatch.setDepthTest(true);
        mBatch.setDepthWrite(false);
        mBatch.setCullFace(false);

        if (!debug.triangles().empty())
        {
            mBatch.setBlend(true);
            mBatch.setBlendMode(BatchRenderer::BlendMode::Alpha);
            for (const DebugTriangle3D& triangle : debug.triangles())
            {
                mBatch.setColor(triangle.color.r(), triangle.color.g(), triangle.color.b(),
                                triangle.color.a());
                mBatch.drawTriangle3D(triangle.a, triangle.b, triangle.c);
            }
        }

        mBatch.setBlend(false);
        for (const DebugLine3D& line : debug.lines())
        {
            mBatch.setDepthTest(line.depthTest);
            mBatch.setColor(line.color.r(), line.color.g(), line.color.b(), line.color.a());
            mBatch.drawLine3D(line.from.x, line.from.y, line.from.z, line.to.x, line.to.y,
                              line.to.z);
        }
        mBatch.drawRenderBatch();
    }
    drawMeshVectors(frame, debug);
    drawOutlines(frame, debug);
    debug.clear();
}

bool DebugPass::initializeVectors(const VertexLayout& layout)
{
    GPU& gpu = GPU::getSingleton();
    BufferDesc uniformDesc;
    uniformDesc.size = sizeof(VectorBlock);
    uniformDesc.usage = BufferUniform;
    uniformDesc.residency = Residency::Stream;
    uniformDesc.debugName = "debug.vectors.uniform";
    mVectorUniform = gpu.createBuffer(uniformDesc);

    PipelineDesc pipelineDesc;
    pipelineDesc.vs = {kVectorVertexShader, 0, "debug_vectors.vert"};
    pipelineDesc.gs = {kVectorGeometryShader, 0, "debug_vectors.geom"};
    pipelineDesc.fs = {kVectorFragmentShader, 0, "debug_vectors.frag"};
    pipelineDesc.layout = layout;
    pipelineDesc.topology = Topology::Points;
    pipelineDesc.depth.test = true;
    pipelineDesc.depth.write = false;
    pipelineDesc.raster.cull = CullMode::None;
    pipelineDesc.debugName = "debug.vectors";
    mVectorPipeline = gpu.createPipeline(pipelineDesc);
    if (mVectorUniform.valid() && mVectorPipeline.valid())
        return true;

    gpu.destroy(mVectorUniform);
    gpu.destroy(mVectorPipeline);
    mVectorUniform = BufferHandle();
    mVectorPipeline = PipelineHandle();
    return false;
}

void DebugPass::drawMeshVectors(const FrameContext& frame, const DebugDraw3D& debug)
{
    GPU& gpu = GPU::getSingleton();
    AssetManager& assets = Assets();
    for (const DebugMeshVectors3D& command : debug.meshVectorCommands())
    {
        const Mesh* mesh = assets.getMesh(command.mesh);
        if (!mesh || !mesh->positionBuffer.valid() || !mesh->attribBuffer.valid())
            continue;
        if (!mVectorPipeline.valid() && !initializeVectors(mesh->colorLayout))
            return;

        VectorBlock block;
        block.viewProjection = frame.viewProjection;
        block.model = command.transform;
        const glm::mat3 basis(command.transform);
        block.normalMatrix = std::abs(glm::determinant(basis)) > 0.000001f
                                 ? glm::mat4(glm::transpose(glm::inverse(basis)))
                                 : glm::mat4(1.0f);
        block.options = glm::vec4(command.length, static_cast<f32>(command.flags), 0.0f, 0.0f);
        gpu.updateBuffer(mVectorUniform, 0, sizeof(block), &block);
        gpu.bindUniform(0, mVectorUniform);
        gpu.setPipeline(mVectorPipeline);

        DrawDesc draw;
        draw.vertexBuffers[0] = mesh->positionBuffer;
        draw.vertexBuffers[1] = mesh->attribBuffer;
        draw.vertexBufferCount = 2;
        draw.count = mesh->vertexCount;
        gpu.draw(draw);
    }
}

bool DebugPass::initializeOutline(const VertexLayout& layout)
{
    GPU& gpu = GPU::getSingleton();
    BufferDesc uniformDesc;
    uniformDesc.size = sizeof(OutlineBlock);
    uniformDesc.usage = BufferUniform;
    uniformDesc.residency = Residency::Stream;
    uniformDesc.debugName = "debug.outline.uniform";
    mOutlineUniform = gpu.createBuffer(uniformDesc);

    PipelineDesc maskDesc;
    maskDesc.vs = {kOutlineVertexShader, 0, "debug_outline.vert"};
    maskDesc.fs = {kOutlineFragmentShader, 0, "debug_outline.frag"};
    maskDesc.layout = layout;
    maskDesc.blend.writeRGB = false;
    maskDesc.blend.writeA = false;
    maskDesc.depth.test = true;
    maskDesc.depth.write = false;
    maskDesc.stencil.enabled = true;
    maskDesc.stencil.compare = Compare::Always;
    maskDesc.stencil.pass = StencilOp::Replace;
    maskDesc.raster.cull = CullMode::None;
    maskDesc.debugName = "debug.outline.mask";
    mOutlineMaskPipeline = gpu.createPipeline(maskDesc);

    PipelineDesc outlineDesc;
    outlineDesc.vs = {kOutlineVertexShader, 0, "debug_outline.vert"};
    outlineDesc.fs = {kOutlineFragmentShader, 0, "debug_outline.frag"};
    outlineDesc.layout = layout;
    outlineDesc.depth.test = true;
    outlineDesc.depth.write = false;
    outlineDesc.stencil.enabled = true;
    outlineDesc.stencil.compare = Compare::NotEqual;
    outlineDesc.stencil.writeMask = 0;
    // Culling front faces (leaving only the inflated backside visible) is the
    // classic version of this trick, but it silently draws nothing on meshes
    // whose winding is inverted relative to their normals - an interior room
    // authored so its walls face inward, for one. The stencil test already
    // rejects every fragment that lands back on top of the object's own
    // silhouette (mask == 1), so no culling is needed for correctness; only
    // the pixels genuinely outside the silhouette - the outline ring - pass.
    outlineDesc.raster.cull = CullMode::None;
    outlineDesc.debugName = "debug.outline.color";
    mOutlinePipeline = gpu.createPipeline(outlineDesc);

    if (mOutlineUniform.valid() && mOutlineMaskPipeline.valid() && mOutlinePipeline.valid())
        return true;

    gpu.destroy(mOutlineUniform);
    gpu.destroy(mOutlineMaskPipeline);
    gpu.destroy(mOutlinePipeline);
    mOutlineUniform = BufferHandle();
    mOutlineMaskPipeline = PipelineHandle();
    mOutlinePipeline = PipelineHandle();
    return false;
}

void DebugPass::drawOutlines(const FrameContext& frame, const DebugDraw3D& debug)
{
    GPU& gpu = GPU::getSingleton();
    AssetManager& assets = Assets();
    gpu.setStencilRef(1);

    for (const DebugMeshOutline3D& command : debug.outlineCommands())
    {
        const Mesh* mesh = assets.getMesh(command.mesh);
        if (!mesh || !mesh->positionBuffer.valid() || !mesh->attribBuffer.valid() ||
            !mesh->indexBuffer.valid())
            continue;
        if (!mOutlinePipeline.valid() && !initializeOutline(mesh->colorLayout))
            return;

        OutlineBlock block;
        block.viewProjection = frame.viewProjection;
        block.model = command.transform;
        block.color = glm::vec4(command.color.red(), command.color.green(), command.color.blue(),
                                command.color.alpha());
        block.options = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);

        DrawDesc draw;
        draw.vertexBuffers[0] = mesh->positionBuffer;
        draw.vertexBuffers[1] = mesh->attribBuffer;
        draw.vertexBufferCount = 2;
        draw.indexBuffer = mesh->indexBuffer;
        draw.indexType = mesh->indexType;
        // Same convention ForwardPass draws a submesh with (first = the
        // submesh's index offset); a count of 0 asks for the whole mesh.
        draw.first = command.indexCount != 0 ? command.indexOffset : 0;
        draw.count = command.indexCount != 0 ? command.indexCount : mesh->indexCount;

        gpu.updateBuffer(mOutlineUniform, 0, sizeof(block), &block);
        gpu.bindUniform(0, mOutlineUniform);
        gpu.setPipeline(mOutlineMaskPipeline);
        gpu.draw(draw);

        block.options.x = 1.0f + command.thickness;
        gpu.updateBuffer(mOutlineUniform, 0, sizeof(block), &block);
        gpu.setPipeline(mOutlinePipeline);
        gpu.draw(draw);
    }
}

void DebugPass::shutdown()
{
    GPU& gpu = GPU::getSingleton();
    if (mBatchInitialized)
        mBatch.shutdown();
    gpu.destroy(mVectorPipeline);
    gpu.destroy(mVectorUniform);
    gpu.destroy(mOutlineMaskPipeline);
    gpu.destroy(mOutlinePipeline);
    gpu.destroy(mOutlineUniform);
    mVectorPipeline = PipelineHandle();
    mVectorUniform = BufferHandle();
    mOutlineMaskPipeline = PipelineHandle();
    mOutlinePipeline = PipelineHandle();
    mOutlineUniform = BufferHandle();
    mBatchInitialized = false;
    DebugDraw().clear();
}

} // namespace

namespace
{

// Any two vectors orthogonal to `direction` and to each other work as a
// gizmo's local right/up - which pair does not matter, only that neither
// degenerates when `direction` is near-vertical.
void perpendicularBasis(const glm::vec3& direction, glm::vec3& right, glm::vec3& up)
{
    const glm::vec3 reference =
        std::abs(direction.y) > 0.99f ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    right = glm::normalize(glm::cross(direction, reference));
    up = glm::cross(right, direction);
}

Color lighten(Color color, f32 factor)
{
    const auto mix = [factor](u8 channel)
    {
        return static_cast<u8>(glm::mix(static_cast<f32>(channel), 255.0f, factor));
    };
    return Color(mix(color.r()), mix(color.g()), mix(color.b()), color.a());
}

} // namespace

DebugDraw3D::DebugDraw3D()
{
    mLines.reserve(1024);
    mTriangles.reserve(128);
    mMeshVectors.reserve(64);
    mOutlines.reserve(16);
}

DebugDraw3D& DebugDraw3D::getSingleton()
{
    static DebugDraw3D instance;
    return instance;
}

void DebugDraw3D::clear()
{
    mLines.clear();
    mTriangles.clear();
    mMeshVectors.clear();
    mOutlines.clear();
}

void DebugDraw3D::line(const glm::vec3& from, const glm::vec3& to, Color color, bool depthTest)
{
    mLines.push_back({from, to, color, depthTest});
}

void DebugDraw3D::box(const AABB& bounds, Color color)
{
    if (bounds.empty())
        return;

    const glm::vec3& a = bounds.min;
    const glm::vec3& b = bounds.max;
    const glm::vec3 corners[8] = {
        {a.x, a.y, a.z}, {b.x, a.y, a.z}, {b.x, a.y, b.z}, {a.x, a.y, b.z},
        {a.x, b.y, a.z}, {b.x, b.y, a.z}, {b.x, b.y, b.z}, {a.x, b.y, b.z},
    };
    constexpr u8 edges[24] = {0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6,
                              6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7};
    for (u8 i = 0; i < 24; i += 2)
        line(corners[edges[i]], corners[edges[i + 1]], color);
}

void DebugDraw3D::axis(const glm::mat4& transform, f32 size)
{
    const glm::vec3 origin = glm::vec3(transform[3]);
    line(origin, origin + glm::vec3(transform[0]) * size, Color::Red);
    line(origin, origin + glm::vec3(transform[1]) * size, Color::Green);
    line(origin, origin + glm::vec3(transform[2]) * size, Color::Blue);
}

void DebugDraw3D::grid(f32 y, u32 slices, f32 spacing, bool axes)
{
    if (slices == 0 || !(spacing > 0.0f))
        return;

    const f32 extent = static_cast<f32>(slices) * spacing;
    const Color minor(72, 76, 84);
    for (s32 i = -static_cast<s32>(slices); i <= static_cast<s32>(slices); ++i)
    {
        const f32 offset = static_cast<f32>(i) * spacing;
        const Color xColor = axes && i == 0 ? Color(Color::Red) : minor;
        const Color zColor = axes && i == 0 ? Color(Color::Blue) : minor;
        line(glm::vec3(-extent, y, offset), glm::vec3(extent, y, offset), xColor);
        line(glm::vec3(offset, y, -extent), glm::vec3(offset, y, extent), zColor);
    }
}

void DebugDraw3D::cursor3D(const glm::vec3& position, const glm::vec3& cameraRight,
                           const glm::vec3& cameraUp, f32 radius)
{
    constexpr u32 segments = 24;
    glm::vec3 previous = position + cameraRight * radius;
    for (u32 i = 1; i <= segments; ++i)
    {
        const f32 angle = static_cast<f32>(i) / static_cast<f32>(segments) * glm::two_pi<f32>();
        const glm::vec3 next =
            position + (cameraRight * glm::cos(angle) + cameraUp * glm::sin(angle)) * radius;
        line(previous, next, i % 2 == 0 ? Color(Color::White) : Color(255, 40, 40), false);
        previous = next;
    }

    const f32 crossSize = radius * 0.6f;
    line(position - cameraRight * crossSize, position + cameraRight * crossSize, Color(255, 40, 40),
         false);
    line(position - cameraUp * crossSize, position + cameraUp * crossSize, Color(255, 40, 40),
         false);
}

void DebugDraw3D::triangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, Color color,
                           bool filled)
{
    if (filled)
        mTriangles.push_back({a, b, c, color});
    else
    {
        line(a, b, color);
        line(b, c, color);
        line(c, a, color);
    }
}

void DebugDraw3D::pickedTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                                 const glm::vec3& hit, f32 normalLength)
{
    triangle(a, b, c, Color(255, 190, 32, 72), true);
    triangle(a, b, c, Color::Yellow, false);

    const glm::vec3 cross = glm::cross(b - a, c - a);
    const f32 lengthSquared = glm::dot(cross, cross);
    if (lengthSquared > 0.0000001f && normalLength > 0.0f)
        line(hit, hit + cross * (normalLength / std::sqrt(lengthSquared)), Color::Cyan);
}

void DebugDraw3D::meshVectors(MeshHandle mesh, const glm::mat4& transform, f32 length, u8 flags)
{
    if (!mesh.valid() || !(length > 0.0f) || flags == 0)
        return;
    mMeshVectors.push_back({mesh, transform, length, flags});
}

void DebugDraw3D::outline(MeshHandle mesh, const glm::mat4& transform, Color color, f32 thickness)
{
    outlineRange(mesh, transform, 0, 0, color, thickness);
}

void DebugDraw3D::outlineRange(MeshHandle mesh, const glm::mat4& transform, u32 indexOffset,
                               u32 indexCount, Color color, f32 thickness)
{
    if (!mesh.valid() || thickness <= 0.0f)
        return;
    mOutlines.push_back({mesh, transform, color, thickness, indexOffset, indexCount});
}

void DebugDraw3D::circle(const glm::vec3& center, const glm::vec3& u, const glm::vec3& v,
                         f32 radius, u32 segments, Color color)
{
    segments = glm::max(segments, 3u);
    glm::vec3 previous = center + u * radius;
    for (u32 i = 1; i <= segments; ++i)
    {
        const f32 t = (static_cast<f32>(i) / static_cast<f32>(segments)) * glm::two_pi<f32>();
        const glm::vec3 point = center + (u * std::cos(t) + v * std::sin(t)) * radius;
        line(previous, point, color);
        previous = point;
    }
}


void DebugDraw3D::cross(const glm::vec3& position, f32 size, Color color)
{
    line(position - glm::vec3(size, 0.0f, 0.0f), position + glm::vec3(size, 0.0f, 0.0f), color);
    line(position - glm::vec3(0.0f, size, 0.0f), position + glm::vec3(0.0f, size, 0.0f), color);
    line(position - glm::vec3(0.0f, 0.0f, size), position + glm::vec3(0.0f, 0.0f, size), color);
}

// Two short segments splayed off `from`, pointing back along from->to. The
// head sits in the plane perpendicular to world up, so a link seen from
// above still reads as directed.
void DebugDraw3D::arrowHead(const glm::vec3& from, const glm::vec3& to, f32 size, Color color)
{
    const glm::vec3 delta = to - from;
    if (glm::dot(delta, delta) < 1e-6f)
        return;
    const glm::vec3 forward = glm::normalize(delta);
    glm::vec3 side = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), forward);
    if (glm::dot(side, side) < 1e-6f)
        side = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), forward);
    side = glm::normalize(side);

    line(from, from + forward * size + side * (size / 3.0f), color);
    line(from, from + forward * size - side * (size / 3.0f), color);
}

void DebugDraw3D::arrow(const glm::vec3& from, const glm::vec3& to, f32 headFrom, f32 headTo,
                        Color color)
{
    line(from, to, color);
    if (headFrom > 0.001f)
        arrowHead(from, to, headFrom, color);
    if (headTo > 0.001f)
        arrowHead(to, from, headTo, color);
}

void DebugDraw3D::arc(const glm::vec3& from, const glm::vec3& to, f32 height, f32 headFrom,
                      f32 headTo, Color color)
{
    constexpr u32 kArcPoints = 8;
    constexpr f32 kPad = 0.05f;
    constexpr f32 kScale = (1.0f - kPad * 2.0f) / static_cast<f32>(kArcPoints);

    const glm::vec3 delta = to - from;
    const f32 rise = glm::length(delta) * height;

    // Parabola through both ends, peaking at the midpoint - the bow is what
    // separates it from an ordinary walked segment.
    const auto evaluate = [&](f32 u)
    {
        const f32 bow = 1.0f - (u * 2.0f - 1.0f) * (u * 2.0f - 1.0f);
        return from + delta * u + glm::vec3(0.0f, rise * bow, 0.0f);
    };

    glm::vec3 previous = evaluate(kPad);
    for (u32 i = 1; i <= kArcPoints; ++i)
    {
        const glm::vec3 point = evaluate(kPad + static_cast<f32>(i) * kScale);
        line(previous, point, color);
        previous = point;
    }

    if (headFrom > 0.001f)
        arrowHead(evaluate(kPad), evaluate(kPad + 0.05f), headFrom, color);
    if (headTo > 0.001f)
        arrowHead(evaluate(1.0f - kPad), evaluate(1.0f - kPad - 0.05f), headTo, color);
}

// Three orthogonal great circles: a single flat ring reads as a disc from
// some angles and vanishes from others, three of them read as a sphere from
// any of them.
void DebugDraw3D::pointLightGizmo(const glm::vec3& position, f32 range, Color color, u32 segments)
{
    if (!(range > 0.0f))
        return;
    circle(position, glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), range, segments,
           color);
    circle(position, glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), range, segments,
           color);
    circle(position, glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), range, segments,
           color);
}

// Two cones sharing the apex: the outer at full colour, the inner lightened
// so the falloff band between them is the thing the eye picks out.
void DebugDraw3D::spotLightGizmo(const glm::vec3& position, const glm::vec3& direction, f32 range,
                                 f32 innerAngleDegrees, f32 outerAngleDegrees, Color color,
                                 u32 segments)
{
    if (!(range > 0.0f) || !(glm::length(direction) > 0.0001f))
        return;
    const glm::vec3 forward = glm::normalize(direction);
    glm::vec3 right;
    glm::vec3 up;
    perpendicularBasis(forward, right, up);
    const glm::vec3 endCenter = position + forward * range;
    const u32 ribs = glm::max(4u, segments / 3);

    spotCone(position, endCenter, right, up, range, outerAngleDegrees, segments, ribs, color);
    spotCone(position, endCenter, right, up, range, innerAngleDegrees, segments, ribs,
             lighten(color, 0.6f));
}

void DebugDraw3D::spotCone(const glm::vec3& apex, const glm::vec3& endCenter,
                           const glm::vec3& right, const glm::vec3& up, f32 range, f32 angleDegrees,
                           u32 segments, u32 ribs, Color color)
{
    const f32 endRadius = range * std::tan(glm::radians(glm::clamp(angleDegrees, 0.0f, 89.0f)));
    circle(endCenter, right, up, endRadius, segments, color);
    for (u32 i = 0; i < ribs; ++i)
    {
        const f32 t = (static_cast<f32>(i) / static_cast<f32>(ribs)) * glm::two_pi<f32>();
        const glm::vec3 point = endCenter + (right * std::cos(t) + up * std::sin(t)) * endRadius;
        line(apex, point, color);
    }
}

void DebugDraw3D::rectangleLightGizmo(const glm::vec3& position, const glm::vec3& direction,
                                      f32 width, f32 height, Color color, f32 normalLength)
{
    if (!(width > 0.0f) || !(height > 0.0f) || !(glm::length(direction) > 0.0001f))
        return;
    const glm::vec3 forward = glm::normalize(direction);
    glm::vec3 right;
    glm::vec3 up;
    perpendicularBasis(forward, right, up);
    const glm::vec3 halfRight = right * (width * 0.5f);
    const glm::vec3 halfUp = up * (height * 0.5f);
    const glm::vec3 corners[4] = {
        position - halfRight - halfUp,
        position + halfRight - halfUp,
        position + halfRight + halfUp,
        position - halfRight + halfUp,
    };
    for (u32 i = 0; i < 4; ++i)
        line(corners[i], corners[(i + 1) % 4], color);
    if (normalLength > 0.0f)
        line(position, position + forward * normalLength, color);
}

// Position is meaningless for a directional light - the small ring and its
// rays exist only to show the direction, not to mark a place in the world.
void DebugDraw3D::directionalLightGizmo(const glm::vec3& position, const glm::vec3& direction,
                                        Color color, f32 radius, f32 rayLength, u32 rayCount)
{
    if (!(glm::length(direction) > 0.0001f))
        return;
    const glm::vec3 forward = glm::normalize(direction);
    glm::vec3 right;
    glm::vec3 up;
    perpendicularBasis(forward, right, up);
    circle(position, right, up, radius, 24, color);

    rayCount = glm::max(rayCount, 1u);
    for (u32 i = 0; i < rayCount; ++i)
    {
        const f32 t = (static_cast<f32>(i) / static_cast<f32>(rayCount)) * glm::two_pi<f32>();
        const glm::vec3 start =
            position + (right * std::cos(t) + up * std::sin(t)) * (radius * 0.6f);
        line(start, start + forward * rayLength, color);
    }
    line(position, position + forward * rayLength, color);
}

const std::vector<DebugLine3D>& DebugDraw3D::lines() const
{
    return mLines;
}

const std::vector<DebugTriangle3D>& DebugDraw3D::triangles() const
{
    return mTriangles;
}

const std::vector<DebugMeshVectors3D>& DebugDraw3D::meshVectorCommands() const
{
    return mMeshVectors;
}

const std::vector<DebugMeshOutline3D>& DebugDraw3D::outlineCommands() const
{
    return mOutlines;
}

bool DebugDraw3D::empty() const
{
    return mLines.empty() && mTriangles.empty() && mMeshVectors.empty() && mOutlines.empty();
}

RenderTechnique* createDebugPass()
{
    return new DebugPass();
}

} // namespace Radion
