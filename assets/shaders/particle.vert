#version 450 core

// Draws the particles. NO vertex buffer at all.
//
//   gl_VertexID    -> the 4 corners of the quad (triangle strip)
//   gl_InstanceID  -> index into the NEW alive list, and from there the
//                     particle
//
// Same idea as the tree impostors and the grass: the vertices are computed,
// not read. The only input is the SSBO.
//
// The instance count does NOT come from here or from the CPU: it comes from
// the indirect buffer particle_finish.comp wrote.

// --- particle_common.glsl (pasted; see the note in it) ---
struct Particle
{
    vec3  position;      float mass;
    vec3  velocity;      float life;
    vec3  force;         float maxLife;
    vec2  sizeBeginEnd;  float rotation;   float rotationVelocity;
    vec4  color;
    vec4  colorEnd;
};
// --- end ---

layout(std430, binding = 0) readonly buffer ParticleBuffer { Particle particles[]; };
layout(std430, binding = 5) readonly buffer AliveNew       { uint aliveNEW[]; };

layout(std140, binding = 0) uniform DrawBlock
{
    mat4 uViewProj;
    vec4 uCameraRight; // xyz
    vec4 uCameraUp;    // xyz, w = 1.0 for additive blending
};

out vec2  vUV;
out vec4  vColor;
out float vLifeLerp;

void main()
{
    Particle p = particles[aliveNEW[gl_InstanceID]];

    // 0..1 from birth to death. Everything that changes over the particle's
    // life - size, colour, opacity - comes from here.
    float lifeLerp = 1.0 - clamp(p.life / max(0.0001, p.maxLife), 0.0, 1.0);
    vLifeLerp = lifeLerp;

    float size = mix(p.sizeBeginEnd.x, p.sizeBeginEnd.y, lifeLerp);

    // The 4 corners in a triangle strip: 0=(-1,-1) 1=(1,-1) 2=(-1,1) 3=(1,1)
    vec2 corner = vec2(float(gl_VertexID & 1) * 2.0 - 1.0,
                       float(gl_VertexID >> 1) * 2.0 - 1.0);
    vUV = corner * 0.5 + 0.5;

    // Spins the quad in its own plane. Without this, thousands of aligned
    // quads read as a grid - an explosion gives itself away immediately.
    float s = sin(p.rotation), cs = cos(p.rotation);
    vec2 r = vec2(corner.x * cs - corner.y * s,
                  corner.x * s + corner.y * cs);

    // Billboard: the quad lives in the camera's plane, so whatever the
    // viewing angle the particle is never seen edge-on.
    vec3 world = p.position + (uCameraRight.xyz * r.x + uCameraUp.xyz * r.y) * size;

    vColor = mix(p.color, p.colorEnd, lifeLerp);

    gl_Position = uViewProj * vec4(world, 1.0);
}
