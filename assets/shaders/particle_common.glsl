// Structures shared by the four passes and by drawing.
//
// This file is not #included by the loader - it is pasted by hand at the top
// of every particle shader, and the copies have to agree with each other and
// with the C++ (ParticleRender.cpp). If they diverge, std430 reads swapped
// fields and particles appear in nonsense places, with nothing to warn about
// it.
//
// std430 layout: every line is one vec4 (16 bytes). 96 bytes total. Same
// grouping as Wicked's own Particle (ShaderInterop_EmittedParticle.h), with
// colour as a vec4 rather than packed into a uint - here readability is worth
// more than the 12 bytes, the same trade ShaderEntity already makes.

struct Particle
{
    vec3  position;      float mass;
    vec3  velocity;      float life;
    vec3  force;         float maxLife;
    vec2  sizeBeginEnd;  float rotation;   float rotationVelocity;
    vec4  color;
    vec4  colorEnd;      // per particle: each firework burst fades to its own
                         // colour
};

// The counters live together in one buffer so kickoff can swap them on a
// single thread with no synchronisation at all.
//
// deadCount is an INT on purpose: emit does atomicAdd(-1) without knowing
// whether any free slots are left, so it can go negative when many threads
// race for the last ones. Kickoff clamps it back to zero - that is what the
// reference does ("max(0, deadCount)"). A uint here would wrap to a huge
// number and emit would start inventing indices.
struct Counters
{
    uint aliveCount;         // alive to simulate this frame
    int  deadCount;          // top of the free-slot stack
    uint emitCount;          // unused by any shader; kept for diagnostics
    uint aliveCountAfterSim; // survivors -> alive next frame
};
