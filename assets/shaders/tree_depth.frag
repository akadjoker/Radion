#version 450 core

// Depth-only for the shadow views. It exists because depth.frag is empty
// (`void main(){}`), which is right for solid geometry and wrong for leaves:
// a twig card is a rectangle whose shape lives entirely in its alpha, so
// without the cut below every tree casts the shadow of a box of cards.

in vec2 vUV;

layout(std140, binding = 6) uniform TreeBlock
{
    vec4 uTreeWind;
    vec4 uTreeSurface;
    vec4 uTreeTemporal;
};

#define uIsLeaves  int(uTreeWind.w)
#define uAlphaCut  uTreeSurface.y

layout(binding = 2) uniform sampler2D uTwigTex;

void main()
{
    // The trunk needs no test - it is solid, and sampling the bark here would
    // cost a fetch per fragment to learn nothing.
    if (uIsLeaves == 1 && texture(uTwigTex, vUV).a < uAlphaCut)
        discard;
}
