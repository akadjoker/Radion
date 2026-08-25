#version 450 core

// What a particle looks like: a computed radial falloff (a soft round dot,
// independent of any asset) multiplied by whatever texture the render call
// bound - a 1x1 white pixel when the system's own `texture` field is unset,
// so the shape alone still works exactly as before. Set that field to an
// actual soft sprite and every particle in the pool reads as one, the same
// way any other particle system leans on a texture instead of raw math.

in vec2  vUV;
in vec4  vColor;
in float vLifeLerp;

layout(binding = 0) uniform sampler2D uParticleTex;

layout(std140, binding = 0) uniform DrawBlock
{
    mat4 uViewProj;
    vec4 uCameraRight;
    vec4 uCameraUp; // w = 1.0 for additive blending
};

#define uAdditive (uCameraUp.w > 0.5)

out vec4 FragColor;

void main()
{
    // Distance to the quad's centre: 0 in the middle, 1 at the edge of the
    // inscribed circle.
    vec2 d = vUV * 2.0 - 1.0;
    float r = dot(d, d); // r^2, avoids the square root

    // NO discard. Two reasons:
    //  - discard turns off early-Z for the whole shader, and with transparent
    //    particles overlapping that costs real fill rate;
    //  - it is not needed: alpha 0 contributes nothing in either blend mode
    //    (additive sums zero, normal alpha blends nothing in).
    //
    // The max() is mandatory: r reaches 2 at the quad's corners, so (1-r)
    // goes negative there, and squaring it would turn it POSITIVE again - the
    // four corners would light up. It read as a bright square around every
    // particle.
    float falloff = max(0.0, 1.0 - r);
    falloff *= falloff;

    vec4 tex = texture(uParticleTex, vUV);
    vec4 color = vColor;
    color.rgb *= tex.rgb;
    color.a *= falloff * tex.a;

    if (uAdditive)
    {
        // Additive blend here is (GL_SRC_ALPHA, GL_ONE), not (GL_ONE, GL_ONE)
        // - the GPU still multiplies by the fragment's own alpha before
        // adding it to the framebuffer. Premultiplying color by alpha AND
        // outputting alpha 0 double-multiplies it away to nothing; output
        // alpha 1 instead so GL_SRC_ALPHA passes the premultiplied color
        // through unscaled.
        FragColor = vec4(color.rgb * color.a, 1.0);
    }
    else
    {
        FragColor = color;
    }
}
