#version 450 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 7) in vec2 aUV2;
layout(std140, binding = 0) uniform BakeBlock {
    mat4 uShadowVP;
    mat4 uModel;
    vec4 uLightDirection;
    vec4 uLightColor;
    vec4 uParams;
    vec4 uAmbientSky;
    vec4 uJitter; // xy = this sample's rasterization offset, in clip space
};
// A chart's edge pixels have their centre outside the triangle, so plain
// interpolation extrapolates their world position and normal to a point that
// is not on the surface - and then looks that point up in the shadow map.
// Centroid sampling moves the sample inside the covered area instead, which
// is what the border artifacts along every seam were.
centroid out vec3 vWorldPosition;
centroid out vec3 vNormal;
centroid out vec4 vShadowPosition;
void main()
{
    vec4 world = uModel * vec4(aPosition, 1.0);
    vWorldPosition = world.xyz;
    vNormal = mat3(uModel) * aNormal;
    vShadowPosition = uShadowVP * world;
    gl_Position = vec4(aUV2 * 2.0 - 1.0 + uJitter.xy, 0.0, 1.0);
}
