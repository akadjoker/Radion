#version 450 core
layout(location = 0) in vec3 aPosition;
layout(std140, binding = 0) uniform BakeBlock {
    mat4 uShadowVP;
    mat4 uModel;
    vec4 uLightDirection;
    vec4 uLightColor;
};
void main() { gl_Position = uShadowVP * uModel * vec4(aPosition, 1.0); }
