#version 450 core

layout (location = 0) in vec3 aPos;

layout(std140, binding = 0) uniform VolumetricProxy
{
    mat4 uViewProj;
    vec4 uLightPosAndScale; // xyz position, w scale
};

void main()
{
    vec3 world = uLightPosAndScale.xyz + aPos * uLightPosAndScale.w;
    gl_Position = uViewProj * vec4(world, 1.0);
}
