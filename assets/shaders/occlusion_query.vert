#version 450 core
layout(location = 0) in vec3 aPosition;
layout(std140, binding = 0) uniform OcclusionBlock {
    mat4 uViewProjection;
    mat4 uModel;
};
void main()
{
    gl_Position = uViewProjection * uModel * vec4(aPosition, 1.0);
}
