#include "PCH.h"
#include "MiniRenderer.h"
#include "Engine.h"
#include "Log.h"
#include "Mesh.h"

#include <glad.h>
#include "Math.h"
#include <cmath>

using namespace Radion;

namespace
{

const char* kMiniVertexShader = R"glsl(
#version 450 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;
layout(location = 3) in vec4 aTangent;
layout(location = 4) in vec4 aJoints;
layout(location = 5) in vec4 aWeights;
layout(location = 6) in float aSelected;
layout(location = 7) in vec4 aColor;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;
uniform mat4 uBonePalette[128]; // kMiniRendererMaxBones, matched in MiniRenderer.h

out VS_OUT {
    vec3 positionWS;
    vec3 normalWS;
    flat vec3 flatNormalWS; // provoking vertex's normal - per-face, no extra data
    vec2 texCoord;
    mat3 TBN;
    flat float selected;
    vec4 color;
} vs_out;

void main()
{
    mat4 skin = uBonePalette[int(aJoints.x)] * aWeights.x
              + uBonePalette[int(aJoints.y)] * aWeights.y
              + uBonePalette[int(aJoints.z)] * aWeights.z
              + uBonePalette[int(aJoints.w)] * aWeights.w;
    mat3 skinNormal = mat3(skin);

    vec3 skinnedPos = vec3(skin * vec4(aPosition, 1.0));
    vec3 skinnedNormal = normalize(skinNormal * aNormal);
    vec3 skinnedTangent = normalize(skinNormal * aTangent.xyz);

    vs_out.positionWS = vec3(uModel * vec4(skinnedPos, 1.0));
    vs_out.normalWS = normalize(uNormalMatrix * skinnedNormal);
    vs_out.flatNormalWS = vs_out.normalWS;
    vs_out.texCoord = aTexCoord;

    vec3 N = vs_out.normalWS;
    vec3 T = normalize(uNormalMatrix * skinnedTangent);
    T = normalize(T - dot(T, N) * N);
    vec3 B = cross(N, T) * aTangent.w;
    vs_out.TBN = mat3(T, B, N);
    vs_out.selected = aSelected;
    vs_out.color = aColor;

    gl_Position = uProjection * uView * vec4(vs_out.positionWS, 1.0);
}
)glsl";

const char* kMiniFragmentShader = R"glsl(
#version 450 core

in VS_OUT {
    vec3 positionWS;
    vec3 normalWS;
    flat vec3 flatNormalWS;
    vec2 texCoord;
    mat3 TBN;
    flat float selected;
    vec4 color;
} fs_in;

out vec4 outColor;

uniform int uDebugView; // 0 = off, 1 = normals, 2 = tangents, 3 = uvs
uniform bool uFacetedShading;
uniform bool uUnlit;
uniform bool uVertexColors; // multiply the surface by the mesh's per-vertex colour (linear)

layout(binding = 0) uniform sampler2D uAlbedoMap;
layout(binding = 1) uniform sampler2D uNormalMap;
layout(binding = 2) uniform sampler2D uRoughnessMap;
layout(binding = 3) uniform sampler2D uMetallicMap;

uniform vec3 uLightDirection;
uniform float uLightIntensity;
uniform vec3 uAmbientColor;
uniform float uAmbientIntensity;
uniform vec3 uCameraPos;

uniform int uShadingMode; // 0 = solid (N.L only), 1 = textured (full PBR)
uniform float uAlpha;
uniform vec3 uTint;
// Scalars the material contributes where it has no map for them (x = roughness,
// y = metallic). A map, when there is one, already says everything, so the
// factor is 1 there and imported textured assets look as they always did.
uniform vec2 uSurfaceFactor;

uniform bool uPointPass; // the GL_POINTS overlay: flat colour, selection aware
uniform vec3 uPointColor;
uniform vec3 uSelectedPointColor;

const float PI = 3.14159265359;

vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

float distributionGGX(vec3 N, vec3 H, float roughness)
{
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    return a2 / (PI * denom * denom);
}

float geometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r * r) / 8.0;
    return NdotV / (NdotV * (1.0 - k) + k);
}

float geometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    return geometrySchlickGGX(NdotV, roughness) * geometrySchlickGGX(NdotL, roughness);
}

void main()
{
    if (uPointPass)
    {
        // 2 marks a hidden vertex: it is still in the buffer, it just is not drawn.
        if (fs_in.selected > 1.5)
            discard;
        outColor = vec4(fs_in.selected > 0.5 ? uSelectedPointColor : uPointColor, 1.0);
        return;
    }

    vec3 shadingNormal = normalize(uFacetedShading ? fs_in.flatNormalWS : fs_in.normalWS);

    if (uDebugView == 1)
    {
        outColor = vec4(shadingNormal * 0.5 + 0.5, 1.0);
        return;
    }
    if (uDebugView == 2)
    {
        outColor = vec4(normalize(fs_in.TBN[0]) * 0.5 + 0.5, 1.0);
        return;
    }
    if (uDebugView == 3)
    {
        outColor = vec4(fract(fs_in.texCoord), 0.0, 1.0);
        return;
    }

    vec3 tint = uVertexColors ? uTint * fs_in.color.rgb : uTint;
    vec3 color;

    if (uUnlit)
    {
        color = tint;
    }
    else if (uShadingMode == 0)
    {
        // Solid preview: face/vertex normal lit with N.L only, no texture
        // fetches and no BRDF - the cheap path multi-viewport playback wants.
        float NdotL = max(dot(shadingNormal, normalize(-uLightDirection)), 0.0);
        color = tint * (uAmbientColor * uAmbientIntensity + NdotL * uLightIntensity);
    }
    else
    {
        vec3 albedo = pow(texture(uAlbedoMap, fs_in.texCoord).rgb, vec3(2.2)) * tint;
        float roughness = clamp(texture(uRoughnessMap, fs_in.texCoord).r * uSurfaceFactor.x, 0.05, 1.0);
        float metallic = texture(uMetallicMap, fs_in.texCoord).r * uSurfaceFactor.y;

        vec3 normalSample = texture(uNormalMap, fs_in.texCoord).rgb * 2.0 - 1.0;
        vec3 N = normalize(fs_in.TBN * normalSample);

        vec3 V = normalize(uCameraPos - fs_in.positionWS);
        vec3 L = normalize(-uLightDirection);
        vec3 H = normalize(V + L);

        vec3 F0 = mix(vec3(0.04), albedo, metallic);
        vec3 F = fresnelSchlick(max(dot(H, V), 0.0), F0);

        float NDF = distributionGGX(N, H, roughness);
        float G = geometrySmith(N, V, L, roughness);
        vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);

        float denom = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001;
        vec3 specular = (NDF * G * F) / denom;

        float NdotL = max(dot(N, L), 0.0);
        vec3 Lo = (kD * albedo / PI + specular) * uLightIntensity * NdotL;
        vec3 ambient = uAmbientColor * albedo * uAmbientIntensity;

        color = ambient + Lo;
    }

    color = color / (color + vec3(1.0));
    color = pow(color, vec3(1.0 / 2.2));

    outColor = vec4(color, uAlpha);
}
)glsl";

struct MiniVertex
{
    Math::vec3 position;
    Math::vec3 normal;
    Math::vec2 uv;
    Math::vec4 tangent;
    Math::vec4 joints;
    Math::vec4 weights;
    Math::vec4 color;
};

Math::vec3 colorForSubmesh(u32 index)
{
    const f32 hue = std::fmod(static_cast<f32>(index) * 0.6180339887f, 1.0f);
    const f32 h6 = hue * 6.0f;
    const f32 x = 1.0f - std::fabs(std::fmod(h6, 2.0f) - 1.0f);
    if (h6 < 1.0f)
        return Math::vec3(1.0f, x, 0.0f);
    if (h6 < 2.0f)
        return Math::vec3(x, 1.0f, 0.0f);
    if (h6 < 3.0f)
        return Math::vec3(0.0f, 1.0f, x);
    if (h6 < 4.0f)
        return Math::vec3(0.0f, x, 1.0f);
    if (h6 < 5.0f)
        return Math::vec3(x, 0.0f, 1.0f);
    return Math::vec3(1.0f, 0.0f, x);
}

GLuint resolveSlotTexture(const Material* material, MaterialSlot slot, GLuint fallback)
{
    if (!material || !material->textures[slot].texture.valid())
        return fallback;
    return GPU::getSingleton().nativeTextureId(material->textures[slot].texture);
}

void bindMaterialTextures(const Material* material, GLuint whiteTexture, GLuint flatNormalTexture)
{
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, resolveSlotTexture(material, SlotAlbedo, whiteTexture));
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, resolveSlotTexture(material, SlotNormal, flatNormalTexture));
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, resolveSlotTexture(material, SlotSurface, whiteTexture));
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, resolveSlotTexture(material, SlotEmissive, whiteTexture));
}

// glTF/PBR rule: the factor multiplies the albedo texture, and alone colours an untextured surface.
Math::vec3 baseColorFactor(const Material* material)
{
    return material ? Math::vec3(material->params.baseColor) : Math::vec3(1.0f);
}

// A submesh with no material is plain matte, not the fully metallic black a white metallic map would give.
Math::vec2 surfaceFactor(const Material* material)
{
    static const Material kDefault;
    const Material& source = material ? *material : kDefault;
    const f32 roughness = source.textures[SlotSurface].texture.valid() ? 1.0f : source.params.surface.x;
    const f32 metallic = source.textures[SlotEmissive].texture.valid() ? 1.0f : source.params.surface.y;
    return Math::vec2(roughness, metallic);
}

const Material* materialForSubmesh(const MeshData& mesh, u32 submeshIndex)
{
    if (submeshIndex >= mesh.submeshes.size())
        return nullptr;
    const u32 slot = mesh.submeshes[submeshIndex].materialSlot;
    return slot < mesh.materials.size() ? &mesh.materials[slot] : nullptr;
}

} // namespace

MiniRenderer::MiniRenderer(Engine& engine)
    : mEngine(engine)
{
}

MiniRenderer::~MiniRenderer()
{
    shutdown();
}

bool MiniRenderer::initialize()
{
    if (!compileShaders())
        return false;
    if (!createDefaultTextures())
        return false;

    glGenVertexArrays(1, &mVAO);
    glGenBuffers(1, &mVBO);
    glGenBuffers(1, &mSelectionVBO);
    glGenBuffers(1, &mEBO);

    return true;
}

void MiniRenderer::shutdown()
{
    destroyBuffers();

    if (mWhiteTexture)
    {
        glDeleteTextures(1, &mWhiteTexture);
        mWhiteTexture = 0;
    }
    if (mFlatNormalTexture)
    {
        glDeleteTextures(1, &mFlatNormalTexture);
        mFlatNormalTexture = 0;
    }
    if (mShaderProgram)
    {
        glDeleteProgram(mShaderProgram);
        mShaderProgram = 0;
    }
}

void MiniRenderer::invalidate()
{
    mUploadedMesh = nullptr;
}

void MiniRenderer::destroyBuffers()
{
    if (mVAO)
    {
        glDeleteVertexArrays(1, &mVAO);
        mVAO = 0;
    }
    if (mSelectionVBO)
    {
        glDeleteBuffers(1, &mSelectionVBO);
        mSelectionVBO = 0;
        mSelectionCapacity = 0;
    }
    if (mVBO)
    {
        glDeleteBuffers(1, &mVBO);
        mVBO = 0;
    }
    if (mEBO)
    {
        glDeleteBuffers(1, &mEBO);
        mEBO = 0;
    }
    mUploadedMesh = nullptr;
    mIndexCount = 0;
}

bool MiniRenderer::createDefaultTextures()
{
    const u8 white[4] = {255, 255, 255, 255};
    glGenTextures(1, &mWhiteTexture);
    glBindTexture(GL_TEXTURE_2D, mWhiteTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, white);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    const u8 flatNormal[4] = {128, 128, 255, 255};
    glGenTextures(1, &mFlatNormalTexture);
    glBindTexture(GL_TEXTURE_2D, mFlatNormalTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, flatNormal);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glBindTexture(GL_TEXTURE_2D, 0);
    return mWhiteTexture != 0 && mFlatNormalTexture != 0;
}

bool MiniRenderer::compileShaders()
{
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &kMiniVertexShader, nullptr);
    glCompileShader(vertexShader);

    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
        Log::error("MiniRenderer: vertex shader compilation failed: %s", infoLog);
        glDeleteShader(vertexShader);
        return false;
    }

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &kMiniFragmentShader, nullptr);
    glCompileShader(fragmentShader);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);
        Log::error("MiniRenderer: fragment shader compilation failed: %s", infoLog);
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        return false;
    }

    mShaderProgram = glCreateProgram();
    glAttachShader(mShaderProgram, vertexShader);
    glAttachShader(mShaderProgram, fragmentShader);
    glLinkProgram(mShaderProgram);

    glGetProgramiv(mShaderProgram, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(mShaderProgram, 512, nullptr, infoLog);
        Log::error("MiniRenderer: shader program linking failed: %s", infoLog);
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glDeleteProgram(mShaderProgram);
        mShaderProgram = 0;
        return false;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return true;
}

void MiniRenderer::uploadMesh(const MeshData& mesh)
{
    const usize vertexCount = mesh.vertexCount();

    std::vector<MiniVertex> vertices(vertexCount);
    for (usize v = 0; v < vertexCount; ++v)
    {
        vertices[v].position = mesh.positions[v];
        vertices[v].normal = v < mesh.normals.size() ? mesh.normals[v] : Math::vec3(0.0f, 1.0f, 0.0f);
        vertices[v].uv = v < mesh.uvs.size() ? mesh.uvs[v] : Math::vec2(0.0f);
        vertices[v].tangent = v < mesh.tangents.size() ? mesh.tangents[v] : Math::vec4(1.0f, 0.0f, 0.0f, 1.0f);

        if (v < mesh.colors.size())
        {
            const u32 packed = mesh.colors[v];
            vertices[v].color = Math::vec4(static_cast<f32>(packed & 0xFF), static_cast<f32>((packed >> 8) & 0xFF),
                                          static_cast<f32>((packed >> 16) & 0xFF),
                                          static_cast<f32>((packed >> 24) & 0xFF)) /
                                255.0f;
        }
        else
            vertices[v].color = Math::vec4(1.0f);

        if (v < mesh.skin.size())
        {
            const MeshSkinVertex& skin = mesh.skin[v];
            vertices[v].joints = Math::vec4(skin.joints[0], skin.joints[1], skin.joints[2], skin.joints[3]);
            vertices[v].weights = skin.weights;
        }
        else
        {
            vertices[v].joints = Math::vec4(0.0f);
            vertices[v].weights = Math::vec4(1.0f, 0.0f, 0.0f, 0.0f);
        }
    }

    glBindVertexArray(mVAO);

    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(MiniVertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(u32), mesh.indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MiniVertex), (void*)offsetof(MiniVertex, position));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MiniVertex), (void*)offsetof(MiniVertex, normal));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(MiniVertex), (void*)offsetof(MiniVertex, uv));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(MiniVertex), (void*)offsetof(MiniVertex, tangent));
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(MiniVertex), (void*)offsetof(MiniVertex, joints));
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(MiniVertex), (void*)offsetof(MiniVertex, weights));
    glEnableVertexAttribArray(7);
    glVertexAttribPointer(7, 4, GL_FLOAT, GL_FALSE, sizeof(MiniVertex), (void*)offsetof(MiniVertex, color));

    // Own stream so a vertex selection re-uploads one byte per vertex, not the interleaved geometry.
    glBindBuffer(GL_ARRAY_BUFFER, mSelectionVBO);
    const std::vector<u8> cleared(vertexCount, 0);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertexCount), cleared.data(),
                 GL_DYNAMIC_DRAW);
    mSelectionCapacity = static_cast<u32>(vertexCount);
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 1, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(u8), (void*)0);

    glBindVertexArray(0);

    mIndexCount = static_cast<u32>(mesh.indices.size());
    mVertexCount = static_cast<u32>(vertexCount);
    mUploadedMesh = &mesh;
    ++mUploadRevision;
}

void MiniRenderer::setHiddenFaces(const u8* faceHidden, u32 faceCount)
{
    mHiddenFaces.clear();
    mHasHiddenFaces = false;
    if (!faceHidden || faceCount == 0)
        return;

    mHiddenFaces.assign(faceHidden, faceHidden + faceCount);
    for (const u8 hidden : mHiddenFaces)
    {
        if (hidden)
        {
            mHasHiddenFaces = true;
            break;
        }
    }
}

void MiniRenderer::drawTriangleRange(u32 indexOffset, u32 indexCount)
{
    const u32 firstFace = indexOffset / 3;
    const u32 faceCount = indexCount / 3;
    // A mask made for another mesh is worse than none: ignore it.
    const bool masked = mHasHiddenFaces && mHiddenFaces.size() == mIndexCount / 3;
    if (!masked)
    {
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indexCount), GL_UNSIGNED_INT,
                       reinterpret_cast<const void*>(static_cast<uintptr_t>(indexOffset) * sizeof(u32)));
        return;
    }

    std::vector<GLsizei> counts;
    std::vector<const void*> offsets;
    u32 runStart = 0;
    u32 runLength = 0;
    auto flush = [&]()
    {
        if (runLength == 0)
            return;
        counts.push_back(static_cast<GLsizei>(runLength * 3));
        offsets.push_back(reinterpret_cast<const void*>(static_cast<uintptr_t>(runStart) * 3 * sizeof(u32)));
        runLength = 0;
    };
    for (u32 face = firstFace; face < firstFace + faceCount; ++face)
    {
        if (mHiddenFaces[face])
        {
            flush();
            continue;
        }
        if (runLength == 0)
            runStart = face;
        ++runLength;
    }
    flush();

    if (!counts.empty())
        glMultiDrawElements(GL_TRIANGLES, counts.data(), GL_UNSIGNED_INT, offsets.data(),
                            static_cast<GLsizei>(counts.size()));
}

void MiniRenderer::setVertexSelection(const u8* selected, u32 count)
{
    if (!mSelectionVBO || !selected || count == 0)
        return;

    glBindVertexArray(mVAO);
    glBindBuffer(GL_ARRAY_BUFFER, mSelectionVBO);

    if (count > mSelectionCapacity)
    {
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(count), selected, GL_DYNAMIC_DRAW);
        mSelectionCapacity = count;
        glEnableVertexAttribArray(6);
        glVertexAttribPointer(6, 1, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(u8), (void*)0);
    }
    else
    {
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(count), selected);
    }

    glBindVertexArray(0);
}

void MiniRenderer::renderViewport(const MeshData* mesh,
                                   const Math::mat4& viewMatrix,
                                   const Math::mat4& projectionMatrix,
                                   const Math::vec3& cameraPos,
                                   const MiniDrawParams& params)
{
    if (!mShaderProgram || !mesh || mesh->positions.empty() || mesh->indices.empty())
        return;

    if (mesh != mUploadedMesh)
        uploadMesh(*mesh);

    if (mIndexCount == 0)
        return;

    glUseProgram(mShaderProgram);

    const Math::mat4 model(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(mShaderProgram, "uModel"), 1, GL_FALSE, Math::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(mShaderProgram, "uView"), 1, GL_FALSE, Math::value_ptr(viewMatrix));
    glUniformMatrix4fv(glGetUniformLocation(mShaderProgram, "uProjection"), 1, GL_FALSE,
                        Math::value_ptr(projectionMatrix));

    const Math::mat3 normalMatrix = Math::transpose(Math::inverse(Math::mat3(model)));
    glUniformMatrix3fv(glGetUniformLocation(mShaderProgram, "uNormalMatrix"), 1, GL_FALSE,
                        Math::value_ptr(normalMatrix));

    if (params.bonePalette && params.boneCount > 0)
    {
        const u32 count = params.boneCount < kMiniRendererMaxBones ? params.boneCount : kMiniRendererMaxBones;
        glUniformMatrix4fv(glGetUniformLocation(mShaderProgram, "uBonePalette"), static_cast<GLsizei>(count),
                            GL_FALSE, Math::value_ptr(params.bonePalette[0]));
    }
    else
    {
        const Math::mat4 identity(1.0f);
        glUniformMatrix4fv(glGetUniformLocation(mShaderProgram, "uBonePalette"), 1, GL_FALSE,
                            Math::value_ptr(identity));
    }

    glUniform3fv(glGetUniformLocation(mShaderProgram, "uLightDirection"), 1, Math::value_ptr(mConfig.lightDirection));
    glUniform1f(glGetUniformLocation(mShaderProgram, "uLightIntensity"), mConfig.lightIntensity);
    glUniform3fv(glGetUniformLocation(mShaderProgram, "uAmbientColor"), 1, Math::value_ptr(mConfig.ambientColor));
    glUniform1f(glGetUniformLocation(mShaderProgram, "uAmbientIntensity"), mConfig.ambientIntensity);
    glUniform3fv(glGetUniformLocation(mShaderProgram, "uCameraPos"), 1, Math::value_ptr(cameraPos));

    const f32 effectiveAlpha = (params.xray && params.alpha >= 1.0f) ? 0.35f : params.alpha;

    const int shadingMode = params.mode == MiniRenderMode::Textured ? 1 : 0;
    glUniform1i(glGetUniformLocation(mShaderProgram, "uShadingMode"), shadingMode);
    glUniform1f(glGetUniformLocation(mShaderProgram, "uAlpha"), effectiveAlpha);
    // Only the textured look reads material colour.
    const bool useMaterialColor = shadingMode == 1;
    const Math::vec3 firstTint =
        useMaterialColor ? params.tint * baseColorFactor(materialForSubmesh(*mesh, 0)) : params.tint;
    glUniform3fv(glGetUniformLocation(mShaderProgram, "uTint"), 1, Math::value_ptr(firstTint));
    const GLint surfaceLocation = glGetUniformLocation(mShaderProgram, "uSurfaceFactor");
    const Math::vec2 firstSurface = useMaterialColor ? surfaceFactor(materialForSubmesh(*mesh, 0)) : Math::vec2(1.0f);
    glUniform2fv(surfaceLocation, 1, Math::value_ptr(firstSurface));
    glUniform1i(glGetUniformLocation(mShaderProgram, "uDebugView"), static_cast<int>(params.debugView));
    glUniform1i(glGetUniformLocation(mShaderProgram, "uFacetedShading"), params.facetedShading ? 1 : 0);
    glUniform1i(glGetUniformLocation(mShaderProgram, "uUnlit"), params.unlit ? 1 : 0);
    glUniform1i(glGetUniformLocation(mShaderProgram, "uVertexColors"), params.vertexColors ? 1 : 0);

    if (shadingMode == 1)
        bindMaterialTextures(materialForSubmesh(*mesh, 0), mWhiteTexture, mFlatNormalTexture);

    // X-ray: depth test off so front and back geometry both reach blending.
    if (params.xray)
        glDisable(GL_DEPTH_TEST);
    else
        glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Blended draws never write depth so they do not occlude what is behind.
    const bool blended = effectiveAlpha < 1.0f;
    glDepthMask(blended ? GL_FALSE : GL_TRUE);
    if (blended)
    {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    else
    {
        glDisable(GL_BLEND);
    }

    glPolygonMode(GL_FRONT_AND_BACK, params.mode == MiniRenderMode::Wireframe ? GL_LINE : GL_FILL);

    glBindVertexArray(mVAO);
    const bool colorPerSubmesh = params.colorBySubmesh && mesh->submeshes.size() > 1;
    // Two submeshes with different materials cannot share one glDrawElements.
    const bool texturedPerSubmesh = shadingMode == 1 && mesh->submeshes.size() > 1;
    const bool perSubmesh =
        (colorPerSubmesh || texturedPerSubmesh || params.submeshVisible) && !mesh->submeshes.empty();
    if (perSubmesh)
    {
        const GLint tintLocation = glGetUniformLocation(mShaderProgram, "uTint");
        for (u32 i = 0; i < static_cast<u32>(mesh->submeshes.size()); ++i)
        {
            if (params.submeshVisible && i < params.submeshVisibleCount && !params.submeshVisible[i])
                continue;
            const SubMesh& submesh = mesh->submeshes[i];
            if (texturedPerSubmesh)
                bindMaterialTextures(materialForSubmesh(*mesh, i), mWhiteTexture, mFlatNormalTexture);
            Math::vec3 tint = colorPerSubmesh ? params.tint * colorForSubmesh(i) : params.tint;
            if (useMaterialColor)
            {
                const Material* material = materialForSubmesh(*mesh, i);
                tint *= baseColorFactor(material);
                glUniform2fv(surfaceLocation, 1, Math::value_ptr(surfaceFactor(material)));
            }
            glUniform3fv(tintLocation, 1, Math::value_ptr(tint));
            drawTriangleRange(submesh.indexOffset, submesh.indexCount);
        }
    }
    else
    {
        drawTriangleRange(0, mIndexCount);
    }

    // Overlay passes reuse the program, drawn as flat-tinted marks over the shaded result.
    if (params.showWireframeOverlay && params.mode != MiniRenderMode::Wireframe)
    {
        glUniform1i(glGetUniformLocation(mShaderProgram, "uDebugView"), 0);
        glUniform1i(glGetUniformLocation(mShaderProgram, "uUnlit"), 0);
        glUniform1i(glGetUniformLocation(mShaderProgram, "uShadingMode"), 0);
        glUniform1f(glGetUniformLocation(mShaderProgram, "uAmbientIntensity"), 1.0f);
        glUniform3f(glGetUniformLocation(mShaderProgram, "uAmbientColor"), 1.0f, 1.0f, 1.0f);
        glUniform1f(glGetUniformLocation(mShaderProgram, "uLightIntensity"), 0.0f);
        glUniform3f(glGetUniformLocation(mShaderProgram, "uTint"), 0.05f, 0.05f, 0.05f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glEnable(GL_POLYGON_OFFSET_LINE);
        glPolygonOffset(-1.0f, -1.0f);
        drawTriangleRange(0, mIndexCount);
        glDisable(GL_POLYGON_OFFSET_LINE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    if (params.showVertexPoints)
    {
        glUniform1i(glGetUniformLocation(mShaderProgram, "uDebugView"), 0);
        glUniform1i(glGetUniformLocation(mShaderProgram, "uPointPass"), 1);
        glUniform3fv(glGetUniformLocation(mShaderProgram, "uPointColor"), 1,
                     Math::value_ptr(params.vertexColor));
        glUniform3fv(glGetUniformLocation(mShaderProgram, "uSelectedPointColor"), 1,
                     Math::value_ptr(params.selectedVertexColor));
        // Points sit on their own surface and would fail GL_LESS; depth writes stay off so they do not occlude later
        // overlays.
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);
        glPointSize(params.vertexPointSize); // fixed-function size - GL_PROGRAM_POINT_SIZE stays off, the shader sets no gl_PointSize
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(mVertexCount));
        glDepthFunc(GL_LESS);
        glUniform1i(glGetUniformLocation(mShaderProgram, "uPointPass"), 0);
    }

    glBindVertexArray(0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}
