#include "PCH.h"
#include "MiniBatch.h"

#include "Log.h"

#include <glad.h>
#include <glm/gtc/type_ptr.hpp>

using namespace Radion;

namespace
{

const char* kBatchVertexShader = R"glsl(
#version 450 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec4 aColor;

uniform mat4 uViewProjection;

out vec4 vColor;

void main()
{
    vColor = aColor;
    gl_Position = uViewProjection * vec4(aPosition, 1.0);
}
)glsl";

const char* kBatchFragmentShader = R"glsl(
#version 450 core

in vec4 vColor;
out vec4 outColor;

void main()
{
    outColor = vColor;
}
)glsl";

} // namespace

MiniBatch::MiniBatch() = default;

MiniBatch::~MiniBatch()
{
    shutdown();
}

bool MiniBatch::initialize()
{
    if (!compileShader())
        return false;

    glGenVertexArrays(1, &mVAO);
    glGenBuffers(1, &mVBO);

    glBindVertexArray(mVAO);
    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
    glBindVertexArray(0);

    return true;
}

void MiniBatch::shutdown()
{
    if (mVAO)
    {
        glDeleteVertexArrays(1, &mVAO);
        mVAO = 0;
    }
    if (mVBO)
    {
        glDeleteBuffers(1, &mVBO);
        mVBO = 0;
    }
    if (mShaderProgram)
    {
        glDeleteProgram(mShaderProgram);
        mShaderProgram = 0;
    }
    mVBOCapacity = 0;
}

bool MiniBatch::compileShader()
{
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &kBatchVertexShader, nullptr);
    glCompileShader(vertexShader);

    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
        Log::error("MiniBatch: vertex shader compilation failed: %s", infoLog);
        glDeleteShader(vertexShader);
        return false;
    }

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &kBatchFragmentShader, nullptr);
    glCompileShader(fragmentShader);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);
        Log::error("MiniBatch: fragment shader compilation failed: %s", infoLog);
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
        Log::error("MiniBatch: shader program linking failed: %s", infoLog);
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

void MiniBatch::begin()
{
    mTriangles.clear();
    mLines.clear();
    mPoints.clear();
}

void MiniBatch::line(const glm::vec3& a, const glm::vec3& b, const glm::vec4& color)
{
    mLines.push_back({a, color});
    mLines.push_back({b, color});
}

void MiniBatch::point(const glm::vec3& p, const glm::vec4& color, f32 size)
{
    mPoints.push_back({p, color});
    mPointSize = size;
}

void MiniBatch::triangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec4& color)
{
    mTriangles.push_back({a, color});
    mTriangles.push_back({b, color});
    mTriangles.push_back({c, color});
}

void MiniBatch::grid(f32 y, u32 slices, f32 spacing, bool axes)
{
    if (slices == 0 || !(spacing > 0.0f))
        return;

    const f32 extent = static_cast<f32>(slices) * spacing;
    const glm::vec4 minor(0.28f, 0.30f, 0.33f, 1.0f);
    const glm::vec4 red(0.85f, 0.2f, 0.2f, 1.0f);
    const glm::vec4 blue(0.25f, 0.4f, 0.9f, 1.0f);

    for (s32 i = -static_cast<s32>(slices); i <= static_cast<s32>(slices); ++i)
    {
        const f32 offset = static_cast<f32>(i) * spacing;
        const glm::vec4& xColor = (axes && i == 0) ? red : minor;
        const glm::vec4& zColor = (axes && i == 0) ? blue : minor;
        line(glm::vec3(-extent, y, offset), glm::vec3(extent, y, offset), xColor);
        line(glm::vec3(offset, y, -extent), glm::vec3(offset, y, extent), zColor);
    }
}

void MiniBatch::flush(const glm::mat4& viewProjection)
{
    if (!mShaderProgram)
        return;

    const usize triangleCount = mTriangles.size();
    const usize lineCount = mLines.size();
    const usize pointCount = mPoints.size();
    const usize total = triangleCount + lineCount + pointCount;
    if (total == 0)
        return;

    std::vector<Vertex> upload;
    upload.reserve(total);
    upload.insert(upload.end(), mTriangles.begin(), mTriangles.end());
    upload.insert(upload.end(), mLines.begin(), mLines.end());
    upload.insert(upload.end(), mPoints.begin(), mPoints.end());

    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    if (total > mVBOCapacity)
    {
        glBufferData(GL_ARRAY_BUFFER, upload.size() * sizeof(Vertex), upload.data(), GL_DYNAMIC_DRAW);
        mVBOCapacity = static_cast<u32>(total);
    }
    else
    {
        glBufferSubData(GL_ARRAY_BUFFER, 0, upload.size() * sizeof(Vertex), upload.data());
    }

    glUseProgram(mShaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(mShaderProgram, "uViewProjection"), 1, GL_FALSE,
                        glm::value_ptr(viewProjection));

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE); // overlay only, never punches a hole in the mesh's own depth

    glBindVertexArray(mVAO);

    GLint first = 0;
    if (triangleCount > 0)
    {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);
        glDrawArrays(GL_TRIANGLES, first, static_cast<GLsizei>(triangleCount));
        glDisable(GL_POLYGON_OFFSET_FILL);
        first += static_cast<GLint>(triangleCount);
    }
    if (lineCount > 0)
    {
        glDrawArrays(GL_LINES, first, static_cast<GLsizei>(lineCount));
        first += static_cast<GLint>(lineCount);
    }
    if (pointCount > 0)
    {
        glPointSize(mPointSize);
        glDrawArrays(GL_POINTS, first, static_cast<GLsizei>(pointCount));
    }

    glBindVertexArray(0);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}
