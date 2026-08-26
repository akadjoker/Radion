#include "PCH.h"

#include "AssetManager.h"
#include "Engine.h"
#include "Log.h"
#include "MiniBatch.h"
#include "MiniRenderer.h"

#include <glad.h>
#include "Math.h"

using namespace Radion;

int main(int, char**)
{
    Engine engine;
    EngineConfig config;
    config.title = "MiniRenderer smoke test";
    config.width = 1280;
    config.height = 720;
    if (!engine.initialize(config))
        return 1;
    engine.setBuiltinPanelsVisible(false);

    MeshData box;
    if (!Assets().buildMeshData(MeshDesc::box(::Radion::Math::vec3(1.5f)), box))
    {
        Log::error("MiniRenderer test: failed to build the box MeshData");
        return 1;
    }

    MiniRenderer renderer(engine);
    if (!renderer.initialize())
    {
        Log::error("MiniRenderer test: initialize() failed");
        return 1;
    }

    MiniBatch batch;
    if (!batch.initialize())
    {
        Log::error("MiniRenderer test: MiniBatch::initialize() failed");
        return 1;
    }

    f32 orbit = 0.0f;

    while (engine.update())
    {
        const f32 deltaTime = ::Radion::Math::min(engine.getWindow().getDeltaTime(), 0.1f);
        orbit += deltaTime * 0.6f;

        int width = 0, height = 0;
        engine.getWindow().getDrawableSize(width, height);
        if (width <= 0 || height <= 0)
        {
            engine.flip();
            continue;
        }

        const ::Radion::Math::vec3 cameraPos = ::Radion::Math::vec3(std::cos(orbit), 0.6f, std::sin(orbit)) * 4.0f;
        const ::Radion::Math::mat4 view = ::Radion::Math::lookAt(cameraPos, ::Radion::Math::vec3(0.0f), ::Radion::Math::vec3(0.0f, 1.0f, 0.0f));
        const ::Radion::Math::mat4 projection = ::Radion::Math::perspective(::Radion::Math::radians(60.0f),
                                                       static_cast<f32>(width) / static_cast<f32>(height),
                                                       0.1f, 100.0f);

        glViewport(0, 0, width, height);
        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        renderer.renderViewport(&box, view, projection, cameraPos);

        // MiniBatch smoke test: one yellow point per box vertex, and the
        // first triangle painted in translucent red - proves the vertex/face
        // highlight path a mesh-edit selection would use.
        batch.begin();
        for (const ::Radion::Math::vec3& p : box.positions)
            batch.point(p, ::Radion::Math::vec4(1.0f, 0.85f, 0.1f, 1.0f));
        if (box.indices.size() >= 3)
        {
            batch.triangle(box.positions[box.indices[0]], box.positions[box.indices[1]],
                           box.positions[box.indices[2]], ::Radion::Math::vec4(1.0f, 0.15f, 0.15f, 0.5f));
        }
        batch.flush(projection * view);

        engine.flip();
    }

    batch.shutdown();
    renderer.shutdown();
    engine.shutdown();
    return 0;
}
