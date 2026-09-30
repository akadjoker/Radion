#include "PCH.h"

#include "GameObject.h"
#include "Light.h"
#include "Scene.h"
#include "ScriptCache.h"
#include "ZenBehaviour.h"

#include "zen/vm.h"

#include <chrono>
#include <cstdio>
#include <string>

using namespace Radion;

namespace
{

f64 milliseconds(std::chrono::steady_clock::time_point begin,
                 std::chrono::steady_clock::time_point end)
{
    return std::chrono::duration<f64, std::milli>(end - begin).count();
}

// One Scene::update() runs on_start() and its native-call loop; timed around that one call, as a script crosses into C++ as one VM invocation.
void runScenario(Scene& scene, GameObject* object, const char* label, const std::string& script,
                 u64 iterations)
{
    ZenBehaviour* behaviour = object->findComponent<ZenBehaviour>();
    if (!behaviour)
        behaviour = object->addComponent<ZenBehaviour>();

    if (!behaviour->loadSource(script))
    {
        std::printf("%-38s FAILED TO LOAD: %s\n", label, behaviour->lastError().c_str());
        return;
    }
    const auto begin = std::chrono::steady_clock::now();
    scene.update(1.0f / 60.0f);
    const auto end = std::chrono::steady_clock::now();

    if (behaviour->hasError())
    {
        std::printf("%-38s SCRIPT ERROR: %s\n", label, behaviour->lastError().c_str());
        return;
    }

    const f64 ms = milliseconds(begin, end);
    const f64 nsPerCall = ms * 1.0e6 / static_cast<f64>(iterations);
    std::printf("%-38s %9llu calls  %9.3f ms  %8.2f ns/call\n", label,
               static_cast<unsigned long long>(iterations), ms, nsPerCall);
}

} // namespace

int main()
{
    Scene scene;

    constexpr u32 kObjectCount = 10000;
    GameObject* benchObject = nullptr;
    for (u32 i = 0; i < kObjectCount; ++i)
    {
        char name[32];
        std::snprintf(name, sizeof(name), "Object%u", i);
        GameObject* object = scene.createGameObject(name);
        if (i == 0)
        {
            object->addComponent<DirectionalLight>();
            benchObject = object;
        }
    }

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f); // Flushes the pending adds, so find() sees every object.

    runScenario(scene, benchObject, "A: node.yaw()",
               "class BenchYaw:\n"
               "    def on_start(self):\n"
               "        for i in range(1000000):\n"
               "            self.node.yaw(0.1)\n",
               1000000);

    runScenario(scene, benchObject, "B: node.get_position()",
               "class BenchGetPosition:\n"
               "    def on_start(self):\n"
               "        for i in range(1000000):\n"
               "            p = self.node.get_position()\n",
               1000000);

    runScenario(scene, benchObject, "C: light.set_intensity() [cached]",
               "class BenchSetIntensity:\n"
               "    def on_start(self):\n"
               "        light = self.node.get_component(Light)\n"
               "        for i in range(1000000):\n"
               "            light.set_intensity(1.0)\n",
               1000000);

    runScenario(scene, benchObject, "D: scene.find()",
               "class BenchFind:\n"
               "    def on_start(self):\n"
               "        for i in range(100000):\n"
               "            found = self.scene.find(\"Object7777\")\n",
               100000);

    return 0;
}
