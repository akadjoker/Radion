#include "PCH.h"

#include "GameObject.h"
#include "Scene.h"
#include "SceneSerializer.h"

#include <chrono>
#include <cstdio>

using namespace Radion;

namespace
{
constexpr u32 kObjectCount = 4096;
constexpr u32 kBranchingFactor = 8;
constexpr u32 kRuns = 15;
constexpr u32 kUpdateFrames = 180;
constexpr u32 kMaxSamples = kUpdateFrames;

using Clock = std::chrono::steady_clock;

struct Distribution
{
    f64 samples[kMaxSamples] = {};
    u32 count = 0;

    void add(f64 milliseconds)
    {
        samples[count++] = milliseconds;
    }

    f64 minimum() const
    {
        f64 result = samples[0];
        for (u32 i = 1; i < count; ++i)
            result = samples[i] < result ? samples[i] : result;
        return result;
    }

    f64 maximum() const
    {
        f64 result = samples[0];
        for (u32 i = 1; i < count; ++i)
            result = samples[i] > result ? samples[i] : result;
        return result;
    }

    f64 median() const
    {
        f64 sorted[kMaxSamples] = {};
        for (u32 i = 0; i < count; ++i)
        {
            sorted[i] = samples[i];
            for (u32 j = i; j > 0 && sorted[j] < sorted[j - 1]; --j)
            {
                const f64 value = sorted[j];
                sorted[j] = sorted[j - 1];
                sorted[j - 1] = value;
            }
        }
        const u32 middle = count / 2;
        return count % 2 ? sorted[middle] : (sorted[middle - 1] + sorted[middle]) * 0.5;
    }
};

f64 elapsedMilliseconds(Clock::time_point begin, Clock::time_point end)
{
    return std::chrono::duration<f64, std::milli>(end - begin).count();
}

bool buildFixture(Scene& scene, GameObject* objects[kObjectCount])
{
    for (u32 index = 0; index < kObjectCount; ++index)
    {
        char name[32] = {};
        std::snprintf(name, sizeof(name), "benchmark-node-%u", index);
        GameObject* parent = index ? objects[(index - 1) / kBranchingFactor] : nullptr;
        GameObject* object = scene.createGameObject(name, parent);
        if (!object)
            return false;
        object->setPosition(glm::vec3(static_cast<f32>(index % 32),
                                      static_cast<f32>((index / 32) % 16),
                                      static_cast<f32>(index / 512)));
        objects[index] = object;
    }
    scene.update(0.0f);
    return scene.gameObjectCount() == kObjectCount;
}

bool deserializeFixture(const SceneSerializer& serializer, const nlohmann::json& document)
{
    Scene scene;
    SceneLoadResult result;
    return serializer.fromJson(document, scene, result) && result.success() &&
           scene.gameObjectCount() == kObjectCount;
}

void printDistribution(const char* name, const Distribution& values, bool trailingComma = true)
{
    std::printf("    \"%s\": {\"median_ms\": %.3f, \"min_ms\": %.3f, \"max_ms\": %.3f}%s\n",
                name, values.median(), values.minimum(), values.maximum(), trailingComma ? "," : "");
}
} // namespace

int main()
{
    Scene fixture;
    GameObject* objects[kObjectCount] = {};
    if (!buildFixture(fixture, objects))
    {
        std::fprintf(stderr, "radion_scene_bench: could not build fixture\n");
        return 1;
    }

    const SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(fixture);
    const std::string text = document.dump();
    if (text.empty())
    {
        std::fprintf(stderr, "radion_scene_bench: empty serialized scene\n");
        return 1;
    }

    // Prime all code paths before collecting data. The fixture and document
    // remain immutable through the timed operations.
    if (!deserializeFixture(serializer, document))
    {
        std::fprintf(stderr, "radion_scene_bench: fixture validation failed\n");
        return 1;
    }
    fixture.update(1.0f / 60.0f);

    Distribution serialize;
    Distribution parse;
    Distribution deserialize;
    Distribution update;

    for (u32 run = 0; run < kRuns; ++run)
    {
        const Clock::time_point serializeBegin = Clock::now();
        const std::string encoded = serializer.toJson(fixture).dump();
        serialize.add(elapsedMilliseconds(serializeBegin, Clock::now()));
        if (encoded.size() != text.size())
        {
            std::fprintf(stderr, "radion_scene_bench: serialized size changed\n");
            return 1;
        }

        const Clock::time_point parseBegin = Clock::now();
        const nlohmann::json parsed = nlohmann::json::parse(text);
        parse.add(elapsedMilliseconds(parseBegin, Clock::now()));

        const Clock::time_point deserializeBegin = Clock::now();
        const bool loaded = deserializeFixture(serializer, parsed);
        deserialize.add(elapsedMilliseconds(deserializeBegin, Clock::now()));
        if (!loaded)
        {
            std::fprintf(stderr, "radion_scene_bench: deserialize validation failed\n");
            return 1;
        }
    }

    for (u32 frame = 0; frame < kUpdateFrames; ++frame)
    {
        // Move a deterministic subset so update exercises transform invalidation
        // as well as the regular object/component passes.
        for (u32 index = frame % 17; index < kObjectCount; index += 17)
            objects[index]->setPosition(
                glm::vec3(static_cast<f32>(index % 32), static_cast<f32>(frame % 60),
                          static_cast<f32>(index / 512)));
        const Clock::time_point updateBegin = Clock::now();
        fixture.update(1.0f / 60.0f);
        update.add(elapsedMilliseconds(updateBegin, Clock::now()));
    }

    std::printf("{\n");
    std::printf("  \"benchmark\": \"radion_scene_bench\",\n");
    std::printf("  \"fixture\": {\"objects\": %u, \"branching_factor\": %u, \"bytes\": %zu},\n",
                kObjectCount, kBranchingFactor, text.size());
    std::printf("  \"samples\": {\"serialization\": %u, \"parse\": %u, \"deserialize\": %u, \"update_frames\": %u},\n",
                kRuns, kRuns, kRuns, kUpdateFrames);
    std::printf("  \"milliseconds\": {\n");
    printDistribution("serialize", serialize);
    printDistribution("parse", parse);
    printDistribution("deserialize", deserialize);
    printDistribution("update_per_frame", update, false);
    std::printf("  }\n");
    std::printf("}\n");
    return 0;
}
