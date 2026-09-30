#include "PCH.h"

#include "Animation.h"
#include "Camera.h"
#include "CharacterController.h"
#include "Collider.h"
#include "CollisionWorld.h"
#include "GameObject.h"
#include "Light.h"
#include "MeshRenderer.h"
#include "Scene.h"
#include "SceneSerializer.h"
#include "dynamics/HingeJoint.h"
#include "dynamics/RigidBody.h"
#include "ScriptCache.h"
#include "ZenBehaviour.h"

#include "zen/vm.h"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <string>

using namespace Radion;

namespace
{

int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (condition)
        return;
    std::fprintf(stderr, "ZenBehaviourTests:%d: failed: %s\n", line, expression);
    ++gFailures;
}

#define CHECK(expression) check((expression), #expression, __LINE__)

void testScriptRotatesObjectOnPlay()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Spinner");
    CHECK(object != nullptr);

    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();
    CHECK(behaviour != nullptr);

    const char* script =
        "class Rotate:\n"
        "    def __init__(self):\n"
        "        self.node.set_name(\"Started by node\")\n"
        "    def on_update(self, dt):\n"
        "        self.node.yaw(90.0 * dt)\n";

    CHECK(behaviour->loadSource(script));
    CHECK(!behaviour->hasError());

    const Math::quat startRotation = object->rotation();

    scene.setRunningInEditor(false);
    for (int i = 0; i < 10; ++i)
        scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Started by node");
    CHECK(object->rotation() != startRotation);
}

// The shipped 3D sample: ScriptComponent base, no __init__, self.node before on_start.
void testMoveScriptUsesScriptComponentContract()
{
    const std::filesystem::path path =
        std::filesystem::path(RADION_TEST_ASSET_DIR) / "scripts" / "move.py";

    Scene scene;
    GameObject* object = scene.createGameObject("MoveRotate");
    CHECK(object != nullptr);
    if (!object)
        return;

    object->setPosition(Math::vec3(4.0f, 2.0f, -1.0f));
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();
    CHECK(behaviour != nullptr);
    if (!behaviour)
        return;

    CHECK(behaviour->loadFile(path.string()));

    const Math::quat startRotation = object->rotation();
    scene.setRunningInEditor(false);
    scene.update(0.5f);

    const Math::vec3 position = object->position();
    const f32 dx = position.x - 4.0f;
    const f32 dz = position.z + 1.0f;
    CHECK(!behaviour->hasError());
    CHECK(std::abs(position.y - 2.0f) < 0.001f);
    CHECK(std::abs(dx * dx + dz * dz - 9.0f) < 0.001f);
    CHECK(object->rotation() != startRotation);
}

void testScriptDoesNotRunInEditorMode()
{
    Scene scene;
    GameObject* object = scene.createGameObject("EditorSpinner");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class Rotate:\n"
        "    def on_update(self, dt):\n"
        "        self.owner.yaw(90.0 * dt)\n";
    CHECK(behaviour->loadSource(script));

    const Math::quat startRotation = object->rotation();
    scene.setRunningInEditor(true);
    for (int i = 0; i < 10; ++i)
        scene.update(1.0f / 60.0f);

    CHECK(object->rotation() == startRotation);
    CHECK(!behaviour->hasError());
}

void testClassWithOnlyOneHookLoadsFine()
{
    Scene scene;
    GameObject* object = scene.createGameObject("StartOnly");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class StartOnly:\n"
        "    def on_start(self):\n"
        "        self.owner.set_name(\"Started\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Started");
}

void testScriptWithNoBehaviourClassFailsToLoad()
{
    Scene scene;
    GameObject* object = scene.createGameObject("NoBehaviour");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    CHECK(!behaviour->loadSource("value = 1\n"));
    CHECK(behaviour->hasError());
    CHECK(!behaviour->lastError().empty());
}

void testScriptErrorIsCapturedNotThrown()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Broken");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class Broken:\n"
        "    def on_update(self, dt):\n"
        "        undefined_function()\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);
    CHECK(behaviour->hasError());
    CHECK(!behaviour->lastError().empty());

    scene.update(1.0f / 60.0f);
    CHECK(behaviour->hasError());
}

void testGameObjectTransformAndVec3Arithmetic()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Mover");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class Mover:\n"
        "    def on_start(self):\n"
        "        self.owner.set_position(Vec3(1.0, 2.0, 3.0))\n"
        "        a = Vec3(1.0, 0.0, 1.0)\n"
        "        b = Vec3(0.0, 1.0, 1.0)\n"
        "        c = (a + b) * 2.0\n"
        "        self.owner.set_scale(c)\n"
        "        p = self.owner.get_position()\n"
        "        self.owner.set_rotation(Vec3(0.0, p.x + p.y + p.z, 0.0))\n";

    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->position() == Math::vec3(1.0f, 2.0f, 3.0f));
    CHECK(object->scale() == Math::vec3(2.0f, 2.0f, 4.0f));

    const Math::vec3 rotationDegrees = Math::degrees(Math::eulerAngles(object->rotation()));
    CHECK(std::abs(rotationDegrees.y - 6.0f) < 0.01f);
}

void testSceneFindAndCreateBindings()
{
    Scene scene;
    GameObject* target = scene.createGameObject("Target");
    GameObject* controller = scene.createGameObject("Controller");
    ZenBehaviour* behaviour = controller->addComponent<ZenBehaviour>();

    const char* script =
        "class Finder:\n"
        "    def on_start(self):\n"
        "        found = self.scene.find(\"Target\")\n"
        "        found.set_name(\"Found\")\n"
        "        found.set_active(False)\n"
        "        created = self.scene.create(\"Spawned\")\n"
        "        created.set_position(Vec3(4.0, 5.0, 6.0))\n";

    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(target->name() == "Found");
    CHECK(!target->active());

    GameObject* spawned = scene.findGameObject("Spawned");
    CHECK(spawned != nullptr);
    if (spawned)
        CHECK(spawned->position() == Math::vec3(4.0f, 5.0f, 6.0f));
}

// Collecting between frames must not free the native Vec3 field names (interned, compared by pointer) or touch real objects through wrappers.
void testCollectionBetweenFramesKeepsBindingsAlive()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Churner");
    GameObject* anchor = scene.createGameObject("Anchor");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class Churner:\n"
        "    def on_update(self, dt):\n"
        "        p = self.owner.get_position()\n"
        "        q = (p + Vec3(1.0, 2.0, 3.0)) * 1.0\n"
        "        self.owner.set_position(q)\n"
        "        other = self.scene.find(\"Anchor\")\n"
        "        other.set_position(Vec3(q.x, q.y, q.z))\n";

    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    const int frames = 32;
    for (int i = 0; i < frames; ++i)
    {
        scene.update(1.0f / 60.0f);
        ScriptCache::getSingleton().vm().collect();
    }

    CHECK(!behaviour->hasError());
    CHECK(object->position() == Math::vec3(32.0f, 64.0f, 96.0f));
    CHECK(anchor->position() == object->position());
    CHECK(anchor->name() == "Anchor");
}

// Many objects share one script: one compile, yet each keeps its own state.
void testSharedScriptCompilesOnceAndKeepsPerInstanceState()
{
    Scene scene;
    GameObject* a = scene.createGameObject("A");
    GameObject* b = scene.createGameObject("B");
    GameObject* c = scene.createGameObject("C");

    const char* script =
        "class SharedRotate:\n"
        "    def on_start(self):\n"
        "        name = self.owner.get_name()\n"
        "        if name == \"A\":\n"
        "            self.speed = 30.0\n"
        "        elif name == \"B\":\n"
        "            self.speed = 60.0\n"
        "        else:\n"
        "            self.speed = 90.0\n"
        "    def on_update(self, dt):\n"
        "        self.owner.yaw(self.speed * dt)\n";

    ZenBehaviour* behaviourA = a->addComponent<ZenBehaviour>();
    ZenBehaviour* behaviourB = b->addComponent<ZenBehaviour>();
    ZenBehaviour* behaviourC = c->addComponent<ZenBehaviour>();

    const int compilesBefore = ScriptCache::getSingleton().compileCount();

    CHECK(behaviourA->loadSource(script));
    CHECK(behaviourB->loadSource(script));
    CHECK(behaviourC->loadSource(script));

    CHECK(ScriptCache::getSingleton().compileCount() - compilesBefore == 1);

    scene.setRunningInEditor(false);
    const int frames = 10;
    const f32 dt = 1.0f / 60.0f;
    for (int i = 0; i < frames; ++i)
        scene.update(dt);

    CHECK(!behaviourA->hasError());
    CHECK(!behaviourB->hasError());
    CHECK(!behaviourC->hasError());

    const f32 yawA = Math::degrees(Math::eulerAngles(a->rotation())).y;
    const f32 yawB = Math::degrees(Math::eulerAngles(b->rotation())).y;
    const f32 yawC = Math::degrees(Math::eulerAngles(c->rotation())).y;

    CHECK(std::abs(yawA - 30.0f * dt * frames) < 0.05f);
    CHECK(std::abs(yawB - 60.0f * dt * frames) < 0.05f);
    CHECK(std::abs(yawC - 90.0f * dt * frames) < 0.05f);
}

// ScriptCache removal is indexed swap-pop; a burst of scripted objects leaves no stale GC roots.
void testManyScriptedObjectsUseIndependentRootsAndReleaseThem()
{
    ScriptCache& cache = ScriptCache::getSingleton();
    const usize rootsBefore = cache.protectedInstanceCount();

    {
        Scene scene;
        constexpr int objectCount = 512;
        const char* script =
            "class BurstObject:\n"
            "    def on_start(self):\n"
            "        self.ticks = 0\n"
            "    def on_update(self, dt):\n"
            "        self.ticks = self.ticks + 1\n";

        const int compilesBefore = cache.compileCount();
        for (int i = 0; i < objectCount; ++i)
        {
            GameObject* object = scene.createGameObject("Burst");
            ZenBehaviour* behaviour = object ? object->addComponent<ZenBehaviour>() : nullptr;
            CHECK(behaviour != nullptr);
            if (behaviour)
                CHECK(behaviour->loadSource(script));
        }
        CHECK(cache.compileCount() - compilesBefore == 1);

        scene.setRunningInEditor(false);
        scene.update(1.0f / 60.0f);
        CHECK(cache.protectedInstanceCount() == rootsBefore + objectCount);
        scene.update(1.0f / 60.0f);
    }

    CHECK(cache.protectedInstanceCount() == rootsBefore);
}

static void writeReloadScript(const std::filesystem::path& path, f32 speed)
{
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << "class ReloadRotate:\n";
    out << "    def on_update(self, dt):\n";
    out << "        self.owner.yaw(" << speed << " * dt)\n";
}

void testReloadPropagatesToAllSharingComponents()
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "radion_zen_behaviour_reload_test.py";
    writeReloadScript(path, 30.0f);

    Scene scene;
    GameObject* first = scene.createGameObject("First");
    GameObject* second = scene.createGameObject("Second");
    ZenBehaviour* behaviourFirst = first->addComponent<ZenBehaviour>();
    ZenBehaviour* behaviourSecond = second->addComponent<ZenBehaviour>();

    CHECK(behaviourFirst->loadFile(path.string()));
    CHECK(behaviourSecond->loadFile(path.string()));

    scene.setRunningInEditor(false);
    const f32 dt = 1.0f / 60.0f;
    scene.update(dt);

    const f32 yawFirstBefore = Math::degrees(Math::eulerAngles(first->rotation())).y;
    const f32 yawSecondBefore = Math::degrees(Math::eulerAngles(second->rotation())).y;
    CHECK(std::abs(yawFirstBefore - 30.0f * dt) < 0.01f);
    CHECK(std::abs(yawSecondBefore - 30.0f * dt) < 0.01f);

    writeReloadScript(path, 300.0f);
    CHECK(behaviourFirst->reload());

    scene.update(dt);

    const f32 yawFirstAfter = Math::degrees(Math::eulerAngles(first->rotation())).y;
    const f32 yawSecondAfter = Math::degrees(Math::eulerAngles(second->rotation())).y;
    CHECK(std::abs((yawFirstAfter - yawFirstBefore) - 300.0f * dt) < 0.05f);
    CHECK(std::abs((yawSecondAfter - yawSecondBefore) - 300.0f * dt) < 0.05f);

    std::error_code removeError;
    std::filesystem::remove(path, removeError);
}

// The editor's Play snapshot is a scene document; without serialization Stop restores scripted objects without their script.
void testZenBehaviourSurvivesSerializerRoundTrip()
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "radion_zen_behaviour_roundtrip.py";
    writeReloadScript(path, 45.0f);

    Scene scene;
    GameObject* object = scene.createGameObject("Scripted");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();
    CHECK(behaviour->loadFile(path.string()));
    // Editor mode for the flush: the script must not turn the object before it is serialized.
    scene.setRunningInEditor(true);
    scene.update(1.0f / 60.0f);

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene);

    Scene reloaded;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, reloaded, result));
    CHECK(result.success());

    GameObject* reObject = reloaded.findGameObject(object->id());
    CHECK(reObject != nullptr);
    ZenBehaviour* reBehaviour = reObject ? reObject->findComponent<ZenBehaviour>() : nullptr;
    CHECK(reBehaviour != nullptr);
    if (reBehaviour)
    {
        CHECK(reBehaviour->scriptPath() == path.string());
        CHECK(!reBehaviour->hasError());

        reloaded.setRunningInEditor(false);
        const f32 dt = 1.0f / 60.0f;
        reloaded.update(dt);
        CHECK(!reBehaviour->hasError());
        CHECK(std::abs(Math::degrees(Math::eulerAngles(reObject->rotation())).y - 45.0f * dt) < 0.01f);
    }

    std::error_code removeError;
    std::filesystem::remove(path, removeError);
}

// A missing or non-compiling script must not sink the scene load: component and path come back, with the error for the inspector.
void testMissingScriptFileLoadsAsWarningNotError()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Orphan");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();
    const std::string missing =
        (std::filesystem::temp_directory_path() / "radion_zen_no_such_script.py").string();
    CHECK(!behaviour->loadFile(missing));
    scene.update(1.0f / 60.0f);

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene);

    Scene reloaded;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, reloaded, result));
    CHECK(result.success());

    GameObject* reObject = reloaded.findGameObject(object->id());
    ZenBehaviour* reBehaviour = reObject ? reObject->findComponent<ZenBehaviour>() : nullptr;
    CHECK(reBehaviour != nullptr);
    if (reBehaviour)
    {
        CHECK(reBehaviour->scriptPath() == missing);
        CHECK(reBehaviour->hasError());
    }
}

void testDeclaredPropertyScan()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Declarer");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "SPEED = 120.0\n"
        "class Declarer:\n"
        "    def __init__(self):\n"
        "        self.speed = SPEED\n"
        "        self.lives = 3\n"
        "        self.label = \"hello\"\n"
        "        self.enabled = True\n"
        "        self._hidden = 1.0\n"
        "        self.computed = self.lives * 2\n"
        "    def on_update(self, dt):\n"
        "        step = 1.0\n";

    CHECK(behaviour->loadSource(script));
    CHECK(behaviour->declaredPropertyCount() == 4);

    const ScriptProperty* speed = behaviour->declaredProperty("speed");
    CHECK(speed != nullptr);
    if (speed)
    {
        CHECK(speed->kind == ScriptProperty::Kind::Number);
        CHECK(!speed->integer);
        CHECK(std::abs(speed->number - 120.0) < 1e-9);
    }

    const ScriptProperty* lives = behaviour->declaredProperty("lives");
    CHECK(lives != nullptr);
    if (lives)
    {
        CHECK(lives->kind == ScriptProperty::Kind::Number);
        CHECK(lives->integer);
        CHECK(lives->number == 3.0);
    }

    const ScriptProperty* label = behaviour->declaredProperty("label");
    CHECK(label != nullptr);
    if (label)
    {
        CHECK(label->kind == ScriptProperty::Kind::String);
        CHECK(label->text == "hello");
    }

    const ScriptProperty* enabled = behaviour->declaredProperty("enabled");
    CHECK(enabled != nullptr);
    if (enabled)
    {
        CHECK(enabled->kind == ScriptProperty::Kind::Bool);
        CHECK(enabled->flag);
    }

    CHECK(behaviour->declaredProperty("_hidden") == nullptr);
    CHECK(behaviour->declaredProperty("computed") == nullptr);
    CHECK(behaviour->declaredProperty("step") == nullptr);
}

void testClassBodyPropertiesComeFromTheCompiledClass()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Declared");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class Declared:\n"
        "    speed = 90.0\n"
        "    lives = 3\n"
        "    label = \"spin\"\n"
        "    enabled = True\n"
        "    offset = -2.5\n"
        "    _hidden = 1.0\n"
        "    nothing = None\n"
        "\n"
        "    def on_update(self, dt):\n"
        "        self.scratch = dt\n"
        "        self.owner.yaw(self.speed * dt)\n";

    CHECK(behaviour->loadSource(script));
    CHECK(behaviour->declaredPropertyCount() == 5);

    const ScriptProperty* speed = behaviour->declaredProperty("speed");
    CHECK(speed && speed->kind == ScriptProperty::Kind::Number && !speed->integer);
    if (speed)
        CHECK(std::abs(speed->number - 90.0) < 1e-9);

    const ScriptProperty* lives = behaviour->declaredProperty("lives");
    CHECK(lives && lives->kind == ScriptProperty::Kind::Number && lives->integer);
    if (lives)
        CHECK(lives->number == 3.0);

    const ScriptProperty* label = behaviour->declaredProperty("label");
    CHECK(label && label->kind == ScriptProperty::Kind::String && label->text == "spin");

    const ScriptProperty* enabled = behaviour->declaredProperty("enabled");
    CHECK(enabled && enabled->kind == ScriptProperty::Kind::Bool && enabled->flag);

    const ScriptProperty* offset = behaviour->declaredProperty("offset");
    CHECK(offset && offset->kind == ScriptProperty::Kind::Number);
    if (offset)
        CHECK(std::abs(offset->number + 2.5) < 1e-9);

    CHECK(behaviour->declaredProperty("_hidden") == nullptr);
    CHECK(behaviour->declaredProperty("nothing") == nullptr);
    CHECK(behaviour->declaredProperty("scratch") == nullptr);

    behaviour->setNumberOverride("speed", 180.0, false);
    scene.setRunningInEditor(false);
    const f32 dt = 1.0f / 60.0f;
    scene.update(dt);
    CHECK(!behaviour->hasError());
    CHECK(std::abs(Math::degrees(Math::eulerAngles(object->rotation())).y - 180.0f * dt) < 0.01f);

    // The bindings' own fields are added on bind and must never become properties.
    CHECK(behaviour->declaredProperty("owner") == nullptr);
    CHECK(behaviour->declaredProperty("scene") == nullptr);
}

void testClassBodyAndInitPropertiesCoexist()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Both");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class Both:\n"
        "    declared = 5.0\n"
        "    shared = 1.0\n"
        "\n"
        "    def __init__(self):\n"
        "        self.from_init = \"ctor\"\n"
        "        self.shared = 2.0\n"
        "\n"
        "    def on_update(self, dt):\n"
        "        self.owner.yaw(self.declared * dt)\n";

    CHECK(behaviour->loadSource(script));
    CHECK(behaviour->declaredPropertyCount() == 3);

    const ScriptProperty* declared = behaviour->declaredProperty("declared");
    CHECK(declared && declared->number == 5.0);

    const ScriptProperty* fromInit = behaviour->declaredProperty("from_init");
    CHECK(fromInit && fromInit->kind == ScriptProperty::Kind::String &&
          fromInit->text == "ctor");

    const ScriptProperty* shared = behaviour->declaredProperty("shared");
    CHECK(shared && shared->number == 1.0);
}

void testInitRunsAndOverrideWinsOverIt()
{
    Scene scene;
    GameObject* plain = scene.createGameObject("Plain");
    GameObject* tuned = scene.createGameObject("Tuned");

    const char* script =
        "class Spin:\n"
        "    def __init__(self):\n"
        "        self.speed = 60.0\n"
        "    def on_update(self, dt):\n"
        "        self.owner.yaw(self.speed * dt)\n";

    ZenBehaviour* plainBehaviour = plain->addComponent<ZenBehaviour>();
    ZenBehaviour* tunedBehaviour = tuned->addComponent<ZenBehaviour>();
    CHECK(plainBehaviour->loadSource(script));
    CHECK(tunedBehaviour->loadSource(script));

    tunedBehaviour->setNumberOverride("speed", 180.0, false);
    CHECK(tunedBehaviour->overrideCount() == 1);
    CHECK(plainBehaviour->overrideCount() == 0);

    scene.setRunningInEditor(false);
    const f32 dt = 1.0f / 60.0f;
    scene.update(dt);

    CHECK(!plainBehaviour->hasError());
    CHECK(!tunedBehaviour->hasError());

    const f32 plainYaw = Math::degrees(Math::eulerAngles(plain->rotation())).y;
    const f32 tunedYaw = Math::degrees(Math::eulerAngles(tuned->rotation())).y;
    CHECK(std::abs(plainYaw - 60.0f * dt) < 0.01f);
    CHECK(std::abs(tunedYaw - 180.0f * dt) < 0.01f);
}

void testClearOverrideRestoresTheDeclaredDefault()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Reverter");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class Revert:\n"
        "    def __init__(self):\n"
        "        self.speed = 60.0\n"
        "    def on_update(self, dt):\n"
        "        self.owner.yaw(self.speed * dt)\n";
    CHECK(behaviour->loadSource(script));
    behaviour->setNumberOverride("speed", 600.0, false);

    scene.setRunningInEditor(false);
    const f32 dt = 1.0f / 60.0f;
    scene.update(dt);
    const f32 overriddenStep = Math::degrees(Math::eulerAngles(object->rotation())).y;
    CHECK(std::abs(overriddenStep - 600.0f * dt) < 0.05f);

    behaviour->clearOverride("speed");
    CHECK(behaviour->overrideCount() == 0);

    scene.update(dt);
    const f32 after = Math::degrees(Math::eulerAngles(object->rotation())).y;
    CHECK(std::abs((after - overriddenStep) - 60.0f * dt) < 0.05f);
    CHECK(!behaviour->hasError());
}

// Overrides go through the scene file; the defaults deliberately do not.
void testOverridesSurviveSerializerRoundTrip()
{
    // From a file: the scene stores the script's path, which loadSource() lacks.
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "radion_zen_behaviour_properties.py";
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out << "class Stored:\n";
        out << "    def __init__(self):\n";
        out << "        self.speed = 1.0\n";
        out << "        self.lives = 1\n";
        out << "        self.label = \"a\"\n";
        out << "        self.enabled = False\n";
        out << "    def on_update(self, dt):\n";
        out << "        self.owner.yaw(self.speed * dt)\n";
    }

    Scene scene;
    GameObject* object = scene.createGameObject("Stored");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();
    CHECK(behaviour->loadFile(path.string()));

    behaviour->setNumberOverride("speed", 45.0, false);
    behaviour->setNumberOverride("lives", 7.0, true);
    behaviour->setStringOverride("label", "z");
    behaviour->setBoolOverride("enabled", true);

    scene.setRunningInEditor(true);
    scene.update(1.0f / 60.0f);

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene);

    Scene reloaded;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, reloaded, result));
    CHECK(result.success());

    GameObject* reObject = reloaded.findGameObject(object->id());
    ZenBehaviour* reBehaviour = reObject ? reObject->findComponent<ZenBehaviour>() : nullptr;
    CHECK(reBehaviour != nullptr);
    if (!reBehaviour)
        return;

    CHECK(reBehaviour->overrideCount() == 4);

    const ScriptProperty* speed = reBehaviour->findOverride("speed");
    CHECK(speed && speed->kind == ScriptProperty::Kind::Number && !speed->integer);
    if (speed)
        CHECK(std::abs(speed->number - 45.0) < 1e-9);

    // The int stays an int: "self.lives == 7" would break against a restored 7.0.
    const ScriptProperty* lives = reBehaviour->findOverride("lives");
    CHECK(lives && lives->kind == ScriptProperty::Kind::Number && lives->integer);
    if (lives)
        CHECK(lives->number == 7.0);

    const ScriptProperty* label = reBehaviour->findOverride("label");
    CHECK(label && label->kind == ScriptProperty::Kind::String && label->text == "z");

    const ScriptProperty* enabled = reBehaviour->findOverride("enabled");
    CHECK(enabled && enabled->kind == ScriptProperty::Kind::Bool && enabled->flag);

    reloaded.setRunningInEditor(false);
    const f32 dt = 1.0f / 60.0f;
    reloaded.update(dt);
    CHECK(!reBehaviour->hasError());
    CHECK(std::abs(Math::degrees(Math::eulerAngles(reObject->rotation())).y - 45.0f * dt) < 0.01f);

    std::error_code removeError;
    std::filesystem::remove(path, removeError);
}

// on_collision(self, other): called once per contact from CollisionWorld::step(), with the far collider's GameObject as other.
void testOnCollisionSeesOtherObjectName()
{
    Scene scene;
    GameObject* watcher = scene.createGameObject("Watcher");
    GameObject* bumper = scene.createGameObject("Bumper");
    bumper->setPosition(Math::vec3(0.5f, 0.0f, 0.0f));

    Collider* watcherCollider = watcher->addComponent<Collider>();
    watcherCollider->setSphere(1.0f);
    watcherCollider->setType(30);
    Collider* bumperCollider = bumper->addComponent<Collider>();
    bumperCollider->setSphere(1.0f);
    bumperCollider->setType(31);

    ZenBehaviour* behaviour = watcher->addComponent<ZenBehaviour>();
    const char* script =
        "class Watcher:\n"
        "    def on_collision(self, other):\n"
        "        self.owner.set_name(other.get_name())\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.collisions().enable(30, 31, CollisionResponse::None);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(watcher->name() == "Bumper");
}

// callCollision() carries `began`; the 1-argument on_collision form still works since zen does not check native-call arity.
void testCallCollisionPassesBeganFlag()
{
    Scene scene;
    GameObject* watcher = scene.createGameObject("Watcher");
    GameObject* other = scene.createGameObject("Other");
    ZenBehaviour* behaviour = watcher->addComponent<ZenBehaviour>();

    const char* script =
        "class BeganWatcher:\n"
        "    def on_collision(self, other, began):\n"
        "        if began:\n"
        "            self.owner.set_name(\"Began\")\n"
        "        else:\n"
        "            self.owner.set_name(\"Ended\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(0.0f);
    CHECK(behaviour->callCollision(other, true));
    CHECK(!behaviour->hasError());
    CHECK(watcher->name() == "Began");

    CHECK(behaviour->callCollision(other, false));
    CHECK(!behaviour->hasError());
    CHECK(watcher->name() == "Ended");
}

void testComponentBaseIsActive()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Lit");
    DirectionalLight* light = object->addComponent<DirectionalLight>();
    CHECK(light != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class ToggleLight:\n"
        "    def on_start(self):\n"
        "        light = self.node.get_component(Light)\n"
        "        if light.is_active():\n"
        "            light.set_active(False)\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(light != nullptr);
    if (light)
        CHECK(!light->active());
}

// get_component(Class) takes the class itself: a matching component resolves to a handle, none to nil.
void testGetComponentByClass()
{
    Scene scene;
    GameObject* lit = scene.createGameObject("Lit");
    lit->addComponent<DirectionalLight>();
    GameObject* bare = scene.createGameObject("Bare");

    ZenBehaviour* behaviour = lit->addComponent<ZenBehaviour>();
    const char* script =
        "class Fetch:\n"
        "    def on_start(self):\n"
        "        found = self.node.get_component(Light)\n"
        "        if found != None:\n"
        "            self.node.set_name(\"HasLight\")\n"
        "        other = self.scene.find(\"Bare\")\n"
        "        missing = other.get_component(Camera)\n"
        "        if missing == None:\n"
        "            other.set_name(\"NoCamera\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(lit->name() == "HasLight");
    CHECK(bare->name() == "NoCamera");
}

void testScriptDrivesRigidBody()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Crate");
    Physics::RigidBody* body = object->addComponent<Physics::RigidBody>();
    CHECK(body != nullptr);
    if (!body)
        return;
    body->setBox(Math::vec3(0.5f));
    body->setMass(4.0f);

    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();
    const char* script =
        "class Push:\n"
        "    def on_start(self):\n"
        "        rb = self.node.get_component<RigidBody>()\n"
        "        if rb == None:\n"
        "            return\n"
        "        self.node.set_name(\"Found\")\n"
        "        rb.set_velocity(Vec3(3.0, 0.0, 0.0))\n"
        "        rb.add_force(Vec3(0.0, 100.0, 0.0))\n"
        "        if rb.get_mass() > 3.9 and rb.get_mass() < 4.1:\n"
        "            self.node.set_name(\"MassOk\")\n"
        "        if rb.is_dynamic():\n"
        "            self.node.set_name(\"Dynamic\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Dynamic");
    // Close to 3, not exactly: the same update integrates a step of damping over the script's velocity.
    CHECK(std::abs(body->velocity().x - 3.0f) < 0.05f);
}

void testScriptCommandsAJointServo()
{
    Scene scene;
    GameObject* base = scene.createGameObject("Base");
    Physics::RigidBody* baseBody = base->addComponent<Physics::RigidBody>();
    // Small enough not to touch the arm a metre away: boxes exactly a metre apart hold the joint still by contact.
    baseBody->setBox(Math::vec3(0.2f));
    baseBody->setBodyType(Physics::BodyType::Static);

    GameObject* arm = scene.createGameObject("Arm");
    arm->setPosition(Math::vec3(1.0f, 0.0f, 0.0f));
    Physics::RigidBody* armBody = arm->addComponent<Physics::RigidBody>();
    armBody->setBox(Math::vec3(0.5f));
    armBody->setMass(2.0f);
    armBody->setInertiaTensor(Physics::Inertia::box(2.0f, Math::vec3(0.5f)));
    Physics::HingeJoint* hinge = arm->addComponent<Physics::HingeJoint>();
    hinge->setConnectedBody(base);
    hinge->setAuthoredAxis(Math::vec3(0.0f, 0.0f, 1.0f));

    ZenBehaviour* behaviour = arm->addComponent<ZenBehaviour>();
    const char* script =
        "class Drive:\n"
        "    def on_start(self):\n"
        "        j = self.node.get_component<Joint>()\n"
        "        if j == None:\n"
        "            return\n"
        "        if j.get_kind() == \"Hinge\":\n"
        "            self.node.set_name(\"KnowsKind\")\n"
        "        j.set_limits(-1.0, 1.0)\n"
        "        j.set_servo(0.5, 500.0, 2.0)\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);
    CHECK(!behaviour->hasError());
    CHECK(arm->name() == "KnowsKind");
    CHECK(hinge->servoEnabled());
    CHECK(std::abs(hinge->servoTargetAngle() - 0.5f) < 1e-4f);

    for (u32 i = 0; i < 400; ++i)
        scene.update(1.0f / 120.0f);
    CHECK(std::abs(hinge->currentAngle() - 0.5f) < 0.05f);
}

// Two fetches must return the same script instance (ScriptCache::instanceFor()); Zen instances compare by identity, so == is a real same-object check.
void testComponentHandleIsCached()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Lit");
    object->addComponent<DirectionalLight>();
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class Compare:\n"
        "    def on_start(self):\n"
        "        first = self.node.get_component(Light)\n"
        "        second = self.node.get_component(Light)\n"
        "        if first == second:\n"
        "            self.node.set_name(\"SameInstance\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "SameInstance");
}

// Handles are cached by component address in a persistent class: without Scene::componentRemoved() dropping the entry, a new component at that address inherits a stale handle.
void testHandleForgottenWhenOwnerDies()
{
    Scene scene;
    GameObject* holder = scene.createGameObject("Holder");
    GameObject* target = scene.createGameObject("Target");
    DirectionalLight* light = target->addComponent<DirectionalLight>();
    CHECK(light != nullptr);

    ZenBehaviour* behaviour = holder->addComponent<ZenBehaviour>();
    const char* script =
        "class Grab:\n"
        "    def on_start(self):\n"
        "        other = self.scene.find(\"Target\")\n"
        "        self.light = other.get_component(Light)\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);
    CHECK(!behaviour->hasError());
    CHECK(ScriptCache::getSingleton().hasCachedInstance(light));

    scene.destroy(target);
    scene.update(1.0f / 60.0f);

    CHECK(!ScriptCache::getSingleton().hasCachedInstance(light));
}

// A stored component handle must survive a frame and a full collection; the Light class is persistent, but the script instance must survive too.
void testHandleSurvivesBetweenFrames()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Lit");
    DirectionalLight* light = object->addComponent<DirectionalLight>();
    CHECK(light != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class HoldLight:\n"
        "    def on_start(self):\n"
        "        self.light = self.node.get_component(Light)\n"
        "    def on_update(self, dt):\n"
        "        self.light.set_active(False)\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);
    ScriptCache::getSingleton().vm().collect();
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(light != nullptr);
    if (light)
        CHECK(!light->active());
}

void testMeshRendererVisibleInReflections()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Mirror");
    MeshRenderer* renderer = object->addComponent<MeshRenderer>();
    CHECK(renderer != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class HideFromReflections:\n"
        "    def on_start(self):\n"
        "        r = self.node.get_component(MeshRenderer)\n"
        "        r.set_visible_in_reflections(False)\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(renderer != nullptr);
    if (renderer)
        CHECK(renderer->visibleInReflections() == false);
}

void testMeshRendererSubmeshVisibility()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Chunked");
    MeshRenderer* renderer = object->addComponent<MeshRenderer>();
    CHECK(renderer != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class HideSubmesh:\n"
        "    def on_start(self):\n"
        "        r = self.node.get_component(MeshRenderer)\n"
        "        r.set_submesh_visible(2, False)\n"
        "        if r.is_submesh_visible(2) == False:\n"
        "            self.node.set_name(\"SubmeshHidden\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "SubmeshHidden");
    CHECK(renderer != nullptr);
    if (renderer)
        CHECK(renderer->submeshVisible(2) == false);
}

// A count must reach the script as an integer: array indexing refuses a float, while int and float compare equal.
void testMeshRendererCountsAreIntegers()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Counted");
    MeshRenderer* renderer = object->addComponent<MeshRenderer>();
    CHECK(renderer != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class CountSubmeshes:\n"
        "    def on_start(self):\n"
        "        r = self.node.get_component(MeshRenderer)\n"
        "        r.set_submesh_visible(1, False)\n"
        "        names = [\"none\", \"one\", \"two\"]\n"
        "        self.node.set_name(names[r.get_hidden_submesh_count()])\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "one");
}

// Without get_submesh_count() a script could hide submesh 7 of 3; no mesh means zero submeshes.
void testMeshRendererSubmeshCount()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Unmeshed");
    MeshRenderer* renderer = object->addComponent<MeshRenderer>();
    CHECK(renderer != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class CountSubmesh:\n"
        "    def on_start(self):\n"
        "        r = self.node.get_component(MeshRenderer)\n"
        "        self.node.set_name(str(r.get_submesh_count()))\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "0");
    CHECK(renderer != nullptr);
    if (renderer)
        CHECK(renderer->submeshCount() == 0);
}

// Every set_* shape in one script: an unbound name or wrong arity raises in the VM, so reaching the last line proves all eight registered. has_mesh() is not asserted (createMesh uploads; no GL context here). set_mesh_file() on a missing file is False with no error either way.
void testMeshRendererSetsPrimitiveMesh()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Shaped");
    MeshRenderer* renderer = object->addComponent<MeshRenderer>();
    CHECK(renderer != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class SetShapes:\n"
        "    def on_start(self):\n"
        "        r = self.node.get_component(MeshRenderer)\n"
        "        r.set_box(1.0, 2.0, 3.0)\n"
        "        r.set_sphere(0.5)\n"
        "        r.set_plane(2.0, 2.0)\n"
        "        r.set_cylinder(0.5, 1.0)\n"
        "        r.set_cone(0.5, 1.0)\n"
        "        r.set_capsule(0.4, 1.0)\n"
        "        r.set_torus(1.0, 0.25)\n"
        "        if not r.set_mesh_file(\"no_such_mesh.rmesh\"):\n"
        "            self.node.set_name(\"Bound\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Bound");
}

// get_component<Light>() reaches the same native with the same args[0] as get_component(Light): generic values come before explicit arguments.
void testGenericSpellingReachesTheSameNative()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Generic");
    object->addComponent<DirectionalLight>();
    object->addComponent<MeshRenderer>();
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class GenericCalls:\n"
        "    def on_start(self):\n"
        "        plain = self.node.get_component(Light)\n"
        "        generic = self.node.get_component<Light>()\n"
        "        if plain == generic and self.node.has_component<MeshRenderer>():\n"
        "            self.node.set_name(\"Same\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Same");
}

void testCharacterControllerTuning()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Tuned");
    CharacterController* controller = object->addComponent<CharacterController>();
    CHECK(controller != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class TuneController:\n"
        "    def on_start(self):\n"
        "        c = self.node.get_component(CharacterController)\n"
        "        c.set_radius(0.75)\n"
        "        c.set_gravity(-9.8)\n"
        "        c.set_max_iterations(4)\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(controller != nullptr);
    if (controller)
    {
        CHECK(std::abs(controller->radius() - 0.75f) < 0.0001f);
        CHECK(std::abs(controller->gravity() - (-9.8f)) < 0.0001f);
        CHECK(controller->maxIterations() == 4);
    }
}

// With no octree attached move() translates the owner directly (CharacterController.cpp:228-233); check the result and the actual position.
void testCharacterControllerMoveReturnsResult()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Mover");
    CharacterController* controller = object->addComponent<CharacterController>();
    CHECK(controller != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class MoveController:\n"
        "    def on_start(self):\n"
        "        c = self.node.get_component(CharacterController)\n"
        "        result = c.move(Vec3(1.0, 0.0, 0.0))\n"
        "        if result.collided == False and result.displacement.x == 1.0:\n"
        "            self.node.set_name(\"Moved\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Moved");
    CHECK(std::abs(object->position().x - 1.0f) < 0.0001f);
}

void testCharacterControllerMoveInputRoundTrip()
{
    Scene scene;
    GameObject* object = scene.createGameObject("InputRoundTrip");
    CharacterController* controller = object->addComponent<CharacterController>();
    CHECK(controller != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class InputController:\n"
        "    def on_start(self):\n"
        "        c = self.node.get_component(CharacterController)\n"
        "        c.set_move_input(Vec3(2.0, 0.0, 3.0))\n"
        "        v = c.get_move_input()\n"
        "        if v.x == 2.0 and v.z == 3.0:\n"
        "            self.node.set_name(\"InputRoundTripOk\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "InputRoundTripOk");
    CHECK(controller != nullptr);
    if (controller)
    {
        CHECK(std::abs(controller->moveInput().x - 2.0f) < 0.0001f);
        CHECK(std::abs(controller->moveInput().z - 3.0f) < 0.0001f);
    }
}

// Handle classes declare zero fields, so reading x/y/z off a handle passed as a Vec3 would dereference null: the call is refused and the transform left alone.
void testVec3ArgumentTypeIsChecked()
{
    Scene scene;
    GameObject* object = scene.createGameObject("BadArgument");
    object->setPosition(Math::vec3(5.0f, 6.0f, 7.0f));
    object->addComponent<CharacterController>();
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class BadArgument:\n"
        "    def on_start(self):\n"
        "        self.node.set_position(self.node)\n"
        "        c = self.node.get_component(CharacterController)\n"
        "        c.teleport(c)\n"
        "        if c.move(c) == None:\n"
        "            self.node.set_name(\"Survived\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Survived");
    CHECK(object->position() == Math::vec3(5.0f, 6.0f, 7.0f));
}

// Same fixture as SceneTests.cpp's testAnimatedPlayers: two-bone skeleton, one two-second "Move" clip translating bone 0 along X.
AnimationSetHandle makeMoveAnimationSet()
{
    Skeleton skeleton;
    skeleton.addBone("root", -1, Math::mat4(1.0f), Math::mat4(1.0f));
    skeleton.addBone("hand", 0, Math::mat4(1.0f), Math::mat4(1.0f));
    skeleton.finalize();

    AnimationClip clip;
    clip.setName("Move");
    clip.setDuration(2.0f);
    BoneTrack track;
    track.bone = 0;
    track.times = {0.0f, 2.0f};
    track.positions = {Math::vec3(0.0f), Math::vec3(2.0f, 0.0f, 0.0f)};
    track.rotations = {Math::quat(1, 0, 0, 0), Math::quat(1, 0, 0, 0)};
    track.scales = {Math::vec3(1.0f), Math::vec3(1.0f)};
    clip.tracks().push_back(track);

    const std::vector<AnimationClip> clips = {clip};
    return Animations().create(skeleton, clips);
}

void testAnimatorPlaysClipFromScript()
{
    const AnimationSetHandle animationSet = makeMoveAnimationSet();
    Scene scene;
    GameObject* object = scene.createGameObject("AnimatedScript");
    Animator* animator = object->addComponent<Animator>();
    CHECK(animator != nullptr);
    if (animator)
        animator->bind(animationSet);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class PlayFromScript:\n"
        "    def on_start(self):\n"
        "        a = self.node.get_component(Animator)\n"
        "        a.play(\"Move\")\n"
        "        a.get_layer(0)\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(animator != nullptr);
    if (animator)
    {
        CHECK(animator->layer(0).isPlaying("Move"));
        CHECK(std::abs(animator->layer(0).duration() - 2.0f) < 0.0001f);
    }

    Animations().destroy(animationSet);
}

// PLAY_LOOP/ONCE/PINGPONG match PlayMode; observed through finished(), true only for Once.
void testAnimatorPlayModeConstants()
{
    const AnimationSetHandle animationSet = makeMoveAnimationSet();
    Scene scene;
    GameObject* object = scene.createGameObject("PlayModeScript");
    Animator* animator = object->addComponent<Animator>();
    CHECK(animator != nullptr);
    if (animator)
        animator->bind(animationSet);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class PlayModeConstants:\n"
        "    def on_start(self):\n"
        "        a = self.node.get_component(Animator)\n"
        "        a.play(\"Move\", PLAY_ONCE)\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(animator != nullptr);
    if (animator)
    {
        // Past the end of a 2s clip Once must report finished: needs mCurrent resolved and mMode read back as Once.
        animator->layer(0).seek(10.0f);
        CHECK(animator->layer(0).finished());
    }

    Animations().destroy(animationSet);
}

// Animator::layer() resizes mLayers, so a layer-0 handle must resolve correctly after a later get_layer() reallocates; caching AnimationLayer* would read freed memory.
void testAnimationLayerHandleSurvivesLayerGrowth()
{
    const AnimationSetHandle animationSet = makeMoveAnimationSet();
    Scene scene;
    GameObject* object = scene.createGameObject("LayerGrowth");
    Animator* animator = object->addComponent<Animator>();
    CHECK(animator != nullptr);
    if (animator)
        animator->bind(animationSet);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class LayerGrowth:\n"
        "    def on_start(self):\n"
        "        a = self.node.get_component(Animator)\n"
        "        l0 = a.get_layer(0)\n"
        "        a.get_layer(3)\n"
        "        l0.play(\"Move\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(animator != nullptr);
    if (animator)
    {
        CHECK(animator->layerCount() == 4);
        CHECK(animator->layer(0).isPlaying("Move"));
    }

    Animations().destroy(animationSet);
}

// Known 2s clip started from C++; both updates use dt=0 so mCurrent is resolved (Animator::update() fills it only after play()) and mTime stays at the seeked value.
void testAnimationLayerTimeAndSeek()
{
    const AnimationSetHandle animationSet = makeMoveAnimationSet();
    Scene scene;
    GameObject* object = scene.createGameObject("LayerSeek");
    Animator* animator = object->addComponent<Animator>();
    CHECK(animator != nullptr);
    if (animator)
    {
        animator->bind(animationSet);
        animator->play("Move", PlayMode::Loop, 0.0f);
    }

    scene.setRunningInEditor(false);
    scene.update(0.0f);

    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();
    const char* script =
        "class LayerSeek:\n"
        "    def on_start(self):\n"
        "        a = self.node.get_component(Animator)\n"
        "        l = a.get_layer(0)\n"
        "        l.seek(3.0)\n"
        "        wrapped = l.get_wrapped_time()\n"
        "        normalized = l.get_normalized_time()\n"
        "        finished = l.is_finished()\n"
        "        if wrapped == 1.0 and normalized == 1.5 and finished == False:\n"
        "            self.node.set_name(\"SeekOk\")\n";
    CHECK(behaviour->loadSource(script));

    scene.update(0.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "SeekOk");
    CHECK(animator != nullptr);
    if (animator)
    {
        CHECK(std::abs(animator->layer(0).time() - 3.0f) < 0.0001f);
        CHECK(std::abs(animator->layer(0).wrappedTime() - 1.0f) < 0.0001f);
        CHECK(std::abs(animator->layer(0).normalizedTime() - 1.5f) < 0.0001f);
        CHECK(!animator->layer(0).finished());
    }

    Animations().destroy(animationSet);
}

void testGameObjectHierarchyFromScript()
{
    Scene scene;
    GameObject* parent = scene.createGameObject("Parent");
    GameObject* childA = scene.createGameObject("ChildA", parent);
    GameObject* childB = scene.createGameObject("ChildB", parent);
    CHECK(parent != nullptr);
    CHECK(childA != nullptr);
    CHECK(childB != nullptr);

    ZenBehaviour* behaviour = parent->addComponent<ZenBehaviour>();
    const char* script =
        "class HierarchyWalk:\n"
        "    def on_start(self):\n"
        "        count = self.node.get_child_count()\n"
        "        first = self.node.get_child(0)\n"
        "        second = self.node.get_child(1)\n"
        "        missing = self.node.get_child(5)\n"
        "        found = self.node.find_child(\"ChildB\")\n"
        "        parent_of_self = self.node.get_parent()\n"
        "        root = self.node.get_root()\n"
        "        root_parent = root.get_parent()\n"
        "        ok = count == 2\n"
        "        if first.get_name() != \"ChildA\":\n"
        "            ok = False\n"
        "        if second.get_name() != \"ChildB\":\n"
        "            ok = False\n"
        "        if missing != None:\n"
        "            ok = False\n"
        "        if found == None:\n"
        "            ok = False\n"
        "        if parent_of_self == None:\n"
        "            ok = False\n"
        "        if root_parent != None:\n"
        "            ok = False\n"
        "        if ok:\n"
        "            self.node.set_name(\"HierarchyOk\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(parent->name() == "HierarchyOk");
    CHECK(childA->parent() == parent);
    CHECK(childB->parent() == parent);
}

// dispose() only raises a flag; Scene::update()'s end-of-frame sweep queues it and flushChanges() deletes it, inside the same update() call after the script's component-update phase. The script reads is_disposed() and re-finds the object right after dispose(); C++ confirms it is gone once update() returns.
void testGameObjectDisposeIsDeferred()
{
    Scene scene;
    GameObject* target = scene.createGameObject("Target");
    GameObject* controller = scene.createGameObject("Controller");
    CHECK(target != nullptr);
    CHECK(controller != nullptr);

    ZenBehaviour* behaviour = controller->addComponent<ZenBehaviour>();
    const char* script =
        "class Disposer:\n"
        "    def on_start(self):\n"
        "        found = self.scene.find(\"Target\")\n"
        "        found.dispose()\n"
        "        still_reachable = self.scene.find(\"Target\") != None\n"
        "        if found.is_disposed() and still_reachable:\n"
        "            self.node.set_name(\"FlagSeenBeforeSweep\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(controller->name() == "FlagSeenBeforeSweep");
    CHECK(scene.findGameObject("Target") == nullptr);
    (void)target;
}

void testGameObjectAddAndRemoveComponent()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Rigged");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class RigCamera:\n"
        "    def on_update(self, dt):\n"
        "        if self.node.has_component(Camera):\n"
        "            self.node.remove_component(Camera)\n"
        "            self.node.set_name(\"Removed\")\n"
        "        else:\n"
        "            cam = self.node.add_component(Camera)\n"
        "            cam.set_aspect(1.5)\n"
        "            self.node.set_name(\"Added\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Added");
    Camera* camera = object->getComponent<Camera>();
    CHECK(camera != nullptr);
    if (camera)
        CHECK(std::abs(camera->aspect() - 1.5f) < 0.0001f);

    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Removed");
    CHECK(object->getComponent<Camera>() == nullptr);
}

void testGameObjectGlobalTransform()
{
    Scene scene;
    GameObject* parent = scene.createGameObject("Parent");
    parent->setPosition(Math::vec3(10.0f, 0.0f, 0.0f));
    GameObject* child = scene.createGameObject("Child", parent);
    child->setPosition(Math::vec3(1.0f, 2.0f, 3.0f));

    ZenBehaviour* behaviour = child->addComponent<ZenBehaviour>();
    const char* script =
        "class ReadTransform:\n"
        "    def on_start(self):\n"
        "        local = self.node.get_position()\n"
        "        world = self.node.get_global_position()\n"
        "        if local.x != world.x:\n"
        "            self.node.set_name(\"TransformsDiffer\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(child->name() == "TransformsDiffer");
    CHECK(child->position() == Math::vec3(1.0f, 2.0f, 3.0f));
    CHECK(child->globalPosition() == Math::vec3(11.0f, 2.0f, 3.0f));
}

// reparent() requires an existing parent (Scene.cpp:375-388), so it cannot run in the frame create() queued the object; it moves the object immediately, so no extra update.
void testSceneCreateWithParentAndReparent()
{
    Scene scene;
    GameObject* parentA = scene.createGameObject("ParentA");
    GameObject* parentB = scene.createGameObject("ParentB");
    GameObject* controller = scene.createGameObject("Controller");
    ZenBehaviour* behaviour = controller->addComponent<ZenBehaviour>();

    const char* script =
        "class SpawnAndMove:\n"
        "    def on_start(self):\n"
        "        self.parent_a = self.scene.find(\"ParentA\")\n"
        "        self.parent_b = self.scene.find(\"ParentB\")\n"
        "        self.child = self.scene.create(\"Child\", self.parent_a)\n"
        "        self.reparented = False\n"
        "    def on_update(self, dt):\n"
        "        if self.reparented == False:\n"
        "            if self.child.get_parent() != None:\n"
        "                moved = self.scene.reparent(self.child, self.parent_b)\n"
        "                self.reparented = True\n"
        "                if moved:\n"
        "                    self.node.set_name(\"ReparentOk\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    GameObject* child = scene.findGameObject("Child");
    CHECK(child != nullptr);
    if (child)
        CHECK(child->parent() == parentA);

    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(controller->name() == "ReparentOk");
    if (child)
        CHECK(child->parent() == parentB);
}

// readGameObject() must check the argument's class, not just a native_data pointer (a Camera handle has one): scene.destroy() refuses it and leaves the real object alone.
void testReadGameObjectRejectsOtherHandles()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Camwielder");
    Camera* camera = object->addComponent<Camera>();
    CHECK(camera != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class BadDestroy:\n"
        "    def on_start(self):\n"
        "        cam = self.node.get_component(Camera)\n"
        "        result = self.scene.destroy(cam)\n"
        "        if result == False:\n"
        "            self.node.set_name(\"RejectedNonGameObject\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "RejectedNonGameObject");
    CHECK(scene.findGameObject(object->id()) == object);
    CHECK(!object->disposed());
    CHECK(object->getComponent<Camera>() == camera);
}

void testCallEventInvokesOnEventHook()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Eventer");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class EventHandler:\n"
        "    def on_event(self, event, value):\n"
        "        self.owner.set_name(event)\n"
        "        self.owner.yaw(value)\n";
    CHECK(behaviour->loadSource(script));

    // createGameObject() only queues the object: owner() does not resolve until a flush, so one no-op update must run before a direct callEvent().
    scene.setRunningInEditor(false);
    scene.update(0.0f);
    CHECK(behaviour->callEvent("Jump", 45.0));
    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Jump");
    const f32 yawAfterJump = Math::degrees(Math::eulerAngles(object->rotation())).y;
    CHECK(std::abs(yawAfterJump - 45.0f) < 0.01f);

    CHECK(behaviour->callEvent("Idle"));
    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Idle");
    const f32 yawAfterIdle = Math::degrees(Math::eulerAngles(object->rotation())).y;
    CHECK(std::abs(yawAfterIdle - yawAfterJump) < 0.01f);
}

void testCallEventReturnsFalseWithoutOnEventHook()
{
    Scene scene;
    GameObject* object = scene.createGameObject("NoEvents");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class NoEventHook:\n"
        "    def on_update(self, dt):\n"
        "        pass\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(0.0f);
    CHECK(!behaviour->callEvent("Anything"));
    CHECK(!behaviour->hasError());
}

void testCallFunctionInvokesNamedMethodAndHasFunctionSeesIt()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Custom");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class Custom:\n"
        "    def take_damage(self, amount):\n"
        "        self.owner.yaw(amount)\n"
        "    def on_update(self, dt):\n"
        "        pass\n";
    CHECK(behaviour->loadSource(script));

    CHECK(behaviour->hasFunction("take_damage"));
    CHECK(behaviour->hasFunction("on_update"));
    CHECK(!behaviour->hasFunction("no_such_function"));

    scene.setRunningInEditor(false);
    scene.update(0.0f);
    CHECK(behaviour->callFunction("take_damage", 30.0));
    CHECK(!behaviour->hasError());
    const f32 yaw = Math::degrees(Math::eulerAngles(object->rotation())).y;
    CHECK(std::abs(yaw - 30.0f) < 0.01f);
}

void testCallFunctionFailsForUnknownName()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Bare");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class Bare:\n"
        "    def on_update(self, dt):\n"
        "        pass\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(0.0f);
    CHECK(!behaviour->callFunction("no_such_function", 1.0));
    CHECK(behaviour->hasError());
    CHECK(!behaviour->lastError().empty());
}

// The mtime is forced forward past the filesystem's resolution: the comparison is under test, not a race with the OS clock. sourceTimestamp() reads the shared ScriptCache entry, so it moves for every component sharing the path.
void testReloadIfChangedDetectsDiskEditAndSourceTimestampTracksIt()
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "radion_zen_behaviour_reload_if_changed_test.py";
    writeReloadScript(path, 30.0f);

    Scene scene;
    GameObject* object = scene.createGameObject("Watched");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();
    CHECK(behaviour->loadFile(path.string()));

    const s64 firstStamp = behaviour->sourceTimestamp();
    CHECK(firstStamp != 0);

    CHECK(!behaviour->reloadIfChanged());
    CHECK(behaviour->sourceTimestamp() == firstStamp);

    writeReloadScript(path, 300.0f);
    std::filesystem::last_write_time(
        path, std::filesystem::last_write_time(path) + std::chrono::seconds(5));

    CHECK(behaviour->reloadIfChanged());
    CHECK(behaviour->sourceTimestamp() != firstStamp);

    scene.setRunningInEditor(false);
    const f32 dt = 1.0f / 60.0f;
    scene.update(dt);
    CHECK(!behaviour->hasError());
    CHECK(std::abs(Math::degrees(Math::eulerAngles(object->rotation())).y - 300.0f * dt) < 0.05f);

    std::error_code removeError;
    std::filesystem::remove(path, removeError);
}

void testReloadIfChangedFalseWithoutFile()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Inline");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();
    CHECK(behaviour->loadSource("class Empty:\n    def on_start(self):\n        pass\n"));

    CHECK(behaviour->sourceTimestamp() == 0);
    CHECK(!behaviour->reloadIfChanged());
}

void testCallAndGlobalRoundTrip()
{
    ScriptVM vm;
    std::string error;

    CHECK(vm.setGlobal("value", 41.0));

    const char* script =
        "def bump():\n"
        "    return value + 1\n";
    CHECK(vm.runString(script, "<test>", error));
    CHECK(vm.hasFunction("bump"));
    CHECK(!vm.hasFunction("no_such_function"));

    ScriptValue result;
    CHECK(vm.call("bump", nullptr, 0, result, error));
    CHECK(result.kind == ScriptValue::Kind::Number);
    CHECK(result.numberValue == 42.0);
}

// A GameObject handle resolves by id on every call, and flushChanges() drops the id before the delete, so a handle held across destruction stops resolving instead of reading freed memory; getters must give the same "empty" value as goGetName()/goGetChildCount().
void testGameObjectHandleSurvivesOwnerDestruction()
{
    Scene scene;
    GameObject* holder = scene.createGameObject("Holder");
    GameObject* target = scene.createGameObject("Target");
    CHECK(holder != nullptr);
    CHECK(target != nullptr);

    ZenBehaviour* behaviour = holder->addComponent<ZenBehaviour>();
    const char* script =
        "class HoldTarget:\n"
        "    def on_start(self):\n"
        "        self.target = self.scene.find(\"Target\")\n"
        "    def on_update(self, dt):\n"
        "        name = self.target.get_name()\n"
        "        self.target.set_name(\"x\")\n"
        "        count = self.target.get_child_count()\n"
        "        if name == \"\" and count == 0:\n"
        "            self.node.set_name(\"Survived\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);
    CHECK(!behaviour->hasError());

    scene.destroy(target);
    scene.update(1.0f / 60.0f);
    CHECK(!behaviour->hasError());

    scene.update(1.0f / 60.0f);
    CHECK(!behaviour->hasError());
    CHECK(holder->name() == "Survived");
}

// Same over a Light handle: Scene::componentRemoved() clears the native pointer via ScriptCache::forgetInstance(); the persistent class survives, so get_intensity() must answer 0 (null-safe default).
void testComponentHandleSurvivesComponentRemoval()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Lit");
    DirectionalLight* light = object->addComponent<DirectionalLight>();
    CHECK(light != nullptr);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class HoldLightHandle:\n"
        "    def on_start(self):\n"
        "        self.light = self.node.get_component(Light)\n"
        "    def on_update(self, dt):\n"
        "        self.light.set_intensity(5.0)\n"
        "        value = self.light.get_intensity()\n"
        "        if value == 0.0:\n"
        "            self.node.set_name(\"Survived\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);
    CHECK(!behaviour->hasError());

    object->removeComponent<DirectionalLight>();
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Survived");
    CHECK(object->getComponent<Light>() == nullptr);
}

// Same over an AnimationLayer handle: it resolves the owner by id and its current Animator (animatorForLayerHandle), so removing the Animator stops it resolving.
void testAnimationLayerHandleSurvivesAnimatorRemoval()
{
    const AnimationSetHandle animationSet = makeMoveAnimationSet();
    Scene scene;
    GameObject* object = scene.createGameObject("Animated");
    Animator* animator = object->addComponent<Animator>();
    CHECK(animator != nullptr);
    if (animator)
        animator->bind(animationSet);
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    const char* script =
        "class HoldLayerHandle:\n"
        "    def on_start(self):\n"
        "        a = self.node.get_component(Animator)\n"
        "        self.layer = a.get_layer(0)\n"
        "    def on_update(self, dt):\n"
        "        self.layer.play(\"Move\")\n"
        "        duration = self.layer.get_duration()\n"
        "        if duration == 0.0:\n"
        "            self.node.set_name(\"Survived\")\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);
    CHECK(!behaviour->hasError());

    object->removeComponent<Animator>();
    scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Survived");
    CHECK(object->getComponent<Animator>() == nullptr);

    Animations().destroy(animationSet);
}

// A handle resolves by id and Scene never reuses an id (Scene::stampId() draws from mNextId): a fresh object at the old address must not be retargeted.
void testGameObjectHandleFollowsIdNotPointer()
{
    Scene scene;
    GameObject* holder = scene.createGameObject("Holder");
    GameObject* target = scene.createGameObject("Target");
    CHECK(holder != nullptr);
    CHECK(target != nullptr);

    ZenBehaviour* behaviour = holder->addComponent<ZenBehaviour>();
    const char* script =
        "class HoldTarget:\n"
        "    def on_start(self):\n"
        "        self.target = self.scene.find(\"Target\")\n"
        "    def on_update(self, dt):\n"
        "        name = self.target.get_name()\n"
        "        if name == \"\":\n"
        "            self.node.set_name(\"StillGone\")\n"
        "        else:\n"
        "            self.node.set_name(name)\n";
    CHECK(behaviour->loadSource(script));

    scene.setRunningInEditor(false);
    scene.update(1.0f / 60.0f);
    CHECK(!behaviour->hasError());

    scene.destroy(target);
    scene.update(1.0f / 60.0f);

    GameObject* freshObject = scene.createGameObject("NewOne");
    CHECK(freshObject != nullptr);

    scene.update(1.0f / 60.0f);
    CHECK(!behaviour->hasError());
    CHECK(holder->name() == "StillGone");
}

} // namespace

int main()
{
    testScriptRotatesObjectOnPlay();
    testMoveScriptUsesScriptComponentContract();
    testScriptDoesNotRunInEditorMode();
    testClassWithOnlyOneHookLoadsFine();
    testScriptWithNoBehaviourClassFailsToLoad();
    testScriptErrorIsCapturedNotThrown();
    testGameObjectTransformAndVec3Arithmetic();
    testSceneFindAndCreateBindings();
    testCollectionBetweenFramesKeepsBindingsAlive();
    testSharedScriptCompilesOnceAndKeepsPerInstanceState();
    testManyScriptedObjectsUseIndependentRootsAndReleaseThem();
    testReloadPropagatesToAllSharingComponents();
    testZenBehaviourSurvivesSerializerRoundTrip();
    testMissingScriptFileLoadsAsWarningNotError();
    testDeclaredPropertyScan();
    testClassBodyPropertiesComeFromTheCompiledClass();
    testClassBodyAndInitPropertiesCoexist();
    testInitRunsAndOverrideWinsOverIt();
    testClearOverrideRestoresTheDeclaredDefault();
    testOverridesSurviveSerializerRoundTrip();
    testCallEventInvokesOnEventHook();
    testCallEventReturnsFalseWithoutOnEventHook();
    testCallFunctionInvokesNamedMethodAndHasFunctionSeesIt();
    testCallFunctionFailsForUnknownName();
    testReloadIfChangedDetectsDiskEditAndSourceTimestampTracksIt();
    testReloadIfChangedFalseWithoutFile();
    testCallAndGlobalRoundTrip();
    testOnCollisionSeesOtherObjectName();
    testCallCollisionPassesBeganFlag();
    testComponentBaseIsActive();
    testGetComponentByClass();
    testScriptDrivesRigidBody();
    testScriptCommandsAJointServo();
    testComponentHandleIsCached();
    testHandleForgottenWhenOwnerDies();
    testHandleSurvivesBetweenFrames();
    testMeshRendererVisibleInReflections();
    testMeshRendererSubmeshVisibility();
    testMeshRendererCountsAreIntegers();
    testMeshRendererSubmeshCount();
    testMeshRendererSetsPrimitiveMesh();
    testGenericSpellingReachesTheSameNative();
    testCharacterControllerTuning();
    testCharacterControllerMoveReturnsResult();
    testCharacterControllerMoveInputRoundTrip();
    testVec3ArgumentTypeIsChecked();
    testAnimatorPlaysClipFromScript();
    testAnimatorPlayModeConstants();
    testAnimationLayerHandleSurvivesLayerGrowth();
    testAnimationLayerTimeAndSeek();
    testGameObjectHierarchyFromScript();
    testGameObjectDisposeIsDeferred();
    testGameObjectAddAndRemoveComponent();
    testGameObjectGlobalTransform();
    testSceneCreateWithParentAndReparent();
    testReadGameObjectRejectsOtherHandles();
    testGameObjectHandleSurvivesOwnerDestruction();
    testComponentHandleSurvivesComponentRemoval();
    testAnimationLayerHandleSurvivesAnimatorRemoval();
    testGameObjectHandleFollowsIdNotPointer();

    if (gFailures)
        std::fprintf(stderr, "%d zen behaviour test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
