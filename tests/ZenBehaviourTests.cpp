// ZenBehaviourTests.cpp - exercises Radion::ZenBehaviour end to end: a Zen
// behaviour class attached to a GameObject through the Scene/GameObject API,
// run for a few frames, and checked for the exact effect it should have had.

#include "PCH.h"

#include "Collider.h"
#include "CollisionWorld.h"
#include "GameObject.h"
#include "Scene.h"
#include "SceneSerializer.h"
#include "ScriptCache.h"
#include "ZenBehaviour.h"

#include "zen/vm.h"

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

// The exact use case the ZenBehaviour component exists for: a script class
// that spins its own object on every on_update(self, dt), only once Play
// (i.e. runningInEditor(false)) is active.
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

    const glm::quat startRotation = object->rotation();

    scene.setRunningInEditor(false);
    for (int i = 0; i < 10; ++i)
        scene.update(1.0f / 60.0f);

    CHECK(!behaviour->hasError());
    CHECK(object->name() == "Started by node");
    CHECK(object->rotation() != startRotation);
}

// The shipped 3D sample uses the current script contract: it inherits from
// ScriptComponent, has no __init__, and receives self.node before on_start.
// Loading the real file guards both the VM-facing base class and the example
// users copy into their projects.
void testMoveScriptUsesScriptComponentContract()
{
    const std::filesystem::path path =
        std::filesystem::path(RADION_TEST_ASSET_DIR) / "scripts" / "move.py";

    Scene scene;
    GameObject* object = scene.createGameObject("MoveRotate");
    CHECK(object != nullptr);
    if (!object)
        return;

    object->setPosition(glm::vec3(4.0f, 2.0f, -1.0f));
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();
    CHECK(behaviour != nullptr);
    if (!behaviour)
        return;

    CHECK(behaviour->loadFile(path.string()));

    const glm::quat startRotation = object->rotation();
    scene.setRunningInEditor(false);
    scene.update(0.5f);

    const glm::vec3 position = object->position();
    const f32 dx = position.x - 4.0f;
    const f32 dz = position.z + 1.0f;
    CHECK(!behaviour->hasError());
    CHECK(std::abs(position.y - 2.0f) < 0.001f);
    CHECK(std::abs(dx * dx + dz * dz - 9.0f) < 0.001f);
    CHECK(object->rotation() != startRotation);
}

// Same script, but the scene never leaves editor mode: on_update() must
// never reach the script, so the object never moves.
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

    const glm::quat startRotation = object->rotation();
    scene.setRunningInEditor(true);
    for (int i = 0; i < 10; ++i)
        scene.update(1.0f / 60.0f);

    CHECK(object->rotation() == startRotation);
    CHECK(!behaviour->hasError());
}

// A behaviour class only has to define the hooks it needs - on_update() is
// simply never called when the class does not define it.
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

// A script that defines no class with on_start/on_update/on_destroy has no
// way to be identified as a behaviour, and loading it must fail loudly
// instead of silently doing nothing.
void testScriptWithNoBehaviourClassFailsToLoad()
{
    Scene scene;
    GameObject* object = scene.createGameObject("NoBehaviour");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();

    CHECK(!behaviour->loadSource("value = 1\n"));
    CHECK(behaviour->hasError());
    CHECK(!behaviour->lastError().empty());
}

// A runtime error inside on_update() must not bring the frame down - it is
// captured, the behaviour stops calling into the script, and lastError()
// carries the message for the editor to show.
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

    // A second frame on a failed behaviour must stay a no-op, not crash.
    scene.update(1.0f / 60.0f);
    CHECK(behaviour->hasError());
}

// GameObject's transform binding (position/scale/rotation getters and
// setters) and the Vec3 class (constructor, x/y/z fields, __add__, __mul__)
// - verified from C++ against the real GameObject transform, not just "it
// compiled".
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
    CHECK(object->position() == glm::vec3(1.0f, 2.0f, 3.0f));
    CHECK(object->scale() == glm::vec3(2.0f, 2.0f, 4.0f));

    const glm::vec3 rotationDegrees = glm::degrees(glm::eulerAngles(object->rotation()));
    CHECK(std::abs(rotationDegrees.y - 6.0f) < 0.01f);
}

// The "scene" field: find() locating another object by name and create()
// adding a new one, both checked against the real Scene/GameObject state.
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
        CHECK(spawned->position() == glm::vec3(4.0f, 5.0f, 6.0f));
}

// A full collection between every frame, with a script that allocates on
// each one. Two things must survive it: the field NAMES of the native Vec3
// class (they are interned strings the class alone holds, compared by
// pointer on every "p.x"), and the GameObject/Scene wrappers the script
// keeps churning out - collecting a wrapper must not touch the real object
// it points at.
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
    CHECK(object->position() == glm::vec3(32.0f, 64.0f, 96.0f));
    CHECK(anchor->position() == object->position());
    CHECK(anchor->name() == "Anchor");
}

// The scenario the whole ScriptCache design exists for: many objects (here,
// three) sharing one script. The body must compile exactly once, and each
// object still ends up with its own state (a different self.speed, picked
// from its own self.owner.get_name() in on_start), not one shared globally.
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

    const f32 yawA = glm::degrees(glm::eulerAngles(a->rotation())).y;
    const f32 yawB = glm::degrees(glm::eulerAngles(b->rotation())).y;
    const f32 yawC = glm::degrees(glm::eulerAngles(c->rotation())).y;

    CHECK(std::abs(yawA - 30.0f * dt * frames) < 0.05f);
    CHECK(std::abs(yawB - 60.0f * dt * frames) < 0.05f);
    CHECK(std::abs(yawC - 90.0f * dt * frames) < 0.05f);
}

// Script-heavy scenes should touch the components that exist, not every
// possible component slot on every object. This also leaves no stale roots
// behind when a whole burst of scripted objects goes away: ScriptCache keeps
// its GC roots in a vector for marking, but removal is indexed/swap-pop.
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

// reload() on ANY one component sharing a script path must apply to every
// other component sharing that same path, not just the caller's own.
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

    const f32 yawFirstBefore = glm::degrees(glm::eulerAngles(first->rotation())).y;
    const f32 yawSecondBefore = glm::degrees(glm::eulerAngles(second->rotation())).y;
    CHECK(std::abs(yawFirstBefore - 30.0f * dt) < 0.01f);
    CHECK(std::abs(yawSecondBefore - 30.0f * dt) < 0.01f);

    // Only "first" calls reload(), but the recompiled script must apply to
    // "second" too on its very next frame - they share one cache entry.
    writeReloadScript(path, 300.0f);
    CHECK(behaviourFirst->reload());

    scene.update(dt);

    const f32 yawFirstAfter = glm::degrees(glm::eulerAngles(first->rotation())).y;
    const f32 yawSecondAfter = glm::degrees(glm::eulerAngles(second->rotation())).y;
    CHECK(std::abs((yawFirstAfter - yawFirstBefore) - 300.0f * dt) < 0.05f);
    CHECK(std::abs((yawSecondAfter - yawSecondBefore) - 300.0f * dt) < 0.05f);

    std::error_code removeError;
    std::filesystem::remove(path, removeError);
}

// A ZenBehaviour is the one thing on the shared Script slot with a fixed
// shape to write (a script path), so it does go through the serializer -
// and it has to: the editor's Play snapshot is a scene document, and
// without this Stop would restore every scripted object stripped of its
// script.
void testZenBehaviourSurvivesSerializerRoundTrip()
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "radion_zen_behaviour_roundtrip.py";
    writeReloadScript(path, 45.0f);

    Scene scene;
    GameObject* object = scene.createGameObject("Scripted");
    ZenBehaviour* behaviour = object->addComponent<ZenBehaviour>();
    CHECK(behaviour->loadFile(path.string()));
    // Editor mode for the flush: the add is queued until an update, and the
    // script must not have turned the object before it is serialized.
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

        // The restored component is a live behaviour, not just a stored
        // path: it still turns its own object on the next frame.
        reloaded.setRunningInEditor(false);
        const f32 dt = 1.0f / 60.0f;
        reloaded.update(dt);
        CHECK(!reBehaviour->hasError());
        CHECK(std::abs(glm::degrees(glm::eulerAngles(reObject->rotation())).y - 45.0f * dt) < 0.01f);
    }

    std::error_code removeError;
    std::filesystem::remove(path, removeError);
}

// A script file that is gone (or no longer compiles) must not sink the whole
// scene: the load succeeds, the component and its path come back, and the
// error is there for the inspector to show.
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

// The property scanner, over the shapes it has to get right: literals of
// each kind, a top-level constant referenced by name, a private name, a
// value it cannot read, and a second method whose locals are not properties.
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

    // Private, unreadable, and another method's local are all left out.
    CHECK(behaviour->declaredProperty("_hidden") == nullptr);
    CHECK(behaviour->declaredProperty("computed") == nullptr);
    CHECK(behaviour->declaredProperty("step") == nullptr);
}

// The path that needs no constructor and no text parsing: a field declared
// in the class body is recorded by the compiler, so the name, the value and
// the type come off the compiled class exactly as written.
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

    // Private, valueless, and a field only ever written inside a method are
    // all out: working state is not something the inspector drives.
    CHECK(behaviour->declaredProperty("_hidden") == nullptr);
    CHECK(behaviour->declaredProperty("nothing") == nullptr);
    CHECK(behaviour->declaredProperty("scratch") == nullptr);

    // And they are real: no constructor ran, yet the object turns at 90.
    behaviour->setNumberOverride("speed", 180.0, false);
    scene.setRunningInEditor(false);
    const f32 dt = 1.0f / 60.0f;
    scene.update(dt);
    CHECK(!behaviour->hasError());
    CHECK(std::abs(glm::degrees(glm::eulerAngles(object->rotation())).y - 180.0f * dt) < 0.01f);

    // The bindings' own fields are added to the class when an instance is
    // bound; they must never turn into properties.
    CHECK(behaviour->declaredProperty("owner") == nullptr);
    CHECK(behaviour->declaredProperty("scene") == nullptr);
}

// A class body declaration and a constructor in the same script: both show
// up, and the class body wins for a name they both mention.
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

    // Declared in both: the class body's value is the one listed.
    const ScriptProperty* shared = behaviour->declaredProperty("shared");
    CHECK(shared && shared->number == 1.0);
}

// __init__ has to actually run - the declared defaults only exist because
// it does - and an override has to land on top of what it wrote.
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

    // The plain one runs the script's own default, the tuned one three
    // times that - one script, one compile, two different objects.
    const f32 plainYaw = glm::degrees(glm::eulerAngles(plain->rotation())).y;
    const f32 tunedYaw = glm::degrees(glm::eulerAngles(tuned->rotation())).y;
    CHECK(std::abs(plainYaw - 60.0f * dt) < 0.01f);
    CHECK(std::abs(tunedYaw - 180.0f * dt) < 0.01f);
}

// Dropping an override puts the script's own default back, live.
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
    const f32 overriddenStep = glm::degrees(glm::eulerAngles(object->rotation())).y;
    CHECK(std::abs(overriddenStep - 600.0f * dt) < 0.05f);

    behaviour->clearOverride("speed");
    CHECK(behaviour->overrideCount() == 0);

    scene.update(dt);
    const f32 after = glm::degrees(glm::eulerAngles(object->rotation())).y;
    CHECK(std::abs((after - overriddenStep) - 60.0f * dt) < 0.05f);
    CHECK(!behaviour->hasError());
}

// Overrides go through the scene file; the defaults deliberately do not.
void testOverridesSurviveSerializerRoundTrip()
{
    // From a file, not a source string: the scene stores the script's path,
    // so a behaviour built with loadSource() has nothing to write there.
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

    // The int stays an int: a script testing "self.lives == 7" would break
    // against a 7.0 restored from the file.
    const ScriptProperty* lives = reBehaviour->findOverride("lives");
    CHECK(lives && lives->kind == ScriptProperty::Kind::Number && lives->integer);
    if (lives)
        CHECK(lives->number == 7.0);

    const ScriptProperty* label = reBehaviour->findOverride("label");
    CHECK(label && label->kind == ScriptProperty::Kind::String && label->text == "z");

    const ScriptProperty* enabled = reBehaviour->findOverride("enabled");
    CHECK(enabled && enabled->kind == ScriptProperty::Kind::Bool && enabled->flag);

    // And they are live, not just stored: the restored object turns at the
    // overridden speed, not the script's 1.0.
    reloaded.setRunningInEditor(false);
    const f32 dt = 1.0f / 60.0f;
    reloaded.update(dt);
    CHECK(!reBehaviour->hasError());
    CHECK(std::abs(glm::degrees(glm::eulerAngles(reObject->rotation())).y - 45.0f * dt) < 0.01f);

    std::error_code removeError;
    std::filesystem::remove(path, removeError);
}

// on_collision(self, other) - CollisionWorld::step() (wired into
// Scene::update()) calls it once per contact, with "other" bound as the
// GameObject wrapper of the collider on the far side. Proven by having the
// script copy the other object's own name onto self.owner, then reading it
// back from C++.
void testOnCollisionSeesOtherObjectName()
{
    Scene scene;
    GameObject* watcher = scene.createGameObject("Watcher");
    GameObject* bumper = scene.createGameObject("Bumper");
    bumper->setPosition(glm::vec3(0.5f, 0.0f, 0.0f));

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

// ScriptVM::call()/setGlobal() directly - the generic C++ <-> script call
// path, independent of ZenBehaviour's own class-based dispatch.
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
    testCallAndGlobalRoundTrip();
    testOnCollisionSeesOtherObjectName();

    if (gFailures)
        std::fprintf(stderr, "%d zen behaviour test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
