#include "PCH.h"

#include "ActionRunner.h"
#include "AssetManager.h"
#include "AudioPlayer.h"
#include "ByteArray.h"
#include "DebugDraw3D.h"
#include "EnvironmentProbe.h"
#include "FileSystem.h"
#include "Forest.h"
#include "FrameContext.h"
#include "Grass.h"
#include "Hair.h"
#include "Landscape.h"
#include "LensFlarePass.h"
#include "Light.h"
#include "Lighting.h"
#include "MaterialManager.h"
#include "MaterialParserInternal.h"
#include "Ocean.h"
#include "ParticleEffect.h"
#include "ParticleEffectPool.h"
#include "PhysicsBody.h"
#include "PostProcess.h"
#include "Prefab.h"
#include "Profiler.h"
#include "RadionFormat.h"
#include "RenderList.h"
#include "RibbonTrail.h"
#include "Road.h"
#include "Scene.h"
#include "SceneSerializer.h"
#include "Shadows.h"
#include "Sky.h"
#include "Terrain.h"
#include "UiControls.h"
#include "VolumetricPass.h"

#include <cstdio>
#include <filesystem>
#include <limits>

using namespace Radion;

namespace
{

int gFailures = 0;
int gScriptDestroyed = 0;

class TestScript final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Script;

    TestScript() : Component(Type, ComponentEventUpdate | ComponentEventLateUpdate)
    {
    }

    int awakeCount = 0;
    int startCount = 0;
    int enableCount = 0;
    int disableCount = 0;
    int updateCount = 0;
    int lateUpdateCount = 0;

private:
    void onAwake() override
    {
        ++awakeCount;
    }
    void onStart() override
    {
        ++startCount;
    }
    void onEnable() override
    {
        ++enableCount;
    }
    void onDisable() override
    {
        ++disableCount;
    }
    void onUpdate(f32) override
    {
        ++updateCount;
    }
    void onLateUpdate(f32) override
    {
        ++lateUpdateCount;
    }
    void onDestroy() override
    {
        ++gScriptDestroyed;
    }
};

class SelfRemovingComponent final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::SelfDestroy;

    SelfRemovingComponent() : Component(Type, ComponentEventUpdate)
    {
    }

private:
    void onUpdate(f32) override
    {
        owner()->removeComponent<SelfRemovingComponent>();
    }
};

class CountingComponent final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Waypoints;

    CountingComponent() : Component(Type, ComponentEventUpdate)
    {
    }

    u32 updates = 0;

private:
    void onUpdate(f32) override
    {
        ++updates;
    }
};

void check(bool condition, const char* expression, int line)
{
    if (condition)
        return;
    std::fprintf(stderr, "SceneTests:%d: failed: %s\n", line, expression);
    ++gFailures;
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(f32 a, f32 b, f32 epsilon = 0.0001f)
{
    return std::abs(a - b) <= epsilon;
}

bool near(const glm::vec3& a, const glm::vec3& b, f32 epsilon = 0.0001f)
{
    return glm::length(a - b) <= epsilon;
}

bool near(const glm::mat4& a, const glm::mat4& b, f32 epsilon = 0.0001f)
{
    for (u32 column = 0; column < 4; ++column)
        for (u32 row = 0; row < 4; ++row)
            if (!near(a[column][row], b[column][row], epsilon))
                return false;
    return true;
}

// Three bones stacked along +Y: root at the origin, then one unit up, then
// one more - a straight two-link chain whose tip starts at (0,2,0).
void buildIKTestSkeleton(Skeleton& skeleton)
{
    const glm::mat4 step = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    CHECK(skeleton.addBone("root", -1, glm::mat4(1.0f), glm::mat4(1.0f)));
    CHECK(skeleton.addBone("mid", 0, step, glm::inverse(step)));
    CHECK(skeleton.addBone("tip", 1, step, glm::inverse(step)));
    CHECK(skeleton.finalize());
}

void testInverseKinematics()
{
    Skeleton skeleton;
    buildIKTestSkeleton(skeleton);

    std::vector<LocalPose> localPose;
    std::vector<glm::mat4> globalPose;
    std::vector<glm::mat4> palette;
    skeleton.bindPose(localPose);
    skeleton.evaluate(localPose, globalPose, palette);

    // The starting pose is what the chain reaching assumes: tip two units up.
    CHECK(near(glm::vec3(globalPose[2][3]), glm::vec3(0.0f, 2.0f, 0.0f)));

    // A target the chain can physically reach: same distance from the root as
    // the chain is long, just in another direction. Unconstrained CCD should
    // fold onto it.
    IKChain chain;
    chain.tipBone = 2;
    chain.length = 2;
    chain.iterations = 32;
    chain.target = glm::vec3(2.0f, 0.0f, 0.0f);
    IKSolver::solve(skeleton, chain, glm::mat4(1.0f), localPose, globalPose);

    const glm::vec3 tip = glm::vec3(globalPose[2][3]);
    CHECK(std::isfinite(tip.x) && std::isfinite(tip.y) && std::isfinite(tip.z));
    // Loose on purpose: CCD converges towards the target, it does not land on
    // it exactly in a finite number of passes.
    CHECK(glm::distance(tip, chain.target) < 0.05f);

    // The solver has to hand back a localPose and a globalPose that still
    // agree with each other - the palette is rebuilt from localPose, so a
    // globalPose that drifted from it would skin against a pose nothing else
    // ever sees.
    std::vector<glm::mat4> reEvaluated;
    std::vector<glm::mat4> rePalette;
    skeleton.evaluate(localPose, reEvaluated, rePalette);
    for (u32 i = 0; i < skeleton.boneCount(); ++i)
        CHECK(near(reEvaluated[i], globalPose[i], 0.001f));

    // A target the chain cannot reach must not produce NaN - it should just
    // stretch as far as it goes.
    skeleton.bindPose(localPose);
    skeleton.evaluate(localPose, globalPose, palette);
    chain.target = glm::vec3(50.0f, 0.0f, 0.0f);
    IKSolver::solve(skeleton, chain, glm::mat4(1.0f), localPose, globalPose);
    const glm::vec3 stretched = glm::vec3(globalPose[2][3]);
    CHECK(std::isfinite(stretched.x) && std::isfinite(stretched.y) && std::isfinite(stretched.z));
    CHECK(near(glm::length(stretched), 2.0f, 0.01f)); // still two units of bone

    // A target exactly on the joint has no direction to rotate along: it must
    // bail rather than normalise a zero vector.
    skeleton.bindPose(localPose);
    skeleton.evaluate(localPose, globalPose, palette);
    chain.target = glm::vec3(0.0f, 1.0f, 0.0f); // the mid joint itself
    IKSolver::solve(skeleton, chain, glm::mat4(1.0f), localPose, globalPose);
    for (u32 i = 0; i < skeleton.boneCount(); ++i)
    {
        const glm::vec3 bone = glm::vec3(globalPose[i][3]);
        CHECK(std::isfinite(bone.x) && std::isfinite(bone.y) && std::isfinite(bone.z));
    }

    // The constrained path: this only asserts it stays finite and bounded, NOT
    // that it converges - a hinge deliberately cannot reach most targets, and
    // the reference's own per-axis formula is what decides how far it gets.
    skeleton.bindPose(localPose);
    skeleton.evaluate(localPose, globalPose, palette);
    chain.target = glm::vec3(1.5f, 0.5f, 0.0f);
    chain.constraints[0] = IKConstraint::knee();
    chain.constraints[1] = IKConstraint::thigh();
    IKSolver::solve(skeleton, chain, glm::mat4(1.0f), localPose, globalPose);
    for (u32 i = 0; i < skeleton.boneCount(); ++i)
    {
        const glm::vec3 bone = glm::vec3(globalPose[i][3]);
        CHECK(std::isfinite(bone.x) && std::isfinite(bone.y) && std::isfinite(bone.z));
        CHECK(glm::length(bone) < 3.0f); // no joint flew off
    }

    // inverted() is the reference's knee_bending < 0 case: min and max swap,
    // nothing negates.
    const IKConstraint knee = IKConstraint::knee();
    const IKConstraint flipped = knee.inverted();
    CHECK(near(flipped.minimum, knee.maximum));
    CHECK(near(flipped.maximum, knee.minimum));
    CHECK(flipped.enabled);

    // A chain pointing at nothing must be a no-op, not a crash.
    std::vector<LocalPose> untouched;
    skeleton.bindPose(untouched);
    localPose = untouched;
    skeleton.evaluate(localPose, globalPose, palette);
    IKChain empty;
    empty.tipBone = -1;
    IKSolver::solve(skeleton, empty, glm::mat4(1.0f), localPose, globalPose);
    for (u32 i = 0; i < skeleton.boneCount(); ++i)
        CHECK(near(localPose[i].position, untouched[i].position));
}

void testAnimatedPlayers()
{
    Skeleton skeleton;
    const glm::mat4 rootBind =
        glm::translate(glm::mat4(1.0f), glm::vec3(3.0f, -2.0f, 5.0f)) *
        glm::mat4_cast(glm::angleAxis(glm::radians(37.0f), glm::normalize(glm::vec3(1, 2, 3)))) *
        glm::scale(glm::mat4(1.0f), glm::vec3(2.0f, 3.0f, 0.5f));
    CHECK(skeleton.addBone("root", -1, rootBind, glm::inverse(rootBind)));
    CHECK(skeleton.addBone("hand", 0, glm::translate(glm::mat4(1.0f), glm::vec3(0, 1, 0)),
                           glm::translate(glm::mat4(1.0f), glm::vec3(0, -1, 0))));
    CHECK(skeleton.finalize());
    std::vector<LocalPose> bindPose;
    skeleton.bindPose(bindPose);
    const glm::mat4 recomposed = glm::translate(glm::mat4(1.0f), bindPose[0].position) *
                                 glm::mat4_cast(bindPose[0].rotation) *
                                 glm::scale(glm::mat4(1.0f), bindPose[0].scale);
    CHECK(near(recomposed, rootBind));

    AnimationClip clip;
    clip.setName("Move");
    clip.setDuration(2.0f);
    BoneTrack track;
    track.bone = 0;
    track.times = {0.0f, 2.0f};
    track.positions = {glm::vec3(0.0f), glm::vec3(2.0f, 0.0f, 0.0f)};
    track.rotations = {glm::quat(1, 0, 0, 0), glm::quat(1, 0, 0, 0)};
    track.scales = {glm::vec3(1.0f), glm::vec3(1.0f)};
    clip.tracks().push_back(track);
    const std::vector<AnimationClip> clips = {clip};
    const AnimationSetHandle animationSet = Animations().create(skeleton, clips);

    Scene scene;
    GameObject* players[4];
    Animator* animators[4];
    for (u32 i = 0; i < 4; ++i)
    {
        players[i] = scene.createGameObject("player");
        players[i]->addComponent<MeshRenderer>();
        animators[i] = players[i]->addComponent<Animator>();
        animators[i]->bind(animationSet);
        animators[i]->play("Move", PlayMode::Loop, 0.0f);
        animators[i]->layer(0).setSpeed(static_cast<f32>(i + 1));
    }

    scene.update(0.25f);
    CHECK(scene.animatedCount() == 4);
    CHECK(scene.renderableCount() == 4);
    for (u32 i = 0; i < 4; ++i)
    {
        CHECK(players[i]->getComponent<MeshRenderer>() != nullptr);
        CHECK(players[i]->getComponent<Animator>() == animators[i]);
        CHECK(animators[i]->palette().size() == 2);
    }
    CHECK(near(animators[0]->localPose()[0].position.x, 0.25f));
    CHECK(near(animators[3]->localPose()[0].position.x, 1.0f));
    CHECK(animators[0]->localPose()[0].position != animators[3]->localPose()[0].position);

    players[2]->setActive(false);
    const f32 stopped = animators[2]->layer(0).time();
    scene.update(0.25f);
    CHECK(near(animators[2]->layer(0).time(), stopped));
    CHECK(animators[0]->layer(0).time() != animators[1]->layer(0).time());

    CHECK(Animations().destroy(animationSet));
    CHECK(!animators[0]->bound());
    scene.update(0.25f);
}

void testRadionFormat()
{
    ByteArray data;
    AssetFormat::Writer writer(data);
    writer.header(AssetFormat::MeshMagic);
    const u64 marker = writer.beginChunk(AssetFormat::MaterialSlots);
    writer.writeU32(2);
    writer.string("Sinbad/Body");
    writer.string("Sinbad/Clothes");
    CHECK(writer.endChunk(marker));

    data.seek(0);
    AssetFormat::Reader reader(data);
    std::string error;
    CHECK(reader.header(AssetFormat::MeshMagic, &error));
    AssetFormat::ChunkHeader chunk;
    CHECK(reader.next(chunk));
    CHECK(chunk.id == AssetFormat::MaterialSlots);
    CHECK(reader.enter(chunk));
    u32 count = 0;
    CHECK(reader.readU32(count));
    CHECK(count == 2);
    std::string first;
    std::string second;
    CHECK(reader.string(first));
    CHECK(reader.string(second));
    CHECK(first == "Sinbad/Body");
    CHECK(second == "Sinbad/Clothes");
    CHECK(reader.remaining() == 0);
    reader.leave();

    ByteArray truncated(data.data(), data.size() - 1, false);
    truncated.seek(0);
    AssetFormat::Reader invalid(truncated);
    CHECK(invalid.header(AssetFormat::MeshMagic));
    CHECK(!invalid.next(chunk));
}

void testHierarchyAndOrder()
{
    Scene scene;
    GameObject* parent = scene.createGameObject("parent");
    GameObject* a = scene.createGameObject("a", parent);
    GameObject* b = scene.createGameObject("b", parent);
    GameObject* c = scene.createGameObject("c", parent);

    CHECK(parent && a && b && c);
    CHECK(parent->id() != 0);
    CHECK(a->id() != parent->id());
    CHECK(b->id() != a->id());
    parent->setTag("actors");
    CHECK(parent->tag() == "actors");
    CHECK(parent->active());
    CHECK(parent->visible());
    CHECK(a->isActiveInHierarchy());
    CHECK(a->isVisibleInHierarchy());
    parent->setActive(false);
    parent->setVisible(false);
    CHECK(!a->isActiveInHierarchy());
    CHECK(!a->isVisibleInHierarchy());
    CHECK(a->active());
    CHECK(a->visible());
    parent->setActive(true);
    parent->setVisible(true);
    parent->setDebugFlags(DebugObjectAxis | DebugMeshBounds);
    CHECK(parent->hasDebugFlag(DebugObjectAxis));
    CHECK(parent->debugFlags() == (DebugObjectAxis | DebugMeshBounds));
    parent->removeDebugFlags(DebugMeshBounds);
    CHECK(parent->debugFlags() == DebugObjectAxis);
    parent->setDebugFlags(DebugNone);
    CHECK(parent->parent() == nullptr);
    CHECK(parent->childCount() == 3);
    CHECK(parent->child(0) == a);
    CHECK(parent->childIndex(b) == 1);
    CHECK(parent->findChild("c") == c);
    CHECK(parent->root() == parent);

    CHECK(parent->moveChildUp(b));
    CHECK(parent->child(0) == b);
    CHECK(!parent->moveChildUp(b));
    CHECK(parent->moveChildDown(b));
    CHECK(parent->child(1) == b);
    CHECK(parent->moveChild(c, 0));
    CHECK(parent->child(0) == c);

    CHECK(!a->addChild(parent));
    CHECK(!parent->addChild(parent));
    CHECK(!parent->addChild(nullptr));

    scene.update(0.016f);
    CHECK(parent->parent() == &scene.root());
    CHECK(parent->root() == &scene.root());
}

void testGameObjectIds()
{
    Scene scene;
    GameObject* first = scene.createGameObject("first");
    GameObject* second = scene.createGameObject("second");

    CHECK(first->id() == 1);
    CHECK(second->id() == 2);
    CHECK(scene.root().id() == 0);
    CHECK(scene.findGameObject(first->id()) == first);
    CHECK(scene.findGameObject(second->id()) == second);
    CHECK(scene.findGameObject(0) == nullptr);
    CHECK(scene.findGameObject(9999) == nullptr);

    // Per-Scene, not global: a second Scene starts counting from 1 again.
    Scene other;
    GameObject* stranger = other.createGameObject("stranger");
    CHECK(stranger->id() == 1);
    CHECK(other.findGameObject(1) == stranger);
    CHECK(scene.findGameObject(1) == first);

    // Restoring an id, as loading a saved scene or undoing a delete does.
    GameObject* restored = scene.createGameObject(100, "restored");
    CHECK(restored && restored->id() == 100);
    CHECK(scene.findGameObject(100) == restored);
    // The counter jumped past it, so nothing later lands on 100.
    CHECK(scene.createGameObject("after")->id() == 101);

    // Taken and reserved ids are refused rather than quietly renumbered.
    CHECK(scene.createGameObject(100, "clash") == nullptr);
    CHECK(scene.createGameObject(0, "zero") == nullptr);
    CHECK(scene.findGameObject(100) == restored);

    // A destroyed object takes its whole branch out of the table.
    GameObject* branch = scene.createGameObject(200, "branch");
    GameObject* leaf = scene.createGameObject(201, "leaf", branch);
    scene.update(0.016f);
    CHECK(scene.findGameObject(201) == leaf);
    scene.destroy(branch);
    scene.update(0.016f);
    CHECK(scene.findGameObject(200) == nullptr);
    CHECK(scene.findGameObject(201) == nullptr);

    // Removed is not destroyed: the object keeps its id and can come back,
    // which is what a delete/undo pair does.
    GameObject* parked = scene.createGameObject(300, "parked");
    scene.update(0.016f);
    scene.remove(parked);
    scene.update(0.016f);
    CHECK(parked->id() == 300);
    CHECK(scene.findGameObject(300) == parked);
    CHECK(scene.add(parked));
    scene.update(0.016f);
    CHECK(scene.findGameObject(300) == parked);
}

void testSceneSerializerEmptyRoundTrip()
{
    Scene scene;
    SceneSerializer serializer;

    const nlohmann::json document = serializer.toJson(scene);
    CHECK(document["format"] == "radion-scene");
    CHECK(document["version"] == 1);
    CHECK(document["scene"]["objects"].empty());

    Scene reloaded;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, reloaded, result));
    CHECK(result.success());
    CHECK(reloaded.gameObjectCount() == 0);
}

void testSceneSerializerFailedLoadLeavesSceneIntact()
{
    Scene scene;
    GameObject* marker = scene.createGameObject("marker");
    marker->setPosition(glm::vec3(1.0f, 2.0f, 3.0f));
    scene.update(0.016f);

    SceneSerializer serializer;
    SceneLoadResult result;
    // Not even a JSON object - the earliest possible rejection.
    CHECK(!serializer.fromJson(nlohmann::json::array(), scene, result));
    CHECK(!result.success());
    // Untouched: the marker object created before the failed load is still
    // exactly what it was.
    CHECK(scene.gameObjectCount() == 1);
    CHECK(scene.findGameObject(marker->id()) == marker);
    CHECK(near(marker->position(), glm::vec3(1.0f, 2.0f, 3.0f)));

    nlohmann::json badVersion;
    badVersion["format"] = "radion-scene";
    badVersion["version"] = 999;
    badVersion["scene"] = nlohmann::json::object();
    badVersion["scene"]["objects"] = nlohmann::json::array();
    SceneLoadResult versionResult;
    CHECK(!serializer.fromJson(badVersion, scene, versionResult));
    CHECK(!versionResult.success());
    CHECK(scene.gameObjectCount() == 1);
}

void testSceneSerializerHierarchyRoundTrip()
{
    Scene scene;
    GameObject* grandparent = scene.createGameObject("grandparent");
    grandparent->setPosition(glm::vec3(1.0f, 2.0f, 3.0f));
    grandparent->setTag("root-actor");
    grandparent->setStatic(true);

    GameObject* parent = scene.createGameObject("parent", grandparent);
    parent->setRotation(glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    parent->setVisible(false);

    GameObject* childA = scene.createGameObject("childA", parent);
    childA->setScale(glm::vec3(2.0f, 1.0f, 0.5f));
    GameObject* childB = scene.createGameObject("childB", parent);
    childB->setActive(false);

    scene.update(0.016f);

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene);
    CHECK(document["scene"]["objects"].size() == 4);

    // Determinism: serializing an unchanged scene twice must be byte for
    // byte identical.
    CHECK(document.dump(4) == serializer.toJson(scene).dump(4));

    Scene reloaded;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, reloaded, result));
    CHECK(result.success());
    CHECK(reloaded.gameObjectCount() == 4);

    GameObject* reGrandparent = reloaded.findGameObject(grandparent->id());
    GameObject* reParent = reloaded.findGameObject(parent->id());
    GameObject* reChildA = reloaded.findGameObject(childA->id());
    GameObject* reChildB = reloaded.findGameObject(childB->id());
    CHECK(reGrandparent && reParent && reChildA && reChildB);

    CHECK(reGrandparent->parent() == &reloaded.root());
    CHECK(reParent->parent() == reGrandparent);
    CHECK(reChildA->parent() == reParent);
    CHECK(reChildB->parent() == reParent);
    // Sibling order preserved.
    CHECK(reParent->childIndex(reChildA) == 0);
    CHECK(reParent->childIndex(reChildB) == 1);

    CHECK(reGrandparent->tag() == "root-actor");
    CHECK(reGrandparent->isStatic());
    CHECK(near(reGrandparent->position(), grandparent->position()));
    CHECK(!reParent->visible());
    CHECK(near(reParent->rotation().x, parent->rotation().x));
    CHECK(near(reParent->rotation().w, parent->rotation().w));
    CHECK(near(reChildA->scale(), childA->scale()));
    CHECK(!reChildB->active());

    // Re-serializing the reload must match the original document - a
    // round-trip that changed nothing must not perturb the file.
    CHECK(serializer.toJson(reloaded).dump(4) == document.dump(4));
}

void testSceneSerializerValidation()
{
    SceneSerializer serializer;

    auto baseDocument = [](nlohmann::json objects)
    {
        nlohmann::json document;
        document["format"] = "radion-scene";
        document["version"] = 1;
        document["scene"] = nlohmann::json::object();
        document["scene"]["objects"] = std::move(objects);
        return document;
    };

    auto validObject = [](u64 id, const nlohmann::json& parent)
    {
        nlohmann::json object;
        object["id"] = id;
        object["parent"] = parent;
        object["name"] = "object";
        object["tag"] = "";
        object["flags"] = {{"active", true}, {"visible", true}, {"static", false}};
        object["transform"] = {{"position", {0.0, 0.0, 0.0}},
                               {"rotation", {0.0, 0.0, 0.0, 1.0}},
                               {"scale", {1.0, 1.0, 1.0}}};
        return object;
    };

    // Duplicate id.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(validObject(1, nullptr));
        objects.push_back(validObject(1, nullptr));
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
        CHECK(scene.gameObjectCount() == 0);
    }

    // Id zero is reserved.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(validObject(0, nullptr));
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
    }

    // Self-parent.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(validObject(1, 1));
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
    }

    // Parent that does not exist.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(validObject(1, 42));
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
    }

    // Parent cycle: 1 -> 2 -> 1.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(validObject(1, 2));
        objects.push_back(validObject(2, 1));
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
    }

    // A child listed before its parent in the array must still work - order
    // in the file is not assumed to be pre-order.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(validObject(2, 1)); // child first
        objects.push_back(validObject(1, nullptr));
        CHECK(serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(result.success());
        GameObject* parent = scene.findGameObject(1);
        GameObject* child = scene.findGameObject(2);
        CHECK(parent && child && child->parent() == parent);
    }

    // NaN/Infinity position.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json object = validObject(1, nullptr);
        object["transform"]["position"] = {std::numeric_limits<f32>::quiet_NaN(), 0.0, 0.0};
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(object);
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
    }

    // Zero-length rotation quaternion.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json object = validObject(1, nullptr);
        object["transform"]["rotation"] = {0.0, 0.0, 0.0, 0.0};
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(object);
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
    }

    // Zero scale component.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json object = validObject(1, nullptr);
        object["transform"]["scale"] = {1.0, 0.0, 1.0};
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(object);
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
    }

    // Wrong-sized transform array.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json object = validObject(1, nullptr);
        object["transform"]["position"] = {0.0, 0.0};
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(object);
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
    }

    // A missing name is a warning, not an error - the load still succeeds.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json object = validObject(1, nullptr);
        object.erase("name");
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(object);
        CHECK(serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(result.success());
        bool sawWarning = false;
        for (const SceneDiagnostic& diagnostic : result.diagnostics)
            sawWarning |= diagnostic.severity == SceneDiagnosticSeverity::Warning;
        CHECK(sawWarning);
    }
}

void testSceneSerializerEditorCreatedMarkersRoundTrip()
{
    Scene scene;
    scene.setStaticCullingEnabled(false);
    scene.setDynamicCullingEnabled(true);
    scene.setOcclusionQueryEnabled(true);
    GameObject* object = scene.createGameObject("world systems");
    Terrain* terrain = object->addComponent<Terrain>();
    terrain->setActive(false);
    terrain->setUvTiles(23.5f);
    terrain->grassGenerationSettings().spacing = 2.25f;
    terrain->grassGenerationSettings().density = 0.42f;
    terrain->grassGenerationSettings().seed = 12345u;
    terrain->treeGenerationSettings().maximumSlopeDegrees = 19.0f;
    terrain->treeGenerationSettings().maximumInstances = 321u;
    object->addComponent<Landscape>();
    object->addComponent<Road>();
    object->addComponent<Grass>();
    Hair* hair = object->addComponent<Hair>();
    hair->setStrandCount(1234);
    hair->setSegments(7);
    hair->setMinimumGrowthNormalY(0.15f);
    hair->setFollowers(3);
    hair->setLengthRange(0.21f, 0.64f);
    hair->setWidth(0.006f);
    hair->setColor(glm::vec3(0.18f, 0.07f, 0.02f));
    hair->setSpecularStrength(0.17f);
    hair->setSpecularTint(0.8f);
    hair->setTransmission(0.22f);
    hair->setSoftFringe(false);
    object->addComponent<Forest>();
    object->addComponent<Ocean>();
    scene.update(0.0f);

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene);
    Scene restored;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, restored, result));
    CHECK(result.success());
    GameObject* loaded = restored.findGameObject(object->id());
    CHECK(loaded != nullptr);
    CHECK(loaded && loaded->getComponent<Terrain>() != nullptr);
    CHECK(loaded && loaded->getComponent<Landscape>() != nullptr);
    CHECK(loaded && loaded->getComponent<Road>() != nullptr);
    CHECK(loaded && loaded->getComponent<Grass>() != nullptr);
    CHECK(loaded && loaded->getComponent<Hair>() != nullptr);
    CHECK(loaded && loaded->getComponent<Forest>() != nullptr);
    CHECK(loaded && loaded->getComponent<Ocean>() != nullptr);
    CHECK(loaded && loaded->getComponent<Terrain>() && !loaded->getComponent<Terrain>()->active());
    if (Hair* loadedHair = loaded ? loaded->getComponent<Hair>() : nullptr)
    {
        CHECK(loadedHair->strandCount() == 1234u);
        CHECK(loadedHair->segments() == 7u);
        CHECK(near(loadedHair->minimumGrowthNormalY(), 0.15f));
        CHECK(loadedHair->followers() == 3u);
        CHECK(near(loadedHair->minimumLength(), 0.21f));
        CHECK(near(loadedHair->maximumLength(), 0.64f));
        CHECK(near(loadedHair->width(), 0.006f));
        CHECK(near(loadedHair->color(), glm::vec3(0.18f, 0.07f, 0.02f)));
        CHECK(near(loadedHair->specularStrength(), 0.17f));
        CHECK(near(loadedHair->specularTint(), 0.8f));
        CHECK(near(loadedHair->transmission(), 0.22f));
        CHECK(!loadedHair->softFringe());
        CHECK(loadedHair->rootCount() == 0u);
    }
    if (Terrain* loadedTerrain = loaded ? loaded->getComponent<Terrain>() : nullptr)
    {
        CHECK(near(loadedTerrain->uvTiles(), 23.5f));
        CHECK(near(loadedTerrain->grassGenerationSettings().spacing, 2.25f));
        CHECK(near(loadedTerrain->grassGenerationSettings().density, 0.42f));
        CHECK(loadedTerrain->grassGenerationSettings().seed == 12345u);
        CHECK(near(loadedTerrain->treeGenerationSettings().maximumSlopeDegrees, 19.0f));
        CHECK(loadedTerrain->treeGenerationSettings().maximumInstances == 321u);
        CHECK((loadedTerrain->material().flags & MaterialTerrain) != 0);
    }
    CHECK(!restored.staticCullingEnabled());
    CHECK(restored.dynamicCullingEnabled());
    CHECK(restored.occlusionQueryEnabled());
}

void testSceneSerializerRenderSettingsRoundTrip()
{
    Scene scene;
    CascadeShadowSettings shadows;
    ShadowAtlasSettings atlas;
    PostProcessStack post;
    LensFlarePass lensFlare;
    EnvironmentProbe probe;
    Lighting lighting;
    VolumetricPass volumetric;
    SkySettings sky;
    RenderResolution resolution{1600, 900, 0.75f};
    shadows.distance = 432.0f;
    shadows.quality = 4;
    shadows.bias = 0.25f;
    shadows.angularDiameter = 1.5f;
    atlas.pointBias = 0.012f;
    post.ssaoDebug = true;
    lensFlare.sunDistance = 54321.0f;
    probe.enabled = false;
    probe.refresh = EnvironmentProbe::Refresh::Timed;
    probe.interval = 2.5f;
    lighting.tiled = false;
    lighting.use25D = false;
    volumetric.pointStrength = 3.25f;
    volumetric.samples = 27;
    sky.timeOfDay = 19.5f;
    sky.cloudCoverage = 0.73f;
    SceneRenderSettings settings{&shadows,  &atlas,      &post, &lensFlare, &probe,
                                 &lighting, &volumetric, &sky,  &resolution};

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene, &settings);
    const nlohmann::json& savedShadows = document["renderSettings"]["cascadeShadows"];
    CHECK(savedShadows.contains("quality"));
    CHECK(savedShadows.contains("angularDiameter"));
    CHECK(!savedShadows.contains("filterRadius"));
    shadows.distance = 1.0f;
    shadows.quality = 0;
    shadows.bias = 0.0f;
    shadows.angularDiameter = 0.0f;
    atlas.pointBias = 0.0f;
    post.ssaoDebug = false;
    lensFlare.sunDistance = 1.0f;
    probe.enabled = true;
    probe.refresh = EnvironmentProbe::Refresh::Manual;
    probe.interval = 0.1f;
    lighting.tiled = true;
    lighting.use25D = true;
    volumetric.pointStrength = 0.0f;
    volumetric.samples = 4;
    sky.timeOfDay = 0.0f;
    sky.cloudCoverage = 0.0f;
    resolution = RenderResolution();

    Scene restored;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, restored, result, &settings));
    CHECK(near(shadows.distance, 432.0f));
    CHECK(shadows.quality == 4);
    CHECK(near(shadows.bias, 0.25f));
    CHECK(near(shadows.angularDiameter, 1.5f));
    CHECK(near(atlas.pointBias, 0.012f));
    CHECK(post.ssaoDebug);
    CHECK(near(lensFlare.sunDistance, 54321.0f));
    CHECK(!probe.enabled);
    CHECK(probe.refresh == EnvironmentProbe::Refresh::Timed);
    CHECK(near(probe.interval, 2.5f));
    CHECK(!lighting.tiled);
    CHECK(!lighting.use25D);
    CHECK(near(volumetric.pointStrength, 3.25f));
    CHECK(volumetric.samples == 27);
    CHECK(near(sky.timeOfDay, 19.5f));
    CHECK(near(sky.cloudCoverage, 0.73f));
    CHECK(resolution.width == 1600);
    CHECK(resolution.height == 900);
    CHECK(near(resolution.scale, 0.75f));
}

void testSceneSerializerCameraRoundTrip()
{
    Scene scene;
    GameObject* object = scene.createGameObject("cam");
    Camera* camera = object->addComponent<Camera>();
    camera->setPerspective(75.0f, 1.5f, 0.5f, 500.0f);
    scene.setActiveCamera(camera);
    scene.update(0.016f);

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene);
    CHECK(document.dump(4) == serializer.toJson(scene).dump(4)); // deterministic

    Scene reloaded;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, reloaded, result));
    CHECK(result.success());

    GameObject* reObject = reloaded.findGameObject(object->id());
    CHECK(reObject != nullptr);
    Camera* reCamera = reObject ? reObject->getComponent<Camera>() : nullptr;
    CHECK(reCamera != nullptr);
    if (reCamera)
    {
        CHECK(reCamera->projectionMode() == CameraProjection::Perspective);
        CHECK(near(reCamera->fieldOfView(), 75.0f));
        CHECK(near(reCamera->aspect(), 1.5f));
        CHECK(near(reCamera->nearPlane(), 0.5f));
        CHECK(near(reCamera->farPlane(), 500.0f));
    }
    CHECK(reloaded.activeCamera() == reCamera);

    // Orthographic round-trips the other branch of the same field.
    Scene orthoScene;
    GameObject* orthoObject = orthoScene.createGameObject("ortho");
    Camera* orthoCamera = orthoObject->addComponent<Camera>();
    orthoCamera->setOrthographic(12.0f, 2.0f, 1.0f, 200.0f);
    orthoScene.update(0.016f);
    const nlohmann::json orthoDocument = serializer.toJson(orthoScene);
    Scene orthoReloaded;
    SceneLoadResult orthoResult;
    CHECK(serializer.fromJson(orthoDocument, orthoReloaded, orthoResult));
    Camera* reOrtho = orthoReloaded.findGameObject(orthoObject->id())->getComponent<Camera>();
    CHECK(reOrtho && reOrtho->projectionMode() == CameraProjection::Orthographic);
    CHECK(reOrtho && near(reOrtho->orthographicSize(), 12.0f));
}

void testSceneSerializerLightRoundTrip()
{
    Scene scene;
    GameObject* spotObject = scene.createGameObject("spot");
    SpotLight* spot = spotObject->addComponent<SpotLight>();
    spot->setColor(glm::vec3(1.0f, 0.5f, 0.25f));
    spot->setIntensity(3.0f);
    spot->setCastShadows(true);
    spot->setVolumetric(true);
    spot->setRange(15.0f);
    spot->setAngles(10.0f, 20.0f);

    GameObject* sunObject = scene.createGameObject("sun");
    DirectionalLight* sun = sunObject->addComponent<DirectionalLight>();
    sun->setColor(glm::vec3(1.0f));
    sun->setIntensity(2.0f);

    scene.update(0.016f);

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene);

    Scene reloaded;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, reloaded, result));
    CHECK(result.success());

    GameObject* reSpotObject = reloaded.findGameObject(spotObject->id());
    SpotLight* reSpot = reSpotObject ? reSpotObject->findComponent<SpotLight>() : nullptr;
    CHECK(reSpot != nullptr);
    if (reSpot)
    {
        CHECK(near(reSpot->color(), glm::vec3(1.0f, 0.5f, 0.25f)));
        CHECK(near(reSpot->intensity(), 3.0f));
        CHECK(reSpot->castsShadows());
        CHECK(reSpot->volumetric());
        CHECK(near(reSpot->range(), 15.0f));
        CHECK(near(reSpot->innerAngle(), 10.0f));
        CHECK(near(reSpot->outerAngle(), 20.0f));
    }

    GameObject* reSunObject = reloaded.findGameObject(sunObject->id());
    DirectionalLight* reSun =
        reSunObject ? reSunObject->findComponent<DirectionalLight>() : nullptr;
    CHECK(reSun != nullptr);
}

void testSceneSerializerComponentValidation()
{
    SceneSerializer serializer;

    auto baseDocument = [](nlohmann::json objects)
    {
        nlohmann::json document;
        document["format"] = "radion-scene";
        document["version"] = 1;
        document["scene"] = nlohmann::json::object();
        document["scene"]["objects"] = std::move(objects);
        return document;
    };

    auto objectWithComponents = [](u64 id, nlohmann::json components)
    {
        nlohmann::json object;
        object["id"] = id;
        object["parent"] = nullptr;
        object["name"] = "object";
        object["tag"] = "";
        object["flags"] = {{"active", true}, {"visible", true}, {"static", false}};
        object["transform"] = {{"position", {0.0, 0.0, 0.0}},
                               {"rotation", {0.0, 0.0, 0.0, 1.0}},
                               {"scale", {1.0, 1.0, 1.0}}};
        object["components"] = std::move(components);
        return object;
    };

    // Unknown component type: a warning, not a load failure.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json unknown;
        unknown["type"] = "SomeFutureComponent";
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(objectWithComponents(1, nlohmann::json::array({unknown})));
        CHECK(serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(result.success());
        bool sawWarning = false;
        for (const SceneDiagnostic& diagnostic : result.diagnostics)
            sawWarning |= diagnostic.severity == SceneDiagnosticSeverity::Warning;
        CHECK(sawWarning);
    }

    // Two components of the same type on one object.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json camera;
        camera["type"] = "Camera";
        camera["projection"] = "Perspective";
        camera["fieldOfView"] = 60.0;
        camera["orthographicSize"] = 10.0;
        camera["aspect"] = 1.7778;
        camera["near"] = 0.1;
        camera["far"] = 1000.0;
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(objectWithComponents(1, nlohmann::json::array({camera, camera})));
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
    }

    // Camera missing its projection field.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json camera;
        camera["type"] = "Camera";
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(objectWithComponents(1, nlohmann::json::array({camera})));
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
    }

    // MeshRenderer with no mesh field at all. A present null field is the
    // editor's intentional blank Mesh Instance and is therefore valid.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json renderer;
        renderer["type"] = "MeshRenderer";
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(objectWithComponents(1, nlohmann::json::array({renderer})));
        CHECK(!serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(!result.success());
    }

    // An explicit null mesh preserves an empty Mesh Instance.
    {
        Scene scene;
        SceneLoadResult result;
        nlohmann::json renderer;
        renderer["type"] = "MeshRenderer";
        renderer["mesh"] = nullptr;
        nlohmann::json objects = nlohmann::json::array();
        objects.push_back(objectWithComponents(1, nlohmann::json::array({renderer})));
        CHECK(serializer.fromJson(baseDocument(objects), scene, result));
        CHECK(result.success());
        GameObject* object = scene.findGameObject(1);
        MeshRenderer* meshRenderer = object ? object->findComponent<MeshRenderer>() : nullptr;
        CHECK(meshRenderer != nullptr);
        CHECK(meshRenderer && !meshRenderer->mesh().valid());
    }
}

void testAudioPlayerRoundTrip()
{
    Scene scene;
    GameObject* object = scene.createGameObject("speaker");
    AudioPlayer* player = object->addComponent<AudioPlayer>();
    CHECK(player != nullptr);
    if (!player)
        return;

    player->setSource("sounds/engine.ogg");
    player->setMusic(false);
    player->setAutoplay(true);
    player->setLoop(true);
    player->setVolume(0.75f);
    player->setPitch(1.25f);
    player->setPan(-0.5f);
    player->setMinDistance(3.0f);
    player->setMaxDistance(42.0f);
    player->setRolloff(2.0f);
    player->setSpatial(true);

    scene.update(0.016f);

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene);

    Scene reloaded;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, reloaded, result));
    CHECK(result.success());

    GameObject* reObject = reloaded.findGameObject(object->id());
    AudioPlayer* rePlayer = reObject ? reObject->getComponent<AudioPlayer>() : nullptr;
    CHECK(rePlayer != nullptr);
    if (!rePlayer)
        return;

    CHECK(rePlayer->source() == "sounds/engine.ogg");
    CHECK(!rePlayer->music());
    CHECK(rePlayer->autoplay());
    CHECK(rePlayer->loop());
    CHECK(std::fabs(rePlayer->volume() - 0.75f) < 1e-5f);
    CHECK(std::fabs(rePlayer->pitch() - 1.25f) < 1e-5f);
    CHECK(std::fabs(rePlayer->pan() + 0.5f) < 1e-5f);
    CHECK(rePlayer->spatial());
    CHECK(std::fabs(rePlayer->minDistance() - 3.0f) < 1e-5f);
    CHECK(std::fabs(rePlayer->maxDistance() - 42.0f) < 1e-5f);
    CHECK(std::fabs(rePlayer->rolloff() - 2.0f) < 1e-5f);

    // Music releases the loaded sound, so it has to be applied before the
    // source on read - otherwise the file read is thrown away.
    GameObject* musicObject = scene.createGameObject("radio");
    AudioPlayer* music = musicObject->addComponent<AudioPlayer>();
    music->setMusic(true);
    music->setSource("music/theme.ogg");
    scene.update(0.016f);

    Scene musicReloaded;
    SceneLoadResult musicResult;
    CHECK(serializer.fromJson(serializer.toJson(scene), musicReloaded, musicResult));
    GameObject* reMusicObject = musicReloaded.findGameObject(musicObject->id());
    AudioPlayer* reMusic = reMusicObject ? reMusicObject->getComponent<AudioPlayer>() : nullptr;
    CHECK(reMusic != nullptr);
    CHECK(reMusic && reMusic->music());
    CHECK(reMusic && reMusic->source() == "music/theme.ogg");
}

void testPhysicsBodyRoundTrip()
{
    Scene scene;
    GameObject* object = scene.createGameObject("crate");
    PhysicsBody* body = object->addComponent<PhysicsBody>();
    CHECK(body != nullptr);
    if (!body)
        return;

    body->setBodyType(Physics::BodyType::Kinematic);
    body->setBox(glm::vec3(0.5f, 0.75f, 1.0f));
    body->setMass(4.0f);
    body->setFriction(0.35f);
    body->setRestitution(0.2f);
    body->setCollisionGroup(2u);
    body->setCollisionMask(0xFFFFFFFEu);
    body->setEnabled(false);

    scene.update(0.016f);

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene);

    Scene reloaded;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, reloaded, result));
    CHECK(result.success());

    GameObject* reObject = reloaded.findGameObject(object->id());
    PhysicsBody* reBody = reObject ? reObject->getComponent<PhysicsBody>() : nullptr;
    CHECK(reBody != nullptr);
    if (!reBody)
        return;

    CHECK(reBody->bodyType() == Physics::BodyType::Kinematic);
    CHECK(reBody->shape() == PhysicsBodyShape::Box);
    CHECK(near(reBody->halfExtents(), glm::vec3(0.5f, 0.75f, 1.0f)));
    CHECK(near(reBody->mass(), 4.0f));
    CHECK(near(reBody->friction(), 0.35f));
    CHECK(near(reBody->restitution(), 0.2f));
    CHECK(reBody->collisionGroup() == 2u);
    CHECK(reBody->collisionMask() == 0xFFFFFFFEu);
    CHECK(!reBody->enabled());
}

// A dynamic sphere dropped above a static box falls under gravity and comes
// to rest on top of it, with the simulated pose written back into the
// GameObject every step - what actually makes the fall visible.
void testPhysicsBodyFalls()
{
    Scene scene;

    GameObject* floorObject = scene.createGameObject("floor");
    PhysicsBody* floor = floorObject->addComponent<PhysicsBody>();
    CHECK(floor != nullptr);
    if (!floor)
        return;
    floor->setBodyType(Physics::BodyType::Static);
    floor->setBox(glm::vec3(5.0f, 0.5f, 5.0f));

    GameObject* ballObject = scene.createGameObject("ball");
    PhysicsBody* ball = ballObject->addComponent<PhysicsBody>();
    CHECK(ball != nullptr);
    if (!ball)
        return;
    ball->setSphere(0.5f);
    ballObject->setPosition(glm::vec3(0.0f, 3.0f, 0.0f));

    for (int i = 0; i < 300; ++i)
        scene.update(1.0f / 60.0f);

    const glm::vec3 restPosition = ballObject->globalPosition();
    CHECK(near(restPosition.y, 1.0f, 0.1f));
    CHECK(near(restPosition.x, 0.0f, 0.2f));
    CHECK(near(restPosition.z, 0.0f, 0.2f));
    CHECK(glm::length(ball->velocity()) < 0.5f);
    // Static never gets written back - it must still be exactly where it
    // was placed.
    CHECK(near(floorObject->globalPosition(), glm::vec3(0.0f)));
}

// Disposing a GameObject mid-simulation must drop its body from the Scene's
// PhysicsWorld cleanly - no crash and no further updates once it is gone.
void testPhysicsBodyDestroyedMidSimulation()
{
    Scene scene;

    GameObject* floorObject = scene.createGameObject("floor2");
    PhysicsBody* floor = floorObject->addComponent<PhysicsBody>();
    CHECK(floor != nullptr);
    if (!floor)
        return;
    floor->setBodyType(Physics::BodyType::Static);
    floor->setBox(glm::vec3(5.0f, 0.5f, 5.0f));

    GameObject* ballObject = scene.createGameObject("fallingBall");
    PhysicsBody* ball = ballObject->addComponent<PhysicsBody>();
    CHECK(ball != nullptr);
    if (!ball)
        return;
    ball->setSphere(0.5f);
    ballObject->setPosition(glm::vec3(0.0f, 3.0f, 0.0f));

    for (int i = 0; i < 30; ++i)
        scene.update(1.0f / 60.0f);
    CHECK(scene.physicsBodies().size() == 2);

    ballObject->dispose();
    scene.update(1.0f / 60.0f);
    CHECK(scene.physicsBodies().size() == 1);
    CHECK(scene.gameObjectCount() == 1);

    for (int i = 0; i < 10; ++i)
        scene.update(1.0f / 60.0f);
    CHECK(scene.physicsBodies().size() == 1);
}

// The editor runs the same update loop while placing objects, so a Dynamic
// body must not fall until Play actually starts the simulation.
void testPhysicsBodyRunningInEditorFreezesSimulation()
{
    Scene scene;
    scene.setRunningInEditor(true);

    GameObject* floorObject = scene.createGameObject("floor3");
    PhysicsBody* floor = floorObject->addComponent<PhysicsBody>();
    CHECK(floor != nullptr);
    if (!floor)
        return;
    floor->setBodyType(Physics::BodyType::Static);
    floor->setBox(glm::vec3(5.0f, 0.5f, 5.0f));

    GameObject* ballObject = scene.createGameObject("editorBall");
    PhysicsBody* ball = ballObject->addComponent<PhysicsBody>();
    CHECK(ball != nullptr);
    if (!ball)
        return;
    ball->setSphere(0.5f);
    ballObject->setPosition(glm::vec3(0.0f, 3.0f, 0.0f));

    for (int i = 0; i < 60; ++i)
        scene.update(1.0f / 60.0f);
    CHECK(near(ballObject->globalPosition(), glm::vec3(0.0f, 3.0f, 0.0f)));

    scene.setRunningInEditor(false);
    for (int i = 0; i < 60; ++i)
        scene.update(1.0f / 60.0f);
    CHECK(ballObject->globalPosition().y < 2.9f);
}

// A Dynamic body placed by hand while the simulation is already running -
// setPosition() after addComponent(), a script teleporting it - must carry on
// from the new pose, not snap back to where the RigidBody last was.
void testPhysicsBodyTeleportInPlay()
{
    Scene scene;

    GameObject* floorObject = scene.createGameObject("floor4");
    PhysicsBody* floor = floorObject->addComponent<PhysicsBody>();
    CHECK(floor != nullptr);
    if (!floor)
        return;
    floor->setBodyType(Physics::BodyType::Static);
    floor->setBox(glm::vec3(5.0f, 0.5f, 5.0f));

    GameObject* ballObject = scene.createGameObject("teleportBall");
    PhysicsBody* ball = ballObject->addComponent<PhysicsBody>();
    CHECK(ball != nullptr);
    if (!ball)
        return;
    ball->setSphere(0.5f);
    ballObject->setPosition(glm::vec3(0.0f, 3.0f, 0.0f));

    for (int i = 0; i < 30; ++i)
        scene.update(1.0f / 60.0f);
    const f32 fallenY = ballObject->globalPosition().y;
    CHECK(fallenY < 3.0f && fallenY > 1.0f);

    ballObject->setPosition(glm::vec3(2.0f, 6.0f, -1.0f));
    scene.update(1.0f / 60.0f);
    const glm::vec3 afterTeleport = ballObject->globalPosition();
    CHECK(near(afterTeleport.x, 2.0f, 0.05f));
    CHECK(near(afterTeleport.z, -1.0f, 0.05f));
    CHECK(afterTeleport.y > 5.5f && afterTeleport.y <= 6.0f);
}

void testUiControlsRoundTrip()
{
    Scene scene;
    GameObject* canvasObject = scene.createGameObject("hud");
    UiCanvas* canvas = canvasObject->addComponent<UiCanvas>();
    CHECK(canvas != nullptr);

    GameObject* panelObject = scene.createGameObject("panel", canvasObject);
    UiPanel* panel = panelObject->addComponent<UiPanel>();
    CHECK(panel != nullptr);
    if (!panel)
        return;
    panel->setAnchors(glm::vec4(0.0f, 0.0f, 1.0f, 0.0f));
    panel->setOffsets(glm::vec4(4.0f, 8.0f, -4.0f, 96.0f));
    panel->setInteractive(true);
    panel->setLayer(3);
    panel->setColor(Color::fromRGBFloat(0.2f, 0.3f, 0.4f, 0.5f));

    GameObject* labelObject = scene.createGameObject("label", panelObject);
    UiLabel* label = labelObject->addComponent<UiLabel>();
    CHECK(label != nullptr);
    if (!label)
        return;
    label->setText("Score: 0");
    label->setFontSize(24.0f);
    label->setColor(Color::fromRGBFloat(1.0f, 0.8f, 0.2f));

    GameObject* buttonObject = scene.createGameObject("button", panelObject);
    UiButton* button = buttonObject->addComponent<UiButton>();
    CHECK(button != nullptr);
    if (!button)
        return;
    button->setText("Start");
    button->setRect(10.0f, 20.0f, 140.0f, 36.0f);

    GameObject* checkObject = scene.createGameObject("check", panelObject);
    UiCheckBox* checkBox = checkObject->addComponent<UiCheckBox>();
    CHECK(checkBox != nullptr);
    if (!checkBox)
        return;
    checkBox->setText("Fullscreen");
    checkBox->setChecked(true);

    GameObject* sliderObject = scene.createGameObject("slider", panelObject);
    UiSlider* slider = sliderObject->addComponent<UiSlider>();
    CHECK(slider != nullptr);
    if (!slider)
        return;
    slider->setRange(0.0f, 200.0f);
    slider->setValue(75.0f);

    scene.update(0.016f);

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene);

    Scene reloaded;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, reloaded, result));
    CHECK(result.success());

    GameObject* reCanvasObject = reloaded.findGameObject(canvasObject->id());
    CHECK(reCanvasObject != nullptr);
    CHECK(reCanvasObject && reCanvasObject->getComponent<UiCanvas>() != nullptr);

    GameObject* rePanelObject = reloaded.findGameObject(panelObject->id());
    UiPanel* rePanel = rePanelObject ? rePanelObject->getComponent<UiPanel>() : nullptr;
    CHECK(rePanel != nullptr);
    if (!rePanel)
        return;
    CHECK(rePanel->anchors() == glm::vec4(0.0f, 0.0f, 1.0f, 0.0f));
    CHECK(rePanel->offsets() == glm::vec4(4.0f, 8.0f, -4.0f, 96.0f));
    CHECK(rePanel->interactive());
    CHECK(rePanel->layer() == 3);
    CHECK(rePanel->color() == Color::fromRGBFloat(0.2f, 0.3f, 0.4f, 0.5f));

    GameObject* reLabelObject = reloaded.findGameObject(labelObject->id());
    UiLabel* reLabel = reLabelObject ? reLabelObject->getComponent<UiLabel>() : nullptr;
    CHECK(reLabel != nullptr);
    CHECK(reLabel && reLabel->text() == "Score: 0");
    CHECK(reLabel && std::fabs(reLabel->fontSize() - 24.0f) < 1e-5f);
    CHECK(reLabel && reLabel->color() == Color::fromRGBFloat(1.0f, 0.8f, 0.2f));

    GameObject* reButtonObject = reloaded.findGameObject(buttonObject->id());
    UiButton* reButton = reButtonObject ? reButtonObject->getComponent<UiButton>() : nullptr;
    CHECK(reButton != nullptr);
    CHECK(reButton && reButton->text() == "Start");
    CHECK(reButton && !reButton->consumeClick());
    if (reButton)
    {
        const glm::vec4 expectedOffsets(10.0f, 20.0f, 150.0f, 56.0f);
        CHECK(reButton->offsets() == expectedOffsets);
    }

    GameObject* reCheckObject = reloaded.findGameObject(checkObject->id());
    UiCheckBox* reCheckBox = reCheckObject ? reCheckObject->getComponent<UiCheckBox>() : nullptr;
    CHECK(reCheckBox != nullptr);
    CHECK(reCheckBox && reCheckBox->text() == "Fullscreen");
    CHECK(reCheckBox && reCheckBox->checked());

    GameObject* reSliderObject = reloaded.findGameObject(sliderObject->id());
    UiSlider* reSlider = reSliderObject ? reSliderObject->getComponent<UiSlider>() : nullptr;
    CHECK(reSlider != nullptr);
    CHECK(reSlider && std::fabs(reSlider->minimum() - 0.0f) < 1e-5f);
    CHECK(reSlider && std::fabs(reSlider->maximum() - 200.0f) < 1e-5f);
    CHECK(reSlider && std::fabs(reSlider->value() - 75.0f) < 1e-5f);

    // Hierarchy rides along with the rest of a scene save: every widget
    // is still parented under the panel, itself under the canvas.
    CHECK(rePanelObject && rePanelObject->parent() == reCanvasObject);
    CHECK(reLabelObject && reLabelObject->parent() == rePanelObject);
    CHECK(reButtonObject && reButtonObject->parent() == rePanelObject);
    CHECK(reCheckObject && reCheckObject->parent() == rePanelObject);
    CHECK(reSliderObject && reSliderObject->parent() == rePanelObject);
}

void testPrefabRoundTrip()
{
    Scene scene;
    GameObject* root = scene.createGameObject("turret");
    root->setPosition(glm::vec3(4.0f, 0.0f, -2.0f));
    root->setTag("enemy");
    GameObject* barrel = scene.createGameObject("barrel", root);
    barrel->setPosition(glm::vec3(0.0f, 1.5f, 0.0f));
    GameObject* muzzle = scene.createGameObject("muzzle", barrel);
    AudioPlayer* player = muzzle->addComponent<AudioPlayer>();
    player->setSource("sounds/shot.wav");
    player->setSpatial(true);

    scene.update(0.016f);

    Prefab prefab;
    prefab.saveFromObject(*root);
    CHECK(prefab.valid());
    CHECK(prefab.data()["scene"]["objects"].size() == 3);
    // The subtree root's parent is nulled, so the document parses without
    // needing an id that was never written into it.
    CHECK(prefab.data()["scene"]["objects"][0]["parent"].is_null());

    // Into the same scene it came from: fresh ids, originals untouched.
    SceneLoadResult result;
    GameObject* first = prefab.instantiate(scene, nullptr, result);
    scene.update(0.016f);
    CHECK(result.success());
    CHECK(first != nullptr);
    if (!first)
        return;
    CHECK(first != root);
    CHECK(first->id() != root->id());
    CHECK(first->name() == "turret");
    CHECK(first->tag() == "enemy");
    CHECK(first->childCount() == 1);
    CHECK(scene.gameObjectCount() == 6);
    // The scene's own root keeps its name - the document's belongs to the
    // subtree, not to what it is dropped into.
    CHECK(scene.root().name() != "turret");

    // A second instance is independent of the first.
    GameObject* second = prefab.instantiate(scene, nullptr, result);
    scene.update(0.016f);
    CHECK(second != nullptr);
    CHECK(second && second->id() != first->id());
    CHECK(scene.gameObjectCount() == 9);

    // Components ride along: the grandchild's AudioPlayer came back.
    GameObject* cloneBarrel = first->childCount() > 0 ? first->child(0) : nullptr;
    GameObject* cloneMuzzle = cloneBarrel && cloneBarrel->childCount() > 0 ? cloneBarrel->child(0)
                                                                          : nullptr;
    AudioPlayer* clonePlayer = cloneMuzzle ? cloneMuzzle->getComponent<AudioPlayer>() : nullptr;
    CHECK(clonePlayer != nullptr);
    CHECK(clonePlayer && clonePlayer->source() == "sounds/shot.wav");
    CHECK(clonePlayer && clonePlayer->spatial());

    // Under an explicit parent.
    GameObject* holder = scene.createGameObject("holder");
    scene.update(0.016f);
    GameObject* third = prefab.instantiate(scene, holder, result);
    scene.update(0.016f);
    CHECK(third != nullptr);
    CHECK(third && third->parent() == holder);

    // Into a scene that never saw the original.
    Scene other;
    SceneLoadResult otherResult;
    GameObject* elsewhere = prefab.instantiate(other, nullptr, otherResult);
    other.update(0.016f);
    CHECK(otherResult.success());
    CHECK(elsewhere != nullptr);
    CHECK(other.gameObjectCount() == 3);

    // Through disk and back.
    const std::filesystem::path file =
        std::filesystem::temp_directory_path() / "radion_prefab_test.rprefab";
    CHECK(prefab.saveToFile(file.string(), *root));
    Prefab reloaded;
    CHECK(reloaded.load(file.string()));
    CHECK(reloaded.valid());
    CHECK(reloaded.data() == prefab.data());
    Scene fromDisk;
    SceneLoadResult diskResult;
    GameObject* diskObject = reloaded.instantiate(fromDisk, nullptr, diskResult);
    fromDisk.update(0.016f);
    CHECK(diskResult.success());
    CHECK(diskObject != nullptr);
    CHECK(diskObject && diskObject->name() == "turret");
    CHECK(fromDisk.gameObjectCount() == 3);
    std::filesystem::remove(file);

    // An unloaded prefab instantiates nothing rather than half a subtree.
    Prefab empty;
    CHECK(!empty.valid());
    SceneLoadResult emptyResult;
    CHECK(empty.instantiate(scene, nullptr, emptyResult) == nullptr);

    // A document with no objects is an error, not a crash.
    Prefab malformed;
    nlohmann::json bad;
    bad["scene"]["objects"] = nlohmann::json::array();
    malformed.loadFromJson(bad);
    SceneLoadResult badResult;
    CHECK(malformed.instantiate(scene, nullptr, badResult) == nullptr);
    CHECK(!badResult.success());
}

void testTransforms()
{
    Scene scene;
    GameObject* parent = scene.createGameObject("parent");
    GameObject* child = scene.createGameObject("child", parent);
    scene.update(0.0f);

    parent->setPosition(glm::vec3(10.0f, 0.0f, 0.0f));
    parent->setScale(glm::vec3(2.0f));
    parent->yaw(90.0f);
    child->setPosition(glm::vec3(0.0f, 0.0f, -1.0f));

    CHECK(near(child->globalPosition(), glm::vec3(8.0f, 0.0f, 0.0f)));
    CHECK(near(child->globalScale(), glm::vec3(2.0f)));
    CHECK(near(child->forward(), glm::vec3(-1.0f, 0.0f, 0.0f)));

    child->setGlobalPosition(glm::vec3(10.0f, 2.0f, 0.0f));
    CHECK(near(child->globalPosition(), glm::vec3(10.0f, 2.0f, 0.0f)));
    CHECK(near(parent->distanceTo(*child), 2.0f));
    CHECK(near(parent->directionTo(*child), glm::vec3(0.0f, 1.0f, 0.0f)));

    child->lookAt(glm::vec3(10.0f, 2.0f, -10.0f));
    CHECK(near(child->forward(), glm::vec3(0.0f, 0.0f, -1.0f)));

    const glm::vec3 oldPosition = child->position();
    child->setPosition(glm::vec3(std::numeric_limits<f32>::quiet_NaN()));
    CHECK(near(child->position(), oldPosition));
    const glm::quat oldRotation = child->rotation();
    child->setRotation(glm::quat(0.0f, 0.0f, 0.0f, 0.0f));
    CHECK(near(glm::dot(child->rotation(), oldRotation), 1.0f));
}

void testSceneQueues()
{
    Scene scene;
    GameObject* mesh = scene.createGameObject("mesh");
    mesh->addComponent<MeshRenderer>();
    GameObject* cameraObject = scene.createGameObject("camera");
    Camera* camera = cameraObject->addComponent<Camera>();
    scene.setActiveCamera(camera);

    CHECK(scene.renderableCount() == 0);
    CHECK(scene.cameraCount() == 0);
    scene.update(0.25f);
    CHECK(near(scene.deltaTime(), 0.25f));
    CHECK(scene.renderableCount() == 1);
    CHECK(scene.cameraCount() == 1);
    CHECK(scene.activeCamera() == camera);
    CHECK(mesh->getComponent<MeshRenderer>() != nullptr);
    CHECK(mesh->addComponent<MeshRenderer>() == nullptr);

    CHECK(mesh->removeComponent<MeshRenderer>());
    CHECK(mesh->getComponent<MeshRenderer>() == nullptr);
    CHECK(scene.renderableCount() == 0);
    CHECK(mesh->addComponent<MeshRenderer>() != nullptr);
    CHECK(scene.renderableCount() == 1);

    CHECK(scene.remove(mesh));
    CHECK(scene.renderableCount() == 1);
    scene.update(0.0f);
    CHECK(scene.renderableCount() == 0);
    CHECK(mesh->parent() == nullptr);

    CHECK(scene.add(mesh));
    scene.update(0.0f);
    CHECK(scene.renderableCount() == 1);
    CHECK(mesh->parent() == &scene.root());

    CHECK(scene.destroy(cameraObject));
    scene.update(0.0f);
    CHECK(scene.cameraCount() == 0);
    CHECK(scene.activeCamera() == nullptr);

    CHECK(scene.destroy(mesh));
    scene.update(0.0f);
    CHECK(scene.renderableCount() == 0);
}

void testLights()
{
    Scene scene;
    GameObject* object = scene.createGameObject("light");
    SpotLight* light = object->addComponent<SpotLight>();

    light->setColor(glm::vec3(2.0f, -1.0f, 0.5f));
    light->setIntensity(4.0f);
    light->setRange(25.0f);
    light->setAngles(20.0f, 35.0f);
    light->setCastShadows(true);
    light->setVolumetric(true);

    CHECK(scene.lightCount() == 0);
    scene.update(0.0f);
    CHECK(scene.lightCount() == 1);
    CHECK(light->lightType() == LightType::Spot);
    CHECK(near(light->color(), glm::vec3(2.0f, 0.0f, 0.5f)));
    CHECK(near(light->intensity(), 4.0f));
    CHECK(near(light->range(), 25.0f));
    CHECK(near(light->innerAngle(), 20.0f));
    CHECK(near(light->outerAngle(), 35.0f));
    CHECK(light->castsShadows());
    CHECK(light->volumetric());

    CHECK(object->removeComponent<Light>());
    CHECK(scene.lightCount() == 0);
    CHECK(object->addComponent<PointLight>() != nullptr);
    CHECK(scene.lightCount() == 1);
    object->dispose();
    scene.update(0.0f);
    CHECK(scene.lightCount() == 0);

    RenderList list;
    RenderLight renderLight;
    renderLight.flags = RenderLightCastShadow | RenderLightVolumetric;
    for (usize i = 0; i < RenderList::MaxLights; ++i)
        CHECK(list.addLight(renderLight));
    CHECK(!list.addLight(renderLight));
    CHECK(list.lights().size() == RenderList::MaxLights);
    CHECK(list.stats().lights == RenderList::MaxLights);
    CHECK(list.stats().droppedLights == 1);
    list.clear();
    CHECK(list.lights().empty());
    CHECK(list.stats().lights == 0);
    CHECK(list.stats().droppedLights == 0);
}

void testShadowLayout()
{
    const DirectionalShadowRegion single = directionalShadowRegion(4096, 1, 0);
    CHECK(single.x == 0 && single.y == 0 && single.width == 4096 && single.height == 4096);
    const DirectionalShadowRegion split2 = directionalShadowRegion(4096, 2, 1);
    CHECK(split2.x == 0 && split2.y == 2048 && split2.width == 4096 && split2.height == 2048);
    const DirectionalShadowRegion split4 = directionalShadowRegion(4096, 4, 3);
    CHECK(split4.x == 2048 && split4.y == 2048 && split4.width == 2048 &&
          split4.height == 2048);

    CascadeShadowCalculator cascades;
    CHECK(cascades.settings.count == 4);
    CHECK(cascades.settings.resolution == 1024);
    CHECK(near(cascades.settings.filterRadiusWorld, 0.5f));
    CHECK(cascades.settings.filterTaps == 8);
    ShadowCamera camera;
    camera.view =
        glm::lookAt(glm::vec3(0.0f, 3.0f, 8.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    camera.nearPlane = 0.1f;
    CascadeShadowData data;
    CHECK(cascades.update(camera, glm::vec3(-0.4f, -1.0f, -0.2f), data));
    CHECK(data.count == 4);
    CHECK(data.splits[0] > camera.nearPlane);
    for (u32 cascade = 1; cascade < data.count; ++cascade)
        CHECK(data.splits[cascade] > data.splits[cascade - 1]);
    CHECK(data.halfExtents[1] > data.halfExtents[0]);
    for (u32 cascade = 0; cascade < data.count; ++cascade)
        CHECK(!data.casterPlanes[cascade].empty());
    for (u32 cascade = 0; cascade < MaxShadowCascades; ++cascade)
        for (u32 column = 0; column < 4; ++column)
            for (u32 row = 0; row < 4; ++row)
                CHECK(std::isfinite(data.viewProjection[cascade][column][row]));

    // Sub-texel camera translation must not make the shadow footprint crawl
    // in X/Y. Z may legitimately change continuously because it preserves
    // caster depth precision, so compare projected coordinates rather than
    // requiring the complete matrices to be bit-identical.
    ShadowCamera shifted = camera;
    const glm::vec3 delta(0.00001f, 0.0f, 0.0f);
    shifted.view = glm::lookAt(glm::vec3(0.0f, 3.0f, 8.0f) + delta, glm::vec3(0.0f) + delta,
                               glm::vec3(0.0f, 1.0f, 0.0f));
    CascadeShadowData shiftedData;
    CHECK(cascades.update(shifted, glm::vec3(-0.4f, -1.0f, -0.2f), shiftedData));
    for (u32 cascade = 0; cascade < data.count; ++cascade)
    {
        const glm::vec4 before = data.viewProjection[cascade] * glm::vec4(0, 0, 0, 1);
        const glm::vec4 after = shiftedData.viewProjection[cascade] * glm::vec4(0, 0, 0, 1);
        CHECK(near(before.x / before.w, after.x / after.w, 0.000001f));
        CHECK(near(before.y / before.w, after.y / after.w, 0.000001f));
    }

    std::vector<RenderLight> lights(3);
    lights[0].type = RenderLightType::Directional;
    lights[0].flags = RenderLightCastShadow;
    lights[1].type = RenderLightType::Point;
    lights[1].flags = RenderLightCastShadow;
    lights[1].position = glm::vec3(0.0f, 0.0f, 5.0f);
    lights[1].range = 10.0f;
    lights[2].type = RenderLightType::Spot;
    lights[2].flags = RenderLightCastShadow | RenderLightVolumetric;
    lights[2].position = glm::vec3(0.0f, 0.0f, 20.0f);
    lights[2].range = 5.0f;

    ShadowAtlasLayout atlas;
    atlas.settings.size = 4096;
    atlas.update(glm::vec3(0.0f), lights);
    CHECK(atlas.tiles().size() == 2);
    CHECK(atlas.tiles()[0].lightIndex == 1);
    CHECK(atlas.tiles()[0].faceCount == 6);
    CHECK(atlas.tiles()[0].size >= atlas.settings.minimumTileSize);
    for (const ShadowTile& tile : atlas.tiles())
    {
        CHECK(tile.x + tile.size * tile.faceCount <= atlas.settings.size);
        CHECK(tile.y + tile.size <= atlas.settings.size);
        CHECK(tile.lightIndex != 0);
    }
}

void testComponentEventsAndDispose()
{
    gScriptDestroyed = 0;
    Scene scene;
    GameObject* object = scene.createGameObject("scripted");
    GameObject* child = scene.createGameObject("child", object);
    child->addComponent<MeshRenderer>();
    TestScript* script = object->addComponent<TestScript>();

    CHECK(script->awakeCount == 1);
    CHECK(script->enableCount == 1);
    CHECK(script->startCount == 0);
    scene.update(0.016f);
    CHECK(script->startCount == 1);
    CHECK(script->updateCount == 1);
    CHECK(script->lateUpdateCount == 1);

    script->setActive(false);
    CHECK(script->disableCount == 1);
    scene.update(0.016f);
    CHECK(script->updateCount == 1);
    script->setActive(true);
    CHECK(script->enableCount == 2);
    scene.update(0.016f);
    CHECK(script->startCount == 1);
    CHECK(script->updateCount == 2);

    object->dispose();
    CHECK(object->disposed());
    scene.update(0.0f);
    CHECK(scene.gameObjectCount() == 0);
    CHECK(scene.renderableCount() == 0);
    CHECK(gScriptDestroyed == 1);
}

void testActionRunner()
{
    Scene scene;
    GameObject* object = scene.createGameObject("action");
    ActionRunner* actions = object->addComponent<ActionRunner>();
    actions->moveTo(glm::vec3(10.0f, 0.0f, 0.0f), 1.0f)
        .wait(0.5f)
        .moveBy(glm::vec3(0.0f, 2.0f, 0.0f), 0.5f)
        .dispose();

    scene.update(0.5f);
    CHECK(near(object->position(), glm::vec3(5.0f, 0.0f, 0.0f)));
    scene.update(0.75f);
    CHECK(near(object->position(), glm::vec3(10.0f, 0.0f, 0.0f)));
    scene.update(0.5f);
    CHECK(near(object->position(), glm::vec3(10.0f, 1.0f, 0.0f)));
    scene.update(0.5f);
    CHECK(scene.gameObjectCount() == 0);

    GameObject* bolt = scene.createGameObject("bolt");
    ActionRunner* flight = bolt->addComponent<ActionRunner>();
    flight->projectile({glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(0.0f, 0.0f, -2.0f)}, 10.0f, 2.0f);
    scene.update(1.0f);
    CHECK(near(bolt->position(), glm::vec3(1.0f, 2.0f, -7.0f)));
    scene.update(1.0f);
    CHECK(scene.gameObjectCount() == 0);
}

void testRibbonTrail()
{
    TrailDraws().clear();
    Scene scene;
    GameObject* effect = scene.createGameObject("trail");
    GameObject* base = scene.createGameObject("base", effect);
    GameObject* tip = scene.createGameObject("tip", effect);
    tip->setPosition(glm::vec3(0.0f, 0.0f, 1.0f));
    RibbonTrail* trail = effect->addComponent<RibbonTrail>();
    CHECK(trail->setBlade(base, tip));
    trail->setMinDistance(0.1f);
    trail->setLifetime(0.25f);
    trail->setEmitting(true);
    scene.update(0.0f);
    CHECK(trail->sampleCount() == 1);

    tip->setPosition(glm::vec3(0.0f, 1.0f, 1.0f));
    scene.update(0.0f);
    CHECK(trail->sampleCount() > 2);
    CHECK(!TrailDraws().commands().empty());

    trail->setEmitting(false);
    scene.update(0.3f);
    CHECK(trail->sampleCount() == 0);
    TrailDraws().clear();
}

void testRoadPoints()
{
    Scene scene;
    GameObject* roadObject = scene.createGameObject("road");
    GameObject* first = scene.createGameObject("first", roadObject);
    GameObject* second = scene.createGameObject("second", roadObject);
    Road* road = roadObject->addComponent<Road>();
    CHECK(road->addPoint(first, 4.0f));
    CHECK(road->addPoint(second, 8.0f));
    CHECK(!road->addPoint(first, 4.0f));
    CHECK(road->pointCount() == 2);
    CHECK(road->point(0) == first);
    CHECK(near(road->pointWidth(1), 8.0f));
    GameObject* front = scene.createGameObject("front", roadObject);
    CHECK(road->insertPoint(0, front, 3.0f));
    CHECK(road->point(0) == front);
    CHECK(near(road->pointWidth(0), 3.0f));
    CHECK(!road->insertPoint(8, front, 3.0f));
    road->setPointWidth(1, 10.0f);
    CHECK(near(road->pointWidth(1), 10.0f));
    CHECK(road->removePoint(first));
    CHECK(!road->removePoint(first));
    CHECK(road->pointCount() == 2);
    road->clearPoints();
    CHECK(road->pointCount() == 0);
}

void testRoadSplineRoundTrip()
{
    Scene scene;
    GameObject* roadObject = scene.createGameObject("road");
    Road* road = roadObject->addComponent<Road>();
    GameObject* a = scene.createGameObject("a", roadObject);
    GameObject* b = scene.createGameObject("b", roadObject);
    GameObject* c = scene.createGameObject("c", roadObject);
    a->setGlobalPosition(glm::vec3(-4.0f, 1.0f, 2.0f));
    b->setGlobalPosition(glm::vec3(3.0f, 2.0f, -5.0f));
    c->setGlobalPosition(glm::vec3(9.0f, 0.5f, -8.0f));
    CHECK(road->addPoint(a, 4.0f));
    CHECK(road->addPoint(b, 7.5f));
    CHECK(road->addPoint(c, 3.0f));
    road->setSubdivisions(19);
    road->setTextureRepeat(8.5f);
    road->setSurfaceOffset(0.12f);
    road->setConformTerrain(false);

    const char* path = "scene_test_road.rroad";
    CHECK(road->saveSpline(path));
    road->setPointWidth(0, 99.0f);
    CHECK(road->loadSpline(path, scene));
    CHECK(road->pointCount() == 3);
    CHECK(near(road->point(0)->globalPosition(), glm::vec3(-4.0f, 1.0f, 2.0f)));
    CHECK(near(road->point(1)->globalPosition(), glm::vec3(3.0f, 2.0f, -5.0f)));
    CHECK(near(road->point(2)->globalPosition(), glm::vec3(9.0f, 0.5f, -8.0f)));
    CHECK(near(road->pointWidth(0), 4.0f));
    CHECK(near(road->pointWidth(1), 7.5f));
    CHECK(near(road->pointWidth(2), 3.0f));
    std::remove(path);
}

void testCameraAndRay()
{
    Scene scene;
    GameObject* cameraObject = scene.createGameObject("camera");
    Camera* camera = cameraObject->addComponent<Camera>();
    scene.setActiveCamera(camera);
    scene.update(0.0f);

    camera->setPerspective(60.0f, 2.0f, 0.1f, 100.0f);
    cameraObject->setPosition(glm::vec3(0.0f, 0.0f, 5.0f));
    cameraObject->lookAt(glm::vec3(0.0f));

    const FloatRect viewport(100.0f, 50.0f, 800.0f, 400.0f);
    const Ray center = camera->rayFromMouse(500.0f, 250.0f, viewport);
    CHECK(near(center.direction, cameraObject->forward()));
    CHECK(near(glm::length(center.direction), 1.0f));
    CHECK(center.origin.z < cameraObject->globalPosition().z);

    camera->setOrthographic(10.0f, 2.0f, 0.1f, 100.0f);
    const Ray left = camera->rayFromMouse(100.0f, 250.0f, viewport);
    const Ray right = camera->rayFromMouse(900.0f, 250.0f, viewport);
    CHECK(near(left.direction, right.direction));
    CHECK(left.origin.x < right.origin.x);

    const f32 oldAspect = camera->aspect();
    camera->setAspect(0.0f);
    CHECK(near(camera->aspect(), oldAspect));
}

void testDestroyBranch()
{
    Scene scene;
    GameObject* branch = scene.createGameObject("branch");
    GameObject* first = scene.createGameObject("first", branch);
    first->addComponent<MeshRenderer>();
    GameObject* nested = scene.createGameObject("nested", branch);
    GameObject* second = scene.createGameObject("second", nested);
    second->addComponent<MeshRenderer>();
    GameObject* cameraObject = scene.createGameObject("camera", nested);
    Camera* camera = cameraObject->addComponent<Camera>();
    scene.setActiveCamera(camera);
    scene.update(0.0f);

    CHECK(first && second && camera);
    CHECK(scene.renderableCount() == 2);
    CHECK(scene.cameraCount() == 1);
    CHECK(scene.destroy(branch));
    scene.update(0.0f);
    CHECK(scene.renderableCount() == 0);
    CHECK(scene.cameraCount() == 0);
    CHECK(scene.activeCamera() == nullptr);
    CHECK(scene.root().childCount() == 0);
}

void testRenderListRejectsInvalidPipelines()
{
    Mesh mesh;
    mesh.bounds.expand(glm::vec3(-0.5f));
    mesh.bounds.expand(glm::vec3(0.5f));

    SubMesh submesh;
    submesh.bounds = mesh.bounds;
    mesh.submeshes.push_back(submesh);
    mesh.materials.emplace_back();

    MeshHandle meshHandle;
    meshHandle.index = 7;
    meshHandle.generation = 1;

    RenderList list;
    list.setCamera(glm::mat4(1.0f), glm::vec3(0.0f));
    CHECK(list.submit(meshHandle, mesh, glm::mat4(1.0f)) == 0);
    CHECK(list.stats().packets == 0);

    mesh.materials[0].pipeline.index = 3;
    mesh.materials[0].pipeline.generation = 1;
    CHECK(list.submit(meshHandle, mesh, glm::mat4(1.0f)) == 1);
    CHECK(list.packets(RenderCategory::Opaque).size() == 1);
    CHECK(list.instance(0).pipeline == mesh.materials[0].pipeline);

    const PipelineHandle firstPipeline = mesh.materials[0].pipeline;
    mesh.materials[0].pipeline.index = 4;
    CHECK(list.submit(meshHandle, mesh, glm::mat4(1.0f)) == 1);
    CHECK(list.instance(0).pipeline == firstPipeline);
    CHECK(list.instance(1).pipeline == mesh.materials[0].pipeline);

    list.clear();
    mesh.materials[0].flags |= MaterialAlphaTest;
    CHECK(list.submit(meshHandle, mesh, glm::mat4(1.0f)) == 1);
    CHECK(list.packets(RenderCategory::AlphaTest).size() == 1);

    list.clear();
    mesh.materials[0].flags &= ~MaterialAlphaTest;
    mesh.materials[0].blend = BlendMode::Alpha;
    CHECK(list.submit(meshHandle, mesh, glm::mat4(1.0f)) == 1);
    CHECK(list.packets(RenderCategory::Transparent).size() == 1);

    list.clear();
    mesh.materials[0].flags |= MaterialRefraction;
    CHECK(list.submit(meshHandle, mesh, glm::mat4(1.0f)) == 1);
    CHECK(list.packets(RenderCategory::Refraction).size() == 1);
}

void testProfilerHistory()
{
    Profiler& profiler = Profiler::getSingleton();
    profiler.beginFrame();
    CHECK(profiler.begin("Test scope"));
    profiler.end();
    profiler.endFrame();

    CHECK(profiler.sampleCount() >= 2);
    CHECK(profiler.frameMilliseconds() >= 0.0f);
    bool found = false;
    for (u32 i = 0; i < profiler.sampleCount(); ++i)
    {
        const ProfileSample& sample = profiler.samples()[i];
        if (sample.name == "Test scope")
        {
            found = true;
            CHECK(sample.historyCount == 1);
            CHECK(sample.average >= 0.0f);
            CHECK(sample.maximum >= sample.average);
        }
    }
    CHECK(found);
}

void testPresentationFit()
{
    PresentationSettings settings;
    settings.aspect = AspectMode::Ratio4x3;
    settings.fit = FitMode::Letterbox;
    PresentationView view = computePresentation(settings, 1920, 1080);
    CHECK(near(view.aspect, 4.0f / 3.0f));
    CHECK(view.rect.x == 240);
    CHECK(view.rect.y == 0);
    CHECK(view.rect.width == 1440);
    CHECK(view.rect.height == 1080);

    settings.fit = FitMode::Crop;
    view = computePresentation(settings, 1920, 1080);
    CHECK(view.rect.x == 0);
    CHECK(view.rect.y == -180);
    CHECK(view.rect.width == 1920);
    CHECK(view.rect.height == 1440);

    settings.aspect = AspectMode::Ratio16x9;
    settings.fit = FitMode::Stretch;
    view = computePresentation(settings, 800, 600);
    CHECK(view.rect.x == 0 && view.rect.y == 0);
    CHECK(view.rect.width == 800 && view.rect.height == 600);

    settings.aspect = AspectMode::Window;
    settings.fit = FitMode::Letterbox;
    view = computePresentation(settings, 800, 600);
    CHECK(near(view.aspect, 4.0f / 3.0f));
    CHECK(view.rect.width == 800 && view.rect.height == 600);
}

void testDebugDrawGeometry()
{
    DebugDraw3D& debug = DebugDraw();
    debug.clear();

    AABB bounds;
    bounds.expand(glm::vec3(-1.0f));
    bounds.expand(glm::vec3(1.0f));
    debug.box(bounds);
    CHECK(debug.lines().size() == 12);

    debug.axis(glm::mat4(1.0f));
    CHECK(debug.lines().size() == 15);

    debug.line(glm::vec3(0.0f), glm::vec3(1.0f), Color::Cyan, false);
    CHECK(!debug.lines().back().depthTest);
    debug.clear();
    debug.box(bounds);

    debug.grid(0.0f, 2, 1.0f);
    CHECK(debug.lines().size() == 22);
    debug.triangle(glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                   Color::White, true);
    CHECK(debug.triangles().size() == 1);
    debug.pickedTriangle(glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                         glm::vec3(0.25f, 0.25f, 0.0f));
    CHECK(debug.triangles().size() == 2);
    CHECK(debug.lines().size() == 26);
    MeshHandle mesh;
    mesh.index = 1;
    mesh.generation = 1;
    debug.meshVectors(mesh, glm::mat4(1.0f), 0.25f, DebugVectorsNormal | DebugVectorsTangent);
    CHECK(debug.meshVectorCommands().size() == 1);
    debug.outline(mesh, glm::mat4(1.0f), Color::Orange, 3.0f);
    CHECK(debug.outlineCommands().size() == 1);
    debug.outline(MeshHandle(), glm::mat4(1.0f));
    debug.outline(mesh, glm::mat4(1.0f), Color::Orange, 0.0f);
    CHECK(debug.outlineCommands().size() == 1);
    CHECK(!debug.empty());
    debug.clear();
    CHECK(debug.empty());
}

// The generator runs on the CPU and touches no GPU state, so the whole of it
// is checkable here: every preset has to come out as a closed, finite mesh
// with both submeshes present and its indices inside the vertex range.
void testProceduralTrees()
{
    CHECK(Assets().treePresetCount() > 0);

    for (u32 p = 0; p < Assets().treePresetCount(); ++p)
    {
        const TreePreset& preset = Assets().treePreset(p);

        MeshData tree;
        Assets().buildTree(tree, preset.params);

        CHECK(tree.submeshes.size() == 2);
        CHECK(tree.positions.size() > 0);
        CHECK(tree.normals.size() == tree.positions.size());
        CHECK(tree.uvs.size() == tree.positions.size());
        CHECK(tree.tangents.size() == tree.positions.size());
        CHECK(tree.indices.size() % 3 == 0);

        // Bark first, twigs after, and between them they have to account for
        // every index - a gap would mean geometry that never gets drawn.
        CHECK(tree.submeshes[0].indexOffset == 0);
        CHECK(tree.submeshes[0].indexCount > 0);
        CHECK(tree.submeshes[1].indexCount > 0);
        CHECK(tree.submeshes[1].indexOffset == tree.submeshes[0].indexCount);
        CHECK(tree.submeshes[1].indexOffset + tree.submeshes[1].indexCount == tree.indices.size());

        bool indicesInRange = true;
        for (u32 index : tree.indices)
            indicesInRange = indicesInRange && index < tree.positions.size();
        CHECK(indicesInRange);

        bool finite = true;
        for (usize i = 0; i < tree.positions.size(); ++i)
        {
            finite = finite && std::isfinite(tree.positions[i].x) &&
                     std::isfinite(tree.positions[i].y) && std::isfinite(tree.positions[i].z) &&
                     std::isfinite(tree.uvs[i].x) && std::isfinite(tree.uvs[i].y) &&
                     near(glm::length(tree.normals[i]), 1.0f, 1e-3f) &&
                     near(glm::length(glm::vec3(tree.tangents[i])), 1.0f, 1e-3f);
        }
        CHECK(finite);

        // Trunk base at the origin, growing up: a tree that came out upside
        // down or off-centre would plant wrong on the terrain.
        CHECK(tree.bounds.max.y > tree.bounds.min.y);
        CHECK(near(tree.bounds.min.y, 0.0f, 0.5f));
    }

    // Same seed, same tree; a different seed, a different one. The generator
    // holds no state between calls.
    MeshData first;
    MeshData second;
    TreeParams params = Assets().treePreset(0).params;
    Assets().buildTree(first, params);
    Assets().buildTree(second, params);
    CHECK(first.positions.size() == second.positions.size());
    CHECK(first.positions == second.positions);

    params.seed += 1;
    MeshData other;
    Assets().buildTree(other, params);
    CHECK(other.positions != first.positions);

    // Odd or tiny segment counts are corrected, not honoured: the fork builder
    // walks segments/2 and would leave holes.
    MeshData odd;
    params = Assets().treePreset(0).params;
    params.segments = 5;
    Assets().buildTree(odd, params);
    CHECK(odd.indices.size() % 3 == 0);
    CHECK(odd.positions.size() > 0);

    // Levels below 1 still has to produce a trunk rather than an empty mesh.
    MeshData minimal;
    params = Assets().treePreset(0).params;
    params.levels = 0;
    params.trunkSteps = 0;
    Assets().buildTree(minimal, params);
    CHECK(minimal.positions.size() > 0);

    u32 state = 1234u;
    for (u32 i = 0; i < 8; ++i)
    {
        const TreeParams random = Assets().randomTreeParams(state);
        CHECK(random.clumpMin < random.clumpMax);
        CHECK(random.lengthFalloffFactor < 1.0f);
        CHECK(random.segments % 2 == 0);

        MeshData mesh;
        Assets().buildTree(mesh, random);
        CHECK(mesh.positions.size() > 0);
        CHECK(mesh.indices.size() % 3 == 0);
    }
}

// Species need a GPU to upload to, so what is checkable here is the part that
// runs before one: the component registers, the ranges clamp, and nothing
// plants against a library that is still empty.
void testForestWithoutSpecies()
{
    Scene scene;
    GameObject* object = scene.createGameObject("forest");
    Forest* forest = object->addComponent<Forest>();
    CHECK(forest != nullptr);
    CHECK(object->getComponent<Forest>() == forest);
    scene.update(0.0f);
    CHECK(scene.renderableCount() == 1);

    CHECK(forest->speciesCount() == 0);
    CHECK(forest->count() == 0);
    CHECK(forest->visibleCount() == 0);
    CHECK(!forest->plant(glm::vec3(0.0f), 0));
    CHECK(forest->paint(glm::vec3(0.0f), 10.0f, 100) == 0);
    CHECK(!forest->rebuildSpecies(0, TreeParams(), 10.0f));
    CHECK(!forest->speciesMesh(0).valid());

    forest->setDrawDistance(-5.0f);
    CHECK(near(forest->drawDistance(), 0.0f));
    forest->setDrawDistance(250.0f);
    CHECK(near(forest->drawDistance(), 250.0f));

    // An inverted range collapses to its minimum rather than producing
    // negative scales further down.
    forest->setScaleRange(2.0f, 1.0f);
    forest->setSeed(99u);
    CHECK(forest->paint(glm::vec3(0.0f), 10.0f, 10) == 0);

    CHECK(scene.destroy(object));
    scene.update(0.0f);
}

void testParticleEffect()
{
    Scene scene;
    GameObject* object = scene.createGameObject("effect");
    object->setGlobalPosition(glm::vec3(1.0f, 2.0f, 3.0f));
    ParticleEffect* effect = object->addComponent<ParticleEffect>();
    CHECK(effect != nullptr);
    CHECK(object->getComponent<ParticleEffect>() == effect);

    effect->setEmitter(ParticleEffect::presetSmoke());
    CHECK(effect->mode() == ParticleEffectMode::OneShot);
    CHECK(!effect->isPlaying());

    effect->setMode(ParticleEffectMode::Continuous);
    CHECK(effect->mode() == ParticleEffectMode::Continuous);

    effect->setMode(ParticleEffectMode::OneShot);
    effect->setBurstCount(64);
    CHECK(effect->burstCount() == 64);

    effect->play();
    CHECK(effect->isPlaying());
    scene.update(0.0f);
    CHECK(effect->isPlaying());

    // Without a GPU-driven update the particles never really spawn, but the
    // one-shot timer still advances. After lifeMax it reports finished.
    //
    // Auto-destroy is off for this part: onUpdate() disposes the owner the
    // moment isFinished() goes true, and Scene::update()'s own flushChanges()
    // at the end of that same call physically deletes it - no grace frame,
    // by design. Checking effect/object through either pointer straight
    // after would be a use-after-free, not something that happens to work.
    effect->setAutoDestroy(false);
    scene.update(effect->emitter().lifeMax * 1.1f);
    CHECK(effect->isFinished());
    CHECK(!object->disposed());

    // mAliveTimer only ever grows, so isFinished() stays true - re-enabling
    // auto-destroy and running one more update is what actually disposes the
    // owner. gameObjectCount(), not the (about to be freed) pointers, is
    // what verifies it.
    effect->setAutoDestroy(true);
    const usize objectCountBeforeDestroy = scene.gameObjectCount();
    scene.update(0.0f);
    CHECK(scene.gameObjectCount() == objectCountBeforeDestroy - 1);

    // Continuous effect: should not auto-destroy while playing.
    GameObject* persistent = scene.createGameObject("smoke");
    persistent->setGlobalPosition(glm::vec3(0.0f));
    ParticleEffect* smoke = persistent->addComponent<ParticleEffect>();
    smoke->setMode(ParticleEffectMode::Continuous);
    smoke->setEmitter(ParticleEffect::presetSmoke());
    smoke->play();
    scene.update(1.0f);
    CHECK(!persistent->disposed());
    smoke->stop();

    // Pool: one-shot spawned through the pool stays active, then gets reclaimed.
    ParticleSystem::Emitter burst = ParticleEffect::presetExplosion();
    ParticleEffect* spawned = ParticleEffectPool::getSingleton().spawn(burst, 32, glm::vec3(0.0f));
    CHECK(spawned != nullptr);
    CHECK(spawned->isPlaying());
    CHECK(ParticleEffectPool::getSingleton().activeCount() >= 1);

    scene.update(burst.lifeMax * 1.1f);
    ParticleEffectPool::getSingleton().reclaim();
    CHECK(ParticleEffectPool::getSingleton().activeCount() == 0);
    CHECK(ParticleEffectPool::getSingleton().availableCount() >= 1);
}

void testComponentSelfRemovalCompactsLazily()
{
    Scene scene;
    GameObject* object = scene.createGameObject("component_removal");
    CHECK(object->addComponent<SelfRemovingComponent>() != nullptr);
    CountingComponent* counter = object->addComponent<CountingComponent>();
    CHECK(counter != nullptr);

    scene.update(1.0f / 60.0f);
    CHECK(object->getComponent<SelfRemovingComponent>() == nullptr);
    CHECK(counter->updates == 1);

    scene.update(1.0f / 60.0f);
    CHECK(counter->updates == 2);
}

void testStaticMeshLoad()
{
    const std::filesystem::path bistroFile = "/media/projectos/assets/bistro/extrior.rstm";
    std::error_code error;
    if (!std::filesystem::is_regular_file(bistroFile, error))
    {
        std::fprintf(stderr, "SceneTests: skipping optional Bistro fixture\n");
        return;
    }

    FileSystem& files = FileSystem::getSingleton();
    files.addSearchPath(bistroFile.parent_path().string());

    MeshData bistro;
    CHECK(Assets().importMesh("extrior.rstm", bistro));
    CHECK(bistro.positions.size() == 3112792);
    CHECK(bistro.indices.size() == 8496360);
    CHECK(bistro.submeshes.size() == 132);
    CHECK(bistro.materials.size() == 132);
}

void testMaterialSaveParserRoundTrip()
{
    Material material;
    material.name = "mirror \"test\"";
    material.nameHash = 0x5a17u;
    material.flags = MaterialCastShadow | MaterialAnimated | MaterialLit | MaterialLandscape |
                     MaterialMirror | MaterialParallax | MaterialMetallicRoughnessMap;
    material.blend = BlendMode::PremultipliedAlpha;
    material.cull = CullMode::Front;
    material.params.baseColor = glm::vec4(0.11f, 0.22f, 0.33f, 0.44f);
    material.params.emissive = glm::vec4(1.1f, 2.2f, 3.3f, 4.4f);
    material.params.surface = glm::vec4(0.12f, 0.34f, 0.56f, 0.78f);
    material.params.uvTransform = glm::vec4(2.0f, 3.0f, 0.25f, -0.5f);
    material.params.uvAnim = glm::vec4(0.1f, -0.2f, 0.3f, 0.4f);
    material.params.sequence = glm::vec4(7.0f, 24.0f, 1.0f, 0.5f);
    material.params.custom0 = glm::vec4(0.21f, 0.52f, 0.013f, 1.75f);
    material.params.custom1 = glm::vec4(9000.0f, 0.047f, 0.63f, 8.5f);
    const MaterialSlot editableSlots[] = {SlotAlbedo, SlotNormal, SlotSurface, SlotEmissive,
                                          SlotHeight};
    for (MaterialSlot slot : editableSlots)
    {
        material.textures[slot].source = TextureSource::Static;
        material.textures[slot].file = "textures\\slot " + std::to_string(slot) + ".png";
    }
    material.textures[SlotAlbedo].file = "textures\\quoted \"albedo\".png";
    material.animCount = 1;
    material.anims[0].field = 6;
    material.anims[0].mask = 0xF;
    material.anims[0].curve = Curve::PingPong;
    material.anims[0].speed = 2.5f;
    material.anims[0].phase = 0.75f;
    material.anims[0].min = glm::vec4(-1.0f);
    material.anims[0].max = glm::vec4(3.0f);

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "radion_material_roundtrip_test.material";
    CHECK(MaterialManager::getSingleton().save(path.string(), {material}));

    std::vector<MaterialDefinition> definitions;
    MaterialParseError error;
    CHECK(MaterialParser::parseFile(path.string(), definitions, &error));
    CHECK(definitions.size() == 1);
    if (definitions.size() == 1)
    {
        const Material& restored = definitions[0].material;
        CHECK(definitions[0].name == material.name);
        CHECK(restored.flags == material.flags);
        CHECK(restored.blend == material.blend);
        CHECK(restored.cull == material.cull);
        CHECK(restored.params.baseColor == material.params.baseColor);
        CHECK(restored.params.emissive == material.params.emissive);
        CHECK(restored.params.surface == material.params.surface);
        CHECK(restored.params.uvTransform == material.params.uvTransform);
        CHECK(restored.params.uvAnim == material.params.uvAnim);
        CHECK(restored.params.sequence == material.params.sequence);
        CHECK(restored.params.custom0 == material.params.custom0);
        CHECK(restored.params.custom1 == material.params.custom1);
        CHECK(restored.animCount == material.animCount);
        CHECK(restored.anims[0].field == material.anims[0].field);
        CHECK(restored.anims[0].mask == material.anims[0].mask);
        CHECK(restored.anims[0].curve == material.anims[0].curve);
        CHECK(restored.anims[0].speed == material.anims[0].speed);
        CHECK(restored.anims[0].phase == material.anims[0].phase);
        CHECK(restored.anims[0].min == material.anims[0].min);
        CHECK(restored.anims[0].max == material.anims[0].max);
        CHECK(definitions[0].textures.size() == sizeof(editableSlots) / sizeof(editableSlots[0]));
        for (const MaterialTextureSource& texture : definitions[0].textures)
        {
            CHECK(texture.file == material.textures[texture.slot].file);
            CHECK(texture.filter == Filter::Anisotropic);
            CHECK(texture.wrap == Wrap::Repeat);
            CHECK(texture.generateMips);
        }
    }
    std::error_code removeError;

    CHECK(
        !MaterialParser::parse("material bad { properties { roughness - } }", definitions, &error));
    CHECK(!MaterialParser::parse("material \"unterminated", definitions, &error));
    // A duplicate name is survivable, unlike the syntax errors around it: the
    // file parses, the first occurrence stands and the rest are skipped. One
    // bad entry taking a Bistro-sized sidecar's other thousands with it is the
    // worse outcome, so MaterialParser warns and carries on.
    CHECK(MaterialParser::parse("material same {} material same {}", definitions, &error));
    CHECK(definitions.size() == 1);
    CHECK(!MaterialParser::parse("material bad { textures { texture albedo { type Static } } }",
                                 definitions, &error));
    CHECK(!MaterialParser::parse("material bad { textures { texture albedo { type Sequence } } }",
                                 definitions, &error));
    CHECK(!MaterialParser::parse(
        "material bad { textures { texture albedo { type RenderTarget } } }", definitions, &error));
    CHECK(!MaterialParser::parse("material bad { animations { pulse { type SineWave speed 1 } } }",
                                 definitions, &error));

    // Exercise replacement of an existing sidecar too: save uses a temporary
    // file and backup, but neither may remain after a successful commit.
    material.params.custom0.w = 2.25f;
    CHECK(MaterialManager::getSingleton().save(path.string(), {material}));
    CHECK(MaterialParser::parseFile(path.string(), definitions, &error));
    CHECK(definitions.size() == 1);
    if (definitions.size() == 1)
        CHECK(definitions[0].material.params.custom0.w == material.params.custom0.w);
    CHECK(!std::filesystem::exists(path.string() + ".tmp"));
    CHECK(!std::filesystem::exists(path.string() + ".bak"));

    std::filesystem::remove(path, removeError);
}

void testMeshRendererMaterialOwnershipState()
{
    Scene scene;
    MeshRenderer* renderer = scene.createGameObject("material owner")->addComponent<MeshRenderer>();
    Material material;
    material.name = "owned override";
    renderer->setMaterialOverride(0, material);
    CHECK(renderer->materialOverrideCount() == 1);
    CHECK(!renderer->materialOverrides()[0].paramsBuffer.valid());
    CHECK(!renderer->materialOverrides()[0].pipeline.valid());

    // A new mesh has a different slot layout. Keeping the old override here
    // used to retain both stale authored data and any UBO allocated for it.
    MeshHandle replacement;
    replacement.index = 7;
    replacement.generation = 1;
    renderer->setMesh(replacement);
    CHECK(renderer->materialOverrideCount() == 0);

    // A stack-owned Scene can be destroyed after an explicit
    // Engine::shutdown(). At that point its override handles refer to the
    // device that has already been torn down: cleanup must invalidate them,
    // not call the strict GPU singleton and abort the process.
    if (!GPU::tryGet())
    {
        Material orphaned;
        orphaned.paramsBuffer.index = 7;
        orphaned.paramsBuffer.generation = 1;
        orphaned.paramsDirty = false;
        MaterialManager::getSingleton().release(orphaned);
        CHECK(!orphaned.paramsBuffer.valid());
        CHECK(orphaned.paramsDirty);
    }
}

void testAllAuthoredMaterialFilesParse()
{
    std::error_code iteratorError;
    usize materialFileCount = 0;
    for (std::filesystem::recursive_directory_iterator it(RADION_TEST_ASSET_DIR, iteratorError),
         end;
         it != end && !iteratorError; it.increment(iteratorError))
    {
        if (!it->is_regular_file())
            continue;
        const std::string extension = it->path().extension().string();
        if (extension != ".mat" && extension != ".material")
            continue;

        ++materialFileCount;
        std::vector<MaterialDefinition> definitions;
        MaterialParseError error;
        if (!MaterialParser::parseFile(it->path().string(), definitions, &error))
        {
            std::fprintf(stderr, "Material parse failed: %s:%u:%u: %s\n",
                         it->path().string().c_str(), error.line, error.column,
                         error.message.c_str());
            CHECK(false);
        }
        CHECK(!definitions.empty());
    }
    CHECK(!iteratorError);
    if (materialFileCount == 0)
        std::fprintf(stderr, "SceneTests: skipping optional authored material fixtures\n");
}

} // namespace

int main()
{
    testRadionFormat();
    testAnimatedPlayers();
    testInverseKinematics();
    testHierarchyAndOrder();
    testGameObjectIds();
    testSceneSerializerEmptyRoundTrip();
    testSceneSerializerFailedLoadLeavesSceneIntact();
    testSceneSerializerHierarchyRoundTrip();
    testSceneSerializerValidation();
    testSceneSerializerEditorCreatedMarkersRoundTrip();
    testSceneSerializerRenderSettingsRoundTrip();
    testSceneSerializerCameraRoundTrip();
    testSceneSerializerLightRoundTrip();
    testSceneSerializerComponentValidation();
    testAudioPlayerRoundTrip();
    testPhysicsBodyRoundTrip();
    testPhysicsBodyFalls();
    testPhysicsBodyDestroyedMidSimulation();
    testPhysicsBodyRunningInEditorFreezesSimulation();
    testPhysicsBodyTeleportInPlay();
    testUiControlsRoundTrip();
    testPrefabRoundTrip();
    testTransforms();
    testSceneQueues();
    testLights();
    testShadowLayout();
    testComponentEventsAndDispose();
    testActionRunner();
    testRibbonTrail();
    testRoadPoints();
    testRoadSplineRoundTrip();
    testCameraAndRay();
    testDestroyBranch();
    testRenderListRejectsInvalidPipelines();
    testProfilerHistory();
    testPresentationFit();
    testDebugDrawGeometry();
    testProceduralTrees();
    testForestWithoutSpecies();
    testParticleEffect();
    testComponentSelfRemovalCompactsLazily();
    testMaterialSaveParserRoundTrip();
    testMeshRendererMaterialOwnershipState();
    testAllAuthoredMaterialFilesParse();
    testStaticMeshLoad();

    if (gFailures)
        std::fprintf(stderr, "%d scene test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
