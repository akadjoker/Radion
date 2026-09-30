#ifndef RADION_ZEN_BEHAVIOUR_H
#define RADION_ZEN_BEHAVIOUR_H

#include "ScriptCache.h"
#include "ScriptComponent.h"

#include "zen/value.h"

#include <string>

namespace Radion
{

class GameObject;

// Script defines one class with any of on_start(self)/on_update(self, dt)/on_destroy(self)/on_collision(self, other, began)/on_event(self, event, value).
// One instance per component via the shared ScriptCache; the owner is the instance's "node" field (see SceneScriptBindings), `owner` kept as an alias.
class ZenBehaviour : public ScriptComponent
{
public:
    ZenBehaviour();
    ~ZenBehaviour() override;

    // Neither load creates the instance or calls on_start(); onUpdate() does both lazily, the first time the scene is not running in the editor.
    bool loadFile(const std::string& path);
    bool loadSource(const std::string& source);

    // Every ZenBehaviour on that path re-instantiates on its next update. False if loaded from a source string.
    bool reload();

    // Reloads only if the on-disk timestamp moved. False for source-string behaviours or an unchanged file.
    bool reloadIfChanged();
    // Read off the shared ScriptCache entry, so it moves for every component on this path. 0 for source strings.
    s64 sourceTimestamp() const;

    const std::string& scriptPath() const;
    bool hasError() const;
    const std::string& lastError() const;

    // False (no-op) when the class defines no on_event.
    bool callEvent(const std::string& event, f64 value = 0.0);

    // General escape hatch beside the fixed hooks. False (and hasError()) if the class defines no such name.
    bool callFunction(const std::string& name, f64 value = 0.0);

    // Checked against the compiled class's vtable; no instance touched.
    bool hasFunction(const std::string& name) const;

    // Shared by every component running the script; empty until a load succeeds.
    usize declaredPropertyCount() const;
    const ScriptProperty* declaredPropertyAt(usize index) const;
    const ScriptProperty* declaredProperty(const std::string& name) const;

    // Written over the script's defaults after __init__; what the inspector edits and the scene file stores.
    usize overrideCount() const;
    const ScriptProperty* overrideAt(usize index) const;
    const ScriptProperty* findOverride(const std::string& name) const;
    void setNumberOverride(const std::string& name, f64 value, bool integer = false);
    void setStringOverride(const std::string& name, const std::string& value);
    void setBoolOverride(const std::string& name, bool value);
    void clearOverride(const std::string& name);
    void clearOverrides();
    usize applyOverrides();

    bool isZenBehaviour() const override;

    // Called by CollisionWorld, gated in editor mode like onUpdate(). CollisionWorld keeps no enter/exit state, so began is always true;
    // see callCollision().
    void onCollision(GameObject* other);

    // The began-aware form. Older on_collision(self, other) scripts still work: zen ignores the extra argument on a native-invoked call.
    bool callCollision(GameObject* other, bool began);

protected:
    void onUpdate(f32 deltaTime) override;
    void onDestroy() override;

private:
    bool ensureInstance();
    void releaseInstance();
    void fail(const std::string& error);
    ScriptProperty& overrideSlot(const std::string& name);
    bool writeProperty(const ScriptProperty& property);

    std::string mScriptPath;
    std::string mLastError;
    // Valid for the cache's whole life; a reload rebuilds it in place and bumps its version, which onUpdate() watches.
    const ScriptCache::Entry* mEntry = nullptr;
    std::vector<ScriptProperty> mOverrides;
    zen::Value mInstance = zen::val_nil();
    int mBoundVersion = 0;
    bool mLoaded = false;
    bool mFailed = false;
    bool mStartCalled = false;
    bool mFromFile = false;
};

// Tells a Zen behaviour from a C++ one on the shared Script slot, without RTTI.
template <> struct ComponentMatch<ZenBehaviour>
{
    static bool test(const Component* component)
    {
        return static_cast<const ScriptComponent*>(component)->isZenBehaviour();
    }
};

} // namespace Radion

#endif // RADION_ZEN_BEHAVIOUR_H
