#ifndef RADION_SCRIPT_CACHE_H
#define RADION_SCRIPT_CACHE_H

#include "ScriptProperty.h"
#include "ScriptVM.h"

#include "zen/value.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace zen
{
class VM;
struct GC;
struct Obj;
struct ObjClass;
}

namespace Radion
{

// One shared zen::VM for every Zen-scripted behaviour in the process: a
// script body compiles exactly once no matter how many GameObjects run it -
// 500 bullets sharing one script cost one compile and 500 cheap instances,
// not 500 compiles. Entries are keyed by script path (file loads) or by the
// source text itself (string loads, so two identical inline scripts still
// share one entry).
//
// Convention for picking the behaviour class out of a compiled script: the
// first top-level class the run just defined (in declaration order) that
// defines at least one of on_start/on_update/on_destroy/on_collision. A hint name
// (the file's stem, for loadFile()) is tried first and wins on an exact
// match, so a script with several classes can still be steered explicitly.
class ScriptCache
{
public:
    static ScriptCache& getSingleton();

    // False once the singleton has been torn down at exit. A ZenBehaviour
    // that outlives it - one owned by a Scene with static storage - would
    // otherwise reach into a destroyed object from its own destructor, so
    // every call it makes into the cache during teardown is guarded by this.
    static bool alive();

    ScriptCache(const ScriptCache&) = delete;
    ScriptCache& operator=(const ScriptCache&) = delete;

    // A compiled behaviour class plus the vtable slots for its __init__ and
    // on_start/on_update/on_destroy/on_collision methods, resolved once here
    // instead of once per component. -1 means the class does not define that
    // one.
    struct Entry
    {
        zen::Value classValue = zen::val_nil();
        int initSlot = -1;
        int onStartSlot = -1;
        int onUpdateSlot = -1;
        int onDestroySlot = -1;
        int onCollisionSlot = -1;
        // Built once per compile, not once per component: what a script
        // declares belongs to the script. A ZenBehaviour reads this list to
        // know what the inspector may drive, and keeps only its own
        // overrides on top of it. Comes from the compiled class first (a
        // field the class body gave a value), then from reading the source
        // for anything a constructor declares instead.
        std::vector<ScriptProperty> properties;
        // Bumped every time this key is (re)compiled. A ZenBehaviour
        // compares this against the version its own instance was built
        // from and re-instantiates when they differ - what lets reload()
        // called on any one component propagate to every other component
        // sharing the same script.
        int version = 0;
        // The file's last-write time when it was compiled, as a raw tick
        // count, so the header needs no <filesystem>. Zero for entries
        // loaded from a source string. refreshChangedFiles() compares
        // against it; nothing else reads it.
        s64 sourceTime = 0;
    };

    // One script-side handle per native component pointer, reused by
    // instanceFor() so a script that fetches the same component twice gets
    // back the same instance rather than a fresh wrapper each time.
    struct CachedInstance
    {
        void* key = nullptr;
        zen::ObjClass* klass = nullptr;
        zen::Value value = zen::val_nil();
    };

    // An Entry pointer stays valid for the whole life of the cache: entries
    // are never erased, and reloadFile() rebuilds one in place rather than
    // replacing it. That is what lets a component hold the pointer it got
    // from load*() and never look the key up again - a per-frame lookup
    // would mean building the "file:"/"src:" key string on every update, for
    // every scripted object.
    //
    // Compiles the file on first use; every later call for the same path is
    // a cache lookup. Nullptr (with outError set) if the file cannot be
    // read/compiled, or compiles fine but defines no usable behaviour class.
    const Entry* loadFile(const std::string& path, std::string& outError);

    // Same, keyed by the source text itself rather than a path.
    const Entry* loadSource(const std::string& source, std::string& outError);

    // Recompiles the entry that was loaded from this exact path, bumping
    // its version. False (outError set) if no such file entry exists yet or
    // recompiling fails - the old entry is left untouched either way.
    bool reloadFile(const std::string& path, std::string& outError);

    // Recompiles every cached file whose last-write time moved since it was
    // compiled, and returns how many were rebuilt. Without this the cache
    // would happily run a script the user edited ten minutes ago: a compile
    // is kept for the life of the process and nothing else ever looks at the
    // file again. Meant for one call at the moment scripts start mattering
    // (the editor's Play), not for the frame loop - it stats one file per
    // cached script.
    int refreshChangedFiles();

    zen::VM& vm();

    // How many times a script body has actually been compiled - what a
    // caching regression would move, checked directly in tests instead of
    // inferred from timing.
    int compileCount() const;

    // Roots a script instance Value for the GC: instances of script-defined
    // classes are ordinary (non-persistent) GC objects, and a ZenBehaviour
    // is the only thing holding one between frames, outside of any VM
    // global/stack the collector already walks on its own. See
    // ScriptCache.cpp for how this hooks into zen's GC.
    void protectInstance(zen::Value instance);
    void unprotectInstance(zen::Value instance);
    usize protectedInstanceCount() const;

    // One script-side wrapper per native component pointer: the same pointer
    // asked for twice against the same klass gets back the same Value, not a
    // new instance each time. Classes handed here are expected to be
    // persistent (see SceneScriptBindings::registerComponentClasses), so the
    // wrapper is never touched by the GC once created.
    zen::Value instanceFor(zen::ObjClass* klass, void* pointer);

    // Defined for parity with the reference this is ported from; nothing
    // calls it there either.
    void forgetInstance(void* pointer);
    // Whether a handle is currently cached for this pointer. For tests: the
    // cache is what keeps a destroyed component's handle alive, so being able
    // to see it is what makes that testable.
    bool hasCachedInstance(const void* pointer) const;

    // Class pointers for the script-facing component classes, one named
    // pointer per component - set once by
    // SceneScriptBindings::registerComponentClasses() right after it
    // registers the matching class, and read by every getter it defines.
    void setCameraClass(zen::ObjClass* klass);
    zen::ObjClass* cameraClass() const;
    void setLightClass(zen::ObjClass* klass);
    zen::ObjClass* lightClass() const;

private:
    ScriptCache();
    ~ScriptCache();

    Entry* buildEntry(const std::string& key, bool isFile, const std::string& sourceOrPath,
                      const std::string& hintName, std::string& outError);

    static void gcMarkExtraRoots(zen::GC* gc, void* userData);

    ScriptVM mScriptVM;
    std::unordered_map<std::string, Entry> mEntries;
    std::vector<zen::Value> mProtectedInstances;
    // Value roots are removed on destruction/reload. Keeping the vector for
    // the GC's linear marking pass but indexing it by object makes removal
    // swap-and-pop O(1), rather than a quadratic shutdown when thousands of
    // short-lived scripted objects have existed.
    std::unordered_map<zen::Obj*, usize> mProtectedInstanceIndices;
    int mCompileCount = 0;

    std::unordered_map<void*, CachedInstance> mInstances;
    zen::ObjClass* mCameraClass = nullptr;
    zen::ObjClass* mLightClass = nullptr;
};

} // namespace Radion

#endif // RADION_SCRIPT_CACHE_H
