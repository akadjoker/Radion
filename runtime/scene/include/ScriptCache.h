#ifndef RADION_SCRIPT_CACHE_H
#define RADION_SCRIPT_CACHE_H

#include "Component.h"
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

// One shared zen::VM: a script compiles once however many GameObjects run it. Keyed by script path or source text.
// The behaviour class is the first top-level class defining on_start/on_update/on_destroy/on_collision/on_event; a hint name (file stem) wins on an exact match.
class ScriptCache
{
public:
    static ScriptCache& getSingleton();

    // False once the singleton is torn down at exit; ZenBehaviour destructors must not reach into it then.
    static bool alive();

    ScriptCache(const ScriptCache&) = delete;
    ScriptCache& operator=(const ScriptCache&) = delete;

    // Vtable slots resolved once per class; -1 means not defined.
    struct Entry
    {
        zen::Value classValue = zen::val_nil();
        int initSlot = -1;
        int onStartSlot = -1;
        int onUpdateSlot = -1;
        int onDestroySlot = -1;
        int onCollisionSlot = -1;
        int onEventSlot = -1;
        // Built once per compile: from the compiled class first, then from source for constructor-declared fields.
        std::vector<ScriptProperty> properties;
        // Bumped on every (re)compile; components re-instantiate when it differs from their own.
        int version = 0;
        // Raw tick count so the header needs no <filesystem>; zero for source-string entries.
        s64 sourceTime = 0;
    };

    // Reused by instanceFor() so the same component yields the same instance.
    struct CachedInstance
    {
        void* key = nullptr;
        zen::ObjClass* klass = nullptr;
        zen::Value value = zen::val_nil();
    };

    // Entry pointers stay valid for the cache's life (never erased, reloadFile() rebuilds in place), so components keep them.
    // Nullptr (outError set) if unreadable, uncompilable, or no usable behaviour class.
    const Entry* loadFile(const std::string& path, std::string& outError);

    const Entry* loadSource(const std::string& source, std::string& outError);

    // False (outError set) if no such entry or recompile fails; the old entry is left untouched.
    bool reloadFile(const std::string& path, std::string& outError);

    // Meant for one call when scripts start mattering (editor Play), not the frame loop: stats one file per script.
    int refreshChangedFiles();

    zen::VM& vm();

    int compileCount() const;

    // Roots an instance for the GC: only a ZenBehaviour holds one between frames, outside anything the collector walks.
    void protectInstance(zen::Value instance);
    void unprotectInstance(zen::Value instance);
    usize protectedInstanceCount() const;

    // Same klass and pointer return the same Value. Classes must be persistent (see registerComponentClasses), so the GC never touches the wrapper.
    zen::Value instanceFor(zen::ObjClass* klass, void* pointer);

    // Defined for parity with the reference; nothing calls it there either.
    void forgetInstance(void* pointer);
    // For tests.
    bool hasCachedInstance(const void* pointer) const;

    // Indexed by ComponentType; read by the get/add/remove/has_component dispatch. One slot per type instead of a named pointer per component.
    void setComponentClass(ComponentType type, zen::ObjClass* klass);
    zen::ObjClass* componentClass(ComponentType type) const;

private:
    ScriptCache();
    ~ScriptCache();

    Entry* buildEntry(const std::string& key, bool isFile, const std::string& sourceOrPath,
                      const std::string& hintName, std::string& outError);

    static void gcMarkExtraRoots(zen::GC* gc, void* userData);

    ScriptVM mScriptVM;
    std::unordered_map<std::string, Entry> mEntries;
    std::vector<zen::Value> mProtectedInstances;
    // Indexed by object so removal is swap-and-pop O(1) while the GC still marks linearly.
    std::unordered_map<zen::Obj*, usize> mProtectedInstanceIndices;
    int mCompileCount = 0;

    std::unordered_map<void*, CachedInstance> mInstances;
    zen::ObjClass* mComponentClasses[static_cast<u8>(ComponentType::Count)] = {};
};

} // namespace Radion

#endif // RADION_SCRIPT_CACHE_H
