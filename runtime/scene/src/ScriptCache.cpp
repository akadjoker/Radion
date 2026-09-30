#include "PCH.h"

#include "ScriptCache.h"

#include "FileSystem.h"
#include "Log.h"
#include "SceneScriptBindings.h"

#include "zen/memory.h"
#include "zen/object.h"
#include "zen/vm.h"

#include <filesystem>
#include <fstream>
#include <iterator>

namespace Radion
{

static bool classDefinesSlot(zen::ObjClass* klass, int slot)
{
    return slot >= 0 && slot < klass->vtable_size && !zen::is_nil(klass->vtable[slot]);
}

static bool classDefinesAnyHook(zen::VM& vm, zen::ObjClass* klass)
{
    return classDefinesSlot(klass, vm.intern_selector("on_start", 8)) ||
           classDefinesSlot(klass, vm.intern_selector("on_update", 9)) ||
           classDefinesSlot(klass, vm.intern_selector("on_destroy", 10)) ||
           classDefinesSlot(klass, vm.intern_selector("on_collision", 12)) ||
           classDefinesSlot(klass, vm.intern_selector("on_event", 8));
}

// A slot belongs to the new script if it did not exist or now holds a different class. A recompile reuses the same slot index (Compiler::require_global_slot), so counting only new slots would miss it.
static bool isNewOrRebuiltClass(const std::vector<zen::Value>& before, int index,
                                zen::ObjClass* klass)
{
    if (index >= (int)before.size())
        return true;
    const zen::Value previous = before[(usize)index];
    return !zen::is_class(previous) || zen::as_class(previous) != klass;
}

// An exact name match among the just-defined classes wins, otherwise the first (declaration order) defining a hook.
static zen::ObjClass* pickBehaviourClass(zen::VM& vm, const std::vector<zen::Value>& before,
                                         const std::string& hintName)
{
    const int count = vm.num_globals();

    if (!hintName.empty())
    {
        for (int i = 0; i < count; ++i)
        {
            const zen::Value value = vm.get_global(i);
            if (!zen::is_class(value) || !isNewOrRebuiltClass(before, i, zen::as_class(value)))
                continue;
            const char* name = vm.global_name(i);
            if (name && hintName == name)
                return zen::as_class(value);
        }
    }

    for (int i = 0; i < count; ++i)
    {
        const zen::Value value = vm.get_global(i);
        if (!zen::is_class(value) || !isNewOrRebuiltClass(before, i, zen::as_class(value)))
            continue;
        if (classDefinesAnyHook(vm, zen::as_class(value)))
            return zen::as_class(value);
    }

    return nullptr;
}

// Only what the compiler recorded: fields given a value in the class body. A field that only appears in a method has a name but no value, which keeps working state out of the inspector.
static void collectClassProperties(zen::ObjClass* klass, std::vector<ScriptProperty>& out)
{
    if (!klass || !klass->field_names || !klass->field_defaults)
        return;

    const int count = klass->num_field_defaults < klass->num_fields ? klass->num_field_defaults
                                                                    : klass->num_fields;
    for (int i = 0; i < count; ++i)
    {
        zen::ObjString* name = klass->field_names[i];
        if (!name || name->chars[0] == '\0' || name->chars[0] == '_')
            continue;

        const zen::Value value = klass->field_defaults[i];
        ScriptProperty property;
        property.name = name->chars;
        if (zen::is_int(value))
        {
            property.kind = ScriptProperty::Kind::Number;
            property.number = (f64)value.as.integer;
            property.integer = true;
        }
        else if (zen::is_float(value))
        {
            property.kind = ScriptProperty::Kind::Number;
            property.number = value.as.number;
        }
        else if (zen::is_bool(value))
        {
            property.kind = ScriptProperty::Kind::Bool;
            property.flag = value.as.boolean;
        }
        else if (zen::is_string(value))
        {
            property.kind = ScriptProperty::Kind::String;
            property.text.assign(zen::safe_string_chars(value),
                                 (usize)zen::safe_string_len(value));
        }
        else
        {
            // None, or no editor representation.
            continue;
        }
        out.push_back(property);
    }
}

static bool hasProperty(const std::vector<ScriptProperty>& list, const std::string& name)
{
    for (usize i = 0; i < list.size(); ++i)
        if (list[i].name == name)
            return true;
    return false;
}

static int resolveSlot(zen::VM& vm, zen::ObjClass* klass, const char* name, int len)
{
    const int slot = vm.intern_selector(name, len);
    return classDefinesSlot(klass, slot) ? slot : -1;
}

// Zero-initialised before dynamic initialisation, so false before and after the singleton's life; only the constructor and destructor write it.
static bool gScriptCacheAlive = false;

static const char* const kFileKeyPrefix = "file:";
static const usize kFileKeyPrefixLength = 5;

// Raw tick count; only compared against another reading of the same file, so the epoch does not matter.
static s64 fileWriteTime(const std::string& path)
{
    std::error_code error;
    const std::filesystem::file_time_type time = std::filesystem::last_write_time(path, error);
    return error ? 0 : (s64)time.time_since_epoch().count();
}

ScriptCache& ScriptCache::getSingleton()
{
    static ScriptCache instance;
    return instance;
}

bool ScriptCache::alive()
{
    return gScriptCacheAlive;
}

ScriptCache::ScriptCache()
{
    gScriptCacheAlive = true;
    mScriptVM.registerModule(SceneScriptBindings::library());
    // Passed directly, not via getSingleton(): this constructor is still under the static-local guard, so calling back would re-enter it.
    SceneScriptBindings::registerComponentClasses(*this);
    zen::GC& gc = mScriptVM.vm()->get_gc();
    gc.extra_mark = &ScriptCache::gcMarkExtraRoots;
    gc.extra_mark_ud = this;
}

ScriptCache::~ScriptCache()
{
    gScriptCacheAlive = false;
}

void ScriptCache::gcMarkExtraRoots(zen::GC* gc, void* userData)
{
    ScriptCache* cache = static_cast<ScriptCache*>(userData);

    // Every cached class stays alive for the cache's lifetime, else a shadowed ObjClass (and instances running it) would dangle when the GC swept it.
    for (auto& entry : cache->mEntries)
        zen::gc_mark_value(gc, entry.second.classValue);

    // Instances of script-defined classes are ordinary GC objects; a ZenBehaviour is the only holder between frames.
    for (const zen::Value& value : cache->mProtectedInstances)
        zen::gc_mark_value(gc, value);
}

// Read here, not via ScriptVM::runFile(): the source is needed twice (compile, and scanning declared properties).
static std::string readWholeFile(const std::string& path, bool& ok)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
        ok = false;
        return std::string();
    }
    ok = true;
    return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

ScriptCache::Entry* ScriptCache::buildEntry(const std::string& key, bool isFile,
                                            const std::string& sourceOrPath,
                                            const std::string& hintName, std::string& outError)
{
    zen::VM& vm = *mScriptVM.vm();

    std::string source;
    if (isFile)
    {
        bool ok = false;
        source = readWholeFile(sourceOrPath, ok);
        if (!ok)
        {
            outError = "zen: cannot open file: ";
            outError += sourceOrPath;
            return nullptr;
        }
    }
    const std::string& text = isFile ? source : sourceOrPath;
    const char* moduleName = isFile ? sourceOrPath.c_str() : "zen_behaviour";

    std::vector<zen::Value> before;
    before.reserve((usize)vm.num_globals());
    for (int i = 0; i < vm.num_globals(); ++i)
        before.push_back(vm.get_global(i));

    if (!mScriptVM.runString(text.c_str(), moduleName, outError))
        return nullptr;

    ++mCompileCount;

    zen::ObjClass* klass = pickBehaviourClass(vm, before, hintName);
    if (!klass)
    {
        outError = "zen: no behaviour class (on_start/on_update/on_destroy) found in script";
        return nullptr;
    }

    Entry& stored = mEntries[key];
    stored.classValue = zen::val_obj((zen::Obj*)klass);
    stored.initSlot = resolveSlot(vm, klass, "__init__", 8);
    stored.onStartSlot = resolveSlot(vm, klass, "on_start", 8);
    stored.onUpdateSlot = resolveSlot(vm, klass, "on_update", 9);
    stored.onDestroySlot = resolveSlot(vm, klass, "on_destroy", 10);
    stored.onCollisionSlot = resolveSlot(vm, klass, "on_collision", 12);
    stored.onEventSlot = resolveSlot(vm, klass, "on_event", 8);
    stored.version += 1;
    stored.sourceTime = isFile ? fileWriteTime(sourceOrPath) : 0;

    stored.properties.clear();
    collectClassProperties(klass, stored.properties);

    // Constructor-declared fields live on the instance, not the class, so the source is scanned; class-declared ones win.
    std::vector<ScriptProperty> fromInit;
    ScriptProperties::scan(text.c_str(), fromInit);
    for (usize i = 0; i < fromInit.size(); ++i)
        if (!hasProperty(stored.properties, fromInit[i].name))
            stored.properties.push_back(fromInit[i]);

    return &stored;
}

const ScriptCache::Entry* ScriptCache::loadFile(const std::string& path, std::string& outError)
{
    // Scene files keep the authored path while FileSystem owns the search paths: compile from the resolved path so editor and standalone runner agree; the component keeps the authored path.
    const std::string resolved = FileSystem::getSingleton().resolve(path);
    if (resolved.empty())
    {
        outError = "zen: cannot open file: ";
        outError += path;
        return nullptr;
    }
    const std::string key = kFileKeyPrefix + resolved;
    auto it = mEntries.find(key);
    if (it != mEntries.end())
        return &it->second;

    const std::string hint = std::filesystem::path(resolved).stem().string();
    return buildEntry(key, true, resolved, hint, outError);
}

const ScriptCache::Entry* ScriptCache::loadSource(const std::string& source, std::string& outError)
{
    const std::string key = "src:" + source;
    auto it = mEntries.find(key);
    if (it != mEntries.end())
        return &it->second;

    return buildEntry(key, false, source, std::string(), outError);
}

bool ScriptCache::reloadFile(const std::string& path, std::string& outError)
{
    const std::string resolved = FileSystem::getSingleton().resolve(path);
    if (resolved.empty())
    {
        outError = "zen: cannot open file: ";
        outError += path;
        return false;
    }
    const std::string key = kFileKeyPrefix + resolved;
    if (mEntries.find(key) == mEntries.end())
    {
        outError = "zen: cannot reload a script that was never loaded: ";
        outError += path;
        return false;
    }

    const std::string hint = std::filesystem::path(resolved).stem().string();
    return buildEntry(key, true, resolved, hint, outError) != nullptr;
}

int ScriptCache::refreshChangedFiles()
{
    int rebuilt = 0;
    for (auto& pair : mEntries)
    {
        if (pair.first.compare(0, kFileKeyPrefixLength, kFileKeyPrefix) != 0)
            continue;

        const std::string path = pair.first.substr(kFileKeyPrefixLength);
        const s64 written = fileWriteTime(path);
        // A missing file keeps its existing compile rather than dropping a working script.
        if (written == 0 || written == pair.second.sourceTime)
            continue;

        // buildEntry() replaces the mapped value in place (no insert, no rehash), so iterating stays valid. A failed recompile leaves the old entry.
        const std::string hint = std::filesystem::path(path).stem().string();
        std::string error;
        if (buildEntry(pair.first, true, path, hint, error))
            ++rebuilt;
        else
            Log::warning("ScriptCache: '%s' changed but does not compile: %s", path.c_str(),
                         error.c_str());
    }
    return rebuilt;
}

zen::VM& ScriptCache::vm()
{
    return *mScriptVM.vm();
}

int ScriptCache::compileCount() const
{
    return mCompileCount;
}

void ScriptCache::protectInstance(zen::Value instance)
{
    if (!zen::is_instance(instance))
        return;
    zen::Obj* object = instance.as.obj;
    if (mProtectedInstanceIndices.find(object) != mProtectedInstanceIndices.end())
        return;
    mProtectedInstanceIndices[object] = mProtectedInstances.size();
    mProtectedInstances.push_back(instance);
}

void ScriptCache::unprotectInstance(zen::Value instance)
{
    if (!zen::is_instance(instance))
        return;
    const auto found = mProtectedInstanceIndices.find(instance.as.obj);
    if (found == mProtectedInstanceIndices.end())
        return;

    const usize index = found->second;
    const usize last = mProtectedInstances.size() - 1;
    if (index != last)
    {
        const zen::Value moved = mProtectedInstances[last];
        mProtectedInstances[index] = moved;
        mProtectedInstanceIndices[moved.as.obj] = index;
    }
    mProtectedInstances.pop_back();
    mProtectedInstanceIndices.erase(found);
}

usize ScriptCache::protectedInstanceCount() const
{
    return mProtectedInstances.size();
}

zen::Value ScriptCache::instanceFor(zen::ObjClass* klass, void* pointer)
{
    if (!pointer)
        return zen::val_nil();

    auto found = mInstances.find(pointer);
    if (found != mInstances.end() && found->second.klass == klass)
        return found->second.value;

    const zen::Value value = vm().make_instance(klass);
    zen::as_instance(value)->native_data = pointer;

    CachedInstance cached;
    cached.key = pointer;
    cached.klass = klass;
    cached.value = value;
    mInstances[pointer] = cached;
    return value;
}

bool ScriptCache::hasCachedInstance(const void* pointer) const
{
    return mInstances.find(const_cast<void*>(pointer)) != mInstances.end();
}

void ScriptCache::forgetInstance(void* pointer)
{
    auto found = mInstances.find(pointer);
    if (found == mInstances.end())
        return;
    // The handle's class is persistent, so dropping the cache entry alone would leave native_data dangling; clearing it is what the self*(args) null checks see.
    if (zen::is_instance(found->second.value))
        zen::as_instance(found->second.value)->native_data = nullptr;
    mInstances.erase(found);
}

void ScriptCache::setComponentClass(ComponentType type, zen::ObjClass* klass)
{
    const u8 index = static_cast<u8>(type);
    if (index >= static_cast<u8>(ComponentType::Count))
        return;
    mComponentClasses[index] = klass;
}

zen::ObjClass* ScriptCache::componentClass(ComponentType type) const
{
    const u8 index = static_cast<u8>(type);
    return index < static_cast<u8>(ComponentType::Count) ? mComponentClasses[index] : nullptr;
}

} // namespace Radion
