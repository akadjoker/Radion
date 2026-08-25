#ifndef RADION_SCRIPT_VM_H
#define RADION_SCRIPT_VM_H

#include "Types.h"

#include <string>

namespace zen
{
class VM;
struct NativeLib;
}

namespace Radion
{

// A tagged value used to pass arguments to, and read return values and
// globals from, a Zen script. Keeps every zen:: type out of ScriptVM's
// public API - callers outside runtime/script never need vendor/zen headers
// just to call a script function or read/write a global.
struct ScriptValue
{
    enum class Kind : u8
    {
        Nil,
        Bool,
        Number,
        String
    };

    Kind kind = Kind::Nil;
    bool boolValue = false;
    f64 numberValue = 0.0;
    std::string stringValue;

    static ScriptValue fromBool(bool value);
    static ScriptValue fromNumber(f64 value);
    static ScriptValue fromString(const std::string& value);
};

// Thin wrapper around a zen::VM instance: owns it, opens the simple builtin
// modules and the "radion" native module, and runs script text or files.
// The calls below never log - they hand back success/failure plus the error
// text. What the VM prints on its own (a script's print, and the stack trace
// behind a runtime error) is routed into Radion's Log, so it reaches the
// editor console instead of a stderr nobody is reading.
class ScriptVM
{
public:
    ScriptVM();
    ~ScriptVM();

    ScriptVM(const ScriptVM&) = delete;
    ScriptVM& operator=(const ScriptVM&) = delete;

    // Compiles and runs a source string. Returns true on success. On
    // failure, outError holds the compile or runtime error message and the
    // function returns false without printing anything.
    bool runString(const char* source, const char* moduleName, std::string& outError);

    // Reads a file and runs it the same way as runString(). outError holds
    // the reason on failure, including "file not found" style errors.
    bool runFile(const char* path, std::string& outError);

    // Lets code outside runtime/script (scene bindings, e.g.) add native
    // classes/functions to this VM without runtime/script depending on them.
    // The lib's init_fn, if any, runs immediately - no "import" required
    // from the script for whatever it registers.
    bool registerModule(const zen::NativeLib& lib);

    // True if a top-level function with this name exists and is callable.
    bool hasFunction(const char* name) const;

    // Calls a top-level script function by name. All three fail softly like
    // runString()/runFile() - outError holds the reason, nothing is printed.
    bool call(const char* function, std::string& outError);
    bool call(const char* function, const ScriptValue* args, int argCount,
              std::string& outError);
    bool call(const char* function, const ScriptValue* args, int argCount,
              ScriptValue& outResult, std::string& outError);

    bool setGlobal(const char* name, bool value);
    bool setGlobal(const char* name, f64 value);
    bool setGlobal(const char* name, const std::string& value);
    bool getGlobal(const char* name, bool& outValue) const;
    bool getGlobal(const char* name, f64& outValue) const;
    bool getGlobal(const char* name, std::string& outValue) const;

    // Direct access to the underlying VM, for bindings code that needs more
    // than call()/setGlobal() offer - defining native classes, building
    // instances by hand (runtime/scene's SceneScriptBindings, e.g.).
    zen::VM* vm() const;

private:
    zen::VM* mVM;
};

} // namespace Radion

#endif // RADION_SCRIPT_VM_H
