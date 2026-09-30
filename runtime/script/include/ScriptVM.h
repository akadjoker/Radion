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

// A tagged value for script arguments/returns/globals; keeps zen:: types out of the public API.
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

// Thin wrapper around a zen::VM. Calls never log: they return success plus error text. VM output (print, stack traces)
// is routed into Radion's Log so it reaches the editor console.
class ScriptVM
{
public:
    ScriptVM();
    ~ScriptVM();

    ScriptVM(const ScriptVM&) = delete;
    ScriptVM& operator=(const ScriptVM&) = delete;

    // Compiles and runs a source string; on failure outError holds the message and nothing is printed.
    bool runString(const char* source, const char* moduleName, std::string& outError);

    // Reads and runs a file like runString(); outError holds the reason on failure.
    bool runFile(const char* path, std::string& outError);

    // Lets outside code (scene bindings) add native classes/functions; the lib's init_fn runs immediately, no import needed.
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

    // Direct VM access for bindings (native classes, hand-built instances).
    zen::VM* vm() const;

private:
    zen::VM* mVM;
};

} // namespace Radion

#endif // RADION_SCRIPT_VM_H
