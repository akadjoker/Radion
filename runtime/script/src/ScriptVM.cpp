#include "ScriptVM.h"

#include "Log.h"
#include "ScriptModule.h"

#include "zen/compiler.h"
#include "zen/module.h"
#include "zen/object.h"
#include "zen/value.h"
#include "zen/vm.h"

#include <cstdio>
#include <cstdlib>

namespace Radion
{

ScriptValue ScriptValue::fromBool(bool value)
{
    ScriptValue result;
    result.kind = Kind::Bool;
    result.boolValue = value;
    return result;
}

ScriptValue ScriptValue::fromNumber(f64 value)
{
    ScriptValue result;
    result.kind = Kind::Number;
    result.numberValue = value;
    return result;
}

ScriptValue ScriptValue::fromString(const std::string& value)
{
    ScriptValue result;
    result.kind = Kind::String;
    result.stringValue = value;
    return result;
}

static zen::Value scriptValueToZen(zen::VM* vm, const ScriptValue& value)
{
    switch (value.kind)
    {
    case ScriptValue::Kind::Bool:
        return zen::val_bool(value.boolValue);
    case ScriptValue::Kind::Number:
        return zen::val_float(value.numberValue);
    case ScriptValue::Kind::String:
    {
        zen::ObjString* s = vm->make_string(value.stringValue.c_str(),
                                            (int)value.stringValue.size());
        return zen::val_obj((zen::Obj*)s);
    }
    case ScriptValue::Kind::Nil:
    default:
        return zen::val_nil();
    }
}

static ScriptValue zenToScriptValue(zen::Value value)
{
    if (zen::is_bool(value))
        return ScriptValue::fromBool(value.as.boolean);
    if (zen::is_int(value))
        return ScriptValue::fromNumber((f64)value.as.integer);
    if (zen::is_float(value))
        return ScriptValue::fromNumber(value.as.number);
    if (zen::is_string(value))
        return ScriptValue::fromString(
            std::string(zen::safe_string_chars(value), (usize)zen::safe_string_len(value)));
    return ScriptValue();
}

static std::string currentErrorMessage(zen::VM* vm)
{
    zen::ObjFiber* fiber = vm->current_fiber();
    return (fiber && fiber->error) ? fiber->error->chars : "zen: runtime error";
}

// zen emits one line per call, newline included, and Log adds its own.
static int trimmedLength(const char* text, int length)
{
    while (length > 0 && (text[length - 1] == '\n' || text[length - 1] == '\r'))
        --length;
    return length;
}

static void scriptPrint(const char* text, int length, void* userData)
{
    (void)userData;
    Log::info("%.*s", trimmedLength(text, length), text);
}

// Runtime errors reach here as a header line plus one line per stack frame.
// The frames are what makes a script error diagnosable and they exist
// nowhere else - fiber->error holds only the bare message.
static void scriptPrintError(const char* text, int length, void* userData)
{
    (void)userData;
    Log::error("%.*s", trimmedLength(text, length), text);
}

static char* readWholeFile(const char* path, long* outSize)
{
    FILE* file = std::fopen(path, "rb");
    if (!file)
        return nullptr;

    std::fseek(file, 0, SEEK_END);
    const long size = std::ftell(file);
    std::fseek(file, 0, SEEK_SET);

    char* buffer = (char*)std::malloc((size_t)size + 1);
    if (!buffer)
    {
        std::fclose(file);
        return nullptr;
    }

    const size_t bytesRead = std::fread(buffer, 1, (size_t)size, file);
    buffer[bytesRead] = '\0';
    std::fclose(file);

    if (outSize)
        *outSize = (long)bytesRead;
    return buffer;
}

ScriptVM::ScriptVM()
    : mVM(new zen::VM())
{
    zen::ZenCallbacks callbacks = zen::zen_default_callbacks();
    callbacks.print = &scriptPrint;
    callbacks.print_err = &scriptPrintError;
    mVM->set_callbacks(callbacks);

    mVM->open_lib_globals(&zen::zen_lib_base);
    mVM->register_lib(&zen::zen_lib_math);
    mVM->register_lib(&zen::zen_lib_time);
    mVM->register_lib(&zen::zen_lib_struct);
    mVM->register_lib(&zen::zen_lib_io);
    mVM->register_lib(&zen::zen_lib_os);
    mVM->register_lib(&zen::zen_lib_path);
    mVM->register_lib(&zen::zen_lib_json);
    mVM->register_lib(&zen::zen_lib_net);
    mVM->register_lib(&zen::zen_lib_http);
    mVM->register_lib(&ScriptModule::get());
}

ScriptVM::~ScriptVM()
{
    delete mVM;
}

bool ScriptVM::runString(const char* source, const char* moduleName, std::string& outError)
{
    outError.clear();

    zen::Compiler compiler;
    zen::ObjFunc* fn = compiler.compile(&mVM->get_gc(), mVM, source, moduleName);
    if (!fn)
    {
        outError = compiler.error_info_count() > 0 ? compiler.error_message(0)
                                                    : "zen: compilation failed";
        return false;
    }

    mVM->run(fn);
    if (mVM->had_error())
    {
        outError = currentErrorMessage(mVM);
        return false;
    }

    return true;
}

bool ScriptVM::runFile(const char* path, std::string& outError)
{
    long size = 0;
    char* source = readWholeFile(path, &size);
    if (!source)
    {
        outError = "zen: cannot open file: ";
        outError += path;
        return false;
    }

    const bool ok = runString(source, path, outError);
    std::free(source);
    return ok;
}

bool ScriptVM::registerModule(const zen::NativeLib& lib)
{
    mVM->open_lib(&lib);
    return true;
}

bool ScriptVM::hasFunction(const char* name) const
{
    const int idx = mVM->find_global(name);
    if (idx < 0)
        return false;
    const zen::Value value = mVM->get_global(idx);
    return zen::is_closure(value) || zen::is_native(value);
}

bool ScriptVM::call(const char* function, std::string& outError)
{
    ScriptValue result;
    return call(function, nullptr, 0, result, outError);
}

bool ScriptVM::call(const char* function, const ScriptValue* args, int argCount,
                    std::string& outError)
{
    ScriptValue result;
    return call(function, args, argCount, result, outError);
}

bool ScriptVM::call(const char* function, const ScriptValue* args, int argCount,
                    ScriptValue& outResult, std::string& outError)
{
    outError.clear();
    outResult = ScriptValue();

    if (!hasFunction(function))
    {
        outError = "zen: undefined function '";
        outError += function;
        outError += "'";
        return false;
    }

    static const int kMaxArgs = 8;
    zen::Value zenArgs[kMaxArgs];
    const int count = argCount < kMaxArgs ? argCount : kMaxArgs;
    for (int i = 0; i < count; ++i)
        zenArgs[i] = scriptValueToZen(mVM, args[i]);

    const zen::Value result = mVM->call_global(function, zenArgs, count);
    if (mVM->had_error())
    {
        outError = currentErrorMessage(mVM);
        return false;
    }

    outResult = zenToScriptValue(result);
    return true;
}

bool ScriptVM::setGlobal(const char* name, bool value)
{
    mVM->def_global(name, zen::val_bool(value));
    return true;
}

bool ScriptVM::setGlobal(const char* name, f64 value)
{
    mVM->def_global(name, zen::val_float(value));
    return true;
}

bool ScriptVM::setGlobal(const char* name, const std::string& value)
{
    zen::ObjString* s = mVM->make_string(value.c_str(), (int)value.size());
    mVM->def_global(name, zen::val_obj((zen::Obj*)s));
    return true;
}

bool ScriptVM::getGlobal(const char* name, bool& outValue) const
{
    const int idx = mVM->find_global(name);
    if (idx < 0)
        return false;
    const zen::Value value = mVM->get_global(idx);
    if (!zen::is_bool(value))
        return false;
    outValue = value.as.boolean;
    return true;
}

bool ScriptVM::getGlobal(const char* name, f64& outValue) const
{
    const int idx = mVM->find_global(name);
    if (idx < 0)
        return false;
    const zen::Value value = mVM->get_global(idx);
    if (zen::is_int(value))
    {
        outValue = (f64)value.as.integer;
        return true;
    }
    if (zen::is_float(value))
    {
        outValue = value.as.number;
        return true;
    }
    return false;
}

bool ScriptVM::getGlobal(const char* name, std::string& outValue) const
{
    const int idx = mVM->find_global(name);
    if (idx < 0)
        return false;
    const zen::Value value = mVM->get_global(idx);
    if (!zen::is_string(value))
        return false;
    outValue.assign(zen::safe_string_chars(value), (usize)zen::safe_string_len(value));
    return true;
}

zen::VM* ScriptVM::vm() const
{
    return mVM;
}

} // namespace Radion
