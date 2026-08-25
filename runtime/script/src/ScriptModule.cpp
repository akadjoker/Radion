#include "ScriptModule.h"

#include "zen/module.h"
#include "zen/object.h"
#include "zen/value.h"
#include "zen/vm.h"

namespace Radion
{

static int scriptModuleVersion(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    zen::ObjString* version = vm->make_string("0.1.0");
    args[0] = zen::val_obj((zen::Obj*)version);
    return 1;
}

static const zen::NativeReg kRadionFunctions[] = {
    {"version", scriptModuleVersion, 0},
};

static const zen::NativeLib kRadionLib = {
    "radion",
    kRadionFunctions,
    (int)(sizeof(kRadionFunctions) / sizeof(kRadionFunctions[0])),
    nullptr,
    0,
    nullptr,
};

const zen::NativeLib& ScriptModule::get()
{
    return kRadionLib;
}

} // namespace Radion
