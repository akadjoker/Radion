#ifndef RADION_SCRIPT_MODULE_H
#define RADION_SCRIPT_MODULE_H

namespace zen
{
struct NativeLib;
}

namespace Radion
{

// Registration point for the "radion" native module exposed to Zen scripts.
// Keeps the zen::NativeLib table and its native functions grouped in one
// place instead of loose functions in the zen namespace.
class ScriptModule
{
public:
    static const zen::NativeLib& get();
};

} // namespace Radion

#endif // RADION_SCRIPT_MODULE_H
