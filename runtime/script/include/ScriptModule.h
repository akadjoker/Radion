#ifndef RADION_SCRIPT_MODULE_H
#define RADION_SCRIPT_MODULE_H

namespace zen
{
struct NativeLib;
}

namespace Radion
{

// Registration point for the "radion" native module exposed to Zen scripts.
class ScriptModule
{
public:
    static const zen::NativeLib& get();
};

} // namespace Radion

#endif // RADION_SCRIPT_MODULE_H
