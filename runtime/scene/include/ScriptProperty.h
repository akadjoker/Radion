#ifndef RADION_SCRIPT_PROPERTY_H
#define RADION_SCRIPT_PROPERTY_H

#include "Types.h"

#include <string>
#include <vector>

namespace Radion
{

struct ScriptProperty
{
    enum class Kind : u8
    {
        Number,
        String,
        Bool
    };

    std::string name;
    Kind kind = Kind::Number;
    f64 number = 0.0;
    std::string text;
    bool flag = false;
    // No '.' and no exponent, so it returns to the VM as an int: "self.lives == 3" must not get 3.0.
    bool integer = false;
};

// Fallback path: class-body fields are read off the compiled class by ScriptCache; constructor-declared fields need the source scanned.
// Only literals (and top-level constants bound to literals) are read; a leading underscore is private and not listed.
class ScriptProperties
{
public:
    static usize scan(const char* source, std::vector<ScriptProperty>& out);
};

} // namespace Radion

#endif // RADION_SCRIPT_PROPERTY_H
