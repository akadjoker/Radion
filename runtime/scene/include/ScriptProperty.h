#ifndef RADION_SCRIPT_PROPERTY_H
#define RADION_SCRIPT_PROPERTY_H

#include "Types.h"

#include <string>
#include <vector>

namespace Radion
{

// One value a script declares for the editor to drive.
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
    // The literal carried no '.' and no exponent, so it goes back into the
    // VM as an int and not a float: a script testing "self.lives == 3" must
    // not be handed 3.0.
    bool integer = false;
};

// Reads the properties a constructor declares out of a script's source text.
//
// This is the fallback path. A field declared in the class body is recorded
// by the compiler itself and ScriptCache reads it off the compiled class -
// exact name, value and type, nothing parsed:
//
//     class Rotate:
//         speed = 120.0
//         label = "spin"
//
// A constructor writes its fields on the instance instead, so there is
// nothing on the class to read and the source has to be scanned:
//
//     SPEED = 120.0            # a top-level constant may be referenced
//     class Rotate:
//         def __init__(self):
//             self.speed = SPEED
//             self.enabled = True
//             self._phase = 0.0    # leading underscore: private, not listed
//
// Only literals (and names of top-level constants bound to literals) are
// read. Anything else in __init__ is skipped rather than guessed at.
class ScriptProperties
{
public:
    static usize scan(const char* source, std::vector<ScriptProperty>& out);
};

} // namespace Radion

#endif // RADION_SCRIPT_PROPERTY_H
