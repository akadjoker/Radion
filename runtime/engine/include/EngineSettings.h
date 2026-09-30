#ifndef RADION_ENGINE_SETTINGS_H
#define RADION_ENGINE_SETTINGS_H

#include "Types.h"

#include <string>

namespace Radion
{

class Engine;
 
class EngineSettings
{
public:
    // A missing file is not an error (defaults); a malformed one is logged.
    static bool load(Engine& engine, const std::string& filename);
    static bool save(const Engine& engine, const std::string& filename);
};

} // namespace Radion

#endif // RADION_ENGINE_SETTINGS_H
