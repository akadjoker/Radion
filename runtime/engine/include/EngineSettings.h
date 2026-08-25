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
    // Missing file is not an error: a demo asks for its settings on startup
    // and gets its own defaults when there are none yet. A malformed one is,
    // and is logged.
    static bool load(Engine& engine, const std::string& filename);
    static bool save(const Engine& engine, const std::string& filename);
};

} // namespace Radion

#endif // RADION_ENGINE_SETTINGS_H
