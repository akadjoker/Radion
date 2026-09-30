#ifndef RADION_BLENDER_API_BLENDER_COMMANDS_H
#define RADION_BLENDER_API_BLENDER_COMMANDS_H

#include "api/CommandRegistry.h"

namespace Radion
{
class BlenderApplication;
}

namespace Radion::BlenderApi
{

// Handlers run on the frame-loop thread; `app` must outlive the registry.
void registerBlenderCommands(CommandRegistry& registry, BlenderApplication& app);

} // namespace Radion::BlenderApi

#endif // RADION_BLENDER_API_BLENDER_COMMANDS_H
