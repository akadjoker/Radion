#ifndef RADION_BLENDER_API_BLENDER_COMMANDS_H
#define RADION_BLENDER_API_BLENDER_COMMANDS_H

#include "api/CommandRegistry.h"

namespace Radion
{
class BlenderApplication;
}

namespace Radion::BlenderApi
{

// Registers every command that drives the editor. The handlers run on the
// frame-loop thread (see MainThreadQueue) and call the same BlenderApplication
// operations the menus do, so an edit made through the API is one undo step and
// shows up in the viewport like any other.
//
// `app` must outlive the registry.
void registerBlenderCommands(CommandRegistry& registry, BlenderApplication& app);

} // namespace Radion::BlenderApi

#endif // RADION_BLENDER_API_BLENDER_COMMANDS_H
