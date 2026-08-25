#ifndef RADION_DEFAULT_PACK_H
#define RADION_DEFAULT_PACK_H

namespace Radion
{

class FileSystem;

// The shaders and lens flare textures compiled into the binary, so a build
// runs with no assets folder beside it. Mounted as a fallback, never as an
// override: anything found through a search path wins, which is what keeps
// editing a shader on disk working the way it always did.
class DefaultPack
{
public:
    static bool available();
    // Safe to call on a build with nothing embedded - returns false and
    // leaves `files` alone.
    static bool mount(FileSystem& files);
    // Bytes of the embedded pack, 0 when there is none.
    static unsigned long long size();
};

} // namespace Radion

#endif // RADION_DEFAULT_PACK_H
