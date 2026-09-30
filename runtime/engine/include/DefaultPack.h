#ifndef RADION_DEFAULT_PACK_H
#define RADION_DEFAULT_PACK_H

namespace Radion
{

class FileSystem;

// Embedded shaders/textures, mounted as a fallback only: anything on a search path wins.
class DefaultPack
{
public:
    static bool available();
    // Returns false and leaves `files` alone when nothing is embedded.
    static bool mount(FileSystem& files);
    // Bytes of the embedded pack, 0 when there is none.
    static unsigned long long size();
};

} // namespace Radion

#endif // RADION_DEFAULT_PACK_H
