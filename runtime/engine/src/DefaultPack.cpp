#include "PCH.h"

#include "DefaultPack.h"

#include "FileSystem.h"
#include "Log.h"

namespace Radion
{

#if defined(RADION_HAS_DEFAULT_PACK)
namespace Generated
{
extern const unsigned char kDefaultPackData[];
extern const unsigned long long kDefaultPackSize;
extern const char* const kDefaultPackKey;
} // namespace Generated
#endif

bool DefaultPack::available()
{
#if defined(RADION_HAS_DEFAULT_PACK)
    return Generated::kDefaultPackSize > 0;
#else
    return false;
#endif
}

unsigned long long DefaultPack::size()
{
#if defined(RADION_HAS_DEFAULT_PACK)
    return Generated::kDefaultPackSize;
#else
    return 0;
#endif
}

bool DefaultPack::mount(FileSystem& files)
{
#if defined(RADION_HAS_DEFAULT_PACK)
    if (Generated::kDefaultPackSize == 0)
        return false;

    if (!files.mountFallbackPack(Generated::kDefaultPackData,
                                 static_cast<usize>(Generated::kDefaultPackSize),
                                 Generated::kDefaultPackKey))
        return false;

    Log::info("DefaultPack: %llu bytes embedded, mounted behind the search paths",
              Generated::kDefaultPackSize);
    return true;
#else
    (void)files;
    return false;
#endif
}

} // namespace Radion
