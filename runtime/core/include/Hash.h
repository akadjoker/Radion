#ifndef RADION_HASH_H
#define RADION_HASH_H

#include "Types.h"

#include <cstddef>

namespace Radion
{

// FNV-1a, 32-bit. Not for anything security-sensitive
constexpr u32 hashName(const char* text)
{
    u32 hash = 2166136261u;
    for (; *text; ++text)
        hash = (hash ^ static_cast<u32>(*text)) * 16777619u;
    return hash;
}

} // namespace Radion

#endif // RADION_HASH_H
