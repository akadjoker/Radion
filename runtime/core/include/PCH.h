#ifndef RADION_PCH_H
#define RADION_PCH_H

// Common C++ headers used throughout the desktop engine. Public headers must
// remain self-contained and include their own direct dependencies.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Expensive, stable, and used nearly everywhere. GL and SDL stay out: only
// gpu/backends/gl and core need them, and putting them here would spread them
// across the whole engine.
#include "Math.h"

#include "Log.h"
#include "Types.h"

#endif // RADION_PCH_H
