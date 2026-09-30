#ifndef RADION_PCH_H
#define RADION_PCH_H

// Public headers must stay self-contained.
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

// GL and SDL stay out: only gpu/backends/gl and core need them.
#include "Math.h"

#include "Log.h"
#include "Types.h"

#endif // RADION_PCH_H
