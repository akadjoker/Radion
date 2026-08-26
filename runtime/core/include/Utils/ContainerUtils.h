#ifndef RADION_UTILS_CONTAINER_UTILS_H
#define RADION_UTILS_CONTAINER_UTILS_H

#include "Types.h"

namespace Radion
{
namespace Utils
{
// Removes the first matching element in O(1) after the lookup by moving the
// last element into its slot. Deliberately does not preserve order; this is
// the useful semantic for Scene's registration lists and works with both
// std::vector during the migration and ct::Vector afterwards.
template <typename Sequence, typename T> bool eraseUnordered(Sequence& values, const T& value)
{
    for (usize index = 0; index < values.size(); ++index)
    {
        if (values[index] != value)
            continue;
        values[index] = values.back();
        values.pop_back();
        return true;
    }
    return false;
}
} // namespace Utils
} // namespace Radion

#endif // RADION_UTILS_CONTAINER_UTILS_H
