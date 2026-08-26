#include "PCH.h"

#include "collision/Broadphase.h"

#include <algorithm>

namespace Radion::Physics
{

void Broadphase::clear()
{
    mProxies.clear();
    mOrder.clear();
}

void Broadphase::reserve(usize count)
{
    mProxies.reserve(count);
    mOrder.reserve(count);
}

void Broadphase::add(const BroadphaseProxy& proxy)
{
    mProxies.push_back(proxy);
}

bool Broadphase::overlaps(const AABB& a, const AABB& b)
{
    return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y && a.max.y >= b.min.y &&
           a.min.z <= b.max.z && a.max.z >= b.min.z;
}

void Broadphase::findPairs(std::vector<BroadphasePair>& out)
{
    out.clear();
    const usize count = mProxies.size();
    if (count < 2)
        return;

    // Sweep along whichever axis the bodies are most spread over: variance,
    // not extent, because one distant outlier stretches the extent without
    // separating anything.
    glm::vec3 sum(0.0f);
    glm::vec3 sumSquared(0.0f);
    bool hasMovableProxy = false;
    for (const BroadphaseProxy& proxy : mProxies)
    {
        const glm::vec3 center = (proxy.bounds.min + proxy.bounds.max) * 0.5f;
        sum += center;
        sumSquared += center * center;
        hasMovableProxy = hasMovableProxy || proxy.movable;
    }
    if (!hasMovableProxy)
        return;
    const f32 inverse = 1.0f / static_cast<f32>(count);
    const glm::vec3 mean = sum * inverse;
    const glm::vec3 variance = sumSquared * inverse - mean * mean;
    mSweepAxis = 0;
    if (variance.y > variance.x && variance.y >= variance.z)
        mSweepAxis = 1;
    else if (variance.z > variance.x && variance.z > variance.y)
        mSweepAxis = 2;

    mOrder.resize(count);
    for (usize i = 0; i < count; ++i)
        mOrder[i] = static_cast<u32>(i);

    const std::vector<BroadphaseProxy>& proxies = mProxies;
    const u32 axis = mSweepAxis;
    std::sort(mOrder.begin(), mOrder.end(), [&proxies, axis](u32 left, u32 right)
              { return proxies[left].bounds.min[axis] < proxies[right].bounds.min[axis]; });

    for (usize i = 0; i < count; ++i)
    {
        const BroadphaseProxy& first = mProxies[mOrder[i]];
        const f32 end = first.bounds.max[axis];
        for (usize j = i + 1; j < count; ++j)
        {
            const BroadphaseProxy& second = mProxies[mOrder[j]];
            // Sorted by lower bound, so once one starts past where this one
            // ends, so does everything after it.
            if (second.bounds.min[axis] > end)
                break;
            if (!first.movable && !second.movable)
                continue;
            if (!shouldCollide(first.filter, second.filter))
                continue;
            if (!overlaps(first.bounds, second.bounds))
                continue;
            BroadphasePair pair;
            pair.a = glm::min(first.id, second.id);
            pair.b = glm::max(first.id, second.id);
            out.push_back(pair);
        }
    }
}

} // namespace Radion::Physics
