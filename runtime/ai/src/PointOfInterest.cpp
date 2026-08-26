// PointOfInterest.cpp - implementation of the PointsOfInterest registry.

#include "PCH.h"

#include "PointOfInterest.h"

#include <cfloat>
#include <cstdlib>

namespace Radion::AI
{

namespace
{
PointOfInterestID nextPointOfInterestId()
{
    static PointOfInterestID counter = 1; // 0 is reserved as the null/invalid id
    return counter++;
}
} // namespace

// PointOfInterest id is assigned lazily on first use to keep the default
// constructor trivial; add() is the only place that needs it.
PointOfInterestID PointOfInterest::id() const
{
    if (mId == 0)
        mId = nextPointOfInterestId();
    return mId;
}

bool PointsOfInterest::add(PointOfInterest* poi)
{
    PointOfInterestID id = poi->id();
    if (mPois.find(id) == mPois.end())
    {
        mPois[id] = poi;
        return true;
    }
    return false;
}

bool PointsOfInterest::remove(PointOfInterestID poiID)
{
    auto it = mPois.find(poiID);
    if (it == mPois.end())
        return false;

    delete it->second;
    mPois.erase(it);
    return true;
}

void PointsOfInterest::clear()
{
    for (auto& kv : mPois)
        delete kv.second;
    mPois.clear();
}

PointOfInterest* PointsOfInterest::find(PointOfInterestID poiID) const
{
    auto it = mPois.find(poiID);
    return it == mPois.end() ? nullptr : it->second;
}

PointOfInterest* PointsOfInterest::selectRandom(PointOfInterestID current) const
{
    if (mPois.empty())
        return nullptr;
    if (mPois.size() == 1)
        return mPois.begin()->second;

    std::vector<PointOfInterestID> ids;
    ids.reserve(mPois.size());
    for (const auto& kv : mPois)
        ids.push_back(kv.first);

    PointOfInterestID newId = current;
    while (newId == current)
        newId = ids[std::rand() % static_cast<int>(ids.size())];

    return find(newId);
}

PointOfInterest* PointsOfInterest::findNearest(const glm::vec3& position) const
{
    PointOfInterest* closest = nullptr;
    float closestDist = FLT_MAX;
    for (const auto& kv : mPois)
    {
        PointOfInterest* poi = kv.second;
        float dist = glm::length(poi->position() - position);
        if (dist < closestDist)
        {
            closestDist = dist;
            closest = poi;
        }
    }
    return closest;
}

} // namespace Radion::AI
