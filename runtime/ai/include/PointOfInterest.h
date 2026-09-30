#ifndef RADION_AI_POINTOFINTEREST_H
#define RADION_AI_POINTOFINTEREST_H

#include "Types.h"

#include "Math.h"
#include <unordered_map>
#include <vector>

namespace Radion::AI
{

using PointOfInterestID = Radion::u32;

class PointOfInterest
{
public:
    PointOfInterest() = default;
    PointOfInterest(const Math::vec3& position, float radius) : mPosition(position), mRadius(radius)
    {
    }

    // Assigned lazily so default-constructed POIs do not consume an id.
    PointOfInterestID id() const;

    const Math::vec3& position() const
    {
        return mPosition;
    }
    void setPosition(const Math::vec3& position)
    {
        mPosition = position;
    }
    float radius() const
    {
        return mRadius;
    }
    void setRadius(float radius)
    {
        mRadius = radius;
    }

private:
    mutable PointOfInterestID mId = 0;
    Math::vec3 mPosition = Math::vec3(0.0f);
    float mRadius = 1.0f;
};

// Registry of points of interest. OWNS the POIs added via add() and deletes
// them on remove/clear/destruction.
class PointsOfInterest
{
public:
    using Map = std::unordered_map<PointOfInterestID, PointOfInterest*>;

    PointsOfInterest() = default;
    ~PointsOfInterest()
    {
        clear();
    }

    bool add(PointOfInterest* poi);
    bool remove(PointOfInterestID poiID);
    void clear();
    PointOfInterest* find(PointOfInterestID poiID) const;

    PointOfInterest* selectRandom(PointOfInterestID current) const;

    PointOfInterest* findNearest(const Math::vec3& position) const;

    const Map& map() const
    {
        return mPois;
    }

private:
    Map mPois;
};

} // namespace Radion::AI

#endif // RADION_AI_POINTOFINTEREST_H
