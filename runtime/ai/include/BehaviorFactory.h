#ifndef RADION_AI_BEHAVIORFACTORY_H
#define RADION_AI_BEHAVIORFACTORY_H

// SteerBehavior is not registered: it wraps a std::function with no by-name meaning.

#include "Types.h"

namespace Radion::AI
{

class Behavior;

enum class BehaviorType : u8
{
    Separation,
    Alignment,
    Cohesion,
    Avoidance,
    Cruising,
    StayWithinSphere,
    Combat,
    Seek,
    Flee,
    Wander,
    ObstacleAvoidance,
    Pathfind,
    NavMesh,
    Formation,
    Count
};

class BehaviorFactory
{
public:
    // Null for BehaviorType::Count.
    static Behavior* create(BehaviorType type);

    static const char* name(BehaviorType type);

    // On a match true and `out` set; otherwise false, `out` untouched.
    static bool fromName(const char* name, BehaviorType& out);
};

} // namespace Radion::AI

#endif // RADION_AI_BEHAVIORFACTORY_H
