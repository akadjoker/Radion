#ifndef RADION_AI_BEHAVIORFACTORY_H
#define RADION_AI_BEHAVIORFACTORY_H

// BehaviorFactory.h - the registry of concrete Behavior subclasses: an enum
// naming each one, and a factory that creates/names/looks them up by that
// name. What lets the editor and the serializer add/list/save a behavior by
// type instead of a caller writing `new SeparationBehavior(...)` in code.
//
// SteerBehavior (Steering.h) is not registered here: it wraps a
// std::function supplied from C++ and has no by-name meaning for an editor
// combo or a save file.

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
    // A heap-allocated, default-configured instance of `type`, ready for
    // Agent::addBehavior() - or null for BehaviorType::Count.
    static Behavior* create(BehaviorType type);

    // Bare enumerator spelling ("Separation"), for the editor combo and the
    // serializer's "type" field.
    static const char* name(BehaviorType type);

    // Reverse of name(): true and `out` set on a match, false (and `out`
    // untouched) otherwise.
    static bool fromName(const char* name, BehaviorType& out);
};

} // namespace Radion::AI

#endif // RADION_AI_BEHAVIORFACTORY_H
