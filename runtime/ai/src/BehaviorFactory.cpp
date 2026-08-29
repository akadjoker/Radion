// BehaviorFactory.cpp - create/name/lookup for every registered BehaviorType.

#include "PCH.h"

#include "BehaviorFactory.h"

#include "Behavior.h"
#include "FormationBehavior.h"
#include "NavMeshBehavior.h"
#include "PathfindBehavior.h"
#include "Steering.h"

#include <cstring>

namespace Radion::AI
{

Behavior* BehaviorFactory::create(BehaviorType type)
{
    switch (type)
    {
    case BehaviorType::Separation: return new SeparationBehavior();
    case BehaviorType::Alignment: return new AlignmentBehavior();
    case BehaviorType::Cohesion: return new CohesionBehavior();
    case BehaviorType::Avoidance: return new AvoidanceBehavior();
    case BehaviorType::Cruising: return new CruisingBehavior();
    case BehaviorType::StayWithinSphere: return new StayWithinSphereBehavior();
    case BehaviorType::Combat: return new CombatBehavior();
    case BehaviorType::Seek: return new SeekBehavior();
    case BehaviorType::Flee: return new FleeBehavior();
    case BehaviorType::Wander: return new WanderBehavior();
    case BehaviorType::ObstacleAvoidance: return new ObstacleAvoidanceBehavior();
    case BehaviorType::Pathfind: return new PathfindBehavior();
    case BehaviorType::NavMesh: return new NavMeshBehavior();
    case BehaviorType::Formation: return new FormationBehavior();
    case BehaviorType::Count: break;
    }
    return nullptr;
}

const char* BehaviorFactory::name(BehaviorType type)
{
    switch (type)
    {
    case BehaviorType::Separation: return "Separation";
    case BehaviorType::Alignment: return "Alignment";
    case BehaviorType::Cohesion: return "Cohesion";
    case BehaviorType::Avoidance: return "Avoidance";
    case BehaviorType::Cruising: return "Cruising";
    case BehaviorType::StayWithinSphere: return "StayWithinSphere";
    case BehaviorType::Combat: return "Combat";
    case BehaviorType::Seek: return "Seek";
    case BehaviorType::Flee: return "Flee";
    case BehaviorType::Wander: return "Wander";
    case BehaviorType::ObstacleAvoidance: return "ObstacleAvoidance";
    case BehaviorType::Pathfind: return "Pathfind";
    case BehaviorType::NavMesh: return "NavMesh";
    case BehaviorType::Formation: return "Formation";
    case BehaviorType::Count: break;
    }
    return "";
}

bool BehaviorFactory::fromName(const char* name, BehaviorType& out)
{
    if (!name)
        return false;
    for (u8 i = 0; i < static_cast<u8>(BehaviorType::Count); ++i)
    {
        const BehaviorType candidate = static_cast<BehaviorType>(i);
        if (std::strcmp(name, BehaviorFactory::name(candidate)) == 0)
        {
            out = candidate;
            return true;
        }
    }
    return false;
}

} // namespace Radion::AI
