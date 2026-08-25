// World.cpp - implementation of the AI world container.

#include "PCH.h"

#include "World.h"

#include "Group.h"
#include "Log.h"

namespace Radion::AI
{

World::~World()
{
    for (Group* group : mGroups)
        delete group;
    mGroups.clear();
}

void World::add(Group& group)
{
    // Rejects a group already in this world: without this, the same pointer
    // landed in mGroups twice and the destructor's `delete group` ran on it
    // twice too.
    if (std::find(mGroups.begin(), mGroups.end(), &group) != mGroups.end())
    {
        Log::warning("World: add() called with a group already owned by this world; ignored");
        return;
    }
    mGroups.push_back(&group);
}

void World::remove(Group& group)
{
    auto it = std::find(mGroups.begin(), mGroups.end(), &group);
    if (it != mGroups.end())
        mGroups.erase(it);
}

void World::iterate(float timeDelta)
{
    for (Group* group : mGroups)
        group->iterate(timeDelta);
}

} // namespace Radion::AI
