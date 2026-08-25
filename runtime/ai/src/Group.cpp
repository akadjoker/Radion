// Group.cpp - implementation of the entity group.

#include "PCH.h"

#include "Group.h"

#include "Entity.h"
#include "Log.h"

namespace Radion::AI
{

Group::~Group()
{
    for (Entity* entity : mEntities)
        delete entity;
    mEntities.clear();
}

void Group::add(Entity& entity)
{
    // Rejects an entity already owned by a group - itself or another one.
    // Without this, adding the same entity twice put the same pointer in
    // mEntities twice and the destructor's `delete entity` ran on it twice
    // too; adding it to a second group silently orphaned it in the first,
    // which still holds and will still delete the same pointer.
    if (entity.currentGroup())
    {
        Log::warning("Group: add() called with an entity already owned by a group; ignored");
        return;
    }
    mEntities.push_back(&entity);
    entity.setCurrentGroup(this);
}

void Group::remove(Entity& entity)
{
    auto it = std::find(mEntities.begin(), mEntities.end(), &entity);
    if (it == mEntities.end())
        return; // not this group's entity - do not touch its currentGroup()
    mEntities.erase(it);
    entity.setCurrentGroup(nullptr);
}

void Group::iterate(float timeDelta)
{
    for (Entity* entity : mEntities)
        entity->iterate(timeDelta);
}

} // namespace Radion::AI
