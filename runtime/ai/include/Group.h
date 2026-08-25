#ifndef RADION_AI_GROUP_H
#define RADION_AI_GROUP_H

// Group.h - a collection of entities, usually a flock/squad.
//
// The Group OWNS the entities added via add() and deletes them on
// destruction.

#include <vector>

namespace Radion::AI
{

class Entity;
class World;

class Group
{
public:
    explicit Group(World& world) : mWorld(world)
    {
    }
    virtual ~Group();

    // Ownership: the group takes ownership of entity, which must therefore
    // be heap-allocated and not already owned by a group - add() rejects an
    // entity that already has one (itself or another), but a stack object is
    // still a caller bug this cannot catch.
    void add(Entity& entity);
    void remove(Entity& entity);

    std::vector<Entity*>& entities()
    {
        return mEntities;
    }
    const std::vector<Entity*>& entities() const
    {
        return mEntities;
    }

    virtual void iterate(float timeDelta);

protected:
    std::vector<Entity*> mEntities;
    World& mWorld;
};

} // namespace Radion::AI

#endif // RADION_AI_GROUP_H
