#ifndef RADION_AI_WORLD_H
#define RADION_AI_WORLD_H

// World.h - the container that drives the whole flocking simulation.
//
// The World OWNS the groups added via add() and deletes them on destruction.
// iterate() steps every group, which steps every entity.

#include <vector>

namespace Radion::AI
{

class Group;

class World
{
public:
    World() = default;
    virtual ~World();

    // Ownership: the world takes ownership of group, which must therefore be
    // heap-allocated and not already owned elsewhere - add() rejects a group
    // already in this world, but a group added to two different worlds, or a
    // stack object, is still a caller bug this cannot catch.
    void add(Group& group);
    void remove(Group& group);

    std::vector<Group*>& groups()
    {
        return mGroups;
    }
    const std::vector<Group*>& groups() const
    {
        return mGroups;
    }

    virtual void iterate(float timeDelta);

protected:
    std::vector<Group*> mGroups;
};

} // namespace Radion::AI

#endif // RADION_AI_WORLD_H
