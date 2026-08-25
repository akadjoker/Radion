#ifndef RADION_CONTAINERS_H
#define RADION_CONTAINERS_H

#include "Types.h"

#include <unordered_map>
#include <vector>

namespace Radion
{

template <typename K, typename V, typename Hash = std::hash<K>>
using HashMap = std::unordered_map<K, V, Hash>;

// A borrowed reference to something a pool owns. Generation 0 never belongs to
// a live slot, so a default-built handle is always invalid.
template <typename Tag> struct Handle
{
    u32 index = 0;
    u32 generation = 0;

    bool valid() const
    {
        return generation != 0;
    }

    explicit operator bool() const
    {
        return valid();
    }

    bool operator==(const Handle& other) const
    {
        return index == other.index && generation == other.generation;
    }

    bool operator!=(const Handle& other) const
    {
        return !(*this == other);
    }
};

// Packs a handle into one integer so it can key a HashMap - a Handle itself
// has no std::hash, and index alone would collide across generations.
template <typename Tag> u64 packHandle(Handle<Tag> handle)
{
    return (static_cast<u64>(handle.index) << 32) | static_cast<u64>(handle.generation);
}

// Slots are recycled but generations are not: a handle kept past destruction
// finds a live slot with a different generation and is rejected instead of
// silently addressing whatever took its place.
template <typename T, typename H> class Pool
{
public:
    H add(const T& value)
    {
        u32 index;
        if (!mFree.empty())
        {
            index = mFree.back();
            mFree.pop_back();
        }
        else
        {
            index = static_cast<u32>(mSlots.size());
            mSlots.push_back(Slot());
        }

        Slot& slot = mSlots[index];
        slot.value = value;
        slot.alive = true;
        if (slot.generation == 0)
            slot.generation = 1;

        H handle;
        handle.index = index;
        handle.generation = slot.generation;
        return handle;
    }

    T* get(H handle)
    {
        if (handle.generation == 0 || handle.index >= mSlots.size())
            return nullptr;
        Slot& slot = mSlots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return nullptr;
        return &slot.value;
    }

    const T* get(H handle) const
    {
        return const_cast<Pool*>(this)->get(handle);
    }

    // Copies the value out before freeing so the caller can still release what
    // it owned, and refuses stale handles.
    bool remove(H handle, T& out)
    {
        T* value = get(handle);
        if (!value)
            return false;

        out = *value;
        Slot& slot = mSlots[handle.index];
        slot.alive = false;
        slot.value = T();
        ++slot.generation;
        if (slot.generation == 0)
            slot.generation = 1;
        mFree.push_back(handle.index);
        return true;
    }

    // Walks every live slot, for shutdown and for stats.
    template <typename Fn> void forEach(Fn function)
    {
        for (usize i = 0; i < mSlots.size(); ++i)
        {
            if (mSlots[i].alive)
                function(mSlots[i].value);
        }
    }

    usize liveCount() const
    {
        return mSlots.size() - mFree.size();
    }

    usize capacity() const
    {
        return mSlots.size();
    }

    void clear()
    {
        mSlots.clear();
        mFree.clear();
    }

private:
    struct Slot
    {
        T value = T();
        u32 generation = 0;
        bool alive = false;
    };

    std::vector<Slot> mSlots;
    std::vector<u32> mFree;
};

} // namespace Radion

#endif // RADION_CONTAINERS_H
