#include "PCH.h"

#include "SelfDestroy.h"

#include "GameObject.h"

namespace Radion
{

SelfDestroy::SelfDestroy() : Component(Type, ComponentEventUpdate)
{
}

void SelfDestroy::setLifetime(f32 seconds)
{
    mLifetime = seconds;
}
f32 SelfDestroy::lifetime() const
{
    return mLifetime;
}
f32 SelfDestroy::elapsed() const
{
    return mElapsed;
}
f32 SelfDestroy::remaining() const
{
    return Math::max(mLifetime - mElapsed, 0.0f);
}
void SelfDestroy::restart()
{
    mElapsed = 0.0f;
    mDisposed = false;
}

void SelfDestroy::onUpdate(f32 deltaTime)
{
    if (mDisposed)
        return;

    mElapsed += deltaTime;
    if (mElapsed < mLifetime)
        return;

    // dispose() only flags the object for the Scene's end-of-frame sweep,
    // so guard against calling it again every frame the object still lives.
    mDisposed = true;
    owner()->dispose();
}

} // namespace Radion
