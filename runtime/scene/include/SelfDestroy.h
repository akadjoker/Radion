#ifndef RADION_SELFDESTROY_H
#define RADION_SELFDESTROY_H

#include "Component.h"

namespace Radion
{

// A countdown that disposes its own GameObject once its lifetime elapses -
// a bullet impact effect or an explosion light attaches this to clean
// itself up without any outside script watching it.
class SelfDestroy final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::SelfDestroy;

    void setLifetime(f32 seconds);
    f32 lifetime() const;
    f32 elapsed() const;
    f32 remaining() const;
    void restart();

private:
    friend class GameObject;

    SelfDestroy();
    void onUpdate(f32 deltaTime) override;

    f32 mLifetime = 1.0f;
    f32 mElapsed = 0.0f;
    bool mDisposed = false;
};

} // namespace Radion

#endif // RADION_SELFDESTROY_H
