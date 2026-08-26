#ifndef RADION_ACTION_RUNNER_H
#define RADION_ACTION_RUNNER_H

#include "Component.h"
#include "Math.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace Radion
{

enum class ActionType : u8
{
    Wait,
    MoveTo,
    MoveBy,
    SetPosition,
    RotateTo,
    RotateBy,
    Dispose
};

class ActionRunner final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::ActionRunner;

    ActionRunner& wait(f32 seconds);
    ActionRunner& moveTo(const Math::Vec3& position, f32 seconds);
    ActionRunner& moveBy(const Math::Vec3& offset, f32 seconds);
    ActionRunner& projectile(const Ray& ray, f32 speed, f32 lifetime);
    ActionRunner& rotateTo(const Math::Quaternion& rotation, f32 seconds);
    ActionRunner& rotateBy(const Math::Vec3& degrees, f32 seconds);
    ActionRunner& dispose();

    void clear();
    bool empty() const;
    usize count() const;
    ActionType current() const;

private:
    friend class GameObject;

    struct Command
    {
        Math::Vec4 value = Math::Vec4(0.0f);
        f32 duration = 0.0f;
        ActionType type = ActionType::Wait;
    };

    ActionRunner();
    void onUpdate(f32 deltaTime) override;
    void begin(const Command& command);
    void apply(const Command& command, f32 amount);
    ActionRunner& append(ActionType type, f32 duration = 0.0f,
                         const Math::Vec4& value = Math::Vec4(0.0f));

    std::vector<Command> mCommands;
    usize mCurrent = 0;
    f32 mElapsed = 0.0f;
    Math::Vec3 mStartPosition = Math::Vec3(0.0f);
    Math::Quaternion mStartRotation = Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    Math::Vec4 mTarget = Math::Vec4(0.0f);
    bool mStarted = false;
};

} // namespace Radion

#endif // RADION_ACTION_RUNNER_H
