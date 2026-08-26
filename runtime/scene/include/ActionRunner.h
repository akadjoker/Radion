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
    ActionRunner& moveTo(const glm::vec3& position, f32 seconds);
    ActionRunner& moveBy(const glm::vec3& offset, f32 seconds);
    ActionRunner& projectile(const Ray& ray, f32 speed, f32 lifetime);
    ActionRunner& rotateTo(const glm::quat& rotation, f32 seconds);
    ActionRunner& rotateBy(const glm::vec3& degrees, f32 seconds);
    ActionRunner& dispose();

    void clear();
    bool empty() const;
    usize count() const;
    ActionType current() const;

private:
    friend class GameObject;

    struct Command
    {
        glm::vec4 value = glm::vec4(0.0f);
        f32 duration = 0.0f;
        ActionType type = ActionType::Wait;
    };

    ActionRunner();
    void onUpdate(f32 deltaTime) override;
    void begin(const Command& command);
    void apply(const Command& command, f32 amount);
    ActionRunner& append(ActionType type, f32 duration = 0.0f,
                         const glm::vec4& value = glm::vec4(0.0f));

    std::vector<Command> mCommands;
    usize mCurrent = 0;
    f32 mElapsed = 0.0f;
    glm::vec3 mStartPosition = glm::vec3(0.0f);
    glm::quat mStartRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    glm::vec4 mTarget = glm::vec4(0.0f);
    bool mStarted = false;
};

} // namespace Radion

#endif // RADION_ACTION_RUNNER_H
