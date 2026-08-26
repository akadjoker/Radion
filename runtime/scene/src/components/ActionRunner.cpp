#include "PCH.h"

#include "ActionRunner.h"

#include "GameObject.h"

namespace Radion
{

ActionRunner::ActionRunner() : Component(Type, ComponentEventUpdate)
{
    mCommands.reserve(8);
}

ActionRunner& ActionRunner::wait(f32 seconds)
{
    return append(ActionType::Wait, seconds);
}

ActionRunner& ActionRunner::moveTo(const glm::vec3& position, f32 seconds)
{
    return append(ActionType::MoveTo, seconds, glm::vec4(position, 0.0f));
}

ActionRunner& ActionRunner::moveBy(const glm::vec3& offset, f32 seconds)
{
    return append(ActionType::MoveBy, seconds, glm::vec4(offset, 0.0f));
}

ActionRunner& ActionRunner::projectile(const Ray& ray, f32 speed, f32 lifetime)
{
    const f32 duration = glm::max(0.0f, lifetime);
    const f32 directionLength = glm::length(ray.direction);
    const glm::vec3 direction =
        directionLength > 0.000001f ? ray.direction / directionLength : glm::vec3(0.0f);
    append(ActionType::SetPosition, 0.0f, glm::vec4(ray.origin, 0.0f));
    moveBy(direction * glm::max(0.0f, speed) * duration, duration);
    return dispose();
}

ActionRunner& ActionRunner::rotateTo(const glm::quat& rotation, f32 seconds)
{
    const glm::quat value = glm::normalize(rotation);
    return append(ActionType::RotateTo, seconds, glm::vec4(value.x, value.y, value.z, value.w));
}

ActionRunner& ActionRunner::rotateBy(const glm::vec3& degrees, f32 seconds)
{
    const glm::quat value = glm::quat(glm::radians(degrees));
    return append(ActionType::RotateBy, seconds, glm::vec4(value.x, value.y, value.z, value.w));
}

ActionRunner& ActionRunner::dispose()
{
    return append(ActionType::Dispose);
}

void ActionRunner::clear()
{
    mCommands.clear();
    mCurrent = 0;
    mElapsed = 0.0f;
    mStarted = false;
}

bool ActionRunner::empty() const
{
    return mCurrent >= mCommands.size();
}

usize ActionRunner::count() const
{
    return mCommands.size() - glm::min(mCurrent, mCommands.size());
}

ActionType ActionRunner::current() const
{
    return empty() ? ActionType::Wait : mCommands[mCurrent].type;
}

void ActionRunner::onUpdate(f32 deltaTime)
{
    f32 remaining = glm::max(0.0f, deltaTime);
    while (mCurrent < mCommands.size())
    {
        const Command& command = mCommands[mCurrent];
        if (!mStarted)
            begin(command);
        if (command.type == ActionType::Dispose)
        {
            owner()->dispose();
            clear();
            return;
        }

        const f32 consumed = glm::min(remaining, glm::max(0.0f, command.duration - mElapsed));
        mElapsed += consumed;
        const f32 amount =
            command.duration > 0.0f ? glm::min(mElapsed / command.duration, 1.0f) : 1.0f;
        apply(command, amount);
        remaining -= consumed;

        if (mElapsed < command.duration)
            return;
        ++mCurrent;
        mElapsed = 0.0f;
        mStarted = false;
        if (remaining <= 0.0f && mCurrent < mCommands.size() && mCommands[mCurrent].duration > 0.0f)
            return;
    }
    clear();
}

void ActionRunner::begin(const Command& command)
{
    mStarted = true;
    mStartPosition = owner()->position();
    mStartRotation = owner()->rotation();
    mTarget = command.value;
    if (command.type == ActionType::MoveBy)
        mTarget = glm::vec4(mStartPosition + glm::vec3(command.value), 0.0f);
    else if (command.type == ActionType::RotateBy)
    {
        const glm::quat offset(command.value.w, command.value.x, command.value.y, command.value.z);
        const glm::quat target = glm::normalize(mStartRotation * offset);
        mTarget = glm::vec4(target.x, target.y, target.z, target.w);
    }
}

void ActionRunner::apply(const Command& command, f32 amount)
{
    switch (command.type)
    {
    case ActionType::SetPosition:
        owner()->setPosition(glm::vec3(command.value));
        break;
    case ActionType::MoveTo:
    case ActionType::MoveBy:
        owner()->setPosition(glm::mix(mStartPosition, glm::vec3(mTarget), amount));
        break;
    case ActionType::RotateTo:
    case ActionType::RotateBy:
    {
        const glm::quat target(mTarget.w, mTarget.x, mTarget.y, mTarget.z);
        owner()->setRotation(glm::slerp(mStartRotation, target, amount));
        break;
    }
    default:
        break;
    }
}

ActionRunner& ActionRunner::append(ActionType type, f32 duration, const glm::vec4& value)
{
    if (empty())
        clear();
    mCommands.push_back({value, glm::max(0.0f, duration), type});
    return *this;
}

} // namespace Radion
