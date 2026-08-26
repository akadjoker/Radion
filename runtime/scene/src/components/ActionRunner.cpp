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

ActionRunner& ActionRunner::moveTo(const Math::vec3& position, f32 seconds)
{
    return append(ActionType::MoveTo, seconds, Math::vec4(position, 0.0f));
}

ActionRunner& ActionRunner::moveBy(const Math::vec3& offset, f32 seconds)
{
    return append(ActionType::MoveBy, seconds, Math::vec4(offset, 0.0f));
}

ActionRunner& ActionRunner::projectile(const Ray& ray, f32 speed, f32 lifetime)
{
    const f32 duration = Math::max(0.0f, lifetime);
    const f32 directionLength = Math::length(ray.direction);
    const Math::vec3 direction =
        directionLength > 0.000001f ? ray.direction / directionLength : Math::vec3(0.0f);
    append(ActionType::SetPosition, 0.0f, Math::vec4(ray.origin, 0.0f));
    moveBy(direction * Math::max(0.0f, speed) * duration, duration);
    return dispose();
}

ActionRunner& ActionRunner::rotateTo(const Math::quat& rotation, f32 seconds)
{
    const Math::quat value = Math::normalize(rotation);
    return append(ActionType::RotateTo, seconds, Math::vec4(value.x, value.y, value.z, value.w));
}

ActionRunner& ActionRunner::rotateBy(const Math::vec3& degrees, f32 seconds)
{
    const Math::quat value = Math::quat(Math::radians(degrees));
    return append(ActionType::RotateBy, seconds, Math::vec4(value.x, value.y, value.z, value.w));
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
    return mCommands.size() - Math::min(mCurrent, mCommands.size());
}

ActionType ActionRunner::current() const
{
    return empty() ? ActionType::Wait : mCommands[mCurrent].type;
}

void ActionRunner::onUpdate(f32 deltaTime)
{
    f32 remaining = Math::max(0.0f, deltaTime);
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

        const f32 consumed = Math::min(remaining, Math::max(0.0f, command.duration - mElapsed));
        mElapsed += consumed;
        const f32 amount =
            command.duration > 0.0f ? Math::min(mElapsed / command.duration, 1.0f) : 1.0f;
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
        mTarget = Math::vec4(mStartPosition + Math::vec3(command.value), 0.0f);
    else if (command.type == ActionType::RotateBy)
    {
        const Math::quat offset(command.value.w, command.value.x, command.value.y, command.value.z);
        const Math::quat target = Math::normalize(mStartRotation * offset);
        mTarget = Math::vec4(target.x, target.y, target.z, target.w);
    }
}

void ActionRunner::apply(const Command& command, f32 amount)
{
    switch (command.type)
    {
    case ActionType::SetPosition:
        owner()->setPosition(Math::vec3(command.value));
        break;
    case ActionType::MoveTo:
    case ActionType::MoveBy:
        owner()->setPosition(Math::mix(mStartPosition, Math::vec3(mTarget), amount));
        break;
    case ActionType::RotateTo:
    case ActionType::RotateBy:
    {
        const Math::quat target(mTarget.w, mTarget.x, mTarget.y, mTarget.z);
        owner()->setRotation(Math::slerp(mStartRotation, target, amount));
        break;
    }
    default:
        break;
    }
}

ActionRunner& ActionRunner::append(ActionType type, f32 duration, const Math::vec4& value)
{
    if (empty())
        clear();
    mCommands.push_back({value, Math::max(0.0f, duration), type});
    return *this;
}

} // namespace Radion
