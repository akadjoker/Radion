#ifndef RADION_AI_ACTION_H
#define RADION_AI_ACTION_H



#include "State.h"

#include <functional>

namespace Radion::AI
{

class State;

// Base class for all state actions.
class Action
{
public:
    explicit Action(State* state) : mState(state)
    {
    }
    virtual ~Action() = default;

    virtual void execute()
    {
    }
    virtual const char* label() const
    {
        return "Base Action";
    }

    State& state() const
    {
        return *mState;
    }
    State* statePtr() const
    {
        return mState;
    }

protected:
    State* mState; // non-owning; owned by the parent State
};

// Applies a numeric operation to the owning state's scalar value.
class ValueBasedAction : public Action
{
public:
    ValueBasedAction(State* state, float value) : Action(state), mValue(value)
    {
    }
    virtual ~ValueBasedAction() = default;

    float parameter() const
    {
        return mValue;
    }

protected:
    float mValue;
};

class AddAction final : public ValueBasedAction
{
public:
    AddAction(State* state, float value) : ValueBasedAction(state, value)
    {
    }
    void execute() override
    {
        state().setValue(state().value() + mValue);
    }
    const char* label() const override
    {
        return "Add to Value";
    }
};

class SubtractAction final : public ValueBasedAction
{
public:
    SubtractAction(State* state, float value) : ValueBasedAction(state, value)
    {
    }
    void execute() override
    {
        state().setValue(state().value() - mValue);
    }
    const char* label() const override
    {
        return "Subtract from Value";
    }
};

class InverseSubtractAction final : public ValueBasedAction
{
public:
    InverseSubtractAction(State* state, float value) : ValueBasedAction(state, value)
    {
    }
    void execute() override
    {
        state().setValue(mValue - state().value());
    }
    const char* label() const override
    {
        return "Subtract Value from Constant";
    }
};

class MultiplyAction final : public ValueBasedAction
{
public:
    MultiplyAction(State* state, float value) : ValueBasedAction(state, value)
    {
    }
    void execute() override
    {
        state().setValue(state().value() * mValue);
    }
    const char* label() const override
    {
        return "Multiply Value";
    }
};

class DivideAction final : public ValueBasedAction
{
public:
    DivideAction(State* state, float value) : ValueBasedAction(state, value)
    {
    }
    void execute() override
    {
        state().setValue(state().value() / mValue);
    }
    const char* label() const override
    {
        return "Divide Value";
    }
};

class InverseDivideAction final : public ValueBasedAction
{
public:
    InverseDivideAction(State* state, float value) : ValueBasedAction(state, value)
    {
    }
    void execute() override
    {
        state().setValue(mValue / state().value());
    }
    const char* label() const override
    {
        return "Divide Constant by Value";
    }
};

// Callback action - the C++ replacement for the demo's Python scripted
// actions (GI_AISDK ScriptedAction). A user functor is executed each time the
// action runs; capture the owning entity in the lambda for entity-driven AI.
class CallbackAction final : public Action
{
public:
    using Callback = std::function<void(State&)>;

    CallbackAction(State* state, Callback callback) : Action(state), mCallback(std::move(callback))
    {
    }
    void execute() override
    {
        if (mCallback)
            mCallback(state());
    }
    const char* label() const override
    {
        return "Callback Action";
    }

private:
    Callback mCallback;
};

} // namespace Radion::AI

#endif // RADION_AI_ACTION_H
