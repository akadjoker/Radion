#ifndef RADION_AI_STATE_H
#define RADION_AI_STATE_H

// State.h - a single state of the Radion AI state machine.
//
// A state carries a scalar value (drives ComparitorTransition), a name, a set
// of per-frame actions, enter actions and exit actions, plus outgoing
// transitions. The State OWNS its actions and transitions and frees them on
// destruction.

#include <string>
#include <vector>

namespace Radion::AI
{

class Action;
class Transition;

class State
{
public:
    explicit State(const std::string& name) : mName(name)
    {
    }
    State(float initialValue, const std::string& name)
        : mValue(initialValue), mInitialValue(initialValue), mName(name)
    {
    }
    virtual ~State();

    // Scalar value the transitions can test against.
    float value() const
    {
        return mValue;
    }
    float initialValue() const
    {
        return mInitialValue;
    }
    void setValue(float value)
    {
        mValue = value;
    }
    void setInitialValue(float value)
    {
        mInitialValue = value;
    }

    // Lifecycle. reset() restores the value and re-enters the state.
    void reset();
    void enter();
    void iterate();
    void exit();

    const std::string& name() const
    {
        return mName;
    }
    void setName(const std::string& name)
    {
        mName = name;
    }

    // Ownership: the State takes ownership of actions and transitions passed
    // here and deletes them in its destructor.
    void addAction(Action* action)
    {
        mActions.push_back(action);
    }
    void addEnterAction(Action* action)
    {
        mEnterActions.push_back(action);
    }
    void addExitAction(Action* action)
    {
        mExitActions.push_back(action);
    }
    void addTransition(Transition* transition)
    {
        mTransitions.push_back(transition);
    }

    std::vector<Action*>& actions()
    {
        return mActions;
    }
    const std::vector<Action*>& actions() const
    {
        return mActions;
    }
    std::vector<Action*>& enterActions()
    {
        return mEnterActions;
    }
    const std::vector<Action*>& enterActions() const
    {
        return mEnterActions;
    }
    std::vector<Action*>& exitActions()
    {
        return mExitActions;
    }
    const std::vector<Action*>& exitActions() const
    {
        return mExitActions;
    }
    std::vector<Transition*>& transitions()
    {
        return mTransitions;
    }
    const std::vector<Transition*>& transitions() const
    {
        return mTransitions;
    }

protected:
    float mValue = 0.0f;
    float mInitialValue = 0.0f;
    std::vector<Transition*> mTransitions;
    std::vector<Action*> mActions;
    std::vector<Action*> mEnterActions;
    std::vector<Action*> mExitActions;
    std::string mName;
};

} // namespace Radion::AI

#endif // RADION_AI_STATE_H
