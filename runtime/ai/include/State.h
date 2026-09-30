#ifndef RADION_AI_STATE_H
#define RADION_AI_STATE_H

// The State OWNS its actions and transitions.

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

    // Takes ownership of the actions and transitions passed here.
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
