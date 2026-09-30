#ifndef RADION_AI_STATEMACHINE_H
#define RADION_AI_STATEMACHINE_H

// iterate(): the first transition that fires exits the current state, enters the target and stops the scan.

#include <string>
#include <vector>

namespace Radion::AI
{

class State;

class StateMachine
{
public:
    using StateList = std::vector<State*>;

    StateMachine() = default;
    virtual ~StateMachine();

    // Owns every state added; deleted on destruction or removal.
    void addState(State* state);
    void removeState(State* state);
    State* findState(const std::string& name) const;
    State* currentState() const
    {
        return mCurrentState;
    }

    void iterate();
    void reset(); // current = first state (or keeps current), then reset it

    StateList& states()
    {
        return mStates;
    }
    const StateList& states() const
    {
        return mStates;
    }

    std::string toDot() const;

protected:
    // Factory hook so derived machines can build State subclasses; the machine owns the result.
    virtual State* constructState(const std::string& name);

    StateList mStates;
    State* mCurrentState = nullptr;
};

} // namespace Radion::AI

#endif // RADION_AI_STATEMACHINE_H
