#ifndef RADION_AI_STATEMACHINE_H
#define RADION_AI_STATEMACHINE_H

// StateMachine.h - finite state machine for the Radion AI library.
//
// A StateMachine owns a set of States (raw pointers, deleted on destruction).
// iterate() runs the current state's per-frame actions and then checks its
// transitions; the first transition that fires exits the current state,
// enters the target state and stops the scan.

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

    // Ownership: the StateMachine owns every state added via addState() and
    // deletes them on destruction or when removed.
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

    // Graphviz .dot dump of states and transitions, for debugging.
    std::string toDot() const;

protected:
    // Factory hook (mirrors cStateMachine::ConstructState) so derived machines
    // can build State subclasses. Returns a new State owned by the machine.
    virtual State* constructState(const std::string& name);

    StateList mStates;
    State* mCurrentState = nullptr;
};

} // namespace Radion::AI

#endif // RADION_AI_STATEMACHINE_H
