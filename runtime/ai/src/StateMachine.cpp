// StateMachine.cpp - implementation of the Radion AI state machine.

#include "PCH.h"

#include "StateMachine.h"

#include "Action.h"
#include "State.h"
#include "Transition.h"

#include <cstdio>

namespace Radion::AI
{

StateMachine::~StateMachine()
{
    for (State* state : mStates)
        delete state;
    mStates.clear();
    mCurrentState = nullptr;
}

void StateMachine::iterate()
{
    // Every call below is a user callback, and a callback can reach back in
    // here: setCurrentState(), removeState() - which deletes the State and
    // every transition pointing at it - or anything that destroys the object
    // the callbacks captured. So the state is pinned before each one and
    // rechecked after: if it moved, this tick is over, because the list this
    // loop is walking belongs to a state that may no longer exist.
    State* state = mCurrentState;
    if (!state)
        return;

    state->iterate();
    if (mCurrentState != state)
        return;

    // By index, with size() re-read every turn: a callback may have erased
    // transitions out of this very vector (removeState() does exactly that
    // to any transition aimed at the state it removes).
    std::vector<Transition*>& transitions = state->transitions();
    for (usize i = 0; i < transitions.size(); ++i)
    {
        Transition* transition = transitions[i];
        if (!transition)
            continue;
        if (!transition->shouldTransition())
        {
            if (mCurrentState != state)
                return;
            continue;
        }

        // Read the target before exit() runs: exit() can remove states, and
        // removeState() deletes the transitions that name them.
        State* target = transition->targetPtr();
        state->exit();
        if (mCurrentState != state)
            return; // exit() already moved the machine somewhere else
        if (!target)
            return;
        mCurrentState = target;
        target->enter();
        return;
    }
}

void StateMachine::addState(State* state)
{
    if (!state)
        return;
    if (std::find(mStates.begin(), mStates.end(), state) != mStates.end())
        return;

    if (!mCurrentState)
        mCurrentState = state;
    mStates.push_back(state);
}

void StateMachine::removeState(State* state)
{
    auto it = std::find(mStates.begin(), mStates.end(), state);
    if (it == mStates.end())
        return;

    if (mCurrentState == state)
        mCurrentState = nullptr;

    // A transition living in any other state can still point at `state`
    // through targetPtr()/sourcePtr() - that pointer would dangle the moment
    // `state` is deleted below. iterate() could then install it as
    // mCurrentState on the next tick, and toDot() dereferences it
    // unconditionally: both are a confirmed use-after-free without this.
    // `state`'s own outgoing transitions do not need pruning here - they die
    // with it, owned by its destructor.
    for (State* other : mStates)
    {
        if (other == state)
            continue;
        std::vector<Transition*>& transitions = other->transitions();
        for (usize i = transitions.size(); i > 0; --i)
        {
            Transition* transition = transitions[i - 1];
            if (transition->targetPtr() == state || transition->sourcePtr() == state)
            {
                delete transition;
                transitions.erase(transitions.begin() + (i - 1));
            }
        }
    }

    delete state;
    mStates.erase(it);
}

State* StateMachine::findState(const std::string& name) const
{
    for (State* state : mStates)
    {
        if (state->name() == name)
            return state;
    }
    return nullptr;
}

void StateMachine::reset()
{
    // Mirrors the reference cStateMachine::Reset(): jump to the first state
    // registered and re-enter it.
    if (!mStates.empty())
        mCurrentState = mStates.front();
    if (mCurrentState)
        mCurrentState->reset();
}

State* StateMachine::constructState(const std::string& name)
{
    return new State(name);
}

std::string StateMachine::toDot() const
{
    std::string dot = "digraph StateMachine {\n";

    char number[32];
    for (State* state : mStates)
    {
        dot += "\t";
        dot += state->name();
        dot += "[label = \"";
        dot += state->name();
        dot += "\\nInit: ";
        std::snprintf(number, sizeof(number), "%0.3f", state->initialValue());
        dot += number;
        dot += "\\nValue: ";
        std::snprintf(number, sizeof(number), "%0.3f", state->value());
        dot += number;
        dot += "\"]";
        if (state == mCurrentState)
            dot += "[color=red]";
        dot += ";\n";
    }

    for (State* state : mStates)
    {
        for (Transition* trans : state->transitions())
        {
            dot += "\t";
            dot += trans->source().name();
            dot += "->";
            dot += trans->target().name();
            dot += "[label = \"";
            dot += trans->label();
            dot += "\"];\n";
        }
    }
    dot += "};\n";
    return dot;
}

} // namespace Radion::AI
