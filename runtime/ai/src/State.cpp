// State.cpp - implementation of the Radion AI state.

#include "PCH.h"

#include "State.h"

#include "Action.h"
#include "Transition.h"

namespace Radion::AI
{

State::~State()
{
    for (Transition* t : mTransitions)
        delete t;
    mTransitions.clear();

    for (Action* a : mActions)
        delete a;
    mActions.clear();

    for (Action* a : mEnterActions)
        delete a;
    mEnterActions.clear();

    for (Action* a : mExitActions)
        delete a;
    mExitActions.clear();
}

void State::reset()
{
    mValue = mInitialValue;
    enter();
}

void State::enter()
{
    for (Action* a : mEnterActions)
        a->execute();
}

void State::iterate()
{
    for (Action* a : mActions)
        a->execute();
}

void State::exit()
{
    for (Action* a : mExitActions)
        a->execute();
}

} // namespace Radion::AI
