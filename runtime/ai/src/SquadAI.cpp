#include "PCH.h"

#include "SquadAI.h"

#include "Action.h"
#include "Agent.h"
#include "SquadEntity.h"
#include "State.h"
#include "Transition.h"

namespace Radion::AI
{

using Radion::Agent;

StateMachine* buildLeaderStateMachine(Agent& leader)
{
    StateMachine* machine = new StateMachine();

    State* awaiting = new State("AwaitingSquadTaskCompletion");
    State* command = new State("CommandSquadToPOI");
    State* standing = new State("StandingGround");

    machine->addState(awaiting);
    machine->addState(command);
    machine->addState(standing);

    // Dispatch once: arrival alone must not re-dispatch every frame, or state text and goals oscillate.
    awaiting->addTransition(new CallbackTransition(awaiting, command,
                                                   [&leader](State&)
                                                   {
                                                       return leader.hasCommandChanged();
                                                   }));

    command->addEnterAction(new CallbackAction(command,
                                               [&leader](State&)
                                               {
                                                   switch (leader.command())
                                                   {
                                                   case SquadCommand::PatrolPointsOfInterest:
                                                       leader.sendSquadToRandomPOI();
                                                       break;
                                                   case SquadCommand::PatrolWaypointNetwork:
                                                       leader.sendSquadToRandomWaypoint();
                                                       break;
                                                   case SquadCommand::RallyToLeaderPosition:
                                                       leader.commandSquadToRallyOnLeader();
                                                       break;
                                                   case SquadCommand::AttackTarget:
                                                       leader.sendSquadToTarget();
                                                       break;
                                                   default:
                                                       break;
                                                   }
                                                   leader.acknowledgeCommand();
                                               }));

    // Waiting for a new command prevents one-frame state-machine ping-pong.
    command->addTransition(new CallbackTransition(command, awaiting,
                                                  [&leader](State&)
                                                  {
                                                      return leader.command() != SquadCommand::StandGround;
                                                  }));

    command->addTransition(new CallbackTransition(command, standing,
                                                  [&leader](State&)
                                                  {
                                                      return leader.command() ==
                                                             SquadCommand::StandGround;
                                                  }));

    standing->addTransition(new CallbackTransition(standing, command,
                                                   [&leader](State&)
                                                   {
                                                       return leader.command() !=
                                                              SquadCommand::StandGround;
                                                   }));

    machine->reset();
    return machine;
}

StateMachine* buildMemberStateMachine(Agent& member)
{
    StateMachine* machine = new StateMachine();

    State* waiting = new State("WaitingForCommand");
    State* moving = new State("MovingToGoal");
    State* waypoint = new State("WaypointReached");

    machine->addState(waiting);
    machine->addState(moving);
    machine->addState(waypoint);

    waiting->addEnterAction(new CallbackAction(waiting,
                                               [&member](State&)
                                               {
                                                   member.onWaitingForCommand();
                                               }));

    waiting->addTransition(new CallbackTransition(waiting, moving,
                                                  [&member](State&)
                                                  {
                                                      return member.hasValidPath() &&
                                                             !member.hasValidWaypoint() &&
                                                             member.command() !=
                                                                 SquadCommand::StandGround;
                                                  }));

    moving->addTransition(
        new CallbackTransition(moving, waypoint,
                               [&member](State&)
                               {
                                   return (member.waypointReached() || member.goalReached()) &&
                                          member.command() != SquadCommand::StandGround;
                               }));

    moving->addTransition(new CallbackTransition(moving, waiting,
                                                 [&member](State&)
                                                 {
                                                     return member.command() ==
                                                            SquadCommand::StandGround;
                                                 }));

    waypoint->addEnterAction(new CallbackAction(waypoint,
                                                [&member](State&)
                                                {
                                                    if (member.waypointReached())
                                                        member.onWaypointReached();
                                                    else
                                                        member.onGoalReached();
                                                }));

    waypoint->addTransition(
        new CallbackTransition(waypoint, moving,
                               [&member](State&)
                               {
                                   return (member.hasValidPath() || !member.goalReached()) &&
                                          member.command() != SquadCommand::StandGround;
                               }));

    waypoint->addTransition(
        new CallbackTransition(waypoint, waiting,
                               [&member](State&)
                               {
                                   return (member.goalReached() && !member.hasValidPath()) ||
                                          member.command() == SquadCommand::StandGround;
                               }));

    machine->reset();
    return machine;
}

} // namespace Radion::AI
