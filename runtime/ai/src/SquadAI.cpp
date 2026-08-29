// SquadAI.cpp - squad state machines, built from their definitions with C++
// callbacks for the actions and transitions.

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

    // AwaitingSquadTaskCompletion -> CommandSquadToPOI
    //   A command is dispatched once. Arrival alone must not dispatch it again
    //   every frame, otherwise the state text and squad goals oscillate.
    awaiting->addTransition(new CallbackTransition(awaiting, command,
                                                   [&leader](State&)
                                                   {
                                                       return leader.hasCommandChanged();
                                                   }));

    // CommandSquadToPOI enter action
    //   dispatch the squad according to the current command
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

    // CommandSquadToPOI -> AwaitingSquadTaskCompletion
    //   dispatch is complete after the enter action; waiting for a new
    //   command prevents a one-frame state-machine ping-pong.
    command->addTransition(new CallbackTransition(command, awaiting,
                                                  [&leader](State&)
                                                  {
                                                      return leader.command() != SquadCommand::StandGround;
                                                  }));

    // CommandSquadToPOI -> StandingGround
    //   should_transition: get_command() == StandGround
    command->addTransition(new CallbackTransition(command, standing,
                                                  [&leader](State&)
                                                  {
                                                      return leader.command() ==
                                                             SquadCommand::StandGround;
                                                  }));

    // StandingGround -> CommandSquadToPOI
    //   should_transition: get_command() != StandGround
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

    // WaitingForCommand enter action
    waiting->addEnterAction(new CallbackAction(waiting,
                                               [&member](State&)
                                               {
                                                   member.onWaitingForCommand();
                                               }));

    // WaitingForCommand -> MovingToGoal
    //   should_transition: has_valid_path() and not has_valid_waypoint()
    //                      and get_command() != StandGround
    waiting->addTransition(new CallbackTransition(waiting, moving,
                                                  [&member](State&)
                                                  {
                                                      return member.hasValidPath() &&
                                                             !member.hasValidWaypoint() &&
                                                             member.command() !=
                                                                 SquadCommand::StandGround;
                                                  }));

    // MovingToGoal -> WaypointReached
    //   should_transition: (waypoint_reached() or goal_reached())
    //                      and get_command() != StandGround
    moving->addTransition(
        new CallbackTransition(moving, waypoint,
                               [&member](State&)
                               {
                                   return (member.waypointReached() || member.goalReached()) &&
                                          member.command() != SquadCommand::StandGround;
                               }));

    // MovingToGoal -> WaitingForCommand
    //   should_transition: get_command() == StandGround
    moving->addTransition(new CallbackTransition(moving, waiting,
                                                 [&member](State&)
                                                 {
                                                     return member.command() ==
                                                            SquadCommand::StandGround;
                                                 }));

    // WaypointReached enter action
    waypoint->addEnterAction(new CallbackAction(waypoint,
                                                [&member](State&)
                                                {
                                                    if (member.waypointReached())
                                                        member.onWaypointReached();
                                                    else
                                                        member.onGoalReached();
                                                }));

    // WaypointReached -> MovingToGoal
    //   should_transition: (has_valid_path() or not goal_reached())
    //                      and get_command() != StandGround
    waypoint->addTransition(
        new CallbackTransition(waypoint, moving,
                               [&member](State&)
                               {
                                   return (member.hasValidPath() || !member.goalReached()) &&
                                          member.command() != SquadCommand::StandGround;
                               }));

    // WaypointReached -> WaitingForCommand
    //   should_transition: (goal_reached() and not has_valid_path())
    //                      or get_command() == StandGround
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
