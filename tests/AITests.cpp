// AITests.cpp - smoke tests for the AI code that used to be radion_ai
// (runtime/ai), now folded into radion_scene as Radion::Agent + the
// steering/pathfinding/state-machine support it drives.
//
// Covers the subsystems: state machine, waypoint network A*, grid A*,
// flocking, steering (seek/flee/wander/obstacle avoidance) and the squad
// glue that ties them together - driven through Scene + GameObject + Agent
// instead of the old World/Group/Entity tree.

#include "PCH.h"

#include "AI.h"
#include "Scene.h"

#include <cmath>
#include <cstdio>

using namespace Radion;
using namespace Radion::AI;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (condition)
        return;
    std::fprintf(stderr, "AITests:%d: failed: %s\n", line, expression);
    ++gFailures;
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool finiteVec(const glm::vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool near(const glm::vec3& a, const glm::vec3& b, float epsilon = 0.0001f)
{
    return glm::length(a - b) <= epsilon;
}

// A visibility functor that rejects everything (forces path generation).
struct NoVisibility final : WaypointVisibility
{
    bool isVisible(const glm::vec3&, const glm::vec3&) const override
    {
        return false;
    }
};

Agent::Settings defaultAgentSettings()
{
    Agent::Settings s;
    s.type = 1;
    s.senseRange = 8.0f;
    s.maxVelocityChange = 2.0f;
    s.maxSpeed = 5.0f;
    s.desiredSpeed = 2.0f;
    return s;
}

// Agent's constructor is private (GameObject::addComponent<Agent>() only),
// so every agent in these tests is a Component on its own GameObject -
// replaces `new Entity(world, settings)` + `group->add(*e)`.
Agent* makeAgent(Scene& scene, const Agent::Settings& settings, const char* name = "agent")
{
    GameObject* object = scene.createGameObject(name);
    Agent* agent = object->addComponent<Agent>();
    agent->applySettings(settings);
    return agent;
}

// --- State machine ----------------------------------------------------------

void testStateMachine()
{
    StateMachine machine;
    State* idle = new State("Idle");
    State* counting = new State("Counting");
    State* done = new State("Done");
    machine.addState(idle);
    machine.addState(counting);
    machine.addState(done);

    int ticks = 0;
    counting->addAction(new CallbackAction(counting,
                                           [&ticks](State&)
                                           {
                                               ++ticks;
                                           }));
    idle->addTransition(new CallbackTransition(idle, counting,
                                               [](State&)
                                               {
                                                   return true;
                                               }));
    counting->addTransition(new CallbackTransition(counting, done,
                                                   [&ticks](State&)
                                                   {
                                                       return ticks >= 3;
                                                   }));

    machine.reset();
    CHECK(machine.currentState() == idle);

    machine.iterate(); // Idle -> Counting
    CHECK(machine.currentState() == counting);

    machine.iterate(); // tick 1
    machine.iterate(); // tick 2
    CHECK(machine.currentState() == counting);
    CHECK(ticks == 2);

    machine.iterate(); // tick 3 -> Done
    CHECK(machine.currentState() == done);
    CHECK(ticks == 3);
}

// --- Waypoint network A* ----------------------------------------------------

void testWaypointNetwork()
{
    WaypointNetwork network;

    // Linear chain A-B-C-D.
    Waypoint* a =
        new Waypoint(glm::vec3(0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    Waypoint* b =
        new Waypoint(glm::vec3(10.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    Waypoint* c =
        new Waypoint(glm::vec3(20.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    Waypoint* d =
        new Waypoint(glm::vec3(30.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    CHECK(network.addWaypoint(a));
    CHECK(network.addWaypoint(b));
    CHECK(network.addWaypoint(c));
    CHECK(network.addWaypoint(d));
    CHECK(!network.addWaypoint(a)); // duplicate id rejected

    a->addEdge(NetworkEdge{b->id()});
    b->addEdge(NetworkEdge{a->id()});
    b->addEdge(NetworkEdge{c->id()});
    c->addEdge(NetworkEdge{b->id()});
    c->addEdge(NetworkEdge{d->id()});
    d->addEdge(NetworkEdge{c->id()});

    Path path;
    CHECK(network.findPath(a->id(), d->id(), path));
    CHECK(path.size() == 4);
    CHECK(path.front() == a->id());
    CHECK(path.back() == d->id());

    // Same endpoint trivially succeeds.
    Path selfPath;
    CHECK(network.findPath(b->id(), b->id(), selfPath));
    CHECK(selfPath.size() == 1);

    // Position-based search (everything visible).
    Path posPath;
    CHECK(network.findPath(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(30.0f, 0.0f, 0.0f),
                           WaypointVisibility(), posPath));
    CHECK(posPath.size() == 4);

    // Nothing visible -> no valid waypoint -> search fails.
    Path noPath;
    CHECK(!network.findPath(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(30.0f, 0.0f, 0.0f),
                            NoVisibility(), noPath));

    // Closed edge makes A-D unreachable.
    b->edges()[1].open = false; // close B->C
    Path blockedPath;
    CHECK(!network.findPath(a->id(), d->id(), blockedPath));
}

// --- Grid A* ----------------------------------------------------------------

void testGridPathfinder()
{
    // 10x10 grid with a vertical wall at x=5 (y 1..8); route around it.
    GridMap grid(10);
    for (int y = 1; y <= 8; ++y)
        grid.setBlocked(5, y);

    GridPathfinder finder(&grid);
    std::vector<GridCellCoord> path;
    CHECK(finder.findPath(0, 4, 9, 4, path));
    CHECK(!path.empty());
    CHECK(path.front().x == 0 && path.front().y == 4);
    CHECK(path.back().x == 9 && path.back().y == 4);
    for (const GridCellCoord& cell : path)
        CHECK(!grid.isBlocked(cell.x, cell.y));

    // Start == goal.
    std::vector<GridCellCoord> same;
    CHECK(finder.findPath(3, 3, 3, 3, same));
    CHECK(same.size() == 1);

    // Fully enclosed goal (all 8 neighbours blocked) -> unreachable.
    GridMap enclosed(5);
    const int cx = 2, cy = 2;
    for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
            if (dx != 0 || dy != 0)
                enclosed.setBlocked(cx + dx, cy + dy);
    GridPathfinder enclosedFinder(&enclosed);
    std::vector<GridCellCoord> none;
    CHECK(!enclosedFinder.findPath(0, 0, cx, cy, none));
}

// --- Flocking ---------------------------------------------------------------

void testFlocking()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    Agent* a = makeAgent(scene, settings, "a");
    Agent* b = makeAgent(scene, settings, "b");
    Agent* c = makeAgent(scene, settings, "c");
    // Registers a/b/c with the Scene (GameObject::add() is deferred) before
    // any per-agent state is set - the flush itself runs one behavior-less
    // update() per agent, so doing it before positions/behaviors exist keeps
    // it a no-op rather than an extra, uncounted simulation step.
    scene.update(0.0f);

    a->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    b->setPosition(glm::vec3(3.0f, 0.0f, 0.0f));
    c->setPosition(glm::vec3(1.5f, 0.0f, 2.5f));
    // Same flock: groupId() replaces AI::Group membership (0 means "no group").
    a->setGroupId(1);
    b->setGroupId(1);
    c->setGroupId(1);

    // Behaviors are owned per-agent now (Agent::addBehavior()), so each
    // needs its own instance - a single shared one would be double-deleted
    // when more than one owner's destructor ran.
    for (Agent* e : {a, b, c})
    {
        e->addBehavior<SeparationBehavior>(4.0f, 0.2f, 1.0f);
        e->addBehavior<CohesionBehavior>(2.0f);
        e->addBehavior<AlignmentBehavior>(1.0f);
        e->addBehavior<StayWithinSphereBehavior>(glm::vec3(0.0f, 0.0f, 0.0f), 20.0f);
    }

    // The three start separated; after one step every agent should have a
    // non-zero desired move (it sensed its flockmates).
    scene.updateAgents(0.016f);
    CHECK(glm::length(a->desiredMove()) > 0.0f);
    CHECK(glm::length(b->desiredMove()) > 0.0f);
    CHECK(glm::length(c->desiredMove()) > 0.0f);

    // Run a while; the sim must stay finite and the flock roughly together.
    for (int i = 0; i < 300; ++i)
        scene.updateAgents(0.016f);

    CHECK(finiteVec(a->position()));
    CHECK(finiteVec(b->position()));
    CHECK(finiteVec(c->position()));
    CHECK(glm::length(a->position() - b->position()) < 12.0f);
    CHECK(glm::length(b->position() - c->position()) < 12.0f);
    CHECK(glm::length(a->position() - glm::vec3(0.0f, 0.0f, 0.0f)) < 25.0f);
}
// --- grid search algorithms ------------------------------------------------

void testGridAlgorithms()
{
    // 10x10 grid with a vertical wall at x=5 (y 1..8); the path must go around.
    GridMap grid(10);
    for (int y = 1; y <= 8; ++y)
        grid.setBlocked(5, y);

    GridPathfinder finder(&grid);

    std::vector<GridCellCoord> path;
    for (GridSearchAlgorithm algorithm :
         {GridSearchAlgorithm::AStar, GridSearchAlgorithm::Dijkstra, GridSearchAlgorithm::BestFirst,
          GridSearchAlgorithm::BreadthFirst})
    {
        finder.settings().algorithm = algorithm;
        CHECK(finder.findPath(0, 4, 9, 4, path));
        CHECK(!path.empty());
        CHECK(path.front().x == 0 && path.front().y == 4);
        CHECK(path.back().x == 9 && path.back().y == 4);
        for (const GridCellCoord& cell : path)
            CHECK(!grid.isBlocked(cell.x, cell.y));
    }
}
// --- Squad glue -------------------------------------------------------------

void testSquadMovement()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    // Simple two-waypoint network far along +X.
    WaypointNetwork network;
    Waypoint* wpStart =
        new Waypoint(glm::vec3(0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    Waypoint* wpGoal =
        new Waypoint(glm::vec3(50.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    network.addWaypoint(wpStart);
    network.addWaypoint(wpGoal);
    wpStart->addEdge(NetworkEdge{wpGoal->id()});
    wpGoal->addEdge(NetworkEdge{wpStart->id()});

    PointsOfInterest pois;
    PointOfInterest* target = new PointOfInterest(glm::vec3(50.0f, 0.0f, 0.0f), 5.0f);
    CHECK(pois.add(target));

    Agent* leader = makeAgent(scene, settings, "leader");
    Agent* member = makeAgent(scene, settings, "member");
    scene.update(0.0f); // register both before configuring them

    leader->setWaypointNetwork(&network);
    leader->setSquadId(0);
    leader->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));

    member->setWaypointNetwork(&network);
    member->setSquadId(1);
    member->setPosition(glm::vec3(5.0f, 0.0f, 0.0f));

    leader->addSquadMember(member);

    // Force pathfinding: block line of sight so the member has to route.
    member->addBehavior<PathfindBehavior>(PathfindBehavior::Settings{
        0.2f, 50.0f, 0.0f, 25.0f, 0.5f, 0.0f, glm::vec3(0.0f, 1.0f, 0.0f), &network, nullptr});

    // Owned by the agent now (Agent::setStateMachine()) - no manual delete.
    member->setStateMachine(buildMemberStateMachine(*member));
    leader->setStateMachine(buildLeaderStateMachine(*leader));

    // Order the squad to the target POI.
    leader->setPointsOfInterest(&pois);
    leader->setSelectedPointOfInterest(target);
    leader->setCommand(SquadCommand::AttackTarget);

    for (int i = 0; i < 600; ++i)
    {
        scene.updateAgents(0.016f);
        if (!finiteVec(member->position()))
            break;
    }

    CHECK(finiteVec(member->position()));
    // With no LOS the member routes through the network and walks far along +X.
    CHECK(member->position().x > 15.0f);
}

void testMemberStateMachine()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    WaypointNetwork network;
    Waypoint* wp =
        new Waypoint(glm::vec3(50.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    network.addWaypoint(wp);

    // Agent::update() is called directly here (not through the Scene's own
    // agent list), so no scene.update() flush is needed first.
    Agent* member = makeAgent(scene, settings, "member");
    member->setWaypointNetwork(&network);
    member->setSquadId(1);
    member->setPosition(glm::vec3(50.5f, 0.0f, 0.0f)); // inside wp radius
    member->setGoal(glm::vec3(50.0f, 0.0f, 0.0f));

    StateMachine* machine = buildMemberStateMachine(*member);
    member->setStateMachine(machine);

    CHECK(machine->currentState()->name() == "WaitingForCommand");

    // A valid path + a non-standground command leaves WaitingForCommand.
    member->setCommand(SquadCommand::AttackTarget);
    member->setNextWaypoint(0);
    member->setPath(Path{wp->id()});
    member->update(0.016f);
    CHECK(machine->currentState()->name() == "MovingToGoal");

    // Member sits inside the waypoint radius -> WaypointReached.
    member->update(0.016f);
    CHECK(machine->currentState()->name() == "WaypointReached");

    // Standing ground pulls it back to WaitingForCommand.
    member->setCommand(SquadCommand::StandGround);
    member->update(0.016f);
    CHECK(machine->currentState()->name() == "WaitingForCommand");
}

void testLeaderStateMachine()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    Agent* leader = makeAgent(scene, settings, "leader");
    leader->setSquadId(0);
    leader->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));

    Agent* member = makeAgent(scene, settings, "member");
    member->setSquadId(1);
    member->setPosition(glm::vec3(5.0f, 0.0f, 0.0f));
    member->setGoal(member->position()); // already at its goal
    leader->addSquadMember(member);

    StateMachine* machine = buildLeaderStateMachine(*leader);
    leader->setStateMachine(machine);

    CHECK(machine->currentState()->name() == "AwaitingSquadTaskCompletion");

    // Squad is at its goal -> issue the next command.
    leader->update(0.016f);
    CHECK(machine->currentState()->name() == "CommandSquadToPOI");

    // Stand ground -> CommandSquadToPOI hands off to StandingGround.
    leader->setCommand(SquadCommand::StandGround);
    leader->update(0.016f);
    CHECK(machine->currentState()->name() == "StandingGround");
    leader->update(0.016f);
    CHECK(machine->currentState()->name() == "StandingGround");

    // New command leaves StandingGround.
    leader->setCommand(SquadCommand::PatrolPointsOfInterest);
    leader->update(0.016f);
    CHECK(machine->currentState()->name() == "CommandSquadToPOI");
}

// --- steering --------------------------------------------------------------

void testSteerLibrary()
{
    Scene scene;
    Agent& e = *makeAgent(scene, defaultAgentSettings());
    e.setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    e.setVelocity(glm::vec3(2.0f, 0.0f, 0.0f));
    e.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward = +Z

    SteerLibrary steer(e);

    // seek: (target - position) - velocity
    CHECK(near(steer.seek(glm::vec3(10.0f, 0.0f, 0.0f)), glm::vec3(8.0f, 0.0f, 0.0f)));
    // flee: (position - target) - velocity
    CHECK(near(steer.flee(glm::vec3(10.0f, 0.0f, 0.0f)), glm::vec3(-12.0f, 0.0f, 0.0f)));
    // targetSpeed: forward * clip(target - current, -maxForce, +maxForce); maxForce = 2
    CHECK(near(steer.targetSpeed(5.0f), glm::vec3(0.0f, 0.0f, 2.0f)));
    // predictFuturePosition
    CHECK(near(e.predictFuturePosition(1.0f), glm::vec3(2.0f, 0.0f, 0.0f)));
    // local/global transforms round-trip
    CHECK(near(e.globalizePosition(e.localizePosition(glm::vec3(3.0f, 4.0f, 5.0f))),
               glm::vec3(3.0f, 4.0f, 5.0f)));
}

void testPlaneAndRectangleObstacle()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 0.5f;
    Agent& vehicle = *makeAgent(scene, settings);

    // Default plane: XY at the origin, +Z half-space is outside.
    PlaneObstacle plane;

    // Facing the plane head-on from the outside.
    vehicle.setPosition(glm::vec3(1.0f, 2.0f, 5.0f));
    vehicle.setOrientation(glm::angleAxis(glm::pi<f32>(), glm::vec3(0.0f, 1.0f, 0.0f))); // -Z
    PathIntersection pi;
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 5.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, glm::vec3(1.0f, 2.0f, 0.0f), 0.001f));
    CHECK(near(pi.surfaceNormal, glm::vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(pi.vehicleOutside);

    // Heading away from the plane: no intersection.
    vehicle.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)); // +Z
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    // Path parallel to the plane: no intersection.
    vehicle.setOrientation(glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f))); // +X
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    // Behind an Outside-only plane, heading into it: the back is not solid.
    vehicle.setPosition(glm::vec3(0.0f, 0.0f, -5.0f));
    vehicle.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)); // +Z, toward the plane
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    // The same approach against an Inside-only plane does hit, normal -Z.
    plane.setSeenFrom(ObstacleSeenFrom::Inside);
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(near(pi.surfaceNormal, glm::vec3(0.0f, 0.0f, -1.0f), 0.001f));
    CHECK(!pi.vehicleOutside);

    // A 2x2 rectangle only blocks paths crossing inside its (radius-grown) bounds.
    RectangleObstacle rect(2.0f, 2.0f);
    vehicle.setPosition(glm::vec3(0.5f, 0.5f, 5.0f));
    vehicle.setOrientation(glm::angleAxis(glm::pi<f32>(), glm::vec3(0.0f, 1.0f, 0.0f))); // -Z
    rect.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 5.0f) < 0.001f);

    // Crossing the plane well outside the rectangle: miss.
    vehicle.setPosition(glm::vec3(5.0f, 0.0f, 5.0f));
    rect.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    // Grazing inside the vehicle-radius-grown edge (half-width 1 + radius 0.5).
    vehicle.setPosition(glm::vec3(1.4f, 0.0f, 5.0f));
    rect.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
}

void testBoxObstacle()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 0.0f; // exact face bounds, no radius growth
    Agent& vehicle = *makeAgent(scene, settings);

    // 2x2x2 box at the origin, world-aligned.
    BoxObstacle box(2.0f, 2.0f, 2.0f, glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                    glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f));

    // Head-on along -Z: hits the front face at z = +1, steer hint outward.
    vehicle.setPosition(glm::vec3(0.0f, 0.0f, 5.0f));
    vehicle.setOrientation(glm::angleAxis(glm::pi<f32>(), glm::vec3(0.0f, 1.0f, 0.0f))); // -Z
    PathIntersection pi;
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 4.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, glm::vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(near(pi.steerHint, glm::vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(pi.vehicleOutside);
    CHECK(pi.obstacle == &box);

    // Head-on along -X: hits the +X side face at x = +1.
    vehicle.setPosition(glm::vec3(5.0f, 0.0f, 0.0f));
    vehicle.setOrientation(glm::angleAxis(glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 4.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, glm::vec3(1.0f, 0.0f, 0.0f), 0.001f));
    CHECK(near(pi.steerHint, glm::vec3(1.0f, 0.0f, 0.0f), 0.001f));

    // Descending onto the top: hits the +Y face at y = +1.
    vehicle.setPosition(glm::vec3(0.0f, 5.0f, 0.0f));
    vehicle.setOrientation(glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 4.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, glm::vec3(0.0f, 1.0f, 0.0f), 0.001f));
    CHECK(near(pi.steerHint, glm::vec3(0.0f, 1.0f, 0.0f), 0.001f));

    // A path passing beside the box misses every face.
    vehicle.setPosition(glm::vec3(5.0f, 5.0f, 5.0f));
    vehicle.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)); // +Z, away
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);
}

void testSphereObstacleSeenFrom()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 0.5f;
    Agent& vehicle = *makeAgent(scene, settings);

    // A vehicle outside an Inside-only sphere must be pulled back toward it.
    SphereObstacle pen(2.0f, glm::vec3(0.0f));
    pen.setSeenFrom(ObstacleSeenFrom::Inside);
    vehicle.setPosition(glm::vec3(10.0f, 0.0f, 0.0f));
    vehicle.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    PathIntersection pi;
    pen.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(pi.distance == 0.0f);
    CHECK(near(pi.steerHint, glm::vec3(-1.0f, 0.0f, 0.0f), 0.001f));
    CHECK(pi.vehicleOutside);

    // Inside a hollow (Both) shell, the exit ahead pushes back inward.
    SphereObstacle shell(3.0f, glm::vec3(0.0f));
    shell.setSeenFrom(ObstacleSeenFrom::Both);
    vehicle.setPosition(glm::vec3(0.0f));
    shell.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(!pi.vehicleOutside);
    CHECK(std::fabs(pi.distance - 3.5f) < 0.001f); // radius + vehicle radius
    CHECK(near(pi.surfaceNormal, glm::vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(near(pi.steerHint, glm::vec3(0.0f, 0.0f, -1.0f), 0.001f));

    // A path whose closest approach is beyond the radius misses entirely.
    SphereObstacle ball(2.0f, glm::vec3(0.0f));
    vehicle.setPosition(glm::vec3(0.0f, 10.0f, 0.0f));
    ball.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);
}

void testCruisingAxisDistribution()
{
    Scene scene;
    Agent& e = *makeAgent(scene, defaultAgentSettings());
    e.setVelocity(glm::vec3(0.0f)); // below desiredSpeed, so signum is +1

    // The flocking demo's own chances: X larger than Y. The per-axis chances
    // form cumulative bands, so Y must still fire - with a raw-roll compare
    // it never could (its band was a subset of X's).
    CruisingBehavior cruising(0.45f, 0.2f, 0.35f, 1.0f, 0.5f, 0.2f);

    std::srand(12345);
    int hits[3] = {0, 0, 0};
    for (int i = 0; i < 2000; ++i)
    {
        e.setDesiredMove(glm::vec3(0.0f));
        cruising.iterate(0.016f, e);
        const glm::vec3 move = e.desiredMove();
        if (move.x != 0.0f)
            ++hits[0];
        if (move.y != 0.0f)
            ++hits[1];
        if (move.z != 0.0f)
            ++hits[2];
    }
    CHECK(hits[0] > 0);
    CHECK(hits[1] > 0);
    CHECK(hits[2] > 0);
    CHECK(hits[0] > hits[1]); // X's band (0.45) is wider than Y's (0.2)
}

void testGridAStarOptimality()
{
    // Around the same wall as testGridPathfinder. Breadth-first is optimal in
    // moves by construction, and with unit diagonal cost so is Dijkstra and
    // A* under the admissible default heuristic - all three must agree.
    GridMap grid(10);
    for (int y = 1; y <= 8; ++y)
        grid.setBlocked(5, y);

    GridPathfinder finder(&grid);
    std::vector<GridCellCoord> bfsPath;
    finder.settings().algorithm = GridSearchAlgorithm::BreadthFirst;
    CHECK(finder.findPath(0, 4, 9, 4, bfsPath));

    std::vector<GridCellCoord> aStarPath;
    finder.settings().algorithm = GridSearchAlgorithm::AStar;
    finder.settings().heuristic = GridHeuristic::MaxDxDy;
    CHECK(finder.findPath(0, 4, 9, 4, aStarPath));
    CHECK(aStarPath.size() == bfsPath.size());

    std::vector<GridCellCoord> dijkstraPath;
    finder.settings().algorithm = GridSearchAlgorithm::Dijkstra;
    CHECK(finder.findPath(0, 4, 9, 4, dijkstraPath));
    CHECK(dijkstraPath.size() == bfsPath.size());

    // Open grid: the optimal move count is the Chebyshev distance.
    GridMap open(10);
    GridPathfinder openFinder(&open);
    openFinder.settings().algorithm = GridSearchAlgorithm::AStar;
    std::vector<GridCellCoord> diagonal;
    CHECK(openFinder.findPath(0, 0, 7, 3, diagonal));
    CHECK(diagonal.size() == 8); // max(7, 3) moves + start
}

void testStateMachineRemoveState()
{
    StateMachine machine;
    State* a = new State("A");
    State* b = new State("B");
    State* c = new State("C");
    machine.addState(a);
    machine.addState(b);
    machine.addState(c);

    a->addTransition(new CallbackTransition(a, b,
                                            [](State&)
                                            {
                                                return true;
                                            }));
    b->addTransition(new CallbackTransition(b, c,
                                            [](State&)
                                            {
                                                return true;
                                            }));

    machine.reset();
    CHECK(machine.currentState() == a);
    CHECK(machine.findState("B") == b);

    // Removing B must also prune A's transition into it - iterating
    // afterwards would otherwise walk a freed state.
    machine.removeState(b);
    CHECK(machine.findState("B") == nullptr);
    machine.iterate();
    CHECK(machine.currentState() == a);
}

// The callbacks a state machine runs can reach back into the machine, and
// removeState() deletes both the State and every transition aimed at it. So
// iterate() must survive its own callbacks rewriting the very list it is
// walking - it used to keep walking, over freed transitions.
void testStateMachineSurvivesCallbackMutation()
{
    // 1. A shouldTransition() that removes a state and answers false: the
    //    loop used to carry on over a vector that just had entries erased.
    {
        StateMachine machine;
        State* a = new State("A");
        State* doomed = new State("Doomed");
        State* other = new State("Other");
        machine.addState(a);
        machine.addState(doomed);
        machine.addState(other);

        // A's first transition points at Doomed, so removing Doomed erases
        // this very transition out from under the loop.
        a->addTransition(new CallbackTransition(a, doomed,
                                                [&machine, doomed](State&)
                                                {
                                                    machine.removeState(doomed);
                                                    return false;
                                                }));
        a->addTransition(new CallbackTransition(a, other,
                                                [](State&)
                                                {
                                                    return true;
                                                }));
        machine.reset();
        CHECK(machine.currentState() == a);
        machine.iterate();
        CHECK(machine.findState("Doomed") == nullptr);
        // Whatever it decided, it must still be on a live state.
        CHECK(machine.currentState() == a || machine.currentState() == other);
    }

    // 2. An exit() that removes the state being left - which deletes it,
    //    and leaves the machine with no current state. Installing the
    //    transition's target afterwards would resurrect a machine the
    //    callback just emptied, having read `state` after it was freed.
    {
        StateMachine machine;
        State* a = new State("A");
        State* b = new State("B");
        machine.addState(a);
        machine.addState(b);

        a->addExitAction(new CallbackAction(a,
                                            [&machine, a](State&)
                                            {
                                                machine.removeState(a);
                                            }));
        a->addTransition(new CallbackTransition(a, b,
                                                [](State&)
                                                {
                                                    return true;
                                                }));
        machine.reset();
        machine.iterate();
        CHECK(machine.findState("A") == nullptr);
        CHECK(machine.currentState() == nullptr); // stays empty, as asked
        machine.iterate();                        // and survives another tick
    }

    // 3. A state's own iterate() that empties the machine.
    {
        StateMachine machine;
        State* a = new State("A");
        State* b = new State("B");
        machine.addState(a);
        machine.addState(b);
        a->addAction(new CallbackAction(a,
                                        [&machine, b](State&)
                                        {
                                            machine.removeState(b);
                                        }));
        a->addTransition(new CallbackTransition(a, b,
                                                [](State&)
                                                {
                                                    return true;
                                                }));
        machine.reset();
        machine.iterate();
        CHECK(machine.findState("B") == nullptr);
        CHECK(machine.currentState() == a);
    }
}

void testPointsOfInterest()
{
    PointsOfInterest pois;
    PointOfInterest* first = new PointOfInterest(glm::vec3(0.0f, 0.0f, 0.0f), 1.0f);
    PointOfInterest* second = new PointOfInterest(glm::vec3(10.0f, 0.0f, 0.0f), 1.0f);
    PointOfInterest* third = new PointOfInterest(glm::vec3(0.0f, 0.0f, 20.0f), 1.0f);
    CHECK(pois.add(first));
    CHECK(pois.add(second));
    CHECK(pois.add(third));

    CHECK(pois.find(first->id()) == first);
    CHECK(pois.findNearest(glm::vec3(9.0f, 0.0f, 1.0f)) == second);
    CHECK(pois.findNearest(glm::vec3(0.0f, 0.0f, 19.0f)) == third);

    // selectRandom never hands back the POI the caller is already at.
    for (int i = 0; i < 50; ++i)
    {
        const PointOfInterest* pick = pois.selectRandom(first->id());
        CHECK(pick != nullptr);
        CHECK(pick->id() != first->id());
    }

    // Taken before the remove: it frees the point, so reading its id back
    // off the pointer afterwards is a use-after-free.
    const u32 secondId = second->id();
    CHECK(pois.remove(secondId));
    CHECK(pois.find(secondId) == nullptr);
}

void testPursuitEvasion()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    Agent& hunter = *makeAgent(scene, settings, "hunter");
    hunter.setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    hunter.setVelocity(glm::vec3(0.0f, 0.0f, 2.0f));
    hunter.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    Agent& quarry = *makeAgent(scene, settings, "quarry");
    quarry.setPosition(glm::vec3(0.0f, 0.0f, 10.0f));

    SteerLibrary steer(hunter);

    // A parked quarry's predicted position is its position: pursuit == seek.
    CHECK(near(steer.pursuit(quarry), steer.seek(quarry.position())));

    // Ahead-parallel quarry: estimated intercept (10/2 * 4 = 20s) is capped
    // at maxPredictionTime, so the target is one second of quarry travel.
    quarry.setVelocity(glm::vec3(0.0f, 0.0f, 3.0f));
    quarry.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    CHECK(near(steer.pursuit(quarry, 1.0f),
               steer.seek(quarry.position() + quarry.velocity() * 1.0f)));

    // A stationary menace is predicted at the cap rather than dividing by
    // zero, and its predicted position is where it already is.
    Agent& menace = *makeAgent(scene, settings, "menace");
    menace.setPosition(glm::vec3(5.0f, 0.0f, 0.0f));
    CHECK(near(steer.evasion(menace, 2.0f), steer.flee(menace.position())));

    // Slow distant menace: rough intercept (5s) exceeds the cap (2s), so the
    // flee target is two seconds of menace travel.
    menace.setVelocity(glm::vec3(0.0f, 0.0f, 1.0f));
    CHECK(near(steer.evasion(menace, 2.0f),
               steer.flee(menace.position() + menace.velocity() * 2.0f)));
}

void testDirectionalPredicates()
{
    Scene scene;
    Agent& e = *makeAgent(scene, defaultAgentSettings());
    e.setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    e.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    SteerLibrary steer(e);
    CHECK(steer.isAhead(glm::vec3(0.0f, 0.0f, 5.0f)));
    CHECK(!steer.isBehind(glm::vec3(0.0f, 0.0f, 5.0f)));
    CHECK(steer.isBehind(glm::vec3(0.0f, 0.0f, -5.0f)));
    CHECK(!steer.isAhead(glm::vec3(0.0f, 0.0f, -5.0f)));
    CHECK(steer.isAside(glm::vec3(5.0f, 0.0f, 0.0f)));
    CHECK(!steer.isAhead(glm::vec3(0.0f, 0.0f, 0.0f))); // degenerate offset

    // Local frame convention: right = +X, up = +Y, forward = +Z.
    CHECK(near(e.localizeDirection(e.forward()), glm::vec3(0.0f, 0.0f, 1.0f)));
    CHECK(near(e.localizeDirection(e.side()), glm::vec3(1.0f, 0.0f, 0.0f)));
    CHECK(near(e.localizeDirection(e.up()), glm::vec3(0.0f, 1.0f, 0.0f)));
    CHECK(near(e.globalizeDirection(glm::vec3(0.0f, 0.0f, 1.0f)), e.forward()));

    // Right-handed yaw: +90 degrees about +Y swings forward from +Z to +X.
    e.setOrientation(glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    CHECK(near(e.forward(), glm::vec3(1.0f, 0.0f, 0.0f), 0.001f));
    CHECK(near(e.side(), glm::vec3(0.0f, 0.0f, -1.0f), 0.001f));

    // alignWithVelocity regenerates a right-handed orthonormal frame with
    // forward along the velocity and up preserved.
    e.setVelocity(glm::vec3(0.0f, 0.0f, -3.0f));
    e.alignWithVelocity();
    CHECK(near(e.forward(), glm::vec3(0.0f, 0.0f, -1.0f), 0.001f));
    CHECK(near(e.up(), glm::vec3(0.0f, 1.0f, 0.0f), 0.001f));
    CHECK(near(glm::cross(e.up(), e.forward()), e.side(), 0.001f));
}

void testBoidNeighborhoodAndSeparation()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    Agent& self = *makeAgent(scene, settings, "self");
    self.setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    self.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    Agent& ahead = *makeAgent(scene, settings, "ahead");
    ahead.setPosition(glm::vec3(0.0f, 0.0f, 5.0f));
    Agent& behind = *makeAgent(scene, settings, "behind");
    behind.setPosition(glm::vec3(0.0f, 0.0f, -5.0f));
    Agent& veryClose = *makeAgent(scene, settings, "veryClose");
    veryClose.setPosition(glm::vec3(0.0f, 0.0f, -0.5f));
    Agent& far = *makeAgent(scene, settings, "far");
    far.setPosition(glm::vec3(0.0f, 0.0f, 50.0f));

    SteerLibrary steer(self);
    CHECK(steer.inBoidNeighborhood(ahead, 1.0f, 10.0f, 0.0f));
    CHECK(!steer.inBoidNeighborhood(behind, 1.0f, 10.0f, 0.0f)); // outside cone
    CHECK(steer.inBoidNeighborhood(veryClose, 1.0f, 10.0f, 0.0f)); // inside min sphere
    CHECK(!steer.inBoidNeighborhood(far, 1.0f, 10.0f, 0.0f)); // outside max sphere
    CHECK(!steer.inBoidNeighborhood(self, 1.0f, 10.0f, 0.0f)); // never itself

    // One neighbor ahead: separation pushes straight back (-Z), alignment
    // returns the error direction toward the neighbor's heading.
    std::vector<EntityDist> flock;
    flock.push_back(EntityDist{5.0f, &ahead});
    CHECK(near(steer.separation(10.0f, -1.0f, flock), glm::vec3(0.0f, 0.0f, -1.0f)));
    CHECK(near(steer.cohesion(10.0f, -1.0f, flock), glm::vec3(0.0f, 0.0f, 1.0f)));
}

void testTargetSpeedClamp()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings(); // maxVelocityChange = 2
    Agent& e = *makeAgent(scene, settings);
    e.setVelocity(glm::vec3(2.0f, 0.0f, 0.0f)); // speed 2
    e.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    SteerLibrary steer(e);
    // Already at the target speed: no correction.
    CHECK(near(steer.targetSpeed(2.0f), glm::vec3(0.0f)));
    // Braking is clamped to maxForce, along -forward.
    CHECK(near(steer.targetSpeed(-5.0f), glm::vec3(0.0f, 0.0f, -2.0f)));
}

void testSeekFlee()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    // A seeker converges on a target far along +X.
    Agent* chaser = makeAgent(scene, settings, "chaser");
    scene.update(0.0f);
    chaser->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    const glm::vec3 target(100.0f, 0.0f, 0.0f);
    chaser->addBehavior<SeekBehavior>(target);

    for (int i = 0; i < 600; ++i)
        scene.updateAgents(0.016f);

    CHECK(finiteVec(chaser->position()));
    CHECK(chaser->position().x > 30.0f);

    // A runner flees from the same target and ends up on the opposite side.
    Agent* runner = makeAgent(scene, settings, "runner");
    scene.update(0.0f);
    runner->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    runner->addBehavior<FleeBehavior>(target);

    for (int i = 0; i < 600; ++i)
        scene.updateAgents(0.016f);

    CHECK(finiteVec(runner->position()));
    CHECK(runner->position().x < -30.0f);
}

void testWander()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    Agent* e = makeAgent(scene, settings, "wanderer");
    scene.update(0.0f);
    e->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    e->addBehavior<WanderBehavior>();

    for (int i = 0; i < 300; ++i)
        scene.updateAgents(0.016f);

    // Wandered away from the start, stayed finite and bounded.
    CHECK(finiteVec(e->position()));
    CHECK(glm::length(e->position()) > 0.001f);
    CHECK(glm::length(e->position()) < 50.0f);
}

void testObstacleAvoidance()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 0.5f;

    // A sphere slightly off the +X travel line so there is a lateral component.
    SphereObstacle sphere(3.0f, glm::vec3(8.0f, 0.0f, 2.0f));
    ObstacleGroup obstacles;
    obstacles.push_back(&sphere);

    Agent* vehicle = makeAgent(scene, settings, "vehicle");
    scene.update(0.0f);
    vehicle->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    vehicle->setVelocity(glm::vec3(5.0f, 0.0f, 0.0f)); // moving +X
    // Face +X so the vehicle's forward path intersects the sphere.
    vehicle->setOrientation(glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    ObstacleAvoidanceBehavior* avoidance = vehicle->addBehavior<ObstacleAvoidanceBehavior>(2.0f);
    avoidance->setObstacles(obstacles);

    scene.updateAgents(0.016f);
    // The avoidance force is lateral: it must push toward -Z (past the sphere).
    CHECK(finiteVec(vehicle->desiredMove()));
    CHECK(vehicle->desiredMove().z < 0.0f);

    for (int i = 0; i < 600; ++i)
        scene.updateAgents(0.016f);

    // The vehicle steered around to the -Z side without entering the sphere.
    CHECK(finiteVec(vehicle->position()));
    CHECK(vehicle->position().z < 0.0f);
    CHECK(glm::length(vehicle->position() - sphere.center) > sphere.radius);
}

// --- predictNearestApproachTime / avoidNeighbors ----------------------------
//
// Regression coverage for the sign bug fixed in Steering.cpp:306 (see
// docs/AI_PATHFINDING_REVIEW.md section 6) - nothing exercised this math
// directly before, which is how the inverted sign passed unnoticed.

void testNearestApproach()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 1.0f;

    // Head-on: we move +X, the other moves -X from further down +X. Closing
    // distance, so the nearest approach must be in the future (time > 0).
    Agent& us = *makeAgent(scene, settings, "us");
    us.setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    us.setVelocity(glm::vec3(1.0f, 0.0f, 0.0f));

    Agent& oncoming = *makeAgent(scene, settings, "oncoming");
    oncoming.setPosition(glm::vec3(10.0f, 0.0f, 0.0f));
    oncoming.setVelocity(glm::vec3(-1.0f, 0.0f, 0.0f));

    SteerLibrary steer(us);
    const float tHeadOn = steer.predictNearestApproachTime(oncoming);
    CHECK(std::isfinite(tHeadOn));
    CHECK(tHeadOn > 0.0f);
    // Symmetric closing speeds meet halfway: at t=5 both are at x=5.
    CHECK(std::fabs(tHeadOn - 5.0f) < 0.01f);
    const float headOnDist = steer.computeNearestApproachPositions(oncoming, tHeadOn);
    CHECK(headOnDist < 0.1f); // they meet almost exactly

    // Receding: swap the velocities so both move apart. The nearest approach
    // was in the past (time < 0), so avoidNeighbors must ignore it.
    Agent& receding = *makeAgent(scene, settings, "receding");
    receding.setPosition(glm::vec3(10.0f, 0.0f, 0.0f));
    receding.setVelocity(glm::vec3(1.0f, 0.0f, 0.0f)); // same direction as us, but faster gap
    Agent& fast = *makeAgent(scene, settings, "fast");
    fast.setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    fast.setVelocity(glm::vec3(-1.0f, 0.0f, 0.0f));
    SteerLibrary steerFast(fast);
    const float tReceding = steerFast.predictNearestApproachTime(receding);
    CHECK(tReceding < 0.0f);

    // avoidNeighbors: a slower entity dead ahead on a closing path must
    // produce a nonzero lateral steer; a receding one must produce none.
    std::vector<EntityDist> ahead;
    ahead.push_back(EntityDist{glm::length(oncoming.position() - us.position()), &oncoming});
    glm::vec3 steerAway = steer.avoidNeighbors(8.0f, ahead);
    CHECK(finiteVec(steerAway));
    CHECK(glm::length(steerAway) > 0.0f);

    std::vector<EntityDist> awayFrom;
    awayFrom.push_back(EntityDist{glm::length(receding.position() - fast.position()), &receding});
    glm::vec3 steerNone = steerFast.avoidNeighbors(8.0f, awayFrom);
    CHECK(near(steerNone, glm::vec3(0.0f)));
}

// --- PathfindBehavior line-of-sight short-circuit ---------------------------

void testPathfindLineOfSight()
{
    struct ToggleVisibility final : WaypointVisibility
    {
        bool visible = false;
        bool isVisible(const glm::vec3&, const glm::vec3&) const override
        {
            return visible;
        }
    };

    Scene scene;

    // A waypoint far off the direct line, so "heading to the waypoint" and
    // "heading straight to the goal" are distinguishable by Z position.
    WaypointNetwork network;
    Waypoint* wpDetour =
        new Waypoint(glm::vec3(5.0f, 0.0f, 30.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    network.addWaypoint(wpDetour);

    ToggleVisibility visibility;
    visibility.visible = false; // goal not visible yet - keep following the seeded route

    Agent::Settings settings = defaultAgentSettings();
    Agent* member = makeAgent(scene, settings, "member");
    scene.update(0.0f);
    member->setWaypointNetwork(&network);
    member->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    member->setGoal(glm::vec3(20.0f, 0.0f, 0.0f));
    member->setGoalRadius(1.0f);
    // Seed a "mid-route, no LOS yet" state directly (findPath() itself is
    // covered by testWaypointNetwork/testSquadMovement) - this test is only
    // about the LOS short-circuit in PathfindBehavior::iterate.
    member->setNextWaypoint(wpDetour->id());

    member->addBehavior<PathfindBehavior>(PathfindBehavior::Settings{
        0.3f, 1.0f, 0.0f, 25.0f, 0.05f, 0.0f, glm::vec3(0.0f, 1.0f, 0.0f), &network,
        &visibility});

    // No LOS: the member walks toward the seeded waypoint, off toward +Z.
    for (int i = 0; i < 30; ++i)
        scene.updateAgents(0.016f);
    CHECK(finiteVec(member->position()));
    CHECK(member->position().z > 1.0f);

    // LOS opens up: the next poll (<= 0.05s away) must clear the waypoint and
    // switch to heading straight for the goal.
    visibility.visible = true;
    for (int i = 0; i < 10; ++i)
        scene.updateAgents(0.016f);
    CHECK(member->losStatus());
    CHECK(member->nextWaypoint() == 0);
    CHECK(!member->hasValidPath());

    for (int i = 0; i < 400; ++i)
        scene.updateAgents(0.016f);

    CHECK(finiteVec(member->position()));
    // Walked toward the goal along +X, back off the Z=30 detour.
    CHECK(member->position().x > 10.0f);
    CHECK(std::fabs(member->position().z) < 5.0f);
}

// --- FormationBehavior -------------------------------------------------------
//
// Diamond/Abreast/SingleFile offsets are ported verbatim from
// docs/ai/AI_Demo/Source/FormationBehavior.cpp; these lock the placement
// numbers in. Pentagon's mirrored flank direction is a deliberate deviation
// from that reference (see the comment in FormationBehavior.cpp) - checked
// here for the symmetry it is supposed to have instead.

Agent* makeFormationLeader(Scene& scene, const Agent::Settings& settings)
{
    Agent* leader = makeAgent(scene, settings, "leader");
    leader->setSquadId(0);
    leader->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    leader->setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward = +Z
    return leader;
}

void testFormationAbreast()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    Agent* leader = makeFormationLeader(scene, settings);
    Agent* pointMan = makeAgent(scene, settings, "pointMan");
    Agent* rightFlank = makeAgent(scene, settings, "rightFlank");
    scene.update(0.0f); // register all three before FormationBehavior scans them

    leader->setSquadFormation(static_cast<int>(SquadFormation::Abreast));

    pointMan->setSquadId(1);
    pointMan->setPosition(glm::vec3(0.0f, 0.0f, 5.0f));
    pointMan->setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    pointMan->setGoal(glm::vec3(0.0f, 0.0f, 20.0f));

    rightFlank->setSquadId(2);
    rightFlank->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));

    // Each member owns its own FormationBehavior instance - the leader/
    // point-man cache in it is no longer shared (DESVIO 2).
    pointMan->addBehavior<FormationBehavior>(1.0f, 1.0f);
    rightFlank->addBehavior<FormationBehavior>(1.0f, 1.0f);

    scene.updateAgents(0.016f);

    // Abreast case 2: goal = pointMan.position + pointManRight * 40.
    // pointManRight is +X (side vector) while pointMan faces +Z.
    CHECK(near(rightFlank->goal(), pointMan->position() + glm::vec3(40.0f, 0.0f, 0.0f), 0.01f));
}

void testFormationPentagonSymmetry()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    Agent* leader = makeFormationLeader(scene, settings);
    Agent* pointMan = makeAgent(scene, settings, "pointMan");
    Agent* rightFlank = makeAgent(scene, settings, "rightFlank");
    Agent* leftFlank = makeAgent(scene, settings, "leftFlank");
    scene.update(0.0f);

    leader->setSquadFormation(static_cast<int>(SquadFormation::Pentagon));

    pointMan->setSquadId(1);
    pointMan->setPosition(glm::vec3(1.0f, 0.0f, 1.0f));
    pointMan->setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

    rightFlank->setSquadId(2);
    rightFlank->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));

    leftFlank->setSquadId(3);
    leftFlank->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));

    pointMan->addBehavior<FormationBehavior>(1.0f, 1.0f);
    rightFlank->addBehavior<FormationBehavior>(1.0f, 1.0f);
    leftFlank->addBehavior<FormationBehavior>(1.0f, 1.0f);

    scene.updateAgents(0.016f);

    // Pentagon's flank *goal* positions only use mLeaderLook/mLeaderRight, not
    // v1/v2, so they are symmetric regardless of the deviation. What v1/v2
    // drive is the flank's facing direction: case 2 (right) faces v1 =
    // leaderLook rotated +45 about Y, case 3 (left) faces v2 = leaderLook
    // rotated -45 about Y. With the leader facing +Z those two facings must
    // be mirror images across the look axis (X negated, Z equal).
    glm::vec3 forwardRight = glm::mat3_cast(rightFlank->orientation())[2];
    glm::vec3 forwardLeft = glm::mat3_cast(leftFlank->orientation())[2];
    CHECK(std::fabs(forwardRight.x + forwardLeft.x) < 0.01f);
    CHECK(std::fabs(forwardRight.z - forwardLeft.z) < 0.01f);
}

// --- Phase 1 tests: Agent replacing AI::World/Group/Entity ------------------

// Test 1 (Fase 7 #1): every GameObject destroyed, in a shuffled (not
// creation) order, must take its Agent's Scene registration with it -
// exactly the bug radion-fisica-destrutor-desregista-scene already had for
// RigidBody. Scene::mAgents is never reserve()d, so it reallocates as
// agents register; ASan (this test binary's default) turns a dangling
// pointer left behind by a bad deregistration into a hard failure rather
// than a silent corruption.
void testAgentUnregisterInShuffledOrder()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    constexpr int kCount = 37;
    std::vector<GameObject*> objects;
    for (int i = 0; i < kCount; ++i)
        objects.push_back(makeAgent(scene, settings, "agent")->owner());
    scene.update(0.0f);
    CHECK(scene.agentCount() == static_cast<usize>(kCount));

    // Fisher-Yates over a fixed seed: deterministic, but not creation order.
    std::vector<int> order(static_cast<usize>(kCount));
    for (int i = 0; i < kCount; ++i)
        order[static_cast<usize>(i)] = i;
    std::srand(4242);
    for (int i = kCount - 1; i > 0; --i)
    {
        const int j = std::rand() % (i + 1);
        std::swap(order[static_cast<usize>(i)], order[static_cast<usize>(j)]);
    }

    int remaining = kCount;
    for (int index : order)
    {
        CHECK(scene.destroy(objects[static_cast<usize>(index)]));
        scene.update(0.0f);
        --remaining;
        CHECK(scene.agentCount() == static_cast<usize>(remaining));
    }
    CHECK(scene.agentCount() == 0);

    // If a deregistration were missed, this dereferences the freed Agent.
    scene.updateAgents(0.016f);
}

// Test 2 (Fase 7 #2): groupId() replaces AI::Group membership. A groupId of
// 0 means "no group" and must see no group members even standing among
// others; enemy masks are independent of groupId and must still cross group
// boundaries, exactly as updateEnemyVisibility() scanned every Group in the
// World regardless of which one an agent belonged to.
void testSensingByGroupId()
{
    Scene scene;
    Agent::Settings friendlySettings = defaultAgentSettings(); // type = 1
    Agent::Settings enemySettings = defaultAgentSettings();
    enemySettings.type = 2;

    Agent* a1 = makeAgent(scene, friendlySettings, "a1");
    Agent* a2 = makeAgent(scene, friendlySettings, "a2");
    Agent* b1 = makeAgent(scene, enemySettings, "b1");
    Agent* b2 = makeAgent(scene, enemySettings, "b2");
    Agent* lone = makeAgent(scene, friendlySettings, "lone");
    scene.update(0.0f);

    a1->setGroupId(1);
    a2->setGroupId(1);
    b1->setGroupId(2);
    b2->setGroupId(2);
    CHECK(lone->groupId() == 0); // default: no group

    a1->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    a2->setPosition(glm::vec3(1.0f, 0.0f, 0.0f));
    b1->setPosition(glm::vec3(2.0f, 0.0f, 0.0f));
    b2->setPosition(glm::vec3(3.0f, 0.0f, 0.0f));
    lone->setPosition(glm::vec3(0.5f, 0.0f, 0.0f));

    scene.updateAgents(0.016f);

    // No group (id 0) never populates visibleGroupMembers(), even standing
    // in the middle of two groups well within sense range.
    CHECK(lone->visibleGroupMembers().empty());

    // Same-group sensing is unaffected by the switch from Group pointers to
    // plain ids.
    CHECK(a1->visibleGroupMembers().size() == 1);
    CHECK(a1->visibleGroupMembers()[0].entity == a2);
    CHECK(b1->visibleGroupMembers().size() == 1);
    CHECK(b1->visibleGroupMembers()[0].entity == b2);

    // Enemy masks cross groups: a1's enemyMask (~1) flags type 2, regardless
    // of a1/b1/b2 belonging to different groups. Sorted by distance.
    CHECK(a1->visibleEnemies().size() == 2);
    CHECK(a1->visibleEnemies()[0].entity == b1);
    CHECK(a1->visibleEnemies()[1].entity == b2);
}

// Test 3 (Fase 7 #3): the 180 degree turn pushOwnerPose()/pullAgentPose()
// apply to reconcile Agent's forward = +Z with GameObject::forward() = -Z.
void testAgentPoseSyncFlipsForward()
{
    Scene scene;
    Agent* agent = makeAgent(scene, defaultAgentSettings(), "agent");
    scene.update(0.0f); // register before touching velocity/orientation

    agent->setVelocity(glm::vec3(1.0f, 0.0f, 0.0f));
    agent->alignWithVelocity();
    CHECK(near(agent->forward(), glm::vec3(1.0f, 0.0f, 0.0f)));

    // pullAgentPose() is private (friend Scene) - driven the same way
    // Scene::update() drives it every frame, through the AI block.
    scene.update(0.016f);

    CHECK(near(agent->owner()->forward(), glm::vec3(1.0f, 0.0f, 0.0f), 0.0001f));
}

// Test 4 (Fase 7 #4): behaviors are owned - the agent builds them, deletes
// them on removal, and frees whatever is left exactly once (a double free or
// a leak both fail this test's run under ASan even though nothing here
// CHECK()s it directly). No `new` appears anywhere in it, which is the
// point: there is no window in which an allocation belongs to nobody.
void testAgentBehaviorsAreOwned()
{
    Scene scene;
    Agent* a = makeAgent(scene, defaultAgentSettings(), "a");
    scene.update(0.0f);

    Behavior* separation = a->addBehavior<SeparationBehavior>(4.0f, 0.2f, 1.0f);
    CHECK(separation != nullptr);
    CHECK(a->behaviorCount() == 1);
    CHECK(a->behaviorAt(0) == separation);
    CHECK(a->behavior(BehaviorType::Separation) == separation);

    Behavior* cohesion = a->addBehavior<CohesionBehavior>(1.0f);
    CHECK(a->behaviorCount() == 2);

    // By type, which is how the editor and the loader will reach them.
    CHECK(a->removeBehavior(BehaviorType::Separation));
    CHECK(a->behaviorCount() == 1);
    CHECK(a->behaviorAt(0) == cohesion);
    CHECK(a->behavior(BehaviorType::Separation) == nullptr);
    CHECK(!a->removeBehavior(BehaviorType::Separation)); // already gone

    // The runtime-typed door, for a name out of a combo box or a save file.
    Behavior* wander = a->addBehavior(BehaviorType::Wander);
    CHECK(wander != nullptr);
    CHECK(wander->type() == BehaviorType::Wander);
    CHECK(a->behaviorCount() == 2);

    // A behavior belongs to exactly one agent. Handing an owned one to a
    // second agent is refused - and, since adoptBehavior() destroys what it
    // refuses, there is no orphaned allocation either way.
    Agent* b = makeAgent(scene, defaultAgentSettings(), "b");
    scene.update(0.0f);
    CHECK(!b->adoptBehavior(cohesion));
    CHECK(b->behaviorCount() == 0);
    CHECK(a->behaviorCount() == 2);
    CHECK(cohesion->owner() == a);

    CHECK(b->adoptBehavior(new AlignmentBehavior(1.0f))); // unowned - accepted
    CHECK(b->behaviorCount() == 1);
    CHECK(!b->adoptBehavior(nullptr));

    CHECK(scene.destroy(a->owner()));
    CHECK(scene.destroy(b->owner()));
    scene.update(0.0f);
    CHECK(scene.agentCount() == 0);
}

// An agent that cannot reach its goal must not search for it every frame.
// A failed search leaves the path empty, which is the very condition that
// triggers the search - so without a rate limit it was a full A* per agent
// per frame, worst exactly when the graph is hardest to search.
void testPathfindRepathIsRateLimited()
{
    Scene scene;

    // Two waypoints with no edge between them: reachable by neither, so
    // every search fails and the retry path is the one under test.
    WaypointNetwork network;
    Waypoint* wpStart =
        new Waypoint(glm::vec3(0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    Waypoint* wpIsland =
        new Waypoint(glm::vec3(80.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    network.addWaypoint(wpStart);
    network.addWaypoint(wpIsland);

    Agent* agent = makeAgent(scene, defaultAgentSettings(), "walker");
    scene.update(0.0f);
    agent->setWaypointNetwork(&network);
    agent->setSquadId(1);
    agent->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    agent->setGoal(glm::vec3(80.0f, 0.0f, 0.0f));
    agent->setGoalRadius(2.0f);

    NoVisibility blocked; // never any line of sight, so it must route
    PathfindBehavior* pathfind = agent->addBehavior<PathfindBehavior>(PathfindBehavior::Settings{
        0.2f, 2.0f, 0.0f, 25.0f, 0.05f, 1.0f, glm::vec3(0.0f, 1.0f, 0.0f), &network, &blocked});
    CHECK(pathfind != nullptr);
    if (!pathfind)
        return;
    CHECK(std::abs(pathfind->settings().repathInterval - 1.0f) < 1e-5f);

    // Half the interval: no search may have run yet, so nothing is set.
    for (u32 i = 0; i < 30; ++i)
        scene.updateAgents(1.0f / 60.0f);
    CHECK(agent->nextWaypoint() == 0);

    // Well past it: the search has been allowed to run, and still finds
    // nothing (the island has no edges) - which is the case that used to
    // retry forever.
    for (u32 i = 0; i < 300; ++i)
        scene.updateAgents(1.0f / 60.0f);
    CHECK(finiteVec(agent->position()));
    CHECK(agent->path().empty()); // no route exists, and none was invented
}

// Destroying a squad member takes it out of its leader's member list. It
// used to stay there as a dangling pointer, and the leader's next order
// walked straight into it.
void testDestroyedMemberLeavesTheSquad()
{
    Scene scene;
    Agent* leader = makeAgent(scene, defaultAgentSettings(), "leader");
    Agent* first = makeAgent(scene, defaultAgentSettings(), "first");
    Agent* second = makeAgent(scene, defaultAgentSettings(), "second");
    scene.update(0.0f);

    leader->setSquadId(0);
    first->setSquadId(1);
    second->setSquadId(2);
    leader->addSquadMember(first);
    leader->addSquadMember(second);
    CHECK(leader->squadMembers().size() == 2);
    CHECK(first->squadLeader() == leader);

    CHECK(scene.destroy(first->owner()));
    scene.update(0.0f);
    CHECK(leader->squadMembers().size() == 1);
    CHECK(leader->squadMembers()[0] == second);

    // Reaches every member: with the dead one still listed this is the
    // use-after-free.
    leader->setCommand(AI::SquadCommand::RallyToLeaderPosition);
    CHECK(second->command() == AI::SquadCommand::RallyToLeaderPosition);

    // The other direction: losing the leader leaves no dangling back
    // pointer on the members either.
    CHECK(scene.destroy(leader->owner()));
    scene.update(0.0f);
    CHECK(second->squadLeader() == nullptr);
}

// A leader ordered to a random waypoint with no network attached returns
// instead of dereferencing one: the guard used to sit one line below the
// dereference it was guarding.
void testLeaderWithoutWaypointNetwork()
{
    Scene scene;
    Agent* leader = makeAgent(scene, defaultAgentSettings(), "leader");
    Agent* member = makeAgent(scene, defaultAgentSettings(), "member");
    scene.update(0.0f);

    leader->setSquadId(0);
    member->setSquadId(1);
    leader->addSquadMember(member);
    CHECK(leader->waypointNetwork() == nullptr);

    leader->sendSquadToRandomWaypoint();
    CHECK(leader->selectedWaypoint() == nullptr);
    CHECK(member->goal() == glm::vec3(0.0f)); // never handed a destination
}

// --- BehaviorFactory (Fase 2) ------------------------------------------------

// Every BehaviorType round-trips through name()/fromName(), and create()
// hands back a live instance whose own type() agrees.
void testBehaviorFactoryRoundTrip()
{
    for (u8 i = 0; i < static_cast<u8>(BehaviorType::Count); ++i)
    {
        const BehaviorType type = static_cast<BehaviorType>(i);
        BehaviorType parsed = BehaviorType::Count;
        CHECK(BehaviorFactory::fromName(BehaviorFactory::name(type), parsed));
        CHECK(parsed == type);

        Behavior* behavior = BehaviorFactory::create(type);
        CHECK(behavior != nullptr);
        CHECK(behavior->type() == type);
        delete behavior;
    }

    // SteerBehavior is deliberately outside the registry (Steering.h): no
    // name, and BehaviorType::Count itself creates nothing.
    BehaviorType outType = BehaviorType::Count;
    CHECK(!BehaviorFactory::fromName("Steer", outType));
    CHECK(BehaviorFactory::create(BehaviorType::Count) == nullptr);
}

// Each registered behavior's paramCount() matches its own kParams table, and
// every parameter round-trips a written value back out through get/set by
// index.
void testBehaviorParamRoundTrip()
{
    struct Expected
    {
        BehaviorType type;
        u32 count;
    };
    const Expected expected[] = {
        {BehaviorType::Separation, 3},        {BehaviorType::Alignment, 1},
        {BehaviorType::Cohesion, 1},           {BehaviorType::Avoidance, 2},
        {BehaviorType::Cruising, 6},           {BehaviorType::StayWithinSphere, 2},
        {BehaviorType::Combat, 3},             {BehaviorType::Seek, 1},
        {BehaviorType::Flee, 1},               {BehaviorType::Wander, 0},
        {BehaviorType::ObstacleAvoidance, 1},  {BehaviorType::Pathfind, 6},
        {BehaviorType::NavMesh, 7},            {BehaviorType::Formation, 2},
    };

    for (const Expected& e : expected)
    {
        Behavior* behavior = BehaviorFactory::create(e.type);
        CHECK(behavior != nullptr);
        CHECK(behavior->paramCount() == e.count);

        for (u32 i = 0; i < behavior->paramCount(); ++i)
        {
            const BehaviorParam& info = behavior->paramInfo(i);
            CHECK(info.name != nullptr && info.name[0] != '\0');
            CHECK(info.tooltip != nullptr && info.tooltip[0] != '\0');

            if (info.kind == BehaviorParam::Kind::Float)
            {
                const f32 testValue =
                    glm::clamp(info.minValue + 0.5f, info.minValue, info.maxValue);
                behavior->setParamFloat(i, testValue);
                CHECK(std::fabs(behavior->paramFloat(i) - testValue) < 0.0001f);
            }
            else if (info.kind == BehaviorParam::Kind::Vec3)
            {
                const glm::vec3 testValue(1.5f, -2.5f, 3.5f);
                behavior->setParamVec3(i, testValue);
                CHECK(near(behavior->paramVec3(i), testValue));
            }
        }

        delete behavior;
    }
}

// --- Radion::Obstacle (Fase 2b) ----------------------------------------------

// Adding/removing the component enters/leaves the Scene's group; destroying
// the GameObject (not just removeComponent()) has to reach the same path.
void testObstacleRegistersWithScene()
{
    Scene scene;
    GameObject* object = scene.createGameObject("obstacle");
    Radion::Obstacle* obstacle = object->addComponent<Radion::Obstacle>();
    CHECK(obstacle != nullptr);
    scene.update(0.0f);

    CHECK(scene.obstacleCount() == 1);
    CHECK(scene.obstacles()[0] == obstacle);
    CHECK(scene.obstacleGroup()[0] == obstacle->obstacle());

    CHECK(scene.destroy(object));
    scene.update(0.0f);
    CHECK(scene.obstacleCount() == 0);
}

// Switching an obstacle off takes it out of the group the avoidance
// behaviors read, without taking it out of the scene - the rule colliders
// and lights already follow. Same for its object being deactivated.
void testInactiveObstacleLeavesTheGroup()
{
    Scene scene;
    GameObject* object = scene.createGameObject("obstacle");
    Radion::Obstacle* obstacle = object->addComponent<Radion::Obstacle>();
    obstacle->setSphere(2.0f);
    scene.update(0.0f);
    CHECK(scene.obstacleGroup().size() == 1);

    obstacle->setActive(false);
    scene.update(0.0f);
    CHECK(scene.obstacleCount() == 1); // still attached
    CHECK(scene.obstacleGroup().empty());

    obstacle->setActive(true);
    scene.update(0.0f);
    CHECK(scene.obstacleGroup().size() == 1);

    object->setActive(false);
    scene.update(0.0f);
    CHECK(scene.obstacleGroup().empty());

    object->setActive(true);
    scene.update(0.0f);
    CHECK(scene.obstacleGroup().size() == 1);
    CHECK(scene.obstacleGroup()[0] == obstacle->obstacle());
}

// Same shape as testAgentUnregisterInShuffledOrder(): N obstacles created
// WITHOUT reserve(), so mObstacleComponents/mObstacleGroup are guaranteed to
// reallocate, destroyed in a baralhada order. Each step also checks the two
// arrays stayed paired index for index through every swap-and-pop removal.
void testObstacleUnregisterInShuffledOrder()
{
    Scene scene;
    constexpr int kCount = 29;
    std::vector<GameObject*> objects;
    for (int i = 0; i < kCount; ++i)
    {
        GameObject* object = scene.createGameObject("obstacle");
        object->addComponent<Radion::Obstacle>();
        objects.push_back(object);
    }
    scene.update(0.0f);
    CHECK(scene.obstacleCount() == static_cast<usize>(kCount));

    std::vector<int> order(static_cast<usize>(kCount));
    for (int i = 0; i < kCount; ++i)
        order[static_cast<usize>(i)] = i;
    std::srand(777);
    for (int i = kCount - 1; i > 0; --i)
    {
        const int j = std::rand() % (i + 1);
        std::swap(order[static_cast<usize>(i)], order[static_cast<usize>(j)]);
    }

    int remaining = kCount;
    for (int index : order)
    {
        CHECK(scene.destroy(objects[static_cast<usize>(index)]));
        scene.update(0.0f);
        --remaining;
        CHECK(scene.obstacleCount() == static_cast<usize>(remaining));
        for (usize k = 0; k < scene.obstacleCount(); ++k)
            CHECK(scene.obstacleGroup()[k] == scene.obstacles()[k]->obstacle());
    }
    CHECK(scene.obstacleCount() == 0);

    // If a deregistration were missed, this walks over the freed Obstacle.
    scene.debugDrawObstacles();
}

// Swapping the shape reconstructs the owned AI::Obstacle instance (a new
// address, not the old one mutated in place) and the Scene's ObstacleGroup
// entry - refreshObstacle()'s whole job - follows it to the new address.
void testObstacleShapeSwapRebuildsInstance()
{
    Scene scene;
    GameObject* object = scene.createGameObject("obstacle");
    Radion::Obstacle* obstacle = object->addComponent<Radion::Obstacle>();
    scene.update(0.0f);

    obstacle->setSphere(2.0f);
    AI::Obstacle* sphereInstance = obstacle->obstacle();
    CHECK(sphereInstance != nullptr);
    CHECK(obstacle->shape() == ObstacleShape::Sphere);
    CHECK(obstacle->radius() == 2.0f);

    obstacle->setBox(1.0f, 2.0f, 3.0f);
    AI::Obstacle* boxInstance = obstacle->obstacle();
    CHECK(boxInstance != nullptr);
    CHECK(boxInstance != sphereInstance);
    CHECK(obstacle->shape() == ObstacleShape::Box);
    CHECK(obstacle->width() == 1.0f);
    CHECK(obstacle->height() == 2.0f);
    CHECK(obstacle->depth() == 3.0f);
    CHECK(scene.obstacleGroup()[0] == boxInstance);

    obstacle->setSeenFrom(AI::ObstacleSeenFrom::Both);
    CHECK(obstacle->seenFrom() == AI::ObstacleSeenFrom::Both);
    CHECK(obstacle->obstacle()->seenFrom() == AI::ObstacleSeenFrom::Both);

    CHECK(scene.destroy(object));
    scene.update(0.0f);
}

// ObstacleAvoidanceBehavior with no setObstacles() call at all: it has to
// find the sphere entirely through Agent::scene()->obstacleGroup(). Same
// geometry and same assertion as testObstacleAvoidance() above, which builds
// its own ObstacleGroup by hand - matching results is what proves the
// fallback wiring, not just that it does not crash.
void testObstacleAvoidanceReadsSceneGroup()
{
    Scene scene;

    GameObject* wall = scene.createGameObject("wall");
    wall->setPosition(glm::vec3(8.0f, 0.0f, 2.0f));
    Radion::Obstacle* wallObstacle = wall->addComponent<Radion::Obstacle>();
    wallObstacle->setSphere(3.0f);
    scene.update(0.0f);

    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 0.5f;
    Agent* vehicle = makeAgent(scene, settings, "vehicle");
    scene.update(0.0f);
    vehicle->setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    vehicle->setVelocity(glm::vec3(5.0f, 0.0f, 0.0f)); // moving +X
    vehicle->setOrientation(glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    vehicle->addBehavior<ObstacleAvoidanceBehavior>(2.0f); // no setObstacles() call

    scene.updateAgents(0.016f);
    CHECK(finiteVec(vehicle->desiredMove()));
    CHECK(vehicle->desiredMove().z < 0.0f);

    for (int i = 0; i < 600; ++i)
        scene.updateAgents(0.016f);

    CHECK(finiteVec(vehicle->position()));
    CHECK(vehicle->position().z < 0.0f);
    // Sphere centre/radius as set above (8, 0, 2), 3.0 - the vehicle must
    // have steered around it, not through it.
    CHECK(glm::length(vehicle->position() - glm::vec3(8.0f, 0.0f, 2.0f)) > 3.0f);
}

} // namespace

int main()
{
    testStateMachine();
    testWaypointNetwork();
    testGridPathfinder();
    testGridAlgorithms();
    testFlocking();
    testSquadMovement();
    testMemberStateMachine();
    testLeaderStateMachine();
    testPlaneAndRectangleObstacle();
    testBoxObstacle();
    testSphereObstacleSeenFrom();
    testCruisingAxisDistribution();
    testGridAStarOptimality();
    testStateMachineRemoveState();
    testStateMachineSurvivesCallbackMutation();
    testPointsOfInterest();
    testSteerLibrary();
    testPursuitEvasion();
    testDirectionalPredicates();
    testBoidNeighborhoodAndSeparation();
    testTargetSpeedClamp();
    testSeekFlee();
    testWander();
    testObstacleAvoidance();
    testNearestApproach();
    testPathfindLineOfSight();
    testFormationAbreast();
    testFormationPentagonSymmetry();
    testAgentUnregisterInShuffledOrder();
    testSensingByGroupId();
    testAgentPoseSyncFlipsForward();
    testAgentBehaviorsAreOwned();
    testPathfindRepathIsRateLimited();
    testDestroyedMemberLeavesTheSquad();
    testLeaderWithoutWaypointNetwork();
    testBehaviorFactoryRoundTrip();
    testBehaviorParamRoundTrip();
    testObstacleRegistersWithScene();
    testInactiveObstacleLeavesTheGroup();
    testObstacleUnregisterInShuffledOrder();
    testObstacleShapeSwapRebuildsInstance();
    testObstacleAvoidanceReadsSceneGroup();

    if (gFailures)
        std::fprintf(stderr, "%d AI test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
