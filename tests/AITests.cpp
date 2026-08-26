// AITests.cpp - smoke tests for the radion_ai library (runtime/ai).
//
// Covers the subsystems: state machine, waypoint network A*, grid A*,
// flocking, steering (seek/flee/wander/obstacle avoidance) and the squad
// glue that ties them together.

#include "PCH.h"

#include "AI.h"

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

bool finiteVec(const Math::Vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool near(const Math::Vec3& a, const Math::Vec3& b, float epsilon = 0.0001f)
{
    return glm::length(a - b) <= epsilon;
}

// A visibility functor that rejects everything (forces path generation).
struct NoVisibility final : WaypointVisibility
{
    bool isVisible(const Math::Vec3&, const Math::Vec3&) const override
    {
        return false;
    }
};

Entity::Settings defaultEntitySettings()
{
    Entity::Settings s;
    s.type = 1;
    s.senseRange = 8.0f;
    s.maxVelocityChange = 2.0f;
    s.maxSpeed = 5.0f;
    s.desiredSpeed = 2.0f;
    return s;
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
        new Waypoint(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    Waypoint* b =
        new Waypoint(Math::Vec3(10.0f, 0.0f, 0.0f), Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    Waypoint* c =
        new Waypoint(Math::Vec3(20.0f, 0.0f, 0.0f), Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    Waypoint* d =
        new Waypoint(Math::Vec3(30.0f, 0.0f, 0.0f), Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
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
    CHECK(network.findPath(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(30.0f, 0.0f, 0.0f),
                           WaypointVisibility(), posPath));
    CHECK(posPath.size() == 4);

    // Nothing visible -> no valid waypoint -> search fails.
    Path noPath;
    CHECK(!network.findPath(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(30.0f, 0.0f, 0.0f),
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
    World world;
    Group* group = new Group(world);
    world.add(*group);

    Entity::Settings settings = defaultEntitySettings();
    // Group owns its entities, so they must be heap-allocated (the World ->
    // Group -> Entity ownership chain deletes them).
    Entity* a = new Entity(world, settings);
    Entity* b = new Entity(world, settings);
    Entity* c = new Entity(world, settings);
    a->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    b->setPosition(Math::Vec3(3.0f, 0.0f, 0.0f));
    c->setPosition(Math::Vec3(1.5f, 0.0f, 2.5f));

    SeparationBehavior separation(4.0f, 0.2f, 1.0f);
    CohesionBehavior cohesion(2.0f);
    AlignmentBehavior alignment(1.0f);
    StayWithinSphereBehavior stayInSphere(Math::Vec3(0.0f, 0.0f, 0.0f), 20.0f);
    for (Entity* e : {a, b, c})
    {
        e->addBehavior(separation);
        e->addBehavior(cohesion);
        e->addBehavior(alignment);
        e->addBehavior(stayInSphere);
        group->add(*e);
    }

    // The three start separated; after one step every entity should have a
    // non-zero desired move (it sensed its flockmates).
    world.iterate(0.016f);
    CHECK(glm::length(a->desiredMove()) > 0.0f);
    CHECK(glm::length(b->desiredMove()) > 0.0f);
    CHECK(glm::length(c->desiredMove()) > 0.0f);

    // Run a while; the sim must stay finite and the flock roughly together.
    for (int i = 0; i < 300; ++i)
        world.iterate(0.016f);

    CHECK(finiteVec(a->position()));
    CHECK(finiteVec(b->position()));
    CHECK(finiteVec(c->position()));
    CHECK(glm::length(a->position() - b->position()) < 12.0f);
    CHECK(glm::length(b->position() - c->position()) < 12.0f);
    CHECK(glm::length(a->position() - Math::Vec3(0.0f, 0.0f, 0.0f)) < 25.0f);
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
    World world;
    Group* squad = new Group(world);
    world.add(*squad);

    Entity::Settings settings = defaultEntitySettings();

    // Simple two-waypoint network far along +X.
    WaypointNetwork network;
    Waypoint* wpStart =
        new Waypoint(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    Waypoint* wpGoal =
        new Waypoint(Math::Vec3(50.0f, 0.0f, 0.0f), Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    network.addWaypoint(wpStart);
    network.addWaypoint(wpGoal);
    wpStart->addEdge(NetworkEdge{wpGoal->id()});
    wpGoal->addEdge(NetworkEdge{wpStart->id()});

    PointsOfInterest pois;
    PointOfInterest* target = new PointOfInterest(Math::Vec3(50.0f, 0.0f, 0.0f), 5.0f);
    CHECK(pois.add(target));

    // Group owns its entities, so they must be heap-allocated.
    SquadLeaderEntity* leader = new SquadLeaderEntity(world, settings);
    leader->setWaypointNetwork(&network);
    leader->setSquadId(0);
    leader->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    squad->add(*leader);

    SquadMemberEntity* member = new SquadMemberEntity(world, settings);
    member->setWaypointNetwork(&network);
    member->setSquadId(1);
    member->setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
    squad->add(*member);

    leader->addSquadMember(member);

    // Force pathfinding: block line of sight so the member has to route.
    PathfindBehavior pathfind(PathfindBehavior::Settings{
        0.2f, 50.0f, 0.0f, 25.0f, 0.5f, Math::Vec3(0.0f, 1.0f, 0.0f), &network, nullptr});
    member->addBehavior(pathfind);

    StateMachine* memberMachine = buildMemberStateMachine(*member);
    member->setStateMachine(memberMachine);
    StateMachine* leaderMachine = buildLeaderStateMachine(*leader);
    leader->setStateMachine(leaderMachine);

    // Order the squad to the target POI.
    leader->setPointsOfInterest(&pois);
    leader->setSelectedPointOfInterest(target);
    leader->setCommand(SquadCommand::AttackTarget);

    for (int i = 0; i < 600; ++i)
    {
        world.iterate(0.016f);
        if (!finiteVec(member->position()))
            break;
    }

    CHECK(finiteVec(member->position()));
    // With no LOS the member routes through the network and walks far along +X.
    CHECK(member->position().x > 15.0f);

    delete memberMachine;
    delete leaderMachine;
}

void testMemberStateMachine()
{
    World world;
    Group* squad = new Group(world);
    world.add(*squad);

    Entity::Settings settings = defaultEntitySettings();

    WaypointNetwork network;
    Waypoint* wp =
        new Waypoint(Math::Vec3(50.0f, 0.0f, 0.0f), Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    network.addWaypoint(wp);

    SquadMemberEntity* member = new SquadMemberEntity(world, settings);
    member->setWaypointNetwork(&network);
    member->setSquadId(1);
    member->setPosition(Math::Vec3(50.5f, 0.0f, 0.0f)); // inside wp radius
    member->setGoal(Math::Vec3(50.0f, 0.0f, 0.0f));
    squad->add(*member);

    StateMachine* machine = buildMemberStateMachine(*member);
    member->setStateMachine(machine);

    CHECK(machine->currentState()->name() == "WaitingForCommand");

    // A valid path + a non-standground command leaves WaitingForCommand.
    member->setCommand(SquadCommand::AttackTarget);
    member->setNextWaypoint(0);
    member->setPath(Path{wp->id()});
    member->iterate(0.016f);
    CHECK(machine->currentState()->name() == "MovingToGoal");

    // Member sits inside the waypoint radius -> WaypointReached.
    member->iterate(0.016f);
    CHECK(machine->currentState()->name() == "WaypointReached");

    // Standing ground pulls it back to WaitingForCommand.
    member->setCommand(SquadCommand::StandGround);
    member->iterate(0.016f);
    CHECK(machine->currentState()->name() == "WaitingForCommand");

    delete machine;
}

void testLeaderStateMachine()
{
    World world;
    Group* squad = new Group(world);
    world.add(*squad);

    Entity::Settings settings = defaultEntitySettings();

    SquadLeaderEntity* leader = new SquadLeaderEntity(world, settings);
    leader->setSquadId(0);
    leader->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    squad->add(*leader);

    SquadMemberEntity* member = new SquadMemberEntity(world, settings);
    member->setSquadId(1);
    member->setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
    member->setGoal(member->position()); // already at its goal
    squad->add(*member);
    leader->addSquadMember(member);

    StateMachine* machine = buildLeaderStateMachine(*leader);
    leader->setStateMachine(machine);

    CHECK(machine->currentState()->name() == "AwaitingSquadTaskCompletion");

    // Squad is at its goal -> issue the next command.
    leader->iterate(0.016f);
    CHECK(machine->currentState()->name() == "CommandSquadToPOI");

    // Stand ground -> CommandSquadToPOI hands off to StandingGround.
    leader->setCommand(SquadCommand::StandGround);
    leader->iterate(0.016f);
    CHECK(machine->currentState()->name() == "StandingGround");
    leader->iterate(0.016f);
    CHECK(machine->currentState()->name() == "StandingGround");

    // New command leaves StandingGround.
    leader->setCommand(SquadCommand::PatrolPointsOfInterest);
    leader->iterate(0.016f);
    CHECK(machine->currentState()->name() == "CommandSquadToPOI");

    delete machine;
}

// --- steering --------------------------------------------------------------

void testSteerLibrary()
{
    World world;
    Entity::Settings settings = defaultEntitySettings();
    Entity e(world, settings);
    e.setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    e.setVelocity(Math::Vec3(2.0f, 0.0f, 0.0f));
    e.setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)); // forward = +Z

    SteerLibrary steer(e);

    // seek: (target - position) - velocity
    CHECK(near(steer.seek(Math::Vec3(10.0f, 0.0f, 0.0f)), Math::Vec3(8.0f, 0.0f, 0.0f)));
    // flee: (position - target) - velocity
    CHECK(near(steer.flee(Math::Vec3(10.0f, 0.0f, 0.0f)), Math::Vec3(-12.0f, 0.0f, 0.0f)));
    // targetSpeed: forward * clip(target - current, -maxForce, +maxForce); maxForce = 2
    CHECK(near(steer.targetSpeed(5.0f), Math::Vec3(0.0f, 0.0f, 2.0f)));
    // predictFuturePosition
    CHECK(near(e.predictFuturePosition(1.0f), Math::Vec3(2.0f, 0.0f, 0.0f)));
    // local/global transforms round-trip
    CHECK(near(e.globalizePosition(e.localizePosition(Math::Vec3(3.0f, 4.0f, 5.0f))),
               Math::Vec3(3.0f, 4.0f, 5.0f)));
}

void testPlaneAndRectangleObstacle()
{
    World world;
    Entity::Settings settings = defaultEntitySettings();
    settings.radius = 0.5f;
    Entity vehicle(world, settings);

    // Default plane: XY at the origin, +Z half-space is outside.
    PlaneObstacle plane;

    // Facing the plane head-on from the outside.
    vehicle.setPosition(Math::Vec3(1.0f, 2.0f, 5.0f));
    vehicle.setOrientation(glm::angleAxis(glm::pi<f32>(), Math::Vec3(0.0f, 1.0f, 0.0f))); // -Z
    PathIntersection pi;
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 5.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, Math::Vec3(1.0f, 2.0f, 0.0f), 0.001f));
    CHECK(near(pi.surfaceNormal, Math::Vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(pi.vehicleOutside);

    // Heading away from the plane: no intersection.
    vehicle.setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)); // +Z
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    // Path parallel to the plane: no intersection.
    vehicle.setOrientation(glm::angleAxis(glm::radians(90.0f), Math::Vec3(0.0f, 1.0f, 0.0f))); // +X
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    // Behind an Outside-only plane, heading into it: the back is not solid.
    vehicle.setPosition(Math::Vec3(0.0f, 0.0f, -5.0f));
    vehicle.setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)); // +Z, toward the plane
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    // The same approach against an Inside-only plane does hit, normal -Z.
    plane.setSeenFrom(ObstacleSeenFrom::Inside);
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(near(pi.surfaceNormal, Math::Vec3(0.0f, 0.0f, -1.0f), 0.001f));
    CHECK(!pi.vehicleOutside);

    // A 2x2 rectangle only blocks paths crossing inside its (radius-grown) bounds.
    RectangleObstacle rect(2.0f, 2.0f);
    vehicle.setPosition(Math::Vec3(0.5f, 0.5f, 5.0f));
    vehicle.setOrientation(glm::angleAxis(glm::pi<f32>(), Math::Vec3(0.0f, 1.0f, 0.0f))); // -Z
    rect.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 5.0f) < 0.001f);

    // Crossing the plane well outside the rectangle: miss.
    vehicle.setPosition(Math::Vec3(5.0f, 0.0f, 5.0f));
    rect.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    // Grazing inside the vehicle-radius-grown edge (half-width 1 + radius 0.5).
    vehicle.setPosition(Math::Vec3(1.4f, 0.0f, 5.0f));
    rect.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
}

void testBoxObstacle()
{
    World world;
    Entity::Settings settings = defaultEntitySettings();
    settings.radius = 0.0f; // exact face bounds, no radius growth
    Entity vehicle(world, settings);

    // 2x2x2 box at the origin, world-aligned.
    BoxObstacle box(2.0f, 2.0f, 2.0f, Math::Vec3(1.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f),
                    Math::Vec3(0.0f, 0.0f, 1.0f), Math::Vec3(0.0f));

    // Head-on along -Z: hits the front face at z = +1, steer hint outward.
    vehicle.setPosition(Math::Vec3(0.0f, 0.0f, 5.0f));
    vehicle.setOrientation(glm::angleAxis(glm::pi<f32>(), Math::Vec3(0.0f, 1.0f, 0.0f))); // -Z
    PathIntersection pi;
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 4.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, Math::Vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(near(pi.steerHint, Math::Vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(pi.vehicleOutside);
    CHECK(pi.obstacle == &box);

    // Head-on along -X: hits the +X side face at x = +1.
    vehicle.setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
    vehicle.setOrientation(glm::angleAxis(glm::radians(-90.0f), Math::Vec3(0.0f, 1.0f, 0.0f)));
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 4.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, Math::Vec3(1.0f, 0.0f, 0.0f), 0.001f));
    CHECK(near(pi.steerHint, Math::Vec3(1.0f, 0.0f, 0.0f), 0.001f));

    // Descending onto the top: hits the +Y face at y = +1.
    vehicle.setPosition(Math::Vec3(0.0f, 5.0f, 0.0f));
    vehicle.setOrientation(glm::angleAxis(glm::radians(90.0f), Math::Vec3(1.0f, 0.0f, 0.0f)));
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 4.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, Math::Vec3(0.0f, 1.0f, 0.0f), 0.001f));
    CHECK(near(pi.steerHint, Math::Vec3(0.0f, 1.0f, 0.0f), 0.001f));

    // A path passing beside the box misses every face.
    vehicle.setPosition(Math::Vec3(5.0f, 5.0f, 5.0f));
    vehicle.setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)); // +Z, away
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);
}

void testSphereObstacleSeenFrom()
{
    World world;
    Entity::Settings settings = defaultEntitySettings();
    settings.radius = 0.5f;
    Entity vehicle(world, settings);

    // A vehicle outside an Inside-only sphere must be pulled back toward it.
    SphereObstacle pen(2.0f, Math::Vec3(0.0f));
    pen.setSeenFrom(ObstacleSeenFrom::Inside);
    vehicle.setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    vehicle.setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f));
    PathIntersection pi;
    pen.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(pi.distance == 0.0f);
    CHECK(near(pi.steerHint, Math::Vec3(-1.0f, 0.0f, 0.0f), 0.001f));
    CHECK(pi.vehicleOutside);

    // Inside a hollow (Both) shell, the exit ahead pushes back inward.
    SphereObstacle shell(3.0f, Math::Vec3(0.0f));
    shell.setSeenFrom(ObstacleSeenFrom::Both);
    vehicle.setPosition(Math::Vec3(0.0f));
    shell.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(!pi.vehicleOutside);
    CHECK(std::fabs(pi.distance - 3.5f) < 0.001f); // radius + vehicle radius
    CHECK(near(pi.surfaceNormal, Math::Vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(near(pi.steerHint, Math::Vec3(0.0f, 0.0f, -1.0f), 0.001f));

    // A path whose closest approach is beyond the radius misses entirely.
    SphereObstacle ball(2.0f, Math::Vec3(0.0f));
    vehicle.setPosition(Math::Vec3(0.0f, 10.0f, 0.0f));
    ball.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);
}

void testCruisingAxisDistribution()
{
    World world;
    Entity::Settings settings = defaultEntitySettings();
    Entity e(world, settings);
    e.setVelocity(Math::Vec3(0.0f)); // below desiredSpeed, so signum is +1

    // The flocking demo's own chances: X larger than Y. The per-axis chances
    // form cumulative bands, so Y must still fire - with a raw-roll compare
    // it never could (its band was a subset of X's).
    CruisingBehavior cruising(0.45f, 0.2f, 0.35f, 1.0f, 0.5f, 0.2f);

    std::srand(12345);
    int hits[3] = {0, 0, 0};
    for (int i = 0; i < 2000; ++i)
    {
        e.setDesiredMove(Math::Vec3(0.0f));
        cruising.iterate(0.016f, e);
        const Math::Vec3 move = e.desiredMove();
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

void testPointsOfInterest()
{
    PointsOfInterest pois;
    PointOfInterest* first = new PointOfInterest(Math::Vec3(0.0f, 0.0f, 0.0f), 1.0f);
    PointOfInterest* second = new PointOfInterest(Math::Vec3(10.0f, 0.0f, 0.0f), 1.0f);
    PointOfInterest* third = new PointOfInterest(Math::Vec3(0.0f, 0.0f, 20.0f), 1.0f);
    CHECK(pois.add(first));
    CHECK(pois.add(second));
    CHECK(pois.add(third));

    CHECK(pois.find(first->id()) == first);
    CHECK(pois.findNearest(Math::Vec3(9.0f, 0.0f, 1.0f)) == second);
    CHECK(pois.findNearest(Math::Vec3(0.0f, 0.0f, 19.0f)) == third);

    // selectRandom never hands back the POI the caller is already at.
    for (int i = 0; i < 50; ++i)
    {
        const PointOfInterest* pick = pois.selectRandom(first->id());
        CHECK(pick != nullptr);
        CHECK(pick->id() != first->id());
    }

    CHECK(pois.remove(second->id()));
    CHECK(pois.find(second->id()) == nullptr);
}

void testPursuitEvasion()
{
    World world;
    Entity::Settings settings = defaultEntitySettings();

    Entity hunter(world, settings);
    hunter.setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    hunter.setVelocity(Math::Vec3(0.0f, 0.0f, 2.0f));
    hunter.setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    Entity quarry(world, settings);
    quarry.setPosition(Math::Vec3(0.0f, 0.0f, 10.0f));

    SteerLibrary steer(hunter);

    // A parked quarry's predicted position is its position: pursuit == seek.
    CHECK(near(steer.pursuit(quarry), steer.seek(quarry.position())));

    // Ahead-parallel quarry: estimated intercept (10/2 * 4 = 20s) is capped
    // at maxPredictionTime, so the target is one second of quarry travel.
    quarry.setVelocity(Math::Vec3(0.0f, 0.0f, 3.0f));
    quarry.setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f));
    CHECK(near(steer.pursuit(quarry, 1.0f),
               steer.seek(quarry.position() + quarry.velocity() * 1.0f)));

    // A stationary menace is predicted at the cap rather than dividing by
    // zero, and its predicted position is where it already is.
    Entity menace(world, settings);
    menace.setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
    CHECK(near(steer.evasion(menace, 2.0f), steer.flee(menace.position())));

    // Slow distant menace: rough intercept (5s) exceeds the cap (2s), so the
    // flee target is two seconds of menace travel.
    menace.setVelocity(Math::Vec3(0.0f, 0.0f, 1.0f));
    CHECK(near(steer.evasion(menace, 2.0f),
               steer.flee(menace.position() + menace.velocity() * 2.0f)));
}

void testDirectionalPredicates()
{
    World world;
    Entity::Settings settings = defaultEntitySettings();
    Entity e(world, settings);
    e.setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    e.setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    SteerLibrary steer(e);
    CHECK(steer.isAhead(Math::Vec3(0.0f, 0.0f, 5.0f)));
    CHECK(!steer.isBehind(Math::Vec3(0.0f, 0.0f, 5.0f)));
    CHECK(steer.isBehind(Math::Vec3(0.0f, 0.0f, -5.0f)));
    CHECK(!steer.isAhead(Math::Vec3(0.0f, 0.0f, -5.0f)));
    CHECK(steer.isAside(Math::Vec3(5.0f, 0.0f, 0.0f)));
    CHECK(!steer.isAhead(Math::Vec3(0.0f, 0.0f, 0.0f))); // degenerate offset

    // Local frame convention: right = +X, up = +Y, forward = +Z.
    CHECK(near(e.localizeDirection(e.forward()), Math::Vec3(0.0f, 0.0f, 1.0f)));
    CHECK(near(e.localizeDirection(e.side()), Math::Vec3(1.0f, 0.0f, 0.0f)));
    CHECK(near(e.localizeDirection(e.up()), Math::Vec3(0.0f, 1.0f, 0.0f)));
    CHECK(near(e.globalizeDirection(Math::Vec3(0.0f, 0.0f, 1.0f)), e.forward()));

    // Right-handed yaw: +90 degrees about +Y swings forward from +Z to +X.
    e.setOrientation(glm::angleAxis(glm::radians(90.0f), Math::Vec3(0.0f, 1.0f, 0.0f)));
    CHECK(near(e.forward(), Math::Vec3(1.0f, 0.0f, 0.0f), 0.001f));
    CHECK(near(e.side(), Math::Vec3(0.0f, 0.0f, -1.0f), 0.001f));

    // alignWithVelocity regenerates a right-handed orthonormal frame with
    // forward along the velocity and up preserved.
    e.setVelocity(Math::Vec3(0.0f, 0.0f, -3.0f));
    e.alignWithVelocity();
    CHECK(near(e.forward(), Math::Vec3(0.0f, 0.0f, -1.0f), 0.001f));
    CHECK(near(e.up(), Math::Vec3(0.0f, 1.0f, 0.0f), 0.001f));
    CHECK(near(glm::cross(e.up(), e.forward()), e.side(), 0.001f));
}

void testBoidNeighborhoodAndSeparation()
{
    World world;
    Entity::Settings settings = defaultEntitySettings();
    Entity self(world, settings);
    self.setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    self.setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    Entity ahead(world, settings);
    ahead.setPosition(Math::Vec3(0.0f, 0.0f, 5.0f));
    Entity behind(world, settings);
    behind.setPosition(Math::Vec3(0.0f, 0.0f, -5.0f));
    Entity veryClose(world, settings);
    veryClose.setPosition(Math::Vec3(0.0f, 0.0f, -0.5f));
    Entity far(world, settings);
    far.setPosition(Math::Vec3(0.0f, 0.0f, 50.0f));

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
    CHECK(near(steer.separation(10.0f, -1.0f, flock), Math::Vec3(0.0f, 0.0f, -1.0f)));
    CHECK(near(steer.cohesion(10.0f, -1.0f, flock), Math::Vec3(0.0f, 0.0f, 1.0f)));
}

void testTargetSpeedClamp()
{
    World world;
    Entity::Settings settings = defaultEntitySettings(); // maxVelocityChange = 2
    Entity e(world, settings);
    e.setVelocity(Math::Vec3(2.0f, 0.0f, 0.0f)); // speed 2
    e.setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    SteerLibrary steer(e);
    // Already at the target speed: no correction.
    CHECK(near(steer.targetSpeed(2.0f), Math::Vec3(0.0f)));
    // Braking is clamped to maxForce, along -forward.
    CHECK(near(steer.targetSpeed(-5.0f), Math::Vec3(0.0f, 0.0f, -2.0f)));
}

void testSeekFlee()
{
    World world;
    Group* group = new Group(world);
    world.add(*group);

    Entity::Settings settings = defaultEntitySettings();

    // A seeker converges on a target far along +X.
    Entity* chaser = new Entity(world, settings);
    chaser->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    group->add(*chaser);
    const Math::Vec3 target(100.0f, 0.0f, 0.0f);
    SeekBehavior seek(target);
    chaser->addBehavior(seek);

    for (int i = 0; i < 600; ++i)
        world.iterate(0.016f);

    CHECK(finiteVec(chaser->position()));
    CHECK(chaser->position().x > 30.0f);

    // A runner flees from the same target and ends up on the opposite side.
    Entity* runner = new Entity(world, settings);
    runner->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    group->add(*runner);
    FleeBehavior flee(target);
    runner->addBehavior(flee);

    for (int i = 0; i < 600; ++i)
        world.iterate(0.016f);

    CHECK(finiteVec(runner->position()));
    CHECK(runner->position().x < -30.0f);
}

void testWander()
{
    World world;
    Group* group = new Group(world);
    world.add(*group);

    Entity::Settings settings = defaultEntitySettings();
    Entity* e = new Entity(world, settings);
    e->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    group->add(*e);

    WanderBehavior wander;
    e->addBehavior(wander);

    for (int i = 0; i < 300; ++i)
        world.iterate(0.016f);

    // Wandered away from the start, stayed finite and bounded.
    CHECK(finiteVec(e->position()));
    CHECK(glm::length(e->position()) > 0.001f);
    CHECK(glm::length(e->position()) < 50.0f);
}

void testObstacleAvoidance()
{
    World world;
    Group* group = new Group(world);
    world.add(*group);

    Entity::Settings settings = defaultEntitySettings();
    settings.radius = 0.5f;

    // A sphere slightly off the +X travel line so there is a lateral component.
    SphereObstacle sphere(3.0f, Math::Vec3(8.0f, 0.0f, 2.0f));
    ObstacleGroup obstacles;
    obstacles.push_back(&sphere);

    Entity* vehicle = new Entity(world, settings);
    vehicle->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    vehicle->setVelocity(Math::Vec3(5.0f, 0.0f, 0.0f)); // moving +X
    // Face +X so the vehicle's forward path intersects the sphere.
    vehicle->setOrientation(glm::angleAxis(glm::radians(90.0f), Math::Vec3(0.0f, 1.0f, 0.0f)));
    group->add(*vehicle);

    ObstacleAvoidanceBehavior avoid(2.0f, obstacles);
    vehicle->addBehavior(avoid);

    world.iterate(0.016f);
    // The avoidance force is lateral: it must push toward -Z (past the sphere).
    CHECK(finiteVec(vehicle->desiredMove()));
    CHECK(vehicle->desiredMove().z < 0.0f);

    for (int i = 0; i < 600; ++i)
        world.iterate(0.016f);

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
    World world;
    Entity::Settings settings = defaultEntitySettings();
    settings.radius = 1.0f;

    // Head-on: we move +X, the other moves -X from further down +X. Closing
    // distance, so the nearest approach must be in the future (time > 0).
    Entity us(world, settings);
    us.setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    us.setVelocity(Math::Vec3(1.0f, 0.0f, 0.0f));

    Entity oncoming(world, settings);
    oncoming.setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    oncoming.setVelocity(Math::Vec3(-1.0f, 0.0f, 0.0f));

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
    Entity receding(world, settings);
    receding.setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    receding.setVelocity(Math::Vec3(1.0f, 0.0f, 0.0f)); // same direction as us, but faster gap
    Entity fast(world, settings);
    fast.setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    fast.setVelocity(Math::Vec3(-1.0f, 0.0f, 0.0f));
    SteerLibrary steerFast(fast);
    const float tReceding = steerFast.predictNearestApproachTime(receding);
    CHECK(tReceding < 0.0f);

    // avoidNeighbors: a slower entity dead ahead on a closing path must
    // produce a nonzero lateral steer; a receding one must produce none.
    std::vector<EntityDist> ahead;
    ahead.push_back(EntityDist{glm::length(oncoming.position() - us.position()), &oncoming});
    Math::Vec3 steerAway = steer.avoidNeighbors(8.0f, ahead);
    CHECK(finiteVec(steerAway));
    CHECK(glm::length(steerAway) > 0.0f);

    std::vector<EntityDist> awayFrom;
    awayFrom.push_back(EntityDist{glm::length(receding.position() - fast.position()), &receding});
    Math::Vec3 steerNone = steerFast.avoidNeighbors(8.0f, awayFrom);
    CHECK(near(steerNone, Math::Vec3(0.0f)));
}

// --- PathfindBehavior line-of-sight short-circuit ---------------------------

void testPathfindLineOfSight()
{
    struct ToggleVisibility final : WaypointVisibility
    {
        bool visible = false;
        bool isVisible(const Math::Vec3&, const Math::Vec3&) const override
        {
            return visible;
        }
    };

    World world;
    Group* squad = new Group(world);
    world.add(*squad);

    // A waypoint far off the direct line, so "heading to the waypoint" and
    // "heading straight to the goal" are distinguishable by Z position.
    WaypointNetwork network;
    Waypoint* wpDetour =
        new Waypoint(Math::Vec3(5.0f, 0.0f, 30.0f), Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    network.addWaypoint(wpDetour);

    ToggleVisibility visibility;
    visibility.visible = false; // goal not visible yet - keep following the seeded route

    Entity::Settings settings = defaultEntitySettings();
    SquadMemberEntity* member = new SquadMemberEntity(world, settings);
    member->setWaypointNetwork(&network);
    member->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    member->setGoal(Math::Vec3(20.0f, 0.0f, 0.0f));
    member->setGoalRadius(1.0f);
    // Seed a "mid-route, no LOS yet" state directly (findPath() itself is
    // covered by testWaypointNetwork/testSquadMovement) - this test is only
    // about the LOS short-circuit in PathfindBehavior::iterate.
    member->setNextWaypoint(wpDetour->id());
    squad->add(*member);

    PathfindBehavior pathfind(PathfindBehavior::Settings{
        0.3f, 1.0f, 0.0f, 25.0f, 0.05f, Math::Vec3(0.0f, 1.0f, 0.0f), &network, &visibility});
    member->addBehavior(pathfind);

    // No LOS: the member walks toward the seeded waypoint, off toward +Z.
    for (int i = 0; i < 30; ++i)
        world.iterate(0.016f);
    CHECK(finiteVec(member->position()));
    CHECK(member->position().z > 1.0f);

    // LOS opens up: the next poll (<= 0.05s away) must clear the waypoint and
    // switch to heading straight for the goal.
    visibility.visible = true;
    for (int i = 0; i < 10; ++i)
        world.iterate(0.016f);
    CHECK(member->losStatus());
    CHECK(member->nextWaypoint() == 0);
    CHECK(!member->hasValidPath());

    for (int i = 0; i < 400; ++i)
        world.iterate(0.016f);

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

SquadLeaderEntity* makeFormationLeader(World& world, Group& squad, const Entity::Settings& settings)
{
    SquadLeaderEntity* leader = new SquadLeaderEntity(world, settings);
    leader->setSquadId(0);
    leader->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    leader->setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)); // forward = +Z
    squad.add(*leader);
    return leader;
}

void testFormationAbreast()
{
    World world;
    Group* squad = new Group(world);
    world.add(*squad);
    Entity::Settings settings = defaultEntitySettings();

    SquadLeaderEntity* leader = makeFormationLeader(world, *squad, settings);
    leader->setSquadFormation(static_cast<int>(SquadFormation::Abreast));

    SquadMemberEntity* pointMan = new SquadMemberEntity(world, settings);
    pointMan->setSquadId(1);
    pointMan->setPosition(Math::Vec3(0.0f, 0.0f, 5.0f));
    pointMan->setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f));
    pointMan->setGoal(Math::Vec3(0.0f, 0.0f, 20.0f));
    squad->add(*pointMan);

    SquadMemberEntity* rightFlank = new SquadMemberEntity(world, settings);
    rightFlank->setSquadId(2);
    rightFlank->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    squad->add(*rightFlank);

    FormationBehavior formation(1.0f, 1.0f);
    pointMan->addBehavior(formation);
    rightFlank->addBehavior(formation);

    world.iterate(0.016f);

    // Abreast case 2: goal = pointMan.position + pointManRight * 40.
    // pointManRight is +X (side vector) while pointMan faces +Z.
    CHECK(near(rightFlank->goal(), pointMan->position() + Math::Vec3(40.0f, 0.0f, 0.0f), 0.01f));
}

void testFormationPentagonSymmetry()
{
    World world;
    Group* squad = new Group(world);
    world.add(*squad);
    Entity::Settings settings = defaultEntitySettings();

    SquadLeaderEntity* leader = makeFormationLeader(world, *squad, settings);
    leader->setSquadFormation(static_cast<int>(SquadFormation::Pentagon));

    SquadMemberEntity* pointMan = new SquadMemberEntity(world, settings);
    pointMan->setSquadId(1);
    pointMan->setPosition(Math::Vec3(1.0f, 0.0f, 1.0f));
    pointMan->setOrientation(Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f));
    squad->add(*pointMan);

    SquadMemberEntity* rightFlank = new SquadMemberEntity(world, settings);
    rightFlank->setSquadId(2);
    rightFlank->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    squad->add(*rightFlank);

    SquadMemberEntity* leftFlank = new SquadMemberEntity(world, settings);
    leftFlank->setSquadId(3);
    leftFlank->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    squad->add(*leftFlank);

    FormationBehavior formation(1.0f, 1.0f);
    pointMan->addBehavior(formation);
    rightFlank->addBehavior(formation);
    leftFlank->addBehavior(formation);

    world.iterate(0.016f);

    // Pentagon's flank *goal* positions only use mLeaderLook/mLeaderRight, not
    // v1/v2, so they are symmetric regardless of the deviation. What v1/v2
    // drive is the flank's facing direction: case 2 (right) faces v1 =
    // leaderLook rotated +45 about Y, case 3 (left) faces v2 = leaderLook
    // rotated -45 about Y. With the leader facing +Z those two facings must
    // be mirror images across the look axis (X negated, Z equal).
    Math::Vec3 forwardRight = glm::mat3_cast(rightFlank->orientation())[2];
    Math::Vec3 forwardLeft = glm::mat3_cast(leftFlank->orientation())[2];
    CHECK(std::fabs(forwardRight.x + forwardLeft.x) < 0.01f);
    CHECK(std::fabs(forwardRight.z - forwardLeft.z) < 0.01f);
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

    if (gFailures)
        std::fprintf(stderr, "%d AI test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
