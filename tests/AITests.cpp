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

bool finiteVec(const Math::vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool near(const Math::vec3& a, const Math::vec3& b, float epsilon = 0.0001f)
{
    return Math::length(a - b) <= epsilon;
}

struct NoVisibility final : WaypointVisibility
{
    bool isVisible(const Math::vec3&, const Math::vec3&) const override
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

// Agent's constructor is private (addComponent<Agent>() only), so each agent is a Component on its own GameObject.
Agent* makeAgent(Scene& scene, const Agent::Settings& settings, const char* name = "agent")
{
    GameObject* object = scene.createGameObject(name);
    Agent* agent = object->addComponent<Agent>();
    agent->applySettings(settings);
    return agent;
}

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

    machine.iterate();
    CHECK(machine.currentState() == counting);

    machine.iterate();
    machine.iterate();
    CHECK(machine.currentState() == counting);
    CHECK(ticks == 2);

    machine.iterate();
    CHECK(machine.currentState() == done);
    CHECK(ticks == 3);
}

void testWaypointNetwork()
{
    WaypointNetwork network;

    Waypoint* a =
        new Waypoint(Math::vec3(0.0f, 0.0f, 0.0f), Math::quat(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    Waypoint* b =
        new Waypoint(Math::vec3(10.0f, 0.0f, 0.0f), Math::quat(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    Waypoint* c =
        new Waypoint(Math::vec3(20.0f, 0.0f, 0.0f), Math::quat(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    Waypoint* d =
        new Waypoint(Math::vec3(30.0f, 0.0f, 0.0f), Math::quat(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    CHECK(network.addWaypoint(a));
    CHECK(network.addWaypoint(b));
    CHECK(network.addWaypoint(c));
    CHECK(network.addWaypoint(d));
    CHECK(!network.addWaypoint(a));

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

    Path selfPath;
    CHECK(network.findPath(b->id(), b->id(), selfPath));
    CHECK(selfPath.size() == 1);

    Path posPath;
    CHECK(network.findPath(Math::vec3(0.0f, 0.0f, 0.0f), Math::vec3(30.0f, 0.0f, 0.0f),
                           WaypointVisibility(), posPath));
    CHECK(posPath.size() == 4);

    Path noPath;
    CHECK(!network.findPath(Math::vec3(0.0f, 0.0f, 0.0f), Math::vec3(30.0f, 0.0f, 0.0f),
                            NoVisibility(), noPath));

    b->edges()[1].open = false;
    Path blockedPath;
    CHECK(!network.findPath(a->id(), d->id(), blockedPath));
}

void testGridPathfinder()
{
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

    std::vector<GridCellCoord> same;
    CHECK(finder.findPath(3, 3, 3, 3, same));
    CHECK(same.size() == 1);

    // No cutting through the seam where two blocked cells meet at a corner.
    //
    //     . . .          (2,1) blocked
    //     . s #          (1,2) blocked
    //     . # g          s = start (1,1), g = goal (2,2)
    //
    {
        GridMap seam(4);
        seam.setBlocked(2, 1);
        seam.setBlocked(1, 2);
        GridPathfinder seamFinder(&seam);

        std::vector<GridCellCoord> through;
        const bool found = seamFinder.findPath(1, 1, 2, 2, through);
        if (found)
        {
            for (usize i = 1; i < through.size(); ++i)
            {
                const int dx = through[i].x - through[i - 1].x;
                const int dy = through[i].y - through[i - 1].y;
                if (dx != 0 && dy != 0)
                {
                    CHECK(!seam.isBlocked(through[i].x, through[i - 1].y));
                    CHECK(!seam.isBlocked(through[i - 1].x, through[i].y));
                }
            }
        }
        // Must go the long way round, more than the two cells a corner cut would take.
        CHECK(!found || through.size() > 2);
    }

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

void testFlocking()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    Agent* a = makeAgent(scene, settings, "a");
    Agent* b = makeAgent(scene, settings, "b");
    Agent* c = makeAgent(scene, settings, "c");
    // Register a/b/c with the Scene (add() is deferred) before per-agent state is set, so the flush's behavior-less update() stays a no-op.
    scene.update(0.0f);

    a->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    b->setPosition(Math::vec3(3.0f, 0.0f, 0.0f));
    c->setPosition(Math::vec3(1.5f, 0.0f, 2.5f));
    a->setGroupId(1);
    b->setGroupId(1);
    c->setGroupId(1);

    // Behaviors are owned per-agent: a shared instance would be double-deleted.
    for (Agent* e : {a, b, c})
    {
        e->addBehavior<SeparationBehavior>(4.0f, 0.2f, 1.0f);
        e->addBehavior<CohesionBehavior>(2.0f);
        e->addBehavior<AlignmentBehavior>(1.0f);
        e->addBehavior<StayWithinSphereBehavior>(Math::vec3(0.0f, 0.0f, 0.0f), 20.0f);
    }

    scene.updateAgents(0.016f);
    CHECK(Math::length(a->desiredMove()) > 0.0f);
    CHECK(Math::length(b->desiredMove()) > 0.0f);
    CHECK(Math::length(c->desiredMove()) > 0.0f);

    for (int i = 0; i < 300; ++i)
        scene.updateAgents(0.016f);

    CHECK(finiteVec(a->position()));
    CHECK(finiteVec(b->position()));
    CHECK(finiteVec(c->position()));
    CHECK(Math::length(a->position() - b->position()) < 12.0f);
    CHECK(Math::length(b->position() - c->position()) < 12.0f);
    CHECK(Math::length(a->position() - Math::vec3(0.0f, 0.0f, 0.0f)) < 25.0f);
}

void testGridAlgorithms()
{
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

void testSquadMovement()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    WaypointNetwork network;
    Waypoint* wpStart =
        new Waypoint(Math::vec3(0.0f, 0.0f, 0.0f), Math::quat(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    Waypoint* wpGoal =
        new Waypoint(Math::vec3(50.0f, 0.0f, 0.0f), Math::quat(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    network.addWaypoint(wpStart);
    network.addWaypoint(wpGoal);
    wpStart->addEdge(NetworkEdge{wpGoal->id()});
    wpGoal->addEdge(NetworkEdge{wpStart->id()});

    PointsOfInterest pois;
    PointOfInterest* target = new PointOfInterest(Math::vec3(50.0f, 0.0f, 0.0f), 5.0f);
    CHECK(pois.add(target));

    Agent* leader = makeAgent(scene, settings, "leader");
    Agent* member = makeAgent(scene, settings, "member");
    scene.update(0.0f);

    leader->setWaypointNetwork(&network);
    leader->setSquadId(0);
    leader->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));

    member->setWaypointNetwork(&network);
    member->setSquadId(1);
    member->setPosition(Math::vec3(5.0f, 0.0f, 0.0f));

    leader->addSquadMember(member);

    member->addBehavior<PathfindBehavior>(PathfindBehavior::Settings{
        0.2f, 50.0f, 0.0f, 25.0f, 0.5f, 0.0f, Math::vec3(0.0f, 1.0f, 0.0f), &network, nullptr});

    member->setStateMachine(buildMemberStateMachine(*member));
    leader->setStateMachine(buildLeaderStateMachine(*leader));

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
    CHECK(member->position().x > 15.0f);
}

void testMemberStateMachine()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    WaypointNetwork network;
    Waypoint* wp =
        new Waypoint(Math::vec3(50.0f, 0.0f, 0.0f), Math::quat(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    network.addWaypoint(wp);

    // Agent::update() is called directly, so no scene.update() flush is needed.
    Agent* member = makeAgent(scene, settings, "member");
    member->setWaypointNetwork(&network);
    member->setSquadId(1);
    member->setPosition(Math::vec3(50.5f, 0.0f, 0.0f));
    member->setGoal(Math::vec3(50.0f, 0.0f, 0.0f));

    StateMachine* machine = buildMemberStateMachine(*member);
    member->setStateMachine(machine);

    CHECK(machine->currentState()->name() == "WaitingForCommand");

    member->setCommand(SquadCommand::AttackTarget);
    member->setNextWaypoint(0);
    member->setPath(Path{wp->id()});
    member->update(0.016f);
    CHECK(machine->currentState()->name() == "MovingToGoal");

    member->update(0.016f);
    CHECK(machine->currentState()->name() == "WaypointReached");

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
    leader->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));

    Agent* member = makeAgent(scene, settings, "member");
    member->setSquadId(1);
    member->setPosition(Math::vec3(5.0f, 0.0f, 0.0f));
    member->setGoal(member->position());
    leader->addSquadMember(member);

    StateMachine* machine = buildLeaderStateMachine(*leader);
    leader->setStateMachine(machine);

    CHECK(machine->currentState()->name() == "AwaitingSquadTaskCompletion");

    leader->update(0.016f);
    CHECK(machine->currentState()->name() == "CommandSquadToPOI");

    leader->setCommand(SquadCommand::StandGround);
    leader->update(0.016f);
    CHECK(machine->currentState()->name() == "StandingGround");
    leader->update(0.016f);
    CHECK(machine->currentState()->name() == "StandingGround");

    leader->setCommand(SquadCommand::PatrolPointsOfInterest);
    leader->update(0.016f);
    CHECK(machine->currentState()->name() == "CommandSquadToPOI");
}

void testSteerLibrary()
{
    Scene scene;
    Agent& e = *makeAgent(scene, defaultAgentSettings());
    e.setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    e.setVelocity(Math::vec3(2.0f, 0.0f, 0.0f));
    e.setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward = +Z

    SteerLibrary steer(e);

    // seek: (target - position) - velocity
    CHECK(near(steer.seek(Math::vec3(10.0f, 0.0f, 0.0f)), Math::vec3(8.0f, 0.0f, 0.0f)));
    // flee: (position - target) - velocity
    CHECK(near(steer.flee(Math::vec3(10.0f, 0.0f, 0.0f)), Math::vec3(-12.0f, 0.0f, 0.0f)));
    // targetSpeed: forward * clip(target - current, -maxForce, +maxForce); maxForce = 2
    CHECK(near(steer.targetSpeed(5.0f), Math::vec3(0.0f, 0.0f, 2.0f)));
    CHECK(near(e.predictFuturePosition(1.0f), Math::vec3(2.0f, 0.0f, 0.0f)));
    CHECK(near(e.globalizePosition(e.localizePosition(Math::vec3(3.0f, 4.0f, 5.0f))),
               Math::vec3(3.0f, 4.0f, 5.0f)));
}

void testPlaneAndRectangleObstacle()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 0.5f;
    Agent& vehicle = *makeAgent(scene, settings);

    // Default plane: XY at the origin, +Z half-space is outside.
    PlaneObstacle plane;

    vehicle.setPosition(Math::vec3(1.0f, 2.0f, 5.0f));
    vehicle.setOrientation(Math::angleAxis(Math::pi<f32>(), Math::vec3(0.0f, 1.0f, 0.0f))); // -Z
    PathIntersection pi;
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 5.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, Math::vec3(1.0f, 2.0f, 0.0f), 0.001f));
    CHECK(near(pi.surfaceNormal, Math::vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(pi.vehicleOutside);

    vehicle.setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f)); // +Z
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    vehicle.setOrientation(Math::angleAxis(Math::radians(90.0f), Math::vec3(0.0f, 1.0f, 0.0f))); // +X
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    // Behind an Outside-only plane, heading into it: the back is not solid.
    vehicle.setPosition(Math::vec3(0.0f, 0.0f, -5.0f));
    vehicle.setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f)); // +Z, toward the plane
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    plane.setSeenFrom(ObstacleSeenFrom::Inside);
    plane.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(near(pi.surfaceNormal, Math::vec3(0.0f, 0.0f, -1.0f), 0.001f));
    CHECK(!pi.vehicleOutside);

    RectangleObstacle rect(2.0f, 2.0f);
    vehicle.setPosition(Math::vec3(0.5f, 0.5f, 5.0f));
    vehicle.setOrientation(Math::angleAxis(Math::pi<f32>(), Math::vec3(0.0f, 1.0f, 0.0f))); // -Z
    rect.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 5.0f) < 0.001f);

    vehicle.setPosition(Math::vec3(5.0f, 0.0f, 5.0f));
    rect.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);

    // Grazing inside the vehicle-radius-grown edge (half-width 1 + radius 0.5).
    vehicle.setPosition(Math::vec3(1.4f, 0.0f, 5.0f));
    rect.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
}

void testBoxObstacle()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 0.0f; // exact face bounds, no radius growth
    Agent& vehicle = *makeAgent(scene, settings);

    BoxObstacle box(2.0f, 2.0f, 2.0f, Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
                    Math::vec3(0.0f, 0.0f, 1.0f), Math::vec3(0.0f));

    vehicle.setPosition(Math::vec3(0.0f, 0.0f, 5.0f));
    vehicle.setOrientation(Math::angleAxis(Math::pi<f32>(), Math::vec3(0.0f, 1.0f, 0.0f))); // -Z
    PathIntersection pi;
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 4.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, Math::vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(near(pi.steerHint, Math::vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(pi.vehicleOutside);
    CHECK(pi.obstacle == &box);

    vehicle.setPosition(Math::vec3(5.0f, 0.0f, 0.0f));
    vehicle.setOrientation(Math::angleAxis(Math::radians(-90.0f), Math::vec3(0.0f, 1.0f, 0.0f)));
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 4.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, Math::vec3(1.0f, 0.0f, 0.0f), 0.001f));
    CHECK(near(pi.steerHint, Math::vec3(1.0f, 0.0f, 0.0f), 0.001f));

    vehicle.setPosition(Math::vec3(0.0f, 5.0f, 0.0f));
    vehicle.setOrientation(Math::angleAxis(Math::radians(90.0f), Math::vec3(1.0f, 0.0f, 0.0f)));
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(std::fabs(pi.distance - 4.0f) < 0.001f);
    CHECK(near(pi.surfacePoint, Math::vec3(0.0f, 1.0f, 0.0f), 0.001f));
    CHECK(near(pi.steerHint, Math::vec3(0.0f, 1.0f, 0.0f), 0.001f));

    vehicle.setPosition(Math::vec3(5.0f, 5.0f, 5.0f));
    vehicle.setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f)); // +Z, away
    box.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);
}

void testSphereObstacleSeenFrom()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 0.5f;
    Agent& vehicle = *makeAgent(scene, settings);

    SphereObstacle pen(2.0f, Math::vec3(0.0f));
    pen.setSeenFrom(ObstacleSeenFrom::Inside);
    vehicle.setPosition(Math::vec3(10.0f, 0.0f, 0.0f));
    vehicle.setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f));
    PathIntersection pi;
    pen.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(pi.distance == 0.0f);
    CHECK(near(pi.steerHint, Math::vec3(-1.0f, 0.0f, 0.0f), 0.001f));
    CHECK(pi.vehicleOutside);

    SphereObstacle shell(3.0f, Math::vec3(0.0f));
    shell.setSeenFrom(ObstacleSeenFrom::Both);
    vehicle.setPosition(Math::vec3(0.0f));
    shell.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(pi.intersect);
    CHECK(!pi.vehicleOutside);
    CHECK(std::fabs(pi.distance - 3.5f) < 0.001f); // radius + vehicle radius
    CHECK(near(pi.surfaceNormal, Math::vec3(0.0f, 0.0f, 1.0f), 0.001f));
    CHECK(near(pi.steerHint, Math::vec3(0.0f, 0.0f, -1.0f), 0.001f));

    SphereObstacle ball(2.0f, Math::vec3(0.0f));
    vehicle.setPosition(Math::vec3(0.0f, 10.0f, 0.0f));
    ball.findIntersectionWithVehiclePath(vehicle, pi);
    CHECK(!pi.intersect);
}

void testCruisingAxisDistribution()
{
    Scene scene;
    Agent& e = *makeAgent(scene, defaultAgentSettings());
    e.setVelocity(Math::vec3(0.0f)); // below desiredSpeed, so signum is +1

    // Per-axis chances form cumulative bands, so Y must still fire; a raw-roll compare never could.
    CruisingBehavior cruising(0.45f, 0.2f, 0.35f, 1.0f, 0.5f, 0.2f);

    std::srand(12345);
    int hits[3] = {0, 0, 0};
    for (int i = 0; i < 2000; ++i)
    {
        e.setDesiredMove(Math::vec3(0.0f));
        cruising.iterate(0.016f, e);
        const Math::vec3 move = e.desiredMove();
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

// The nudge scales with the distance from desired speed, between minRateChange and maxRateChange.
void testCruisingScalesWithSpeedError()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.desiredSpeed = 2.0f;
    settings.maxSpeed = 5.0f;

    const f32 minRate = 0.1f;
    const f32 maxRate = 0.8f;

    Agent& slow = *makeAgent(scene, settings, "slow");
    scene.update(0.0f);
    slow.setVelocity(Math::vec3(0.0f));
    slow.setDesiredMove(Math::vec3(0.0f));
    CruisingBehavior farOff(1.0f, 0.0f, 0.0f, 1.0f, maxRate, minRate);
    farOff.iterate(0.016f, slow);
    const f32 farMagnitude = Math::length(slow.desiredMove());

    Agent& atSpeed = *makeAgent(scene, settings, "atSpeed");
    scene.update(0.0f);
    atSpeed.setVelocity(Math::vec3(2.0f, 0.0f, 0.0f));
    atSpeed.setDesiredMove(Math::vec3(0.0f));
    CruisingBehavior onTarget(1.0f, 0.0f, 0.0f, 1.0f, maxRate, minRate);
    onTarget.iterate(0.016f, atSpeed);
    const f32 onTargetMagnitude = Math::length(atSpeed.desiredMove());

    CHECK(farMagnitude > onTargetMagnitude);
    CHECK(std::abs(onTargetMagnitude - minRate) < 1e-4f);
    CHECK(farMagnitude <= maxRate + 1e-4f);
}

void testGridAStarOptimality()
{
    // Breadth-first is optimal by construction; with unit diagonal cost so are Dijkstra and A* (admissible heuristic): all must agree.
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

    GridMap open(10);
    GridPathfinder openFinder(&open);
    openFinder.settings().algorithm = GridSearchAlgorithm::AStar;
    std::vector<GridCellCoord> diagonal;
    CHECK(openFinder.findPath(0, 0, 7, 3, diagonal));
    CHECK(diagonal.size() == 8);
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

    // Removing B must prune A's transition into it, or iterating walks a freed state.
    machine.removeState(b);
    CHECK(machine.findState("B") == nullptr);
    machine.iterate();
    CHECK(machine.currentState() == a);
}

// Callbacks can reach back into the machine (removeState() deletes the State and transitions aimed at it): iterate() must survive rewriting the list it walks.
void testStateMachineSurvivesCallbackMutation()
{
    // 1. shouldTransition() removes a state and answers false.
    {
        StateMachine machine;
        State* a = new State("A");
        State* doomed = new State("Doomed");
        State* other = new State("Other");
        machine.addState(a);
        machine.addState(doomed);
        machine.addState(other);

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
        CHECK(machine.currentState() == a || machine.currentState() == other);
    }

    // 2. exit() removes the state being left: installing the target afterwards would resurrect the emptied machine (reading freed `state`).
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
        CHECK(machine.currentState() == nullptr);
        machine.iterate();
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
    PointOfInterest* first = new PointOfInterest(Math::vec3(0.0f, 0.0f, 0.0f), 1.0f);
    PointOfInterest* second = new PointOfInterest(Math::vec3(10.0f, 0.0f, 0.0f), 1.0f);
    PointOfInterest* third = new PointOfInterest(Math::vec3(0.0f, 0.0f, 20.0f), 1.0f);
    CHECK(pois.add(first));
    CHECK(pois.add(second));
    CHECK(pois.add(third));

    CHECK(pois.find(first->id()) == first);
    CHECK(pois.findNearest(Math::vec3(9.0f, 0.0f, 1.0f)) == second);
    CHECK(pois.findNearest(Math::vec3(0.0f, 0.0f, 19.0f)) == third);

    for (int i = 0; i < 50; ++i)
    {
        const PointOfInterest* pick = pois.selectRandom(first->id());
        CHECK(pick != nullptr);
        CHECK(pick->id() != first->id());
    }

    // Taken before the remove: it frees the point, so reading the id afterwards is a use-after-free.
    const u32 secondId = second->id();
    CHECK(pois.remove(secondId));
    CHECK(pois.find(secondId) == nullptr);
}

void testPursuitEvasion()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    Agent& hunter = *makeAgent(scene, settings, "hunter");
    hunter.setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    hunter.setVelocity(Math::vec3(0.0f, 0.0f, 2.0f));
    hunter.setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    Agent& quarry = *makeAgent(scene, settings, "quarry");
    quarry.setPosition(Math::vec3(0.0f, 0.0f, 10.0f));

    SteerLibrary steer(hunter);

    CHECK(near(steer.pursuit(quarry), steer.seek(quarry.position())));

    // Ahead-parallel quarry: intercept (10/2 * 4 = 20s) is capped at maxPredictionTime.
    quarry.setVelocity(Math::vec3(0.0f, 0.0f, 3.0f));
    quarry.setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f));
    CHECK(near(steer.pursuit(quarry, 1.0f),
               steer.seek(quarry.position() + quarry.velocity() * 1.0f)));

    // A stationary menace is predicted at the cap, not divided by zero.
    Agent& menace = *makeAgent(scene, settings, "menace");
    menace.setPosition(Math::vec3(5.0f, 0.0f, 0.0f));
    CHECK(near(steer.evasion(menace, 2.0f), steer.flee(menace.position())));

    // Slow distant menace: intercept (5s) exceeds the cap (2s), so the flee target is two seconds of travel.
    menace.setVelocity(Math::vec3(0.0f, 0.0f, 1.0f));
    CHECK(near(steer.evasion(menace, 2.0f),
               steer.flee(menace.position() + menace.velocity() * 2.0f)));
}

void testDirectionalPredicates()
{
    Scene scene;
    Agent& e = *makeAgent(scene, defaultAgentSettings());
    e.setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    e.setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    SteerLibrary steer(e);
    CHECK(steer.isAhead(Math::vec3(0.0f, 0.0f, 5.0f)));
    CHECK(!steer.isBehind(Math::vec3(0.0f, 0.0f, 5.0f)));
    CHECK(steer.isBehind(Math::vec3(0.0f, 0.0f, -5.0f)));
    CHECK(!steer.isAhead(Math::vec3(0.0f, 0.0f, -5.0f)));
    CHECK(steer.isAside(Math::vec3(5.0f, 0.0f, 0.0f)));
    CHECK(!steer.isAhead(Math::vec3(0.0f, 0.0f, 0.0f))); // degenerate offset

    // Local frame convention: right = +X, up = +Y, forward = +Z.
    CHECK(near(e.localizeDirection(e.forward()), Math::vec3(0.0f, 0.0f, 1.0f)));
    CHECK(near(e.localizeDirection(e.side()), Math::vec3(1.0f, 0.0f, 0.0f)));
    CHECK(near(e.localizeDirection(e.up()), Math::vec3(0.0f, 1.0f, 0.0f)));
    CHECK(near(e.globalizeDirection(Math::vec3(0.0f, 0.0f, 1.0f)), e.forward()));

    // Right-handed yaw: +90 degrees about +Y swings forward from +Z to +X.
    e.setOrientation(Math::angleAxis(Math::radians(90.0f), Math::vec3(0.0f, 1.0f, 0.0f)));
    CHECK(near(e.forward(), Math::vec3(1.0f, 0.0f, 0.0f), 0.001f));
    CHECK(near(e.side(), Math::vec3(0.0f, 0.0f, -1.0f), 0.001f));

    e.setVelocity(Math::vec3(0.0f, 0.0f, -3.0f));
    e.alignWithVelocity();
    CHECK(near(e.forward(), Math::vec3(0.0f, 0.0f, -1.0f), 0.001f));
    CHECK(near(e.up(), Math::vec3(0.0f, 1.0f, 0.0f), 0.001f));
    CHECK(near(Math::cross(e.up(), e.forward()), e.side(), 0.001f));
}

void testBoidNeighborhoodAndSeparation()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    Agent& self = *makeAgent(scene, settings, "self");
    self.setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    self.setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    Agent& ahead = *makeAgent(scene, settings, "ahead");
    ahead.setPosition(Math::vec3(0.0f, 0.0f, 5.0f));
    Agent& behind = *makeAgent(scene, settings, "behind");
    behind.setPosition(Math::vec3(0.0f, 0.0f, -5.0f));
    Agent& veryClose = *makeAgent(scene, settings, "veryClose");
    veryClose.setPosition(Math::vec3(0.0f, 0.0f, -0.5f));
    Agent& far = *makeAgent(scene, settings, "far");
    far.setPosition(Math::vec3(0.0f, 0.0f, 50.0f));

    SteerLibrary steer(self);
    CHECK(steer.inBoidNeighborhood(ahead, 1.0f, 10.0f, 0.0f));
    CHECK(!steer.inBoidNeighborhood(behind, 1.0f, 10.0f, 0.0f)); // outside cone
    CHECK(steer.inBoidNeighborhood(veryClose, 1.0f, 10.0f, 0.0f)); // inside min sphere
    CHECK(!steer.inBoidNeighborhood(far, 1.0f, 10.0f, 0.0f)); // outside max sphere
    CHECK(!steer.inBoidNeighborhood(self, 1.0f, 10.0f, 0.0f)); // never itself

    std::vector<EntityDist> flock;
    flock.push_back(EntityDist{5.0f, &ahead});
    CHECK(near(steer.separation(10.0f, -1.0f, flock), Math::vec3(0.0f, 0.0f, -1.0f)));
    CHECK(near(steer.cohesion(10.0f, -1.0f, flock), Math::vec3(0.0f, 0.0f, 1.0f)));
}

void testTargetSpeedClamp()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings(); // maxVelocityChange = 2
    Agent& e = *makeAgent(scene, settings);
    e.setVelocity(Math::vec3(2.0f, 0.0f, 0.0f)); // speed 2
    e.setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward +Z

    SteerLibrary steer(e);
    CHECK(near(steer.targetSpeed(2.0f), Math::vec3(0.0f)));
    CHECK(near(steer.targetSpeed(-5.0f), Math::vec3(0.0f, 0.0f, -2.0f)));
}

void testSeekFlee()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    Agent* chaser = makeAgent(scene, settings, "chaser");
    scene.update(0.0f);
    chaser->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    const Math::vec3 target(100.0f, 0.0f, 0.0f);
    chaser->addBehavior<SeekBehavior>(target);

    for (int i = 0; i < 600; ++i)
        scene.updateAgents(0.016f);

    CHECK(finiteVec(chaser->position()));
    CHECK(chaser->position().x > 30.0f);

    Agent* runner = makeAgent(scene, settings, "runner");
    scene.update(0.0f);
    runner->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
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
    e->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    e->addBehavior<WanderBehavior>();

    for (int i = 0; i < 300; ++i)
        scene.updateAgents(0.016f);

    CHECK(finiteVec(e->position()));
    CHECK(Math::length(e->position()) > 0.001f);
    CHECK(Math::length(e->position()) < 50.0f);
}

void testObstacleAvoidance()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 0.5f;

    SphereObstacle sphere(3.0f, Math::vec3(8.0f, 0.0f, 2.0f));
    ObstacleGroup obstacles;
    obstacles.push_back(&sphere);

    Agent* vehicle = makeAgent(scene, settings, "vehicle");
    scene.update(0.0f);
    vehicle->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    vehicle->setVelocity(Math::vec3(5.0f, 0.0f, 0.0f)); // moving +X
    vehicle->setOrientation(Math::angleAxis(Math::radians(90.0f), Math::vec3(0.0f, 1.0f, 0.0f)));
    ObstacleAvoidanceBehavior* avoidance = vehicle->addBehavior<ObstacleAvoidanceBehavior>(2.0f);
    avoidance->setObstacles(obstacles);

    scene.updateAgents(0.016f);
    CHECK(finiteVec(vehicle->desiredMove()));
    CHECK(vehicle->desiredMove().z < 0.0f);

    for (int i = 0; i < 600; ++i)
        scene.updateAgents(0.016f);

    CHECK(finiteVec(vehicle->position()));
    CHECK(vehicle->position().z < 0.0f);
    CHECK(Math::length(vehicle->position() - sphere.center) > sphere.radius);
}

// Regression: Agent::update() must call alignWithVelocity() itself, or velocity-steered agents never face their motion and "ahead" tests use the spawn facing. Orientation is never set by hand.
void testObstacleAvoidanceTracksVelocityDirection()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 0.5f;

    SphereObstacle sphere(3.0f, Math::vec3(8.0f, 0.0f, 2.0f));
    ObstacleGroup obstacles;
    obstacles.push_back(&sphere);

    Agent* vehicle = makeAgent(scene, settings, "vehicle");
    scene.update(0.0f);
    vehicle->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    vehicle->setVelocity(Math::vec3(5.0f, 0.0f, 0.0f)); // moving +X, orientation left at spawn
    ObstacleAvoidanceBehavior* avoidance = vehicle->addBehavior<ObstacleAvoidanceBehavior>(2.0f);
    avoidance->setObstacles(obstacles);

    // First tick: forward is still the spawn facing, so the sphere is not ahead; this only lets alignWithVelocity() turn the agent.
    scene.updateAgents(0.016f);

    scene.updateAgents(0.016f);
    CHECK(finiteVec(vehicle->desiredMove()));
    CHECK(vehicle->desiredMove().z < 0.0f);
}

// Regression for the inverted sign in Steering.cpp:306.

void testNearestApproach()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 1.0f;

    // Head-on closing: the nearest approach must be in the future (time > 0).
    Agent& us = *makeAgent(scene, settings, "us");
    us.setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    us.setVelocity(Math::vec3(1.0f, 0.0f, 0.0f));

    Agent& oncoming = *makeAgent(scene, settings, "oncoming");
    oncoming.setPosition(Math::vec3(10.0f, 0.0f, 0.0f));
    oncoming.setVelocity(Math::vec3(-1.0f, 0.0f, 0.0f));

    SteerLibrary steer(us);
    const float tHeadOn = steer.predictNearestApproachTime(oncoming);
    CHECK(std::isfinite(tHeadOn));
    CHECK(tHeadOn > 0.0f);
    CHECK(std::fabs(tHeadOn - 5.0f) < 0.01f);
    const float headOnDist = steer.computeNearestApproachPositions(oncoming, tHeadOn);
    CHECK(headOnDist < 0.1f);

    // Receding: nearest approach in the past (time < 0), so avoidNeighbors must ignore it.
    Agent& receding = *makeAgent(scene, settings, "receding");
    receding.setPosition(Math::vec3(10.0f, 0.0f, 0.0f));
    receding.setVelocity(Math::vec3(1.0f, 0.0f, 0.0f));
    Agent& fast = *makeAgent(scene, settings, "fast");
    fast.setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    fast.setVelocity(Math::vec3(-1.0f, 0.0f, 0.0f));
    SteerLibrary steerFast(fast);
    const float tReceding = steerFast.predictNearestApproachTime(receding);
    CHECK(tReceding < 0.0f);

    std::vector<EntityDist> ahead;
    ahead.push_back(EntityDist{Math::length(oncoming.position() - us.position()), &oncoming});
    Math::vec3 steerAway = steer.avoidNeighbors(8.0f, ahead);
    CHECK(finiteVec(steerAway));
    CHECK(Math::length(steerAway) > 0.0f);

    std::vector<EntityDist> awayFrom;
    awayFrom.push_back(EntityDist{Math::length(receding.position() - fast.position()), &receding});
    Math::vec3 steerNone = steerFast.avoidNeighbors(8.0f, awayFrom);
    CHECK(near(steerNone, Math::vec3(0.0f)));
}

void testPathfindLineOfSight()
{
    struct ToggleVisibility final : WaypointVisibility
    {
        bool visible = false;
        bool isVisible(const Math::vec3&, const Math::vec3&) const override
        {
            return visible;
        }
    };

    Scene scene;

    WaypointNetwork network;
    Waypoint* wpDetour =
        new Waypoint(Math::vec3(5.0f, 0.0f, 30.0f), Math::quat(1.0f, 0.0f, 0.0f, 0.0f), 2.0f);
    network.addWaypoint(wpDetour);

    ToggleVisibility visibility;
    visibility.visible = false; // goal not visible yet - keep following the seeded route

    Agent::Settings settings = defaultAgentSettings();
    Agent* member = makeAgent(scene, settings, "member");
    scene.update(0.0f);
    member->setWaypointNetwork(&network);
    member->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    member->setGoal(Math::vec3(20.0f, 0.0f, 0.0f));
    member->setGoalRadius(1.0f);
    // Seeded "mid-route, no LOS" state; this test is only about the LOS short-circuit in PathfindBehavior::iterate.
    member->setNextWaypoint(wpDetour->id());

    member->addBehavior<PathfindBehavior>(PathfindBehavior::Settings{
        0.3f, 1.0f, 0.0f, 25.0f, 0.05f, 0.0f, Math::vec3(0.0f, 1.0f, 0.0f), &network,
        &visibility});

    for (int i = 0; i < 30; ++i)
        scene.updateAgents(0.016f);
    CHECK(finiteVec(member->position()));
    CHECK(member->position().z > 1.0f);

    visibility.visible = true;
    for (int i = 0; i < 10; ++i)
        scene.updateAgents(0.016f);
    CHECK(member->losStatus());
    CHECK(member->nextWaypoint() == 0);
    CHECK(!member->hasValidPath());

    for (int i = 0; i < 400; ++i)
        scene.updateAgents(0.016f);

    CHECK(finiteVec(member->position()));
    CHECK(member->position().x > 10.0f);
    CHECK(std::fabs(member->position().z) < 5.0f);
}

// Regression: addSquadMember() must assign slot ids itself (1, 2, 3 in join order), or every member stays at -1 and FormationBehavior never forms a shape.
void testAddSquadMemberAssignsSlotIds()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    Agent* leader = makeAgent(scene, settings, "leader");
    leader->setSquadId(0);
    Agent* first = makeAgent(scene, settings, "first");
    Agent* second = makeAgent(scene, settings, "second");
    Agent* third = makeAgent(scene, settings, "third");
    scene.update(0.0f);

    leader->addSquadMember(first);
    leader->addSquadMember(second);
    leader->addSquadMember(third);

    CHECK(first->squadId() == 1);
    CHECK(second->squadId() == 2);
    CHECK(third->squadId() == 3);

    // A caller-assigned slot is kept: 99 is deliberately not the join-order slot (4).
    Agent* preAssigned = makeAgent(scene, settings, "preAssigned");
    preAssigned->setSquadId(99);
    scene.update(0.0f);
    leader->addSquadMember(preAssigned);
    CHECK(preAssigned->squadId() == 99);
}

// Regression: removeSquadMember() must give up the slot and addSquadMember() refill the lowest free one, not size()-based (it would collide with a survivor's id).
void testSquadMemberSlotIsReusedAfterRemoval()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    Agent* leader = makeAgent(scene, settings, "leader");
    leader->setSquadId(0);
    Agent* first = makeAgent(scene, settings, "first");
    Agent* second = makeAgent(scene, settings, "second");
    Agent* third = makeAgent(scene, settings, "third");
    scene.update(0.0f);

    leader->addSquadMember(first);
    leader->addSquadMember(second);
    leader->addSquadMember(third);
    CHECK(first->squadId() == 1);
    CHECK(second->squadId() == 2);
    CHECK(third->squadId() == 3);

    leader->removeSquadMember(second);
    CHECK(second->squadLeader() == nullptr);
    CHECK(second->squadId() == -1);

    Agent* fourth = makeAgent(scene, settings, "fourth");
    scene.update(0.0f);
    leader->addSquadMember(fourth);

    // The vacated slot (2), not one past the head count, which would collide with `third` (id 3).
    CHECK(fourth->squadId() == 2);
    CHECK(third->squadId() == 3);
}

// Pentagon's mirrored flank direction deviates deliberately from the reference (see FormationBehavior.cpp); checked for its symmetry.

Agent* makeFormationLeader(Scene& scene, const Agent::Settings& settings)
{
    Agent* leader = makeAgent(scene, settings, "leader");
    leader->setSquadId(0);
    leader->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    leader->setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f)); // forward = +Z
    return leader;
}

// Two squads in one scene: the leader must come from the squad link, not whichever squadId 0 came last in the list.
void testFormationFollowsItsOwnLeader()
{
    Scene scene;
    Agent::Settings settings = defaultAgentSettings();

    Agent* leaderA = makeAgent(scene, settings, "leaderA");
    Agent* memberA = makeAgent(scene, settings, "memberA");
    Agent* leaderB = makeAgent(scene, settings, "leaderB");
    Agent* memberB = makeAgent(scene, settings, "memberB");
    scene.update(0.0f);

    leaderA->setSquadId(0);
    leaderA->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    leaderA->setSquadFormation(static_cast<int>(SquadFormation::Abreast));
    memberA->setSquadId(2);
    memberA->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    leaderA->addSquadMember(memberA);

    leaderB->setSquadId(0);
    leaderB->setPosition(Math::vec3(100.0f, 0.0f, 0.0f));
    leaderB->setSquadFormation(static_cast<int>(SquadFormation::Abreast));
    memberB->setSquadId(2);
    memberB->setPosition(Math::vec3(100.0f, 0.0f, 0.0f));
    leaderB->addSquadMember(memberB);

    CHECK(memberA->squadLeader() == leaderA);
    CHECK(memberB->squadLeader() == leaderB);

    memberA->addBehavior<FormationBehavior>(1.0f, 1.0f);
    scene.updateAgents(0.016f);

    // A's member must head toward A's leader, not B's a hundred metres up +X (a scan found B).
    CHECK(finiteVec(memberA->desiredMove()));
    CHECK(memberA->desiredMove().x < 1.0f);
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
    pointMan->setPosition(Math::vec3(0.0f, 0.0f, 5.0f));
    pointMan->setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f));
    pointMan->setGoal(Math::vec3(0.0f, 0.0f, 20.0f));

    rightFlank->setSquadId(2);
    rightFlank->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));

    leader->addSquadMember(pointMan);
    leader->addSquadMember(rightFlank);

    // Each member owns its own FormationBehavior; the leader/point-man cache is not shared.
    pointMan->addBehavior<FormationBehavior>(1.0f, 1.0f);
    rightFlank->addBehavior<FormationBehavior>(1.0f, 1.0f);

    scene.updateAgents(0.016f);

    // Abreast case 2: goal = pointMan.position + pointManRight * 40.
    // pointManRight is +X (side vector) while pointMan faces +Z.
    CHECK(near(rightFlank->goal(), pointMan->position() + Math::vec3(40.0f, 0.0f, 0.0f), 0.01f));
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
    pointMan->setPosition(Math::vec3(1.0f, 0.0f, 1.0f));
    pointMan->setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f));

    rightFlank->setSquadId(2);
    rightFlank->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));

    leftFlank->setSquadId(3);
    leftFlank->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));

    leader->addSquadMember(pointMan);
    leader->addSquadMember(rightFlank);
    leader->addSquadMember(leftFlank);

    pointMan->addBehavior<FormationBehavior>(1.0f, 1.0f);
    rightFlank->addBehavior<FormationBehavior>(1.0f, 1.0f);
    leftFlank->addBehavior<FormationBehavior>(1.0f, 1.0f);

    scene.updateAgents(0.016f);

    // Pentagon flank goals are symmetric regardless of v1/v2; v1/v2 drive facing: right faces leaderLook rotated +45 about Y, left -45, so the facings must mirror (X negated, Z equal).
    Math::vec3 forwardRight = Math::mat3_cast(rightFlank->orientation())[2];
    Math::vec3 forwardLeft = Math::mat3_cast(leftFlank->orientation())[2];
    CHECK(std::fabs(forwardRight.x + forwardLeft.x) < 0.01f);
    CHECK(std::fabs(forwardRight.z - forwardLeft.z) < 0.01f);
}

// Destroying GameObjects in shuffled order must take each Agent's Scene registration along; Scene::mAgents reallocates, and ASan turns a dangling pointer into a failure.
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

    scene.updateAgents(0.016f);
}

// groupId 0 means no group: it sees no group members; enemy masks are independent of groupId and cross groups.
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
    CHECK(lone->groupId() == 0);

    a1->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    a2->setPosition(Math::vec3(1.0f, 0.0f, 0.0f));
    b1->setPosition(Math::vec3(2.0f, 0.0f, 0.0f));
    b2->setPosition(Math::vec3(3.0f, 0.0f, 0.0f));
    lone->setPosition(Math::vec3(0.5f, 0.0f, 0.0f));

    scene.updateAgents(0.016f);

    CHECK(lone->visibleGroupMembers().empty());

    CHECK(a1->visibleGroupMembers().size() == 1);
    CHECK(a1->visibleGroupMembers()[0].entity == a2);
    CHECK(b1->visibleGroupMembers().size() == 1);
    CHECK(b1->visibleGroupMembers()[0].entity == b2);

    // Enemy masks cross groups (a1's enemyMask flags type 2); sorted by distance.
    CHECK(a1->visibleEnemies().size() == 2);
    CHECK(a1->visibleEnemies()[0].entity == b1);
    CHECK(a1->visibleEnemies()[1].entity == b2);
}

// The 180 degree turn pushOwnerPose()/pullAgentPose() apply reconciles Agent forward +Z with GameObject::forward() -Z.
void testAgentPoseSyncFlipsForward()
{
    Scene scene;
    Agent* agent = makeAgent(scene, defaultAgentSettings(), "agent");
    scene.update(0.0f); // register before touching velocity/orientation

    agent->setVelocity(Math::vec3(1.0f, 0.0f, 0.0f));
    agent->alignWithVelocity();
    CHECK(near(agent->forward(), Math::vec3(1.0f, 0.0f, 0.0f)));

    scene.update(0.016f);

    CHECK(near(agent->owner()->forward(), Math::vec3(1.0f, 0.0f, 0.0f), 0.0001f));
}

// Behaviors are owned: built, deleted on removal, the rest freed exactly once (a double free or leak fails under ASan); no `new` appears.
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

    CHECK(a->removeBehavior(BehaviorType::Separation));
    CHECK(a->behaviorCount() == 1);
    CHECK(a->behaviorAt(0) == cohesion);
    CHECK(a->behavior(BehaviorType::Separation) == nullptr);
    CHECK(!a->removeBehavior(BehaviorType::Separation));

    Behavior* wander = a->addBehavior(BehaviorType::Wander);
    CHECK(wander != nullptr);
    CHECK(wander->type() == BehaviorType::Wander);
    CHECK(a->behaviorCount() == 2);

    // A behavior belongs to one agent: adoptBehavior() refuses an owned one and destroys what it refuses.
    Agent* b = makeAgent(scene, defaultAgentSettings(), "b");
    scene.update(0.0f);
    CHECK(!b->adoptBehavior(cohesion));
    CHECK(b->behaviorCount() == 0);
    CHECK(a->behaviorCount() == 2);
    CHECK(cohesion->owner() == a);

    CHECK(b->adoptBehavior(new AlignmentBehavior(1.0f)));
    CHECK(b->behaviorCount() == 1);
    CHECK(!b->adoptBehavior(nullptr));

    CHECK(scene.destroy(a->owner()));
    CHECK(scene.destroy(b->owner()));
    scene.update(0.0f);
    CHECK(scene.agentCount() == 0);
}

// A failed search leaves the path empty, which triggers the search again: without a rate limit it is a full A* per agent per frame.
void testPathfindRepathIsRateLimited()
{
    Scene scene;

    WaypointNetwork network;
    Waypoint* wpStart =
        new Waypoint(Math::vec3(0.0f, 0.0f, 0.0f), Math::quat(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    Waypoint* wpIsland =
        new Waypoint(Math::vec3(80.0f, 0.0f, 0.0f), Math::quat(1.0f, 0.0f, 0.0f, 0.0f), 3.0f);
    network.addWaypoint(wpStart);
    network.addWaypoint(wpIsland);

    Agent* agent = makeAgent(scene, defaultAgentSettings(), "walker");
    scene.update(0.0f);
    agent->setWaypointNetwork(&network);
    agent->setSquadId(1);
    agent->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    agent->setGoal(Math::vec3(80.0f, 0.0f, 0.0f));
    agent->setGoalRadius(2.0f);

    NoVisibility blocked; // never any line of sight, so it must route
    PathfindBehavior* pathfind = agent->addBehavior<PathfindBehavior>(PathfindBehavior::Settings{
        0.2f, 2.0f, 0.0f, 25.0f, 0.05f, 1.0f, Math::vec3(0.0f, 1.0f, 0.0f), &network, &blocked});
    CHECK(pathfind != nullptr);
    if (!pathfind)
        return;
    CHECK(std::abs(pathfind->settings().repathInterval - 1.0f) < 1e-5f);

    for (u32 i = 0; i < 30; ++i)
        scene.updateAgents(1.0f / 60.0f);
    CHECK(agent->nextWaypoint() == 0);

    // Well past the interval: the search may run and still finds nothing (no edges).
    for (u32 i = 0; i < 300; ++i)
        scene.updateAgents(1.0f / 60.0f);
    CHECK(finiteVec(agent->position()));
    CHECK(agent->path().empty());
}

// Destroying a squad member removes it from its leader's list; a dangling pointer was walked by the next order.
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

    leader->setCommand(AI::SquadCommand::RallyToLeaderPosition);
    CHECK(second->command() == AI::SquadCommand::RallyToLeaderPosition);

    CHECK(scene.destroy(leader->owner()));
    scene.update(0.0f);
    CHECK(second->squadLeader() == nullptr);
}

// A leader ordered to a random waypoint with no network must return before dereferencing one.
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
    CHECK(member->goal() == Math::vec3(0.0f));
}

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

    // SteerBehavior is outside the registry (Steering.h): no name, and Count creates nothing.
    BehaviorType outType = BehaviorType::Count;
    CHECK(!BehaviorFactory::fromName("Steer", outType));
    CHECK(BehaviorFactory::create(BehaviorType::Count) == nullptr);
}

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
                    Math::clamp(info.minValue + 0.5f, info.minValue, info.maxValue);
                behavior->setParamFloat(i, testValue);
                CHECK(std::fabs(behavior->paramFloat(i) - testValue) < 0.0001f);
            }
            else if (info.kind == BehaviorParam::Kind::Vec3)
            {
                const Math::vec3 testValue(1.5f, -2.5f, 3.5f);
                behavior->setParamVec3(i, testValue);
                CHECK(near(behavior->paramVec3(i), testValue));
            }
        }

        delete behavior;
    }
}

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
    CHECK(scene.obstacleCount() == 1);
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

// N obstacles created WITHOUT reserve() reallocate, destroyed in shuffled order; the two arrays must stay paired through every swap-and-pop.
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

    scene.debugDrawObstacles();
}

// Swapping the shape reconstructs the owned AI::Obstacle (new address) and the Scene's ObstacleGroup entry follows it (refreshObstacle()).
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

// No setObstacles(): found through Agent::scene()->obstacleGroup(); the same result as testObstacleAvoidance() proves the fallback wiring.
void testObstacleAvoidanceReadsSceneGroup()
{
    Scene scene;

    GameObject* wall = scene.createGameObject("wall");
    wall->setPosition(Math::vec3(8.0f, 0.0f, 2.0f));
    Radion::Obstacle* wallObstacle = wall->addComponent<Radion::Obstacle>();
    wallObstacle->setSphere(3.0f);
    scene.update(0.0f);

    Agent::Settings settings = defaultAgentSettings();
    settings.radius = 0.5f;
    Agent* vehicle = makeAgent(scene, settings, "vehicle");
    scene.update(0.0f);
    vehicle->setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    vehicle->setVelocity(Math::vec3(5.0f, 0.0f, 0.0f)); // moving +X
    vehicle->setOrientation(Math::angleAxis(Math::radians(90.0f), Math::vec3(0.0f, 1.0f, 0.0f)));
    vehicle->addBehavior<ObstacleAvoidanceBehavior>(2.0f);

    scene.updateAgents(0.016f);
    CHECK(finiteVec(vehicle->desiredMove()));
    CHECK(vehicle->desiredMove().z < 0.0f);

    for (int i = 0; i < 600; ++i)
        scene.updateAgents(0.016f);

    CHECK(finiteVec(vehicle->position()));
    CHECK(vehicle->position().z < 0.0f);
    CHECK(Math::length(vehicle->position() - Math::vec3(8.0f, 0.0f, 2.0f)) > 3.0f);
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
    testCruisingScalesWithSpeedError();
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
    testObstacleAvoidanceTracksVelocityDirection();
    testNearestApproach();
    testPathfindLineOfSight();
    testAddSquadMemberAssignsSlotIds();
    testSquadMemberSlotIsReusedAfterRemoval();
    testFormationFollowsItsOwnLeader();
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
