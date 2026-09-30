#include "PCH.h"

#include "api/ApiServer.h"
#include "api/CommandRegistry.h"
#include "api/MainThreadQueue.h"

#include <httplib/httplib.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

using namespace Radion::BlenderApi;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "BlenderApiTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

CommandRegistry makeRegistry()
{
    CommandRegistry registry;

    CommandDef add;
    add.name = "add";
    add.description = "adds two numbers";
    add.readOnly = true;
    add.handler = [](const CommandArgs& args)
    {
        CommandResult result;
        result.data["sum"] = args.requireNumber("a") + args.requireNumber("b");
        return result;
    };
    registry.add(add);

    CommandDef fail;
    fail.name = "fail";
    fail.handler = [](const CommandArgs&) -> CommandResult
    { throw CommandError(CommandStatus::Failed, "nothing selected"); };
    registry.add(fail);

    CommandDef crash;
    crash.name = "crash";
    crash.handler = [](const CommandArgs&) -> CommandResult { throw std::runtime_error("boom"); };
    registry.add(crash);

    CommandDef picture;
    picture.name = "picture";
    picture.handler = [](const CommandArgs&)
    {
        CommandResult result;
        result.image = CommandImage{"image/png", "AAAA"};
        return result;
    };
    registry.add(picture);

    CommandDef thread;
    thread.name = "thread_id";
    thread.handler = [](const CommandArgs&)
    {
        CommandResult result;
        result.data["id"] = std::hash<std::thread::id>()(std::this_thread::get_id());
        return result;
    };
    registry.add(thread);

    return registry;
}

void testArgsValidation()
{
    const Json good = {{"n", 3}, {"f", 1.5}, {"s", "hi"}, {"b", true},
                       {"v", {1, 2, 3}}, {"i", {0, 4}}, {"mode", "face"}};
    const CommandArgs args(good);
    CHECK(args.integer("n", 0) == 3);
    CHECK(args.number("f", 0) == 1.5);
    CHECK(args.requireString("s") == "hi");
    CHECK(args.boolean("b", false));
    CHECK(args.numbers("v", 3, {}).size() == 3);
    CHECK(args.indices("i", 10).size() == 2);
    CHECK(args.choice("mode", {"vertex", "face"}, "vertex") == "face");
    // Absent arguments fall back; they are not errors.
    CHECK(args.integer("missing", 7) == 7);
    CHECK(args.string("missing", "x") == "x");

    auto rejects = [&](auto&& call)
    {
        try
        {
            call();
        }
        catch (const CommandError& error)
        {
            return error.status() == CommandStatus::InvalidParams;
        }
        return false;
    };

    CHECK(rejects([&] { args.requireString("missing"); }));
    CHECK(rejects([&] { args.requireString("n"); }));
    CHECK(rejects([&] { args.integer("f", 0); }));
    CHECK(rejects([&] { args.number("f", 0, 0.0, 1.0); }));
    CHECK(rejects([&] { args.numbers("v", 2, {}); }));
    CHECK(rejects([&] { args.indices("i", 1); }));
    CHECK(rejects([&] { args.choice("mode", {"vertex"}, "vertex"); }));
    CHECK(rejects([&] { CommandArgs(Json::array()); }));

    const Json negative = {{"i", {-1}}};
    CHECK(rejects([&] { CommandArgs(negative).indices("i", 10); }));
}

void testRegistryCall()
{
    const CommandRegistry registry = makeRegistry();

    CommandOutcome ok = registry.call("add", {{"a", 2}, {"b", 3}});
    CHECK(ok.status == CommandStatus::Ok);
    CHECK(ok.result.data["sum"] == 5);

    CHECK(registry.call("nope", Json::object()).status == CommandStatus::UnknownCommand);
    CHECK(registry.call("add", Json::object()).status == CommandStatus::InvalidParams);
    CHECK(registry.call("add", {{"a", "x"}, {"b", 1}}).status == CommandStatus::InvalidParams);

    const CommandOutcome failed = registry.call("fail", Json::object());
    CHECK(failed.status == CommandStatus::Failed);
    CHECK(failed.message == "nothing selected");

    CHECK(registry.call("crash", Json::object()).status == CommandStatus::Failed);

    // A second add() of the same name replaces, it does not duplicate.
    CommandRegistry replaced = makeRegistry();
    const size_t count = replaced.commands().size();
    CommandDef again;
    again.name = "add";
    again.handler = [](const CommandArgs&) { return CommandResult(); };
    replaced.add(again);
    CHECK(replaced.commands().size() == count);

    CHECK(registry.describe().size() == count);
}

void testQueueRunsOnDrainingThread()
{
    MainThreadQueue queue;
    std::atomic<bool> done{false};
    std::thread::id ranOn;

    std::thread client(
        [&]
        {
            const CommandOutcome outcome = queue.run(
                [&]
                {
                    ranOn = std::this_thread::get_id();
                    return CommandOutcome();
                },
                std::chrono::seconds(5));
            CHECK(outcome.status == CommandStatus::Ok);
            done = true;
        });

    while (!done)
        queue.drain();
    client.join();
    CHECK(ranOn == std::this_thread::get_id());
}

void testQueueTimeoutDropsTask()
{
    MainThreadQueue queue;
    bool ran = false;
    const CommandOutcome outcome = queue.run(
        [&]
        {
            ran = true;
            return CommandOutcome();
        },
        std::chrono::milliseconds(20));
    CHECK(outcome.status == CommandStatus::Timeout);

    // The request was reported as timed out, so it must never run afterwards.
    queue.drain();
    CHECK(!ran);
}

void testQueueClose()
{
    MainThreadQueue queue;
    CommandStatus waiting = CommandStatus::Ok;
    std::thread client([&]
                       { waiting = queue.run([] { return CommandOutcome(); },
                                             std::chrono::seconds(10)).status; });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    queue.close();
    client.join();
    CHECK(waiting == CommandStatus::Unavailable);
    CHECK(queue.run([] { return CommandOutcome(); }, std::chrono::seconds(1)).status ==
          CommandStatus::Unavailable);

    queue.open();
    std::atomic<bool> done{false};
    std::thread again([&]
                      { CHECK(queue.run([] { return CommandOutcome(); },
                                        std::chrono::seconds(5)).status == CommandStatus::Ok);
                        done = true; });
    while (!done)
        queue.drain();
    again.join();
}

// A started server on an ephemeral port, with a thread standing in for the
// editor's frame loop.
struct ServerFixture
{
    CommandRegistry registry = makeRegistry();
    MainThreadQueue queue;
    std::unique_ptr<ApiServer> server;
    std::atomic<bool> stopPump{false};
    std::thread pump;
    std::thread::id pumpId;

    explicit ServerFixture(const std::string& token = std::string())
    {
        ApiServerConfig config;
        config.port = 0;
        config.token = token;
        config.commandTimeout = std::chrono::seconds(5);
        server = std::make_unique<ApiServer>(registry, queue, config);
        std::string error;
        CHECK(server->start(&error));
        pump = std::thread(
            [this]
            {
                pumpId = std::this_thread::get_id();
                while (!stopPump)
                {
                    queue.drain();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            });
    }

    ~ServerFixture()
    {
        server->stop();
        stopPump = true;
        pump.join();
    }

    httplib::Client client() const
    {
        httplib::Client client("127.0.0.1", server->port());
        client.set_read_timeout(10, 0);
        return client;
    }
};

Json body(const httplib::Result& result)
{
    return result ? Json::parse(result->body, nullptr, false) : Json();
}

void testHttpEndpoints()
{
    ServerFixture fixture;
    httplib::Client client = fixture.client();

    auto health = client.Get("/api/health");
    CHECK(health && health->status == 200);
    CHECK(body(health)["apiVersion"] == ApiServer::kApiVersion);

    auto list = client.Get("/api/commands");
    CHECK(list && list->status == 200);
    CHECK(body(list)["commands"].size() == fixture.registry.commands().size());

    auto sum = client.Post("/api/commands/add", R"({"a":2,"b":40})", "application/json");
    CHECK(sum && sum->status == 200);
    CHECK(body(sum)["ok"] == true);
    CHECK(body(sum)["result"]["sum"] == 42);

    // No body at all means "no arguments", which `picture` accepts.
    auto picture = client.Post("/api/commands/picture", "", "application/json");
    CHECK(picture && picture->status == 200);
    CHECK(body(picture)["image"]["mimeType"] == "image/png");

    auto missing = client.Post("/api/commands/add", R"({"a":1})", "application/json");
    CHECK(missing && missing->status == 400);
    CHECK(body(missing)["error"]["code"] == "invalid_params");

    auto unknown = client.Post("/api/commands/nope", "{}", "application/json");
    CHECK(unknown && unknown->status == 404);

    auto failed = client.Post("/api/commands/fail", "{}", "application/json");
    CHECK(failed && failed->status == 422);
    CHECK(body(failed)["error"]["message"] == "nothing selected");

    auto crashed = client.Post("/api/commands/crash", "{}", "application/json");
    CHECK(crashed && crashed->status == 422);

    auto badJson = client.Post("/api/commands/add", "{not json", "application/json");
    CHECK(badJson && badJson->status == 400);

    auto notFound = client.Get("/api/nothing");
    CHECK(notFound && notFound->status == 404);
    CHECK(body(notFound)["ok"] == false);

    // The handler ran on the frame-loop thread, not the HTTP worker.
    auto thread = client.Post("/api/commands/thread_id", "{}", "application/json");
    CHECK(thread && thread->status == 200);
    CHECK(body(thread)["result"]["id"] ==
          std::hash<std::thread::id>()(fixture.pumpId));
}

void testHostAndOriginChecks()
{
    ServerFixture fixture;
    httplib::Client client = fixture.client();

    httplib::Headers rebinding = {{"Host", "evil.example.com"}};
    auto wrongHost = client.Get("/api/health", rebinding);
    CHECK(wrongHost && wrongHost->status == 403);

    httplib::Headers foreign = {{"Origin", "https://evil.example.com"}};
    auto wrongOrigin = client.Post("/api/commands/add", foreign, R"({"a":1,"b":1})",
                                   "application/json");
    CHECK(wrongOrigin && wrongOrigin->status == 403);

    httplib::Headers local = {{"Origin", "http://localhost:3000"}};
    auto localOrigin = client.Get("/api/health", local);
    CHECK(localOrigin && localOrigin->status == 200);
}

void testBearerToken()
{
    ServerFixture fixture("s3cret");
    httplib::Client client = fixture.client();

    // Health stays open so a client can tell "not running" from "wrong token".
    auto health = client.Get("/api/health");
    CHECK(health && health->status == 200);

    auto denied = client.Get("/api/commands");
    CHECK(denied && denied->status == 401);

    httplib::Headers wrong = {{"Authorization", "Bearer nope"}};
    auto wrongToken = client.Get("/api/commands", wrong);
    CHECK(wrongToken && wrongToken->status == 401);

    httplib::Headers right = {{"Authorization", "Bearer s3cret"}};
    auto allowed = client.Get("/api/commands", right);
    CHECK(allowed && allowed->status == 200);
}

void testServerLifecycle()
{
    CommandRegistry registry = makeRegistry();
    MainThreadQueue queue;
    ApiServerConfig config;
    config.port = 0;

    ApiServer first(registry, queue, config);
    CHECK(first.start());
    CHECK(first.running());
    const int port = first.port();
    CHECK(port > 0);

    // A second server cannot take a port that is in use.
    ApiServerConfig taken = config;
    taken.port = port;
    ApiServer second(registry, queue, taken);
    std::string error;
    CHECK(!second.start(&error));
    CHECK(!error.empty());

    first.stop();
    CHECK(!first.running());
    // Restartable on the same queue after a stop.
    CHECK(first.start());
    first.stop();
}

} // namespace

int main()
{
    testArgsValidation();
    testRegistryCall();
    testQueueRunsOnDrainingThread();
    testQueueTimeoutDropsTask();
    testQueueClose();
    testHttpEndpoints();
    testHostAndOriginChecks();
    testBearerToken();
    testServerLifecycle();

    if (gFailures)
        std::fprintf(stderr, "%d blender api test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
