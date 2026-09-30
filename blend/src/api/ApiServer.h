#ifndef RADION_BLENDER_API_SERVER_H
#define RADION_BLENDER_API_SERVER_H

#include "api/CommandRegistry.h"
#include "api/MainThreadQueue.h"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace httplib
{
class Server;
}

namespace Radion::BlenderApi
{

struct ApiServerConfig
{
    // Loopback only by default: the commands can open and write files. Any other address is refused unless `token` is
    // set.
    std::string host = "127.0.0.1";
    // 0 lets the OS pick a free port; port() reports which.
    int port = 7420;
    // When not empty every request except /api/health must carry "Authorization: Bearer <token>".
    std::string token;
    std::chrono::milliseconds commandTimeout{30000};
    size_t maxBodyBytes = 16u * 1024u * 1024u;
};

// Local HTTP/JSON API over a CommandRegistry.
//
//   GET  /api/health              liveness + API version
//   GET  /api/commands            every command with its description and JSON Schema
//   POST /api/commands/{name}     run one; the body is the arguments object
//
// Success is {"ok":true,"result":{...}} (plus "image":{"mimeType","data"} for a
// command that returns one); failure is {"ok":false,"error":{"code","message"}}
// with a matching HTTP status. Commands never run on the HTTP thread: they go
// through the MainThreadQueue and execute when the editor drains it.
class ApiServer
{
public:
    static constexpr int kApiVersion = 1;

    ApiServer(const CommandRegistry& registry, MainThreadQueue& queue, ApiServerConfig config);
    ~ApiServer();

    ApiServer(const ApiServer&) = delete;
    ApiServer& operator=(const ApiServer&) = delete;

    bool start(std::string* error = nullptr);
    void stop();

    bool running() const;
    int port() const
    {
        return mBoundPort;
    }

private:
    bool authorized(const std::string& authorization) const;
    static bool isLocalHostName(const std::string& hostHeader);
    static bool isLocalOrigin(const std::string& origin);

    const CommandRegistry& mRegistry;
    MainThreadQueue& mQueue;
    ApiServerConfig mConfig;
    std::unique_ptr<httplib::Server> mServer;
    std::thread mThread;
    int mBoundPort = 0;
};

} // namespace Radion::BlenderApi

#endif // RADION_BLENDER_API_SERVER_H
