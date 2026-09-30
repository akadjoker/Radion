#ifndef RADION_BLENDER_API_BLENDER_API_HOST_H
#define RADION_BLENDER_API_BLENDER_API_HOST_H

#include "api/ApiServer.h"
#include "api/CommandRegistry.h"
#include "api/MainThreadQueue.h"

#include <memory>
#include <string>

namespace Radion
{
class BlenderApplication;
}

namespace Radion::BlenderApi
{

// The editor's side of the API: the command table, the hand-off queue and the
// HTTP server, tied together. The application owns one, calls pump() once per
// frame, and starts/stops the server from the preferences or the command line.
class BlenderApiHost
{
public:
    explicit BlenderApiHost(BlenderApplication& app);
    ~BlenderApiHost();

    BlenderApiHost(const BlenderApiHost&) = delete;
    BlenderApiHost& operator=(const BlenderApiHost&) = delete;

    bool start(const ApiServerConfig& config, std::string* error = nullptr);
    void stop();

    bool running() const;
    int port() const;
    const std::string& host() const
    {
        return mHost;
    }
    bool hasToken() const
    {
        return mHasToken;
    }

    // Frame-loop thread, once per frame: runs the commands that arrived since
    // the last one.
    void pump();

private:
    CommandRegistry mRegistry;
    MainThreadQueue mQueue;
    std::unique_ptr<ApiServer> mServer;
    std::string mHost;
    bool mHasToken = false;
};

} // namespace Radion::BlenderApi

#endif // RADION_BLENDER_API_BLENDER_API_HOST_H
