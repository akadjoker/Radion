#include "PCH.h"
#include "api/BlenderApiHost.h"

#include "api/BlenderCommands.h"

namespace Radion::BlenderApi
{

BlenderApiHost::BlenderApiHost(BlenderApplication& app)
{
    registerBlenderCommands(mRegistry, app);
}

BlenderApiHost::~BlenderApiHost()
{
    stop();
}

bool BlenderApiHost::start(const ApiServerConfig& config, std::string* error)
{
    stop();

    auto server = std::make_unique<ApiServer>(mRegistry, mQueue, config);
    if (!server->start(error))
        return false;

    mServer = std::move(server);
    mHost = config.host;
    mHasToken = !config.token.empty();
    Log::info("BlenderApi: listening on http://%s:%d (%zu commands%s)", mHost.c_str(), mServer->port(),
              mRegistry.commands().size(), mHasToken ? ", token required" : "");
    if (!mHasToken)
        Log::warning("BlenderApi: no token set - any local program can drive the editor");
    return true;
}

void BlenderApiHost::stop()
{
    if (!mServer)
        return;
    mServer->stop();
    mServer.reset();
    Log::info("BlenderApi: stopped");
}

bool BlenderApiHost::running() const
{
    return mServer && mServer->running();
}

int BlenderApiHost::port() const
{
    return mServer ? mServer->port() : 0;
}

void BlenderApiHost::pump()
{
    if (mServer)
        mQueue.drain();
}

} // namespace Radion::BlenderApi
