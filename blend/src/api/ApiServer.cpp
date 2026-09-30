#include "api/ApiServer.h"

#include <httplib/httplib.h>

#include <algorithm>
#include <cctype>
#include <utility>

namespace Radion::BlenderApi
{

namespace
{

const char* statusCode(CommandStatus status)
{
    switch (status)
    {
    case CommandStatus::Ok: return "ok";
    case CommandStatus::UnknownCommand: return "unknown_command";
    case CommandStatus::InvalidParams: return "invalid_params";
    case CommandStatus::Failed: return "failed";
    case CommandStatus::Timeout: return "timeout";
    case CommandStatus::Unavailable: return "unavailable";
    }
    return "failed";
}

int httpStatus(CommandStatus status)
{
    switch (status)
    {
    case CommandStatus::Ok: return 200;
    case CommandStatus::UnknownCommand: return 404;
    case CommandStatus::InvalidParams: return 400;
    // The request was fine; the editor just could not do what it asked.
    case CommandStatus::Failed: return 422;
    case CommandStatus::Timeout: return 504;
    case CommandStatus::Unavailable: return 503;
    }
    return 500;
}

void sendJson(httplib::Response& response, int status, const Json& body)
{
    response.status = status;
    // Commands echo paths and mesh names; a bad character encoding must not turn the reply into an exception.
    response.set_content(body.dump(-1, ' ', false, Json::error_handler_t::replace),
                         "application/json; charset=utf-8");
}

void sendError(httplib::Response& response, CommandStatus status, const std::string& message)
{
    sendJson(response, httpStatus(status),
             {{"ok", false}, {"error", {{"code", statusCode(status)}, {"message", message}}}});
}

std::string lower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

std::string hostOnly(const std::string& value)
{
    if (!value.empty() && value.front() == '[')
    {
        const size_t close = value.find(']');
        return close == std::string::npos ? value : value.substr(0, close + 1);
    }
    return value.substr(0, value.find(':'));
}

bool constantTimeEquals(const std::string& a, const std::string& b)
{
    unsigned char difference = static_cast<unsigned char>(a.size() != b.size());
    const size_t length = std::min(a.size(), b.size());
    for (size_t i = 0; i < length; ++i)
        difference |= static_cast<unsigned char>(a[i] ^ b[i]);
    return difference == 0;
}

} // namespace

ApiServer::ApiServer(const CommandRegistry& registry, MainThreadQueue& queue,
                     ApiServerConfig config)
    : mRegistry(registry), mQueue(queue), mConfig(std::move(config))
{
}

ApiServer::~ApiServer()
{
    stop();
}

bool ApiServer::isLocalHostName(const std::string& hostHeader)
{
    const std::string name = lower(hostOnly(hostHeader));
    return name == "localhost" || name == "127.0.0.1" || name == "[::1]";
}

bool ApiServer::isLocalOrigin(const std::string& origin)
{
    const size_t scheme = origin.find("://");
    if (scheme == std::string::npos)
        return false;
    return isLocalHostName(origin.substr(scheme + 3));
}

bool ApiServer::authorized(const std::string& authorization) const
{
    if (mConfig.token.empty())
        return true;
    static const std::string prefix = "Bearer ";
    if (authorization.compare(0, prefix.size(), prefix) != 0)
        return false;
    return constantTimeEquals(authorization.substr(prefix.size()), mConfig.token);
}

bool ApiServer::running() const
{
    return mServer && mServer->is_running();
}

bool ApiServer::start(std::string* error)
{
    if (mServer)
        return true;

    // Listening beyond this machine requires a secret: the commands read and write files.
    const bool loopbackOnly = mConfig.host == "::1" || isLocalHostName(mConfig.host);
    if (!loopbackOnly && mConfig.token.empty())
    {
        if (error)
            *error = "a token is required to listen on '" + mConfig.host + "' (not loopback)";
        return false;
    }

    auto server = std::make_unique<httplib::Server>();
    server->set_payload_max_length(mConfig.maxBodyBytes);

    // httplib's default SO_REUSEPORT lets a second editor bind the same port; allow only quick rebind, refuse sharing on
    // Windows.
    server->set_socket_options(
        [](socket_t sock)
        {
            int enable = 1;
#ifdef _WIN32
            setsockopt(sock, SOL_SOCKET, SO_EXCLUSIVEADDRUSE,
                       reinterpret_cast<const char*>(&enable), sizeof(enable));
#else
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
#endif
        });

    // A web page can make a browser hit 127.0.0.1 (even read it via DNS rebinding); requiring a loopback Host and no
    // foreign Origin keeps the API to local tools.
    server->set_pre_routing_handler(
        [this, loopbackOnly](const httplib::Request& request, httplib::Response& response)
        {
            // Only a loopback server knows what its Host header must say; a wider one is protected by the token.
            if (loopbackOnly && !isLocalHostName(request.get_header_value("Host")))
            {
                sendError(response, CommandStatus::InvalidParams, "Host not allowed");
                response.status = 403;
                return httplib::Server::HandlerResponse::Handled;
            }
            if (request.has_header("Origin") &&
                !isLocalOrigin(request.get_header_value("Origin")))
            {
                sendError(response, CommandStatus::InvalidParams, "Origin not allowed");
                response.status = 403;
                return httplib::Server::HandlerResponse::Handled;
            }
            if (request.path != "/api/health" &&
                !authorized(request.get_header_value("Authorization")))
            {
                sendError(response, CommandStatus::InvalidParams,
                          "missing or wrong bearer token");
                response.status = 401;
                response.set_header("WWW-Authenticate", "Bearer");
                return httplib::Server::HandlerResponse::Handled;
            }
            return httplib::Server::HandlerResponse::Unhandled;
        });

    server->set_exception_handler(
        [](const httplib::Request&, httplib::Response& response, std::exception_ptr)
        { sendError(response, CommandStatus::Failed, "internal error"); response.status = 500; });

    server->set_error_handler(
        [](const httplib::Request&, httplib::Response& response)
        {
            // Fills in only what httplib rejects itself (unknown path, bad method, too big).
            if (response.body.empty())
                sendJson(response, response.status,
                         {{"ok", false},
                          {"error",
                           {{"code", "http_error"},
                            {"message", "HTTP " + std::to_string(response.status)}}}});
        });

    server->Get("/api/health",
                [](const httplib::Request&, httplib::Response& response)
                {
                    sendJson(response, 200,
                             {{"ok", true},
                              {"name", "radion_blender"},
                              {"apiVersion", kApiVersion}});
                });

    server->Get("/api/commands",
                [this](const httplib::Request&, httplib::Response& response)
                { sendJson(response, 200, {{"ok", true}, {"commands", mRegistry.describe()}}); });

    server->Post(
        R"(/api/commands/([A-Za-z0-9_]+))",
        [this](const httplib::Request& request, httplib::Response& response)
        {
            const std::string name = request.matches[1];

            Json args = Json::object();
            if (!request.body.empty())
            {
                args = Json::parse(request.body, nullptr, false);
                if (args.is_discarded())
                {
                    sendError(response, CommandStatus::InvalidParams, "body is not valid JSON");
                    return;
                }
            }

            const CommandOutcome outcome = mQueue.run(
                [this, name, args] { return mRegistry.call(name, args); },
                mConfig.commandTimeout);

            if (outcome.status != CommandStatus::Ok)
            {
                sendError(response, outcome.status, outcome.message);
                return;
            }

            Json body = {{"ok", true}, {"result", outcome.result.data}};
            if (outcome.result.image)
                body["image"] = {{"mimeType", outcome.result.image->mimeType},
                                 {"data", outcome.result.image->dataBase64}};
            sendJson(response, 200, body);
        });

    int port = mConfig.port;
    if (port == 0)
    {
        port = server->bind_to_any_port(mConfig.host);
        if (port < 0)
        {
            if (error)
                *error = "could not bind " + mConfig.host;
            return false;
        }
    }
    else if (!server->bind_to_port(mConfig.host, port))
    {
        if (error)
            *error = "could not bind " + mConfig.host + ":" + std::to_string(port) +
                     " (already in use?)";
        return false;
    }

    mBoundPort = port;
    mServer = std::move(server);
    httplib::Server* raw = mServer.get();
    mThread = std::thread([raw] { raw->listen_after_bind(); });
    mServer->wait_until_ready();
    return true;
}

void ApiServer::stop()
{
    if (!mServer)
        return;

    // Requests blocked on the queue would keep the listener from stopping.
    mQueue.close();
    mServer->stop();
    if (mThread.joinable())
        mThread.join();
    mServer.reset();
    mBoundPort = 0;
    mQueue.open();
}

} // namespace Radion::BlenderApi
