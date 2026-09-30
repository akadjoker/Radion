#ifndef RADION_BLENDER_API_COMMAND_REGISTRY_H
#define RADION_BLENDER_API_COMMAND_REGISTRY_H

#include <nlohmann/json.hpp>

#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace Radion::BlenderApi
{

using Json = nlohmann::json;

enum class CommandStatus
{
    Ok,
    UnknownCommand,
    InvalidParams,
    Failed,
    Timeout,
    Unavailable
};

// Thrown by a handler for a problem the caller can fix (bad or missing
// argument) or an operation that legitimately could not be carried out. Any
// other exception is reported as a plain failure.
class CommandError : public std::runtime_error
{
public:
    CommandError(CommandStatus status, const std::string& message)
        : std::runtime_error(message), mStatus(status)
    {
    }

    CommandStatus status() const
    {
        return mStatus;
    }

private:
    CommandStatus mStatus;
};

struct CommandImage
{
    std::string mimeType;
    std::string dataBase64;
};

struct CommandResult
{
    Json data = Json::object();
    std::optional<CommandImage> image;
};

struct CommandOutcome
{
    CommandStatus status = CommandStatus::Ok;
    CommandResult result;
    std::string message;
};

// Typed, validating view over a command's arguments. Every accessor throws
// CommandError(InvalidParams) with the argument's name in the message, so a
// handler reads as a list of what it needs and never checks types by hand.
class CommandArgs
{
public:
    explicit CommandArgs(const Json& json);

    bool has(const char* name) const;
    // The raw value, for the rare argument that accepts more than one shape
    // (a part given by index or by name, a colour given as hex or as numbers).
    // Null when absent or null.
    const Json* raw(const char* name) const
    {
        return find(name);
    }

    std::string requireString(const char* name) const;
    std::string string(const char* name, const std::string& fallback) const;

    double number(const char* name, double fallback) const;
    double number(const char* name, double fallback, double minimum, double maximum) const;
    double requireNumber(const char* name) const;

    long long integer(const char* name, long long fallback) const;
    long long integer(const char* name, long long fallback, long long minimum,
                      long long maximum) const;
    long long requireInteger(const char* name) const;

    bool boolean(const char* name, bool fallback) const;

    // A number, which is repeated `count` times, or an array of exactly
    // `count` numbers - a uniform scale written as 2 or as [2, 2, 2].
    std::vector<double> numbersOrScalar(const char* name, size_t count,
                                        const std::vector<double>& fallback) const;

    // A JSON array of exactly `count` numbers, or `fallback` when absent.
    std::vector<double> numbers(const char* name, size_t count,
                                const std::vector<double>& fallback) const;
    std::vector<double> requireNumbers(const char* name, size_t count) const;

    // A non-negative integer array; `maxCount` bounds what one call may carry.
    std::vector<unsigned> indices(const char* name, size_t maxCount) const;

    // One of `allowed`; `fallback` when the argument is absent.
    std::string choice(const char* name, const std::vector<std::string>& allowed,
                       const std::string& fallback) const;
    std::string requireChoice(const char* name, const std::vector<std::string>& allowed) const;

private:
    const Json* find(const char* name) const;
    [[noreturn]] void reject(const char* name, const std::string& why) const;

    const Json& mJson;
};

using CommandHandler = std::function<CommandResult(const CommandArgs&)>;

struct CommandDef
{
    std::string name;
    std::string description;
    // JSON Schema for the arguments object. Advertised as-is to clients; the
    // handler's CommandArgs calls are what actually enforce it.
    Json inputSchema = Json::object({{"type", "object"}, {"properties", Json::object()}});
    // True when the command never changes the document.
    bool readOnly = false;
    CommandHandler handler;
};

class CommandRegistry
{
public:
    // Replaces a command of the same name.
    void add(CommandDef def);

    const CommandDef* find(const std::string& name) const;
    const std::vector<CommandDef>& commands() const
    {
        return mCommands;
    }

    // Runs the handler on the calling thread and converts everything it can
    // throw into an outcome; never throws itself.
    CommandOutcome call(const std::string& name, const Json& args) const;

    // [{name, description, readOnly, inputSchema}] - what GET /api/commands serves.
    Json describe() const;

private:
    std::vector<CommandDef> mCommands;
};

} // namespace Radion::BlenderApi

#endif // RADION_BLENDER_API_COMMAND_REGISTRY_H
