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

// Thrown for a problem the caller can fix or an operation that legitimately could not be done; any other exception is a plain failure.
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

class CommandArgs
{
public:
    explicit CommandArgs(const Json& json);

    bool has(const char* name) const;
    // The raw value, for arguments that accept more than one shape; null when absent.
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

    // A scalar repeated `count` times, or an array of exactly `count` numbers.
    std::vector<double> numbersOrScalar(const char* name, size_t count,
                                        const std::vector<double>& fallback) const;

    std::vector<double> numbers(const char* name, size_t count,
                                const std::vector<double>& fallback) const;
    std::vector<double> requireNumbers(const char* name, size_t count) const;

    std::vector<unsigned> indices(const char* name, size_t maxCount) const;

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
    Json inputSchema = Json::object({{"type", "object"}, {"properties", Json::object()}});
    bool readOnly = false;
    // True when success adds exactly one undo step; false for read-only commands and ones that leave or empty the stack.
    bool undoable = true;
    CommandHandler handler;
};

class CommandRegistry
{
public:
    void add(CommandDef def);

    const CommandDef* find(const std::string& name) const;
    bool setUndoable(const std::string& name, bool undoable);
    const std::vector<CommandDef>& commands() const
    {
        return mCommands;
    }

    // Never throws; everything a handler throws becomes an outcome.
    CommandOutcome call(const std::string& name, const Json& args) const;

    Json describe() const;

private:
    std::vector<CommandDef> mCommands;
};

} // namespace Radion::BlenderApi

#endif // RADION_BLENDER_API_COMMAND_REGISTRY_H
