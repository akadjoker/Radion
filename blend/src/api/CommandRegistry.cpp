#include "api/CommandRegistry.h"

#include <cmath>
#include <utility>

namespace Radion::BlenderApi
{

CommandArgs::CommandArgs(const Json& json) : mJson(json)
{
    if (!json.is_object())
        throw CommandError(CommandStatus::InvalidParams, "arguments must be a JSON object");
}

const Json* CommandArgs::find(const char* name) const
{
    const auto it = mJson.find(name);
    if (it == mJson.end() || it->is_null())
        return nullptr;
    return &*it;
}

void CommandArgs::reject(const char* name, const std::string& why) const
{
    throw CommandError(CommandStatus::InvalidParams,
                       std::string("argument '") + name + "' " + why);
}

bool CommandArgs::has(const char* name) const
{
    return find(name) != nullptr;
}

std::string CommandArgs::requireString(const char* name) const
{
    const Json* value = find(name);
    if (!value)
        reject(name, "is required");
    if (!value->is_string())
        reject(name, "must be a string");
    return value->get<std::string>();
}

std::string CommandArgs::string(const char* name, const std::string& fallback) const
{
    return find(name) ? requireString(name) : fallback;
}

double CommandArgs::requireNumber(const char* name) const
{
    const Json* value = find(name);
    if (!value)
        reject(name, "is required");
    if (!value->is_number())
        reject(name, "must be a number");
    const double number = value->get<double>();
    if (!std::isfinite(number))
        reject(name, "must be finite");
    return number;
}

double CommandArgs::number(const char* name, double fallback) const
{
    return find(name) ? requireNumber(name) : fallback;
}

double CommandArgs::number(const char* name, double fallback, double minimum,
                           double maximum) const
{
    const double value = number(name, fallback);
    if (value < minimum || value > maximum)
        reject(name, "must be between " + std::to_string(minimum) + " and " +
                         std::to_string(maximum));
    return value;
}

long long CommandArgs::requireInteger(const char* name) const
{
    const Json* value = find(name);
    if (!value)
        reject(name, "is required");
    if (!value->is_number_integer())
        reject(name, "must be an integer");
    return value->get<long long>();
}

long long CommandArgs::integer(const char* name, long long fallback) const
{
    return find(name) ? requireInteger(name) : fallback;
}

long long CommandArgs::integer(const char* name, long long fallback, long long minimum,
                               long long maximum) const
{
    const long long value = integer(name, fallback);
    if (value < minimum || value > maximum)
        reject(name, "must be between " + std::to_string(minimum) + " and " +
                         std::to_string(maximum));
    return value;
}

bool CommandArgs::boolean(const char* name, bool fallback) const
{
    const Json* value = find(name);
    if (!value)
        return fallback;
    if (!value->is_boolean())
        reject(name, "must be true or false");
    return value->get<bool>();
}

std::vector<double> CommandArgs::requireNumbers(const char* name, size_t count) const
{
    const Json* value = find(name);
    if (!value)
        reject(name, "is required");
    if (!value->is_array() || value->size() != count)
        reject(name, "must be an array of " + std::to_string(count) + " numbers");

    std::vector<double> out;
    out.reserve(count);
    for (const Json& item : *value)
    {
        if (!item.is_number() || !std::isfinite(item.get<double>()))
            reject(name, "must contain only finite numbers");
        out.push_back(item.get<double>());
    }
    return out;
}

std::vector<double> CommandArgs::numbers(const char* name, size_t count,
                                         const std::vector<double>& fallback) const
{
    return find(name) ? requireNumbers(name, count) : fallback;
}

std::vector<double> CommandArgs::numbersOrScalar(const char* name, size_t count,
                                                 const std::vector<double>& fallback) const
{
    const Json* value = find(name);
    if (!value)
        return fallback;
    if (value->is_number())
        return std::vector<double>(count, requireNumber(name));
    return requireNumbers(name, count);
}

std::vector<unsigned> CommandArgs::indices(const char* name, size_t maxCount) const
{
    const Json* value = find(name);
    if (!value)
        return {};
    if (!value->is_array())
        reject(name, "must be an array of non-negative integers");
    if (value->size() > maxCount)
        reject(name, "may hold at most " + std::to_string(maxCount) + " entries");

    std::vector<unsigned> out;
    out.reserve(value->size());
    for (const Json& item : *value)
    {
        if (!item.is_number_integer() || item.get<long long>() < 0 ||
            item.get<long long>() > 0xFFFFFFFFll)
            reject(name, "must contain only non-negative integers");
        out.push_back(static_cast<unsigned>(item.get<long long>()));
    }
    return out;
}

std::string CommandArgs::choice(const char* name, const std::vector<std::string>& allowed,
                                const std::string& fallback) const
{
    return find(name) ? requireChoice(name, allowed) : fallback;
}

std::string CommandArgs::requireChoice(const char* name,
                                       const std::vector<std::string>& allowed) const
{
    const std::string value = requireString(name);
    for (const std::string& candidate : allowed)
    {
        if (candidate == value)
            return value;
    }

    std::string list;
    for (const std::string& candidate : allowed)
        list += (list.empty() ? "" : ", ") + candidate;
    reject(name, "must be one of: " + list);
}

void CommandRegistry::add(CommandDef def)
{
    for (CommandDef& existing : mCommands)
    {
        if (existing.name == def.name)
        {
            existing = std::move(def);
            return;
        }
    }
    mCommands.push_back(std::move(def));
}

const CommandDef* CommandRegistry::find(const std::string& name) const
{
    for (const CommandDef& def : mCommands)
    {
        if (def.name == name)
            return &def;
    }
    return nullptr;
}

CommandOutcome CommandRegistry::call(const std::string& name, const Json& args) const
{
    CommandOutcome outcome;
    const CommandDef* def = find(name);
    if (!def || !def->handler)
    {
        outcome.status = CommandStatus::UnknownCommand;
        outcome.message = "unknown command '" + name + "'";
        return outcome;
    }

    try
    {
        const CommandArgs commandArgs(args);
        outcome.result = def->handler(commandArgs);
    }
    catch (const CommandError& error)
    {
        outcome.status = error.status();
        outcome.message = error.what();
    }
    catch (const Json::exception& error)
    {
        outcome.status = CommandStatus::InvalidParams;
        outcome.message = error.what();
    }
    catch (const std::exception& error)
    {
        outcome.status = CommandStatus::Failed;
        outcome.message = error.what();
    }
    catch (...)
    {
        outcome.status = CommandStatus::Failed;
        outcome.message = "unknown error";
    }
    return outcome;
}

bool CommandRegistry::setUndoable(const std::string& name, bool undoable)
{
    for (CommandDef& def : mCommands)
    {
        if (def.name == name)
        {
            def.undoable = undoable;
            return true;
        }
    }
    return false;
}

Json CommandRegistry::describe() const
{
    Json list = Json::array();
    for (const CommandDef& def : mCommands)
    {
        list.push_back({{"name", def.name},
                        {"description", def.description},
                        {"readOnly", def.readOnly},
                        {"undoable", def.undoable && !def.readOnly},
                        {"inputSchema", def.inputSchema}});
    }
    return list;
}

} // namespace Radion::BlenderApi
