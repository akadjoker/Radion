#ifndef RADION_LOG_H
#define RADION_LOG_H

#include <cstdarg>

namespace Radion
{

enum LogLevel
{
    LOG_INFO = 0,
    LOG_WARNING,
    LOG_ERROR,
    LOG_DEBUG
};

enum class LogMode
{
    None,    // nothing gets through
    Passive, // warnings and errors only
    Verbose  // everything, debug included
};

class Log
{
public:
    static void info(const char* fmt, ...);
    static void warning(const char* fmt, ...);
    static void error(const char* fmt, ...);
    static void debug(const char* fmt, ...);

    static void vwrite(LogLevel level, const char* fmt, std::va_list args);

    static void setMode(LogMode mode);
    static LogMode getMode();
    static bool accepts(LogLevel level);

    // One slot, not a list of subscribers - the editor's Console panel is the
    // only thing that has ever needed every message as it happens rather
    // than reading them back off the platform log. Called with the finished,
    // formatted string (after the level's own filtering), in addition to -
    // never instead of - the SDL_LogMessage() output every build still gets.
    using Sink = void (*)(LogLevel level, const char* message);
    static void setSink(Sink sink);
};

} // namespace Radion

#endif // RADION_LOG_H
