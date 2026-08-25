#include "PCH.h"

/** ****************************************************************************
  Radion Engine - Log implementation.
**************************************************************************** */
#include "Log.h"

#include <SDL2/SDL_log.h>
#include <cstdarg>
#include <cstdio>

namespace Radion
{

namespace
{

SDL_LogPriority toSdlPriority(LogLevel level)
{
    switch (level)
    {
    case LOG_INFO:
        return SDL_LOG_PRIORITY_INFO;
    case LOG_WARNING:
        return SDL_LOG_PRIORITY_WARN;
    case LOG_ERROR:
        return SDL_LOG_PRIORITY_ERROR;
    case LOG_DEBUG:
        return SDL_LOG_PRIORITY_DEBUG;
    }
    return SDL_LOG_PRIORITY_INFO;
}

#if defined(RADION_DEBUG)
LogMode gMode = LogMode::Verbose;
#else
LogMode gMode = LogMode::Passive;
#endif

Log::Sink gSink = nullptr;

} // namespace

void Log::setSink(Sink sink)
{
    gSink = sink;
}

void Log::setMode(LogMode mode)
{
    gMode = mode;
}

LogMode Log::getMode()
{
    return gMode;
}

bool Log::accepts(LogLevel level)
{
    switch (gMode)
    {
    case LogMode::None:
        return false;
    case LogMode::Passive:
        return level == LOG_WARNING || level == LOG_ERROR;
    case LogMode::Verbose:
        return true;
    }
    return true;
}

void Log::vwrite(LogLevel level, const char* fmt, std::va_list args)
{
    if (!accepts(level))
        return;

    char buffer[4096];
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    buffer[sizeof(buffer) - 1] = '\0';

    SDL_LogMessage(SDL_LOG_CATEGORY_APPLICATION, toSdlPriority(level), "%s", buffer);
    if (gSink)
        gSink(level, buffer);
}

void Log::info(const char* fmt, ...)
{
    std::va_list args;
    va_start(args, fmt);
    vwrite(LOG_INFO, fmt, args);
    va_end(args);
}

void Log::warning(const char* fmt, ...)
{
    std::va_list args;
    va_start(args, fmt);
    vwrite(LOG_WARNING, fmt, args);
    va_end(args);
}

void Log::error(const char* fmt, ...)
{
    std::va_list args;
    va_start(args, fmt);
    vwrite(LOG_ERROR, fmt, args);
    va_end(args);
}

void Log::debug(const char* fmt, ...)
{
    std::va_list args;
    va_start(args, fmt);
    vwrite(LOG_DEBUG, fmt, args);
    va_end(args);
}

} // namespace Radion
