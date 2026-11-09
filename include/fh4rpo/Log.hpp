// =============================================================================
//  FH4RPO - Log.hpp
//  Thread safe, allocation-light logger.  Every subsystem reports through this
//  so that a field problem can be diagnosed from FH4RPO.log alone.
// =============================================================================
#pragma once

#include <string>
#include <cstdarg>

namespace fh4rpo {

enum class LogLevel : int
{
    Trace = 0,
    Debug = 1,
    Info  = 2,
    Warn  = 3,
    Error = 4,
    Off   = 5
};

class Log
{
public:
    // path == nullptr -> no file, debugger output only.
    static void Initialize(const char* path, LogLevel level) noexcept;
    static void Shutdown() noexcept;
    static void SetLevel(LogLevel level) noexcept;
    static LogLevel GetLevel() noexcept;
    static const char* FilePath() noexcept;

    static void Write(LogLevel level, const char* fmt, ...) noexcept;
    static void WriteV(LogLevel level, const char* fmt, va_list args) noexcept;
    static void Flush() noexcept;

    // Convenience wrappers - always available, filtered internally.
    static void Trace(const char* fmt, ...) noexcept;
    static void Debug(const char* fmt, ...) noexcept;
    static void Info(const char* fmt, ...) noexcept;
    static void Warn(const char* fmt, ...) noexcept;
    static void Error(const char* fmt, ...) noexcept;
};

} // namespace fh4rpo

#define FH4RPO_LOG_TRACE(...) ::fh4rpo::Log::Trace(__VA_ARGS__)
#define FH4RPO_LOG_DEBUG(...) ::fh4rpo::Log::Debug(__VA_ARGS__)
#define FH4RPO_LOG_INFO(...)  ::fh4rpo::Log::Info(__VA_ARGS__)
#define FH4RPO_LOG_WARN(...)  ::fh4rpo::Log::Warn(__VA_ARGS__)
#define FH4RPO_LOG_ERROR(...) ::fh4rpo::Log::Error(__VA_ARGS__)
