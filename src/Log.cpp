// =============================================================================
//  FH4RPO - Log.cpp
//
//  Deliberately dependency free: a std::FILE* plus a mutex.  No CRT locks are
//  held across game frames and the logger works identically on the host build
//  used by the unit tests.
// =============================================================================
#include "fh4rpo/Log.hpp"

#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <mutex>
#include <atomic>
#include <ctime>
#include <chrono>

#ifdef _WIN32
#  include <windows.h>
#endif

namespace fh4rpo {

namespace {

std::mutex            g_logMutex;
std::FILE*            g_logFile = nullptr;
std::atomic<int>      g_level{ static_cast<int>(LogLevel::Info) };
char                  g_logPath[512] = {};
std::atomic<unsigned> g_droppedLines{ 0 };

const char* LevelName(LogLevel level) noexcept
{
    switch (level)
    {
    case LogLevel::Trace: return "TRACE";
    case LogLevel::Debug: return "DEBUG";
    case LogLevel::Info:  return "INFO ";
    case LogLevel::Warn:  return "WARN ";
    case LogLevel::Error: return "ERROR";
    default:              return "     ";
    }
}

void BuildTimeStamp(char* buffer, std::size_t size) noexcept
{
    using namespace std::chrono;
    const system_clock::time_point now = system_clock::now();
    const std::time_t seconds = system_clock::to_time_t(now);
    const auto millis = duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000;

    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &seconds);
#else
    localtime_r(&seconds, &local);
#endif
    std::snprintf(buffer, size, "%02d:%02d:%02d.%03lld",
                  local.tm_hour, local.tm_min, local.tm_sec,
                  static_cast<long long>(millis));
}

unsigned CurrentThreadIdValue() noexcept
{
#ifdef _WIN32
    return static_cast<unsigned>(GetCurrentThreadId());
#else
    static std::atomic<unsigned> counter{ 1 };
    thread_local unsigned id = counter.fetch_add(1);
    return id;
#endif
}

} // namespace

void Log::Initialize(const char* path, LogLevel level) noexcept
{
    std::lock_guard<std::mutex> lock(g_logMutex);
    g_level.store(static_cast<int>(level), std::memory_order_relaxed);

    if (g_logFile != nullptr)
    {
        std::fclose(g_logFile);
        g_logFile = nullptr;
    }

    if (path == nullptr || path[0] == '\0')
        return;

    std::snprintf(g_logPath, sizeof(g_logPath), "%s", path);

#ifdef _MSC_VER
    if (fopen_s(&g_logFile, path, "wb") != 0)
        g_logFile = nullptr;
#else
    g_logFile = std::fopen(path, "wb");
#endif
    if (g_logFile == nullptr)
        return;

    // UTF-8 BOM: makes the log readable in editors that still default to ANSI.
    const unsigned char bom[3] = { 0xEF, 0xBB, 0xBF };
    std::fwrite(bom, 1, sizeof(bom), g_logFile);
}

void Log::Shutdown() noexcept
{
    std::lock_guard<std::mutex> lock(g_logMutex);
    if (g_logFile != nullptr)
    {
        std::fflush(g_logFile);
        std::fclose(g_logFile);
        g_logFile = nullptr;
    }
}

void Log::SetLevel(LogLevel level) noexcept
{
    g_level.store(static_cast<int>(level), std::memory_order_relaxed);
}

LogLevel Log::GetLevel() noexcept
{
    return static_cast<LogLevel>(g_level.load(std::memory_order_relaxed));
}

const char* Log::FilePath() noexcept { return g_logPath; }

void Log::WriteV(LogLevel level, const char* fmt, va_list args) noexcept
{
    if (static_cast<int>(level) < g_level.load(std::memory_order_relaxed))
        return;
    if (fmt == nullptr)
        return;

    char message[2048] = {};
    std::vsnprintf(message, sizeof(message), fmt, args);
    message[sizeof(message) - 1] = '\0';

    char stamp[32] = {};
    BuildTimeStamp(stamp, sizeof(stamp));

    char line[2304] = {};
    std::snprintf(line, sizeof(line), "[%s][%s][tid:%u] %s\r\n",
                  stamp, LevelName(level), CurrentThreadIdValue(), message);

#ifdef _WIN32
    OutputDebugStringA(line);
#endif

    std::lock_guard<std::mutex> lock(g_logMutex);
    if (g_logFile == nullptr)
        return;

    if (std::fputs(line, g_logFile) < 0)
    {
        g_droppedLines.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Warn and above are flushed immediately so a crash cannot swallow them.
    if (level >= LogLevel::Warn)
        std::fflush(g_logFile);
}

void Log::Write(LogLevel level, const char* fmt, ...) noexcept
{
    va_list args;
    va_start(args, fmt);
    WriteV(level, fmt, args);
    va_end(args);
}

void Log::Flush() noexcept
{
    std::lock_guard<std::mutex> lock(g_logMutex);
    if (g_logFile != nullptr)
        std::fflush(g_logFile);
}

#define FH4RPO_DEFINE_LOG_WRAPPER(NAME, LEVEL)                                  \
    void Log::NAME(const char* fmt, ...) noexcept                               \
    {                                                                           \
        va_list args;                                                           \
        va_start(args, fmt);                                                    \
        WriteV(LogLevel::LEVEL, fmt, args);                                     \
        va_end(args);                                                           \
    }

FH4RPO_DEFINE_LOG_WRAPPER(Trace, Trace)
FH4RPO_DEFINE_LOG_WRAPPER(Debug, Debug)
FH4RPO_DEFINE_LOG_WRAPPER(Info,  Info)
FH4RPO_DEFINE_LOG_WRAPPER(Warn,  Warn)
FH4RPO_DEFINE_LOG_WRAPPER(Error, Error)

#undef FH4RPO_DEFINE_LOG_WRAPPER

} // namespace fh4rpo
