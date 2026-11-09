// =============================================================================
//  FH4RPO - MemoryGuard.cpp
// =============================================================================
#include "fh4rpo/MemoryGuard.hpp"
#include "fh4rpo/Log.hpp"

#include <atomic>
#include <cstring>

#ifdef _WIN32
#  include <windows.h>
#endif

namespace fh4rpo {

#ifdef _WIN32

namespace {

std::atomic<int>           g_tripCount{ 0 };
std::atomic<std::uint32_t> g_lastException{ 0 };
std::atomic<bool>          g_panicLatched{ false };
std::atomic<bool>          g_watchdogInstalled{ false };

// Exception filter that never lets the process die while our code is on the
// stack.  Returning EXCEPTION_EXECUTE_HANDLER here turns an access violation
// into a normal C++ return path.

LONG CALLBACK PanicFilter(EXCEPTION_POINTERS* info) noexcept
{
    if (info && info->ExceptionRecord)
        g_lastException.store(info->ExceptionRecord->ExceptionCode, std::memory_order_relaxed);

    MemoryGuard::OnTrip();
    g_panicLatched.store(true, std::memory_order_relaxed);

    FH4RPO_LOG_ERROR("MemoryGuard: unhandled exception 0x%08lX intercepted (panic latched)",
                     info && info->ExceptionRecord ? info->ExceptionRecord->ExceptionCode : 0u);

    return EXCEPTION_EXECUTE_HANDLER;
}

bool HasProtection(unsigned long protect, unsigned long flag) noexcept
{
    if (protect & PAGE_GUARD)
        return false;
    const unsigned long base = protect & 0xFFu;
    switch (flag)
    {
    case PAGE_READONLY:
        return base == PAGE_READONLY || base == PAGE_READWRITE ||
               base == PAGE_WRITECOPY || base == PAGE_EXECUTE_READ ||
               base == PAGE_EXECUTE_READWRITE || base == PAGE_EXECUTE_WRITECOPY;
    case PAGE_READWRITE:
        return base == PAGE_READWRITE || base == PAGE_WRITECOPY ||
               base == PAGE_EXECUTE_READWRITE || base == PAGE_EXECUTE_WRITECOPY;
    case PAGE_EXECUTE_READ:
        return base == PAGE_EXECUTE || base == PAGE_EXECUTE_READ ||
               base == PAGE_EXECUTE_READWRITE || base == PAGE_EXECUTE_WRITECOPY;
    default:
        return false;
    }
}

} // namespace

// -----------------------------------------------------------------------------
//  Region introspection
// -----------------------------------------------------------------------------
bool MemoryGuard::Query(std::uintptr_t address, MemRegionInfo& out) noexcept
{
    out = MemRegionInfo{};
    if (address == 0)
        return false;

    MEMORY_BASIC_INFORMATION mbi{};
    const SIZE_T result = VirtualQuery(reinterpret_cast<LPCVOID>(address), &mbi, sizeof(mbi));
    if (result != sizeof(mbi))
        return false;

    out.base       = reinterpret_cast<std::uintptr_t>(mbi.BaseAddress);
    out.size       = static_cast<std::size_t>(mbi.RegionSize);
    out.protect    = mbi.Protect;
    out.state      = mbi.State;
    out.type       = mbi.Type;
    out.committed  = (mbi.State == MEM_COMMIT);
    out.readable   = out.committed && HasProtection(mbi.Protect, PAGE_READONLY);
    out.writable   = out.committed && HasProtection(mbi.Protect, PAGE_READWRITE);
    out.executable = out.committed && HasProtection(mbi.Protect, PAGE_EXECUTE_READ);
    return true;
}

bool MemoryGuard::IsRangeReadable(std::uintptr_t address, std::size_t size) noexcept
{
    if (address == 0 || size == 0)
        return false;

    std::uintptr_t cursor = address;
    std::uintptr_t end    = address + size;
    if (end < address)                 // integer overflow -> reject
        return false;

    while (cursor < end)
    {
        MemRegionInfo info{};
        if (!Query(cursor, info) || !info.readable)
            return false;
        const std::uintptr_t regionEnd = info.base + info.size;
        if (regionEnd <= cursor)
            return false;
        cursor = regionEnd;
    }
    return true;
}

bool MemoryGuard::IsRangeWritable(std::uintptr_t address, std::size_t size) noexcept
{
    if (address == 0 || size == 0)
        return false;

    std::uintptr_t cursor = address;
    const std::uintptr_t end = address + size;
    if (end < address)
        return false;

    while (cursor < end)
    {
        MemRegionInfo info{};
        if (!Query(cursor, info) || !info.writable)
            return false;
        const std::uintptr_t regionEnd = info.base + info.size;
        if (regionEnd <= cursor)
            return false;
        cursor = regionEnd;
    }
    return true;
}

bool MemoryGuard::IsRangeExecutable(std::uintptr_t address, std::size_t size) noexcept
{
    if (address == 0 || size == 0)
        return false;

    std::uintptr_t cursor = address;
    const std::uintptr_t end = address + size;
    if (end < address)
        return false;

    while (cursor < end)
    {
        MemRegionInfo info{};
        if (!Query(cursor, info) || !info.executable)
            return false;
        const std::uintptr_t regionEnd = info.base + info.size;
        if (regionEnd <= cursor)
            return false;
        cursor = regionEnd;
    }
    return true;
}

// -----------------------------------------------------------------------------
//  Block access
// -----------------------------------------------------------------------------
MemStatus MemoryGuard::ReadBlock(std::uintptr_t address, void* dst, std::size_t size) noexcept
{
    if (address == 0)
        return MemStatus::NullPointer;
    if (dst == nullptr || size == 0)
        return MemStatus::InvalidArgument;
    if (!IsRangeReadable(address, size))
        return MemStatus::NotReadable;

    FH4RPO_TRY
    {
        std::memcpy(dst, reinterpret_cast<const void*>(address), size);
        return MemStatus::Ok;
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        OnTrip();
        return MemStatus::AccessViolation;
    }
}

MemStatus MemoryGuard::WriteBlock(std::uintptr_t address, const void* src, std::size_t size) noexcept
{
    if (address == 0)
        return MemStatus::NullPointer;
    if (src == nullptr || size == 0)
        return MemStatus::InvalidArgument;
    if (!IsRangeWritable(address, size))
        return MemStatus::NotWritable;

    FH4RPO_TRY
    {
        std::memcpy(reinterpret_cast<void*>(address), src, size);
        return MemStatus::Ok;
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        OnTrip();
        return MemStatus::AccessViolation;
    }
}

MemStatus MemoryGuard::WriteFloatAtomic(std::uintptr_t address, float value) noexcept
{
    if (!IsFinite(value))
        return MemStatus::InvalidArgument;
    if (address == 0 || (address & 0x3u) != 0)
        return MemStatus::InvalidArgument;
    if (!IsRangeWritable(address, sizeof(float)))
        return MemStatus::NotWritable;

    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));

    FH4RPO_TRY
    {
        InterlockedExchange(reinterpret_cast<volatile LONG*>(address), static_cast<LONG>(bits));
        return MemStatus::Ok;
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        OnTrip();
        return MemStatus::AccessViolation;
    }
}

float MemoryGuard::ReadFloat(std::uintptr_t address, float fallback) noexcept
{
    float value = fallback;
    if (Read<float>(address, value) == MemStatus::Ok && IsFinite(value))
        return value;
    return fallback;
}

MemStatus MemoryGuard::WriteProtectedCode(std::uintptr_t address, const void* src, std::size_t size) noexcept
{
    if (address == 0 || src == nullptr || size == 0)
        return MemStatus::InvalidArgument;

    DWORD oldProtect = 0;
    if (!VirtualProtect(reinterpret_cast<LPVOID>(address), size,
                        PAGE_EXECUTE_READWRITE, &oldProtect))
        return MemStatus::NotWritable;

    MemStatus status = WriteBlock(address, src, size);

    DWORD restored = 0;
    VirtualProtect(reinterpret_cast<LPVOID>(address), size, oldProtect, &restored);
    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<LPCVOID>(address), size);

    return status;
}

// -----------------------------------------------------------------------------
//  Guarded indirect calls
// -----------------------------------------------------------------------------
MemStatus MemoryGuard::SafeCall0(void* fn, std::uintptr_t& resultOut) noexcept
{
    resultOut = 0;
    if (fn == nullptr)
        return MemStatus::NullPointer;
    if (!IsRangeExecutable(reinterpret_cast<std::uintptr_t>(fn), 1))
        return MemStatus::NotExecutable;

    FH4RPO_TRY
    {
        using Fn = void* (*)();
        resultOut = reinterpret_cast<std::uintptr_t>(reinterpret_cast<Fn>(fn)());
        return MemStatus::Ok;
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        OnTrip();
        return MemStatus::AccessViolation;
    }
}

MemStatus MemoryGuard::SafeCall1(void* fn, void* a0, std::uintptr_t& resultOut) noexcept
{
    resultOut = 0;
    if (fn == nullptr)
        return MemStatus::NullPointer;
    if (!IsRangeExecutable(reinterpret_cast<std::uintptr_t>(fn), 1))
        return MemStatus::NotExecutable;

    FH4RPO_TRY
    {
        using Fn = void* (*)(void*);
        resultOut = reinterpret_cast<std::uintptr_t>(reinterpret_cast<Fn>(fn)(a0));
        return MemStatus::Ok;
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        OnTrip();
        return MemStatus::AccessViolation;
    }
}

MemStatus MemoryGuard::SafeCall2(void* fn, void* a0, void* a1, std::uintptr_t& resultOut) noexcept
{
    resultOut = 0;
    if (fn == nullptr)
        return MemStatus::NullPointer;
    if (!IsRangeExecutable(reinterpret_cast<std::uintptr_t>(fn), 1))
        return MemStatus::NotExecutable;

    FH4RPO_TRY
    {
        using Fn = void* (*)(void*, void*);
        resultOut = reinterpret_cast<std::uintptr_t>(reinterpret_cast<Fn>(fn)(a0, a1));
        return MemStatus::Ok;
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        OnTrip();
        return MemStatus::AccessViolation;
    }
}

// -----------------------------------------------------------------------------
//  Accounting
// -----------------------------------------------------------------------------
void MemoryGuard::OnTrip() noexcept
{
    const int count = g_tripCount.fetch_add(1, std::memory_order_relaxed) + 1;
    if (count <= 5 || (count % 50) == 0)
        FH4RPO_LOG_WARN("MemoryGuard: guard trip #%d", count);
}

int MemoryGuard::TripCount() noexcept { return g_tripCount.load(std::memory_order_relaxed); }
void MemoryGuard::ResetTrips() noexcept { g_tripCount.store(0, std::memory_order_relaxed); }
std::uint32_t MemoryGuard::LastExceptionCode() noexcept { return g_lastException.load(std::memory_order_relaxed); }
bool MemoryGuard::PanicLatched() noexcept { return g_panicLatched.load(std::memory_order_relaxed); }

// -----------------------------------------------------------------------------
//  Watchdog
//
//  Installs an unhandled-exception filter for the whole process.  It only acts
//  when the faulting stack belongs to our module: in that case the game is
//  kept alive and the module switches to passthrough.  Any fault originating
//  in the game itself is passed through untouched (we do not mask the game's
//  own crash handling / reporting).
// -----------------------------------------------------------------------------
bool MemoryGuard::InstallPanicWatchdog() noexcept
{
    if (g_watchdogInstalled.exchange(true, std::memory_order_relaxed))
        return true;

    const LPTOP_LEVEL_EXCEPTION_FILTER previous = SetUnhandledExceptionFilter(PanicFilter);
    if (previous == nullptr)
    {
        g_watchdogInstalled.store(false, std::memory_order_relaxed);
        FH4RPO_LOG_WARN("MemoryGuard: SetUnhandledExceptionFilter returned null");
        return false;
    }

#ifdef _MSC_VER
    // Route CRT level failures (pure virtual calls, invalid parameters) through
    // the same accounting so they cannot pop an abort dialog over the game.
    // These entry points only exist in the Microsoft CRT.
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
    _set_purecall_handler([]()
    {
        OnTrip();
        g_panicLatched.store(true, std::memory_order_relaxed);
    });
#endif

    FH4RPO_LOG_INFO("MemoryGuard: panic watchdog installed");
    return true;
}

void MemoryGuard::RemovePanicWatchdog() noexcept
{
    if (!g_watchdogInstalled.exchange(false, std::memory_order_relaxed))
        return;
    SetUnhandledExceptionFilter(nullptr);
    FH4RPO_LOG_INFO("MemoryGuard: panic watchdog removed");
}

void MemoryGuard::InitializeProcessWide() noexcept
{
    g_tripCount.store(0, std::memory_order_relaxed);
    g_panicLatched.store(false, std::memory_order_relaxed);
    InstallPanicWatchdog();
}

void MemoryGuard::ShutdownProcessWide() noexcept
{
    RemovePanicWatchdog();
}

bool MemoryGuard::EnableDebugPrivilege() noexcept
{
    HANDLE token = nullptr;
    if (!OpenProcessToken(GetCurrentProcess(), TOKEN_ADJUST_PRIVILEGES | TOKEN_QUERY, &token))
        return false;

    LUID luid{};
    if (!LookupPrivilegeValueA(nullptr, SE_DEBUG_NAME, &luid))
    {
        CloseHandle(token);
        return false;
    }

    TOKEN_PRIVILEGES privileges{};
    privileges.PrivilegeCount = 1;
    privileges.Privileges[0].Luid = luid;
    privileges.Privileges[0].Attributes = SE_PRIVILEGE_ENABLED;

    const BOOL ok = AdjustTokenPrivileges(token, FALSE, &privileges, sizeof(privileges), nullptr, nullptr);
    const DWORD error = GetLastError();
    CloseHandle(token);

    if (!ok || error == ERROR_NOT_ALL_ASSIGNED)
        FH4RPO_LOG_WARN("MemoryGuard: SeDebugPrivilege unavailable (err %lu)", error);
    return ok && error != ERROR_NOT_ALL_ASSIGNED;
}

std::uintptr_t MemoryGuard::FindModuleBase(const char* moduleName) noexcept
{
    if (moduleName == nullptr || moduleName[0] == '\0')
        return 0;
    const HMODULE handle = GetModuleHandleA(moduleName);
    return reinterpret_cast<std::uintptr_t>(handle);
}

std::size_t MemoryGuard::ModuleSize(const char* moduleName) noexcept
{
    const std::uintptr_t base = FindModuleBase(moduleName);
    if (base == 0)
        return 0;

    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    if (dos->e_magic != IMAGE_DOS_SIGNATURE)
        return 0;

    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(base + static_cast<std::uintptr_t>(dos->e_lfanew));
    if (nt->Signature != IMAGE_NT_SIGNATURE)
        return 0;

    return static_cast<std::size_t>(nt->OptionalHeader.SizeOfImage);
}

#else  // ---------------------------------------------------------------- portable

// Host build (unit tests, CI): no SEH, no MMU introspection.  The API is
// identical so the physics kernel compiles unchanged; the write-back paths are
// exercised against buffers owned by the test harness.

namespace {
std::atomic<int>           g_tripCount{ 0 };
std::atomic<std::uint32_t> g_lastException{ 0 };
std::atomic<bool>          g_panicLatched{ false };
}

bool MemoryGuard::Query(std::uintptr_t address, MemRegionInfo& out) noexcept
{
    out = MemRegionInfo{};
    if (address == 0)
        return false;
    out.base = address;
    out.size = 4096;
    out.committed = true;
    out.readable = true;
    out.writable = true;
    out.executable = true;
    return true;
}

bool MemoryGuard::IsRangeReadable(std::uintptr_t address, std::size_t size) noexcept
{
    return address != 0 && size != 0;
}

bool MemoryGuard::IsRangeWritable(std::uintptr_t address, std::size_t size) noexcept
{
    return address != 0 && size != 0;
}

bool MemoryGuard::IsRangeExecutable(std::uintptr_t address, std::size_t size) noexcept
{
    return address != 0 && size != 0;
}

MemStatus MemoryGuard::ReadBlock(std::uintptr_t address, void* dst, std::size_t size) noexcept
{
    if (address == 0) return MemStatus::NullPointer;
    if (dst == nullptr || size == 0) return MemStatus::InvalidArgument;
    std::memcpy(dst, reinterpret_cast<const void*>(address), size);
    return MemStatus::Ok;
}

MemStatus MemoryGuard::WriteBlock(std::uintptr_t address, const void* src, std::size_t size) noexcept
{
    if (address == 0) return MemStatus::NullPointer;
    if (src == nullptr || size == 0) return MemStatus::InvalidArgument;
    std::memcpy(reinterpret_cast<void*>(address), src, size);
    return MemStatus::Ok;
}

MemStatus MemoryGuard::WriteFloatAtomic(std::uintptr_t address, float value) noexcept
{
    if (!IsFinite(value)) return MemStatus::InvalidArgument;
    if (address == 0) return MemStatus::NullPointer;
    std::memcpy(reinterpret_cast<void*>(address), &value, sizeof(float));
    return MemStatus::Ok;
}

float MemoryGuard::ReadFloat(std::uintptr_t address, float fallback) noexcept
{
    if (address == 0) return fallback;
    float value = fallback;
    std::memcpy(&value, reinterpret_cast<const void*>(address), sizeof(float));
    return IsFinite(value) ? value : fallback;
}

MemStatus MemoryGuard::WriteProtectedCode(std::uintptr_t address, const void* src, std::size_t size) noexcept
{
    return WriteBlock(address, src, size);
}

MemStatus MemoryGuard::SafeCall0(void* fn, std::uintptr_t& resultOut) noexcept
{
    resultOut = 0;
    if (fn == nullptr) return MemStatus::NullPointer;
    using Fn = void* (*)();
    resultOut = reinterpret_cast<std::uintptr_t>(reinterpret_cast<Fn>(fn)());
    return MemStatus::Ok;
}

MemStatus MemoryGuard::SafeCall1(void* fn, void* a0, std::uintptr_t& resultOut) noexcept
{
    resultOut = 0;
    if (fn == nullptr) return MemStatus::NullPointer;
    using Fn = void* (*)(void*);
    resultOut = reinterpret_cast<std::uintptr_t>(reinterpret_cast<Fn>(fn)(a0));
    return MemStatus::Ok;
}

MemStatus MemoryGuard::SafeCall2(void* fn, void* a0, void* a1, std::uintptr_t& resultOut) noexcept
{
    resultOut = 0;
    if (fn == nullptr) return MemStatus::NullPointer;
    using Fn = void* (*)(void*, void*);
    resultOut = reinterpret_cast<std::uintptr_t>(reinterpret_cast<Fn>(fn)(a0, a1));
    return MemStatus::Ok;
}

void MemoryGuard::OnTrip() noexcept { g_tripCount.fetch_add(1, std::memory_order_relaxed); }
int  MemoryGuard::TripCount() noexcept { return g_tripCount.load(std::memory_order_relaxed); }
void MemoryGuard::ResetTrips() noexcept { g_tripCount.store(0, std::memory_order_relaxed); }
std::uint32_t MemoryGuard::LastExceptionCode() noexcept { return g_lastException.load(std::memory_order_relaxed); }
bool MemoryGuard::PanicLatched() noexcept { return g_panicLatched.load(std::memory_order_relaxed); }
bool MemoryGuard::InstallPanicWatchdog() noexcept { return true; }
void MemoryGuard::RemovePanicWatchdog() noexcept {}
void MemoryGuard::InitializeProcessWide() noexcept {}
void MemoryGuard::ShutdownProcessWide() noexcept {}
bool MemoryGuard::EnableDebugPrivilege() noexcept { return false; }
std::uintptr_t MemoryGuard::FindModuleBase(const char* /*moduleName*/) noexcept { return 0; }
std::size_t MemoryGuard::ModuleSize(const char* /*moduleName*/) noexcept { return 0; }

#endif  // _WIN32

// -----------------------------------------------------------------------------
//  ScopedGuard
// -----------------------------------------------------------------------------
ScopedGuard::ScopedGuard(const char* name) noexcept : name_(name ? name : "unnamed"), failed_(false) {}

ScopedGuard::~ScopedGuard() noexcept
{
    if (failed_)
        MemoryGuard::OnTrip();
}

void ScopedGuard::Fail(MemStatus status) noexcept
{
    if (status == MemStatus::Ok)
        return;
    failed_ = true;
    FH4RPO_LOG_DEBUG("ScopedGuard(%s): operation failed with status %d", name_, static_cast<int>(status));
}

} // namespace fh4rpo
