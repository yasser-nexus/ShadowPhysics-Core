// =============================================================================
//  FH4RPO - MemoryGuard.hpp
//
//  Zero-Crash architecture, layer 1.
//
//    * Every read / write made on behalf of the game goes through SafeRead /
//      SafeWrite, which are wrapped in __try / __except (SEH) on toolchains
//      that support structured exception handling.
//    * Region validation (VirtualQuery + protection flags) happens *before*
//      the guarded access, so protected pages are rejected without ever
//      raising an exception.
//    * A process wide watchdog keeps the game alive if a fault does escape,
//      and latches a panic flag so the module can go passive.
//    * Trip counters let the module degrade instead of hammering a dead
//      pointer every frame.
//
//  On the portable (host / unit test) build FH4RPO_SEH is 0: the same API is
//  provided without SEH so the physics kernel can be tested on any OS.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "Core.hpp"

#ifdef _WIN32
#  include <windows.h>      // EXCEPTION_EXECUTE_HANDLER for the SEH guards
#endif

// MSVC and clang-cl (the mainstream toolchains for a Windows DLL) implement
// SEH natively.  MinGW / GCC does not - the portable try/catch path is used
// there instead, which still keeps the module from taking the game down.
#if defined(_WIN32) && (defined(_MSC_VER) || defined(__clang__))
#  define FH4RPO_SEH 1
#else
#  define FH4RPO_SEH 0
#endif

// Portable versions of the SEH keywords.
//   * On Windows with MSVC / clang-cl these expand to the real __try/__except,
//     which is what stops a bad pointer from killing the game.
//   * On the host build (unit tests) they degrade to C++ try/catch, so the same
//     source compiles everywhere and the guards are still exercised.
#if FH4RPO_SEH
#  define FH4RPO_TRY                  __try
#  define FH4RPO_EXCEPT(handler)      __except (handler)
#else
#  define FH4RPO_TRY                  try
#  define FH4RPO_EXCEPT(handler)      catch (...)
#endif

namespace fh4rpo {

enum class MemStatus : int
{
    Ok = 0,
    NotReadable,
    NotWritable,
    NotExecutable,
    NullPointer,
    AccessViolation,
    Guarded,
    InvalidArgument
};

struct MemRegionInfo
{
    std::uintptr_t base       = 0;
    std::size_t    size       = 0;
    unsigned long  protect    = 0;
    unsigned long  state      = 0;
    unsigned long  type       = 0;
    bool           readable   = false;
    bool           writable   = false;
    bool           executable = false;
    bool           committed  = false;
};

class MemoryGuard
{
public:
    static void InitializeProcessWide() noexcept;
    static void ShutdownProcessWide() noexcept;

    // -------- region introspection -------------------------------------------
    static bool Query(std::uintptr_t address, MemRegionInfo& out) noexcept;
    static bool IsRangeReadable(std::uintptr_t address, std::size_t size) noexcept;
    static bool IsRangeWritable(std::uintptr_t address, std::size_t size) noexcept;
    static bool IsRangeExecutable(std::uintptr_t address, std::size_t size) noexcept;

    // -------- guarded primitives ---------------------------------------------
    template <typename T>
    static MemStatus Read(std::uintptr_t address, T& out) noexcept;

    template <typename T>
    static MemStatus Write(std::uintptr_t address, const T& value) noexcept;

    static MemStatus ReadBlock(std::uintptr_t address, void* dst, std::size_t size) noexcept;
    static MemStatus WriteBlock(std::uintptr_t address, const void* src, std::size_t size) noexcept;

    // In-place 32 bit float patch using an atomic exchange so the value can
    // never tear while the game's physics thread reads it.
    static MemStatus WriteFloatAtomic(std::uintptr_t address, float value) noexcept;
    static float     ReadFloat(std::uintptr_t address, float fallback = 0.0f) noexcept;

    // Temporarily flips page protection, writes, restores it, flushes I-cache.
    static MemStatus WriteProtectedCode(std::uintptr_t address, const void* src, std::size_t size) noexcept;

    // Calls a game function under SEH.  Results are only returned when the call
    // completed without a structured exception.
    static MemStatus SafeCall0(void* fn, std::uintptr_t& resultOut) noexcept;
    static MemStatus SafeCall1(void* fn, void* a0, std::uintptr_t& resultOut) noexcept;
    static MemStatus SafeCall2(void* fn, void* a0, void* a1, std::uintptr_t& resultOut) noexcept;

    // -------- guard accounting -----------------------------------------------
    static void OnTrip() noexcept;
    static int  TripCount() noexcept;
    static void ResetTrips() noexcept;
    static std::uint32_t LastExceptionCode() noexcept;
    static bool PanicLatched() noexcept;
    static bool InstallPanicWatchdog() noexcept;
    static void RemovePanicWatchdog() noexcept;

    static std::uintptr_t FindModuleBase(const char* moduleName) noexcept;
    static std::size_t    ModuleSize(const char* moduleName) noexcept;
    static bool           EnableDebugPrivilege() noexcept;
};

// -----------------------------------------------------------------------------
//  Read / Write templates.
//
//  Kept in the header so they inline per type; the SEH frames are opaque to the
//  optimiser because the helpers are declared noinline.
// -----------------------------------------------------------------------------
template <typename T>
FH4RPO_NOINLINE MemStatus MemoryGuard::Read(std::uintptr_t address, T& out) noexcept
{
    if (address == 0)
        return MemStatus::NullPointer;
    if (!IsRangeReadable(address, sizeof(T)))
        return MemStatus::NotReadable;

#if FH4RPO_SEH
    __try
    {
        volatile const T* src = reinterpret_cast<volatile const T*>(address);
        out = *src;
        return MemStatus::Ok;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        OnTrip();
        return MemStatus::AccessViolation;
    }
#else
    std::memcpy(&out, reinterpret_cast<const void*>(address), sizeof(T));
    return MemStatus::Ok;
#endif
}

template <typename T>
FH4RPO_NOINLINE MemStatus MemoryGuard::Write(std::uintptr_t address, const T& value) noexcept
{
    if (address == 0)
        return MemStatus::NullPointer;
    if (!IsRangeWritable(address, sizeof(T)))
        return MemStatus::NotWritable;

#if FH4RPO_SEH
    __try
    {
        volatile T* dst = reinterpret_cast<volatile T*>(address);
        *dst = value;
        return MemStatus::Ok;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        OnTrip();
        return MemStatus::AccessViolation;
    }
#else
    std::memcpy(reinterpret_cast<void*>(address), &value, sizeof(T));
    return MemStatus::Ok;
#endif
}

// -----------------------------------------------------------------------------
//  ScopedGuard - counts entry / exit and reports failed sections.
// -----------------------------------------------------------------------------
class ScopedGuard
{
public:
    explicit ScopedGuard(const char* name) noexcept;
    ~ScopedGuard() noexcept;
    void Fail(MemStatus status) noexcept;

private:
    const char* name_;
    bool        failed_;
};

} // namespace fh4rpo
