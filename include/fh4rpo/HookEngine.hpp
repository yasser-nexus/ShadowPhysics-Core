// =============================================================================
//  FH4RPO - HookEngine.hpp
//
//  Zero-Crash architecture, layer 2: a thin, self-cleaning wrapper around
//  MinHook.
//
//    * Every hook is owned by a registered HookDescriptor; the engine can
//      enable / disable / destroy it deterministically.
//    * Shutdown() walks the registry in reverse creation order, disables then
//      removes every trampoline, and finally calls MH_Uninitialize() - so no
//      pointer is ever left dangling inside the game's code after the module
//      is unloaded (auto-unhook, also invoked from DllMain(DLL_PROCESS_DETACH)).
//    * A module whitelist prevents accidental hooking of a foreign / stale
//      address after a game update: the target must live inside the module we
//      expect, and its first bytes must not be an already-installed detour.
//    * Guard flags: each hop body runs under SEH and can be switched off at
//      runtime (panic switch) without removing the trampoline.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include "Core.hpp"

namespace fh4rpo {

enum class HookStatus : int
{
    Ok = 0,
    NotInitialized,
    AlreadyExists,
    InvalidTarget,
    WrongModule,
    TargetNotExecutable,
    MinHookError,
    Disabled,
    Failed
};

struct HookDescriptor
{
    const char*    name          = "hook";
    std::uintptr_t target        = 0;
    void*          detour        = nullptr;
    void**         originalOut   = nullptr;
    const wchar_t* requiredModule = nullptr;  // nullptr = no module constraint
    bool           enabled       = false;
    bool           created       = false;
    int            callCount     = 0;
    int            tripCount     = 0;
};

class HookEngine
{
public:
    static HookEngine& Instance() noexcept;

    bool Initialize() noexcept;
    void Shutdown() noexcept;          // auto-unhook: safe to call many times
    bool IsInitialized() const noexcept;

    // Apply every descriptor registered with Register().
    HookStatus Add(const char* name,
                   std::uintptr_t target,
                   void* detour,
                   void** originalOut,
                   const wchar_t* requiredModule = nullptr) noexcept;

    HookStatus Enable(const char* name) noexcept;
    HookStatus Disable(const char* name) noexcept;
    HookStatus Remove(const char* name) noexcept;

    void* Trampoline(const char* name) const noexcept;
    bool  IsEnabled(const char* name) const noexcept;

    // Global panic switch: keeps trampolines installed but every hop body runs
    // in passthrough mode (game behaves as if the module were not there).
    void SetPassthrough(bool on) noexcept;
    bool Passthrough() const noexcept;
    void NotifyTrip(const char* name) noexcept;

    // Returns true when a hop body should execute; increments the call counter.
    bool ShouldRunHop(const char* name) noexcept;

    int  ActiveCount() const noexcept;
    int  TotalCount() const noexcept;
    void DumpStatus() const noexcept;

    static HookStatus FromMinHook(int status) noexcept;

private:
    HookEngine() = default;
    ~HookEngine() = default;
    HookEngine(const HookEngine&) = delete;
    HookEngine& operator=(const HookEngine&) = delete;

    HookDescriptor* Find(const char* name) noexcept;
    const HookDescriptor* Find(const char* name) const noexcept;

    static constexpr int kMaxHooks = 32;

    HookDescriptor hooks_[kMaxHooks] = {};
    int            hookCount_ = 0;
    bool           initialized_ = false;
    std::atomic<bool> passthrough_{ false };
    std::atomic<int>  totalTrips_{ 0 };
};

// -----------------------------------------------------------------------------
//  Hop guards
//
//  Usage at the top of every detour:
//
//      float __fastcall Detour(void* self, void*, int index, float slip, float load)
//      {
//          if (!FH4RPO_HOP_ALLOWED(hook_friction_matrix))
//              return g_originalFriction(self, nullptr, index, slip, load);
//
//          FH4RPO_TRY { ...body... }
//          FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER) { HopTrip("hook_friction_matrix"); }
//          return fallback;
//      }
//
//  In passthrough mode the detour simply forwards the original arguments, so
//  the game behaves exactly as if the module were not loaded.
// -----------------------------------------------------------------------------
bool HopAllowed(const char* name) noexcept;
void HopTrip(const char* name) noexcept;

} // namespace fh4rpo

#define FH4RPO_HOP_ALLOWED(hookName) (::fh4rpo::HopAllowed(#hookName))
