// =============================================================================
//  FH4RPO - HookEngine.cpp
//
//  Safe, self-cleaning MinHook wrapper.
// =============================================================================
#include "fh4rpo/HookEngine.hpp"
#include "fh4rpo/MemoryGuard.hpp"
#include "fh4rpo/Log.hpp"

#include <windows.h>
#include <cstring>
#include <atomic>
#include "MinHook.h"

namespace fh4rpo {

namespace {

// Recognisable prologue bytes of a detour we (or another hooking library)
// already placed.  A jmp rel32 / jmp rel8 at offset 0 means somebody owns this
// address - we refuse to create a second trampoline on top of it.
bool LooksAlreadyHooked(std::uintptr_t target) noexcept
{
    std::uint8_t prologue[4] = {};
    if (MemoryGuard::ReadBlock(target, prologue, sizeof(prologue)) != MemStatus::Ok)
        return true;

    if (prologue[0] == 0xE9 || prologue[0] == 0xEB)      // jmp rel32 / rel8
        return true;
    if (prologue[0] == 0xFF && (prologue[1] & 0x38) == 0x20) // jmp [mem]
        return true;
    if (prologue[0] == 0xCC && prologue[1] == 0xCC)          // int3 padding / already patched
        return true;
    return false;
}

} // namespace

HookEngine& HookEngine::Instance() noexcept
{
    static HookEngine engine;
    return engine;
}

HookStatus HookEngine::FromMinHook(int status) noexcept
{
    switch (status)
    {
    case MH_OK:                 return HookStatus::Ok;
    case MH_ERROR_ALREADY_INITIALIZED: return HookStatus::Ok;
    case MH_ERROR_NOT_INITIALIZED:     return HookStatus::NotInitialized;
    case MH_ERROR_ALREADY_CREATED:     return HookStatus::AlreadyExists;
    case MH_ERROR_NOT_EXECUTABLE:      return HookStatus::TargetNotExecutable;
    default:                           return HookStatus::MinHookError;
    }
}

bool HookEngine::Initialize() noexcept
{
    if (initialized_)
        return true;

    const MH_STATUS status = MH_Initialize();
    if (status != MH_OK && status != MH_ERROR_ALREADY_INITIALIZED)
    {
        FH4RPO_LOG_ERROR("HookEngine: MH_Initialize failed (%d)", static_cast<int>(status));
        return false;
    }

    initialized_ = true;
    FH4RPO_LOG_INFO("HookEngine: MinHook initialised");
    return true;
}

HookStatus HookEngine::Add(const char* name,
                           std::uintptr_t target,
                           void* detour,
                           void** originalOut,
                           const wchar_t* requiredModule) noexcept
{
    if (!initialized_)
        return HookStatus::NotInitialized;
    if (name == nullptr || detour == nullptr || originalOut == nullptr || target == 0)
    {
        FH4RPO_LOG_ERROR("HookEngine: invalid descriptor (%s)", name ? name : "null");
        return HookStatus::InvalidTarget;
    }
    if (Find(name) != nullptr)
        return HookStatus::AlreadyExists;
    if (hookCount_ >= kMaxHooks)
        return HookStatus::Failed;

    // ---- target validation --------------------------------------------------
    if (!MemoryGuard::IsRangeExecutable(target, 16))
    {
        FH4RPO_LOG_ERROR("HookEngine: %s target 0x%p is not executable", name,
                         reinterpret_cast<void*>(target));
        return HookStatus::TargetNotExecutable;
    }

    if (requiredModule != nullptr)
    {
        char moduleName[64] = {};
        WideCharToMultiByte(CP_UTF8, 0, requiredModule, -1, moduleName, sizeof(moduleName) - 1, nullptr, nullptr);

        const std::uintptr_t base = MemoryGuard::FindModuleBase(moduleName);
        if (base == 0)
        {
            FH4RPO_LOG_ERROR("HookEngine: %s requires module %s (not loaded)", name, moduleName);
            return HookStatus::WrongModule;
        }
        const std::size_t size = MemoryGuard::ModuleSize(moduleName);
        if (size > 0 && (target < base || target >= base + size))
        {
            FH4RPO_LOG_ERROR("HookEngine: %s target 0x%p lies outside %s", name,
                             reinterpret_cast<void*>(target), moduleName);
            return HookStatus::WrongModule;
        }
    }

    if (LooksAlreadyHooked(target))
    {
        FH4RPO_LOG_WARN("HookEngine: %s target 0x%p already detoured, refusing", name,
                        reinterpret_cast<void*>(target));
        return HookStatus::AlreadyExists;
    }

    const MH_STATUS created = MH_CreateHook(reinterpret_cast<LPVOID>(target), detour, originalOut);
    if (created != MH_OK)
    {
        FH4RPO_LOG_ERROR("HookEngine: MH_CreateHook(%s) failed (%d)", name, static_cast<int>(created));
        return FromMinHook(created);
    }

    HookDescriptor& hook = hooks_[hookCount_++];
    hook.name            = name;
    hook.target          = target;
    hook.detour          = detour;
    hook.originalOut     = originalOut;
    hook.requiredModule  = requiredModule;
    hook.created         = true;
    hook.enabled         = false;
    hook.callCount       = 0;
    hook.tripCount       = 0;

    FH4RPO_LOG_INFO("HookEngine: created %s at 0x%p", name, reinterpret_cast<void*>(target));

    const MH_STATUS enabled = MH_EnableHook(reinterpret_cast<LPVOID>(target));
    if (enabled != MH_OK)
    {
        FH4RPO_LOG_ERROR("HookEngine: MH_EnableHook(%s) failed (%d)", name, static_cast<int>(enabled));
        return FromMinHook(enabled);
    }
    hook.enabled = true;
    return HookStatus::Ok;
}

HookDescriptor* HookEngine::Find(const char* name) noexcept
{
    if (name == nullptr)
        return nullptr;
    for (int i = 0; i < hookCount_; ++i)
    {
        if (std::strcmp(hooks_[i].name, name) == 0)
            return &hooks_[i];
    }
    return nullptr;
}

const HookDescriptor* HookEngine::Find(const char* name) const noexcept
{
    return const_cast<HookEngine*>(this)->Find(name);
}

HookStatus HookEngine::Enable(const char* name) noexcept
{
    HookDescriptor* hook = Find(name);
    if (hook == nullptr || !hook->created)
        return HookStatus::Failed;
    if (hook->enabled)
        return HookStatus::Ok;

    const MH_STATUS status = MH_EnableHook(reinterpret_cast<LPVOID>(hook->target));
    if (status != MH_OK)
        return FromMinHook(status);
    hook->enabled = true;
    return HookStatus::Ok;
}

HookStatus HookEngine::Disable(const char* name) noexcept
{
    HookDescriptor* hook = Find(name);
    if (hook == nullptr || !hook->created)
        return HookStatus::Failed;
    if (!hook->enabled)
        return HookStatus::Ok;

    const MH_STATUS status = MH_DisableHook(reinterpret_cast<LPVOID>(hook->target));
    if (status != MH_OK)
        return FromMinHook(status);
    hook->enabled = false;
    return HookStatus::Ok;
}

HookStatus HookEngine::Remove(const char* name) noexcept
{
    HookDescriptor* hook = Find(name);
    if (hook == nullptr || !hook->created)
        return HookStatus::Failed;

    MH_DisableHook(reinterpret_cast<LPVOID>(hook->target));
    const MH_STATUS removed = MH_RemoveHook(reinterpret_cast<LPVOID>(hook->target));
    if (removed != MH_OK)
    {
        FH4RPO_LOG_ERROR("HookEngine: MH_RemoveHook(%s) failed (%d)", name, static_cast<int>(removed));
        return FromMinHook(removed);
    }

    hook->created = false;
    hook->enabled = false;
    FH4RPO_LOG_INFO("HookEngine: removed %s", name);
    return HookStatus::Ok;
}

void HookEngine::Shutdown() noexcept
{
    if (!initialized_)
        return;

    // Reverse order teardown - hooks created later may depend on earlier ones.
    for (int i = hookCount_ - 1; i >= 0; --i)
    {
        HookDescriptor& hook = hooks_[i];
        if (!hook.created)
            continue;

        const MH_STATUS disabled = MH_DisableHook(reinterpret_cast<LPVOID>(hook.target));
        if (disabled != MH_OK)
            FH4RPO_LOG_WARN("HookEngine: MH_DisableHook(%s) -> %d", hook.name, static_cast<int>(disabled));

        const MH_STATUS removed = MH_RemoveHook(reinterpret_cast<LPVOID>(hook.target));
        if (removed != MH_OK)
            FH4RPO_LOG_WARN("HookEngine: MH_RemoveHook(%s) -> %d", hook.name, static_cast<int>(removed));

        if (hook.originalOut != nullptr)
            *hook.originalOut = nullptr;      // never leave a dangling stub pointer

        FH4RPO_LOG_INFO("HookEngine: unhooked %s", hook.name);
        hook.created = false;
        hook.enabled = false;
    }

    hookCount_ = 0;
    MH_Uninitialize();
    initialized_ = false;
    FH4RPO_LOG_INFO("HookEngine: shut down, no trampolines left behind");
}

bool HookEngine::IsInitialized() const noexcept { return initialized_; }

void* HookEngine::Trampoline(const char* name) const noexcept
{
    const HookDescriptor* hook = Find(name);
    if (hook == nullptr || hook->originalOut == nullptr)
        return nullptr;
    return *hook->originalOut;
}

bool HookEngine::IsEnabled(const char* name) const noexcept
{
    const HookDescriptor* hook = Find(name);
    return hook != nullptr && hook->enabled;
}

void HookEngine::SetPassthrough(bool on) noexcept
{
    if (passthrough_.exchange(on, std::memory_order_relaxed) == on)
        return;
    FH4RPO_LOG_WARN("HookEngine: passthrough mode %s", on ? "ENABLED" : "disabled");
}

bool HookEngine::Passthrough() const noexcept { return passthrough_.load(std::memory_order_relaxed); }

void HookEngine::NotifyTrip(const char* name) noexcept
{
    HookDescriptor* hook = Find(name);
    if (hook != nullptr)
        hook->tripCount++;
    const int total = totalTrips_.fetch_add(1, std::memory_order_relaxed) + 1;
    FH4RPO_LOG_WARN("HookEngine: hop %s tripped (total %d)", name ? name : "?", total);
}

int HookEngine::ActiveCount() const noexcept
{
    int count = 0;
    for (int i = 0; i < hookCount_; ++i)
        if (hooks_[i].created && hooks_[i].enabled)
            ++count;
    return count;
}

int HookEngine::TotalCount() const noexcept { return hookCount_; }

void HookEngine::DumpStatus() const noexcept
{
    FH4RPO_LOG_INFO("---- Hook status (%d/%d active, passthrough=%s) ----",
                    ActiveCount(), hookCount_, Passthrough() ? "on" : "off");
    for (int i = 0; i < hookCount_; ++i)
    {
        const HookDescriptor& hook = hooks_[i];
        FH4RPO_LOG_INFO("  [%d] %-24s target=0x%p enabled=%d calls=%d trips=%d",
                        i, hook.name, reinterpret_cast<void*>(hook.target),
                        hook.enabled ? 1 : 0, hook.callCount, hook.tripCount);
    }
}

// -----------------------------------------------------------------------------
//  Hop guards
// -----------------------------------------------------------------------------
bool HookEngine::ShouldRunHop(const char* name) noexcept
{
    if (passthrough_.load(std::memory_order_relaxed))
        return false;

    HookDescriptor* hook = Find(name);
    if (hook != nullptr)
        hook->callCount++;

    return true;
}

bool HopAllowed(const char* name) noexcept
{
    return HookEngine::Instance().ShouldRunHop(name);
}

void HopTrip(const char* name) noexcept
{
    HookEngine::Instance().NotifyTrip(name);
    MemoryGuard::OnTrip();
}

} // namespace fh4rpo
