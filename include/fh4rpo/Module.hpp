// =============================================================================
//  FH4RPO - Module.hpp
//
//  Module lifetime, hook registration, input handling, watchdog.
//  This is the window-free controller the user asked for; the entry point in
//  dllmain.cpp is a thin wrapper around Attach()/Detach().
// =============================================================================
#pragma once

#include <cstdint>
#include <windows.h>
#include "Config.hpp"
#include "PhysicsEngine.hpp"

namespace fh4rpo {

// Snapshot of the hosting process passed down from DllMain so the module never
// has to call the loader API from inside the loader lock.
struct DllMainContext
{
    bool  processAttach = true;
    bool  processDetach = false;
    bool  threadAttach  = false;
    bool  threadDetach  = false;
    void* reserved      = nullptr;
};

class Module
{
public:
    static Module& Instance() noexcept;

    // Called from DllMain(DLL_PROCESS_ATTACH). Must NOT create threads or wait
    // on synchronisation objects (loader lock) - it only records the module
    // handle and spawns the worker that does the real work.
    bool Attach(HMODULE moduleHandle, const DllMainContext& context) noexcept;

    // Called from DllMain(DLL_PROCESS_DETACH): full teardown, auto-unhook.
    void Detach(bool processTerminating) noexcept;

    // Full initialisation - runs on the worker thread.
    bool StartUp() noexcept;
    void ShutDown() noexcept;

    // Hook registration - called once during StartUp().
    bool InstallHooks() noexcept;
    void RemoveHooks() noexcept;

    // Background loop: hotkey polling, config watching, watchdog, overlay.
    void WorkerLoop() noexcept;
    void RequestStop() noexcept;

    HMODULE Handle() const noexcept { return module_; }
    PhysicsEngine& Engine() noexcept { return engine_; }
    bool IsRunning() const noexcept { return running_.load(); }

private:
    Module() noexcept;
    ~Module() noexcept;
    Module(const Module&) = delete;
    Module& operator=(const Module&) = delete;

    void HandleHotkeys() noexcept;
    void HandlePanic() noexcept;
    void WriteStateDump() noexcept;
    void UpdateOverlay() noexcept;

    HMODULE       module_ = nullptr;
    PhysicsEngine engine_{};
    std::atomic<bool> started_{ false };
    std::atomic<bool> running_{ false };
    std::atomic<bool> stopRequested_{ false };
    std::uint64_t lastHotkeyPollMs_ = 0;
    std::uint64_t lastWatchdogMs_ = 0;
};

// Snapshot of the hosting process passed down from DllMain so the module never
// has to call the loader API from inside the loader lock.
} // namespace fh4rpo
