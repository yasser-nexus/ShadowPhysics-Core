// =============================================================================
//  FH4RPO - Module.cpp
//
//  Lifetime, hook wiring, hotkeys, watchdog, overlay.
// =============================================================================
#include "fh4rpo/Module.hpp"
#include "fh4rpo/Log.hpp"
#include "fh4rpo/GameBinds.hpp"
#include "fh4rpo/HookEngine.hpp"
#include "fh4rpo/Telemetry.hpp"

#include <windows.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>

namespace fh4rpo {

// Implemented in PhysicsEngine.cpp
namespace hooks {
    bool InstallAll() noexcept;
    void RemoveAll() noexcept;
}

namespace {

HANDLE          g_workerThread = nullptr;
TelemetryReceiver g_receiver;
std::wstring    g_moduleDirectory;
HMODULE         moduleHandleForWorker = nullptr;

std::wstring GetModuleDirectory(HMODULE module) noexcept
{
    wchar_t path[MAX_PATH] = {};
    if (GetModuleFileNameW(module, path, MAX_PATH) == 0)
        return std::wstring();

    std::wstring full(path);
    const std::size_t slash = full.find_last_of(L"\\/");
    if (slash == std::wstring::npos)
        return std::wstring();
    return full.substr(0, slash + 1);
}

std::string Narrow(const std::wstring& wide) noexcept
{
    if (wide.empty())
        return std::string();
    const int size = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (size <= 1)
        return std::string();
    std::string out(static_cast<std::size_t>(size - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, &out[0], size, nullptr, nullptr);
    return out;
}

std::uint64_t NowMs() noexcept
{
    return static_cast<std::uint64_t>(GetTickCount64());
}

bool KeyPressedOnce(int virtualKey, std::uint64_t& lastPressMs) noexcept
{
    if (virtualKey <= 0)
        return false;

    const bool down = (GetAsyncKeyState(virtualKey) & 0x8000) != 0;
    if (!down)
        return false;

    const std::uint64_t now = NowMs();
    if (now - lastPressMs < 250)
        return false;

    lastPressMs = now;
    return true;
}

} // namespace

Module& Module::Instance() noexcept
{
    static Module module;
    return module;
}

Module::Module() noexcept = default;
Module::~Module() noexcept { ShutDown(); }

// =============================================================================
//  Attach - called from DllMain
// =============================================================================
bool Module::Attach(HMODULE moduleHandle, const DllMainContext& context) noexcept
{
    if (!context.processAttach || moduleHandle == nullptr)
        return false;

    module_ = moduleHandle;
    g_moduleDirectory = GetModuleDirectory(moduleHandle);

    // DllMain runs under the loader lock: no waiting on synchronisation
    // objects, no file I/O that could deadlock, no CRT locks.  The worker
    // thread does the real work; we only spawn it and return.
    g_workerThread = CreateThread(nullptr, 0,
        [](LPVOID parameter) -> DWORD
        {
            moduleHandleForWorker = static_cast<HMODULE>(parameter);
            Module& module = Module::Instance();
            if (!module.StartUp())
            {
                FH4RPO_LOG_ERROR("Module: StartUp failed");
                module.ShutDown();
                return 1;
            }
            module.WorkerLoop();
            module.ShutDown();
            return 0;
        },
        moduleHandle, 0, nullptr);

    if (g_workerThread == nullptr)
        return false;

    CloseHandle(g_workerThread);      // the thread owns itself now
    g_workerThread = nullptr;
    return true;
}

// =============================================================================
//  StartUp
// =============================================================================
bool Module::StartUp() noexcept
{
    if (started_.load(std::memory_order_acquire))
        return true;

    // 1. paths -----------------------------------------------------------------
    Config& config = Config::Instance();
    config.moduleName = "FH4RPO";
    config.iniPath  = Narrow(g_moduleDirectory) + "FH4RPO.ini";
    config.logPath  = Narrow(g_moduleDirectory) + "FH4RPO.log";
    config.stateDumpPath = Narrow(g_moduleDirectory) + "FH4RPO_state.txt";

    // 2. logging ---------------------------------------------------------------
    config.Load(config.iniPath);
    Log::Initialize(config.logPath.c_str(), static_cast<LogLevel>(config.debug.logLevel));

    FH4RPO_LOG_INFO("======================================================");
    FH4RPO_LOG_INFO(" FH4 Realistic Physics Overhaul - build %s %s", __DATE__, __TIME__);
    FH4RPO_LOG_INFO(" module directory: %s", Narrow(g_moduleDirectory).c_str());
    FH4RPO_LOG_INFO(" process: %lu (x64)", static_cast<unsigned long>(GetCurrentProcessId()));
    FH4RPO_LOG_INFO("======================================================");

    // 3. memory guard + watchdog ----------------------------------------------
    if (config.memory.enableDebugPrivilege)
        MemoryGuard::EnableDebugPrivilege();

    MemoryGuard::InitializeProcessWide();

    // 4. binds (address resolution) -------------------------------------------
    GameBinds& binds = GameBinds::Instance();
    binds.Initialize(config);

    // Manual pins from [Addresses] win over every scan.
    struct Pin { const char* name; unsigned long long address; };
    const Pin pins[] = {
        { "hook_friction_matrix", config.memory.pinHookFrictionMatrix },
        { "hook_downforce",       config.memory.pinHookDownforce      },
        { "hook_suspension_solve", config.memory.pinHookSuspension    },
        { "hook_tire_force",      config.memory.pinHookTireForce      },
        { "grip_scale",           config.memory.pinGripScale          },
        { "downforce_scale",      config.memory.pinDownforceScale     },
        { "brake_bias",           config.memory.pinBrakeBias          },
        { "steer_angle",          config.memory.pinSteerAngle         },
        { "friction_matrix",      config.memory.pinFrictionMatrix     },
    };
    for (const Pin& pin : pins)
    {
        if (pin.address != 0)
            binds.SetOverride(pin.name, static_cast<std::uintptr_t>(pin.address));
    }
    binds.Tick(0.0);

    // 5. physics ---------------------------------------------------------------
    engine_.Initialize();
    engine_.RegisterTargets(binds.BuildEngineTargets());

    // 6. telemetry -------------------------------------------------------------
    if (config.telemetry.enabled)
    {
        if (g_receiver.Start(config.telemetry.port, config.telemetry.bindPortFallback,
                             config.telemetry.bindLoopbackOnly))
        {
            engine_.AttachReceiver(&g_receiver);
            g_receiver.SetBroadcast(static_cast<unsigned short>(config.telemetry.broadcastPort),
                                    config.telemetry.broadcastAssist);
            FH4RPO_LOG_INFO("Module: telemetry receiver live on port %d", g_receiver.BoundPort());
        }
        else
        {
            FH4RPO_LOG_WARN("Module: telemetry receiver unavailable - passive mode only");
        }
    }

    // 7. hooks -----------------------------------------------------------------
    InstallHooks();

    running_.store(true, std::memory_order_release);
    started_.store(true, std::memory_order_release);

    FH4RPO_LOG_INFO("Module: startup complete (hooks=%d, binds=%d/%d)",
                    HookEngine::Instance().ActiveCount(),
                    GameBinds::Instance().ResolvedCount(),
                    GameBinds::Instance().TotalCount());
    return true;
}

// =============================================================================
//  ShutDown
// =============================================================================
void Module::ShutDown() noexcept
{
    if (!started_.load(std::memory_order_acquire))
    {
        // Still safe to call: remove whatever may have been partially installed.
        RemoveHooks();
        return;
    }

    started_.store(false, std::memory_order_release);

    running_.store(false, std::memory_order_release);
    stopRequested_.store(true, std::memory_order_release);

    RemoveHooks();

    Config& config = Config::Instance();
    if (!config.stateDumpPath.empty() && config.debug.dumpStateOnCrash)
        WriteStateDump();

    g_receiver.Stop();

    engine_.Shutdown();
    GameBinds::Instance().Shutdown();
    MemoryGuard::ShutdownProcessWide();

    FH4RPO_LOG_INFO("Module: shutdown complete - all memory hooks released");
    Log::Flush();
    Log::Shutdown();
}

// =============================================================================
//  Detach - called from DllMain(DLL_PROCESS_DETACH)
// =============================================================================
void Module::Detach(bool processTerminating) noexcept
{
    stopRequested_.store(true, std::memory_order_release);

    if (processTerminating)
    {
        // The process is exiting: the loader lock is held and other threads may
        // already be gone.  Only release the trampolines - never wait, never
        // allocate, never touch the loader.
        RemoveHooks();
        running_.store(false, std::memory_order_release);
        return;
    }

    // Normal unload (FreeLibrary / ASI unloader): give the worker thread up to
    // ~3 seconds to run its own orderly shutdown, then force the teardown.
    for (int i = 0; i < 300 && started_.load(std::memory_order_acquire); ++i)
        Sleep(10);

    ShutDown();
}

bool Module::InstallHooks() noexcept
{
    return hooks::InstallAll();
}

void Module::RemoveHooks() noexcept
{
    // Idempotent by design: HookEngine::Shutdown() can be called any number of
    // times and always ends with MH_Uninitialize().
    hooks::RemoveAll();
}

void Module::RequestStop() noexcept
{
    stopRequested_.store(true, std::memory_order_release);
}

// =============================================================================
//  Worker loop
// =============================================================================
void Module::WorkerLoop() noexcept
{
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);

    Config& config = Config::Instance();
    std::uint64_t lastPress = 0;
    double lastOverlayWrite = 0.0;
    std::uint64_t lastConfigPoll = 0;

    while (!stopRequested_.load(std::memory_order_acquire))
    {
        // 1. hotkeys -----------------------------------------------------------
        if (KeyPressedOnce(config.keys.toggleMaster, lastPress))
        {
            const bool enabled = !config.masterEnabled.load(std::memory_order_relaxed);
            config.masterEnabled.store(enabled, std::memory_order_relaxed);
            HookEngine::Instance().SetPassthrough(!enabled);
            FH4RPO_LOG_WARN("Module: physics %s (F2)", enabled ? "ENABLED" : "DISABLED");
        }

        if (KeyPressedOnce(config.keys.togglePassive, lastPress))
        {
            const bool passive = !config.passiveMode.load(std::memory_order_relaxed);
            config.passiveMode.store(passive, std::memory_order_relaxed);
            FH4RPO_LOG_WARN("Module: passive mode %s (F3)", passive ? "ON" : "OFF");
        }

        if (KeyPressedOnce(config.keys.reloadConfig, lastPress))
        {
            config.reloadRequested.store(true, std::memory_order_relaxed);
            FH4RPO_LOG_INFO("Module: manual config reload requested (F5)");
            // Rebuild the matrices from the freshly parsed tables.
            config.WatchFile();
            engine_.UpdateVehicleSpec(Config::Instance().vehicle);
        }

        if (KeyPressedOnce(config.keys.panicDisable, lastPress))
        {
            FH4RPO_LOG_WARN("Module: PANIC - disabling all hooks (F4)");
            HookEngine::Instance().SetPassthrough(true);
            Config::Instance().masterEnabled.store(false, std::memory_order_relaxed);
        }

        if (KeyPressedOnce(config.keys.dumpState, lastPress))
        {
            FH4RPO_LOG_INFO("Module: state dump requested (F8)");
            WriteStateDump();
        }

        // 2. automatic config reload (polled at 2 Hz, not every millisecond) --
        const std::uint64_t nowMs = NowMs();
        if (nowMs - lastConfigPoll >= 500)
        {
            lastConfigPoll = nowMs;
            config.WatchFile();
        }

        // 3. bind maintenance ---------------------------------------------------
        GameBinds::Instance().Tick(0.25);
        Config::Instance().reloadRequested.store(false, std::memory_order_relaxed);

        // 4. watchdog ------------------------------------------------------------
        if (MemoryGuard::PanicLatched() &&
            config.memory.guardOnSuspend &&
            !HookEngine::Instance().Passthrough())
        {
            FH4RPO_LOG_ERROR("Module: watchdog latched - switching to passthrough");
            HookEngine::Instance().SetPassthrough(true);
        }

        if (config.debug.panicOnGuardTrip &&
            MemoryGuard::TripCount() >= config.debug.maxGuardTrips &&
            !HookEngine::Instance().Passthrough())
        {
            FH4RPO_LOG_ERROR("Module: guard trip budget exhausted (%d) - passthrough",
                             MemoryGuard::TripCount());
            HookEngine::Instance().SetPassthrough(true);
        }

        // 5. telemetry housekeeping ---------------------------------------------
        if (config.telemetry.enabled && !g_receiver.IsRunning())
        {
            if (g_receiver.Start(config.telemetry.port, config.telemetry.bindPortFallback,
                                 config.telemetry.bindLoopbackOnly))
            {
                engine_.AttachReceiver(&g_receiver);
                FH4RPO_LOG_INFO("Module: telemetry receiver restarted");
            }
        }

        // 6. overlay --------------------------------------------------------------
        if (config.debug.overlayMode > 0)
        {
            const double now = static_cast<double>(NowMs()) * 0.001;
            if (now - lastOverlayWrite >= config.debug.overlayIntervalS)
            {
                lastOverlayWrite = now;
                UpdateOverlay();
            }
        }

        Sleep(1);
    }
}

// =============================================================================
//  Helpers
// =============================================================================
void Module::WriteStateDump() noexcept
{
    Config& config = Config::Instance();
    if (config.stateDumpPath.empty())
        return;

    if (engine_.DumpState(config.stateDumpPath.c_str()))
        FH4RPO_LOG_INFO("Module: state written to %s", config.stateDumpPath.c_str());
    else
        FH4RPO_LOG_WARN("Module: state dump failed");
}

void Module::UpdateOverlay() noexcept
{
    const PhysicsDiagnostics& diag = engine_.Diagnostics();
    const BodyState& body = engine_.Body();
    const AeroResult& aero = engine_.Aero();

    FH4RPO_LOG_DEBUG("OVL spd=%.1f km/h slip=%.1f deg df=%.0f N drag=%.0f N sub=%d trips=%llu tel=%d",
                     body.speedMps * kMpsToKmh,
                     body.bodySlipAngleRad * kRadToDeg,
                     aero.downforceN, aero.dragN, diag.substeps,
                     static_cast<unsigned long long>(diag.guardTrips),
                     diag.telemetryLive ? 1 : 0);
}

void Module::HandleHotkeys() noexcept
{
    // Hotkey polling lives inline in WorkerLoop(); this entry point is kept for
    // callers that want to poll the keys from another thread.
    Config& config = Config::Instance();
    static std::uint64_t lastPress = 0;
    if (KeyPressedOnce(config.keys.dumpState, lastPress))
        WriteStateDump();
}

void Module::HandlePanic() noexcept
{
    HookEngine::Instance().SetPassthrough(true);
    Config::Instance().masterEnabled.store(false, std::memory_order_relaxed);
    FH4RPO_LOG_ERROR("Module: panic handler invoked");
}

} // namespace fh4rpo
