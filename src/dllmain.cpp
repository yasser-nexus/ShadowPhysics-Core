// =============================================================================
//  FH4RPO - dllmain.cpp
//
//  Entry point of the .asi / .dll.
//
//  Rules honoured here (these are what keep a mod from corrupting the host):
//
//    1. Nothing heavy happens inside DllMain.  The loader lock is held during
//       DLL_PROCESS_ATTACH and DLL_PROCESS_DETACH, so we only spawn / signal a
//       worker thread and return immediately.
//    2. A named mutex guarantees a single instance even if the user renames the
//       file and loads it twice.
//    3. On DLL_PROCESS_DETACH we wait (bounded) for the worker to finish and
//       then let it tear down every trampoline.  If the process is terminating
//       we skip the wait - calling into the loader from a terminating process
//       is what produces the classic "crash on exit" -> we only unhook.
// =============================================================================
#include <windows.h>

#include "fh4rpo/Module.hpp"
#include "fh4rpo/Log.hpp"
#include "fh4rpo/HookEngine.hpp"
#include "fh4rpo/MemoryGuard.hpp"
#include "fh4rpo/Config.hpp"

namespace {

HMODULE           g_selfModule = nullptr;
HANDLE            g_singleInstanceMutex = nullptr;
constexpr wchar_t kMutexName[] = L"Global\\FH4RPO_SingleInstance_v1";

// Builds a "already busy" message box without pulling in user32-heavy code.
void ReportAlreadyLoaded() noexcept
{
    OutputDebugStringA("[FH4RPO] another instance is already loaded - refusing to attach\n");
    MessageBoxW(nullptr,
                L"FH4 Realistic Physics Overhaul is already loaded in this process.\n"
                L"Only one instance can attach at a time.",
                L"FH4RPO", MB_OK | MB_ICONINFORMATION | MB_SETFOREGROUND);
}

} // namespace

// =============================================================================
//  DllMain
// =============================================================================
BOOL APIENTRY DllMain(HMODULE module, DWORD reasonForCall, LPVOID reserved)
{
    switch (reasonForCall)
    {
    case DLL_PROCESS_ATTACH:
    {
        g_selfModule = module;
        DisableThreadLibraryCalls(module);      // we do not care about per-thread events

        // ---- single instance guard ------------------------------------------
        g_singleInstanceMutex = CreateMutexW(nullptr, FALSE, kMutexName);
        if (g_singleInstanceMutex == nullptr)
            return FALSE;

        if (GetLastError() == ERROR_ALREADY_EXISTS)
        {
            ReportAlreadyLoaded();
            CloseHandle(g_singleInstanceMutex);
            g_singleInstanceMutex = nullptr;
            return FALSE;
        }

        // ---- hand off to the module -----------------------------------------
        fh4rpo::DllMainContext context;
        context.processAttach = true;

        if (!fh4rpo::Module::Instance().Attach(module, context))
        {
            if (g_singleInstanceMutex)
            {
                CloseHandle(g_singleInstanceMutex);
                g_singleInstanceMutex = nullptr;
            }
            return FALSE;
        }

        return TRUE;
    }

    case DLL_PROCESS_DETACH:
    {
        fh4rpo::DllMainContext context;
        context.processDetach = true;
        context.reserved = reserved;

        // `reserved != nullptr` means the process is exiting: the loader lock is
        // held, other threads may already be dead.  Do not wait, do not touch
        // the heap more than necessary - only release the hooks.
        const bool processTerminating = (reserved != nullptr);

        fh4rpo::Module::Instance().Detach(processTerminating);

        if (g_singleInstanceMutex != nullptr)
        {
            CloseHandle(g_singleInstanceMutex);
            g_singleInstanceMutex = nullptr;
        }

        g_selfModule = nullptr;
        return TRUE;
    }

    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    default:
        return TRUE;
    }
}

// =============================================================================
//  Exported control surface (all optional, used by launchers / tests)
// =============================================================================
extern "C" __declspec(dllexport) void* __stdcall FH4RPO_GetVersion()
{
    static const char version[] = "FH4RPO 1.0.0 (" __DATE__ ")";
    return const_cast<char*>(version);
}

extern "C" __declspec(dllexport) int __stdcall FH4RPO_SetMasterEnabled(int enabled)
{
    fh4rpo::Config& config = fh4rpo::Config::Instance();
    config.masterEnabled.store(enabled != 0, std::memory_order_relaxed);
    fh4rpo::HookEngine::Instance().SetPassthrough(enabled == 0);
    return config.masterEnabled.load() ? 1 : 0;
}

extern "C" __declspec(dllexport) int __stdcall FH4RPO_RequestConfigReload()
{
    fh4rpo::Config::Instance().reloadRequested.store(true, std::memory_order_relaxed);
    fh4rpo::Config::Instance().WatchFile();
    return 1;
}

extern "C" __declspec(dllexport) int __stdcall FH4RPO_DumpState()
{
    fh4rpo::Config& config = fh4rpo::Config::Instance();
    if (config.stateDumpPath.empty())
        return 0;
    return fh4rpo::Module::Instance().Engine().DumpState(config.stateDumpPath.c_str()) ? 1 : 0;
}

extern "C" __declspec(dllexport) int __stdcall FH4RPO_GetGuardTrips()
{
    return fh4rpo::MemoryGuard::TripCount();
}

extern "C" __declspec(dllexport) int __stdcall FH4RPO_IsPassthrough()
{
    return fh4rpo::HookEngine::Instance().Passthrough() ? 1 : 0;
}
