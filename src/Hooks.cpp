// =============================================================================
//  FH4RPO - Hooks.cpp
//
//  The Windows-only half of the physics engine: the four detours, the Present
//  frame clock, and the installation / removal entry points.
//
//  Every detour follows the same contract:
//      1. passthrough check     (HookEngine::Passthrough())
//      2. SEH guarded body      (__try / __except)
//      3. original called with the original arguments on any failure path
//      4. a trip counter so the module can degrade instead of repeating a fault
// =============================================================================
#include "fh4rpo/PhysicsEngine.hpp"
#include "fh4rpo/HookEngine.hpp"
#include "fh4rpo/GameBinds.hpp"
#include "fh4rpo/Module.hpp"
#include "fh4rpo/Log.hpp"

#include <windows.h>
#include <cmath>
#include <cstring>
#include <d3d11.h>
#include <dxgi.h>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")

namespace fh4rpo {

namespace {

double FrameDelta() noexcept
{
    static LARGE_INTEGER frequency{};
    static LARGE_INTEGER last{};
    static bool initialised = false;

    if (!initialised)
    {
        QueryPerformanceFrequency(&frequency);
        QueryPerformanceCounter(&last);
        initialised = true;
        return 1.0 / 60.0;
    }

    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);
    double dt = static_cast<double>(now.QuadPart - last.QuadPart) /
                static_cast<double>(frequency.QuadPart);
    last = now;

    if (!IsFinite(dt) || dt <= 0.0)
        return 1.0 / 60.0;
    return Clamp(dt, 0.0, kMaxFrameDt);
}

} // namespace

// =============================================================================
//  ============================  H O O K S  ===================================
//
//  Four detours, all installed through HookEngine (MinHook) and all removable
//  at any time.  Each one:
//    1. passes through untouched when the engine is in passthrough mode,
//    2. runs its body inside SEH,
//    3. calls the original with the *original* arguments if anything goes
//       wrong, so the game sees a perfectly normal call.
// =============================================================================
// -----------------------------------------------------------------------------
//  Installation entry points used by Module.cpp
// -----------------------------------------------------------------------------
namespace hooks {
bool InstallAll() noexcept;
void RemoveAll() noexcept;
} // namespace hooks

namespace hooks {

// ---- 1. Friction matrix ------------------------------------------------------
// Shape: float __fastcall(void* carState, void* unused, int wheelIndex,
//                         float slipAngleRad, float normalLoadN)
using FnFriction = float(__fastcall*)(void*, void*, int, float, float);
FnFriction g_originalFriction = nullptr;

float __fastcall Detour_FrictionMatrix(void* self, void* unused, int wheelIndex,
                                       float slipAngleRad, float normalLoadN)
{
    if (g_originalFriction == nullptr)
        return 1.0f;

    if (!FH4RPO_HOP_ALLOWED(hook_friction_matrix))
        return g_originalFriction(self, unused, wheelIndex, slipAngleRad, normalLoadN);

    FH4RPO_TRY
    {
        PhysicsEngine& engine = Module::Instance().Engine();
        const float gameValue = g_originalFriction(self, unused, wheelIndex, slipAngleRad, normalLoadN);
        if (!IsFinite(gameValue))
            return 1.0f;

        const float multiplier = engine.EvaluateFrictionMultiplier(wheelIndex, slipAngleRad, normalLoadN);
        const float result = gameValue * multiplier;
        return IsFinite(result) ? ClampFinite(result, -8.0f, 8.0f, gameValue) : gameValue;
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        HopTrip("hook_friction_matrix");
        return 1.0f;
    }
}

// ---- 2. Aerodynamic downforce ------------------------------------------------
// Shape: float __fastcall(void* carState, void* unused, float speedMps, float rideHeight)
using FnDownforce = float(__fastcall*)(void*, void*, float, float);
FnDownforce g_originalDownforce = nullptr;

float __fastcall Detour_Downforce(void* self, void* unused, float speedMps, float rideHeightM)
{
    if (g_originalDownforce == nullptr)
        return 0.0f;

    if (!FH4RPO_HOP_ALLOWED(hook_downforce))
        return g_originalDownforce(self, unused, speedMps, rideHeightM);

    FH4RPO_TRY
    {
        const float gameValue = g_originalDownforce(self, unused, speedMps, rideHeightM);
        PhysicsEngine& engine = Module::Instance().Engine();
        return engine.EvaluateDownforce(gameValue, speedMps, rideHeightM);
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        HopTrip("hook_downforce");
        return 0.0f;
    }
}

// ---- 3. Suspension solve -----------------------------------------------------
// Shape: void __fastcall(void* carState, void* unused, float dt)
using FnSuspension = void(__fastcall*)(void*, void*, float);
FnSuspension g_originalSuspension = nullptr;

void __fastcall Detour_Suspension(void* self, void* unused, float dt)
{
    if (g_originalSuspension == nullptr)
        return;

    if (!FH4RPO_HOP_ALLOWED(hook_suspension_solve))
    {
        g_originalSuspension(self, unused, dt);
        return;
    }

    FH4RPO_TRY
    {
        g_originalSuspension(self, unused, dt);

        // Re-publish the address table: the resolver refreshes it whenever a
        // pointer becomes stale, so the write-back never targets a dead block.
        PhysicsEngine& engine = Module::Instance().Engine();
        engine.RegisterTargets(GameBinds::Instance().BuildEngineTargets());
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        HopTrip("hook_suspension_solve");
    }
}

// ---- 4. Tyre force -----------------------------------------------------------
// Shape: void __fastcall(void* tireState, void* unused, float dt)
using FnTireForce = void(__fastcall*)(void*, void*, float);
FnTireForce g_originalTireForce = nullptr;

void __fastcall Detour_TireForce(void* self, void* unused, float dt)
{
    if (g_originalTireForce == nullptr)
        return;

    if (!FH4RPO_HOP_ALLOWED(hook_tire_force))
    {
        g_originalTireForce(self, unused, dt);
        return;
    }

    FH4RPO_TRY
    {
        g_originalTireForce(self, unused, dt);
        Module::Instance().Engine().EvaluateFrictionMultiplier(-1, 0.0f, 0.0f);
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        HopTrip("hook_tire_force");
    }
}

// ---- 5. Present (frame clock) ------------------------------------------------
// The frame tick.  Hooking the swap chain's Present gives an exact, per-frame
// dt that is independent of the game's own (pausable) clock.
using FnPresent = HRESULT(__stdcall*)(IDXGISwapChain*, UINT, UINT);
FnPresent g_originalPresent = nullptr;
IDXGISwapChain* g_dummySwapChain = nullptr;
ID3D11Device*   g_dummyDevice = nullptr;
ID3D11DeviceContext* g_dummyContext = nullptr;

HRESULT __stdcall Detour_Present(IDXGISwapChain* swapChain, UINT syncInterval, UINT flags)
{
    if (g_originalPresent == nullptr)
        return DXGI_ERROR_INVALID_CALL;

    if (!FH4RPO_HOP_ALLOWED(hook_present))
        return g_originalPresent(swapChain, syncInterval, flags);

    FH4RPO_TRY
    {
        Module::Instance().Engine().OnPresentTick(FrameDelta());
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        HopTrip("hook_present");
    }

    return g_originalPresent(swapChain, syncInterval, flags);
}

// Creates a throw-away swap chain purely to read the vtable slot of Present.
// Everything is released immediately; no window, no visible output.
bool LocatePresentVtable(std::uintptr_t& out) noexcept
{
    out = 0;

    DXGI_SWAP_CHAIN_DESC desc{};
    desc.BufferCount = 1;
    desc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    desc.BufferDesc.Width = 2;
    desc.BufferDesc.Height = 2;
    desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    desc.OutputWindow = GetDesktopWindow();
    desc.SampleDesc.Count = 1;
    desc.Windowed = TRUE;
    desc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    D3D_FEATURE_LEVEL levels[] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_1 };
    D3D_FEATURE_LEVEL obtained{};

    const HRESULT hr = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, levels, 2,
        D3D11_SDK_VERSION, &desc, &g_dummySwapChain, &g_dummyDevice,
        &obtained, &g_dummyContext);

    if (FAILED(hr) || g_dummySwapChain == nullptr)
        return false;

    void** vtable = *reinterpret_cast<void***>(g_dummySwapChain);
    if (vtable == nullptr)
        return false;

    out = reinterpret_cast<std::uintptr_t>(vtable[8]);   // IDXGISwapChain::Present
    return out != 0;
}

void ReleaseDummyDevice() noexcept
{
    if (g_dummyContext) { g_dummyContext->Release(); g_dummyContext = nullptr; }
    if (g_dummySwapChain) { g_dummySwapChain->Release(); g_dummySwapChain = nullptr; }
    if (g_dummyDevice) { g_dummyDevice->Release(); g_dummyDevice = nullptr; }
}

bool InstallAll() noexcept
{
    HookEngine& engine = HookEngine::Instance();
    if (!engine.Initialize())
        return false;

    GameBinds& binds = GameBinds::Instance();
    Config& config = Config::Instance();

    bool anyInstalled = false;

    auto installCode = [&](const char* name, void* detour, void** original)
    {
        const std::uintptr_t target = binds.Resolve(name);
        if (target == 0)
        {
            FH4RPO_LOG_WARN("Hooks: %s unresolved - feature disabled (game keeps its own physics)", name);
            return false;
        }

        const HookStatus status = engine.Add(name, target, detour, original, L"ForzaHorizon4.exe");
        if (status != HookStatus::Ok)
        {
            FH4RPO_LOG_WARN("Hooks: could not hook %s (%d)", name, static_cast<int>(status));
            return false;
        }
        FH4RPO_LOG_INFO("Hooks: %s installed", name);
        return true;
    };

    anyInstalled |= installCode("hook_friction_matrix", reinterpret_cast<void*>(&Detour_FrictionMatrix),
                                reinterpret_cast<void**>(&g_originalFriction));
    anyInstalled |= installCode("hook_downforce", reinterpret_cast<void*>(&Detour_Downforce),
                                reinterpret_cast<void**>(&g_originalDownforce));
    anyInstalled |= installCode("hook_suspension_solve", reinterpret_cast<void*>(&Detour_Suspension),
                                reinterpret_cast<void**>(&g_originalSuspension));
    anyInstalled |= installCode("hook_tire_force", reinterpret_cast<void*>(&Detour_TireForce),
                                reinterpret_cast<void**>(&g_originalTireForce));

    // The Present hook is the only one that is not title specific: it comes
    // from the D3D11 runtime, so it works on every build of the game.
    std::uintptr_t presentVtable = 0;
    if (LocatePresentVtable(presentVtable))
    {
        const HookStatus status = engine.Add("hook_present", presentVtable,
                                             reinterpret_cast<void*>(&Detour_Present),
                                             reinterpret_cast<void**>(&g_originalPresent),
                                             L"dxgi.dll");
        if (status == HookStatus::Ok)
        {
            anyInstalled = true;
            FH4RPO_LOG_INFO("Hooks: Present hooked at 0x%p", reinterpret_cast<void*>(presentVtable));
        }
        else
        {
            FH4RPO_LOG_WARN("Hooks: Present hook failed (%d)", static_cast<int>(status));
        }
        ReleaseDummyDevice();
    }
    else
    {
        FH4RPO_LOG_WARN("Hooks: could not locate IDXGISwapChain::Present");
    }

    if (!anyInstalled)
        FH4RPO_LOG_ERROR("Hooks: nothing installed - running in telemetry-only mode");

    (void)config;
    engine.DumpStatus();
    return anyInstalled;
}

void RemoveAll() noexcept
{
    HookEngine& engine = HookEngine::Instance();
    engine.Shutdown();                 // disables + removes every trampoline
    g_originalFriction = nullptr;
    g_originalDownforce = nullptr;
    g_originalSuspension = nullptr;
    g_originalTireForce = nullptr;
    g_originalPresent = nullptr;
    ReleaseDummyDevice();
}


} // namespace hooks

} // namespace fh4rpo
