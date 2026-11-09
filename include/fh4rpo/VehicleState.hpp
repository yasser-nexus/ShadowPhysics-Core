// =============================================================================
//  FH4RPO - VehicleState.hpp
//
//  The live vehicle model: per-corner wheel state, the body state, and the
//  derived quantities (weight transfer, aero forces, chassis attitude).
//  Everything here is plain data plus trivial accessors - no game pointers.
// =============================================================================
#pragma once

#include <cstdint>
#include "Core.hpp"

namespace fh4rpo {

enum WheelIndex : int
{
    WheelFL = 0,
    WheelFR = 1,
    WheelRL = 2,
    WheelRR = 3,
    kWheelCount = 4
};

constexpr bool IsFrontAxle(int wheel) noexcept { return wheel == WheelFL || wheel == WheelFR; }
constexpr bool IsRearAxle(int wheel)  noexcept { return wheel == WheelRL || wheel == WheelRR; }
constexpr int  AxleIndex(int wheel)   noexcept { return IsFrontAxle(wheel) ? 0 : 1; }
constexpr int  SideIndex(int wheel)   noexcept { return (wheel == WheelFL || wheel == WheelRL) ? 0 : 1; }

// -----------------------------------------------------------------------------
//  WheelState
// -----------------------------------------------------------------------------
struct WheelState
{
    bool  connected        = false;
    bool  grounded         = true;
    bool  onRumbleStrip    = false;

    float slipAngleRad     = 0.0f;   // filtered
    float rawSlipAngleRad  = 0.0f;
    float slipAngleRate    = 0.0f;   // d(slip)/dt, used by the relaxation model
    float slipRatio        = 0.0f;   // filtered
    float rawSlipRatio     = 0.0f;

    float normalLoadN      = 0.0f;   // after weight transfer
    float staticLoadN      = 0.0f;
    float frictionMu       = 1.0f;   // effective mu after every modifier
    float matrixMultiplier = 1.0f;   // output of the handling scaling matrix

    float longitudinalForceN = 0.0f;
    float lateralForceN      = 0.0f;

    float suspensionTravelM   = 0.0f;
    float normalizedTravel    = 0.5f;
    float suspensionVelocityM = 0.0f;
    float suspensionForceN    = 0.0f;

    float tireTempC        = 80.0f;
    float tireWear         = 0.0f;
    float gripScaleFromTemp = 1.0f;

    float surfaceFriction  = 1.0f;
    float wheelSpeedMps    = 0.0f;
    float wheelRadiusM     = 0.34f;
    float rotationAngleRad = 0.0f;
    float brakeTorqueNm    = 0.0f;
    float driveTorqueNm    = 0.0f;

    void Reset() noexcept { *this = WheelState{}; }
};

// -----------------------------------------------------------------------------
//  BodyState - authoritative, integrated by the kernel.
// -----------------------------------------------------------------------------
struct BodyState
{
    Vec3  position;                 // module-internal, integrated
    Vec3  velocity;
    Vec3  accel;
    Vec3  angularVelocity;          // pitch (x), yaw (y), roll (z) rates

    float yaw   = 0.0f;
    float pitch = 0.0f;
    float roll  = 0.0f;

    float speedMps        = 0.0f;
    float forwardSpeedMps = 0.0f;
    float lateralSpeedMps = 0.0f;
    float yawRateRad      = 0.0f;
    float bodySlipAngleRad = 0.0f;

    float steerAngleRad   = 0.0f;
    float throttle        = 0.0f;
    float brake           = 0.0f;
    float clutch          = 0.0f;
    float handbrake       = 0.0f;

    bool  airborne        = false;
    bool  valid           = false;
};

// -----------------------------------------------------------------------------
//  WeightTransferResult - the corner load solution.
// -----------------------------------------------------------------------------
struct WeightTransferResult
{
    float staticLoad[4]      = {};
    float dynamicLoad[4]     = {};   // final normal load, N
    float longitudinalDelta  = 0.0f; // +N moved to the rear under acceleration
    float lateralDelta       = 0.0f; // +N moved to the right under a left turn
    float totalNormalN       = 0.0f;
    float loadTransferFrontN = 0.0f;
    float loadTransferRearN  = 0.0f;
    float pitchMomentNm      = 0.0f;
    float rollMomentNm       = 0.0f;
    float pitchTargetRad     = 0.0f;
    float rollTargetRad      = 0.0f;
    bool  airborne           = false;
};

// -----------------------------------------------------------------------------
//  AeroResult
// -----------------------------------------------------------------------------
struct AeroResult
{
    float downforceN      = 0.0f;
    float downforceFrontN = 0.0f;
    float downforceRearN  = 0.0f;
    float dragN           = 0.0f;
    float sideForceN      = 0.0f;
    float dynamicPressure = 0.0f;   // 0.5 * rho * v^2
    float rideHeightScale = 1.0f;
    float stallScale      = 1.0f;
    float pitchMomentNm   = 0.0f;
    float referenceDownforceN = 0.0f;   // what the game itself supplied
};

// -----------------------------------------------------------------------------
//  Diagnostics - surfaced by DumpState() and the optional overlay.
// -----------------------------------------------------------------------------
struct PhysicsDiagnostics
{
    double  simTime            = 0.0;
    double  frameDt            = 0.0;
    int     substeps           = 0;
    float   updateCostUs       = 0.0f;
    std::uint64_t updates      = 0;
    std::uint64_t guardTrips   = 0;
    std::uint64_t aeroOverrides = 0;
    std::uint64_t matrixWrites = 0;
    std::uint64_t hookCalls    = 0;
    bool    telemetryLive      = false;
    bool    passiveFallback    = false;
    float   frictionFrontMean  = 1.0f;
    float   frictionRearMean   = 1.0f;
    int     driveLayout        = 1;
    char    lastEvent[96]      = {};
};

} // namespace fh4rpo
