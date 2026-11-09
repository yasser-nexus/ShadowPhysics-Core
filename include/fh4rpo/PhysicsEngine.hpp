// =============================================================================
//  FH4RPO - PhysicsEngine.hpp
//
//  Public surface of the physics overhaul.
//
//  Pipeline (one call per rendered frame, from the present hook):
//
//      Telemetry  ->  BodyState  ->  Weight transfer  ->  Tire model
//                 ->  Friction matrix ->  Aero  ->  Drivetrain / brakes
//                 ->  Suspension forces ->  Integrate  ->  Export patch
//
//  Every stage is individually switchable and individually guarded, so a
//  mis-tuned table or a dead game pointer costs at most one frame of effect
//  instead of the process.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>

#include "Core.hpp"
#include "Config.hpp"
#include "VehicleState.hpp"
#include "Telemetry.hpp"
#include "MemoryGuard.hpp"

namespace fh4rpo {

// -----------------------------------------------------------------------------
//  Downforce descriptor - mirrors the one used by the game's aero routine.
// -----------------------------------------------------------------------------
struct DownforceInput
{
    float speedMps          = 0.0f;
    float rideHeightM       = 0.11f;
    float yawRad            = 0.0f;
    float pitchRad          = 0.0f;
    float slipAngleRad      = 0.0f;
    float gameDownforceN    = 0.0f;   // value the game was about to apply
    bool  gameValuePresent  = false;
};

// -----------------------------------------------------------------------------
//  Tire evaluation break-down (exposed for the test harness + dump).
// -----------------------------------------------------------------------------
struct TireOutput
{
    float normalizedSlipAngle = 0.0f;
    float normalizedSlipRatio = 0.0f;
    float combinedSlip        = 0.0f;
    float lateralForceN       = 0.0f;
    float longitudinalForceN  = 0.0f;
    float muUsed              = 0.0f;
    float gripScale           = 0.0f;
    bool  saturated           = false;
};

// -----------------------------------------------------------------------------
//  Engine
// -----------------------------------------------------------------------------
class PhysicsEngine
{
public:
    PhysicsEngine() noexcept;
    ~PhysicsEngine() noexcept;

    PhysicsEngine(const PhysicsEngine&) = delete;
    PhysicsEngine& operator=(const PhysicsEngine&) = delete;

    // mode: true == use live telemetry only, false == allow extrapolation.
    void Initialize() noexcept;
    void Shutdown() noexcept;
    bool IsInitialized() const noexcept;

    // Called once per rendered frame. `frameDt` must come from the real frame
    // timer, never from a game clock that can be paused.
    void OnFrame(double frameDt) noexcept;

    // Feeds one decoded telemetry sample (also used directly by the tests).
    void SubmitTelemetry(const TelemetrySample& sample) noexcept;

    // Optional: the engine then pulls the newest sample from the receiver at
    // the start of every frame, so the network thread never touches the solver.
    void AttachReceiver(TelemetryReceiver* receiver) noexcept;

    // -------------------------------------------------------------------------
    //  Hook entry points - called by the detours in PhysicsEngine.cpp
    // -------------------------------------------------------------------------

    // Handling dynamic scaling: called with the game's own slip angle for a
    // given wheel; returns the multiplier to stamp into the friction matrix.
    float EvaluateFrictionMultiplier(int wheelIndex, float slipAngleRad, float normalLoadN) noexcept;

    // Aero downforce hook: replaces the game's downforce value.
    float EvaluateDownforce(float gameDownforceN, float speedMps, float rideHeightM) noexcept;

    // Called by the present hook before anything is drawn.
    void OnPresentTick(double frameDt) noexcept;

    // -------------------------------------------------------------------------
    //  Accessors
    // -------------------------------------------------------------------------
    const BodyState&            Body() const noexcept { return body_; }
    const WheelState&           Wheel(int index) const noexcept;
    const WeightTransferResult& Transfer() const noexcept { return transfer_; }
    const AeroResult&           Aero() const noexcept { return aero_; }
    const PhysicsDiagnostics&   Diagnostics() const noexcept { return diagnostics_; }
    const VehicleSpecConfig&    Spec() const noexcept { return spec_; }

    // Runtime vehicle spec, derived from telemetry when available.
    void UpdateVehicleSpec(const VehicleSpecConfig& spec) noexcept;

    // -------------------------------------------------------------------------
    //  Output
    // -------------------------------------------------------------------------
    // Caches the values the memory writer should stamp into the game. Returns
    // false when nothing should be written this frame.
    bool BuildGameWriteback(float outDownforceN,
                            float outGripScale,
                            float outBrakeBias,
                            float outSteerAngleDeg) noexcept;

    // Writes the cached write-back values through MemoryGuard into the game
    // structures registered with RegisterTargets(). Returns the number of
    // successful writes.
    int  ApplyWriteback() noexcept;

    struct GameTargets
    {
        std::uintptr_t downforceAddress   = 0;   // f32
        std::uintptr_t gripScaleAddress   = 0;   // f32
        std::uintptr_t brakeBiasAddress   = 0;   // f32
        std::uintptr_t steerAngleAddress  = 0;   // f32
        std::uintptr_t frictionMatrixBase = 0;   // 4x4 f32 (FL,FR,RL,RR x long/lat)
        std::uintptr_t frictionMatrixStride = 16; // bytes between corners
    };
    void RegisterTargets(const GameTargets& targets) noexcept;
    const GameTargets& Targets() const noexcept { return targets_; }

    int  WriteFrictionMatrix() noexcept;

    // Writes a snapshot to `<module>_state.txt` (crash forensics / tuning aid).
    bool DumpState(const char* path) const noexcept;

    // Zeroes the integrator without touching configuration (used after a
    // telemetry discontinuity, e.g. a teleport / fast travel).
    void ResetDynamics() noexcept;

private:
    // --- pipeline stages -----------------------------------------------------
    void AcquireTelemetry(double frameDt) noexcept;
    void UpdateBodyFromTelemetry(const TelemetrySample& sample, float dt) noexcept;
    void ExtrapolateBody(float dt) noexcept;
    void DeriveVehicleSpec() noexcept;
    void SolveWeightTransfer(float dt) noexcept;
    void EvaluateTires(float dt) noexcept;
    void ApplyFrictionMatrix(float dt) noexcept;
    void SolveAero(float dt) noexcept;
    void ApplyVehicleSpec(const VehicleSpecConfig& source) noexcept;
    void SolveAeroStage(float dt) noexcept;
    void SolveDrivetrain(float dt) noexcept;
    void SolveBrakes(float dt) noexcept;
    void SolveSuspension(float dt) noexcept;
    void Integrate(float dt) noexcept;
    void UpdateTireThermal(float dt) noexcept;
    void UpdateChassisAttitude(float dt) noexcept;

    // --- helpers -------------------------------------------------------------
    void   Step(float dt) noexcept;
    float  ComputeCornerLoad(int wheel) const noexcept;
    float  TireMagicFormula(float slip, float B, float C, float D, float E) const noexcept;
    void   UpdateWheelGeometry() noexcept;
    void   SetEvent(const char* text) noexcept;

    // --- state ---------------------------------------------------------------
    bool                 initialized_ = false;
    Config*              config_      = nullptr;
    VehicleSpecConfig    spec_{};
    BodyState            body_{};
    WheelState           wheels_[kWheelCount] = {};
    WeightTransferResult transfer_{};
    AeroResult           aero_{};
    PhysicsDiagnostics   diagnostics_{};
    GameTargets          targets_{};

    TelemetryReceiver*   receiver_ = nullptr;
    TelemetrySample      lastSample_{};
    TelemetrySample      stagedSample_{};
    bool                 hasStagedSample_ = false;
    bool                 hasSample_       = false;
    // Raw accelerations from Data Out, kept apart from body_.accel (which is
    // the *model* output) so the weight transfer stage always uses the game's
    // measured values even after the integrator has run.
    Vec3                 telemetryAccel_{};
    Vec3                 modelAccel_{};
    double               lastSampleTime_  = 0.0;
    double               clock_           = 0.0;

    FixedStepDriver      stepper_{};
    RateLimiter          steerLimiter_{};
    BilinearTable        frontMatrix_{};
    BilinearTable        rearMatrix_{};
    Pcg32                rng_{};

    float                smoothedDownforceN_ = 0.0f;
    float                smoothedDragN_      = 0.0f;
    float                smoothedFrontGrip_  = 1.0f;
    float                smoothedRearGrip_   = 1.0f;
    float                brakeTempC_         = 320.0f;
    float                wheelbaseEffectiveM_ = 2.70f;
    float                prevForwardAccel_   = 0.0f;
    float                prevLateralAccel_   = 0.0f;
    int                  lastGear_           = 0;

    // write-back cache
    bool   writebackPending_ = false;
    float  wbDownforceN_     = 0.0f;
    float  wbGripScale_      = 1.0f;
    float  wbBrakeBias_      = 0.62f;
    float  wbSteerAngleDeg_  = 0.0f;
};

} // namespace fh4rpo
