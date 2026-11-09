// =============================================================================
//  FH4RPO - Config.hpp
//  Every tunable of the overhaul lives here and is hot-reloadable from
//  FH4RPO.ini (drop the file next to the .asi, edit, press F5 in game or just
//  wait - the watcher reloads on change).
// =============================================================================
#pragma once

#include <string>
#include <atomic>

namespace fh4rpo {

enum class DriveLayout : int
{
    FWD = 0,
    RWD = 1,
    AWD = 2
};

enum class LogVerbosity : int
{
    Trace = 0,
    Debug = 1,
    Info  = 2,
    Warn  = 3,
    Off   = 4
};

// -----------------------------------------------------------------------------
//  VehicleSpecConfig - the physical description of the chassis.  These are the
//  "reference" numbers the solver needs; they are typed (not patched into game
//  memory) and can also be auto-derived at runtime from the Data Out stream.
// -----------------------------------------------------------------------------
struct VehicleSpecConfig
{
    float massKg               = 1450.0f;
    float wheelbaseM           = 2.70f;
    float frontTrackM          = 1.58f;
    float rearTrackM           = 1.58f;
    float cgHeightM            = 0.52f;
    float cgToFrontM           = 0.0f;    // <= 0 -> derived from weightDistribution
    float weightDistribution   = 0.53f;   // fraction of mass on the FRONT axle
    float wheelRadiusFrontM    = 0.34f;
    float wheelRadiusRearM     = 0.34f;
    float rollInertiaKgM2      = 520.0f;
    float pitchInertiaKgM2     = 1900.0f;
    float yawInertiaKgM2       = 2300.0f;
    float rollStiffnessNmPerRad = 34000.0f;
    float pitchStiffnessNmPerRad = 52000.0f;
    float rollDampingNmsPerRad = 5400.0f;
    float pitchDampingNmsPerRad = 7200.0f;
    float antiRollBarFrontNmPerRad = 9000.0f;
    float antiRollBarRearNmPerRad  = 5500.0f;
    float maxSuspensionTravelM = 0.13f;
    float rideHeightM          = 0.11f;
    float frontTireWidthM      = 0.245f;
    float rearTireWidthM       = 0.265f;
    float frontTireAspectRatio = 0.35f;
    float rearTireAspectRatio  = 0.35f;
    float frontGearRatioDrive  = 4.10f;   // final drive * gear, only used for slip ratio
    float rearGearRatioDrive   = 4.10f;
    int   driveLayout          = static_cast<int>(DriveLayout::RWD);
};

// -----------------------------------------------------------------------------
//  TireConfig - coefficients of the Pacejka-style combined slip model.
// -----------------------------------------------------------------------------
struct TireConfig
{
    float muNominalFront       = 1.08f;   // peak friction coefficient, front
    float muNominalRear        = 1.12f;   // peak friction coefficient, rear
    float bLong                = 10.5f;   // magic formula stiffness, longitudinal
    float cLong                = 1.62f;   // shape
    float dLongScale           = 1.00f;   // peak scaling
    float eLong                = 0.30f;   // curvature
    float bLat                 = 8.40f;
    float cLat                 = 1.38f;
    float eLat                 = 0.28f;
    float loadSensitivity      = 0.165f;  // mu -= k * (Fz/Fz0 - 1)  (tire load sensitivity)
    float loadReferenceN       = 4000.0f; // Fz0
    float camberGainPerDeg     = 0.0035f;
    float slipAnglePeakDeg     = 7.5f;    // ~peak lateral slip angle
    float slipRatioPeak        = 0.11f;   // ~peak longitudinal slip ratio
    float relaxationLengthM    = 0.28f;   // carcass lag (slip angle / ratio filtering)
    float temperatureOptimumC  = 84.0f;
    float temperatureWindowC   = 55.0f;
    float temperatureGripFloor = 0.72f;   // grip scale when stone cold / overheated
    float wearGripFloor        = 0.80f;
    float wearRate             = 0.000010f;
    float heatGeneration       = 0.085f;
    float coolingRate          = 0.055f;
    float surfaceFrictionRoad  = 1.000f;
    float surfaceFrictionWet   = 0.820f;
    float surfaceFrictionGrass = 0.520f;
    float surfaceFrictionDirt  = 0.640f;
    float surfaceFrictionSnow  = 0.380f;
};

// -----------------------------------------------------------------------------
//  FrictionMatrixConfig - the Handling Dynamic Scaling table.
//  Rows = slip angle (deg, negative = left, positive = right, applied |x| for
//  symmetry unless `asymmetric` is set).  Columns = vertical load (N).
// -----------------------------------------------------------------------------
struct FrictionMatrixConfig
{
    bool  enabled            = true;
    bool  asymmetric         = false;
    bool  perAxle            = true;     // separate tables for front and rear
    bool  useLoadAxis        = true;     // false -> 1-D table (slip angle only)
    float snapDeg            = 2.0f;     // slip angle at which the table is fully engaged
    float fadeRate           = 6.0f;     // 1/s, blend speed towards target multiplier
    float frontAngleKeysDeg[6] = { 0.0f, 2.0f, 5.0f, 8.0f, 14.0f, 26.0f };
    float rearAngleKeysDeg[6]  = { 0.0f, 2.0f, 5.0f, 8.0f, 14.0f, 26.0f };
    float loadKeysN[4]       = { 1200.0f, 3000.0f, 4800.0f, 7000.0f };
    // front: [angle][load]
    float frontValues[6][4]  = {
        { 1.030f, 1.020f, 1.005f, 0.985f },
        { 1.045f, 1.035f, 1.018f, 0.995f },
        { 1.020f, 1.012f, 1.000f, 0.985f },
        { 0.975f, 0.968f, 0.960f, 0.950f },
        { 0.905f, 0.900f, 0.895f, 0.890f },
        { 0.840f, 0.835f, 0.830f, 0.828f }
    };
    // rear: slightly more forgiving at low load so a slide can be caught
    float rearValues[6][4]   = {
        { 1.035f, 1.025f, 1.010f, 0.990f },
        { 1.055f, 1.045f, 1.028f, 1.005f },
        { 1.030f, 1.022f, 1.008f, 0.992f },
        { 0.990f, 0.982f, 0.972f, 0.960f },
        { 0.930f, 0.925f, 0.918f, 0.910f },
        { 0.880f, 0.872f, 0.866f, 0.860f }
    };
};

// -----------------------------------------------------------------------------
//  WeightTransferConfig
// -----------------------------------------------------------------------------
struct WeightTransferConfig
{
    bool  enabled              = true;
    float longTransferGain     = 1.00f;   // 1.0 = physically exact, <1 softens
    float latTransferGain      = 1.00f;
    float pitchAccelGain       = 1.00f;   // chassis pitch response to dFz
    float rollAccelGain        = 1.00f;
    float geometryGain         = 0.35f;   // roll centre / swing arm correction 0..1
    float jackingGain          = 0.0f;    // lateral load -> vertical force
    float minCornerLoadN       = 25.0f;   // never fully unload a corner
    float loadRateLimitN       = 400000.0f; // N/s, keeps dFz physically plausible
    float antiDiveGain         = 0.55f;   // 0 = no anti-dive, 1 = fully balanced
    float antiSquatGain        = 0.45f;
};

// -----------------------------------------------------------------------------
//  AeroConfig - replaces the game's downforce table with a live
//  0.5 * rho * v^2 * Cl * A computation.
// -----------------------------------------------------------------------------
struct AeroConfig
{
    bool  enabled             = true;
    bool  overrideGameDownforce = true;
    float airDensityKgM3      = 1.225f;
    float liftCoefficientFront = 0.62f;
    float liftCoefficientRear  = 0.92f;
    float frontalAreaM2        = 2.15f;
    float dragCoefficient      = 0.34f;
    float groundEffectHeightM  = 0.09f;
    float groundEffectGain     = 0.42f;
    float stallSpeedMps        = 78.0f;    // beyond this, downforce saturates
    float stallFalloff         = 0.22f;
    float balanceSplit         = 0.40f;    // front share of total downforce
    float yawSensitivity       = 0.18f;    // side force from yaw angle
    float smoothTimeS          = 0.02f;
    float maxDownforceN        = 22000.0f;
    float maxDragN             = 16000.0f;
};

// -----------------------------------------------------------------------------
//  DrivetrainConfig / BrakeConfig / SteeringConfig
// -----------------------------------------------------------------------------
struct DrivetrainConfig
{
    bool  enabled             = true;
    float torqueSplitFront    = 0.30f;    // used when layout == AWD
    float engineBrakingGain   = 0.85f;
    float clutchEngagement    = 1.00f;
    float drivelineInertiaKgM2 = 0.24f;
    float rearBiasOnLaunch    = 0.20f;    // extra rear share at low speed
    float launchSpeedMps      = 12.0f;
    float maxEngineTorqueNm   = 620.0f;
    float torqueCurvePeakRpm  = 4600.0f;
    float torqueCurveWidthRpm = 2100.0f;
};

struct BrakeConfig
{
    bool  enabled             = true;
    float maxBrakeTorqueNm    = 3400.0f;  // per axle, at full pedal
    float frontBias           = 0.62f;
    float absSlipThreshold    = 0.22f;
    float absCycleHz          = 12.0f;
    float brakeFadeStartC     = 480.0f;
    float brakeFadeRate       = 0.0012f;
    float handbrakeTorqueNm   = 2100.0f;
};

struct SteeringConfig
{
    bool  enabled             = true;
    float maxSteerAngleDeg    = 34.0f;
    float speedSensitivity    = 0.045f;   // 1/(m/s)
    float minAngleAtSpeedDeg  = 7.5f;
    float ackermannGain       = 0.65f;    // 0 = parallel, 1 = full Ackermann
    float counterSteerAssist  = 0.28f;    // 0..1, helps catch a slide
    float assistSlipDeadzoneDeg = 3.0f;
    float autosteerRate       = 3.2f;     // rad/s, keyboard smoothing
    float returnRate          = 4.5f;
    float ffbGain             = 1.00f;
};

// -----------------------------------------------------------------------------
//  MemoryConfig - the runtime patching layer
// -----------------------------------------------------------------------------
struct MemoryConfig
{
    bool  enableWriteGuards   = true;
    bool  enableSelfTest      = true;
    bool  verifyHookTargets   = true;
    float resolveIntervalS    = 3.0f;     // how often a stale pointer is re-checked
    int   scannerAlignment    = 4;
    int   maxModuleNameLen    = 32;
    bool  enableDebugPrivilege = true;
    bool  guardOnSuspend      = true;

    // Manual address pins ([Addresses] in the INI).  Absolute virtual address
    // of a module RVA.  They take priority over every signature scan, which is
    // the escape hatch when a title update breaks a pattern: pin the value once
    // and the module keeps working without a rebuild.  0 = not pinned.
    unsigned long long pinHookFrictionMatrix = 0;
    unsigned long long pinHookDownforce      = 0;
    unsigned long long pinHookSuspension     = 0;
    unsigned long long pinHookTireForce      = 0;
    unsigned long long pinGripScale          = 0;
    unsigned long long pinDownforceScale     = 0;
    unsigned long long pinBrakeBias          = 0;
    unsigned long long pinSteerAngle         = 0;
    unsigned long long pinFrictionMatrix     = 0;
    unsigned long long pinModuleBase         = 0;   // ASLR base hint, unused by default
};

// -----------------------------------------------------------------------------
//  TelemetryConfig - the Data Out UDP receiver
// -----------------------------------------------------------------------------
struct TelemetryConfig
{
    bool  enabled             = true;
    int   port                = 5300;     // FH4 "Data Out" default
    int   bindPortFallback    = 5301;
    bool  bindLoopbackOnly    = true;
    float timeoutSeconds      = 2.5f;     // no packets -> fall back to passive mode
    float speedFilterHz       = 22.0f;
    bool  autoDeriveSpec      = true;     // learn mass / wheelbase from telemetry
    bool  broadcastAssist     = false;    // re-broadcast patched telemetry
    int   broadcastPort       = 5302;
    int   maxPacketBytes      = 512;
};

// -----------------------------------------------------------------------------
//  PhysicsConfig - solver level switches
// -----------------------------------------------------------------------------
struct PhysicsConfig
{
    bool  enabled             = true;
    bool  passiveModeAllowed  = true;    // no telemetry -> extrapolate state
    float fixedStepHz         = 240.0f;
    int   maxSubSteps         = 8;
    float timeScale           = 1.0f;
    bool  horizontalOnly      = true;
    float maxSpeedMps         = 140.0f;
    float slipAngleFilterHz   = 40.0f;
    bool  applySuspensionForces = true;
    bool  applyChassisTorque  = true;
    bool  applyDrivetrain     = true;
    bool  applyBrakes         = true;
    float globalGripScale     = 1.00f;
    float assistGripFloor     = 0.55f;
    bool  tcsSimulation       = false;
    float tcsSlipTarget       = 0.14f;
    bool  escSimulation       = false;
    float escSlipThresholdDeg = 9.0f;
    float escCorrectionGain   = 0.55f;
};

// -----------------------------------------------------------------------------
//  DebugConfig
// -----------------------------------------------------------------------------
struct DebugConfig
{
    int   logLevel            = static_cast<int>(LogVerbosity::Info);
    bool  logTelemetry        = false;
    bool  logHooks            = true;
    bool  dumpStateOnCrash    = true;
    int   overlayMode         = 0;       // 0 = off, 1 = verbose text
    float overlayIntervalS    = 0.25f;
    bool  panicOnGuardTrip    = false;   // disable all patches after a guard trip
    int   maxGuardTrips       = 12;
};

// -----------------------------------------------------------------------------
//  KeysConfig - virtual key codes
// -----------------------------------------------------------------------------
struct KeysConfig
{
    int toggleMaster   = 0x71;   // F2
    int togglePassive  = 0x72;   // F3
    int reloadConfig   = 0x74;   // F5
    int panicDisable   = 0x73;   // F4
    int dumpState      = 0x77;   // F8
};

// -----------------------------------------------------------------------------
//  Config - aggregate
// -----------------------------------------------------------------------------
struct Config
{
    // identity
    std::string moduleName;
    std::string iniPath;
    std::string logPath;
    std::string stateDumpPath;

    VehicleSpecConfig   vehicle;
    TireConfig          tire;
    FrictionMatrixConfig friction;
    WeightTransferConfig weight;
    AeroConfig          aero;
    DrivetrainConfig    drivetrain;
    BrakeConfig         brakes;
    SteeringConfig      steering;
    MemoryConfig        memory;
    TelemetryConfig     telemetry;
    PhysicsConfig       physics;
    DebugConfig         debug;
    KeysConfig          keys;

    // Runtime state (not persisted)
    std::atomic<bool>  masterEnabled{ true };
    std::atomic<bool>  passiveMode{ false };
    std::atomic<bool>  reloadRequested{ false };
    // Set while the worker thread swaps the tunables.  The frame path checks
    // this first and skips one frame, so the solver never reads a half-written
    // table during a hot reload.
    std::atomic<bool>  reloadInProgress{ false };

    static Config& Instance() noexcept;

    // Parses `path`. Missing file -> defaults + bIsDefaultOut = true.
    bool Load(const std::string& path) noexcept;
    bool Save(const std::string& path) const noexcept;
    void ApplyDefaultsToTunables() noexcept;
    void Sanitize() noexcept;

    bool HasFileChanged() const noexcept;
    void WatchFile() noexcept;      // called once per frame from the module tick
};

} // namespace fh4rpo
