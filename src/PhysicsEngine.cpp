// =============================================================================
//  FH4RPO - PhysicsEngine.cpp
//
//  The complete solver plus the four detours that feed it and write back.
//
//  Conventions used throughout (Forza body frame):
//      +X = right,  +Y = up,  +Z = forward
//      yaw    > 0  = turning left  (counter clockwise seen from above)
//      pitch  > 0  = nose up
//      roll   > 0  = right side down
//      slipAngle: sign follows the game (radians), magnitude drives the tables
//
//  Every stage is wrapped in SEH and every exported number is filtered through
//  ClampFinite, so a corrupt telemetry packet or an unexpected game state can
//  only ever cost one frame of fidelity - never a crash.
// =============================================================================
#include "fh4rpo/PhysicsEngine.hpp"
#include "fh4rpo/Log.hpp"
#include "fh4rpo/HookEngine.hpp"


#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef _WIN32
#  include <windows.h>
#endif

namespace fh4rpo {

namespace {

double NowSeconds() noexcept
{
    using namespace std::chrono;
    static const steady_clock::time_point origin = steady_clock::now();
    return duration<double>(steady_clock::now() - origin).count();
}

// Spring/damper parameters derived once per spec update.
struct SuspensionRig
{
    float cornerStiffnessNPerM = 60000.0f;
    float cornerDampingNsPerM  = 5200.0f;
    float maxTravelM           = 0.13f;
};

SuspensionRig g_rig{};

} // namespace

// =============================================================================
//  Construction
// =============================================================================
PhysicsEngine::PhysicsEngine() noexcept
{
    config_ = &Config::Instance();
    spec_ = config_->vehicle;
    stepper_.Configure(1.0 / 240.0, 8);
    steerLimiter_.Configure(20.0f, 24.0f);
    frontMatrix_.Fill(1.0f);
    rearMatrix_.Fill(1.0f);
}

PhysicsEngine::~PhysicsEngine() noexcept
{
    Shutdown();
}

void PhysicsEngine::Initialize() noexcept
{
    if (initialized_)
        return;

    config_ = &Config::Instance();
    spec_ = config_->vehicle;

    // ---- derived geometry ---------------------------------------------------
    if (spec_.cgToFrontM <= 0.0f)
        spec_.cgToFrontM = spec_.wheelbaseM * (1.0f - spec_.weightDistribution);
    if (spec_.cgToFrontM >= spec_.wheelbaseM)
        spec_.cgToFrontM = spec_.wheelbaseM * 0.5f;

    UpdateVehicleSpec(spec_);

    // ---- friction matrices --------------------------------------------------
    frontMatrix_.Set(config_->friction.frontAngleKeysDeg, 6,
                     config_->friction.loadKeysN, 4,
                     &config_->friction.frontValues[0][0]);
    rearMatrix_.Set(config_->friction.rearAngleKeysDeg, 6,
                    config_->friction.loadKeysN, 4,
                    &config_->friction.rearValues[0][0]);

    stepper_.Configure(1.0 / ClampFinite(config_->physics.fixedStepHz, 30.0f, 1000.0f, 240.0f),
                       config_->physics.maxSubSteps);
    steerLimiter_.Configure(config_->steering.autosteerRate, config_->steering.returnRate);

    telemetryAccel_ = Vec3();
    modelAccel_ = Vec3();
    ResetDynamics();
    diagnostics_ = PhysicsDiagnostics{};
    diagnostics_.driveLayout = spec_.driveLayout;

    initialized_ = true;
    FH4RPO_LOG_INFO("PhysicsEngine: initialised (mass %.0f kg, wheelbase %.2f m, layout %d, %.0f Hz)",
                    spec_.massKg, spec_.wheelbaseM, spec_.driveLayout,
                    static_cast<double>(config_->physics.fixedStepHz));
}

void PhysicsEngine::Shutdown() noexcept
{
    if (!initialized_)
        return;
    initialized_ = false;
    FH4RPO_LOG_INFO("PhysicsEngine: shut down (%llu updates, %llu guard trips)",
                    static_cast<unsigned long long>(diagnostics_.updates),
                    static_cast<unsigned long long>(diagnostics_.guardTrips));
}

bool PhysicsEngine::IsInitialized() const noexcept { return initialized_; }

const WheelState& PhysicsEngine::Wheel(int index) const noexcept
{
    static const WheelState empty{};
    if (index < 0 || index >= kWheelCount)
        return empty;
    return wheels_[index];
}

void PhysicsEngine::UpdateVehicleSpec(const VehicleSpecConfig& source) noexcept
{
    FH4RPO_TRY
    {
        ApplyVehicleSpec(source);
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        MemoryGuard::OnTrip();
        diagnostics_.guardTrips++;
        SetEvent("spec update guarded");
    }
}

void PhysicsEngine::ApplyVehicleSpec(const VehicleSpecConfig& spec) noexcept
{
    spec_ = spec;
    spec_.wheelbaseM        = ClampFinite(spec_.wheelbaseM, 1.4f, 5.4f, 2.70f);
    spec_.massKg            = ClampFinite(spec_.massKg, 300.0f, 12000.0f, 1450.0f);
    spec_.cgHeightM         = ClampFinite(spec_.cgHeightM, 0.15f, 1.10f, 0.52f);
    spec_.weightDistribution = ClampFinite(spec_.weightDistribution, 0.25f, 0.75f, 0.53f);

    if (spec_.cgToFrontM <= 0.0f || spec_.cgToFrontM >= spec_.wheelbaseM)
        spec_.cgToFrontM = spec_.wheelbaseM * (1.0f - spec_.weightDistribution);

    wheelbaseEffectiveM_ = spec_.wheelbaseM;

    wheels_[WheelFL].wheelRadiusM = spec_.wheelRadiusFrontM;
    wheels_[WheelFR].wheelRadiusM = spec_.wheelRadiusFrontM;
    wheels_[WheelRL].wheelRadiusM = spec_.wheelRadiusRearM;
    wheels_[WheelRR].wheelRadiusM = spec_.wheelRadiusRearM;

    // Corner spring rate inferred from the axle-level roll / pitch stiffness.
    const float cornerCount = 4.0f;
    g_rig.cornerStiffnessNPerM = ClampFinite(
        (spec_.rollStiffnessNmPerRad + spec_.pitchStiffnessNmPerRad) / (2.0f * cornerCount),
        5000.0f, 600000.0f, 60000.0f);
    g_rig.cornerDampingNsPerM = ClampFinite(
        (spec_.rollDampingNmsPerRad + spec_.pitchDampingNmsPerRad) / (2.0f * cornerCount),
        100.0f, 200000.0f, 5200.0f);
    g_rig.maxTravelM = spec_.maxSuspensionTravelM;

    // Static corner loads
    const float totalWeightN = spec_.massKg * kGravity;
    const float frontAxleN   = totalWeightN * spec_.weightDistribution;
    const float rearAxleN    = totalWeightN - frontAxleN;

    wheels_[WheelFL].staticLoadN = frontAxleN * 0.5f;
    wheels_[WheelFR].staticLoadN = frontAxleN * 0.5f;
    wheels_[WheelRL].staticLoadN = rearAxleN * 0.5f;
    wheels_[WheelRR].staticLoadN = rearAxleN * 0.5f;

    for (int i = 0; i < kWheelCount; ++i)
    {
        wheels_[i].connected = true;
        if (wheels_[i].normalLoadN <= 0.0f)
            wheels_[i].normalLoadN = wheels_[i].staticLoadN;
    }
}

void PhysicsEngine::ResetDynamics() noexcept
{
    body_ = BodyState{};
    transfer_ = WeightTransferResult{};
    aero_ = AeroResult{};
    for (int i = 0; i < kWheelCount; ++i)
    {
        const float radius = wheels_[i].wheelRadiusM > 0.0f ? wheels_[i].wheelRadiusM : 0.34f;
        const float staticLoad = wheels_[i].staticLoadN;
        wheels_[i].Reset();
        wheels_[i].connected = true;
        wheels_[i].wheelRadiusM = radius;
        wheels_[i].staticLoadN = staticLoad;
        wheels_[i].normalLoadN = staticLoad;
        wheels_[i].tireTempC = 45.0f;
        wheels_[i].matrixMultiplier = 1.0f;
    }

    stepper_.Reset();
    smoothedDownforceN_ = 0.0f;
    smoothedDragN_ = 0.0f;
    smoothedFrontGrip_ = 1.0f;
    smoothedRearGrip_ = 1.0f;
    prevForwardAccel_ = 0.0f;
    prevLateralAccel_ = 0.0f;
    brakeTempC_ = 320.0f;
    lastGear_ = 0;
    hasStagedSample_ = false;
    hasSample_ = false;
    lastSample_ = TelemetrySample{};
}

void PhysicsEngine::SetEvent(const char* text) noexcept
{
    if (text == nullptr)
        return;
    std::snprintf(diagnostics_.lastEvent, sizeof(diagnostics_.lastEvent), "%s", text);
}

// =============================================================================
//  Telemetry ingestion
// =============================================================================
void PhysicsEngine::SubmitTelemetry(const TelemetrySample& sample) noexcept
{
    if (!sample.isValid)
        return;

    // SEH: a malformed sample must not poison the solver.
    FH4RPO_TRY
    {
        stagedSample_ = sample;
        hasStagedSample_ = true;
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        MemoryGuard::OnTrip();
        diagnostics_.guardTrips++;
        hasStagedSample_ = false;
    }
}

void PhysicsEngine::AttachReceiver(TelemetryReceiver* receiver) noexcept
{
    receiver_ = receiver;
}

void PhysicsEngine::AcquireTelemetry(double /*frameDt*/) noexcept
{
    // Pull the newest decoded sample from the network thread, then use it.
    if (receiver_ != nullptr)
    {
        TelemetrySample newest{};
        if (receiver_->PollLatest(newest) && newest.isValid)
        {
            stagedSample_ = newest;
            hasStagedSample_ = true;
        }
    }

    if (!hasStagedSample_)
        return;

    lastSample_ = stagedSample_;
    hasStagedSample_ = false;
    hasSample_ = true;
    lastSampleTime_ = clock_;
}

void PhysicsEngine::UpdateBodyFromTelemetry(const TelemetrySample& sample, float dt) noexcept
{
    if (!sample.isValid || dt <= 0.0f)
        return;

    // ---- teleport / fast-travel detection -----------------------------------
    const Vec3 newPosition(sample.velocity[0], sample.velocity[1], sample.velocity[2]);
    const float speedFromComponents = newPosition.Length2D();
    body_.speedMps = ClampFinite(std::max(sample.speedMps, speedFromComponents), 0.0f,
                                 config_->physics.maxSpeedMps, 0.0f);

    body_.yaw   = ClampFinite(sample.yaw, -kTwoPi * 4.0f, kTwoPi * 4.0f, 0.0f);
    body_.pitch = ClampFinite(sample.pitch, -kPi, kPi, 0.0f);
    body_.roll  = ClampFinite(sample.roll, -kPi, kPi, 0.0f);

    body_.velocity = newPosition;

    const float sinYaw = std::sin(body_.yaw);
    const float cosYaw = std::cos(body_.yaw);
    const Vec3 forward(sinYaw, 0.0f, cosYaw);
    const Vec3 right(cosYaw, 0.0f, -sinYaw);

    body_.forwardSpeedMps = body_.velocity.Dot(forward);
    body_.lateralSpeedMps = body_.velocity.Dot(right);
    body_.yawRateRad      = ClampFinite(sample.angularVel[1], -20.0f, 20.0f, 0.0f);

    // Body slip angle: the angle between where the car points and where it goes.
    body_.bodySlipAngleRad = ClampFinite(std::atan2(body_.lateralSpeedMps,
                                                    std::fabs(body_.forwardSpeedMps) + 1.0f),
                                         -kPi * 0.5f, kPi * 0.5f, 0.0f);

    // ---- driver inputs ------------------------------------------------------
    body_.throttle  = Saturate(sample.throttle);
    body_.brake     = Saturate(sample.brake);
    body_.clutch    = Saturate(sample.clutch);
    body_.handbrake = Saturate(sample.handbrake);

    // ---- steering: speed sensitive limit + counter steer assist ------------
    const float speedFactor = Saturate(body_.speedMps * config_->steering.speedSensitivity);
    const float maxAngleDeg = Lerp(config_->steering.maxSteerAngleDeg,
                                   config_->steering.minAngleAtSpeedDeg, speedFactor);
    float steerTargetRad = sample.steerInput * maxAngleDeg * kDegToRad;

    if (config_->steering.counterSteerAssist > 0.0f)
    {
        const float slipDeg = body_.bodySlipAngleRad * kRadToDeg;
        if (std::fabs(slipDeg) > config_->steering.assistSlipDeadzoneDeg &&
            body_.speedMps > 4.0f)
        {
            const float excess = std::fabs(slipDeg) - config_->steering.assistSlipDeadzoneDeg;
            const float assist = SignOf(slipDeg) * ClampFinite(excess * 0.35f, 0.0f, 12.0f, 0.0f) * kDegToRad;
            steerTargetRad += assist * config_->steering.counterSteerAssist;
        }
    }

    steerTargetRad = ClampFinite(steerTargetRad, -kPi * 0.33f, kPi * 0.33f, 0.0f);
    body_.steerAngleRad = steerLimiter_.Update(body_.steerAngleRad, steerTargetRad, dt);

    // ---- accelerations ------------------------------------------------------
    telemetryAccel_ = Vec3(ClampFinite(sample.accel[0], -120.0f, 120.0f, 0.0f),
                           ClampFinite(sample.accel[1], -120.0f, 120.0f, 0.0f),
                           ClampFinite(sample.accel[2], -120.0f, 120.0f, 0.0f));
    body_.accel = telemetryAccel_;
    prevForwardAccel_ = ExponentialApproach(prevForwardAccel_, body_.accel.z, 0.03f, dt);
    prevLateralAccel_ = ExponentialApproach(prevLateralAccel_, body_.accel.x, 0.03f, dt);

    // ---- per wheel ----------------------------------------------------------
    const float filterTau = 1.0f / ClampFinite(config_->physics.slipAngleFilterHz, 1.0f, 480.0f, 40.0f);

    for (int i = 0; i < kWheelCount; ++i)
    {
        WheelState& wheel = wheels_[i];

        wheel.rawSlipAngleRad = ClampFinite(sample.slipAngle[i], -kPi, kPi, 0.0f);
        wheel.rawSlipRatio    = ClampFinite(sample.slipRatio[i], -10.0f, 10.0f, 0.0f);

        const float previous = wheel.slipAngleRad;
        wheel.slipAngleRad = ExponentialApproach(wheel.slipAngleRad, wheel.rawSlipAngleRad, filterTau, dt);
        wheel.slipAngleRate = SafeDiv(wheel.slipAngleRad - previous, dt);

        wheel.slipRatio = ExponentialApproach(wheel.slipRatio, wheel.rawSlipRatio, filterTau, dt);

        wheel.suspensionTravelM = ClampFinite(sample.suspensionTravel[i], -0.5f, 0.5f, 0.0f);
        wheel.normalizedTravel = ClampFinite(sample.normalizedTravel[i], 0.0f, 1.0f, 0.5f);
        wheel.onRumbleStrip = sample.wheelRumble[i];
        wheel.tireTempC = ExponentialApproach(wheel.tireTempC, sample.tireTempC[i], 0.5f, dt);
        wheel.wheelSpeedMps = ClampFinite(sample.wheelSpeed[i], -400.0f, 400.0f, 0.0f);

        // Surface friction from the rumble / puddle channels.
        float surface = config_->tire.surfaceFrictionRoad;
        if (wheel.onRumbleStrip)
            surface = config_->tire.surfaceFrictionDirt;
        const float wetness = ClampFinite(sample.surfaceRumble[i], 0.0f, 1.0f, 0.0f);
        surface = Lerp(surface, config_->tire.surfaceFrictionWet, wetness * 0.5f);
        wheel.surfaceFriction = ClampFinite(surface, 0.15f, 1.5f, 1.0f);
    }

    // ---- airborne detection -------------------------------------------------
    float travelSum = 0.0f;
    for (int i = 0; i < kWheelCount; ++i)
        travelSum += ClampFinite(sample.normalizedTravel[i], 0.0f, 1.0f, 0.0f);
    const bool allWheelsDrooped = travelSum < 0.08f;
    body_.airborne = allWheelsDrooped && body_.speedMps > 3.0f;

    body_.valid = true;
}

void PhysicsEngine::ExtrapolateBody(float dt) noexcept
{
    // Passive fallback: keep integrating the last known state so downforce,
    // weight transfer and the friction matrix degrade smoothly instead of
    // snapping to zero when Data Out is unavailable (menus, replays, pause).
    if (dt <= 0.0f)
        return;

    body_.velocity += body_.accel * dt;

    const float speed = body_.velocity.Length2D();
    body_.speedMps = ClampFinite(speed, 0.0f, config_->physics.maxSpeedMps, 0.0f);

    const float sinYaw = std::sin(body_.yaw);
    const float cosYaw = std::cos(body_.yaw);
    body_.forwardSpeedMps = body_.velocity.Dot(Vec3(sinYaw, 0.0f, cosYaw));
    body_.lateralSpeedMps = body_.velocity.Dot(Vec3(cosYaw, 0.0f, -sinYaw));
    body_.bodySlipAngleRad = ExponentialApproach(body_.bodySlipAngleRad, 0.0f, 0.6f, dt);

    // Natural decay of the slip estimates - no telemetry means no new slip.
    for (int i = 0; i < kWheelCount; ++i)
    {
        wheels_[i].slipAngleRad = ExponentialApproach(wheels_[i].slipAngleRad, 0.0f, 0.35f, dt);
        wheels_[i].slipRatio    = ExponentialApproach(wheels_[i].slipRatio, 0.0f, 0.35f, dt);
        wheels_[i].wheelSpeedMps = ExponentialApproach(wheels_[i].wheelSpeedMps, body_.speedMps, 0.25f, dt);
    }

    body_.accel = Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsEngine::DeriveVehicleSpec() noexcept
{
    if (!hasSample_ || !config_->telemetry.autoDeriveSpec)
        return;

    // The only reliable runtime signal is the drivetrain type; everything else
    // stays as configured (mass in particular must not be guessed).
    const int layout = Clamp<int>(lastSample_.drivetrainType, 0, 2);
    if (layout != spec_.driveLayout)
    {
        spec_.driveLayout = layout;
        diagnostics_.driveLayout = layout;
        FH4RPO_LOG_INFO("PhysicsEngine: drivetrain layout detected as %s",
                        layout == 0 ? "FWD" : (layout == 1 ? "RWD" : "AWD"));
    }
}

// =============================================================================
//  Stage 1 - dynamic weight transfer
// =============================================================================
void PhysicsEngine::SolveWeightTransfer(float dt) noexcept
{
    const WeightTransferConfig& cfg = config_->weight;
    const float mass = spec_.massKg;
    const float totalWeightN = mass * kGravity;
    const float wheelbase = std::max(spec_.wheelbaseM, 1.0f);
    const float cgHeight = spec_.cgHeightM;

    const float frontAxleN = totalWeightN * spec_.weightDistribution;
    const float rearAxleN  = totalWeightN - frontAxleN;

    transfer_.staticLoad[WheelFL] = frontAxleN * 0.5f;
    transfer_.staticLoad[WheelFR] = frontAxleN * 0.5f;
    transfer_.staticLoad[WheelRL] = rearAxleN * 0.5f;
    transfer_.staticLoad[WheelRR] = rearAxleN * 0.5f;

    if (!cfg.enabled)
    {
        for (int i = 0; i < kWheelCount; ++i)
        {
            wheels_[i].normalLoadN = transfer_.staticLoad[i];
            transfer_.dynamicLoad[i] = transfer_.staticLoad[i];
        }
        transfer_.pitchTargetRad = 0.0f;
        transfer_.rollTargetRad  = 0.0f;
        return;
    }

    // Longitudinal load transfer:  dFz = m * ax * h / L
    // The accelerations are taken from Data Out (telemetryAccel_) so the stage
    // is independent of whatever the integrator produced this substep.
    const float ax = ClampFinite(telemetryAccel_.z, -60.0f, 60.0f, 0.0f);
    const float ay = ClampFinite(telemetryAccel_.x, -60.0f, 60.0f, 0.0f);

    float longDelta = mass * ax * cgHeight / wheelbase * cfg.longTransferGain;
    longDelta = ClampFinite(longDelta, -totalWeightN * 0.85f, totalWeightN * 0.85f, 0.0f);

    // Lateral load transfer:  dFz = m * ay * h / track, split by roll stiffness.
    const float trackFront = std::max(spec_.frontTrackM, 0.8f);
    const float trackRear  = std::max(spec_.rearTrackM, 0.8f);

    const float rollStiffFront = std::max(spec_.rollStiffnessNmPerRad * 0.5f + spec_.antiRollBarFrontNmPerRad, 1.0f);
    const float rollStiffRear  = std::max(spec_.rollStiffnessNmPerRad * 0.5f + spec_.antiRollBarRearNmPerRad, 1.0f);
    const float rollSplit      = rollStiffFront / (rollStiffFront + rollStiffRear);

    const float latTotal = mass * ay * cgHeight * cfg.latTransferGain;
    const float latFront = ClampFinite(latTotal * rollSplit / trackFront, -totalWeightN, totalWeightN, 0.0f);
    const float latRear  = ClampFinite(latTotal * (1.0f - rollSplit) / trackRear, -totalWeightN, totalWeightN, 0.0f);

    // Positive ax (accelerating forward) sends load to the rear axle.
    const float frontAxleDynamic = frontAxleN - longDelta * 0.5f * 2.0f;
    const float rearAxleDynamic  = rearAxleN + longDelta * 0.5f * 2.0f;

    // Positive ay (accelerating to the right) loads the LEFT wheels.
    const float frontPerCorner = frontAxleDynamic * 0.5f;
    const float rearPerCorner  = rearAxleDynamic * 0.5f;

    float target[kWheelCount];
    target[WheelFL] = frontPerCorner + latFront * 0.5f;
    target[WheelFR] = frontPerCorner - latFront * 0.5f;
    target[WheelRL] = rearPerCorner + latRear * 0.5f;
    target[WheelRR] = rearPerCorner - latRear * 0.5f;

    // Anti-dive / anti-squat reduce the visible pitch without touching the loads.
    float pitchMoment = mass * ax * cgHeight;
    if (ax < 0.0f)
        pitchMoment *= (1.0f - cfg.antiDiveGain);
    else
        pitchMoment *= (1.0f - cfg.antiSquatGain);

    // Jacking effect (raised roll centre cars lift the inner wheels a little).
    if (std::fabs(cfg.jackingGain) > kEpsilon)
    {
        const float jack = std::fabs(latTotal) * cfg.jackingGain * 0.5f;
        target[WheelFL] += jack;
        target[WheelFR] += jack;
        target[WheelRL] += jack;
        target[WheelRR] += jack;
    }

    // Rate limit + floor, then commit.
    for (int i = 0; i < kWheelCount; ++i)
    {
        float desired = ClampFinite(target[i], 0.0f, totalWeightN * 1.2f, transfer_.staticLoad[i]);

        if (body_.airborne)
            desired = 0.0f;   // nothing on the tyres while flying

        const float maxStep = cfg.loadRateLimitN * dt;
        float current = wheels_[i].normalLoadN;
        if (current <= 0.0f)
            current = transfer_.staticLoad[i];

        const float delta = desired - current;
        if (std::fabs(delta) > maxStep)
            current += SignOf(delta) * maxStep;
        else
            current = desired;

        current = ClampFinite(current, body_.airborne ? 0.0f : cfg.minCornerLoadN,
                              totalWeightN * 1.5f, transfer_.staticLoad[i]);

        wheels_[i].normalLoadN = current;
        transfer_.dynamicLoad[i] = current;
    }

    transfer_.longitudinalDelta = longDelta;
    transfer_.lateralDelta      = latTotal;
    transfer_.loadTransferFrontN = latFront;
    transfer_.loadTransferRearN  = latRear;
    transfer_.totalNormalN = transfer_.dynamicLoad[0] + transfer_.dynamicLoad[1] +
                             transfer_.dynamicLoad[2] + transfer_.dynamicLoad[3];
    transfer_.pitchMomentNm = pitchMoment;
    transfer_.rollMomentNm  = mass * ay * cgHeight;
    transfer_.airborne      = body_.airborne;

    // Chassis attitude targets driven directly by the transfer solution.
    transfer_.pitchTargetRad = ClampFinite(
        SafeDiv(pitchMoment * cfg.pitchAccelGain, std::max(spec_.pitchStiffnessNmPerRad, 1.0f)) * kDegToRad * 57.29578f,
        -0.16f, 0.16f, 0.0f);
    transfer_.rollTargetRad = ClampFinite(
        -SafeDiv(transfer_.rollMomentNm * cfg.rollAccelGain, std::max(spec_.rollStiffnessNmPerRad, 1.0f)) * kDegToRad * 57.29578f,
        -0.20f, 0.20f, 0.0f);
}

// =============================================================================
//  Stage 2 - tyre model (Pacejka style magic formula + friction ellipse)
// =============================================================================
float PhysicsEngine::TireMagicFormula(float slip, float B, float C, float D, float E) const noexcept
{
    if (!IsFinite(slip) || !IsFinite(B) || !IsFinite(C) || !IsFinite(D) || !IsFinite(E))
        return 0.0f;

    const float Bx = B * slip;
    const float inner = Bx - E * (Bx - std::atan(Bx));
    const float value = D * std::sin(C * std::atan(inner));
    return IsFinite(value) ? value : 0.0f;
}

void PhysicsEngine::EvaluateTires(float dt) noexcept
{
    const TireConfig& tire = config_->tire;
    const float globalScale = config_->physics.globalGripScale;

    float frontGripSum = 0.0f;
    float rearGripSum  = 0.0f;

    for (int i = 0; i < kWheelCount; ++i)
    {
        WheelState& wheel = wheels_[i];
        const bool front = IsFrontAxle(i);

        const float load = std::max(wheel.normalLoadN, 0.0f);
        const float peakAngleRad = std::max(tire.slipAnglePeakDeg * kDegToRad, 0.01f);

        // ---- load sensitivity: mu falls as the tyre is pressed harder --------
        const float loadRatio = SafeDiv(load, std::max(tire.loadReferenceN, 1.0f), 0.0f);
        float mu = front ? tire.muNominalFront : tire.muNominalRear;
        mu *= 1.0f - tire.loadSensitivity * (loadRatio - 1.0f);
        mu = ClampFinite(mu, 0.15f, 2.5f, 1.0f);

        // ---- temperature window ---------------------------------------------
        const float tempDelta = std::fabs(wheel.tireTempC - tire.temperatureOptimumC);
        const float tempRatio = ClampFinite(1.0f - tempDelta / std::max(tire.temperatureWindowC, 1.0f),
                                            0.0f, 1.0f, 0.0f);
        wheel.gripScaleFromTemp = Lerp(tire.temperatureGripFloor, 1.0f, tempRatio);

        // ---- wear -------------------------------------------------------------
        const float wearScale = Lerp(1.0f, tire.wearGripFloor, Saturate(wheel.tireWear));

        float gripScale = mu * wheel.gripScaleFromTemp * wearScale *
                          wheel.surfaceFriction * globalScale;
        gripScale = ClampFinite(gripScale, 0.05f, 3.0f, 1.0f);

        // ---- apply the handling dynamic scaling matrix ----------------------
        gripScale *= ClampFinite(wheel.matrixMultiplier, 0.05f, 4.0f, 1.0f);

        // ---- assists ---------------------------------------------------------
        if (config_->physics.escSimulation && !front)
        {
            const float slipDeg = wheel.slipAngleRad * kRadToDeg;
            if (std::fabs(slipDeg) > config_->physics.escSlipThresholdDeg)
            {
                const float excess = std::fabs(slipDeg) - config_->physics.escSlipThresholdDeg;
                const float correction = 1.0f + Saturate(excess / 20.0f) * config_->physics.escCorrectionGain;
                gripScale *= correction;
            }
        }

        wheel.frictionMu = gripScale;

        // ---- normalised slips ------------------------------------------------
        const float normalisedAngle = SafeDiv(std::tan(ClampFinite(wheel.slipAngleRad, -1.4f, 1.4f, 0.0f)),
                                               std::tan(peakAngleRad), 0.0f);
        const float normalisedRatio = SafeDiv(wheel.slipRatio, std::max(tire.slipRatioPeak, 0.01f), 0.0f);

        const float combined = std::sqrt(normalisedAngle * normalisedAngle +
                                         normalisedRatio * normalisedRatio);

        const float D = gripScale * load;

        float lateral = TireMagicFormula(normalisedAngle, tire.bLat, tire.cLat, D, tire.eLat);
        float longitudinal = TireMagicFormula(normalisedRatio, tire.bLong, tire.cLong,
                                              D * tire.dLongScale, tire.eLong);

        // ---- friction ellipse: combined slip cannot exceed the available grip
        if (combined > 1.0f)
        {
            const float scale = 1.0f / combined;
            lateral *= scale;
            longitudinal *= scale;
        }

        if (load < 1.0f)
        {
            lateral = 0.0f;
            longitudinal = 0.0f;
        }

        // Sign convention: lateral force opposes the slip angle.
        wheel.lateralForceN = ClampFinite(-lateral, -D * 2.0f, D * 2.0f, 0.0f);
        wheel.longitudinalForceN = ClampFinite(longitudinal, -D * 2.0f, D * 2.0f, 0.0f);

        if (front)
            frontGripSum += gripScale;
        else
            rearGripSum += gripScale;
    }

    smoothedFrontGrip_ = ExponentialApproach(smoothedFrontGrip_, frontGripSum * 0.5f, 0.08f, dt);
    smoothedRearGrip_  = ExponentialApproach(smoothedRearGrip_, rearGripSum * 0.5f, 0.08f, dt);
    diagnostics_.frictionFrontMean = smoothedFrontGrip_;
    diagnostics_.frictionRearMean  = smoothedRearGrip_;
}

// =============================================================================
//  Stage 3 - the friction multiplier matrix (Handling Dynamic Scaling)
// =============================================================================
void PhysicsEngine::ApplyFrictionMatrix(float dt) noexcept
{
    const FrictionMatrixConfig& cfg = config_->friction;

    if (!cfg.enabled)
    {
        for (int i = 0; i < kWheelCount; ++i)
            wheels_[i].matrixMultiplier = 1.0f;
        return;
    }

    const float fadeTau = 1.0f / std::max(cfg.fadeRate, 0.01f);

    for (int i = 0; i < kWheelCount; ++i)
    {
        WheelState& wheel = wheels_[i];
        const bool front = IsFrontAxle(i);

        // Angle key: magnitude in degrees (the tables are symmetric by default).
        float angleDeg = wheel.slipAngleRad * kRadToDeg;
        if (!cfg.asymmetric)
            angleDeg = std::fabs(angleDeg);
        angleDeg = ClampFinite(angleDeg, -90.0f, 90.0f, 0.0f);

        const float loadKey = cfg.useLoadAxis ? wheel.normalLoadN : 0.0f;

        const float target = front ? frontMatrix_.Sample(angleDeg, loadKey)
                                   : rearMatrix_.Sample(angleDeg, loadKey);

        const float sanitised = ClampFinite(target, 0.10f, 4.0f, 1.0f);
        wheel.matrixMultiplier = ExponentialApproach(wheel.matrixMultiplier, sanitised, fadeTau, dt);
        wheel.matrixMultiplier = ClampFinite(wheel.matrixMultiplier, 0.10f, 4.0f, 1.0f);
    }

    diagnostics_.matrixWrites++;
}

float PhysicsEngine::EvaluateFrictionMultiplier(int wheelIndex, float slipAngleRad,
                                               float normalLoadN) noexcept
{
    const FrictionMatrixConfig& cfg = config_->friction;
    if (!cfg.enabled || !config_->masterEnabled.load(std::memory_order_relaxed))
        return 1.0f;

    if (!IsFinite(slipAngleRad) || !IsFinite(normalLoadN))
        return 1.0f;

    FH4RPO_TRY
    {
        float angleDeg = ClampFinite(slipAngleRad * kRadToDeg, -90.0f, 90.0f, 0.0f);
        if (!cfg.asymmetric)
            angleDeg = std::fabs(angleDeg);

        const float loadKey = cfg.useLoadAxis ? ClampFinite(normalLoadN, 0.0f, 40000.0f, 0.0f) : 0.0f;
        const bool front = (wheelIndex == WheelFL || wheelIndex == WheelFR || wheelIndex < 0);

        const float multiplier = front ? frontMatrix_.Sample(angleDeg, loadKey)
                                       : rearMatrix_.Sample(angleDeg, loadKey);

        diagnostics_.hookCalls++;
        return ClampFinite(multiplier, 0.10f, 4.0f, 1.0f);
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        MemoryGuard::OnTrip();
        diagnostics_.guardTrips++;
        return 1.0f;
    }
}

// =============================================================================
//  Stage 4 - aerodynamics
// =============================================================================
void PhysicsEngine::SolveAero(float dt) noexcept
{
    FH4RPO_TRY
    {
        SolveAeroStage(dt);
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        MemoryGuard::OnTrip();
        diagnostics_.guardTrips++;
        aero_ = AeroResult{};
        SetEvent("aero guarded");
    }
}

void PhysicsEngine::SolveAeroStage(float dt) noexcept
{
    const AeroConfig& cfg = config_->aero;

    if (!cfg.enabled)
    {
        aero_ = AeroResult{};
        return;
    }

    const float speed = ClampFinite(body_.speedMps, 0.0f, config_->physics.maxSpeedMps, 0.0f);
    const float rho = cfg.airDensityKgM3;
    const float area = cfg.frontalAreaM2;

    // Dynamic pressure: q = 0.5 * rho * v^2
    const float q = 0.5f * rho * speed * speed;
    const float referenceHeight = std::max(spec_.rideHeightM, 0.02f);
    const float height = ClampFinite(spec_.rideHeightM, 0.01f, 0.5f, referenceHeight);

    // Ground effect: downforce grows as the car gets closer to the tarmac.
    const float clearanceRatio = ClampFinite((referenceHeight - height + cfg.groundEffectHeightM) /
                                             std::max(cfg.groundEffectHeightM, 0.005f),
                                             0.0f, 4.0f, 1.0f);
    const float rideScale = 1.0f + cfg.groundEffectGain * ClampFinite(clearanceRatio, 0.0f, 2.0f, 1.0f);

    // Stall: aero efficiency collapses past the wing's critical speed.
    float stallScale = 1.0f;
    if (speed > cfg.stallSpeedMps)
    {
        const float excess = (speed - cfg.stallSpeedMps) / std::max(cfg.stallSpeedMps, 1.0f);
        stallScale = 1.0f - Saturate(excess * cfg.stallFalloff * 4.0f) * 0.6f;
        stallScale = ClampFinite(stallScale, 0.25f, 1.0f, 1.0f);
    }

    const float clTotal = cfg.liftCoefficientFront * cfg.balanceSplit +
                          cfg.liftCoefficientRear * (1.0f - cfg.balanceSplit);

    float totalDownforce = q * area * clTotal * rideScale * stallScale;
    totalDownforce = ClampFinite(totalDownforce, -cfg.maxDownforceN, cfg.maxDownforceN, 0.0f);

    const float drag = ClampFinite(q * area * cfg.dragCoefficient, 0.0f, cfg.maxDragN, 0.0f);

    // Side force from yaw / body slip (the car is pushed sideways when sliding).
    const float sideForce = ClampFinite(cfg.yawSensitivity * q * area * std::sin(body_.bodySlipAngleRad),
                                        -cfg.maxDownforceN, cfg.maxDownforceN, 0.0f);

    aero_.dynamicPressure   = q;
    aero_.rideHeightScale   = rideScale;
    aero_.stallScale        = stallScale;
    aero_.downforceN        = totalDownforce;
    aero_.downforceFrontN   = totalDownforce * cfg.balanceSplit;
    aero_.downforceRearN    = totalDownforce * (1.0f - cfg.balanceSplit);
    aero_.dragN             = drag;
    aero_.sideForceN        = sideForce;
    aero_.pitchMomentNm     = (aero_.downforceRearN - aero_.downforceFrontN) * spec_.wheelbaseM * 0.25f;

    smoothedDownforceN_ = ExponentialApproach(smoothedDownforceN_, totalDownforce,
                                              std::max(cfg.smoothTimeS, 0.001f), dt);
    smoothedDragN_ = ExponentialApproach(smoothedDragN_, drag, std::max(cfg.smoothTimeS, 0.001f), dt);

    aero_.downforceN = smoothedDownforceN_;
    aero_.dragN      = smoothedDragN_;

    // Downforce raises the tyre load - feed it into the corner loads so the
    // grip model actually feels the wing.
    if (config_->physics.applySuspensionForces && !body_.airborne)
    {
        const float totalWeightN = spec_.massKg * kGravity;
        for (int i = 0; i < kWheelCount; ++i)
        {
            const float share = IsFrontAxle(i) ? (0.5f * cfg.balanceSplit)
                                               : (0.5f * (1.0f - cfg.balanceSplit));
            const float extra = smoothedDownforceN_ * share;
            wheels_[i].normalLoadN = ClampFinite(wheels_[i].normalLoadN + extra * 0.5f,
                                                 0.0f, totalWeightN * 3.0f,
                                                 wheels_[i].normalLoadN);
        }
    }
}

float PhysicsEngine::EvaluateDownforce(float gameDownforceN, float speedMps, float rideHeightM) noexcept
{
    const AeroConfig& cfg = config_->aero;
    if (!cfg.enabled || !cfg.overrideGameDownforce ||
        !config_->masterEnabled.load(std::memory_order_relaxed))
    {
        return gameDownforceN;
    }

    FH4RPO_TRY
    {
        const float speed = ClampFinite(speedMps, 0.0f, config_->physics.maxSpeedMps,
                                        body_.speedMps);
        const float height = ClampFinite(rideHeightM, 0.005f, 1.0f, spec_.rideHeightM);

        // F = 0.5 * rho * v^2 * C_l * A  with a ground effect and stall modifier.
        const float q = 0.5f * cfg.airDensityKgM3 * speed * speed;
        const float clearance = ClampFinite(cfg.groundEffectHeightM / std::max(height, 0.005f),
                                            0.0f, 3.0f, 1.0f);
        const float rideScale = 1.0f + cfg.groundEffectGain * clearance;

        float stallScale = 1.0f;
        if (speed > cfg.stallSpeedMps)
        {
            const float excess = (speed - cfg.stallSpeedMps) / std::max(cfg.stallSpeedMps, 1.0f);
            stallScale = ClampFinite(1.0f - Saturate(excess * 4.0f) * cfg.stallFalloff, 0.3f, 1.0f, 1.0f);
        }

        const float clTotal = cfg.liftCoefficientFront * cfg.balanceSplit +
                              cfg.liftCoefficientRear * (1.0f - cfg.balanceSplit);

        float computed = q * cfg.frontalAreaM2 * clTotal * rideScale * stallScale;
        computed = ClampFinite(computed, -cfg.maxDownforceN, cfg.maxDownforceN, gameDownforceN);

        // Aero balance bias: keep the game's sign convention where possible.
        const float bias = (gameDownforceN < 0.0f) ? -1.0f : 1.0f;
        const float result = computed * bias;

        diagnostics_.aeroOverrides++;
        return IsFinite(result) ? result : gameDownforceN;
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        MemoryGuard::OnTrip();
        diagnostics_.guardTrips++;
        return gameDownforceN;
    }
}

// =============================================================================
//  Stage 5 - drivetrain
// =============================================================================
void PhysicsEngine::SolveDrivetrain(float dt) noexcept
{
    const DrivetrainConfig& cfg = config_->drivetrain;

    for (int i = 0; i < kWheelCount; ++i)
        wheels_[i].driveTorqueNm = 0.0f;

    if (!cfg.enabled || !config_->physics.applyDrivetrain)
        return;

    const float rpm = hasSample_ ? ClampFinite(lastSample_.rpm, 0.0f, 20000.0f, 0.0f) : 0.0f;
    const float maxRpm = hasSample_ ? ClampFinite(lastSample_.maxRpm, 500.0f, 20000.0f, 8000.0f) : 8000.0f;

    // Simple asymmetric torque curve: peak at torqueCurvePeakRpm.
    float curve = 0.0f;
    if (rpm > 1.0f)
    {
        const float x = (rpm - cfg.torqueCurvePeakRpm) / std::max(cfg.torqueCurveWidthRpm, 1.0f);
        curve = std::exp(-0.5f * x * x);
        curve = ClampFinite(curve, 0.15f, 1.0f, 0.5f);
    }

    const float throttle = Saturate(body_.throttle);
    float engineTorque = cfg.maxEngineTorqueNm * curve * throttle;

    // Engine braking / off throttle drag through the driveline.
    if (throttle < 0.05f)
        engineTorque -= cfg.maxEngineTorqueNm * cfg.engineBrakingGain * Saturate(1.0f - throttle) *
                        Saturate(rpm / std::max(maxRpm, 1.0f)) * 0.25f;

    if (body_.clutch > 0.85f)
        engineTorque *= 0.1f;

    engineTorque *= cfg.clutchEngagement;
    engineTorque = ClampFinite(engineTorque, -cfg.maxEngineTorqueNm, cfg.maxEngineTorqueNm, 0.0f);

    if (hasSample_)
        lastGear_ = lastSample_.gear;

    // TCS: trim torque when the driven axle is spinning up.
    if (config_->physics.tcsSimulation)
    {
        float drivenSlip = 0.0f;
        int drivenCount = 0;
        for (int i = 0; i < kWheelCount; ++i)
        {
            const bool driven = (spec_.driveLayout == 0) ? IsFrontAxle(i)
                              : (spec_.driveLayout == 1) ? IsRearAxle(i) : true;
            if (driven)
            {
                drivenSlip += std::fabs(wheels_[i].slipRatio);
                ++drivenCount;
            }
        }
        if (drivenCount > 0)
        {
            drivenSlip /= static_cast<float>(drivenCount);
            if (drivenSlip > config_->physics.tcsSlipTarget)
            {
                const float excess = drivenSlip - config_->physics.tcsSlipTarget;
                engineTorque *= Saturate(1.0f - excess * 3.5f);
            }
        }
    }

    // Split front / rear, with a rear bias while launching.
    float frontShare = 0.0f;
    float rearShare  = 1.0f;

    switch (spec_.driveLayout)
    {
    case 0:  frontShare = 1.0f; rearShare = 0.0f; break;
    case 1:  frontShare = 0.0f; rearShare = 1.0f; break;
    default: frontShare = cfg.torqueSplitFront; rearShare = 1.0f - cfg.torqueSplitFront; break;
    }

    if (body_.speedMps < cfg.launchSpeedMps)
    {
        const float launchBlend = 1.0f - Saturate(body_.speedMps / std::max(cfg.launchSpeedMps, 0.1f));
        rearShare = Saturate(rearShare + cfg.rearBiasOnLaunch * launchBlend);
        frontShare = Saturate(1.0f - rearShare);
    }

    for (int i = 0; i < kWheelCount; ++i)
    {
        const float share = IsFrontAxle(i) ? frontShare : rearShare;
        const float axleTorque = engineTorque * share;
        const float axleGear = IsFrontAxle(i) ? spec_.frontGearRatioDrive : spec_.rearGearRatioDrive;
        const float wheelTorque = axleTorque * ClampFinite(axleGear, 0.5f, 12.0f, 4.10f) * 0.5f;

        // Counter-torque from the slip direction keeps the model stable.
        const float damp = ClampFinite(-wheels_[i].slipRatio * 40.0f * dt, -wheelTorque, wheelTorque);

        wheels_[i].driveTorqueNm = ClampFinite(wheelTorque + damp, -20000.0f, 20000.0f, 0.0f);
    }
}

// =============================================================================
//  Stage 6 - brakes
// =============================================================================
void PhysicsEngine::SolveBrakes(float dt) noexcept
{
    const BrakeConfig& cfg = config_->brakes;

    for (int i = 0; i < kWheelCount; ++i)
        wheels_[i].brakeTorqueNm = 0.0f;

    if (!cfg.enabled || !config_->physics.applyBrakes)
        return;

    // Brake fade: torque capacity drops with disk temperature.
    const float fade = Saturate(1.0f - std::max(0.0f, brakeTempC_ - cfg.brakeFadeStartC) *
                                       cfg.brakeFadeRate * 0.01f);
    const float capacity = cfg.maxBrakeTorqueNm * ClampFinite(fade, 0.35f, 1.0f, 1.0f);

    const float bias = ClampFinite(cfg.frontBias, 0.05f, 0.95f, 0.62f);
    const float pedal = Saturate(body_.brake);

    // ABS: modulate the front/rear split when the wheels lock.
    float absScale = 1.0f;
    if (pedal > 0.85f)
    {
        const float cycle = std::sin(static_cast<float>(clock_) * kTwoPi * cfg.absCycleHz);
        float worstSlip = 0.0f;
        for (int i = 0; i < kWheelCount; ++i)
            worstSlip = std::max(worstSlip, std::fabs(wheels_[i].slipRatio));

        if (worstSlip > cfg.absSlipThreshold)
            absScale = 0.85f + 0.15f * cycle;
    }

    for (int i = 0; i < kWheelCount; ++i)
    {
        const float share = IsFrontAxle(i) ? bias : (1.0f - bias);
        float torque = capacity * share * pedal * absScale;

        if (IsRearAxle(i) && body_.handbrake > 0.1f)
            torque += cfg.handbrakeTorqueNm * body_.handbrake;

        wheels_[i].brakeTorqueNm = ClampFinite(torque, 0.0f, cfg.maxBrakeTorqueNm * 2.0f, 0.0f);
    }

    // Disk heating roughly follows braking power, cooling follows airflow.
    const float brakingPower = pedal * 0.6f + 0.05f * Saturate(body_.speedMps / 60.0f);
    brakeTempC_ += (brakingPower * 260.0f - (brakeTempC_ - 35.0f) * 0.35f) * dt;
    brakeTempC_ = ClampFinite(brakeTempC_, 20.0f, 1200.0f, 320.0f);
}

// =============================================================================
//  Stage 7 - suspension
// =============================================================================
void PhysicsEngine::SolveSuspension(float dt) noexcept
{
    if (!config_->physics.applySuspensionForces)
        return;

    const float stiffness = g_rig.cornerStiffnessNPerM;
    const float damping   = g_rig.cornerDampingNsPerM;
    const float maxTravel = std::max(g_rig.maxTravelM, 0.01f);

    for (int i = 0; i < kWheelCount; ++i)
    {
        WheelState& wheel = wheels_[i];

        // Travel x is negative in compression in Forza's convention.
        const float compression = ClampFinite(-wheel.suspensionTravelM, -maxTravel, maxTravel, 0.0f);
        const float velocity = SafeDiv(compression - ClampFinite(wheel.suspensionTravelM, -maxTravel, maxTravel, 0.0f),
                                      std::max(dt, 0.001f), 0.0f);

        float force = stiffness * compression + damping * velocity;

        // Never push harder than the corner can actually support.
        const float ceiling = std::max(wheels_[i].normalLoadN * 4.0f, 1000.0f);
        force = ClampFinite(force, -ceiling, ceiling, 0.0f);

        wheel.suspensionForceN = force;
        wheel.suspensionVelocityM = ExponentialApproach(wheel.suspensionVelocityM, velocity, 0.02f, dt);
    }
}

// =============================================================================
//  Stage 8 - integration
// =============================================================================
void PhysicsEngine::Integrate(float dt) noexcept
{
    if (dt <= 0.0f)
        return;

    const bool telemetryFresh = hasSample_ && ((clock_ - lastSampleTime_) < 0.35);

    // Sum the tyre forces into a body acceleration.  This is what keeps the
    // model honest: the numbers are recomputed from the *game's* state every
    // frame, never accumulated blindly.
    float forceX = 0.0f;
    float forceZ = 0.0f;
    float yawMoment = 0.0f;

    for (int i = 0; i < kWheelCount; ++i)
    {
        const WheelState& wheel = wheels_[i];
        if (!wheel.connected)
            continue;

        const bool front = IsFrontAxle(i);
        const float longitudinal = wheel.longitudinalForceN;
        const float lateral      = wheel.lateralForceN;

        // Steering rotates the front tyre forces into the body frame.
        const float steer = front ? body_.steerAngleRad : 0.0f;
        const float cosSteer = std::cos(steer);
        const float sinSteer = std::sin(steer);

        const float bodyX = longitudinal * sinSteer + lateral * cosSteer;
        const float bodyZ = longitudinal * cosSteer - lateral * sinSteer;

        forceX += bodyX;
        forceZ += bodyZ;

        // Yaw moment = r * Fx_body  (+ track offset contribution).
        const float armX = (SideIndex(i) == 0 ? -1.0f : 1.0f) * (front ? spec_.frontTrackM : spec_.rearTrackM) * 0.5f;
        const float armZ = front ? spec_.cgToFrontM : -(spec_.wheelbaseM - spec_.cgToFrontM);
        yawMoment += (armZ * bodyX) - (armX * bodyZ);

    }

    // Aero drag + side force.
    forceZ -= smoothedDragN_ * (body_.forwardSpeedMps >= 0.0f ? 1.0f : -1.0f);
    forceX += aero_.sideForceN;

    // Gravity is already in the game's telemetry; only apply it in passive mode.
    if (!telemetryFresh)
        forceZ -= spec_.massKg * kGravity * std::sin(body_.pitch);

    const float mass = std::max(spec_.massKg, 1.0f);
    modelAccel_ = Vec3(ClampFinite(forceX / mass, -120.0f, 120.0f, 0.0f),
                       telemetryAccel_.y,
                       ClampFinite(forceZ / mass, -120.0f, 120.0f, 0.0f));

    // With live telemetry the game owns the body state: only the passive path
    // integrates, so the model can never fight the game's own solver.
    if (!telemetryFresh)
        body_.accel = modelAccel_;

    if (!telemetryFresh)
    {
        // Passive integration only - with live telemetry the game owns the state.
        body_.velocity.x += body_.accel.x * dt;
        body_.velocity.z += body_.accel.z * dt;
        body_.speedMps = ClampFinite(body_.velocity.Length2D(), 0.0f, config_->physics.maxSpeedMps, 0.0f);
    }

    // Chassis yaw / pitch / roll integration (visual + force feedback source).
    if (config_->physics.applyChassisTorque)
    {
        const float yawInertia = std::max(spec_.yawInertiaKgM2, 1.0f);
        const float yawAccel = ClampFinite(SafeDiv(yawMoment, yawInertia, 0.0f), -60.0f, 60.0f, 0.0f);
        body_.angularVelocity.y = ExponentialApproach(body_.angularVelocity.y, yawAccel, 0.05f, dt);
        body_.yaw += body_.angularVelocity.y * dt * 0.25f;
        body_.yaw = std::remainder(body_.yaw, kTwoPi);
    }
}

void PhysicsEngine::UpdateTireThermal(float dt) noexcept
{
    const TireConfig& tire = config_->tire;

    for (int i = 0; i < kWheelCount; ++i)
    {
        WheelState& wheel = wheels_[i];

        // Slip work heats the tyre; airflow cools it.
        const float slipWork = std::fabs(wheel.slipRatio) * 0.6f +
                               std::fabs(wheel.slipAngleRad) * 0.9f;
        const float heatInput = slipWork * wheel.normalLoadN * tire.heatGeneration * 0.001f;
        const float cooling = (wheel.tireTempC - 22.0f) * tire.coolingRate *
                              (0.4f + 0.6f * Saturate(body_.speedMps / 30.0f));

        wheel.tireTempC += (heatInput - cooling) * dt;
        wheel.tireTempC = ClampFinite(wheel.tireTempC, -20.0f, 320.0f, 60.0f);

        wheel.tireWear = Saturate(wheel.tireWear + slipWork * tire.wearRate * dt);
        wheel.rotationAngleRad += SafeDiv(wheel.wheelSpeedMps, std::max(wheel.wheelRadiusM, 0.05f), 0.0f) * dt;
        wheel.rotationAngleRad = std::remainder(wheel.rotationAngleRad, kTwoPi);
    }
}

void PhysicsEngine::UpdateChassisAttitude(float dt) noexcept
{
    const float pitchTarget = transfer_.pitchTargetRad;
    const float rollTarget  = transfer_.rollTargetRad;

    // Critically damped spring approach keeps the body from snapping.
    const float pitchTau = ClampFinite(SafeDiv(2.0f * spec_.pitchInertiaKgM2,
                                               std::max(spec_.pitchStiffnessNmPerRad, 1.0f), 0.2f) * 4.0f,
                                       0.02f, 1.5f, 0.20f);
    const float rollTau = ClampFinite(SafeDiv(2.0f * spec_.rollInertiaKgM2,
                                              std::max(spec_.rollStiffnessNmPerRad, 1.0f), 0.1f) * 4.0f,
                                      0.02f, 1.5f, 0.15f);

    body_.pitch = ExponentialApproach(body_.pitch, pitchTarget, pitchTau, dt);
    body_.roll  = ExponentialApproach(body_.roll, rollTarget, rollTau, dt);

    body_.pitch = ClampFinite(body_.pitch, -0.30f, 0.30f, 0.0f);
    body_.roll  = ClampFinite(body_.roll, -0.35f, 0.35f, 0.0f);

    transfer_.pitchTargetRad = body_.pitch;
    transfer_.rollTargetRad  = body_.roll;
}

// =============================================================================
//  Frame driver
// =============================================================================
void PhysicsEngine::Step(float dt) noexcept
{
    if (dt <= 0.0f)
        return;

    const auto started = NowSeconds();

    FH4RPO_TRY
    {
        SolveWeightTransfer(dt);
        EvaluateTires(dt);
        ApplyFrictionMatrix(dt);
        SolveAero(dt);
        SolveDrivetrain(dt);
        SolveBrakes(dt);
        SolveSuspension(dt);
        Integrate(dt);
        UpdateTireThermal(dt);
        UpdateChassisAttitude(dt);
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        MemoryGuard::OnTrip();
        diagnostics_.guardTrips++;
        SetEvent("solver stage guarded (one frame skipped)");
        ResetDynamics();
    }

    diagnostics_.simTime = clock_;
    diagnostics_.updateCostUs = static_cast<float>((NowSeconds() - started) * 1.0e6);
    diagnostics_.updates++;
}

void PhysicsEngine::OnFrame(double frameDt) noexcept
{
    if (!initialized_ || !config_->masterEnabled.load(std::memory_order_relaxed))
        return;

    // A config hot reload swaps whole tables on the worker thread; skip the
    // frame rather than reading a partially updated matrix.
    if (config_->reloadInProgress.load(std::memory_order_acquire))
        return;

    const double scaledDt = ClampFinite(frameDt * config_->physics.timeScale,
                                        0.0, static_cast<double>(kMaxFrameDt), 1.0 / 60.0);
    if (scaledDt <= 0.0)
        return;

    FH4RPO_TRY
    {
        clock_ += scaledDt;
        diagnostics_.frameDt = scaledDt;

        AcquireTelemetry(scaledDt);

        const bool telemetryFresh = hasSample_ && ((clock_ - lastSampleTime_) < 0.35);
        diagnostics_.telemetryLive = telemetryFresh;
        diagnostics_.passiveFallback = !telemetryFresh;

        if (telemetryFresh)
        {
            const float sourceDt = static_cast<float>(scaledDt);
            UpdateBodyFromTelemetry(lastSample_, sourceDt);
            DeriveVehicleSpec();
            SetEvent("telemetry");
        }
        else
        {
            if (!config_->physics.passiveModeAllowed &&
                !config_->passiveMode.load(std::memory_order_relaxed))
            {
                // Live telemetry is mandatory and absent: hold the current
                // state instead of inventing forces.
                diagnostics_.substeps = 0;
                return;
            }
            ExtrapolateBody(static_cast<float>(scaledDt));
            SetEvent("passive extrapolation");
        }

        const float dt = static_cast<float>(scaledDt);
        const int substeps = stepper_.Advance(scaledDt, [this, dt](float fixedDt)
        {
            (void)dt;
            Step(fixedDt);
        });
        diagnostics_.substeps = substeps;

        // Cache the write-back values for this frame.
        const float downforceOut = config_->aero.enabled ? aero_.downforceN : 0.0f;
        BuildGameWriteback(downforceOut,
                           ClampFinite((smoothedFrontGrip_ + smoothedRearGrip_) * 0.5f, 0.05f, 8.0f, 1.0f),
                           config_->brakes.frontBias,
                           body_.steerAngleRad * kRadToDeg);

        ApplyWriteback();
        WriteFrictionMatrix();
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        MemoryGuard::OnTrip();
        diagnostics_.guardTrips++;
        SetEvent("frame guarded");
    }
}

void PhysicsEngine::OnPresentTick(double frameDt) noexcept
{
    OnFrame(frameDt);
}

// =============================================================================
//  Write-back into the game
// =============================================================================
bool PhysicsEngine::BuildGameWriteback(float outDownforceN, float outGripScale,
                                       float outBrakeBias, float outSteerAngleDeg) noexcept
{
    wbDownforceN_    = ClampFinite(outDownforceN, -60000.0f, 60000.0f, 0.0f);
    wbGripScale_     = ClampFinite(outGripScale, 0.05f, 8.0f, 1.0f);
    wbBrakeBias_     = ClampFinite(outBrakeBias, 0.05f, 0.95f, 0.62f);
    wbSteerAngleDeg_ = ClampFinite(outSteerAngleDeg, -90.0f, 90.0f, 0.0f);
    writebackPending_ = true;
    return true;
}

void PhysicsEngine::RegisterTargets(const GameTargets& targets) noexcept
{
    targets_ = targets;
}

int PhysicsEngine::ApplyWriteback() noexcept
{
    if (!writebackPending_ || !config_->masterEnabled.load(std::memory_order_relaxed))
        return 0;

    int written = 0;
    writebackPending_ = false;

    if (targets_.downforceAddress != 0 && config_->aero.overrideGameDownforce)
    {
        if (MemoryGuard::WriteFloatAtomic(targets_.downforceAddress, wbDownforceN_) == MemStatus::Ok)
            ++written;
    }

    if (targets_.gripScaleAddress != 0)
    {
        if (MemoryGuard::WriteFloatAtomic(targets_.gripScaleAddress, wbGripScale_) == MemStatus::Ok)
            ++written;
    }

    if (targets_.brakeBiasAddress != 0 && config_->brakes.enabled)
    {
        if (MemoryGuard::WriteFloatAtomic(targets_.brakeBiasAddress, wbBrakeBias_) == MemStatus::Ok)
            ++written;
    }

    if (targets_.steerAngleAddress != 0 && config_->steering.enabled)
    {
        const float radians = wbSteerAngleDeg_ * kDegToRad;
        if (MemoryGuard::WriteFloatAtomic(targets_.steerAngleAddress, radians) == MemStatus::Ok)
            ++written;
    }

    if (written > 0)
        diagnostics_.matrixWrites++;

    return written;
}

int PhysicsEngine::WriteFrictionMatrix() noexcept
{
    if (targets_.frictionMatrixBase == 0 || !config_->friction.enabled)
        return 0;

    // Layout: 4 corners x (longitudinal, lateral) as float pairs.
    float matrix[16] = {};
    for (int i = 0; i < kWheelCount; ++i)
    {
        const float multiplier = ClampFinite(wheels_[i].matrixMultiplier, 0.10f, 4.0f, 1.0f);
        matrix[i * 4 + 0] = multiplier;   // longitudinal
        matrix[i * 4 + 1] = multiplier;   // lateral
        matrix[i * 4 + 2] = 1.0f;         // reserved
        matrix[i * 4 + 3] = 1.0f;         // reserved
    }

    const std::uintptr_t base = targets_.frictionMatrixBase;
    if (!MemoryGuard::IsRangeWritable(base, sizeof(matrix)))
        return 0;

    if (MemoryGuard::WriteBlock(base, matrix, sizeof(matrix)) != MemStatus::Ok)
    {
        diagnostics_.guardTrips++;
        return 0;
    }

    return kWheelCount;
}

// =============================================================================
//  Diagnostics
// =============================================================================
bool PhysicsEngine::DumpState(const char* path) const noexcept
{
    if (path == nullptr)
        return false;

    FH4RPO_TRY
    {
        std::FILE* file = nullptr;
#ifdef _MSC_VER
        if (fopen_s(&file, path, "w") != 0)
            file = nullptr;
#else
        file = std::fopen(path, "w");
#endif
        if (file == nullptr)
            return false;

        std::fprintf(file, "FH4RPO state dump\n");
        std::fprintf(file, "=================\n\n");

        std::fprintf(file, "[Clock]\n");
        std::fprintf(file, "sim_time=%.4f\n", diagnostics_.simTime);
        std::fprintf(file, "frame_dt=%.6f\n", diagnostics_.frameDt);
        std::fprintf(file, "substeps=%d\n", diagnostics_.substeps);
        std::fprintf(file, "update_cost_us=%.2f\n", diagnostics_.updateCostUs);
        std::fprintf(file, "updates=%llu\n", static_cast<unsigned long long>(diagnostics_.updates));
        std::fprintf(file, "guard_trips=%llu\n", static_cast<unsigned long long>(diagnostics_.guardTrips));
        std::fprintf(file, "hook_calls=%llu\n", static_cast<unsigned long long>(diagnostics_.hookCalls));
        std::fprintf(file, "aero_overrides=%llu\n", static_cast<unsigned long long>(diagnostics_.aeroOverrides));
        std::fprintf(file, "telemetry_live=%d\n", diagnostics_.telemetryLive ? 1 : 0);
        std::fprintf(file, "last_event=%s\n\n", diagnostics_.lastEvent);

        std::fprintf(file, "[Body]\n");
        std::fprintf(file, "speed_mps=%.3f\n", body_.speedMps);
        std::fprintf(file, "forward_speed_mps=%.3f\n", body_.forwardSpeedMps);
        std::fprintf(file, "lateral_speed_mps=%.3f\n", body_.lateralSpeedMps);
        std::fprintf(file, "yaw=%.4f\n", body_.yaw);
        std::fprintf(file, "pitch=%.4f\n", body_.pitch);
        std::fprintf(file, "roll=%.4f\n", body_.roll);
        std::fprintf(file, "body_slip_rad=%.4f\n", body_.bodySlipAngleRad);
        std::fprintf(file, "steer_deg=%.2f\n", body_.steerAngleRad * kRadToDeg);
        std::fprintf(file, "airborne=%d\n\n", body_.airborne ? 1 : 0);

        std::fprintf(file, "[Wheels]\n");
        static const char* names[4] = { "FL", "FR", "RL", "RR" };
        for (int i = 0; i < kWheelCount; ++i)
        {
            const WheelState& w = wheels_[i];
            std::fprintf(file, "%s.load_n=%.1f\n", names[i], w.normalLoadN);
            std::fprintf(file, "%s.slip_angle_deg=%.2f\n", names[i], w.slipAngleRad * kRadToDeg);
            std::fprintf(file, "%s.slip_ratio=%.3f\n", names[i], w.slipRatio);
            std::fprintf(file, "%s.mu=%.3f\n", names[i], w.frictionMu);
            std::fprintf(file, "%s.matrix=%.3f\n", names[i], w.matrixMultiplier);
            std::fprintf(file, "%s.temp_c=%.1f\n", names[i], w.tireTempC);
            std::fprintf(file, "%s.fx_n=%.1f\n", names[i], w.longitudinalForceN);
            std::fprintf(file, "%s.fy_n=%.1f\n", names[i], w.lateralForceN);
        }
        std::fprintf(file, "\n");

        std::fprintf(file, "[Transfer]\n");
        std::fprintf(file, "long_delta_n=%.1f\n", transfer_.longitudinalDelta);
        std::fprintf(file, "lat_delta_n=%.1f\n", transfer_.lateralDelta);
        std::fprintf(file, "total_normal_n=%.1f\n", transfer_.totalNormalN);
        std::fprintf(file, "pitch_moment_nm=%.1f\n", transfer_.pitchMomentNm);
        std::fprintf(file, "roll_moment_nm=%.1f\n", transfer_.rollMomentNm);
        std::fprintf(file, "\n[Aero]\n");
        std::fprintf(file, "downforce_n=%.1f\n", aero_.downforceN);
        std::fprintf(file, "drag_n=%.1f\n", aero_.dragN);
        std::fprintf(file, "dynamic_pressure=%.1f\n", aero_.dynamicPressure);
        std::fprintf(file, "ride_scale=%.3f\n", aero_.rideHeightScale);
        std::fprintf(file, "stall_scale=%.3f\n", aero_.stallScale);

        std::fclose(file);
        return true;
    }
    FH4RPO_EXCEPT(EXCEPTION_EXECUTE_HANDLER)
    {
        MemoryGuard::OnTrip();
        return false;
    }
}


} // namespace fh4rpo
