// =============================================================================
//  FH4RPO - PhysicsTests.cpp
//
//  Self-contained test suite for the portable kernel.  No framework, no
//  external dependency: `fh4rpo_tests` returns 0 on success and prints
//  "ALL TESTS PASSED" (the CTest pass regex looks for exactly that).
//
//  Covered:
//    * numeric hygiene (ClampFinite / SafeDiv / ExponentialApproach / Vec3)
//    * fixed step driver determinism and the anti-spiral clamp
//    * bilinear friction table (interpolation, clamping, monotonic fall-off)
//    * the 324 byte Horizon packet layout and the decoder (incl. hostile input)
//    * weight transfer: load conservation, load shift under braking / cornering
//    * aerodynamics: quadratic speed law, ground effect, stall fall-off
//    * friction matrix: grip reduction at high slip angle, restoration on exit
//    * tyre model: peak near the reference slip angle, load sensitivity
//    * write-back through MemoryGuard into a local buffer
//    * stability: a long randomised drive must stay finite
// =============================================================================
#include "fh4rpo/Core.hpp"
#include "fh4rpo/Config.hpp"
#include "fh4rpo/PhysicsEngine.hpp"
#include "fh4rpo/MemoryGuard.hpp"
#include "fh4rpo/Telemetry.hpp"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <limits>
#include <vector>

using namespace fh4rpo;

namespace {

int         g_testsRun    = 0;
int         g_testsFailed = 0;
const char* g_currentTest = "?";

void BeginTest(const char* name)
{
    g_currentTest = name;
    ++g_testsRun;
    std::printf("[ RUN      ] %s\n", name);
}

void EndTest()
{
    std::printf("[       OK ] %s\n", g_currentTest);
}

void CheckTrue(bool condition, const char* what)
{
    if (!condition)
    {
        ++g_testsFailed;
        std::printf("[     FAIL ] %s :: %s\n", g_currentTest, what);
    }
}

void CheckFalse(bool condition, const char* what) { CheckTrue(!condition, what); }

void CheckNear(double actual, double expected, double tolerance, const char* what)
{
    if (!(std::fabs(actual - expected) <= tolerance))
    {
        ++g_testsFailed;
        std::printf("[     FAIL ] %s :: %s (actual %.6f, expected %.6f +/- %.6f)\n",
                    g_currentTest, what, actual, expected, tolerance);
    }
}

// -----------------------------------------------------------------------------
//  Synthetic telemetry
// -----------------------------------------------------------------------------
TelemetrySample MakeSample(float speedMps, float slipAngleDeg, float throttle,
                           float brake, float steerInput,
                           float lateralG = 0.0f, float longitudinalG = 0.0f)
{
    TelemetrySample s{};
    s.isValid = true;
    s.speedMps = speedMps;
    s.velocity[0] = lateralG * 0.05f * speedMps;
    s.velocity[1] = 0.0f;
    s.velocity[2] = speedMps;
    s.accel[0] = lateralG * kGravity;
    s.accel[1] = kGravity;
    s.accel[2] = longitudinalG * kGravity;
    s.rpm = 5200.0f;
    s.maxRpm = 8000.0f;
    s.idleRpm = 850.0f;
    s.throttle = throttle;
    s.brake = brake;
    s.clutch = 0.0f;
    s.handbrake = 0.0f;
    s.steerInput = steerInput;
    s.gear = 4;

    for (int i = 0; i < 4; ++i)
    {
        s.slipAngle[i] = slipAngleDeg * kDegToRad;
        s.slipRatio[i] = 0.02f;
        s.suspensionTravel[i] = -0.02f;
        s.normalizedTravel[i] = 0.5f;
        s.tireTempC[i] = 85.0f;
        s.wheelSpeed[i] = speedMps;
    }
    return s;
}

void PrepareConfig(Config& config)
{
    config.masterEnabled.store(true);
    config.passiveMode.store(false);

    config.physics.enabled             = true;
    config.physics.passiveModeAllowed  = true;
    config.physics.fixedStepHz         = 240.0f;
    config.physics.maxSubSteps         = 8;
    config.physics.applySuspensionForces = true;
    config.physics.applyChassisTorque  = true;
    config.physics.applyDrivetrain     = true;
    config.physics.applyBrakes         = true;
    config.physics.globalGripScale     = 1.0f;
    config.physics.tcsSimulation       = false;
    config.physics.escSimulation       = false;

    config.friction.enabled          = true;
    config.friction.asymmetric       = false;
    config.friction.useLoadAxis      = true;
    config.friction.snapDeg          = 2.0f;
    config.friction.fadeRate         = 60.0f;      // fast blend so tests converge

    config.weight.enabled            = true;
    config.weight.longTransferGain   = 1.0f;
    config.weight.latTransferGain    = 1.0f;
    config.weight.loadRateLimitN     = 1.0e7f;     // no rate limit in tests
    config.weight.minCornerLoadN     = 0.0f;
    config.weight.jackingGain        = 0.0f;

    config.aero.enabled              = true;
    config.aero.overrideGameDownforce = true;
    config.aero.groundEffectGain     = 0.42f;
    config.aero.smoothTimeS          = 0.02f;
    config.tire.muNominalFront       = 1.08f;
    config.tire.muNominalRear        = 1.12f;

    config.vehicle.massKg            = 1450.0f;
    config.vehicle.wheelbaseM        = 2.70f;
    config.vehicle.cgHeightM         = 0.52f;
    config.vehicle.weightDistribution = 0.53f;
    config.vehicle.driveLayout       = 1;
    config.vehicle.rideHeightM       = 0.11f;

    config.Sanitize();
}

void RunFrames(PhysicsEngine& engine, const TelemetrySample& sample, int frames)
{
    for (int i = 0; i < frames; ++i)
    {
        engine.SubmitTelemetry(sample);
        engine.OnFrame(1.0 / 60.0);
    }
}

// =============================================================================
//  1. numeric helpers
// =============================================================================
void TestNumericHelpers()
{
    BeginTest("Numeric: NaN / infinity scrubbing");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    CheckFalse(IsFinite(nan), "NaN is not finite");
    CheckFalse(IsFinite(inf), "+inf is not finite");
    CheckNear(ClampFinite(nan, 0.0f, 1.0f, 0.5f), 0.5, 1e-6, "ClampFinite(NaN) -> fallback");
    CheckNear(ClampFinite(inf, 0.0f, 1.0f, 0.5f), 0.5, 1e-6, "ClampFinite(inf) -> fallback");
    CheckNear(ClampFinite(2.0f, 0.0f, 1.0f, 0.5f), 1.0, 1e-6, "ClampFinite clamps high");
    CheckNear(ClampFinite(-2.0f, 0.0f, 1.0f, 0.5f), 0.0, 1e-6, "ClampFinite clamps low");
    CheckNear(SafeDiv(1.0f, 0.0f, 42.0f), 42.0, 1e-6, "SafeDiv by zero -> fallback");
    CheckNear(SafeDiv(10.0f, 2.0f), 5.0, 1e-6, "SafeDiv normal");
    CheckNear(Saturate(3.0f), 1.0, 1e-6, "Saturate high");
    CheckNear(Saturate(-3.0f), 0.0, 1e-6, "Saturate low");
    CheckNear(Lerp(0.0f, 10.0f, 0.25f), 2.5, 1e-6, "Lerp");
    CheckNear(Lerp(0.0f, 10.0f, 5.0f), 10.0, 1e-6, "Lerp clamps t");
    EndTest();

    BeginTest("Numeric: exponential approach");
    float value = 0.0f;
    for (int i = 0; i < 1200; ++i)
        value = ExponentialApproach(value, 1.0f, 0.05f, 1.0f / 240.0f);
    CheckNear(value, 1.0, 1e-3, "approaches the target");
    CheckTrue(value > 0.0f && value <= 1.0f, "stays within bounds");
    CheckNear(ExponentialApproach(0.7f, 1.0f, 0.05f, 0.0f), 1.0, 1e-6,
              "zero dt returns the target");
    CheckNear(ExponentialApproach(0.0f, 1.0f, 0.0f, 0.01f), 1.0, 1e-6,
              "zero tau returns the target");
    CheckNear(ExponentialApproach(std::nanf(""), 1.0f, 0.05f, 0.01f), 1.0, 1e-6,
              "NaN state falls back to the target");
    EndTest();

    BeginTest("Numeric: Vec3 geometry");
    const Vec3 a(1.0f, 0.0f, 0.0f);
    const Vec3 b(0.0f, 1.0f, 0.0f);
    CheckNear(a.Dot(b), 0.0, 1e-6, "orthogonal dot product");
    const Vec3 cross = a.Cross(b);
    CheckNear(cross.z, 1.0, 1e-6, "right handed cross product");
    CheckNear(Vec3(3.0f, 4.0f, 0.0f).Length(), 5.0, 1e-6, "length");
    CheckNear(Vec3(3.0f, 0.0f, 4.0f).Length2D(), 5.0, 1e-6, "2D length");
    CheckNear(Vec3(0.0f, 0.0f, 0.0f).Normalized().Length(), 0.0, 1e-6,
              "zero vector normalises safely");
    CheckTrue(Vec3(1.0f, 2.0f, 3.0f).IsValid(), "valid vector");
    CheckFalse(Vec3(std::nanf(""), 0.0f, 0.0f).IsValid(), "NaN vector rejected");
    EndTest();

    BeginTest("Numeric: deterministic noise");
    Pcg32 first(12345);
    Pcg32 second(12345);
    bool identical = true;
    for (int i = 0; i < 64; ++i)
    {
        if (first.Next() != second.Next())
            identical = false;
    }
    CheckTrue(identical, "same seed -> same sequence");

    bool inRange = true;
    for (int i = 0; i < 5000; ++i)
    {
        const float value2 = first.Signed();
        if (value2 < -1.0f || value2 > 1.0f)
            inRange = false;
    }
    CheckTrue(inRange, "Signed() stays inside [-1, 1]");
    EndTest();
}

// =============================================================================
//  2. fixed step driver + rate limiter + ring buffer
// =============================================================================
void TestScheduling()
{
    BeginTest("FixedStep: substeps track the frame budget");
    FixedStepDriver driver;
    driver.Configure(1.0 / 100.0, 10);

    int    totalSteps    = 0;
    double simulatedTime = 0.0;
    for (int frame = 0; frame < 100; ++frame)
        totalSteps += driver.Advance(1.0 / 60.0, [&](float dt) { simulatedTime += dt; });

    CheckNear(simulatedTime, 100.0 / 60.0, 0.03, "simulated time tracks wall time");
    CheckTrue(totalSteps > 150 && totalSteps < 180, "substep count inside the expected band");
    EndTest();

    BeginTest("FixedStep: pathological frame time cannot spiral");
    FixedStepDriver spiralling;
    spiralling.Configure(1.0 / 240.0, 8);
    const int huge = spiralling.Advance(10.0, [](float) {});
    CheckTrue(huge <= 8, "substeps capped by maxSubSteps");
    const int next = spiralling.Advance(1.0 / 240.0, [](float) {});
    CheckTrue(next <= 8, "accumulator drained after the clamp");
    CheckTrue(spiralling.Advance(std::nan(""), [](float) {}) == 0, "NaN dt is ignored");
    EndTest();

    BeginTest("RateLimiter: respects rise and fall rates");
    RateLimiter limiter;
    limiter.Configure(10.0f, 5.0f);
    const float risen = limiter.Update(0.0f, 1.0f, 0.01f);
    CheckNear(risen, 0.1, 1e-5, "rise limited to rate * dt");
    const float fallen = limiter.Update(risen, 0.0f, 0.01f);
    CheckNear(fallen, 0.05, 1e-5, "fall limited to rate * dt");
    CheckNear(limiter.Update(0.0f, std::nanf(""), 0.01f), 0.0, 1e-5,
              "NaN target held at the current value");
    EndTest();

    BeginTest("SpscRing: FIFO order and overflow behaviour");
    SpscRing<int, 8> ring;
    for (int i = 0; i < 8; ++i)
        ring.Push(i);

    // Capacity is 8 slots, one is reserved => at most 7 items are stored.
    CheckTrue(ring.ApproxSize() <= 7, "ring never reports full capacity");

    int  value  = 0;
    int  expect = 0;
    bool ordered = true;
    while (ring.Pop(value))
    {
        if (value != expect)
            ordered = false;
        ++expect;
    }
    CheckTrue(ordered, "items pop in insertion order");
    CheckFalse(ring.Pop(value), "empty ring reports empty");
    EndTest();
}

// =============================================================================
//  3. friction matrix table
// =============================================================================
void TestBilinearTable()
{
    BeginTest("FrictionTable: interpolation and clamping");
    const float rows[3]     = { 0.0f, 10.0f, 20.0f };
    const float cols[2]     = { 1000.0f, 5000.0f };
    const float data[3 * 2] = { 1.0f, 1.0f,
                                0.8f, 0.7f,
                                0.5f, 0.4f };
    BilinearTable table;
    table.Set(rows, 3, cols, 2, data);

    CheckNear(table.Sample(0.0f, 1000.0f), 1.0, 1e-5, "corner value");
    CheckNear(table.Sample(10.0f, 1000.0f), 0.8, 1e-5, "row value");
    CheckNear(table.Sample(5.0f, 1000.0f), 0.9, 1e-5, "row interpolation");
    CheckNear(table.Sample(10.0f, 3000.0f), 0.75, 1e-5, "column interpolation");
    CheckNear(table.Sample(100.0f, 5000.0f), 0.4, 1e-5, "clamped high");
    CheckNear(table.Sample(-100.0f, 0.0f), 1.0, 1e-5, "clamped low");
    CheckNear(table.Sample(std::nanf(""), 1000.0f), table.Sample(0.0f, 1000.0f), 1e-5,
              "NaN key treated as the low key");
    EndTest();

    BeginTest("FrictionTable: monotonic fall-off with slip angle");
    BilinearTable monotonicTable;
    monotonicTable.Set(rows, 3, cols, 2, data);
    float previous  = monotonicTable.Sample(0.0f, 3000.0f);
    bool  monotonic = true;
    for (float angle = 1.0f; angle <= 20.0f; angle += 1.0f)
    {
        const float current = monotonicTable.Sample(angle, 3000.0f);
        if (current > previous + 1e-6f)
            monotonic = false;
        previous = current;
    }
    CheckTrue(monotonic, "multiplier never rises with slip angle");
    EndTest();

    BeginTest("FrictionTable: constant fill");
    BilinearTable filled;
    filled.Fill(1.0f);
    CheckNear(filled.Sample(-30.0f, 200.0f), 1.0, 1e-6, "fill produces unity everywhere");
    CheckNear(filled.Sample(45.0f, 9000.0f), 1.0, 1e-6, "fill produces unity everywhere (2)");
    EndTest();
}

// =============================================================================
//  4. packet layout + decoder
// =============================================================================
void TestTelemetryDecoder()
{
    BeginTest("Telemetry: Horizon packet is 324 bytes with documented offsets");
    CheckTrue(sizeof(HorizonPacket) == 324, "packet size");
    CheckTrue(sizeof(SledPacket) == 232, "sled size");
    CheckTrue(offsetof(HorizonPacket, dash.Speed) == 256, "speed offset");
    CheckTrue(offsetof(HorizonPacket, dash.Accel) == 315, "throttle offset");
    CheckTrue(offsetof(HorizonPacket, dash.Brake) == 316, "brake offset");
    CheckTrue(offsetof(HorizonPacket, dash.Gear) == 319, "gear offset");
    CheckTrue(offsetof(HorizonPacket, dash.Steer) == 320, "steer offset");
    EndTest();

    BeginTest("Telemetry: decoder round trip");
    HorizonPacket packet{};
    packet.sled.IsRaceOn        = 1;
    packet.sled.EngineMaxRpm    = 7800.0f;
    packet.sled.EngineIdleRpm   = 900.0f;
    packet.sled.CurrentEngineRpm = 5200.0f;
    packet.sled.AccelerationX   = 8.5f;
    packet.sled.AccelerationZ   = 4.0f;
    packet.sled.VelocityZ       = 42.0f;
    packet.sled.Yaw             = 0.35f;
    packet.sled.DrivetrainType  = 1;
    for (int i = 0; i < 4; ++i)
    {
        packet.sled.TireSlipAngle[i]              = static_cast<float>(6 + i) * kDegToRad;
        packet.sled.TireSlipRatio[i]              = 0.05f * static_cast<float>(i + 1);
        packet.sled.SuspensionTravelMeters[i]     = -0.03f;
        packet.sled.NormalizedSuspensionTravel[i] = 0.4f;
        packet.sled.WheelRotationSpeed[i]         = 42.0f;
        packet.dash.TireTemp[i]                   = 176.0f;    // 80 C
    }
    packet.dash.Speed = 42.0f;
    packet.dash.Accel = 200;
    packet.dash.Brake = 55;
    packet.dash.Gear  = 4;
    packet.dash.Steer = -64;

    TelemetrySample sample{};
    const bool decoded = TelemetryReceiver::Decode(
        reinterpret_cast<const std::uint8_t*>(&packet), sizeof(packet), sample);

    CheckTrue(decoded, "decoder accepts a valid packet");
    CheckNear(sample.speedMps, 42.0, 1e-4, "speed");
    CheckNear(sample.throttle, 200.0 / 255.0, 1e-4, "throttle normalised");
    CheckNear(sample.brake, 55.0 / 255.0, 1e-4, "brake normalised");
    CheckNear(sample.steerInput, -64.0 / 127.0, 1e-4, "steer normalised");
    CheckTrue(sample.gear == 4, "gear");
    CheckTrue(sample.carClass == 0 || sample.carClass == packet.sled.CarClass, "car class copied");
    CheckNear(sample.tireTempC[0], 80.0, 0.1, "fahrenheit -> celsius");
    CheckNear(sample.slipAngle[0], 6.0 * kDegToRad, 1e-4, "slip angle preserved");
    CheckNear(sample.suspensionTravel[2], -0.03, 1e-5, "suspension travel preserved");
    EndTest();

    BeginTest("Telemetry: decoder rejects hostile input");
    TelemetrySample out{};
    CheckFalse(TelemetryReceiver::Decode(nullptr, 324, out), "null buffer");
    CheckFalse(TelemetryReceiver::Decode(reinterpret_cast<const std::uint8_t*>(&packet), 100, out),
               "short packet");

    HorizonPacket broken = packet;
    broken.sled.IsRaceOn = 7;
    CheckFalse(TelemetryReceiver::Decode(reinterpret_cast<const std::uint8_t*>(&broken),
                                         sizeof(broken), out), "bad race flag");

    broken = packet;
    broken.dash.Speed = std::numeric_limits<float>::quiet_NaN();
    CheckFalse(TelemetryReceiver::Decode(reinterpret_cast<const std::uint8_t*>(&broken),
                                         sizeof(broken), out), "NaN speed");

    broken = packet;
    broken.dash.Speed = 9000.0f;
    CheckFalse(TelemetryReceiver::Decode(reinterpret_cast<const std::uint8_t*>(&broken),
                                         sizeof(broken), out), "impossible speed");

    broken = packet;
    broken.sled.CurrentEngineRpm = -1.0f;
    CheckFalse(TelemetryReceiver::Decode(reinterpret_cast<const std::uint8_t*>(&broken),
                                         sizeof(broken), out), "negative rpm");
    EndTest();
}

// =============================================================================
//  5. weight transfer
// =============================================================================
void TestWeightTransfer()
{
    Config& config = Config::Instance();
    PrepareConfig(config);
    // Isolate the transfer stage: downforce has its own test group and would
    // otherwise add load on top of m*g.
    config.aero.enabled = false;

    PhysicsEngine engine;
    engine.Initialize();
    engine.UpdateVehicleSpec(config.vehicle);
    engine.ResetDynamics();

    BeginTest("WeightTransfer: static loads conserve total weight");
    const TelemetrySample cruise = MakeSample(30.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f, 0.0f);
    RunFrames(engine, cruise, 120);

    const WeightTransferResult& transfer = engine.Transfer();
    const float expectedTotal = config.vehicle.massKg * kGravity;
    CheckNear(transfer.totalNormalN, expectedTotal, expectedTotal * 0.05,
              "sum of corner loads ~= m * g");
    CheckNear(engine.Wheel(WheelFL).staticLoadN + engine.Wheel(WheelFR).staticLoadN,
              expectedTotal * config.vehicle.weightDistribution, expectedTotal * 0.02,
              "front axle carries the configured weight distribution");
    EndTest();

    BeginTest("WeightTransfer: braking loads the front axle");
    PhysicsEngine braking;
    braking.Initialize();
    braking.UpdateVehicleSpec(config.vehicle);
    braking.ResetDynamics();

    const TelemetrySample brakeSample = MakeSample(45.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -1.1f);
    RunFrames(braking, brakeSample, 120);

    const float frontLoad = braking.Wheel(WheelFL).normalLoadN + braking.Wheel(WheelFR).normalLoadN;
    const float rearLoad  = braking.Wheel(WheelRL).normalLoadN + braking.Wheel(WheelRR).normalLoadN;
    CheckTrue(frontLoad > rearLoad, "front axle is heavier under braking");
    CheckNear(frontLoad + rearLoad, expectedTotal, expectedTotal * 0.05,
              "total load still conserved under braking");
    CheckTrue(braking.Transfer().longitudinalDelta < 0.0f,
              "negative longitudinal delta while decelerating");
    EndTest();

    BeginTest("WeightTransfer: cornering loads the outside wheels");
    PhysicsEngine cornering;
    cornering.Initialize();
    cornering.UpdateVehicleSpec(config.vehicle);
    cornering.ResetDynamics();

    // +X acceleration == accelerating to the right == left wheels loaded.
    const TelemetrySample turn = MakeSample(35.0f, 4.0f, 0.5f, 0.0f, 0.4f, 1.0f, 0.0f);
    RunFrames(cornering, turn, 150);

    const float leftLoad  = cornering.Wheel(WheelFL).normalLoadN + cornering.Wheel(WheelRL).normalLoadN;
    const float rightLoad = cornering.Wheel(WheelFR).normalLoadN + cornering.Wheel(WheelRR).normalLoadN;
    CheckTrue(leftLoad > rightLoad, "outside (left) wheels carry more load in a right turn");
    CheckTrue(cornering.Transfer().rollMomentNm > 0.0f, "roll moment has the expected sign");
    EndTest();

    BeginTest("WeightTransfer: airborne frame unloads every corner");
    PhysicsEngine airborne;
    airborne.Initialize();
    airborne.UpdateVehicleSpec(config.vehicle);
    airborne.ResetDynamics();

    TelemetrySample jump = MakeSample(25.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 4; ++i)
    {
        jump.normalizedTravel[i] = 0.0f;
        jump.suspensionTravel[i] = 0.0f;
    }
    RunFrames(airborne, jump, 60);
    CheckTrue(airborne.Body().airborne, "airborne state detected from full droop");
    CheckNear(airborne.Wheel(WheelFL).normalLoadN, 0.0, 50.0, "corner load released in the air");
    EndTest();

    BeginTest("WeightTransfer: disabled mode falls back to static loads");
    const bool savedWeightEnabled = config.weight.enabled;
    config.weight.enabled = false;

    PhysicsEngine passive;
    passive.Initialize();
    passive.UpdateVehicleSpec(config.vehicle);
    passive.ResetDynamics();
    RunFrames(passive, MakeSample(40.0f, 2.0f, 0.0f, 1.0f, 0.5f, 1.2f, -1.0f), 60);

    const float flStatic = passive.Wheel(WheelFL).normalLoadN;
    const float frStatic = passive.Wheel(WheelFR).normalLoadN;
    CheckNear(flStatic, frStatic, 1.0, "no lateral transfer when disabled");
    config.weight.enabled = savedWeightEnabled;
    EndTest();
}

// =============================================================================
//  6. aerodynamics
// =============================================================================
void TestAero()
{
    Config& config = Config::Instance();
    PrepareConfig(config);

    BeginTest("Aero: downforce follows 0.5 rho v^2 C A");
    PhysicsEngine engine;
    engine.Initialize();
    engine.UpdateVehicleSpec(config.vehicle);
    engine.ResetDynamics();

    const float speed = 50.0f;
    const TelemetrySample fast = MakeSample(speed, 0.0f, 0.8f, 0.0f, 0.0f);
    RunFrames(engine, fast, 120);

    const float q = 0.5f * config.aero.airDensityKgM3 * speed * speed;
    const float clTotal = config.aero.liftCoefficientFront * config.aero.balanceSplit +
                          config.aero.liftCoefficientRear * (1.0f - config.aero.balanceSplit);
    const float expectedBase = q * config.aero.frontalAreaM2 * clTotal;
    CheckTrue(engine.Aero().downforceN > expectedBase * 0.9f,
              "downforce at speed is at least the plain quadratic term");
    CheckTrue(engine.Aero().downforceN < expectedBase * 2.2f,
              "ground effect does not multiply downforce absurdly");
    CheckNear(engine.Aero().dynamicPressure, q, q * 0.02, "dynamic pressure");
    EndTest();

    BeginTest("Aero: downforce scales with the square of speed");
    PhysicsEngine slow;
    slow.Initialize();
    slow.UpdateVehicleSpec(config.vehicle);
    slow.ResetDynamics();
    RunFrames(slow, MakeSample(25.0f, 0.0f, 0.8f, 0.0f, 0.0f), 120);

    const float ratio = engine.Aero().downforceN / std::max(slow.Aero().downforceN, 1.0f);
    CheckTrue(ratio > 3.0f && ratio < 5.0f, "doubling speed quadruples downforce");
    EndTest();

    BeginTest("Aero: override replaces the game's value");
    const float gameValue = 1500.0f;
    const float replaced  = engine.EvaluateDownforce(gameValue, 60.0f, config.vehicle.rideHeightM);
    CheckTrue(std::fabs(replaced - gameValue) > 100.0f, "game value was replaced");
    CheckTrue(IsFinite(replaced), "replacement is finite");

    config.aero.overrideGameDownforce = false;
    const float passthrough = engine.EvaluateDownforce(gameValue, 60.0f, config.vehicle.rideHeightM);
    CheckNear(passthrough, gameValue, 1e-3, "override disabled -> passthrough");
    config.aero.overrideGameDownforce = true;
    EndTest();

    BeginTest("Aero: stall fall-off above the critical speed");
    const float belowStall = engine.EvaluateDownforce(0.0f, config.aero.stallSpeedMps * 0.5f,
                                                      config.vehicle.rideHeightM);
    const float aboveStall = engine.EvaluateDownforce(0.0f, config.aero.stallSpeedMps * 1.9f,
                                                      config.vehicle.rideHeightM);
    const float speedRatio = 1.9f * 1.9f / (0.5f * 0.5f);   // 14.4x more dynamic pressure
    CheckTrue(belowStall > 0.0f, "downforce below stall");
    CheckTrue(aboveStall / belowStall < speedRatio * 0.7f,
              "downforce growth is curtailed past the stall speed");
    EndTest();

    BeginTest("Aero: hostile inputs are rejected");
    CheckTrue(IsFinite(engine.EvaluateDownforce(0.0f, std::nanf(""), 0.1f)),
              "NaN speed returns a finite value");
    CheckTrue(IsFinite(engine.EvaluateDownforce(0.0f, 80.0f, std::nanf(""))),
              "NaN ride height returns a finite value");
    CheckTrue(IsFinite(engine.EvaluateDownforce(std::nanf(""), 80.0f, 0.1f)),
              "NaN reference returns a finite value");
    EndTest();
}

// =============================================================================
//  7. friction matrix behaviour inside the engine
// =============================================================================
void TestFrictionMatrixEngine()
{
    Config& config = Config::Instance();
    PrepareConfig(config);

    BeginTest("FrictionMatrix: grip falls as the slip angle grows");
    PhysicsEngine straight;
    straight.Initialize();
    straight.UpdateVehicleSpec(config.vehicle);
    straight.ResetDynamics();
    RunFrames(straight, MakeSample(30.0f, 1.0f, 0.4f, 0.0f, 0.0f), 120);

    PhysicsEngine sliding;
    sliding.Initialize();
    sliding.UpdateVehicleSpec(config.vehicle);
    sliding.ResetDynamics();
    RunFrames(sliding, MakeSample(30.0f, 22.0f, 0.4f, 0.0f, 0.9f), 120);

    const float straightMultiplier = straight.Wheel(WheelRL).matrixMultiplier;
    const float slidingMultiplier  = sliding.Wheel(WheelRL).matrixMultiplier;
    CheckTrue(straightMultiplier > 0.97f && straightMultiplier < 1.10f,
              "near unity at small slip angles");
    CheckTrue(slidingMultiplier < 0.95f, "reduced at a large slip angle");
    CheckTrue(slidingMultiplier > 0.60f, "reduction stays inside the tuned table");
    EndTest();

    BeginTest("FrictionMatrix: same wheel index is deterministic");
    const float a = sliding.EvaluateFrictionMultiplier(WheelRL, 20.0f * kDegToRad, 4000.0f);
    const float b = sliding.EvaluateFrictionMultiplier(WheelRL, 20.0f * kDegToRad, 4000.0f);
    CheckNear(a, b, 1e-6, "pure function of the inputs");

    const float front = sliding.EvaluateFrictionMultiplier(WheelFL, 20.0f * kDegToRad, 4000.0f);
    CheckTrue(front > 0.1f && front < 4.0f, "front multiplier inside the guard rails");
    EndTest();

    BeginTest("FrictionMatrix: table is symmetric by default");
    const float positive = sliding.EvaluateFrictionMultiplier(WheelRR, 15.0f * kDegToRad, 4000.0f);
    const float negative = sliding.EvaluateFrictionMultiplier(WheelRR, -15.0f * kDegToRad, 4000.0f);
    CheckNear(positive, negative, 1e-6, "left and right slips behave identically");
    EndTest();

    BeginTest("FrictionMatrix: disabled engine is bit-exact passthrough");
    const bool savedFrictionEnabled = config.friction.enabled;
    config.friction.enabled = false;
    const float disabled = sliding.EvaluateFrictionMultiplier(WheelRL, 20.0f * kDegToRad, 4000.0f);
    CheckNear(disabled, 1.0, 1e-9, "multiplier is exactly 1.0 when disabled");
    config.friction.enabled = savedFrictionEnabled;
    EndTest();
}

// =============================================================================
//  8. tyre model
// =============================================================================
void TestTireModel()
{
    Config& config = Config::Instance();
    PrepareConfig(config);

    BeginTest("Tires: lateral force peaks near the reference slip angle");
    PhysicsEngine engine;
    engine.Initialize();
    engine.UpdateVehicleSpec(config.vehicle);
    engine.ResetDynamics();

    float peak = 0.0f;
    float peakSlip = 0.0f;
    for (float deg = 0.5f; deg <= 25.0f; deg += 0.5f)
    {
        PhysicsEngine probe;
        probe.Initialize();
        probe.UpdateVehicleSpec(config.vehicle);
        probe.ResetDynamics();
        RunFrames(probe, MakeSample(30.0f, deg, 0.5f, 0.0f, 0.6f), 90);

        const float force = std::fabs(probe.Wheel(WheelRL).lateralForceN);
        if (force > peak)
        {
            peak = force;
            peakSlip = deg;
        }
    }
    CheckTrue(peak > 1000.0f, "peak lateral force is physically plausible");
    CheckTrue(peakSlip > 1.0f && peakSlip < 16.0f, "peak sits near the configured slip angle");
    EndTest();

    BeginTest("Tires: load sensitivity reduces mu under heavier load");
    PhysicsEngine light;
    light.Initialize();
    VehicleSpecConfig lightSpec = config.vehicle;
    lightSpec.massKg = 900.0f;
    light.UpdateVehicleSpec(lightSpec);
    light.ResetDynamics();
    RunFrames(light, MakeSample(25.0f, 5.0f, 0.3f, 0.0f, 0.2f), 90);

    PhysicsEngine heavy;
    heavy.Initialize();
    VehicleSpecConfig heavySpec = config.vehicle;
    heavySpec.massKg = 2200.0f;
    heavy.UpdateVehicleSpec(heavySpec);
    heavy.ResetDynamics();
    RunFrames(heavy, MakeSample(25.0f, 5.0f, 0.3f, 0.0f, 0.2f), 90);

    CheckTrue(light.Wheel(WheelFL).frictionMu >= heavy.Wheel(WheelFL).frictionMu,
              "lighter car has at least as much grip per tyre");
    EndTest();

    BeginTest("Tires: every force output stays finite and bounded");
    bool bounded = true;
    for (int i = 0; i < 4; ++i)
    {
        const WheelState& wheel = heavy.Wheel(i);
        CheckTrue(IsFinite(wheel.lateralForceN), "lateral force is finite");
        CheckTrue(IsFinite(wheel.longitudinalForceN), "longitudinal force is finite");
        CheckTrue(std::fabs(wheel.lateralForceN) < 60000.0f, "lateral force is bounded");
        CheckTrue(std::fabs(wheel.longitudinalForceN) < 60000.0f, "longitudinal force is bounded");
        if (!(wheel.normalLoadN >= 0.0f))
            bounded = false;
    }
    CheckTrue(bounded, "no negative corner load survives the solver");
    EndTest();
}

// =============================================================================
//  9. write-back
// =============================================================================
void TestWriteback()
{
    Config& config = Config::Instance();
    PrepareConfig(config);

    BeginTest("Writeback: values reach the target addresses through MemoryGuard");
    PhysicsEngine engine;
    engine.Initialize();
    engine.UpdateVehicleSpec(config.vehicle);
    engine.ResetDynamics();

    float downforceSlot = 0.0f;
    float gripSlot      = 0.0f;
    float biasSlot      = 0.0f;
    float steerSlot     = 0.0f;
    float matrixSlot[16] = {};

    PhysicsEngine::GameTargets targets{};
    targets.downforceAddress    = reinterpret_cast<std::uintptr_t>(&downforceSlot);
    targets.gripScaleAddress    = reinterpret_cast<std::uintptr_t>(&gripSlot);
    targets.brakeBiasAddress    = reinterpret_cast<std::uintptr_t>(&biasSlot);
    targets.steerAngleAddress   = reinterpret_cast<std::uintptr_t>(&steerSlot);
    targets.frictionMatrixBase  = reinterpret_cast<std::uintptr_t>(matrixSlot);
    targets.frictionMatrixStride = 16;
    engine.RegisterTargets(targets);

    RunFrames(engine, MakeSample(55.0f, 6.0f, 0.9f, 0.0f, 0.3f), 60);

    CheckTrue(IsFinite(downforceSlot), "downforce slot holds a finite value");
    CheckTrue(gripSlot > 0.05f && gripSlot < 8.0f, "grip scale slot inside the guard rails");
    CheckTrue(biasSlot >= 0.05f && biasSlot <= 0.95f, "brake bias slot inside the guard rails");
    CheckTrue(IsFinite(steerSlot), "steer slot holds a finite value");

    bool matrixValid = true;
    for (int i = 0; i < 16; ++i)
    {
        if (!IsFinite(matrixSlot[i]))
            matrixValid = false;
        if (matrixSlot[i] < 0.10f || matrixSlot[i] > 4.0f)
            matrixValid = false;
    }
    CheckTrue(matrixValid, "friction matrix block is fully populated and bounded");
    EndTest();

    BeginTest("Writeback: unwritable targets are refused, not crashed into");
    PhysicsEngine::GameTargets bad{};
    bad.downforceAddress = static_cast<std::uintptr_t>(0x1000);   // almost certainly unmapped
    engine.RegisterTargets(bad);
    const int written = engine.ApplyWriteback();
    CheckTrue(written >= 0, "write path returns cleanly");
    CheckTrue(engine.BuildGameWriteback(1000.0f, 1.0f, 0.6f, 3.0f), "write-back cache still builds");
    EndTest();
}

// =============================================================================
// 10. long random stability run
// =============================================================================
void TestStability()
{
    Config& config = Config::Instance();
    PrepareConfig(config);

    BeginTest("Stability: 3000 randomised frames stay finite");
    PhysicsEngine engine;
    engine.Initialize();
    engine.UpdateVehicleSpec(config.vehicle);
    engine.ResetDynamics();

    Pcg32 rng(0xC0FFEE);
    bool  allFinite = true;

    for (int frame = 0; frame < 3000; ++frame)
    {
        const float speed     = Saturate(static_cast<float>(frame) / 3000.0f) * 70.0f;
        const float slipDeg   = rng.Signed() * 25.0f;
        const float throttle  = Saturate(rng.Signed() * 0.5f + 0.5f);
        const float brake     = Saturate(rng.Signed() * 0.5f + 0.5f);
        const float steer     = rng.Signed();
        const float lateralG  = rng.Signed() * 1.4f;
        const float longG     = rng.Signed() * 1.2f;

        engine.SubmitTelemetry(MakeSample(speed, slipDeg, throttle, brake, steer, lateralG, longG));
        engine.OnFrame(1.0 / 60.0);

        const BodyState& body = engine.Body();
        if (!IsFinite(body.speedMps) || !IsFinite(body.yaw) || !IsFinite(body.pitch) ||
            !IsFinite(body.roll) || !IsFinite(body.steerAngleRad))
            allFinite = false;

        for (int i = 0; i < 4; ++i)
        {
            const WheelState& wheel = engine.Wheel(i);
            if (!IsFinite(wheel.normalLoadN) || !IsFinite(wheel.lateralForceN) ||
                !IsFinite(wheel.longitudinalForceN) || !IsFinite(wheel.tireTempC) ||
                !IsFinite(wheel.matrixMultiplier))
                allFinite = false;
        }

        if (!IsFinite(engine.Aero().downforceN) || !IsFinite(engine.Aero().dragN))
            allFinite = false;

        if (!allFinite)
        {
            std::printf("       -> divergence at frame %d\n", frame);
            break;
        }
    }

    CheckTrue(allFinite, "no NaN or infinity after 3000 frames");
    // The kernel counted every fixed substep, not every rendered frame.
    CheckTrue(engine.Diagnostics().updates >= 3000, "update counter advanced");
    EndTest();

    BeginTest("Stability: telemetry loss degrades to passive extrapolation");
    PhysicsEngine passive;
    passive.Initialize();
    passive.UpdateVehicleSpec(config.vehicle);
    passive.ResetDynamics();

    RunFrames(passive, MakeSample(40.0f, 3.0f, 0.6f, 0.0f, 0.1f), 60);
    CheckTrue(passive.Diagnostics().telemetryLive, "telemetry reported live");

    for (int i = 0; i < 60; ++i)
        passive.OnFrame(1.0 / 60.0);      // no new samples

    CheckTrue(passive.Diagnostics().passiveFallback, "passive fallback engaged");
    CheckTrue(IsFinite(passive.Body().speedMps), "extrapolated speed stays finite");
    CheckTrue(IsFinite(passive.Aero().downforceN), "aero output stays finite");
    EndTest();

    BeginTest("Stability: corrupt sample cannot poison the solver");
    PhysicsEngine guarded;
    guarded.Initialize();
    guarded.UpdateVehicleSpec(config.vehicle);
    guarded.ResetDynamics();

    TelemetrySample hostile = MakeSample(30.0f, 3.0f, 0.5f, 0.0f, 0.2f);
    hostile.speedMps = std::numeric_limits<float>::quiet_NaN();
    hostile.accel[2] = std::numeric_limits<float>::infinity();
    for (int i = 0; i < 4; ++i)
    {
        hostile.slipAngle[i] = std::numeric_limits<float>::quiet_NaN();
        hostile.suspensionTravel[i] = std::numeric_limits<float>::infinity();
    }
    RunFrames(guarded, hostile, 30);

    CheckTrue(IsFinite(guarded.Body().speedMps), "body speed stays finite");
    CheckTrue(IsFinite(guarded.Wheel(WheelFL).normalLoadN), "corner load stays finite");
    CheckTrue(IsFinite(guarded.Aero().downforceN), "aero stays finite");
    EndTest();
}

// =============================================================================
// 11. configuration round trip
// =============================================================================
void TestConfiguration()
{
    BeginTest("Config: defaults survive a save / load cycle");
    Config& config = Config::Instance();
    PrepareConfig(config);

    const std::string path = "fh4rpo_test_config.ini";
    CheckTrue(config.Save(path), "config saved");
    CheckTrue(config.Save(path), "config can be overwritten");

    Config reloaded;
    const bool loaded = reloaded.Load(path);
    CheckTrue(loaded, "config loaded");
    CheckNear(reloaded.vehicle.massKg, config.vehicle.massKg, 1e-2, "mass round trip");
    CheckNear(reloaded.aero.dragCoefficient, config.aero.dragCoefficient, 1e-4,
              "drag coefficient round trip");
    CheckNear(reloaded.friction.frontValues[3][2], config.friction.frontValues[3][2], 1e-4,
              "friction matrix round trip");
    CheckTrue(reloaded.physics.maxSubSteps == config.physics.maxSubSteps, "substep count round trip");
    std::remove(path.c_str());
    EndTest();

    BeginTest("Config: sanitisation clamps hostile values");
    Config hostile;
    hostile.vehicle.massKg = -500.0f;
    hostile.vehicle.wheelbaseM = 900.0f;
    hostile.aero.airDensityKgM3 = -3.0f;
    hostile.tire.muNominalFront = 99.0f;
    hostile.physics.fixedStepHz = 0.0f;
    hostile.physics.maxSubSteps = 10000;
    hostile.telemetry.port = -5;
    hostile.Sanitize();

    CheckTrue(hostile.vehicle.massKg >= 300.0f, "mass clamped up");
    CheckTrue(hostile.vehicle.wheelbaseM <= 5.4f, "wheelbase clamped down");
    CheckTrue(hostile.aero.airDensityKgM3 >= 0.6f, "air density clamped");
    CheckTrue(hostile.tire.muNominalFront <= 2.2f, "mu clamped");
    CheckTrue(hostile.physics.fixedStepHz >= 30.0f, "step rate clamped up");
    CheckTrue(hostile.physics.maxSubSteps <= 32, "substeps clamped");
    CheckTrue(hostile.telemetry.port >= 1, "port clamped");
    EndTest();
}

} // namespace

// =============================================================================
int main()
{
    std::printf("=====================================================\n");
    std::printf(" FH4RPO physics kernel - test suite\n");
    std::printf("=====================================================\n\n");

    TestNumericHelpers();
    TestScheduling();
    TestBilinearTable();
    TestTelemetryDecoder();
    TestWeightTransfer();
    TestAero();
    TestFrictionMatrixEngine();
    TestTireModel();
    TestWriteback();
    TestStability();
    TestConfiguration();

    std::printf("\n-----------------------------------------------------\n");
    std::printf(" %d test groups, %d failure(s)\n", g_testsRun, g_testsFailed);

    if (g_testsFailed == 0)
    {
        std::printf(" ALL TESTS PASSED\n");
        return 0;
    }

    std::printf(" TESTS FAILED\n");
    return 1;
}
