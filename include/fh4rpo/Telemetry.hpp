// =============================================================================
//  FH4RPO - Telemetry.hpp
//
//  Forza Horizon 4 "Data Out" is a fixed 324 byte, little-endian UDP packet
//  emitted once per rendered frame while driving.  It is the *only* sanctioned
//  source of live vehicle state: position, velocity, accelerations, yaw/pitch/
//  roll, the four slip angles, the four suspension travel values and the pedal
//  inputs.  The physics kernel is driven from these numbers and never has to
//  guess what the car is doing - which is precisely what keeps the module
//  stable when EA / Playground push a title update.
//
//  Offsets implemented below (Horizon layout, verified against independent
//  FH4/FH5/FH6 data out implementations):
//      Speed            @256  f32   m/s
//      Accel / Brake    @315 / @316 u8
//      Gear             @319  u8
//      Steer            @320  s8    (-127..127)
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>
#include <thread>
#include "Core.hpp"

namespace fh4rpo {

#pragma pack(push, 1)

// The shared 232 byte "sled" block present in every Forza title.
struct SledPacket
{
    std::int32_t  IsRaceOn;
    std::uint32_t TimestampMS;
    float EngineMaxRpm;
    float EngineIdleRpm;
    float CurrentEngineRpm;
    float AccelerationX;          // right +
    float AccelerationY;          // up +
    float AccelerationZ;          // forward +
    float VelocityX;
    float VelocityY;
    float VelocityZ;
    float AngularVelocityX;       // pitch rate  (rad/s)
    float AngularVelocityY;       // yaw rate    (rad/s)
    float AngularVelocityZ;       // roll rate   (rad/s)
    float Yaw;
    float Pitch;
    float Roll;
    float NormalizedSuspensionTravel[4];   // FL, FR, RL, RR  (0..1)
    float TireSlipRatio[4];
    float WheelRotationSpeed[4];
    std::int32_t WheelOnRumbleStrip[4];
    float WheelInPuddleDepth[4];
    float SurfaceRumble[4];
    float TireSlipAngle[4];       // radians
    float TireCombinedSlip[4];
    float SuspensionTravelMeters[4];
    std::int32_t CarOrdinal;
    std::int32_t CarClass;
    std::int32_t CarPerformanceIndex;
    std::int32_t DrivetrainType;  // 0 = FWD, 1 = RWD, 2 = AWD
    std::int32_t NumCylinders;
};

// Horizon inserts 12 extra bytes right after the sled block.
struct HorizonMiddle
{
    std::int32_t CarCategory;
    std::uint32_t HorizonUnknown1;
    std::uint32_t HorizonUnknown2;
};

struct DashPacket
{
    float PositionX;
    float PositionY;
    float PositionZ;
    float Speed;                  // m/s
    float Power;                  // watts
    float Torque;                 // Nm
    float TireTemp[4];            // Fahrenheit on the wire
    float Boost;
    float Fuel;
    float DistanceTraveled;
    float BestLap;
    float LastLap;
    float CurrentLap;
    float CurrentRaceTime;
    std::uint16_t LapNumber;
    std::uint8_t  RacePosition;
    std::uint8_t  Accel;          // 0..255
    std::uint8_t  Brake;          // 0..255
    std::uint8_t  Clutch;
    std::uint8_t  HandBrake;
    std::uint8_t  Gear;
    std::int8_t   Steer;          // -127..127
    std::int8_t   NormalizedDrivingLine;
    std::int8_t   NormalizedAIBrakeDifference;
    std::uint8_t  Expansion;      // Horizon trailer byte, keeps the packet at 324
};

struct HorizonPacket
{
    SledPacket    sled;
    HorizonMiddle middle;
    DashPacket    dash;
};

#pragma pack(pop)

static_assert(sizeof(SledPacket) == 232, "SledPacket must be exactly 232 bytes");
static_assert(sizeof(HorizonMiddle) == 12, "HorizonMiddle must be exactly 12 bytes");
static_assert(sizeof(DashPacket) == 80, "DashPacket must be exactly 80 bytes");
static_assert(sizeof(HorizonPacket) == 324, "HorizonPacket must be exactly 324 bytes");
static_assert(offsetof(HorizonPacket, dash.Speed) == 256, "Speed must live at byte 256");
static_assert(offsetof(HorizonPacket, dash.Accel) == 315, "Accel must live at byte 315");
static_assert(offsetof(HorizonPacket, dash.Steer) == 320, "Steer must live at byte 320");

// -----------------------------------------------------------------------------
//  Decoded, validated, engine-unit sample handed to the physics kernel.
//  Temperatures are Celsius, angles radians. Everything is guaranteed finite.
// -----------------------------------------------------------------------------
struct TelemetrySample
{
    double timestamp = 0.0;       // seconds, module clock
    bool   isValid   = false;

    float  speedMps      = 0.0f;
    float  velocity[3]   = { 0.0f, 0.0f, 0.0f };   // X right, Y up, Z forward
    float  accel[3]      = { 0.0f, 0.0f, 0.0f };   // device frame, m/s^2
    float  angularVel[3] = { 0.0f, 0.0f, 0.0f };   // pitch, yaw, roll rates
    float  yaw           = 0.0f;
    float  pitch         = 0.0f;
    float  roll          = 0.0f;

    float  rpm           = 0.0f;
    float  maxRpm        = 0.0f;
    float  idleRpm       = 0.0f;

    float  throttle      = 0.0f;  // 0..1
    float  brake         = 0.0f;  // 0..1
    float  clutch        = 0.0f;  // 0..1
    float  handbrake     = 0.0f;  // 0..1
    float  steerInput    = 0.0f;  // -1..1
    int    gear          = 0;     // 0 = neutral, -1 reverse

    float  slipAngle[4]  = {};    // rad, FL FR RL RR
    float  slipRatio[4]  = {};
    float  combinedSlip[4] = {};
    float  suspensionTravel[4] = {};       // metres
    float  normalizedTravel[4] = {};       // 0..1
    float  surfaceRumble[4] = {};
    bool   wheelRumble[4] = {};
    float  tireTempC[4]  = {};
    float  wheelSpeed[4] = {};

    float  powerW        = 0.0f;
    float  torqueNm      = 0.0f;

    std::int32_t carOrdinal     = 0;
    std::int32_t carClass       = 0;
    std::int32_t performanceIdx = 0;
    std::int32_t drivetrainType = 1;
    std::int32_t cylinders      = 0;
};

// -----------------------------------------------------------------------------
//  TelemetryReceiver - background UDP listener.
//
//  Runs on its own thread blocked in recvfrom with an SO_RCVTIMEO so shutdown
//  is prompt.  Packets are validated before being pushed onto the lock free
//  ring; the physics thread drains the ring once per frame and keeps only the
//  newest sample, so a stalled render thread can never grow latency.
// -----------------------------------------------------------------------------
class TelemetryReceiver
{
public:
    static constexpr std::size_t kRingCapacity = 256;

    TelemetryReceiver() = default;
    ~TelemetryReceiver();

    TelemetryReceiver(const TelemetryReceiver&) = delete;
    TelemetryReceiver& operator=(const TelemetryReceiver&) = delete;

    bool Start(int port, int fallbackPort, bool loopbackOnly) noexcept;
    void Stop() noexcept;
    bool IsRunning() const noexcept;

    // Pops the newest sample. Returns false when the ring is empty.
    bool PollLatest(TelemetrySample& out) noexcept;

    // Receiver side statistics
    std::uint64_t PacketsReceived() const noexcept;
    std::uint64_t PacketsRejected() const noexcept;
    double        SecondsSinceLastPacket() const noexcept;
    int           BoundPort() const noexcept;

    // Static, side effect free decoder (also used by the unit tests).
    static bool Decode(const std::uint8_t* data, std::size_t size, TelemetrySample& out) noexcept;

    // Optional assist: re-emit a packet with patched values so dashboards and
    // motion rigs see the same physics the game is feeling.
    void SetBroadcast(unsigned short port, bool enabled) noexcept;
    void PublishPatched(const std::uint8_t* raw, std::size_t size) noexcept;

private:
    void ReceiveLoop() noexcept;
    void BroadcastLoop(const TelemetrySample& sample) noexcept;

    std::thread        thread_;
    std::atomic<bool>  running_{ false };
    std::atomic<std::uint64_t> received_{ 0 };
    std::atomic<std::uint64_t> rejected_{ 0 };
    std::atomic<double> lastPacketTime_{ 0.0 };
    std::atomic<int>   boundPort_{ 0 };
    std::atomic<bool>  broadcastEnabled_{ false };
    std::atomic<unsigned short> broadcastPort_{ 0 };

    // Raw socket handles are kept as integers so this header stays free of
    // winsock includes; the .cpp casts them back.
    std::atomic<std::uintptr_t> socket_{ 0 };
    std::atomic<std::uintptr_t> broadcastSocket_{ 0 };
    std::uint8_t lastRaw_[512] = {};
    std::size_t  lastRawSize_  = 0;

    SpscRing<TelemetrySample, kRingCapacity> ring_;
};

} // namespace fh4rpo
