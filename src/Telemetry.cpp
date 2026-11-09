// =============================================================================
//  FH4RPO - Telemetry.cpp
//
//  UDP receiver for the Forza Horizon 4 "Data Out" stream.
//
//  The decoder (packet -> TelemetrySample) is platform independent and is the
//  piece the unit tests exercise.  The socket half is POSIX sockets on the host
//  build and Winsock on Windows - the surrounding logic (validation, ring
//  hand-off, statistics) is identical on both.
// =============================================================================
#include "fh4rpo/Telemetry.hpp"
#include "fh4rpo/Log.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  include <windows.h>
#  pragma comment(lib, "ws2_32.lib")
   using SocketHandle = SOCKET;
   using SockLen      = int;
#  define FH4RPO_INVALID_SOCKET INVALID_SOCKET
#  define FH4RPO_SOCKET_ERROR   SOCKET_ERROR
#  define CloseSocket closesocket
#else
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <unistd.h>
#  include <cerrno>
   using SocketHandle = int;
   using SockLen      = socklen_t;
#  define FH4RPO_INVALID_SOCKET (-1)
#  define FH4RPO_SOCKET_ERROR   (-1)
#  define CloseSocket ::close
#endif

namespace fh4rpo {

namespace {

std::mutex g_socketApiMutex;
int        g_socketApiRefCount = 0;

bool AcquireSocketApi() noexcept
{
    std::lock_guard<std::mutex> lock(g_socketApiMutex);
    if (g_socketApiRefCount > 0)
    {
        ++g_socketApiRefCount;
        return true;
    }
#ifdef _WIN32
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
    {
        FH4RPO_LOG_ERROR("Telemetry: WSAStartup failed");
        return false;
    }
#endif
    g_socketApiRefCount = 1;
    return true;
}

void ReleaseSocketApi() noexcept
{
    std::lock_guard<std::mutex> lock(g_socketApiMutex);
    if (g_socketApiRefCount <= 0)
        return;
    if (--g_socketApiRefCount == 0)
    {
#ifdef _WIN32
        WSACleanup();
#endif
    }
}

double NowSeconds() noexcept
{
    using namespace std::chrono;
    static const steady_clock::time_point origin = steady_clock::now();
    return duration<double>(steady_clock::now() - origin).count();
}

void SleepMs(int milliseconds) noexcept
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

constexpr std::uintptr_t kInvalidSocket = static_cast<std::uintptr_t>(-1);

} // namespace

// =============================================================================
//  Decoder - platform independent, defensive.
// =============================================================================
bool TelemetryReceiver::Decode(const std::uint8_t* data, std::size_t size,
                               TelemetrySample& out) noexcept
{
    out = TelemetrySample{};

    if (data == nullptr || size < sizeof(HorizonPacket))
        return false;

    HorizonPacket packet{};
    std::memcpy(&packet, data, sizeof(HorizonPacket));

    const SledPacket& sled = packet.sled;
    const DashPacket& dash = packet.dash;

    // ---- sanity gate --------------------------------------------------------
    // Anything that fails here is a port collision, a datagram from another
    // title, or memory garbage - never a packet we should trust.
    if (sled.IsRaceOn != 0 && sled.IsRaceOn != 1)
        return false;

    const float speed = dash.Speed;
    if (!IsFinite(speed) || speed < 0.0f || speed > 250.0f)
        return false;

    const float rpm = sled.CurrentEngineRpm;
    if (!IsFinite(rpm) || rpm < 0.0f || rpm > 30000.0f)
        return false;

    const float maxRpm = sled.EngineMaxRpm;
    if (!IsFinite(maxRpm) || maxRpm < 100.0f || maxRpm > 30000.0f)
        return false;

    if (!IsFinite(sled.AccelerationX) || !IsFinite(sled.AccelerationY) ||
        !IsFinite(sled.AccelerationZ))
        return false;

    if (!IsFinite(sled.VelocityX) || !IsFinite(sled.VelocityZ))
        return false;

    if (!IsFinite(sled.Yaw) || !IsFinite(sled.Pitch) || !IsFinite(sled.Roll))
        return false;

    // ---- copy ---------------------------------------------------------------
    out.timestamp = NowSeconds();
    out.isValid   = true;

    out.speedMps    = ClampFinite(speed, 0.0f, 250.0f, 0.0f);
    out.velocity[0] = ClampFinite(sled.VelocityX, -250.0f, 250.0f, 0.0f);
    out.velocity[1] = ClampFinite(sled.VelocityY, -250.0f, 250.0f, 0.0f);
    out.velocity[2] = ClampFinite(sled.VelocityZ, -250.0f, 250.0f, 0.0f);

    out.accel[0] = ClampFinite(sled.AccelerationX, -200.0f, 200.0f, 0.0f);
    out.accel[1] = ClampFinite(sled.AccelerationY, -200.0f, 200.0f, 0.0f);
    out.accel[2] = ClampFinite(sled.AccelerationZ, -200.0f, 200.0f, 0.0f);

    out.angularVel[0] = ClampFinite(sled.AngularVelocityX, -30.0f, 30.0f, 0.0f);   // pitch rate
    out.angularVel[1] = ClampFinite(sled.AngularVelocityY, -30.0f, 30.0f, 0.0f);   // yaw rate
    out.angularVel[2] = ClampFinite(sled.AngularVelocityZ, -30.0f, 30.0f, 0.0f);   // roll rate

    out.yaw   = ClampFinite(sled.Yaw, -kTwoPi * 4.0f, kTwoPi * 4.0f, 0.0f);
    out.pitch = ClampFinite(sled.Pitch, -kPi, kPi, 0.0f);
    out.roll  = ClampFinite(sled.Roll, -kPi, kPi, 0.0f);

    out.rpm     = ClampFinite(rpm, 0.0f, 30000.0f, 0.0f);
    out.maxRpm  = ClampFinite(maxRpm, 100.0f, 30000.0f, 8000.0f);
    out.idleRpm = ClampFinite(sled.EngineIdleRpm, 0.0f, 5000.0f, 850.0f);

    out.throttle   = Saturate(static_cast<float>(dash.Accel) / 255.0f);
    out.brake      = Saturate(static_cast<float>(dash.Brake) / 255.0f);
    out.clutch     = Saturate(static_cast<float>(dash.Clutch) / 255.0f);
    out.handbrake  = Saturate(static_cast<float>(dash.HandBrake) / 255.0f);
    out.steerInput = ClampFinite(static_cast<float>(dash.Steer) / 127.0f, -1.0f, 1.0f, 0.0f);
    out.gear       = static_cast<int>(dash.Gear);

    for (int i = 0; i < 4; ++i)
    {
        out.slipAngle[i]        = ClampFinite(sled.TireSlipAngle[i], -kPi, kPi, 0.0f);
        out.slipRatio[i]        = ClampFinite(sled.TireSlipRatio[i], -10.0f, 10.0f, 0.0f);
        out.combinedSlip[i]     = ClampFinite(sled.TireCombinedSlip[i], 0.0f, 20.0f, 0.0f);
        out.suspensionTravel[i] = ClampFinite(sled.SuspensionTravelMeters[i], -1.0f, 2.0f, 0.0f);
        out.normalizedTravel[i] = Saturate(sled.NormalizedSuspensionTravel[i]);
        out.surfaceRumble[i]    = Saturate(sled.SurfaceRumble[i]);
        out.wheelRumble[i]      = sled.WheelOnRumbleStrip[i] != 0;
        out.wheelSpeed[i]       = ClampFinite(sled.WheelRotationSpeed[i], -400.0f, 400.0f, 0.0f);

        // Data Out reports tyre temperature in Fahrenheit -> convert to Celsius.
        const float tempF = ClampFinite(dash.TireTemp[i], -148.0f, 752.0f, 68.0f);
        out.tireTempC[i]  = ClampFinite((tempF - 32.0f) * (5.0f / 9.0f), -60.0f, 400.0f, 20.0f);
    }

    out.powerW   = ClampFinite(dash.Power, -2.0e6f, 2.0e6f, 0.0f);
    out.torqueNm = ClampFinite(dash.Torque, -20000.0f, 20000.0f, 0.0f);

    out.carOrdinal     = sled.CarOrdinal;
    out.carClass       = sled.CarClass;
    out.performanceIdx = sled.CarPerformanceIndex;
    out.drivetrainType = Clamp<std::int32_t>(sled.DrivetrainType, 0, 2);
    out.cylinders      = sled.NumCylinders;

    return true;
}

// =============================================================================
//  Receiver
// =============================================================================
TelemetryReceiver::~TelemetryReceiver()
{
    Stop();
}

bool TelemetryReceiver::Start(int port, int fallbackPort, bool loopbackOnly) noexcept
{
    if (running_.load(std::memory_order_acquire))
        return true;

    if (!AcquireSocketApi())
        return false;

    SocketHandle sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == static_cast<SocketHandle>(FH4RPO_INVALID_SOCKET))
    {
        FH4RPO_LOG_ERROR("Telemetry: socket() failed");
        ReleaseSocketApi();
        return false;
    }

    // Allow the same port to be re-bound immediately after a hot reload.
    int reuse = 1;
    ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,
                 reinterpret_cast<const char*>(&reuse), sizeof(reuse));

    sockaddr_in address{};
    address.sin_family      = AF_INET;
    address.sin_port        = htons(static_cast<unsigned short>(port));
    address.sin_addr.s_addr = loopbackOnly ? htonl(INADDR_LOOPBACK) : htonl(INADDR_ANY);

    int boundPort = port;
    if (::bind(sock, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == FH4RPO_SOCKET_ERROR)
    {
        FH4RPO_LOG_WARN("Telemetry: bind(%d) failed, trying fallback %d", port, fallbackPort);

        sockaddr_in fallback = address;
        fallback.sin_port = htons(static_cast<unsigned short>(fallbackPort));

        if (::bind(sock, reinterpret_cast<sockaddr*>(&fallback), sizeof(fallback)) == FH4RPO_SOCKET_ERROR)
        {
            FH4RPO_LOG_ERROR("Telemetry: bind(%d) failed too", fallbackPort);
            CloseSocket(sock);
            ReleaseSocketApi();
            return false;
        }
        boundPort = fallbackPort;
    }

#ifdef _WIN32
    DWORD timeoutMs = 200;
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                 reinterpret_cast<const char*>(&timeoutMs), sizeof(timeoutMs));
#else
    timeval timeout{};
    timeout.tv_sec  = 0;
    timeout.tv_usec = 200 * 1000;
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                 reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#endif

    int bufferSize = 64 * 1024;
    ::setsockopt(sock, SOL_SOCKET, SO_RCVBUF,
                 reinterpret_cast<const char*>(&bufferSize), sizeof(bufferSize));

    socket_.store(static_cast<std::uintptr_t>(sock), std::memory_order_release);
    boundPort_.store(boundPort, std::memory_order_release);
    received_.store(0, std::memory_order_relaxed);
    rejected_.store(0, std::memory_order_relaxed);
    lastPacketTime_.store(0.0, std::memory_order_relaxed);
    ring_.Clear();

    running_.store(true, std::memory_order_release);
    thread_ = std::thread(&TelemetryReceiver::ReceiveLoop, this);

    FH4RPO_LOG_INFO("Telemetry: listening on %s:%d", loopbackOnly ? "127.0.0.1" : "0.0.0.0", boundPort);
    return true;
}

void TelemetryReceiver::Stop() noexcept
{
    if (!running_.exchange(false, std::memory_order_acq_rel))
        return;

    const std::uintptr_t handle = socket_.exchange(kInvalidSocket, std::memory_order_acq_rel);
    if (handle != kInvalidSocket && handle != 0)
    {
#ifdef _WIN32
        ::shutdown(static_cast<SocketHandle>(handle), SD_BOTH);
#else
        ::shutdown(static_cast<SocketHandle>(handle), SHUT_RDWR);
#endif
        CloseSocket(static_cast<SocketHandle>(handle));
    }

    if (thread_.joinable())
        thread_.join();

    const std::uintptr_t broadcastHandle = broadcastSocket_.exchange(kInvalidSocket,
                                                                     std::memory_order_acq_rel);
    if (broadcastHandle != kInvalidSocket && broadcastHandle != 0)
        CloseSocket(static_cast<SocketHandle>(broadcastHandle));

    socket_.store(0, std::memory_order_release);
    ReleaseSocketApi();

    FH4RPO_LOG_INFO("Telemetry: receiver stopped (%llu packets, %llu rejected)",
                    static_cast<unsigned long long>(received_.load()),
                    static_cast<unsigned long long>(rejected_.load()));
}

bool TelemetryReceiver::IsRunning() const noexcept
{
    return running_.load(std::memory_order_acquire);
}

void TelemetryReceiver::ReceiveLoop() noexcept
{
    std::uint8_t buffer[512] = {};
    TelemetrySample sample{};

    while (running_.load(std::memory_order_acquire))
    {
        const std::uintptr_t handle = socket_.load(std::memory_order_acquire);
        if (handle == kInvalidSocket || handle == 0)
            break;

        const int received = static_cast<int>(::recvfrom(
            static_cast<SocketHandle>(handle),
            reinterpret_cast<char*>(buffer), sizeof(buffer), 0, nullptr, nullptr));

        if (received <= 0)
        {
            // Timeout / interrupt: simply retry, the outer flag is re-checked.
            SleepMs(0);
            continue;
        }

        if (!Decode(buffer, static_cast<std::size_t>(received), sample))
        {
            rejected_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        received_.fetch_add(1, std::memory_order_relaxed);
        lastPacketTime_.store(NowSeconds(), std::memory_order_relaxed);

        if (broadcastEnabled_.load(std::memory_order_relaxed) &&
            received <= static_cast<int>(sizeof(lastRaw_)))
        {
            std::memcpy(lastRaw_, buffer, static_cast<std::size_t>(received));
            lastRawSize_ = static_cast<std::size_t>(received);
        }

        if (!ring_.Push(sample))
        {
            // Consumer stalled: drop the backlog but keep the newest sample.
            ring_.Clear();
            ring_.Push(sample);
        }
    }
}

bool TelemetryReceiver::PollLatest(TelemetrySample& out) noexcept
{
    bool got = false;
    TelemetrySample candidate{};
    while (ring_.Pop(candidate))
    {
        out = candidate;
        got = true;
    }
    return got;
}

std::uint64_t TelemetryReceiver::PacketsReceived() const noexcept
{
    return received_.load(std::memory_order_relaxed);
}

std::uint64_t TelemetryReceiver::PacketsRejected() const noexcept
{
    return rejected_.load(std::memory_order_relaxed);
}

double TelemetryReceiver::SecondsSinceLastPacket() const noexcept
{
    const double last = lastPacketTime_.load(std::memory_order_relaxed);
    if (last <= 0.0)
        return 1.0e9;
    const double now = NowSeconds();
    return (now >= last) ? (now - last) : 0.0;
}

int TelemetryReceiver::BoundPort() const noexcept
{
    return boundPort_.load(std::memory_order_relaxed);
}

void TelemetryReceiver::SetBroadcast(unsigned short port, bool enabled) noexcept
{
    broadcastPort_.store(port, std::memory_order_relaxed);
    broadcastEnabled_.store(enabled, std::memory_order_relaxed);

    if (!enabled)
    {
        const std::uintptr_t handle = broadcastSocket_.exchange(kInvalidSocket,
                                                               std::memory_order_acq_rel);
        if (handle != kInvalidSocket && handle != 0)
            CloseSocket(static_cast<SocketHandle>(handle));
        return;
    }

    if (broadcastSocket_.load(std::memory_order_acquire) == kInvalidSocket)
    {
        SocketHandle sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock != static_cast<SocketHandle>(FH4RPO_INVALID_SOCKET))
            broadcastSocket_.store(static_cast<std::uintptr_t>(sock), std::memory_order_release);
    }
}

void TelemetryReceiver::PublishPatched(const std::uint8_t* raw, std::size_t size) noexcept
{
    if (!broadcastEnabled_.load(std::memory_order_relaxed) || raw == nullptr)
        return;
    if (size < sizeof(HorizonPacket))
        return;

    const std::uintptr_t handle = broadcastSocket_.load(std::memory_order_acquire);
    if (handle == kInvalidSocket || handle == 0)
        return;

    sockaddr_in target{};
    target.sin_family      = AF_INET;
    target.sin_port        = htons(broadcastPort_.load(std::memory_order_relaxed));
    target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    ::sendto(static_cast<SocketHandle>(handle),
             reinterpret_cast<const char*>(raw),
             static_cast<int>(size), 0,
             reinterpret_cast<const sockaddr*>(&target), sizeof(target));
}

} // namespace fh4rpo
