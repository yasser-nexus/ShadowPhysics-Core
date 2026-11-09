// =============================================================================
//  FH4 Realistic Physics Overhaul (FH4RPO)
//  Core.hpp - portable math / utility layer.
//
//  This header is intentionally free of any Windows dependency so that the
//  physics kernel can be compiled and unit-tested on a normal workstation
//  (see tests/PhysicsTests.cpp) before it is ever injected into the game.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <atomic>
#include <algorithm>
#include <limits>

// Portable noinline marker - used where a real stack frame is required (SEH).
#if defined(_MSC_VER)
#  define FH4RPO_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#  define FH4RPO_NOINLINE __attribute__((noinline))
#else
#  define FH4RPO_NOINLINE
#endif

namespace fh4rpo {

// -----------------------------------------------------------------------------
//  Constants
// -----------------------------------------------------------------------------
constexpr float  kPi          = 3.14159265358979323846f;
constexpr float  kTwoPi       = 6.28318530717958647692f;
constexpr float  kDegToRad    = kPi / 180.0f;
constexpr float  kRadToDeg    = 180.0f / kPi;
constexpr float  kGravity     = 9.80665f;      // m/s^2
constexpr float  kAirDensity  = 1.2250f;       // kg/m^3 @ 15 deg C, sea level
constexpr float  kKmhToMps    = 1.0f / 3.6f;
constexpr float  kMpsToKmh    = 3.6f;
constexpr float  kMphToMps    = 0.44704f;
constexpr double kMaxFrameDt  = 0.100;         // hard clamp, protects the integrator
constexpr float  kEpsilon     = 1.0e-6f;

// -----------------------------------------------------------------------------
//  Numeric helpers - every value that crosses a boundary (memory / UDP) is
//  funnelled through these so a NaN or +-Inf can never propagate into the game.
// -----------------------------------------------------------------------------
inline bool IsFinite(float v) noexcept
{
    return (v == v) && (v != std::numeric_limits<float>::infinity()) &&
           (v != -std::numeric_limits<float>::infinity());
}

inline bool IsFinite(double v) noexcept
{
    return (v == v) && (v != std::numeric_limits<double>::infinity()) &&
           (v != -std::numeric_limits<double>::infinity());
}

template <typename T>
inline T Clamp(T v, T lo, T hi) noexcept
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// Clamp + NaN scrub.  `fallback` is returned when the input is not finite.
inline float ClampFinite(float v, float lo, float hi, float fallback = 0.0f) noexcept
{
    if (!IsFinite(v))
        return fallback;
    return Clamp(v, lo, hi);
}

inline double ClampFinite(double v, double lo, double hi, double fallback = 0.0) noexcept
{
    if (!IsFinite(v))
        return fallback;
    return Clamp(v, lo, hi);
}

inline float Saturate(float v) noexcept { return ClampFinite(v, 0.0f, 1.0f, 0.0f); }

inline float Lerp(float a, float b, float t) noexcept
{
    return a + (b - a) * ClampFinite(t, 0.0f, 1.0f, 0.0f);
}

inline float SignOf(float v) noexcept { return (v > 0.0f) ? 1.0f : ((v < 0.0f) ? -1.0f : 0.0f); }

inline float SafeDiv(float num, float den, float fallback = 0.0f) noexcept
{
    if (!IsFinite(num) || !IsFinite(den) || std::fabs(den) < kEpsilon)
        return fallback;
    const float r = num / den;
    return IsFinite(r) ? r : fallback;
}

// Smooth, branch-lighter approximation of a critically damped low pass.
// Exact solution of dx/dt = (target - x) / tau integrated over dt.
inline float ExponentialApproach(float current, float target, float tau, float dt) noexcept
{
    if (!IsFinite(current) || !IsFinite(target) || !IsFinite(tau) || !IsFinite(dt))
        return IsFinite(target) ? target : 0.0f;
    if (tau <= kEpsilon || dt <= 0.0f)
        return target;
    const float alpha = 1.0f - std::exp(-dt / tau);
    return current + (target - current) * ClampFinite(alpha, 0.0f, 1.0f, 1.0f);
}

// -----------------------------------------------------------------------------
//  Vec3 - right handed, X = right, Y = up, Z = forward (Forza body frame)
// -----------------------------------------------------------------------------
struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float ix, float iy, float iz) : x(ix), y(iy), z(iz) {}

    Vec3 operator+(const Vec3& o) const noexcept { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const noexcept { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(float s) const noexcept { return Vec3(x * s, y * s, z * s); }
    Vec3& operator+=(const Vec3& o) noexcept { x += o.x; y += o.y; z += o.z; return *this; }

    float Dot(const Vec3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }
    Vec3 Cross(const Vec3& o) const noexcept
    {
        return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }
    float Length() const noexcept { return std::sqrt(x * x + y * y + z * z); }
    float Length2D() const noexcept { return std::sqrt(x * x + z * z); }

    Vec3 Normalized(float fallbackLen = 1.0f) const noexcept
    {
        const float len = Length();
        if (!IsFinite(len) || len < kEpsilon)
            return Vec3();
        const float inv = fallbackLen / len;
        return Vec3(x * inv, y * inv, z * inv);
    }

    bool IsValid() const noexcept { return IsFinite(x) && IsFinite(y) && IsFinite(z); }

    static Vec3 Lerp(const Vec3& a, const Vec3& b, float t) noexcept
    {
        return Vec3(fh4rpo::Lerp(a.x, b.x, t), fh4rpo::Lerp(a.y, b.y, t), fh4rpo::Lerp(a.z, b.z, t));
    }
};

// -----------------------------------------------------------------------------
//  FixedStepDriver - decouples the game frame rate from the physics rate.
//  Guarantees a deterministic, bounded number of substeps per frame.
// -----------------------------------------------------------------------------
class FixedStepDriver
{
public:
    void Configure(double fixedStep, int maxSubSteps) noexcept
    {
        fixedStep_   = ClampFinite(fixedStep, 1.0 / 1000.0, 1.0 / 30.0, 1.0 / 240.0);
        maxSubSteps_ = Clamp(maxSubSteps, 1, 32);
        accumulator_ = 0.0;
    }

    double FixedStep() const noexcept { return fixedStep_; }

    // Calls fn(dt) for every whole substep contained in `frameDt`.
    // Returns the number of substeps actually executed.
    template <typename Fn>
    int Advance(double frameDt, Fn&& fn) noexcept
    {
        if (!IsFinite(frameDt) || frameDt <= 0.0)
            return 0;

        accumulator_ += Clamp(frameDt, 0.0, kMaxFrameDt);
        int steps = 0;
        const float step = static_cast<float>(fixedStep_);
        while (accumulator_ >= fixedStep_ && steps < maxSubSteps_)
        {
            fn(step);
            accumulator_ -= fixedStep_;
            ++steps;
        }
        // If the frame was so long that we hit the substep ceiling, drop the debt
        // instead of accumulating an ever growing spiral of death.
        if (steps >= maxSubSteps_)
            accumulator_ = 0.0;
        return steps;
    }

    void Reset() noexcept { accumulator_ = 0.0; }

private:
    double fixedStep_   = 1.0 / 240.0;
    int    maxSubSteps_ = 8;
    double accumulator_ = 0.0;
};

// -----------------------------------------------------------------------------
//  RateLimiter - used for actuator-like quantities (steering, brake pressure).
// -----------------------------------------------------------------------------
class RateLimiter
{
public:
    void Configure(float risePerSecond, float fallPerSecond) noexcept
    {
        rise_ = std::fabs(risePerSecond);
        fall_ = std::fabs(fallPerSecond);
    }

    float Update(float current, float target, float dt) noexcept
    {
        if (!IsFinite(current) || !IsFinite(target) || !IsFinite(dt) || dt <= 0.0f)
            return IsFinite(target) ? target : 0.0f;
        const float delta = target - current;
        const float limit = (std::fabs(target) > std::fabs(current) ? rise_ : fall_) * dt;
        if (std::fabs(delta) <= limit)
            return target;
        return current + SignOf(delta) * limit;
    }

    void Reset() noexcept {}
private:
    float rise_ = 1.0f;
    float fall_ = 1.0f;
};

// -----------------------------------------------------------------------------
//  BilinearTable - the data structure behind the friction multiplier matrix.
//  Axis keys must be strictly ascending.
// -----------------------------------------------------------------------------
class BilinearTable
{
public:
    // row = outer (slip angle), col = inner (vertical load)
    void Set(const float* rows, int rowCount, const float* cols, int colCount, const float* data) noexcept
    {
        rowCount_ = Clamp(rowCount, 0, kMaxRows);
        colCount_ = Clamp(colCount, 0, kMaxCols);
        for (int i = 0; i < rowCount_; ++i)
        {
            rowKeys_[i] = rows[i];
            for (int j = 0; j < colCount_; ++j)
                values_[i][j] = ClampFinite(data[i * colCount_ + j], 0.0f, 8.0f, 1.0f);
        }
        for (int j = 0; j < colCount_; ++j)
            colKeys_[j] = cols[j];
    }

    void Fill(float value) noexcept
    {
        rowCount_ = 2;
        colCount_ = 2;
        rowKeys_[0] = -90.0f; rowKeys_[1] = 90.0f;
        colKeys_[0] = 0.0f;   colKeys_[1] = 20000.0f;
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                values_[i][j] = value;
    }

    int RowCount() const noexcept { return rowCount_; }
    int ColCount() const noexcept { return colCount_; }

    float Sample(float rowKey, float colKey) const noexcept
    {
        if (rowCount_ < 2 || colCount_ < 2)
            return 1.0f;

        // Keys are clamped into the table domain, so out of range samples are
        // constant extrapolations rather than invented values.
        rowKey = ClampFinite(rowKey, rowKeys_[0], rowKeys_[rowCount_ - 1], rowKeys_[0]);
        colKey = ClampFinite(colKey, colKeys_[0], colKeys_[colCount_ - 1], colKeys_[0]);

        const int i0 = FindSegment(rowKeys_, rowCount_, rowKey);
        const int j0 = FindSegment(colKeys_, colCount_, colKey);

        const float r0 = rowKeys_[i0];
        const float r1 = rowKeys_[i0 + 1];
        const float c0 = colKeys_[j0];
        const float c1 = colKeys_[j0 + 1];

        const float tr = SafeDiv(rowKey - r0, r1 - r0);
        const float tc = SafeDiv(colKey - c0, c1 - c0);

        const float v00 = values_[i0][j0];
        const float v01 = values_[i0][j0 + 1];
        const float v10 = values_[i0 + 1][j0];
        const float v11 = values_[i0 + 1][j0 + 1];

        // vXY = value at (row X, col Y); interpolate along the row axis first.
        const float lowRow  = v00 + (v10 - v00) * tr;
        const float highRow = v01 + (v11 - v01) * tr;
        return ClampFinite(lowRow + (highRow - lowRow) * tc, 0.0f, 8.0f, 1.0f);
    }

private:
    static int FindSegment(const float* keys, int count, float key) noexcept
    {
        int idx = 0;
        while (idx < count - 2 && key > keys[idx + 1])
            ++idx;
        return idx;
    }

    static constexpr int kMaxRows = 16;
    static constexpr int kMaxCols = 8;

    float rowKeys_[kMaxRows] = { 0.0f, 1.0f };
    float colKeys_[kMaxCols] = { 0.0f, 1.0f };
    float values_[kMaxRows][kMaxCols] = {};
    int   rowCount_ = 2;
    int   colCount_ = 2;
};

// -----------------------------------------------------------------------------
//  SpscRing - single producer / single consumer lock free queue used to hand
//  telemetry samples from the network thread to the physics thread without
//  ever taking a lock inside the game's frame loop.
// -----------------------------------------------------------------------------
template <typename T, std::size_t Capacity>
class SpscRing
{
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");

public:
    bool Push(const T& item) noexcept
    {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = (head + 1) & (Capacity - 1);
        if (next == tail_.load(std::memory_order_acquire))
            return false;                        // full: drop the oldest is NOT done on
                                                 // purpose - the producer must never
                                                 // touch consumer owned memory.
        buffer_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    bool Pop(T& out) noexcept
    {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire))
            return false;
        out = buffer_[tail];
        tail_.store((tail + 1) & (Capacity - 1), std::memory_order_release);
        return true;
    }

    void Clear() noexcept
    {
        tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
    }

    std::size_t ApproxSize() const noexcept
    {
        const std::size_t h = head_.load(std::memory_order_acquire);
        const std::size_t t = tail_.load(std::memory_order_acquire);
        return (h - t) & (Capacity - 1);
    }

private:
    T buffer_[Capacity] = {};
    alignas(64) std::atomic<std::size_t> head_{ 0 };
    alignas(64) std::atomic<std::size_t> tail_{ 0 };
};

// -----------------------------------------------------------------------------
//  Deterministic noise (PCG) - road / steering roughness, no global state.
// -----------------------------------------------------------------------------
class Pcg32
{
public:
    explicit Pcg32(std::uint64_t seed = 0x853C49E6748FEA9Bull) : state_(seed) {}

    std::uint32_t Next() noexcept
    {
        const std::uint64_t old = state_;
        state_ = old * 6364136223846793005ull + 1442695040888963407ull;
        const std::uint32_t xorshifted = static_cast<std::uint32_t>(((old >> 18u) ^ old) >> 27u);
        const std::uint32_t rot = static_cast<std::uint32_t>(old >> 59u);
        return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31u));
    }

    // Uniform in [-1, 1]
    float Signed() noexcept
    {
        return static_cast<float>(Next() & 0x00FFFFFFu) / 8388607.5f - 1.0f;
    }

private:
    std::uint64_t state_;
};

} // namespace fh4rpo
