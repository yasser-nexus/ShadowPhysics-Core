// =============================================================================
//  FH4RPO - GameBinds.hpp
//
//  The address layer between the physics kernel and the game image.
//
//  Every value the module reads or writes is declared here with
//    * a symbolic name,
//    * a resolution strategy (signature / rip-relative / pointer chain /
//      explicit override from the INI),
//    * a validity contract (size, alignment, range checks before every write).
//
//  A bind that cannot be resolved is *not* fatal: its feature simply stays
//  disabled and the module keeps running on the telemetry-driven path.  That
//  is the difference between "the mod stopped working after an update" and
//  "the game crashed".
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

#include "Config.hpp"
#include "PhysicsEngine.hpp"

namespace fh4rpo {

enum class BindStage : int
{
    Unresolved = 0,
    Resolved,
    Guarded,      // resolved but failed a safety check, will retry later
    Disabled,     // explicitly disabled by configuration
    Failed
};

enum class BindKind : int
{
    Code = 0,       // function entry point (hook target)
    Float,          // 4 byte float in the game's car state
    Double,
    Int32,
    Matrix4x4,      // friction matrix base
    Chain           // multi level pointer
};

struct AddressOverride
{
    std::string name;
    std::uintptr_t address = 0;
};

struct BindEntry
{
    std::string   name;
    BindKind      kind = BindKind::Float;
    std::string   signature;         // AOB pattern, empty for overrides
    std::uintptr_t moduleOffset = 0; // optional static RVA (used when signature empty)
    std::size_t   size = 4;
    std::size_t   ripDispOffset = 0;
    std::size_t   ripInstructionLength = 0;
    std::vector<std::uint32_t> chain;
    bool          required = false;
    bool          allowWrite = false;
    float         minValue = -1.0e12f;
    float         maxValue =  1.0e12f;

    // runtime
    BindStage      stage = BindStage::Unresolved;
    std::uintptr_t address = 0;
    std::uintptr_t scannedFrom = 0;
    int            failures = 0;
    int            writes = 0;
    int            reads = 0;
    double         lastResolveTime = 0.0;
    std::string    note;
};

class GameBinds
{
public:
    static GameBinds& Instance() noexcept;

    void Initialize(const Config& config) noexcept;
    void Shutdown() noexcept;

    // Called once per frame; re-resolves stale binds on the configured cadence.
    void Tick(double dtSeconds) noexcept;

    // -------- registration ---------------------------------------------------
    void RegisterCode(const char* name, const char* signature, bool required) noexcept;
    void RegisterFloat(const char* name, const char* signature,
                       std::size_t ripDispOffset, std::size_t ripInstrLength,
                       bool allowWrite, float minValue, float maxValue) noexcept;
    void RegisterFloatRva(const char* name, std::uintptr_t rva, bool allowWrite,
                          float minValue, float maxValue) noexcept;
    void RegisterMatrix(const char* name, const char* signature,
                        std::size_t ripDispOffset, std::size_t ripInstrLength) noexcept;

    // User supplied absolute addresses ("[Addresses]" in the INI). Higher
    // priority than any scan and never overwritten by the resolver.
    void SetOverride(const char* name, std::uintptr_t address) noexcept;

    // -------- access ---------------------------------------------------------
    std::uintptr_t Resolve(const char* name) noexcept;
    bool  IsUsable(const char* name) const noexcept;
    float ReadFloat(const char* name, float fallback) const noexcept;
    bool  WriteFloat(const char* name, float value) noexcept;
    bool  WriteFloatBlock(const char* name, const float* values, std::size_t count) noexcept;

    // Builds the structure consumed by PhysicsEngine::RegisterTargets().
    PhysicsEngine::GameTargets BuildEngineTargets() const noexcept;

    int ResolvedCount() const noexcept;
    int FailedCount() const noexcept;
    int TotalCount() const noexcept;
    void DumpReport() const noexcept;

private:
    GameBinds() = default;
    ~GameBinds() = default;
    GameBinds(const GameBinds&) = delete;
    GameBinds& operator=(const GameBinds&) = delete;

    BindEntry* Find(const char* name) noexcept;
    const BindEntry* Find(const char* name) const noexcept;

    bool ResolveEntry(BindEntry& entry) noexcept;
    bool ValidateEntry(const BindEntry& entry) const noexcept;
    void ApplyOverrides() noexcept;

    static constexpr int kMaxBinds = 48;

    BindEntry  binds_[kMaxBinds] = {};
    int        bindCount_ = 0;
    std::string moduleName_;
    bool       initialized_ = false;
    double     clock_ = 0.0;
    double     resolveInterval_ = 3.0;
    int        scannerAlignment_ = 4;
};

} // namespace fh4rpo
