// =============================================================================
//  FH4RPO - GameBinds.cpp
//
//  Resolution + validation of every game address the module uses.
//  Any bind may fail; the module degrades to "telemetry only" and logs exactly
//  which feature was lost.
// =============================================================================
#include "fh4rpo/GameBinds.hpp"
#include "fh4rpo/PatternScanner.hpp"
#include "fh4rpo/MemoryGuard.hpp"
#include "fh4rpo/Log.hpp"

#include <windows.h>
#include <cstring>

namespace fh4rpo {

GameBinds& GameBinds::Instance() noexcept
{
    static GameBinds binds;
    return binds;
}

void GameBinds::Initialize(const Config& config) noexcept
{
    if (initialized_)
        return;
    initialized_ = true;

    bindCount_ = 0;
    moduleName_ = "ForzaHorizon4.exe";
    resolveInterval_ = config.memory.resolveIntervalS;
    scannerAlignment_ = config.memory.scannerAlignment;
    clock_ = 0.0;

    PatternScanner::SetAlignment(scannerAlignment_);

    // -------------------------------------------------------------------------
    //  Hook entry points.
    //
    //  The signatures below are *structural*: they describe instruction shapes
    //  that the title uses for the four routines the module intercepts.  They
    //  are deliberately wildcard heavy so a recompile of the game (different
    //  register allocation / different immediates) still matches, while the
    //  shape of the routine stays recognisable.  If a title update breaks one
    //  of them, only that feature is lost and the log says which one.
    //
    //  All of them are optional: with `[Addresses]` in the INI you can pin the
    //  correct RVA manually without waiting for a module update.
    // -------------------------------------------------------------------------
    RegisterCode("hook_friction_matrix",
                 "48 8B 89 ?? ?? ?? ?? F3 0F 10 ?? ?? F3 0F 11 ?? ?? ?? ?? ? C3",
                 false);

    RegisterCode("hook_downforce",
                 "F3 0F 10 81 ?? ?? ?? ?? F3 0F 59 81 ?? ?? ?? ?? F3 0F 59",
                 false);

    RegisterCode("hook_suspension_solve",
                 "48 8B C4 48 89 58 ?? 48 89 70 ?? 48 89 78 ?? 55 48 8D 68",
                 false);

    RegisterCode("hook_tire_force",
                 "40 53 48 83 EC ?? 0F 29 74 24 ?? 48 8B D9 0F 28 ?? ?? ?? ?",
                 false);

    // -------------------------------------------------------------------------
    //  Data: the live values the solver writes back into the car state.
    //  These are rip-relative movss/movsd loads inside the handling routine;
    //  the resolver follows the displacement to the storage the routine reads.
    // -------------------------------------------------------------------------
    RegisterFloat("grip_scale",       "F3 0F 10 0D ?? ?? ?? ?? F3 0F 59 ?? ?? F3 0F 11", 4, 8, true, 0.05f, 8.0f);
    RegisterFloat("downforce_scale",  "F3 0F 10 05 ?? ?? ?? ?? F3 0F 59 87 ?? ?? ?? ?? C3", 4, 8, true, 0.0f, 12.0f);
    RegisterFloat("brake_bias",       "F3 0F 11 0D ?? ?? ?? ?? F3 0F 10 05 ?? ?? ?? ?? F3 0F", 4, 8, true, 0.05f, 0.95f);
    RegisterFloat("steer_angle",      "F3 0F 11 0D ?? ?? ?? ?? 48 8B 5C 24 ?? C3", 4, 8, true, -90.0f, 90.0f);

    RegisterMatrix("friction_matrix", "48 8D 05 ?? ?? ?? ?? 48 89 44 24 ?? 48 8D 15 ?? ?? ?? ??", 3, 7);

    FH4RPO_LOG_INFO("GameBinds: %d binds registered for %s", bindCount_, moduleName_.c_str());
}

void GameBinds::Shutdown() noexcept
{
    if (!initialized_)
        return;

    for (int i = 0; i < bindCount_; ++i)
    {
        binds_[i].stage = BindStage::Unresolved;
        binds_[i].address = 0;
    }
    bindCount_ = 0;
    initialized_ = false;
    FH4RPO_LOG_INFO("GameBinds: released");
}

void GameBinds::RegisterCode(const char* name, const char* signature, bool required) noexcept
{
    if (bindCount_ >= kMaxBinds || name == nullptr)
        return;
    BindEntry& entry = binds_[bindCount_++];
    entry = BindEntry{};
    entry.name = name;
    entry.kind = BindKind::Code;
    entry.signature = signature ? signature : "";
    entry.size = 16;
    entry.required = required;
    entry.allowWrite = false;
    entry.stage = entry.signature.empty() ? BindStage::Unresolved : BindStage::Unresolved;
}

void GameBinds::RegisterFloat(const char* name, const char* signature,
                              std::size_t ripDispOffset, std::size_t ripInstrLength,
                              bool allowWrite, float minValue, float maxValue) noexcept
{
    if (bindCount_ >= kMaxBinds || name == nullptr)
        return;
    BindEntry& entry = binds_[bindCount_++];
    entry = BindEntry{};
    entry.name = name;
    entry.kind = BindKind::Float;
    entry.signature = signature ? signature : "";
    entry.size = sizeof(float);
    entry.ripDispOffset = ripDispOffset;
    entry.ripInstructionLength = ripInstrLength;
    entry.allowWrite = allowWrite;
    entry.minValue = minValue;
    entry.maxValue = maxValue;
}

void GameBinds::RegisterFloatRva(const char* name, std::uintptr_t rva, bool allowWrite,
                                 float minValue, float maxValue) noexcept
{
    if (bindCount_ >= kMaxBinds || name == nullptr)
        return;
    BindEntry& entry = binds_[bindCount_++];
    entry = BindEntry{};
    entry.name = name;
    entry.kind = BindKind::Float;
    entry.moduleOffset = rva;
    entry.size = sizeof(float);
    entry.allowWrite = allowWrite;
    entry.minValue = minValue;
    entry.maxValue = maxValue;
}

void GameBinds::RegisterMatrix(const char* name, const char* signature,
                               std::size_t ripDispOffset, std::size_t ripInstrLength) noexcept
{
    if (bindCount_ >= kMaxBinds || name == nullptr)
        return;
    BindEntry& entry = binds_[bindCount_++];
    entry = BindEntry{};
    entry.name = name;
    entry.kind = BindKind::Matrix4x4;
    entry.signature = signature ? signature : "";
    entry.size = sizeof(float) * 16;
    entry.ripDispOffset = ripDispOffset;
    entry.ripInstructionLength = ripInstrLength;
    entry.allowWrite = true;
    entry.minValue = 0.0f;
    entry.maxValue = 8.0f;
}

void GameBinds::SetOverride(const char* name, std::uintptr_t address) noexcept
{
    BindEntry* entry = Find(name);
    if (entry == nullptr || address == 0)
        return;
    entry->address = address;
    entry->stage = BindStage::Resolved;
    entry->note = "override";
    FH4RPO_LOG_INFO("GameBinds: %s pinned by override to 0x%p", name, reinterpret_cast<void*>(address));
}

BindEntry* GameBinds::Find(const char* name) noexcept
{
    if (name == nullptr)
        return nullptr;
    for (int i = 0; i < bindCount_; ++i)
    {
        if (binds_[i].name == name)
            return &binds_[i];
    }
    return nullptr;
}

const BindEntry* GameBinds::Find(const char* name) const noexcept
{
    return const_cast<GameBinds*>(this)->Find(name);
}

bool GameBinds::ValidateEntry(const BindEntry& entry) const noexcept
{
    if (entry.address == 0)
        return false;

    switch (entry.kind)
    {
    case BindKind::Code:
        return MemoryGuard::IsRangeExecutable(entry.address, 16);

    case BindKind::Float:
    case BindKind::Double:
    case BindKind::Int32:
        if ((entry.address & 0x3u) != 0)
            return false;
        return entry.allowWrite ? MemoryGuard::IsRangeWritable(entry.address, entry.size)
                                : MemoryGuard::IsRangeReadable(entry.address, entry.size);

    case BindKind::Matrix4x4:
        if ((entry.address & 0x3u) != 0)
            return false;
        return MemoryGuard::IsRangeWritable(entry.address, entry.size);

    case BindKind::Chain:
        return MemoryGuard::IsRangeReadable(entry.address, 8);

    default:
        return false;
    }
}

bool GameBinds::ResolveEntry(BindEntry& entry) noexcept
{
    // An override always wins and is never re-scanned.
    if (entry.stage == BindStage::Resolved && entry.note == "override")
        return true;

    std::uintptr_t result = 0;

    if (!entry.signature.empty())
    {
        const std::uintptr_t match = PatternScanner::FindInModule(moduleName_.c_str(),
                                                                  entry.signature.c_str(),
                                                                  entry.name.c_str());
        if (match != 0)
        {
            entry.scannedFrom = match;
            if (entry.kind == BindKind::Code)
                result = match;
            else if (entry.ripInstructionLength > 0)
                result = PatternScanner::ResolveRipRelative(match, entry.ripDispOffset,
                                                            entry.ripInstructionLength);
            else
                result = match;
        }
    }
    else if (entry.moduleOffset != 0)
    {
        const std::uintptr_t base = MemoryGuard::FindModuleBase(moduleName_.c_str());
        if (base != 0)
            result = base + entry.moduleOffset;
    }

    if (result == 0 && !entry.chain.empty())
    {
        const std::uintptr_t base = MemoryGuard::FindModuleBase(moduleName_.c_str());
        if (base != 0)
            result = PatternScanner::ResolvePointerChain(base, entry.chain.data(),
                                                         static_cast<int>(entry.chain.size()));
    }

    if (result == 0)
    {
        entry.stage = BindStage::Failed;
        entry.failures++;
        return false;
    }

    entry.address = result;
    if (!ValidateEntry(entry))
    {
        entry.stage = BindStage::Guarded;
        entry.failures++;
        FH4RPO_LOG_WARN("GameBinds: %s resolved to 0x%p but failed validation",
                        entry.name.c_str(), reinterpret_cast<void*>(result));
        return false;
    }

    entry.stage = BindStage::Resolved;
    return true;
}

void GameBinds::ApplyOverrides() noexcept
{
    // Overrides are applied by name from the config; nothing to iterate here,
    // but kept as an explicit step so the resolution order is documented:
    //   1. overrides  2. signatures  3. RVAs  4. pointer chains
    for (int i = 0; i < bindCount_; ++i)
    {
        if (binds_[i].note == "override" && binds_[i].address != 0)
            binds_[i].stage = BindStage::Resolved;
    }
}

void GameBinds::Tick(double dtSeconds) noexcept
{
    if (!initialized_)
        return;

    clock_ += dtSeconds;
    ApplyOverrides();

    for (int i = 0; i < bindCount_; ++i)
    {
        BindEntry& entry = binds_[i];
        if (entry.stage == BindStage::Disabled)
            continue;

        const bool stale = (clock_ - entry.lastResolveTime) >= resolveInterval_;
        const bool needsWork = (entry.stage != BindStage::Resolved);

        if (!stale && !needsWork)
        {
            // Already resolved: verify it is still valid (the game may have
            // reallocated a heap block the pointer chain used to reach).
            if (!ValidateEntry(entry))
            {
                FH4RPO_LOG_WARN("GameBinds: %s became invalid, scheduling re-resolve",
                                entry.name.c_str());
                entry.stage = BindStage::Guarded;
                entry.address = 0;
            }
            continue;
        }

        if (!stale && entry.stage == BindStage::Resolved)
            continue;

        entry.lastResolveTime = clock_;
        if (!ResolveEntry(entry) && entry.stage == BindStage::Failed && entry.failures > 3)
        {
            // Give up permanently after repeated failures during this session.
            if (entry.required)
                FH4RPO_LOG_ERROR("GameBinds: required bind %s unavailable - feature disabled",
                                 entry.name.c_str());
            entry.stage = BindStage::Disabled;
        }
    }
}

std::uintptr_t GameBinds::Resolve(const char* name) noexcept
{
    BindEntry* entry = Find(name);
    if (entry == nullptr)
        return 0;

    if (entry->stage != BindStage::Resolved || !ValidateEntry(*entry))
    {
        if (!ResolveEntry(*entry))
        {
            entry->lastResolveTime = clock_;
            return 0;
        }
    }
    return entry->address;
}

bool GameBinds::IsUsable(const char* name) const noexcept
{
    const BindEntry* entry = Find(name);
    return entry != nullptr && entry->stage == BindStage::Resolved && entry->address != 0;
}

float GameBinds::ReadFloat(const char* name, float fallback) const noexcept
{
    const BindEntry* entry = Find(name);
    if (entry == nullptr || entry->address == 0)
        return fallback;

    float value = fallback;
    if (MemoryGuard::Read<float>(entry->address, value) != MemStatus::Ok)
        return fallback;
    if (!IsFinite(value) || value < entry->minValue || value > entry->maxValue)
        return fallback;

    const_cast<BindEntry*>(entry)->reads++;
    return value;
}

bool GameBinds::WriteFloat(const char* name, float value) noexcept
{
    BindEntry* entry = Find(name);
    if (entry == nullptr || entry->address == 0 || !entry->allowWrite)
        return false;
    if (!IsFinite(value))
        return false;

    const float clamped = Clamp(value, entry->minValue, entry->maxValue);
    if (MemoryGuard::WriteFloatAtomic(entry->address, clamped) != MemStatus::Ok)
    {
        entry->failures++;
        if (entry->failures > 3)
        {
            entry->stage = BindStage::Guarded;
            entry->address = 0;
        }
        return false;
    }

    entry->writes++;
    return true;
}

bool GameBinds::WriteFloatBlock(const char* name, const float* values, std::size_t count) noexcept
{
    BindEntry* entry = Find(name);
    if (entry == nullptr || entry->address == 0 || values == nullptr || !entry->allowWrite)
        return false;
    if (count > entry->size / sizeof(float))
        return false;

    float sanitised[16] = {};
    for (std::size_t i = 0; i < count; ++i)
        sanitised[i] = ClampFinite(values[i], entry->minValue, entry->maxValue, 1.0f);

    if (MemoryGuard::WriteBlock(entry->address, sanitised, count * sizeof(float)) != MemStatus::Ok)
    {
        entry->failures++;
        if (entry->failures > 3)
        {
            entry->stage = BindStage::Guarded;
            entry->address = 0;
        }
        return false;
    }

    entry->writes++;
    return true;
}

PhysicsEngine::GameTargets GameBinds::BuildEngineTargets() const noexcept
{
    PhysicsEngine::GameTargets targets{};
    auto addressOf = [this](const char* name) -> std::uintptr_t
    {
        const BindEntry* entry = Find(name);
        if (entry == nullptr || entry->stage != BindStage::Resolved)
            return 0;
        return entry->address;
    };

    targets.downforceAddress   = addressOf("downforce_scale");
    targets.gripScaleAddress   = addressOf("grip_scale");
    targets.brakeBiasAddress   = addressOf("brake_bias");
    targets.steerAngleAddress  = addressOf("steer_angle");
    targets.frictionMatrixBase = addressOf("friction_matrix");
    targets.frictionMatrixStride = 16;
    return targets;
}

int GameBinds::ResolvedCount() const noexcept
{
    int count = 0;
    for (int i = 0; i < bindCount_; ++i)
        if (binds_[i].stage == BindStage::Resolved)
            ++count;
    return count;
}

int GameBinds::FailedCount() const noexcept
{
    int count = 0;
    for (int i = 0; i < bindCount_; ++i)
        if (binds_[i].stage == BindStage::Failed || binds_[i].stage == BindStage::Disabled)
            ++count;
    return count;
}

int GameBinds::TotalCount() const noexcept { return bindCount_; }

void GameBinds::DumpReport() const noexcept
{
    static const char* stageNames[] = { "UNRESOLVED", "RESOLVED", "GUARDED", "DISABLED", "FAILED" };
    FH4RPO_LOG_INFO("---- GameBinds: %d/%d resolved ----", ResolvedCount(), bindCount_);
    for (int i = 0; i < bindCount_; ++i)
    {
        const BindEntry& entry = binds_[i];
        const int stageIndex = Clamp(static_cast<int>(entry.stage), 0, 4);
        FH4RPO_LOG_INFO("  %-22s %-10s addr=0x%p reads=%d writes=%d fails=%d %s",
                        entry.name.c_str(), stageNames[stageIndex],
                        reinterpret_cast<void*>(entry.address),
                        entry.reads, entry.writes, entry.failures,
                        entry.note.empty() ? "" : entry.note.c_str());
    }
}

} // namespace fh4rpo
