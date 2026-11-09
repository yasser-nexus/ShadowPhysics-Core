// =============================================================================
//  FH4RPO - PatternScanner.cpp
// =============================================================================
#include "fh4rpo/PatternScanner.hpp"
#include "fh4rpo/MemoryGuard.hpp"
#include "fh4rpo/Log.hpp"

#include <windows.h>
#include <cstring>
#include <chrono>
#include <atomic>

namespace fh4rpo {

namespace {

std::atomic<int> g_alignment{ 4 };
std::atomic<int> g_lastDurationMs{ 0 };

int HexValue(char c) noexcept
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

constexpr std::size_t kChunkSize = 256 * 1024;

} // namespace

bool PatternScanner::Parse(const char* signature, std::vector<PatternByte>& out) noexcept
{
    out.clear();
    if (signature == nullptr)
        return false;

    const char* cursor = signature;
    while (*cursor != '\0')
    {
        while (*cursor == ' ' || *cursor == '\t')
            ++cursor;
        if (*cursor == '\0')
            break;

        if (*cursor == '?')
        {
            PatternByte byte{};
            byte.wildcard = true;
            out.push_back(byte);
            ++cursor;
            if (*cursor == '?')
                ++cursor;
            continue;
        }

        const int high = HexValue(cursor[0]);
        const int low  = HexValue(cursor[1]);
        if (high < 0 || low < 0)
        {
            out.clear();
            return false;
        }

        PatternByte byte{};
        byte.value = static_cast<std::uint8_t>((high << 4) | low);
        out.push_back(byte);
        cursor += 2;
    }

    return out.size() >= 4;
}

std::uintptr_t PatternScanner::FindInRange(std::uintptr_t start,
                                           std::size_t size,
                                           const std::vector<PatternByte>& pattern) noexcept
{
    if (start == 0 || size == 0 || pattern.size() < 4 || pattern.size() > size)
        return 0;

    const std::size_t patternSize = pattern.size();
    const std::size_t alignment   = static_cast<std::size_t>(Clamp(g_alignment.load(), 1, 16));

    // Fast path length: bytes up to the first wildcard.
    std::size_t fastPrefix = 0;
    while (fastPrefix < patternSize && !pattern[fastPrefix].wildcard)
        ++fastPrefix;

    std::vector<std::uint8_t> buffer(kChunkSize + patternSize);
    const std::uintptr_t end = start + size;

    std::uintptr_t cursor = start;
    while (cursor + patternSize <= end)
    {
        const std::size_t remaining = static_cast<std::size_t>(end - cursor);
        const std::size_t chunk = (remaining < kChunkSize) ? remaining : kChunkSize;

        if (MemoryGuard::ReadBlock(cursor, buffer.data(), chunk) != MemStatus::Ok)
        {
            // Unreadable page - skip it instead of aborting the whole scan.
            cursor += chunk;
            cursor &= ~(alignment - 1);
            if (chunk == 0)
                break;
            continue;
        }

        const std::size_t limit = chunk;
        for (std::size_t i = 0; i + patternSize <= limit; i += alignment)
        {
            const std::uint8_t* candidate = buffer.data() + i;

            // memchr style acceleration on the leading literal bytes.
            if (fastPrefix > 0 && std::memcmp(candidate, pattern.data(), fastPrefix) != 0)
                continue;

            bool matched = true;
            for (std::size_t j = fastPrefix; j < patternSize; ++j)
            {
                if (!pattern[j].wildcard && candidate[j] != pattern[j].value)
                {
                    matched = false;
                    break;
                }
            }
            if (matched)
                return cursor + i;
        }

        // Overlap by patternSize-1 so matches straddling a chunk boundary work.
        if (chunk <= patternSize)
            break;
        cursor += chunk - (patternSize - 1);
        cursor &= ~(alignment - 1);
    }

    return 0;
}

std::uintptr_t PatternScanner::FindInModule(const char* moduleName,
                                            const char* signature,
                                            const char* labelForLog) noexcept
{
    std::vector<PatternByte> pattern;
    if (!Parse(signature, pattern))
    {
        FH4RPO_LOG_ERROR("Scanner: invalid signature for %s", labelForLog ? labelForLog : "?");
        return 0;
    }
    return FindInModule(moduleName, pattern, labelForLog);
}

std::uintptr_t PatternScanner::FindInModule(const char* moduleName,
                                            const std::vector<PatternByte>& pattern,
                                            const char* labelForLog) noexcept
{
    using clock = std::chrono::steady_clock;
    const auto began = clock::now();

    const std::uintptr_t base = MemoryGuard::FindModuleBase(moduleName);
    const std::size_t     size = MemoryGuard::ModuleSize(moduleName);
    if (base == 0 || size == 0)
    {
        FH4RPO_LOG_ERROR("Scanner: module %s is not loaded", moduleName ? moduleName : "?");
        return 0;
    }

    const std::uintptr_t found = FindInRange(base, size, pattern);

    const auto elapsedMs = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - began).count());
    g_lastDurationMs.store(elapsedMs);

    if (found == 0)
    {
        FH4RPO_LOG_WARN("Scanner: %s not found in %s (%d ms scanned)",
                        labelForLog ? labelForLog : "?", moduleName, elapsedMs);
    }
    else
    {
        FH4RPO_LOG_INFO("Scanner: %s found at 0x%p (+0x%llX) in %d ms",
                        labelForLog ? labelForLog : "?",
                        reinterpret_cast<void*>(found),
                        static_cast<unsigned long long>(found - base), elapsedMs);
    }
    return found;
}

std::uintptr_t PatternScanner::ResolveRipRelative(std::uintptr_t instruction,
                                                  std::size_t dispOffset,
                                                  std::size_t instructionLength) noexcept
{
    if (instruction == 0 || instructionLength == 0 || dispOffset + 4 > instructionLength)
        return 0;

    std::int32_t displacement = 0;
    if (MemoryGuard::ReadBlock(instruction + dispOffset, &displacement, sizeof(displacement)) != MemStatus::Ok)
        return 0;

    const std::int64_t target = static_cast<std::int64_t>(instruction) +
                                static_cast<std::int64_t>(instructionLength) +
                                static_cast<std::int64_t>(displacement);
    if (target <= 0)
        return 0;
    return static_cast<std::uintptr_t>(target);
}

std::uintptr_t PatternScanner::ResolvePointerChain(std::uintptr_t base,
                                                   const std::uint32_t* offsets,
                                                   int depth) noexcept
{
    std::uintptr_t cursor = base;
    for (int i = 0; i < depth; ++i)
    {
        if (cursor == 0)
            return 0;
        const std::uintptr_t slot = cursor + offsets[i];
        std::uintptr_t next = 0;
        if (MemoryGuard::Read<std::uintptr_t>(slot, next) != MemStatus::Ok)
            return 0;
        cursor = next;
    }
    return cursor;
}

void PatternScanner::SetAlignment(int alignment) noexcept
{
    g_alignment.store(Clamp(alignment, 1, 16));
}

int PatternScanner::LastScanDurationMs() noexcept
{
    return g_lastDurationMs.load();
}

} // namespace fh4rpo
