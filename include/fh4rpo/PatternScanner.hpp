// =============================================================================
//  FH4RPO - PatternScanner.hpp
//
//  Signature ("AOB") scanning with SEH protected block reads.
//
//  Used for one purpose only: to find the *entry points* the module hooks.
//  All data the solver touches afterwards is reached through those entry
//  points (this pointer + documented offsets), never through a hard coded
//  absolute address - which is what makes the module survive a title update.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>

namespace fh4rpo {

class PatternScanner
{
public:
    struct PatternByte
    {
        std::uint8_t value   = 0;
        bool         wildcard = false;
    };

    // "48 8B 05 ?? ?? ?? ?? F3 0F 11"  (?? and ? are wildcards)
    static bool Parse(const char* signature, std::vector<PatternByte>& out) noexcept;

    // Scans the whole image of `moduleName`.  Returns 0 when not found.
    static std::uintptr_t FindInModule(const char* moduleName,
                                       const char* signature,
                                       const char* labelForLog) noexcept;

    static std::uintptr_t FindInModule(const char* moduleName,
                                       const std::vector<PatternByte>& pattern,
                                       const char* labelForLog) noexcept;

    // Scans an arbitrary, already validated range.
    static std::uintptr_t FindInRange(std::uintptr_t start,
                                      std::size_t size,
                                      const std::vector<PatternByte>& pattern) noexcept;

    // Resolves an instruction of the form  E8/E9/48 8B 0D  xx xx xx xx
    // `instruction` points at the first byte; `dispOffset` is the offset of the
    // 4 byte displacement, `instructionLength` the total length.
    static std::uintptr_t ResolveRipRelative(std::uintptr_t instruction,
                                             std::size_t dispOffset,
                                             std::size_t instructionLength) noexcept;

    // Follows up to `depth` pointer hops: base -> [base+offset0] -> [..+offset1]
    static std::uintptr_t ResolvePointerChain(std::uintptr_t base,
                                              const std::uint32_t* offsets,
                                              int depth) noexcept;

    static void SetAlignment(int alignment) noexcept;
    static int  LastScanDurationMs() noexcept;
};

} // namespace fh4rpo
