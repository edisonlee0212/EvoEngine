#pragma once

// =============================================================================
//  HashRng — stateless GPU-portable RNG used by derive.comp / grow.comp.
//
//  Rationale (Phase 3a):
//    The CPU pipeline already does most of its randomness via
//    HashNodeSeed(node_random, salt) → mt19937 → uniform_real_distribution.
//    HashNodeSeed itself is a pure integer mix and ports trivially to GLSL.
//    The mt19937 + uniform_real path, however, is NOT bit-exact portable to
//    a shader. This file defines the GPU-side replacement and a CPU mirror
//    so the equivalence harness can drive both with identical inputs:
//
//        seed_root = HashNodeSeed(node_random, rule_salt)   // shared
//        u32 word  = PcgHashU32(seed_root, draw_index)      // GPU-portable
//        float u01 = ToUnit01(word)                          // GPU-portable
//
//    The CPU rule set will keep using mt19937 until Phase 3b — at that
//    point CreateTasselTopologyRules() picks up a SampleUnit01Hash()
//    overload (defined here) so the CPU and GPU paths use the same draws.
//    Acceptance gate stays at KS ≤ 0.05 (per docs/gpu_pipeline.md §6).
//
//  Bit-exactness target:
//    PcgHashU32 mirrors the GLSL pcgHashU32() in include/hash_rng.glsl
//    line-for-line. Both clamp inputs identically and produce identical
//    bit patterns on x86_64 + SPIR-V. Verified informally by spot-checking
//    a handful of (seed_root, draw_index) pairs; a full bit-exact unit
//    test is a Phase 3b deliverable.
// =============================================================================

#include <algorithm>
#include <cstdint>

#if defined(LSYSTEM_GPU_PIPELINE)

namespace l_system_plugin::gpu {

// Stateless 32-bit PCG-style mix. Single round; cheap; passes basic chi^2
// for short streams (which is all derive draws ever request).
inline uint32_t PcgHashU32(uint32_t seed, uint32_t key) {
  uint32_t x = seed * 0x747636FBu + key;
  x = ((x >> ((x >> 28) + 4u)) ^ x) * 0x747636FBu;
  return (x >> 22) ^ x;
}

// Mirrors the CPU HashNodeSeed in MaizeTasselRules.hpp lines 38-47 EXACTLY.
// Re-implemented here so this header has no dependency on the rules header.
inline uint32_t HashNodeSeedShared(float node_random, uint32_t salt) {
  const float clamped = std::clamp(node_random, 0.0f, 1.0f);
  uint32_t x = static_cast<uint32_t>(clamped * 4294967295.0f) ^ (salt + 0x9e3779b9u);
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

// 24-bit-precision mantissa packing — matches what GLSL does when you
// `intBitsToFloat`-style normalise a uint into [0, 1). Matches GLSL helper
// `toUnit01()` in hash_rng.glsl.
inline float ToUnit01(uint32_t word) {
  // 24-bit fraction → multiply by 1 / 2^24. Closed-open [0, 1).
  return static_cast<float>(word >> 8) * (1.0f / 16777216.0f);
}

// Convenience: SampleUnit01 keyed on (node_random, rule_salt, draw_index).
// CPU rule code can call this once per draw to match GPU bit-for-bit.
inline float SampleUnit01Hash(float node_random, uint32_t rule_salt, uint32_t draw_index) {
  const uint32_t root = HashNodeSeedShared(node_random, rule_salt);
  return ToUnit01(PcgHashU32(root, draw_index));
}

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
