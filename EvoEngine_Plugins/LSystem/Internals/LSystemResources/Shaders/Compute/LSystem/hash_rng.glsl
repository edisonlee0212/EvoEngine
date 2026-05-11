// =============================================================================
// hash_rng.glsl — GPU mirror of include/gpu/HashRng.hpp.
//
// Include via:
//   #extension GL_GOOGLE_include_directive : enable
//   #include "hash_rng.glsl"
//
// Bit-for-bit equivalent of the C++ PcgHashU32 / HashNodeSeedShared / ToUnit01
// helpers. Any divergence here breaks the determinism gate (KS ≤ 0.05).
// =============================================================================

#ifndef LSYSTEM_HASH_RNG_GLSL
#define LSYSTEM_HASH_RNG_GLSL

uint pcgHashU32(uint seed, uint key) {
  uint x = seed * 0x747636FBu + key;
  x = ((x >> ((x >> 28) + 4u)) ^ x) * 0x747636FBu;
  return (x >> 22) ^ x;
}

uint hashNodeSeedShared(float node_random, uint salt) {
  float clamped = clamp(node_random, 0.0, 1.0);
  uint  x       = uint(clamped * 4294967295.0) ^ (salt + 0x9e3779b9u);
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

float toUnit01(uint word) {
  // 24-bit fraction. Closed-open [0, 1).
  return float(word >> 8) * (1.0 / 16777216.0);
}

float sampleUnit01Hash(float node_random, uint rule_salt, uint draw_index) {
  uint root = hashNodeSeedShared(node_random, rule_salt);
  return toUnit01(pcgHashU32(root, draw_index));
}

#endif  // LSYSTEM_HASH_RNG_GLSL
