// @evoengine-dialect native
// Adapted from https://github.com/Zielon/PBRVulkan and nvpro_core2/nvshaders/random.h.slang.

public uint EE_ROTATE_LEFT(const uint value, const uint count) {
  return (value << count) | (value >> (32u - count));
}

public uint EE_XXHASH32(const uint3 p) {
  const uint4 primes = uint4(2246822519u, 3266489917u, 668265263u, 374761393u);
  uint h32 = p.z + primes.w + p.x * primes.y;
  h32 = primes.z * EE_ROTATE_LEFT(h32, 17u);
  h32 += p.y * primes.y;
  h32 = primes.z * EE_ROTATE_LEFT(h32, 17u);
  h32 = primes.x * (h32 ^ (h32 >> 15u));
  h32 = primes.y * (h32 ^ (h32 >> 13u));
  return h32 ^ (h32 >> 16u);
}

public uint EE_PCG(inout uint state) {
  const uint previous = state * 747796405u + 2891336453u;
  const uint word = ((previous >> ((previous >> 28u) + 4u)) ^ previous) * 277803737u;
  state = previous;
  return (word >> 22u) ^ word;
}

public float EE_PCG_RANDOM(inout uint seed) {
  return float(EE_PCG(seed)) * (1.0f / float(0xffffffffu));
}

public float2 EE_PCG_RANDOM_2(inout uint seed) {
  const float x = EE_PCG_RANDOM(seed);
  const float y = EE_PCG_RANDOM(seed);
  return float2(x, y);
}

public float3 EE_PCG_RANDOM_3(inout uint seed) {
  const float x = EE_PCG_RANDOM(seed);
  const float y = EE_PCG_RANDOM(seed);
  const float z = EE_PCG_RANDOM(seed);
  return float3(x, y, z);
}
