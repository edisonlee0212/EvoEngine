struct WindParticle {
  float4 position_age;
  float4 velocity_lifetime;
};

struct Parameters {
  float4x4 model;
  float4 speed_spread_lifetime;
  uint4 base_count_seed_capacity;
};

[[vk::binding(0, 0)]] RWStructuredBuffer<WindParticle> particles;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

uint Hash(uint value) {
  value ^= value >> 16u;
  value *= 0x7feb352du;
  value ^= value >> 15u;
  value *= 0x846ca68bu;
  return value ^ (value >> 16u);
}

float Random(uint value) { return float(Hash(value)) * (1.0 / 4294967296.0); }

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x >= parameters.base_count_seed_capacity.y) return;
  uint sequence = parameters.base_count_seed_capacity.x + id.x;
  uint seed = parameters.base_count_seed_capacity.z ^ sequence;
  float2 surface = float2(Random(seed), Random(seed + 1u)) - 0.5;
  float cosine = lerp(cos(parameters.speed_spread_lifetime.y), 1.0, Random(seed + 2u));
  float angle = 6.28318530718 * Random(seed + 3u);
  float sine = sqrt(max(0.0, 1.0 - cosine * cosine));
  float3 direction = float3(sine * cos(angle), sine * sin(angle), cosine);
  float3 world_direction = mul(float4(direction, 0.0), parameters.model).xyz;
  world_direction = dot(world_direction, world_direction) > 1e-8 ? normalize(world_direction) : float3(0.0, 0.0, 1.0);
  WindParticle particle;
  particle.position_age = float4(mul(float4(surface, 0.0, 1.0), parameters.model).xyz, 0.0);
  particle.velocity_lifetime = float4(world_direction * parameters.speed_spread_lifetime.x,
                                      parameters.speed_spread_lifetime.z);
  particles[sequence % parameters.base_count_seed_capacity.w] = particle;
}
