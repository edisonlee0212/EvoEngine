struct WindParticle {
  float4 position_age;
  float4 velocity_lifetime;
  float4 collision;
};

struct Parameters {
  uint count;
  uint bucket_count;
  float cell_size;
  float padding;
};

[[vk::binding(0, 0)]] StructuredBuffer<WindParticle> particles;
[[vk::binding(1, 0)]] RWStructuredBuffer<uint> heads;
[[vk::binding(2, 0)]] RWStructuredBuffer<uint> next_particles;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

uint HashCell(int3 cell) {
  uint3 value = uint3(cell) * uint3(73856093u, 19349663u, 83492791u);
  return (value.x ^ value.y ^ value.z) % parameters.bucket_count;
}

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  uint index = id.x;
  if (index >= parameters.count) return;
  WindParticle particle = particles[index];
  if (particle.velocity_lifetime.w <= 0.0 || particle.position_age.w >= particle.velocity_lifetime.w) return;
  uint bucket = HashCell(int3(floor(particle.position_age.xyz / parameters.cell_size)));
  uint previous;
  InterlockedExchange(heads[bucket], index, previous);
  next_particles[index] = previous;
}
