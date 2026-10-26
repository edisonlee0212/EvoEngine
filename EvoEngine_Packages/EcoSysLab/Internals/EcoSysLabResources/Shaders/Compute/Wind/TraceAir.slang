import EcoSysLab.AirGrid;

struct WindParticle {
  float4 position_age;
  float4 velocity_lifetime;
  float4 collision;
};
struct Parameters {
  uint particle_count;
  uint bucket_count;
  float cell_size;
  float time_step;
  float coupling_rate;
};
[[vk::binding(0, 0)]] RWStructuredBuffer<WindParticle> particles;
[[vk::binding(1, 0)]] StructuredBuffer<uint> air_heads;
[[vk::binding(2, 0)]] StructuredBuffer<AirGridAddresses> air_addresses;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x >= parameters.particle_count) return;
  WindParticle particle = particles[id.x];
  if (particle.velocity_lifetime.w <= particle.position_age.w) return;
  int3 coordinate = int3(floor(particle.position_age.xyz / parameters.cell_size));
  uint index = AirGridFind(air_heads, air_addresses, coordinate, parameters.bucket_count);
  if (index == 0xffffffffu) return;
  AirGridValue air = AirGridReadValue(air_addresses, index);
  if (air.velocity_temperature.w <= 0.0) return;
  float mix = 1.0 - exp(-parameters.coupling_rate * parameters.time_step);
  particle.collision.z = lerp(particle.collision.z, air.humidity_strength_density.x, mix);
  particle.collision.w = lerp(particle.collision.w, air.velocity_temperature.w, mix);
  particles[id.x] = particle;
}
