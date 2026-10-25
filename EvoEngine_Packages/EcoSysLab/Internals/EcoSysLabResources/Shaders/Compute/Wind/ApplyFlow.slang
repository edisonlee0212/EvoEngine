struct WindParticle {
  float4 position_age;
  float4 velocity_lifetime;
  float4 collision;
};

struct Parameters {
  uint particle_count;
  uint rod_count;
  uint bucket_count;
  float cell_size;
  float time_step;
  float max_support;
};

[[vk::binding(0, 0)]] RWStructuredBuffer<WindParticle> particles;
[[vk::binding(9, 0)]] StructuredBuffer<float4> next_velocity;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x >= parameters.particle_count) return;
  WindParticle particle = particles[id.x];
  if (particle.velocity_lifetime.w <= particle.position_age.w) return;
  particle.velocity_lifetime.xyz = next_velocity[id.x].xyz;
  particles[id.x] = particle;
}
