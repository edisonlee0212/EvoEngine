struct WindParticle {
  float4 position_age;
  float4 velocity_lifetime;
  float4 collision;
};

struct Parameters {
  float time_step;
  uint count;
  float2 padding;
};

[[vk::binding(0, 0)]] RWStructuredBuffer<WindParticle> particles;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x >= parameters.count) return;
  WindParticle particle = particles[id.x];
  if (particle.velocity_lifetime.w <= 0.0 || particle.position_age.w >= particle.velocity_lifetime.w) return;
  particle.position_age.xyz += particle.velocity_lifetime.xyz * parameters.time_step;
  particle.position_age.w += parameters.time_step;
  particles[id.x] = particle;
}
