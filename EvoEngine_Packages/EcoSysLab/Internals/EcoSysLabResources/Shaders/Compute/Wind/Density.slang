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

[[vk::binding(0, 0)]] StructuredBuffer<WindParticle> particles;
[[vk::binding(6, 0)]] StructuredBuffer<uint> heads;
[[vk::binding(7, 0)]] StructuredBuffer<uint> next_particles;
[[vk::binding(8, 0)]] RWStructuredBuffer<float> density;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

uint HashCell(int3 cell) {
  uint3 value = uint3(cell) * uint3(73856093u, 19349663u, 83492791u);
  return (value.x ^ value.y ^ value.z) % parameters.bucket_count;
}

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  uint index = id.x;
  if (index >= parameters.particle_count) return;
  WindParticle particle = particles[index];
  if (particle.velocity_lifetime.w <= particle.position_age.w) {
    density[index] = 0;
    return;
  }
  float support = max(particle.collision.x, 0.35);
  float search = max(support, parameters.max_support);
  int3 lower = int3(floor((particle.position_age.xyz - search) / parameters.cell_size));
  int3 upper = int3(floor((particle.position_age.xyz + search) / parameters.cell_size));
  float sum = particle.collision.z * 15.0 / (3.14159265 * support * support * support);
  for (int z = lower.z; z <= upper.z; ++z)
    for (int y = lower.y; y <= upper.y; ++y)
      for (int x = lower.x; x <= upper.x; ++x) {
        int3 cell = int3(x, y, z);
        uint neighbor = heads[HashCell(cell)];
        for (uint visited = 0; neighbor != 0xffffffffu && visited < 96; ++visited) {
          WindParticle other = particles[neighbor];
          if (neighbor != index && other.velocity_lifetime.w > other.position_age.w &&
              all(int3(floor(other.position_age.xyz / parameters.cell_size)) == cell)) {
            float radius = max(support, other.collision.x);
            float weight = saturate(1.0 - length(particle.position_age.xyz - other.position_age.xyz) / radius);
            sum += other.collision.z * 15.0 / (3.14159265 * radius * radius * radius) *
                   weight * weight * weight;
          }
          neighbor = next_particles[neighbor];
        }
      }
  density[index] = sum;
}
