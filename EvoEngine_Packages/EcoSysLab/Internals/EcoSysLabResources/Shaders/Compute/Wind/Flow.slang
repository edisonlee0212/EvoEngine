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
[[vk::binding(8, 0)]] StructuredBuffer<float> density;
[[vk::binding(9, 0)]] RWStructuredBuffer<float4> next_velocity;
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
  float3 velocity = particle.velocity_lifetime.xyz;
  if (particle.velocity_lifetime.w <= particle.position_age.w) {
    next_velocity[index] = float4(velocity, 0);
    return;
  }
  float support = max(particle.collision.x, 0.35);
  float search = max(support, parameters.max_support);
  int3 lower = int3(floor((particle.position_age.xyz - search) / parameters.cell_size));
  int3 upper = int3(floor((particle.position_age.xyz + search) / parameters.cell_size));
  float own_density = max(density[index], 0.01);
  float pressure = 100.0 * max(own_density - particle.collision.y, 0.0);
  float3 pressure_force = 0;
  float3 viscosity = 0;
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
            float3 offset = particle.position_age.xyz - other.position_age.xyz;
            float distance = length(offset);
            float weight = saturate(1.0 - distance / radius);
            if (weight > 0) {
              float other_density = max(density[neighbor], 0.01);
              float other_pressure = 100.0 * max(other_density - other.collision.y, 0.0);
              if (distance > 1e-5)
                pressure_force += (offset / distance) * other.collision.z *
                    (pressure / (own_density * own_density) + other_pressure / (other_density * other_density)) *
                    45.0 * weight * weight / (3.14159265 * radius * radius * radius * radius);
              viscosity += other.collision.z / other_density * (other.velocity_lifetime.xyz - velocity) *
                           45.0 * weight / (3.14159265 * radius * radius * radius * radius * radius);
            }
          }
          neighbor = next_particles[neighbor];
        }
      }
  float3 acceleration = pressure_force + 0.1 * viscosity;
  float3 change = parameters.time_step * acceleration;
  float change_length = length(change);
  next_velocity[index] = float4(velocity + (change_length > 1.0 ? change / change_length : change), 0);
}
