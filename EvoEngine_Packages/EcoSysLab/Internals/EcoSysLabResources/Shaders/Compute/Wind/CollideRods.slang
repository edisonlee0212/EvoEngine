struct WindParticle {
  float4 position_age;
  float4 velocity_lifetime;
  float4 collision;
};

struct TreePhysicsVertex {
  float4 position_inverse_mass;
  float4 velocity_radius;
  float4 rest_local;
  uint4 incident_rods;
};

struct TreePhysicsRod {
  uint4 links;
  float4 length_scale_mass_radius;
  float4 rotation;
  float4 rest_rotation;
  uint4 incident_angles;
};

struct Parameters {
  uint particle_count;
  uint rod_count;
  uint bucket_count;
  float cell_size;
};

[[vk::binding(0, 0)]] RWStructuredBuffer<WindParticle> particles;
[[vk::binding(1, 0)]] StructuredBuffer<uint> rod_heads;
[[vk::binding(2, 0)]] StructuredBuffer<uint> rod_next;
[[vk::binding(3, 0)]] StructuredBuffer<uint> max_rod_extent;
[[vk::binding(4, 0)]] StructuredBuffer<TreePhysicsVertex, Std430DataLayout> vertices;
[[vk::binding(5, 0)]] StructuredBuffer<TreePhysicsRod, Std430DataLayout> rods;
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
  if (particle.velocity_lifetime.w <= particle.position_age.w) return;
  float support = max(particle.collision.x, 0.05);
  float radius = asfloat(max_rod_extent[0]) + support;
  int3 lower = int3(floor((particle.position_age.xyz - radius) / parameters.cell_size));
  int3 upper = int3(floor((particle.position_age.xyz + radius) / parameters.cell_size));
  float best_gap = support;
  float3 best_normal = 0;
  float3 best_velocity = 0;
  for (int z = lower.z; z <= upper.z; ++z)
    for (int y = lower.y; y <= upper.y; ++y)
      for (int x = lower.x; x <= upper.x; ++x) {
        int3 cell = int3(x, y, z);
        for (uint rod_index = rod_heads[HashCell(cell)]; rod_index != 0xffffffffu;
             rod_index = rod_next[rod_index]) {
          TreePhysicsRod rod = rods[rod_index];
          TreePhysicsVertex first = vertices[rod.links.x];
          TreePhysicsVertex second = vertices[rod.links.y];
          float3 start = first.position_inverse_mass.xyz;
          float3 end = second.position_inverse_mass.xyz;
          float3 segment = end - start;
          float3 midpoint = 0.5 * (start + end);
          if (any(int3(floor(midpoint / parameters.cell_size)) != cell)) continue;
          float segment_length_squared = dot(segment, segment);
          float t = segment_length_squared > 1e-12 ?
              saturate(dot(particle.position_age.xyz - start, segment) / segment_length_squared) : 0.0;
          float3 offset = particle.position_age.xyz - (start + t * segment);
          float distance = length(offset);
          float gap = distance - rod.length_scale_mass_radius.w;
          if (gap >= best_gap) continue;
          float3 surface_velocity = lerp(first.velocity_radius.xyz, second.velocity_radius.xyz, t);
          float3 normal;
          if (distance > 1e-6) {
            normal = offset / distance;
          } else {
            float3 axis = segment_length_squared > 1e-12 ? normalize(segment) : float3(0, 1, 0);
            float3 side = particle.velocity_lifetime.xyz - surface_velocity;
            side -= dot(side, axis) * axis;
            normal = dot(side, side) > 1e-12 ? -normalize(side) :
                normalize(cross(axis, abs(axis.y) < 0.9 ? float3(0, 1, 0) : float3(1, 0, 0)));
          }
          best_gap = gap;
          best_normal = normal;
          best_velocity = surface_velocity;
        }
      }
  if (best_gap >= support) return;
  if (best_gap < 0)
    particle.position_age.xyz += best_normal * (-best_gap + 1e-4);
  float influence = 1.0 - smoothstep(0.0, support, max(0.0, best_gap));
  float incoming = dot(particle.velocity_lifetime.xyz - best_velocity, best_normal);
  if (incoming < 0) particle.velocity_lifetime.xyz -= influence * incoming * best_normal;
  particles[index] = particle;
}
