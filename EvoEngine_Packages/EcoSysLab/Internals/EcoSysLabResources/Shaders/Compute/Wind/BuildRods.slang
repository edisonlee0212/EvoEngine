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

[[vk::binding(1, 0)]] RWStructuredBuffer<uint> rod_heads;
[[vk::binding(2, 0)]] RWStructuredBuffer<uint> rod_next;
[[vk::binding(3, 0)]] RWStructuredBuffer<uint> max_rod_extent;
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
  if (index >= parameters.rod_count) return;
  TreePhysicsRod rod = rods[index];
  float3 start = vertices[rod.links.x].position_inverse_mass.xyz;
  float3 end = vertices[rod.links.y].position_inverse_mass.xyz;
  float3 midpoint = 0.5 * (start + end);
  float extent = 0.5 * length(end - start) + rod.length_scale_mass_radius.w;
  uint ignored;
  InterlockedMax(max_rod_extent[0], asuint(extent), ignored);
  uint previous;
  InterlockedExchange(rod_heads[HashCell(int3(floor(midpoint / parameters.cell_size)))], index, previous);
  rod_next[index] = previous;
}
