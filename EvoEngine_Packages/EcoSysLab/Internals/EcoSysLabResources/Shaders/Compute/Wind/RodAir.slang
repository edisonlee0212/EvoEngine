import EcoSysLab.AirGrid;

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
  uint cell_count;
  uint rod_count;
  uint bucket_count;
  float cell_size;
  float time_step;
};
[[vk::binding(0, 0)]] StructuredBuffer<AirGridAddresses> air_addresses;
[[vk::binding(1, 0)]] StructuredBuffer<uint> rod_heads;
[[vk::binding(2, 0)]] StructuredBuffer<uint> rod_next;
[[vk::binding(3, 0)]] StructuredBuffer<uint> max_rod_extent;
[[vk::binding(4, 0)]] StructuredBuffer<TreePhysicsVertex, Std430DataLayout> vertices;
[[vk::binding(5, 0)]] StructuredBuffer<TreePhysicsRod, Std430DataLayout> rods;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x >= parameters.cell_count || parameters.rod_count == 0) return;
  int3 coordinate = AirGridReadCell(air_addresses, id.x).coordinate_next.xyz;
  float3 center = (float3(coordinate) + 0.5) * parameters.cell_size;
  int reach = int(ceil(asfloat(max_rod_extent[0]) / parameters.cell_size)) + 1;
  float3 motion = 0;
  float occupancy = 0;
  for (int z = -reach; z <= reach; ++z)
    for (int y = -reach; y <= reach; ++y)
      for (int x = -reach; x <= reach; ++x) {
        int3 neighbor = coordinate + int3(x, y, z);
        uint rod_index = rod_heads[AirGridHash(neighbor, parameters.bucket_count)];
        while (rod_index != 0xffffffffu) {
          TreePhysicsRod rod = rods[rod_index];
          float3 start = vertices[rod.links.x].position_inverse_mass.xyz;
          float3 end = vertices[rod.links.y].position_inverse_mass.xyz;
          float3 segment = end - start;
          if (all(int3(floor((start + end) * 0.5 / parameters.cell_size)) == neighbor)) {
            float fraction = saturate(dot(center - start, segment) / max(dot(segment, segment), 1e-8));
            float distance = length(center - (start + fraction * segment));
            float radius = rod.length_scale_mass_radius.w;
            float weight = saturate((radius + 0.866 * parameters.cell_size - distance) / parameters.cell_size) *
                           saturate(radius / parameters.cell_size);
            motion += weight * lerp(vertices[rod.links.x].velocity_radius.xyz,
                                    vertices[rod.links.y].velocity_radius.xyz, fraction);
            occupancy += weight;
          }
          rod_index = rod_next[rod_index];
        }
      }
  if (occupancy > 0.0) {
    AirGridValue air = AirGridReadValue(air_addresses, id.x);
    float mix = 1.0 - exp(-4.0 * min(occupancy, 1.0) * parameters.time_step);
    air.velocity_temperature.xyz = lerp(air.velocity_temperature.xyz, motion / occupancy, mix);
    AirGridWriteValue(air_addresses, id.x, air);
  }
}
