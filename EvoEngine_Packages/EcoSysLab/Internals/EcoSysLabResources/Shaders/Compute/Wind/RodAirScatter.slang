import EcoSysLab.TreePhysicsData;
import EcoSysLab.AirGrid;

#if defined(EE_FOREST_PHYSICS)
import EcoSysLab.ForestWindRods;
#else
import EcoSysLab.WindRodAccess;
#endif

struct Parameters {
  uint cell_count;
  uint rod_count;
  uint bucket_count;
  uint phase; // clear, scatter, resolve
  float air_cell_size;
  float time_step;
};
[[vk::binding(0, 0)]] StructuredBuffer<AirGridAddresses> air_addresses;
[[vk::binding(1, 0)]] StructuredBuffer<uint> air_heads;
[[vk::binding(2, 0)]] RWStructuredBuffer<float> sums;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  uint index = id.x;
  if (parameters.phase != 1) {
    if (index >= parameters.cell_count) return;
    uint base = 4 * index;
    if (parameters.phase == 0) {
      for (uint i = 0; i < 4; ++i) sums[base + i] = 0;
    } else {
      float weight = sums[base + 3];
      if (weight > 0) {
        float3 motion = float3(sums[base], sums[base + 1], sums[base + 2]) / weight;
        AirGridValue air = AirGridReadValue(air_addresses, index);
        float mix = 1.0 - exp(-4.0 * min(weight, 1.0) * parameters.time_step);
        air.velocity_temperature.xyz = lerp(air.velocity_temperature.xyz, motion, mix);
        AirGridWriteValue(air_addresses, index, air);
      }
    }
    return;
  }
  if (index >= parameters.rod_count) return;
  TreePhysicsRod rod;TreePhysicsVertex a,b;
  if(!LoadWindRod(index,rod,a,b)) return;
  float radius = rod.length_scale_mass_radius.w;
  if (radius <= 0) return;
  float3 start = a.position_inverse_mass.xyz, end = b.position_inverse_mass.xyz;
  float3 segment = end - start;
  float support = radius + 0.866 * parameters.air_cell_size;
  int3 first = int3(floor((min(start, end) - support) / parameters.air_cell_size));
  int3 last = int3(floor((max(start, end) + support) / parameters.air_cell_size));
  for (int z = first.z; z <= last.z; ++z)
    for (int y = first.y; y <= last.y; ++y)
      for (int x = first.x; x <= last.x; ++x) {
        int3 coordinate = int3(x, y, z);
        float3 center = (float3(coordinate) + 0.5) * parameters.air_cell_size;
        float fraction = saturate(dot(center - start, segment) / max(dot(segment, segment), 1e-8));
        float distance = length(center - (start + fraction * segment));
        float weight = saturate((support - distance) / parameters.air_cell_size) *
                       saturate(radius / parameters.air_cell_size);
        if (weight <= 0) continue;
        uint cell = AirGridFind(air_heads, air_addresses, coordinate, parameters.bucket_count);
        if (cell == 0xffffffffu) continue;
        float3 motion = weight * lerp(a.velocity_radius.xyz, b.velocity_radius.xyz, fraction);
        InterlockedAdd(sums[4 * cell], motion.x);
        InterlockedAdd(sums[4 * cell + 1], motion.y);
        InterlockedAdd(sums[4 * cell + 2], motion.z);
        InterlockedAdd(sums[4 * cell + 3], weight);
      }
}
