// Godot hddagi_integrate.glsl::MODE_FILTER, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt.
import EvoEngine.HddagiTrace;
import EvoEngine.HddagiVoxel;
import EvoEngine.HddagiProbe;
struct Params {
  int3 probe_size;
  uint cascade;
  int3 region_offset;
  uint enabled;
};
[[vk::push_constant]] ConstantBuffer<Params> params;
[[vk::binding(0, 0)]] Texture2DArray<float3> source;
[[vk::binding(1, 0)]] Texture2DArray<uint> neighbors;
[[vk::binding(2, 0)]] Texture2DArray<float> proximity;
[[vk::binding(3, 0)]][vk::image_format("r32ui")] RWTexture2DArray<uint> destination;
[numthreads(5, 5, 1)] void main(uint3 group : SV_GroupID, uint3 local : SV_GroupThreadID) {
  int3 probe = int3(group.x, group.y % params.probe_size.y, group.y / params.probe_size.y);
  int3 physical = HddagiWrap(probe + params.region_offset, params.probe_size);
  int3 tile = int3(physical.xy + int2(0, physical.z * params.probe_size.y), params.cascade);
  int3 pixel = int3(tile.xy * 7 + int2(local.xy) + 1, tile.z);
  float3 light = source.Load(int4(pixel, 0));
  float total = 1;
  if (params.enabled != 0 && proximity.Load(int4(tile, 0)) > 0.5) {
    uint visible = neighbors.Load(int4(tile, 0));
    for (uint i = 0; i < 6; ++i) {
      float weight = float((visible >> (i * 4)) & 15) / 15 * 0.7;
      if (weight == 0)
        continue;
      int3 next = probe;
      next[i / 2] += (i & 1) != 0 ? 1 : -1;
      if (any(next < 0) || any(next >= params.probe_size))
        continue;
      next = HddagiWrap(next + params.region_offset, params.probe_size);
      int2 next_tile = next.xy + int2(0, next.z * params.probe_size.y);
      light += source.Load(int4(next_tile * 7 + int2(local.xy) + 1, params.cascade, 0)) * weight;
      total += weight;
    }
  }
  uint packed = HddagiEncodeRadiance(light / total);
  for (uint index = 0; index < 49; ++index) {
    uint wrapped = HDDAGI_WRAP_NEIGHBORS[index];
    if ((wrapped & 65535) != local.x || (wrapped >> 16) != local.y)
      continue;
    destination[int3(tile.xy * 7 + int2(index % 7, index / 7), tile.z)] = packed;
  }
}
